"""
BLE ADC->BLE Pipeline Logger — hello_world (nRF5340)   [v2]
==========================================================
Connects to the device, receives NUS TX notifications, parses binary
packets, and saves every sample to a CSV file. Parses `ble_sender.c`'s
`ble_sm_packet` (tagged `0xA5`) — the only BLE send path in this project.

Two separate seq fields, two different jobs:
  - business_seq: window identity (sm_core.c's window_seq). Monotonic but
    has design-intentional gaps whenever a window is dropped or preempted
    before it reaches the BLE TX queue. A gap here is normal — logged in
    the CSV, never a WARN.
  - tx_seq: transport-level counter, assigned once per actual BLE
    transmit attempt — one per FRAGMENT. Strictly sequential; a gap here
    means a packet that really went out over BLE never arrived, which is
    real link loss. uint16_t on the wire, wraps at 65536 (handled).

Three timebases, don't confuse them:
  - timestamp_ms : DEVICE clock (k_uptime_get_32()), when the window's
                   LAST sample completed. Tells you when data was
                   CAPTURED. Gaps here mean the producer (sm_core) itself
                   fell behind.
  - t_pc         : PC clock (perf_counter), when this script's callback
                   ran. Tells you when data ARRIVED. Gaps here mean the
                   transport fell behind.
  - t_pc - timestamp_ms drift: end-to-end latency. If it grows
                   monotonically and then plateaus, a queue saturated.

CHANGES IN v2 (all fixes are about not corrupting the above measurement):
  * Ctrl+C now actually works. The old `except KeyboardInterrupt` around
    `await asyncio.sleep()` was dead code — the exception is raised in the
    event loop, not the coroutine, so stop_notify(), the reassembly
    finalize, and the end-of-session summary never ran.
  * Console printing throttled to 10 Hz and CSV writes batched. Both used
    to run on EVERY notification, inside the event loop thread; on Windows
    a conhost write can cost milliseconds, which pushed the *next*
    packet's t_pc later. t_pc was measuring "when Python got around to
    it", which is exactly the thing being investigated.
  * Disconnect detection. A mid-session drop used to leave the script
    spinning forever on a dead link.
  * MTU is printed at connect. If it's < 223 your 220-byte fragments are
    being split at the link layer and the whole throughput question is
    already answered.
  * Optional Windows connection-parameter monitoring / boost (--boost).
  * Empty-notification guard (data[0] used to IndexError on a 0-byte
    packet, silently swallowed by bleak).
  * business_seq gap accounting now counts DROPPED WINDOWS, not gap
    events, and survives losing a window's frag 0.
  * End-of-session inter-arrival statistics, to nail down the per-fragment
    service interval directly.

Requirements:
    pip install bleak

Usage:
    python ble_logger_2.py                   # auto-named CSV under captures/
    python ble_logger_2.py -o my_data.csv    # bare filename -> captures/
    python ble_logger_2.py -o C:/logs/x.csv  # explicit dir, used as-is
    python ble_logger_2.py --name MyDevice
    python ble_logger_2.py --boost           # Win11: ask for ThroughputOptimized
"""

import argparse
import asyncio
import csv
import datetime
import signal
import statistics
import struct
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

# ── Device / BLE constants ─────────────────────────────────────────────────────

DEVICE_NAME = "Device_ADC"         # CONFIG_BT_DEVICE_NAME in prj.conf

# NUS TX characteristic (device → PC, notify)
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# ── Packet layout (must match ble_sender.c struct ble_sm_packet) ──────────────
#
#  Offset  Size  Field
#  ------  ----  -----
#    0      1    tag           uint8_t   0xA5, always — sanity check
#    1      2    tx_seq        uint16_t  transport-level, one per FRAGMENT
#    3      4    business_seq  uint32_t  window_seq from sm_core.c
#    7      4    timestamp_ms  uint32_t  device uptime at last sample of window
#   11      1    window_type   uint8_t   0=HR, 1=BP, 2=ACTIVITY
#   12      4    sample_count  uint32_t  WHOLE window's total sample count
#   16      1    complete      uint8_t   0/1
#   17      1    frag_idx      uint8_t   0-based index of this fragment
#   18      1    frag_count    uint8_t   total fragments in this window
#   19      1    raw_count     uint8_t   samples in THIS fragment, <= 50
#   20    var    raw[][2]      int16_t   raw_count*2 values, row-major
#   ──────────────────
#   Header 20 B. Total 20 + raw_count*2*2 (up to 220 for a 50-sample frag).
#
#   The ADC itself samples SM_ADC_NUM_CHANNELS=4 (AIN0-AIN3, see
#   sampling_source.h — a temporary debug widening), but ble_sender.c only
#   forwards the first BLE_SM_TX_NUM_CHANNELS=2 of each sample over BLE;
#   rtt_sender.c (see rtt_logger.py) carries all 4.

BLE_SM_PACKET_TAG      = 0xA5
BLE_SM_SAMPLES_PER_PKT = 50   # ble_sender.c's per-fragment cap
BLE_SM_NUM_CHANNELS    = 2    # ble_sender.c's BLE_SM_TX_NUM_CHANNELS
TX_SEQ_WRAP            = 1 << 16   # tx_seq is a uint16_t on the wire

HEADER_FMT  = "<BHIIBIBBBB"
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 20

WINDOW_TYPE_NAMES = {0: "HR", 1: "BP", 2: "ACTIVITY"}

# A fragment needs this much ATT payload to travel as one PDU. ATT MTU must
# be >= this + 3 (ATT notification header) for the link layer not to split it.
MAX_FRAG_BYTES = HEADER_SIZE + BLE_SM_SAMPLES_PER_PKT * BLE_SM_NUM_CHANNELS * 2  # 220
MIN_USEFUL_MTU = MAX_FRAG_BYTES + 3                                              # 223

CSV_FIELDS = [
    "t_pc", "tx_seq", "business_seq", "timestamp_ms", "window_type", "window_type_name",
    "total_sample_count", "complete", "frag_idx", "frag_count", "raw_count",
    "sample_idx", "ch0_raw", "ch1_raw",
]

PRINT_INTERVAL_S = 0.1      # console refresh cap — keep the event loop free
ROW_BATCH        = 1000     # rows buffered before hitting the file


# ── Packet parser ──────────────────────────────────────────────────────────────

def parse_packet(data: bytes) -> list[dict] | None:
    if len(data) < HEADER_SIZE:
        return None

    fields = struct.unpack_from(HEADER_FMT, data)
    (tag, tx_seq, business_seq, timestamp_ms, window_type,
     sample_count, complete, frag_idx, frag_count, raw_count) = fields

    if tag != BLE_SM_PACKET_TAG:
        return None

    raw_bytes_expected = raw_count * BLE_SM_NUM_CHANNELS * 2
    if len(data) != HEADER_SIZE + raw_bytes_expected:
        return None

    common = {
        "tx_seq":              tx_seq,
        "business_seq":        business_seq,
        "timestamp_ms":        timestamp_ms,
        "window_type":         window_type,
        "window_type_name":    WINDOW_TYPE_NAMES.get(window_type, f"UNKNOWN({window_type})"),
        "total_sample_count":  sample_count,
        "complete":            complete,
        "frag_idx":            frag_idx,
        "frag_count":          frag_count,
        "raw_count":           raw_count,
    }

    if raw_count == 0:
        # Still log the fragment itself (for seq/gap tracking).
        return [{**common, "sample_idx": "", "ch0_raw": "", "ch1_raw": ""}]

    raw_flat = struct.unpack_from(f"<{raw_count * BLE_SM_NUM_CHANNELS}h", data, HEADER_SIZE)

    rows = []
    for i in range(raw_count):
        base = i * BLE_SM_NUM_CHANNELS
        rows.append({
            **common,
            # Window-relative index, so rows from every fragment of a window
            # sort correctly without the reader knowing fragment boundaries.
            "sample_idx": frag_idx * BLE_SM_SAMPLES_PER_PKT + i,
            "ch0_raw":    raw_flat[base + 0],
            "ch1_raw":    raw_flat[base + 1],
        })
    return rows


# ── Window reassembly tracking ─────────────────────────────────────────────────

class WindowReassemblyTracker:
    """Tracks which frag_idx values arrived for the window currently being
    assembled. Fragments of one window are expected to arrive together and in
    order (single ordered queue in ble_sender.c); this does not handle
    reordering, only "did every fragment show up before the next window did".
    """

    def __init__(self):
        self._business_seq = None
        self._frag_count = None
        self._seen = set()
        self.incomplete_windows = 0
        self.warnings: list[str] = []

    def _check_and_reset(self, new_business_seq, new_frag_count):
        if self._business_seq is not None and self._frag_count is not None:
            missing = set(range(self._frag_count)) - self._seen
            if missing:
                self.incomplete_windows += 1
                self.warnings.append(
                    f"window business_seq={self._business_seq} incomplete: "
                    f"got {len(self._seen)}/{self._frag_count} fragments, "
                    f"missing frag_idx={sorted(missing)}"
                )
        self._business_seq = new_business_seq
        self._frag_count = new_frag_count
        self._seen = set()

    def observe(self, business_seq: int, frag_idx: int, frag_count: int) -> None:
        if business_seq != self._business_seq:
            self._check_and_reset(business_seq, frag_count)
        self._seen.add(frag_idx)

    def finalize(self) -> None:
        self._check_and_reset(None, None)


# ── Windows-only: connection parameter visibility / boost ─────────────────────

def setup_windows_conn_params(client: BleakClient, boost: bool):
    """Log the real connection interval from the OS side, and optionally ask
    Windows for ThroughputOptimized parameters.

    Returns an object that MUST be kept alive for the boost to stay in
    effect (disposing it reverts to the system default), or None.
    Entirely best-effort: any failure is non-fatal and just means less
    visibility, never a broken capture.
    """
    if sys.platform != "win32":
        return None

    Pref = None
    for mod in ("winrt.windows.devices.bluetooth", "bleak_winrt.windows.devices.bluetooth"):
        try:
            Pref = __import__(mod, fromlist=["BluetoothLEPreferredConnectionParameters"])
            Pref = Pref.BluetoothLEPreferredConnectionParameters
            break
        except Exception:
            continue
    if Pref is None:
        print("[INFO] WinRT bindings unavailable — no connection-parameter visibility.")
        return None

    dev = getattr(getattr(client, "_backend", None), "_requester", None)
    if dev is None:
        print("[INFO] Could not reach the underlying BluetoothLEDevice (bleak version).")
        return None

    # The connection-parameter API is not present in every pywinrt
    # projection / Windows SDK version, so probe for it rather than assume
    # a name. If it is missing, say what IS available so the right member
    # can be found instead of guessing again.
    def _fmt(p):
        return (f"interval={p.connection_interval * 1.25:.2f} ms  "
                f"latency={p.connection_latency}  "
                f"timeout={p.link_timeout * 10} ms")

    has_params = hasattr(dev, "connection_parameters")
    if has_params:
        try:
            print(f"[PARAMS] initial {_fmt(dev.connection_parameters)}")
        except Exception as e:
            print(f"[INFO] connection_parameters present but unreadable: {e}")
            has_params = False
    else:
        members = sorted(a for a in dir(dev)
                         if not a.startswith("_")
                         and ("conn" in a.lower() or "param" in a.lower()))
        print("[INFO] This pywinrt build has no BluetoothLEDevice.connection_parameters.")
        print(f"       Related members present: {members or '(none)'}")
        print("       Connection interval will not be visible from the PC side; use an "
              "nRF Sniffer instead.")

    if has_params:
        try:
            def _on_changed(sender, _args):
                try:
                    print(f"\n[PARAMS] {_fmt(sender.connection_parameters)}")
                except Exception:
                    pass
            dev.add_connection_parameters_changed(_on_changed)
        except Exception as e:
            print(f"[INFO] Could not subscribe to parameter changes: {e}")

    if not boost:
        return None

    try:
        tput = getattr(Pref, "throughput_optimized", None)
        if tput is None:
            tput = Pref.get_throughput_optimized()
        elif callable(tput):
            tput = tput()
        req = dev.request_preferred_connection_parameters(tput)
        print(f"[BOOST] RequestPreferredConnectionParameters -> status={req.status} (1 = Success)")
        return req
    except Exception as e:
        print(f"[INFO] ThroughputOptimized request failed ({e}); "
              f"needs Windows 11 (10.0.22000+).")
        return None


# ── Windows-only: start_notify() retry ─────────────────────────────────────────

async def start_notify_with_retry(client: BleakClient, uuid: str, callback, attempts: int = 4) -> None:
    """client.start_notify() intermittently fails on Windows with
    OSError -2147023673 ("operation canceled") writing the CCCD. Two
    known causes, same symptom: (1) a WinRT/bleak timing race where the
    write lands before Windows' post-connect state has settled, and (2)
    a previous session on this device being killed hard enough (window
    closed, process force-killed) to skip its own client.disconnect() —
    Windows can then hold a stale GATT session reference for this device
    for a beat. Neither is something this script can prevent outright
    (see the module docstring / project history for the Ctrl+C-handling
    side of (2)), so absorb the symptom here instead: a short settle
    delay before every attempt (including the first), then retry on
    failure rather than crashing the whole capture. Re-raises after the
    last attempt so a genuine, persistent failure still surfaces.
    """
    delay = 0.5
    for attempt in range(1, attempts + 1):
        await asyncio.sleep(delay)
        try:
            await client.start_notify(uuid, callback)
            return
        except OSError as e:
            if attempt == attempts:
                raise
            print(f"[WARN] start_notify attempt {attempt}/{attempts} failed ({e}); retrying...")
            delay = min(delay * 2, 3.0)


# ── Main ───────────────────────────────────────────────────────────────────────

async def run(device_name: str, csv_path: Path, boost: bool) -> int:
    print(f"Scanning for '{device_name}' …")
    device = await BleakScanner.find_device_by_name(device_name, timeout=15.0)
    if device is None:
        print(f"ERROR: device '{device_name}' not found within 15 s.")
        return 1

    print(f"Found: {device.address}")

    loop = asyncio.get_running_loop()
    stop_evt = asyncio.Event()
    disconnected = False

    def _request_stop(*_):
        loop.call_soon_threadsafe(stop_evt.set)

    def _on_disconnect(_client):
        nonlocal disconnected
        disconnected = True
        loop.call_soon_threadsafe(stop_evt.set)

    prev_sigint = None
    try:
        prev_sigint = signal.signal(signal.SIGINT, _request_stop)
    except Exception:
        pass   # not the main thread, or platform quirk — falls back to except below

    # ── session counters ──
    total_fragments  = 0
    total_samples    = 0
    link_losses      = 0    # real losses, from tx_seq gaps only
    dup_packets      = 0
    bad_packets      = 0
    windows_seen     = 0
    windows_dropped  = 0    # business_seq gaps: intentional, NOT a loss
    gap_events       = 0
    last_tx_seq: int | None = None
    last_business_seq: int | None = None
    max_payload     = 0    # largest notification seen — the real MTU evidence
    arrival_ts: list[float] = []       # t_pc of every fragment
    lat_samples: list[float] = []      # (t_pc - device timestamp) drift, seconds
    lat_ref: float | None = None
    reassembly = WindowReassemblyTracker()

    pending_rows: list[dict] = []
    last_print = 0.0
    status_line = ""

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        def flush_rows():
            if pending_rows:
                writer.writerows(pending_rows)
                pending_rows.clear()
                f.flush()

        def on_notify(_sender, data: bytearray) -> None:
            nonlocal total_fragments, total_samples, link_losses, dup_packets
            nonlocal bad_packets, windows_seen, windows_dropped, gap_events
            nonlocal last_tx_seq, last_business_seq, last_print, status_line, lat_ref
            nonlocal max_payload

            # Captured first, before any parsing/IO, so it reflects arrival
            # rather than when this callback finished its work.
            t_pc = time.perf_counter()
            if len(data) > max_payload:
                max_payload = len(data)

            rows = parse_packet(bytes(data))
            if rows is None:
                bad_packets += 1
                first = f"0x{data[0]:02x}" if len(data) else "(empty)"
                print(f"\n[WARN] unrecognized packet: {len(data)} bytes, first byte {first} "
                      f"(expected tag 0x{BLE_SM_PACKET_TAG:02x})")
                return

            head         = rows[0]
            tx_seq       = head["tx_seq"]
            business_seq = head["business_seq"]
            frag_idx     = head["frag_idx"]
            frag_count   = head["frag_count"]

            arrival_ts.append(t_pc)

            # End-to-end drift: device capture clock vs PC arrival clock.
            # Absolute offset is meaningless; the first sample anchors it and
            # everything after is relative growth (queue build-up).
            dev_s = head["timestamp_ms"] / 1000.0
            if lat_ref is None:
                lat_ref = t_pc - dev_s
            lat_samples.append((t_pc - dev_s) - lat_ref)

            # Real link-loss detection: tx_seq only. Modular arithmetic
            # handles the uint16_t wraparound (65535 -> 0 is a diff of 1).
            if last_tx_seq is not None:
                diff = (tx_seq - last_tx_seq) % TX_SEQ_WRAP
                if diff > 1:
                    lost = diff - 1
                    link_losses += lost
                    print(f"\n[WARN] LINK LOSS: tx_seq {last_tx_seq} → {tx_seq} "
                          f"({lost} packet(s) never arrived)")
                elif diff == 0:
                    dup_packets += 1
                    print(f"\n[WARN] duplicate tx_seq={tx_seq} received")
            last_tx_seq = tx_seq

            # business_seq gaps are DESIGN-INTENTIONAL (sm_core dropped or
            # preempted a window before it reached the BLE TX queue).
            # Counted as DROPPED WINDOWS, not gap events, and keyed off a
            # business_seq change rather than frag_idx == 0 so that losing a
            # window's first fragment doesn't skew the baseline.
            if business_seq != last_business_seq:
                windows_seen += 1
                if last_business_seq is not None and business_seq > last_business_seq:
                    d = business_seq - last_business_seq
                    if d > 1:
                        windows_dropped += d - 1
                        gap_events += 1
                last_business_seq = business_seq

            reassembly.observe(business_seq, frag_idx, frag_count)

            for r in rows:
                r["t_pc"] = f"{t_pc:.6f}"
            pending_rows.extend(rows)

            total_fragments += 1
            total_samples += head["raw_count"]

            status_line = (
                f"\r  {total_fragments:>6} frags  |  {total_samples:>8} samples  |  "
                f"{link_losses} link loss  |  {windows_dropped} window(s) dropped "
                f"({gap_events} gaps, normal)  |  "
                f"{reassembly.incomplete_windows} incomplete  |  "
                f"tx_seq={tx_seq:<5} business_seq={business_seq:<6} "
                f"frag={frag_idx + 1}/{frag_count} type={head['window_type_name']:<8} "
                f"maxpkt={max_payload}B"
            )

            # Console + disk work is throttled: on Windows a conhost write
            # costs milliseconds and runs on the event loop thread, delaying
            # the next packet's t_pc — the exact quantity being measured.
            if t_pc - last_print >= PRINT_INTERVAL_S:
                last_print = t_pc
                print(status_line, end="", flush=True)
                if len(pending_rows) >= ROW_BATCH:
                    flush_rows()

            # Drain any late reassembly warnings without doing IO in the hot path.
            while reassembly.warnings:
                print(f"\n[WARN] {reassembly.warnings.pop(0)}")

        client = BleakClient(device, disconnected_callback=_on_disconnect)
        boost_handle = None
        try:
            await client.connect()

            # NOTE: do NOT act on this number. bleak's WinRT backend returns
            # GattSession.MaxPduSize, which is racy at connect time and very
            # often reads the 23-byte default before the session has updated
            # (bleak #1497). The authoritative figure is the largest
            # notification actually observed, reported in the summary — if
            # 220-byte fragments arrive at all, the real MTU is >= 223 by
            # definition, whatever this line says.
            def _read_mtu():
                try:
                    return int(client.mtu_size)
                except Exception:
                    return None

            mtu0 = _read_mtu()
            print(f"ATT MTU at connect: {mtu0 if mtu0 else 'unavailable'} "
                  f"(unreliable on Windows — see 'Effective ATT MTU' in the summary)")

            async def _recheck_mtu():
                await asyncio.sleep(2.0)
                m = _read_mtu()
                if m is not None and m != mtu0:
                    print(f"\n[MTU] settled to {m} bytes after 2 s "
                          f"(the connect-time value was a stale read)")
            mtu_task = asyncio.create_task(_recheck_mtu())

            boost_handle = setup_windows_conn_params(client, boost)

            print(f"Connected. Saving to '{csv_path}'  (Ctrl+C to stop)\n")
            await start_notify_with_retry(client, NUS_TX_UUID, on_notify)

            try:
                # Short sleeps rather than stop_evt.wait(): on Windows the
                # proactor loop needs to wake up for the SIGINT flag to be
                # processed at all.
                while not stop_evt.is_set():
                    await asyncio.sleep(0.25)
                    flush_rows()
            except (KeyboardInterrupt, asyncio.CancelledError):
                pass

            print(status_line, end="", flush=True)

            if not disconnected:
                try:
                    await client.stop_notify(NUS_TX_UUID)
                except Exception:
                    pass
        finally:
            try:
                mtu_task.cancel()
            except Exception:
                pass
            del boost_handle
            flush_rows()
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception:
                pass
            if prev_sigint is not None:
                try:
                    signal.signal(signal.SIGINT, prev_sigint)
                except Exception:
                    pass

    reassembly.finalize()
    for w in reassembly.warnings:
        print(f"\n[WARN] {w}")

    # ── summary ──
    print(f"\n\nDone. {total_fragments} fragments ({total_samples} samples) → {csv_path}")
    if disconnected:
        print("Session ended because the DEVICE DISCONNECTED, not by user request.")

    if max_payload:
        eff = max_payload + 3
        verdict = "OK" if eff >= MIN_USEFUL_MTU else "TOO SMALL"
        print(f"Effective ATT MTU >= {eff} bytes "
              f"(largest notification observed: {max_payload} B)  [{verdict}]")
        if eff < MIN_USEFUL_MTU:
            print("  ^ fragments are being split at the link layer. Check "
                  "CONFIG_BT_L2CAP_TX_MTU (app core)\n"
                  "    and CONFIG_BT_CTLR_DATA_LENGTH_MAX (net core).")
        else:
            print("  ^ MTU is not the bottleneck. Ignore any smaller number "
                  "printed at connect time.")

    if link_losses:
        print(f"Link loss: {link_losses} packet(s) actually lost over the air (tx_seq gaps).")
    else:
        print("Link loss: none — tx_seq was fully contiguous.")
    if dup_packets:
        print(f"Duplicates: {dup_packets}")
    if bad_packets:
        print(f"Unparseable packets: {bad_packets}")

    expected = windows_seen + windows_dropped
    if expected:
        print(f"Windows: {windows_seen} received / {expected} produced "
              f"({100.0 * windows_seen / expected:.1f}%), "
              f"{windows_dropped} dropped before the TX queue in {gap_events} gap(s).")
    if reassembly.incomplete_windows:
        print(f"Incomplete windows: {reassembly.incomplete_windows} "
              f"(see [WARN] lines above).")

    if len(arrival_ts) >= 3:
        span = arrival_ts[-1] - arrival_ts[0]
        deltas = sorted((b - a) * 1000.0 for a, b in zip(arrival_ts, arrival_ts[1:]))
        n = len(deltas)
        print(f"\nFragment arrival over {span:.1f} s:")
        print(f"  rate      {len(arrival_ts) / span:6.2f} frag/s  "
              f"(mean interval {span / (len(arrival_ts) - 1) * 1000:.1f} ms)")
        print(f"  min/p50   {deltas[0]:6.1f} / {statistics.median(deltas):.1f} ms")
        print(f"  p95/max   {deltas[int(n * 0.95)]:6.1f} / {deltas[-1]:.1f} ms")
        print("  (a tight p50 well above the connection interval means a fixed "
              "software delay in the\n   send path, not radio or link-layer "
              "contention — that is the thing to go find)")

    if len(lat_samples) >= 3:
        print(f"\nEnd-to-end latency drift (device capture → PC arrival, relative):")
        print(f"  start {lat_samples[0] * 1000:+.0f} ms   "
              f"mid {lat_samples[len(lat_samples) // 2] * 1000:+.0f} ms   "
              f"end {lat_samples[-1] * 1000:+.0f} ms")
        print("  (monotonic growth that then plateaus = a queue filled up and "
              "stayed full)")

    return 0


# Captures go here, not loose in tools/ — relative to the script's own
# location, so it lands in the same place regardless of where you invoke it.
CAPTURES_DIR = Path(__file__).resolve().parent / "captures"


def main() -> None:
    parser = argparse.ArgumentParser(description="GlassTENG ADC->BLE pipeline logger")
    parser.add_argument(
        "-o", "--output",
        help=f"CSV output file (default: {CAPTURES_DIR.name}/sm_window_log_YYYYMMDD_HHMMSS.csv). "
             f"A bare filename still goes into {CAPTURES_DIR.name}/; "
             f"a path containing a directory is used as-is.",
    )
    parser.add_argument("--name", default=DEVICE_NAME,
                        help=f"BLE device name (default: {DEVICE_NAME})")
    parser.add_argument("--boost", action="store_true",
                        help="Windows 11 only: ask the OS for ThroughputOptimized "
                             "connection parameters. Off by default so baseline "
                             "measurements are not silently altered.")
    args = parser.parse_args()

    if args.output:
        csv_path = Path(args.output)
        if csv_path.parent == Path("."):
            csv_path = CAPTURES_DIR / csv_path
    else:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = CAPTURES_DIR / f"sm_window_log_{ts}.csv"

    csv_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        rc = asyncio.run(run(args.name, csv_path, args.boost))
    except KeyboardInterrupt:
        # Belt and braces: if the signal handler could not be installed, at
        # least exit quietly instead of dumping a traceback.
        rc = 130
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
