"""
RTT ADC Data Logger — hello_world (nRF5340)
====================================
Connects to the target over J-Link/RTT and reads rtt_sender.c's dedicated binary data channel
(channel 1; channel 0 stays the Terminal/log channel), parsing whole,
unfragmented ADC windows straight into a CSV 

RTT packet layout (must match rtt_sender.c's struct rtt_sm_packet):

 Offset  Size  Field
 ------  ----  -----
   0      1    tag           uint8_t   0xA6, always — sanity check /
                                        resync point (deliberately
                                        different from ble_logger_1.py's
                                        0xA5, so the two transports'
                                        framing can never be confused)
   1      4    seq           uint32_t  window_seq from sm_core.c — same
                                        design-intentional-gap semantics
                                        as ble_logger_1.py's business_seq
   5      4    timestamp_ms  uint32_t  device uptime (k_uptime_get_32())
                                        when this window's LAST sample
                                        completed — real device clock, not
                                        an assumed/configured sample rate.
                                        Wraps at ~49.7 days; this script
                                        doesn't attempt to handle a wrap
                                        occurring mid-session.
   9      1    window_type   uint8_t   0=HR, 1=BP, 2=ACTIVITY
  10      4    sample_count  uint32_t  window's true total sample count
  14      1    complete      uint8_t   0/1
  15      1    raw_count     uint8_t   samples in this packet — the WHOLE
                                        window, no fragmentation
  16    var    raw[][2]      int16_t   raw_count*2 values, row-major (2
                                        channels — AIN0/AIN1)
 ──────────────────
 Header: 16 bytes fixed. Total: 16 + raw_count*2*2 bytes (up to 416 for
 a full 100-sample window).

Requirements:
    pip install pylink-square
    (also needs SEGGER J-Link software installed — same DLL J-Link RTT
    Viewer already uses)

Usage:
    python rtt_logger.py                 # auto-names CSV, saved under captures/
    python rtt_logger.py -o my_data.csv  # still saved under captures/ (bare filename)
    python rtt_logger.py --device NRF5340_XXAA_APP --channel 1   # both already the defaults
"""
import argparse
import csv
import datetime
import struct
import sys
import time
from pathlib import Path

import pylink

DEFAULT_DEVICE = "NRF5340_XXAA_APP"
DEFAULT_CHANNEL = 1

RTT_TAG = 0xA6
NUM_CHANNELS = 2   # AIN0/AIN1 — see sampling_source.h's SM_ADC_NUM_CHANNELS
HEADER_FMT = "<BIIBIBB"   # tag, seq, timestamp_ms, window_type, sample_count, complete, raw_count
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16

WINDOW_TYPE_NAMES = {0: "HR", 1: "BP", 2: "ACTIVITY"}

CAPTURES_DIR = Path(__file__).resolve().parent / "captures"

# timestamp_ms is device uptime (ms), same value repeated across every
# row of one window — filter_and_plot.py's load_csv() uses it (when
# present) instead of assuming a fixed sample rate for the time axis.
CSV_FIELDS = [
    "business_seq", "timestamp_ms", "window_type", "window_type_name", "total_sample_count",
    "complete", "raw_count", "sample_idx", "ch0_raw", "ch1_raw",
]


class PacketParser:
    """Stateful byte-stream framing over the raw RTT ring-buffer bytes.

    A single SEGGER_RTT_Write() on the target is not guaranteed to line
    up with a single rtt_read() on the host — one write can arrive split
    across several reads, or several writes can coalesce into one read.
    This buffers everything and only yields packets once a full one has
    accumulated, resyncing on the tag byte if anything ever looks off
    (should not normally happen — SEGGER_RTT_MODE_NO_BLOCK_SKIP either
    writes a packet whole or not at all, never a torn write).
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes) -> None:
        if data:
            self.buf.extend(data)

    def packets(self):
        """Yields every complete packet currently buffered, as
        (seq, timestamp_ms, window_type, sample_count, complete, raw_count,
        raw) tuples. raw is a tuple of raw_count*NUM_CHANNELS int16 values."""
        while True:
            idx = self.buf.find(bytes([RTT_TAG]))
            if idx == -1:
                # No tag anywhere in what's buffered — nothing usable yet.
                # Don't let pure noise (e.g. this channel misconfigured)
                # grow the buffer forever.
                if len(self.buf) > 4096:
                    del self.buf[:-1]
                return
            if idx > 0:
                del self.buf[:idx]   # drop garbage before the tag

            if len(self.buf) < HEADER_SIZE:
                return   # wait for more bytes

            tag, seq, timestamp_ms, window_type, sample_count, complete, raw_count = \
                struct.unpack_from(HEADER_FMT, self.buf, 0)

            total_len = HEADER_SIZE + raw_count * NUM_CHANNELS * 2
            if len(self.buf) < total_len:
                return   # wait for the rest of this packet

            raw = struct.unpack_from(f"<{raw_count * NUM_CHANNELS}h", self.buf, HEADER_SIZE)
            del self.buf[:total_len]
            yield (seq, timestamp_ms, window_type, sample_count, complete, raw_count, raw)


def connect(device: str) -> pylink.JLink:
    print(f"Connecting to J-Link, target '{device}' ...")
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(device)
    print(f"Connected: {jlink.product_name}")

    jlink.rtt_start()
    print("Waiting for RTT control block ...")
    while True:
        try:
            jlink.rtt_get_num_up_buffers()
            break
        except pylink.errors.JLinkRTTException:
            time.sleep(0.1)
    print("RTT control block found.")
    return jlink


def run(device: str, channel: int, csv_path: Path) -> None:
    jlink = connect(device)
    parser = PacketParser()

    total_windows = 0
    total_samples = 0
    seq_gaps = 0
    last_seq = None

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        print(f"Logging to '{csv_path}'  (Ctrl+C to stop)\n")
        try:
            while True:
                data = bytes(jlink.rtt_read(channel, 4096))
                parser.feed(data)

                for seq, timestamp_ms, window_type, sample_count, complete, raw_count, raw in parser.packets():
                    if last_seq is not None and seq != last_seq + 1:
                        seq_gaps += 1
                    last_seq = seq

                    rows = []
                    for i in range(raw_count):
                        base = i * NUM_CHANNELS
                        rows.append({
                            "business_seq":       seq,
                            "timestamp_ms":       timestamp_ms,
                            "window_type":        window_type,
                            "window_type_name":   WINDOW_TYPE_NAMES.get(window_type, f"UNKNOWN({window_type})"),
                            "total_sample_count": sample_count,
                            "complete":           complete,
                            "raw_count":          raw_count,
                            "sample_idx":         i,
                            "ch0_raw":            raw[base + 0],
                            "ch1_raw":            raw[base + 1],
                        })
                    writer.writerows(rows)
                    f.flush()

                    total_windows += 1
                    total_samples += raw_count

                    print(
                        f"\r  {total_windows:>6} windows  |  {total_samples:>8} samples  |  "
                        f"{seq_gaps} seq gap(s) (informational)  |  "
                        f"business_seq={seq:<6} type={WINDOW_TYPE_NAMES.get(window_type, '?'):<8}",
                        end="",
                        flush=True,
                    )

                if not data:
                    time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            jlink.rtt_stop()
            jlink.close()

    print(f"\n\nDone. {total_windows} windows ({total_samples} samples) -> {csv_path}")
    if seq_gaps:
        print(f"Note: {seq_gaps} business_seq gap(s) seen. Unlike ble_logger_1.py, this script "
              f"has no separate transport-level counter to tell design-intentional gaps (state "
              f"machine dropped/preempted a window before it ever reached rtt_sender.c) apart "
              f"from a real RTT ring-buffer overflow (host reading too slowly, whole write "
              f"skipped — see rtt_sender.c's SEGGER_RTT_MODE_NO_BLOCK_SKIP) — RTT sends one "
              f"unfragmented write per window, so there's nothing finer-grained to count. See "
              f"the CSV for which business_seq values are missing.")
    else:
        print("No business_seq gaps detected.")


def main() -> None:
    parser = argparse.ArgumentParser(description="GlassTENG RTT ADC data logger")
    parser.add_argument(
        "-o", "--output",
        help=f"CSV output file (default: {CAPTURES_DIR.name}/rtt_window_log_YYYYMMDD_HHMMSS.csv). "
             f"A bare filename (no directory) still goes into {CAPTURES_DIR.name}/; "
             f"a path containing a directory is used as-is.",
    )
    parser.add_argument(
        "--device", default=DEFAULT_DEVICE,
        help=f"J-Link target device name (default: {DEFAULT_DEVICE})",
    )
    parser.add_argument(
        "--channel", type=int, default=DEFAULT_CHANNEL,
        help=f"RTT up-channel index (default: {DEFAULT_CHANNEL} — must match "
             f"rtt_sender.c's RTT_ADC_CHANNEL)",
    )
    args = parser.parse_args()

    if args.output:
        csv_path = Path(args.output)
        if csv_path.parent == Path("."):
            csv_path = CAPTURES_DIR / csv_path
    else:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = CAPTURES_DIR / f"rtt_window_log_{ts}.csv"

    try:
        run(args.device, args.channel, csv_path)
    except pylink.errors.JLinkException as e:
        print(f"ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
