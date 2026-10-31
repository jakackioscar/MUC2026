/* SPDX-License-Identifier: Apache-2.0
 *
 * Sampling power state machine. Three states (DEEP_IDLE / PULSE_WINDOW /
 * ACTIVE_CAPTURE), single-threaded event consumer. Ported unmodified from
 * nrf5340_firmware; this project only ever drives it into ACTIVE_CAPTURE
 * and keeps it there for the whole session (see main.c and Kconfig's
 * SM_SILENCE_TIMEOUT_MS) — DEEP_IDLE/PULSE_WINDOW exist in the code but
 * are never reached because nothing here posts EV_PULSE_DUE.
 *
 * Does NOT do filtering/HR/BP/PTT computation and does NOT touch BLE.
 * Only decides when to sample, how, and where the resulting buffer goes
 * (sm_take_delivered_buffer()).
 */
#ifndef SM_CORE_H_
#define SM_CORE_H_

#include <zephyr/kernel.h>
#include <stdint.h>

#include "sm_events.h"
#include "sampling_source.h"

enum sm_state {
	SM_STATE_DEEP_IDLE,
	SM_STATE_PULSE_WINDOW,
	SM_STATE_ACTIVE_CAPTURE,
};

/* Register a sampling_source backend (e.g. sampling_source_nrfx_install())
 * BEFORE calling this. Starts the state-machine's consumer thread, enters
 * DEEP_IDLE cold (no pre-sampling), does not start sampling. */
int sm_core_init(void);

/* Post an abstract event into the state machine's queue. Thread- and
 * ISR-safe (K_NO_WAIT msgq put). Returns 0, or -ENOMSG if the queue is
 * full — treated as a programming-error / runaway-event-source signal,
 * not an expected condition at the event rates this design assumes. */
int sm_post_event(const struct sm_event *ev);

/* Introspection for tests / RTT diagnostics. */
enum sm_state sm_get_state(void);
uint32_t sm_get_overflow_count(void);

/* Mainly for tests: the state machine's consumer thread is asynchronous,
 * so a test that does sm_post_event() then immediately checks
 * sm_get_state() races the thread. Call this right after each
 * sm_post_event() (one event at a time — see sm_core.c for why) to
 * deterministically wait for that event's dispatch to finish before
 * asserting. Returns 0 if a dispatch completed within `timeout`,
 * -EAGAIN on timeout. */
int sm_wait_idle(k_timeout_t timeout);

/* Drain one delivered window (hand-off point to sm_ble_bridge.c). Blocks
 * up to `timeout`. Returns a value snapshot copied out at delivery time,
 * NOT a pointer into backend storage — the backend only owns 2 physical
 * buffers that it overwrites in place, so a queued pointer would go
 * stale the moment a later fill reuses that slot.
 * Raw sample data is bounded/truncated (struct sm_delivered_window's
 * raw[]/raw_count, capped at SM_DELIVERY_RAW_CAPACITY per channel) — a
 * transport preview, not the full window; meta.sample_count reports the
 * true total. */
int sm_take_delivered_buffer(struct sm_delivered_window *out, k_timeout_t timeout);

/* Test-only: reset all state back to a cold DEEP_IDLE (stops timers/
 * sampling, drains both queues, zeroes the overflow counter). No
 * legitimate production use — exists so Ztest cases can start from a
 * known state without a fresh process per test. */
void sm_reset_for_test(void);

#endif /* SM_CORE_H_ */
