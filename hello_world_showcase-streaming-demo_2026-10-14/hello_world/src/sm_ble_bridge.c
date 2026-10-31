/* SPDX-License-Identifier: Apache-2.0
 *
 * Bridges sm_core.c's delivered windows to two live destinations:
 *   - rtt_sender.c: best-effort, drops under load if the host isn't
 *     reading fast enough (see rtt_sender.c) — fine for a wired
 *     monitoring channel.
 *   - ble_sender.c: real-time, queued for immediate transmission
 *     (ble_sender_queue_sm_window()) — drops the oldest queued window if
 *     BLE falls behind production rather than blocking, same
 *     backpressure policy as sm_core.c's own delivery queue. This
 *     project previously buffered a whole session in RAM and drained it
 *     over BLE afterward instead (see git history) — abandoned for
 *     hour-scale unattended captures, where RAM usage has to stay flat
 *     regardless of session length, which a full-session buffer cannot
 *     do.
 *
 *   sm_core.c (producer) --sm_take_delivered_buffer()--> [this thread]
 *     --rtt_sender_send_window()--> rtt_sender.c --> SEGGER_RTT_Write()
 *       (synchronous, non-blocking, live)
 *     --ble_sender_queue_sm_window()--> ble_sender.c's sm_tx_q (producer
 *       side, K_NO_WAIT) --> ble_sm_tx_thread_fn (ble_sender.c) --> NUS
 *
 * sm_take_delivered_buffer(K_FOREVER) here is a plain consumer read on
 * sm_core.c's queue — it does not block sm_core.c's own producer thread,
 * which posts with K_NO_WAIT regardless of whether anything is consuming.
 * Neither ble_sender_queue_sm_window() nor rtt_sender_send_window() ever
 * blocks either, so this bridge never blocks waiting on BLE/radio or
 * RTT/host.
 */
#include "sm_ble_bridge.h"
#include "sm_core.h"
#include "sampling_source.h"
#include "ble_sender.h"
#include "rtt_sender.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(sm_ble_bridge, LOG_LEVEL_INF);

/* rtt_sender.h deliberately duplicates sampling_source.h's
 * SM_DELIVERY_RAW_CAPACITY/SM_ADC_NUM_CHANNELS rather than including that
 * header (see rtt_sender.h) — this catches them ever drifting apart
 * instead of silently truncating or overrunning. ble_sender.h has no
 * equivalent pair of constants to check: it takes struct
 * sm_delivered_window directly, so there is nothing to drift. */
BUILD_ASSERT(SM_DELIVERY_RAW_CAPACITY == RTT_SM_MAX_RAW_SAMPLES,
	     "sm_ble_bridge.c copies raw[] 1:1, sizes must match");
BUILD_ASSERT(SM_ADC_NUM_CHANNELS == RTT_SM_NUM_CHANNELS,
	     "sm_ble_bridge.c copies raw[] 1:1, channel counts must match");

static void bridge_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct sm_delivered_window win;

	while (1) {
		if (sm_take_delivered_buffer(&win, K_FOREVER) != 0) {
			continue;
		}

		struct rtt_sm_window_info rtt_info = {
			.seq = win.meta.window_seq,
			.timestamp_ms = win.meta.timestamp_ms,
			.window_type = (uint8_t)win.meta.window_type,
			.sample_count = win.meta.sample_count,
			.complete = win.meta.complete,
			.raw_count = win.raw_count,
		};

		memcpy(rtt_info.raw, win.raw,
		       (size_t)win.raw_count * SM_ADC_NUM_CHANNELS * sizeof(int16_t));

		rtt_sender_send_window(&rtt_info);

		ble_sender_queue_sm_window(&win);
	}
}

/* bridge_thread_fn's two locals (struct sm_delivered_window win + struct
 * rtt_sm_window_info rtt_info) are ~440 bytes EACH at
 * SM_DELIVERY_RAW_CAPACITY/RTT_SM_MAX_RAW_SAMPLES=100 * SM_ADC_NUM_CHANNELS/
 * RTT_SM_NUM_CHANNELS=2 — ~880 bytes combined. Has overflowed a smaller
 * stack before at higher channel counts (see git history) — left with
 * real margin above that. */
K_THREAD_DEFINE(sm_ble_bridge_tid, 3072, bridge_thread_fn, NULL, NULL, NULL, 7, 0, 0);

void sm_ble_bridge_init(void)
{
	LOG_INF("sm-to-BLE bridge ready (real-time)");
}
