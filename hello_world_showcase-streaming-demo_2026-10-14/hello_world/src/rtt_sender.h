/* SPDX-License-Identifier: Apache-2.0
 *
 * Streams full ADC windows over a dedicated RTT channel — see rtt_sender.c.
 */
#ifndef RTT_SENDER_H_
#define RTT_SENDER_H_

#include <stdint.h>
#include <stdbool.h>

/* Deliberately duplicate sampling_source.h's SM_ADC_NUM_CHANNELS /
 * SM_DELIVERY_RAW_CAPACITY rather than including that header — same
 * reasoning as ble_sender.h: this module shouldn't take a dependency on
 * the state machine's headers, sm_ble_bridge.c is the only place that
 * knows about both sides. Keep these in sync by hand if either changes. */
#define RTT_SM_NUM_CHANNELS    2
#define RTT_SM_MAX_RAW_SAMPLES 100

struct rtt_sm_window_info {
	uint32_t seq;
	uint32_t timestamp_ms;      /* k_uptime_get_32() when this window's last
				      * sample completed — see sampling_source.h's
				      * sm_buffer_meta.timestamp_ms */
	uint8_t  window_type;
	uint32_t sample_count;      /* window's true total (metadata) */
	bool     complete;
	uint8_t  raw_count;         /* how many of raw[] below are valid, <= RTT_SM_MAX_RAW_SAMPLES */
	int16_t  raw[RTT_SM_MAX_RAW_SAMPLES * RTT_SM_NUM_CHANNELS];   /* channel-interleaved */
};

/* Configures the dedicated RTT up-channel. Call once at boot, before any
 * rtt_sender_send_window() call. */
void rtt_sender_init(void);

/* Writes one whole window in a single SEGGER_RTT_Write() — unlike
 * ble_sender.c's BLE path, RTT isn't ATT-MTU-limited, so no
 * fragmentation is needed here even at RTT_SM_MAX_RAW_SAMPLES=100.
 * Non-blocking: if the host isn't reading fast enough and the RTT
 * up-buffer's ring is full, the whole write is silently dropped (never
 * partially written, never blocks the caller) — logged as a WRN, same
 * "never block the producer" policy as the rest of this pipeline. */
void rtt_sender_send_window(const struct rtt_sm_window_info *info);

#endif /* RTT_SENDER_H_ */
