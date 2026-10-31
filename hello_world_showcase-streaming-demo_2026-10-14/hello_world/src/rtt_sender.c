/* SPDX-License-Identifier: Apache-2.0
 *
 * Streams full (unfragmented) ADC windows over a dedicated SEGGER RTT
 * up-channel — a wired alternative to ble_sender.c's BLE path, same
 * window data, different transport, live rather than batch: rides the
 * existing J-Link/SWD connection, no radio or connection-interval
 * involved.
 *
 * Channel 1 (channel 0 stays the Terminal/log channel used by
 * CONFIG_LOG_BACKEND_RTT) — SEGGER_RTT_Conf.h's default
 * SEGGER_RTT_MAX_NUM_UP_BUFFERS=3 already provides it, no Kconfig change
 * needed.
 *
 * Deliberately NOT fragmented like the BLE path: RTT isn't ATT-MTU-limited,
 * a whole window (up to RTT_SM_MAX_RAW_SAMPLES=100 samples * 2 channels =
 * 400 bytes raw, 416 with header) fits in one SEGGER_RTT_Write() call.
 *
 * SEGGER_RTT_MODE_NO_BLOCK_SKIP (this channel's mode, set in
 * rtt_sender_init()): if the host isn't reading fast enough and the ring
 * buffer doesn't have room, a write is dropped WHOLE — SEGGER_RTT_Write()
 * returns 0, nothing is partially written, nothing blocks — matching
 * sm_core.c/ble_sender.c's own "never block the producer" policy, just
 * enforced by the RTT library instead of a k_msgq.
 */
#include "rtt_sender.h"

#include <string.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <SEGGER_RTT.h>

LOG_MODULE_REGISTER(rtt_sender, LOG_LEVEL_INF);

#define RTT_ADC_CHANNEL 1

/* One full window is up to 16(header) + 100*2*2(raw) = 416 bytes. Sized
 * for several windows' worth of slack even if the host briefly falls
 * behind, rather than tight. */
#define RTT_ADC_BUF_SIZE 4096

static uint8_t rtt_up_buffer[RTT_ADC_BUF_SIZE];

/* 0xA6 — deliberately different from ble_sender.c's BLE_SM_PACKET_TAG
 * (0xA5), so a byte-stream parser can never confuse the two transports'
 * framing even if pointed at the wrong channel/log by mistake. */
#define RTT_SM_PACKET_TAG 0xA6

struct __packed rtt_sm_packet {
	uint8_t  tag;
	uint32_t seq;
	uint32_t timestamp_ms;   /* device uptime (ms) when this window's last
				   * sample completed — see sampling_source.h's
				   * sm_buffer_meta */
	uint8_t  window_type;
	uint32_t sample_count;   /* whole window's true total */
	uint8_t  complete;
	uint8_t  raw_count;      /* samples in this packet, <= RTT_SM_MAX_RAW_SAMPLES */
	int16_t  raw[RTT_SM_MAX_RAW_SAMPLES * RTT_SM_NUM_CHANNELS];
};

void rtt_sender_init(void)
{
	SEGGER_RTT_ConfigUpBuffer(RTT_ADC_CHANNEL, "ADC_DATA", rtt_up_buffer,
				   sizeof(rtt_up_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);

	LOG_INF("RTT ADC data channel ready (channel %d, tag 0x%02x)",
		RTT_ADC_CHANNEL, RTT_SM_PACKET_TAG);
}

void rtt_sender_send_window(const struct rtt_sm_window_info *info)
{
	struct rtt_sm_packet pkt = {
		.tag = RTT_SM_PACKET_TAG,
		.seq = info->seq,
		.timestamp_ms = info->timestamp_ms,
		.window_type = info->window_type,
		.sample_count = info->sample_count,
		.complete = info->complete ? 1 : 0,
		.raw_count = info->raw_count,
	};

	memcpy(pkt.raw, info->raw,
	       (size_t)info->raw_count * RTT_SM_NUM_CHANNELS * sizeof(int16_t));

	size_t len = offsetof(struct rtt_sm_packet, raw) +
		     (size_t)info->raw_count * RTT_SM_NUM_CHANNELS * sizeof(int16_t);

	unsigned written = SEGGER_RTT_Write(RTT_ADC_CHANNEL, &pkt, len);

	if (written != len) {
		LOG_WRN("RTT ADC write dropped seq=%u (host not reading fast enough?)",
			info->seq);
	}
}
