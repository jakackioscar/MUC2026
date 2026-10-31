/* SPDX-License-Identifier: Apache-2.0
 *
 * BLE sender module. Single direction: NUS TX only; RX callback is NULL.
 *
 * Real-time design: sm_ble_bridge.c queues each window here as soon as
 * sm_core.c delivers it (ble_sender_queue_sm_window()); ble_sm_tx_thread_fn
 * below drains that queue and sends fragments as fast as BLE allows. If the
 * queue fills up (BLE running behind production), the oldest queued window
 * is dropped and counted rather than blocking the producer — same
 * backpressure policy as sm_core.c's own delivery queue. This trades
 * completeness for staying live: for hour-scale unattended captures, RAM
 * usage has to stay flat regardless of session length, which a
 * capture-then-drain design (this project's earlier approach) cannot do —
 * see git history for that version and why it was replaced.
 */

#include "ble_sender.h"
#include "sampling_source.h"

#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_sender, LOG_LEVEL_DBG);

#define CON_STATUS_LED  DK_LED2

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ── BLE state ────────────────────────────────────────────────────────────── */

static struct bt_conn *current_conn;
static struct k_work   adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ── Connection callbacks ─────────────────────────────────────────────────── */

static const struct bt_le_conn_param fast_conn_param =
	BT_LE_CONN_PARAM_INIT(8, 16, 0, 400);

static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
				  ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("adv start failed: %d", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_ERR("connect failed %s err 0x%02x", addr, err);
		return;
	}
	LOG_INF("connected %s", addr);
	current_conn = bt_conn_ref(conn);
	dk_set_led_on(CON_STATUS_LED);

	int param_err = bt_conn_le_param_update(conn, &fast_conn_param);

	if (param_err) {
		LOG_WRN("bt_conn_le_param_update request: %d", param_err);
	}

	int phy_err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);

	if (phy_err) {
		LOG_WRN("bt_conn_le_phy_update request: %d", phy_err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("disconnected %s reason 0x%02x", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected          = connected,
	.disconnected       = disconnected,
	.recycled           = recycled_cb,
};

/* RX path intentionally absent: single-direction design. */
static struct bt_nus_cb nus_cb = {
	.received = NULL,
};

/* ── State-machine window queue ──────────────────────────────────────────── */

/* Deliberately independent of sampling_source.h's SM_ADC_NUM_CHANNELS
 * (currently also 2, but has been widened as high as 6 for past channel-
 * mapping debugging — see sampling_source.h) — BLE only ever needs the
 * first BLE_SM_TX_NUM_CHANNELS of each sample, picked out of win.raw[]'s
 * SM_ADC_NUM_CHANNELS-wide interleaving in the copy loop below. Keeps the
 * BLE wire format/fragment size fixed independent of how many channels the
 * ADC itself is sampling, in case that changes again. */
#define BLE_SM_TX_NUM_CHANNELS 2
BUILD_ASSERT(BLE_SM_TX_NUM_CHANNELS <= SM_ADC_NUM_CHANNELS,
	     "can't send more channels over BLE than the ADC actually samples");

#define BLE_SM_SAMPLES_PER_PKT 50
#define BLE_SM_PACKET_TAG 0xA5

/* Size (full fragment, BLE_SM_TX_NUM_CHANNELS channels): 1(tag) +
 * 2(tx_seq) + 4(seq) + 4(timestamp_ms) + 1(window_type) + 4(sample_count)
 * + 1(complete) + 1(frag_idx) + 1(frag_count) + 1(raw_count) +
 * 200(raw, 50*2*2) = 220 bytes — fits the 244-byte usable ATT payload
 * (MTU 247 - 3 byte header) with 24 bytes to spare. */
struct __packed ble_sm_packet {
	uint8_t  tag;
	uint16_t tx_seq;
	uint32_t seq;
	uint32_t timestamp_ms;
	uint8_t  window_type;
	uint32_t sample_count;
	uint8_t  complete;
	uint8_t  frag_idx;
	uint8_t  frag_count;
	uint8_t  raw_count;
	int16_t  raw[BLE_SM_SAMPLES_PER_PKT * BLE_SM_TX_NUM_CHANNELS];
};

#define SM_TX_QUEUE_DEPTH 4

K_MSGQ_DEFINE(sm_tx_q, sizeof(struct sm_delivered_window), SM_TX_QUEUE_DEPTH, 4);
/* atomic_t: incremented from this file's own TX thread, read from
 * ble_sender_get_overflow_count() by whatever wants a health check (e.g.
 * during a long unattended capture) — see sm_core.c's matching
 * overflow_count for the other half of the pipeline (windows dropped
 * before ever reaching this queue). Never resets on its own. */
static atomic_t sm_tx_overflow_count;

/* Transport-level counter, separate from struct sm_delivered_window's
 * business seq (window identity — sm_core.c assigns it, has
 * design-intentional gaps whenever a window is dropped/preempted before
 * it ever reaches sm_tx_q). tx_seq only increments here, once per actual
 * wire packet (one per FRAGMENT, not one per window), so it is what the
 * PC side should use to detect real link loss: any gap in tx_seq as
 * received means a packet that really went out never arrived. uint16_t
 * wraps at 65536 — the PC script must treat that as normal, not loss. */
static uint16_t sm_tx_seq;

int ble_sender_queue_sm_window(const struct sm_delivered_window *win)
{
	if (k_msgq_put(&sm_tx_q, win, K_NO_WAIT) != 0) {
		struct sm_delivered_window dropped;

		if (k_msgq_get(&sm_tx_q, &dropped, K_NO_WAIT) == 0) {
			uint32_t n = (uint32_t)atomic_inc(&sm_tx_overflow_count) + 1;

			LOG_WRN("sm TX queue full, dropped window_seq=%u (overflow #%u)",
				dropped.meta.window_seq, n);
		}
		k_msgq_put(&sm_tx_q, win, K_NO_WAIT);
	}
	return 0;
}

uint32_t ble_sender_get_overflow_count(void)
{
	return (uint32_t)atomic_get(&sm_tx_overflow_count);
}

static void ble_sm_tx_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct sm_delivered_window win;

	while (1) {
		k_msgq_get(&sm_tx_q, &win, K_FOREVER);

		if (!current_conn) {
			/* Never attempted onto the air — does not consume a
			 * tx_seq, same as a window the state machine itself
			 * dropped before it ever reached sm_tx_q. Whole
			 * window skipped, not per-fragment: there's nowhere
			 * to send any of it. */
			LOG_INF("ble_tx: business_seq=%u dropped (no BLE connection)",
				win.meta.window_seq);
			continue;
		}

		uint8_t frag_count = DIV_ROUND_UP(win.raw_count, BLE_SM_SAMPLES_PER_PKT);

		for (uint8_t f = 0; f < frag_count; f++) {
			uint16_t offset = (uint16_t)f * BLE_SM_SAMPLES_PER_PKT;
			uint8_t this_count = (uint8_t)MIN(BLE_SM_SAMPLES_PER_PKT,
							   win.raw_count - offset);

			struct ble_sm_packet pkt = {
				.tag = BLE_SM_PACKET_TAG,
				.tx_seq = sm_tx_seq++,
				.seq = win.meta.window_seq,
				.timestamp_ms = win.meta.timestamp_ms,
				.window_type = (uint8_t)win.meta.window_type,
				.sample_count = win.meta.sample_count,
				.complete = win.meta.complete ? 1 : 0,
				.frag_idx = f,
				.frag_count = frag_count,
				.raw_count = this_count,
			};

			/* win.raw[] is interleaved SM_ADC_NUM_CHANNELS-wide;
			 * pkt.raw[] only wants the first BLE_SM_TX_NUM_CHANNELS
			 * of each sample, so this is a strided copy, not a
			 * single memcpy. */
			for (uint8_t s = 0; s < this_count; s++) {
				const int16_t *src = &win.raw[(size_t)(offset + s) *
							       SM_ADC_NUM_CHANNELS];
				int16_t *dst = &pkt.raw[(size_t)s * BLE_SM_TX_NUM_CHANNELS];

				memcpy(dst, src, BLE_SM_TX_NUM_CHANNELS * sizeof(int16_t));
			}

			size_t pkt_len = offsetof(struct ble_sm_packet, raw) +
					  (size_t)this_count * BLE_SM_TX_NUM_CHANNELS * sizeof(int16_t);

			/* Not retried on failure: this is live data, a stalled
			 * connection should not back up the whole pipeline
			 * waiting for one fragment. Still increments tx_seq
			 * above even on failure — a real send attempt that
			 * never reached the PC should show up as a gap on the
			 * PC side, not be silently invisible. */
			int err = bt_nus_send(NULL, (const uint8_t *)&pkt, pkt_len);

			if (err) {
				LOG_WRN("bt_nus_send tx_seq=%u business_seq=%u frag=%u/%u: %d",
					pkt.tx_seq, win.meta.window_seq, f + 1, frag_count, err);
			} else {
				LOG_INF("ble_tx: sent tx_seq=%u business_seq=%u frag=%u/%u raw_count=%u",
					pkt.tx_seq, win.meta.window_seq, f + 1, frag_count, this_count);
			}
		}
	}
}

/* struct sm_delivered_window win (~440 B at SM_DELIVERY_RAW_CAPACITY=100 *
 * SM_ADC_NUM_CHANNELS=2) + struct ble_sm_packet pkt (~220 B) per fragment,
 * plus bt_nus_send()'s call depth through L2CAP/HCI. 2048 matches this
 * project's established sizing for that call depth (see git history —
 * this exact thread has overflowed a smaller stack before). */
K_THREAD_DEFINE(ble_sm_tx_tid, 2048, ble_sm_tx_thread_fn, NULL, NULL, NULL, 7, 0, 0);

/* ── Public API ───────────────────────────────────────────────────────────── */

int ble_sender_init(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("dk_leds_init: %d", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}
	LOG_INF("BT enabled");

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("bt_nus_init: %d", err);
		return err;
	}

	k_work_init(&adv_work, adv_work_handler);
	k_work_submit(&adv_work);

	return 0;
}
