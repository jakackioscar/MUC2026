/* SPDX-License-Identifier: Apache-2.0
 *
 * Real nrfx SAADC sampling_source backend: TIMER -> DPPI -> SAADC SAMPLE
 * task, EasyDMA writes conversions straight into RAM, CPU only wakes on
 * the SAADC END/DONE event when a whole buffer is full (nrfx_saadc used
 * directly, not the Zephyr generic ADC API). Two channels (AIN0/AIN1,
 * see SM_ADC_NUM_CHANNELS) run in SAADC "scan" mode into one interleaved
 * buffer per fill: [S0 S1 | S0 S1 | ...].
 *
 * Modeled on Nordic's own nrfx_saadc "maximum performance" sample
 * (external TIMER -> GPPI -> SAMPLE, GPPI -> START-on-END, advanced
 * non-blocking mode; see modules/hal/nordic/nrfx/samples/src/nrfx_saadc/
 * maximum_performance/main.c in the NCS tree), adapted for: multi-channel
 * scan instead of 1, two buffer policies driven by the state machine
 * (BUF_SINGLE_BIG one-shot for PULSE_WINDOW, BUF_PING_PONG continuous
 * double-buffer for ACTIVE_CAPTURE), and sessions that start/stop
 * repeatedly instead of running once forever.
 *
 * HW-only — not validated by this code, a successful build, or
 * native_sim (this backend doesn't run there). Needs a real board, a
 * scope, and known inputs to confirm: DPPI channel allocation is
 * conflict-free, the TIMER triggers SAMPLE at the requested rate, the
 * scan order is really S1,S2,..., gain/reference produce the expected
 * code for a known input voltage, and END-to-EV_BUF_READY latency/jitter.
 *
 * trigger_hz (see nrfx_backend_start()) is the same as the configured
 * per-channel rate_hz, not multiplied by channel count — SAADC scan mode
 * advances through every configured channel on a single TIMER trigger,
 * confirmed against real-hardware timestamps.
 *
 * Gain/reference/channel-index-to-pin mapping are uniform placeholders:
 * per-channel gain differentiation and sensor identity are not decided
 * yet — this only proves the TIMER/DPPI/SAADC/EasyDMA chain works
 * end-to-end with one conservative gain covering 0-3.6V on every channel.
 * Channel 0/1 map to AIN0/AIN1 (P0.04/P0.05).
 *
 * Ported from nrf5340_firmware (ADC->BLE pipeline subset).
 */
#include "sampling_source.h"
#include "sampling_source_nrfx.h"
#include "sm_core.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>

LOG_MODULE_REGISTER(sampling_source_nrfx, LOG_LEVEL_INF);

/* ── HW-only placeholder channel config (see file header) ───────────────── */

#define NRFX_BACKEND_GAIN       NRF_SAADC_GAIN1_6      /* full scale = REFERENCE/GAIN */
#define NRFX_BACKEND_REFERENCE  NRF_SAADC_REFERENCE_INTERNAL /* 0.6V internal */
#define NRFX_BACKEND_ACQ_TIME   NRF_SAADC_ACQTIME_10US
#define NRFX_BACKEND_RESOLUTION NRF_SAADC_RESOLUTION_12BIT

/* Expected code value for a known input (12-bit single-ended, full
 * 2^resolution scale, not 2^(resolution-1)):
 *   full_scale_mV = REFERENCE_mV / GAIN = 600 / (1/6) = 3600 mV
 *   code(Vin_mV)  = round(Vin_mV * 4096 / 3600)
 *   0V   -> code ≈ 0     (small offset/noise is normal)
 *   3.0V -> code ≈ 3413
 */

#define NRFX_BACKEND_TIMER_INST_IDX 0  /* TIMER0, unclaimed elsewhere in this project */

/* Mirrors nrfx_saadc.h's own NRFX_SAADC_DEFAULT_CHANNEL_SE macro
 * (including its NRFX_COND_CODE_1 guards, so it stays correct across
 * SAADC hardware variants), substituting this backend's gain/acq_time
 * for that macro's built-in defaults. */
#define GLASSTENG_SAADC_CHANNEL_SE(_pin_p, _index)                        \
{                                                                          \
	.channel_config =                                                  \
	{                                                                   \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CH_CONFIG_RES,              \
				 (.resistor_p = NRF_SAADC_RESISTOR_DISABLED,\
				  .resistor_n = NRF_SAADC_RESISTOR_DISABLED,),\
				 ())                                        \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CH_GAIN,                     \
				 (.gain = NRFX_BACKEND_GAIN,),               \
				 ())                                        \
		.reference  = NRFX_BACKEND_REFERENCE,                       \
		.acq_time   = NRFX_BACKEND_ACQ_TIME,                        \
		.mode       = NRF_SAADC_MODE_SINGLE_ENDED,                  \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CH_BURST,                    \
				 (.burst = NRF_SAADC_BURST_DISABLED,),       \
				 ())                                        \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CH_CHOPPING,                 \
				 (.chopping = NRF_SAADC_CHOPPING_DISABLED,), \
				 ())                                        \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CH_HIGHSPEED,                \
				 (.highspeed = NRF_SAADC_HIGHSPEED_DISABLED,),\
				 ())                                        \
		NRFX_COND_CODE_1(NRF_SAADC_HAS_CONV_TIME,                   \
				 (.conv_time = NRFX_SAADC_DEFAULT_CONV_TIME,),\
				 ())                                        \
	},                                                                  \
	.pin_p          = (nrfx_analog_input_t)(_pin_p),                    \
	.pin_n          = NRFX_ANALOG_INPUT_DISABLED,                       \
	.channel_index  = (_index),                                         \
}

BUILD_ASSERT(SM_ADC_NUM_CHANNELS == 2, "channel table below is hand-written for 2 channels");

/* pin_p/pin_n MUST use NRFX_ANALOG_EXTERNAL_AINx (nrfx_analog_common.h),
 * NOT the same-looking-but-different NRF_SAADC_INPUT_AINx (nrf_saadc.h)
 * — this bit a real debugging session (see git history around the
 * multi-channel AIN0/AIN1 widening): nrfx_saadc_channel_t.pin_p is typed
 * `nrfx_analog_input_t`, a 0-based index (AIN0=0, AIN1=1, ...) used
 * directly into nrfx_saadc_ain_get()'s lookup table. NRF_SAADC_INPUT_AINx
 * is a SEPARATE enum, 1-based (NC=0 takes slot 0, so AIN0=1) — passing
 * it here silently shifts every channel one physical AIN pin higher than
 * intended (AIN0 configured AIN1, etc.), with no compiler warning
 * because of the explicit cast in GLASSTENG_SAADC_CHANNEL_SE below. */
static const nrfx_saadc_channel_t nrfx_backend_channels[SM_ADC_NUM_CHANNELS] = {
	GLASSTENG_SAADC_CHANNEL_SE(NRFX_ANALOG_EXTERNAL_AIN0, 0),
	GLASSTENG_SAADC_CHANNEL_SE(NRFX_ANALOG_EXTERNAL_AIN1, 1),
};

/* ── Buffers ──────────────────────────────────────────────────────────────
 *
 * Sized from the real Kconfig rate/window values — these back genuine
 * EasyDMA writes, so under-sizing would be memory corruption, not just a
 * logical shortcut.
 */

#define NRFX_BACKEND_MAX_PULSE_SAMPLES \
	(((uint32_t)CONFIG_SM_PULSE_WINDOW_MS * \
	  MAX(CONFIG_SM_SAMPLE_RATE_HR_HZ, CONFIG_SM_SAMPLE_RATE_BP_HZ)) / 1000)

static nrf_saadc_value_t pulse_buf[NRFX_BACKEND_MAX_PULSE_SAMPLES * SM_ADC_NUM_CHANNELS];
static nrf_saadc_value_t ping_pong_buf[2][CONFIG_SM_ACTIVITY_CHUNK_SAMPLES * SM_ADC_NUM_CHANNELS];

static struct sm_buffer bufs[2];

/* ── State ────────────────────────────────────────────────────────────── */

static struct sampling_config active_cfg;
static bool running;
static bool calibrated;
static uint32_t window_seq;
static enum sm_buffer_id next_ping_pong_buf;

static nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(NRFX_BACKEND_TIMER_INST_IDX));

/* Two DPPI connections (via nrfx_gppi), allocated once at install and
 * enabled/disabled per session rather than re-allocated each time — DPPI
 * channels are a limited shared HW resource. */
static nrfx_gppi_handle_t gppi_sampling_handle;     /* TIMER compare -> SAADC SAMPLE */
static nrfx_gppi_handle_t gppi_start_on_end_handle; /* SAADC END -> SAADC START (ping-pong only) */

/* ── Helpers ──────────────────────────────────────────────────────────── */

static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
	/* Never invoked: the TIMER's compare event drives SAADC SAMPLE
	 * entirely via the GPPI/DPPI hardware connection with its interrupt
	 * disabled (see nrfx_backend_start()) — the CPU only wakes on the
	 * SAADC END/DONE event. Required by nrfx_timer_init()'s signature
	 * regardless. */
	ARG_UNUSED(event_type);
	ARG_UNUSED(p_context);
}

static void saadc_evt_handler(nrfx_saadc_evt_t const *p_event)
{
	switch (p_event->type) {
	case NRFX_SAADC_EVT_CALIBRATEDONE:
		calibrated = true;
		LOG_INF("saadc: offset calibration done");
		break;

	case NRFX_SAADC_EVT_READY:
		/* First buffer of a new run has been acquired by the
		 * peripheral — safe to let the (already-running) external
		 * TIMER start driving SAMPLE now. */
		nrfx_gppi_conn_enable(gppi_sampling_handle);
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		if (running && active_cfg.buf_policy == BUF_PING_PONG) {
			enum sm_buffer_id next = next_ping_pong_buf;
			int err = nrfx_saadc_buffer_set(
				ping_pong_buf[next],
				CONFIG_SM_ACTIVITY_CHUNK_SAMPLES * SM_ADC_NUM_CHANNELS);

			if (err != 0) {
				LOG_ERR("saadc buffer_set (ping-pong buf %d): %d",
					next, err);
			}
			next_ping_pong_buf = (next == SM_BUF_0) ? SM_BUF_1 : SM_BUF_0;
		}
		/* BUF_SINGLE_BIG, or not running: deliberately supply
		 * nothing more — one-shot capture, no hardware auto-restart
		 * wanted; the driver reports NRFX_SAADC_EVT_FINISHED once
		 * the already-supplied buffer(s) complete. */
		break;

	case NRFX_SAADC_EVT_DONE: {
		nrf_saadc_value_t *p_buf = p_event->data.done.p_buffer;
		uint16_t raw_count = p_event->data.done.size;
		uint16_t logical_samples = raw_count / SM_ADC_NUM_CHANNELS;
		enum sm_buffer_id id = (p_buf == ping_pong_buf[SM_BUF_1]) ? SM_BUF_1 : SM_BUF_0;

		struct sm_buffer *b = &bufs[id];

		b->meta.window_type  = active_cfg.window;
		b->meta.window_seq   = window_seq;
		b->meta.rate_hz      = active_cfg.rate_hz;
		b->meta.sample_count = logical_samples;
		/* HW-only stub: real per-channel scan skew TBD. */
		memset(b->meta.per_channel_skew_us, 0, sizeof(b->meta.per_channel_skew_us));
		b->meta.complete = true;
		/* Device clock, captured right here at EVT_DONE (this buffer's
		 * last sample just completed) — see sm_buffer_meta's own
		 * comment for why this exists: PC-side "assumed sample rate"
		 * time reconstruction has been wrong before (trigger_hz bug),
		 * this gives every downstream consumer real elapsed device
		 * time instead. */
		b->meta.timestamp_ms = k_uptime_get_32();
		b->raw = (int16_t *)p_buf;

		if (active_cfg.buf_policy == BUF_PING_PONG) {
			/* One window_seq per delivered chunk. */
			window_seq++;
		}

		struct sm_event ev = { .type = EV_BUF_READY, .buf_id = id };

		sm_post_event(&ev);
		break;
	}

	case NRFX_SAADC_EVT_FINISHED:
		/* All supplied buffers exhausted. Disable both GPPI
		 * connections so the (still-enabled) TIMER's ticks stop
		 * reaching SAADC until the next start()+mode_trigger()
		 * cycle re-arms them. */
		nrfx_gppi_conn_disable(gppi_sampling_handle);
		nrfx_gppi_conn_disable(gppi_start_on_end_handle);
		break;

	default:
		break;
	}
}

/* ── sampling_source_api ─────────────────────────────────────────────── */

static int nrfx_backend_start(const struct sampling_config *cfg)
{
	if (running) {
		/* Idempotent backstop: the state machine's transition table
		 * never calls start() on an already-running session, but
		 * don't let a bug elsewhere corrupt a live session's
		 * buffers by racing them. */
		LOG_WRN("nrfx SAADC: start() called while already running — ignored");
		return 0;
	}

	active_cfg = *cfg;
	window_seq++;

	if (!calibrated) {
		/* Calibration runs once in sampling_source_nrfx_install()
		 * and normally finishes in well under a millisecond; not
		 * fatal if it hasn't yet — just a larger offset error. */
		LOG_WRN("saadc: starting before calibration finished");
	}

	uint32_t initial_size;
	nrf_saadc_value_t *initial_buf;

	if (cfg->buf_policy == BUF_SINGLE_BIG) {
		uint32_t exact_samples =
			(uint32_t)((uint64_t)CONFIG_SM_PULSE_WINDOW_MS * cfg->rate_hz / 1000);

		__ASSERT(exact_samples <= NRFX_BACKEND_MAX_PULSE_SAMPLES,
			 "pulse_buf too small for requested rate — Kconfig changed?");

		initial_buf = pulse_buf;
		initial_size = exact_samples * SM_ADC_NUM_CHANNELS;
	} else {
		next_ping_pong_buf = SM_BUF_1; /* buf 0 supplied here; BUF_REQ hands out buf 1 next */
		initial_buf = ping_pong_buf[SM_BUF_0];
		initial_size = CONFIG_SM_ACTIVITY_CHUNK_SAMPLES * SM_ADC_NUM_CHANNELS;
	}

	int err = nrfx_saadc_buffer_set(initial_buf, initial_size);

	if (err == -EALREADY) {
		/* The driver's own internal "secondary buffer" slot is
		 * still occupied from a just-stopped previous session —
		 * stop()'s nrfx_saadc_abort() clears it, but asynchronously,
		 * so a start() landing before that event is serviced can
		 * still race it. Force it and retry once. */
		LOG_WRN("saadc buffer_set: stale buffer from previous session, forcing abort+retry");
		nrfx_saadc_abort();
		err = nrfx_saadc_buffer_set(initial_buf, initial_size);
	}
	if (err != 0) {
		LOG_ERR("saadc buffer_set (initial): %d", err);
		/* running stays false — no half-started session left
		 * behind for the next start() attempt to trip over. */
		return -EIO;
	}

	running = true;

	/* See file header: one TIMER-driven SAMPLE trigger advances SAADC
	 * scan mode through every configured channel, so trigger_hz is just
	 * the configured per-channel rate, no multiply-by-channel-count. */
	uint32_t trigger_hz = cfg->rate_hz;
	uint32_t period_us = 1000000UL / trigger_hz;
	uint32_t ticks = nrfx_timer_us_to_ticks(&timer_inst, period_us);

	nrfx_timer_clear(&timer_inst);
	/* enable_int = false: the compare event only drives SAADC's SAMPLE
	 * task via DPPI in hardware — no CPU interrupt per sample.
	 * Auto-clear (SHORT_COMPARE0_CLEAR) makes it free-running. */
	nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks,
				     NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

	if (cfg->buf_policy == BUF_PING_PONG) {
		nrfx_gppi_conn_enable(gppi_start_on_end_handle);
	} else {
		nrfx_gppi_conn_disable(gppi_start_on_end_handle);
	}

	/* Start the TIMER (and START_ON_END, above) before mode_trigger() —
	 * any TIMER ticks landing before READY fires are simply not routed
	 * anywhere yet (harmless). */
	nrfx_timer_enable(&timer_inst);

	/* Arms the peripheral for this run — every prior session ends in
	 * NRFX_SAADC_EVT_FINISHED, which disables gppi_sampling_handle;
	 * mode_trigger() generates a fresh READY event to re-enable it. */
	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("saadc mode_trigger: %d", err);
		return -EIO;
	}

	LOG_INF("nrfx SAADC: start window=%d rate=%uHz (trigger=%uHz) policy=%d",
		cfg->window, cfg->rate_hz, trigger_hz, cfg->buf_policy);
	return 0;
}

static void nrfx_backend_stop(void)
{
	running = false;
	nrfx_timer_disable(&timer_inst);
	/* Idempotent: NRFX_SAADC_EVT_FINISHED normally already disables
	 * both once any in-flight buffer completes, but that can race this
	 * call — sm_core.c's drain_stale_buf_ready(), called right after
	 * this returns, already absorbs a late EV_BUF_READY from exactly
	 * that race. */
	nrfx_gppi_conn_disable(gppi_sampling_handle);
	nrfx_gppi_conn_disable(gppi_start_on_end_handle);

	/* Reset the SAADC driver's own internal buffer bookkeeping:
	 * nrfx_saadc_buffer_set() fails with -EALREADY if its internal
	 * "secondary buffer" slot (one buffer queued ahead via the last
	 * BUF_REQ) is still occupied, and nothing before this line clears
	 * it. A session that ends with a buffer still queued ahead (which
	 * ACTIVE_CAPTURE always has) would otherwise make the next
	 * start()'s first buffer_set() fail. nrfx_saadc_abort() triggers a
	 * hardware STOP; the resulting STOPPED+END sequence clears that
	 * slot and walks the driver back to idle through its normal
	 * DONE->FINISHED pair. This is asynchronous — see
	 * nrfx_backend_start()'s -EALREADY retry for the backstop covering
	 * the remaining race window. */
	nrfx_saadc_abort();

	LOG_INF("nrfx SAADC: stop");
}

static struct sm_buffer *nrfx_backend_get_buffer(enum sm_buffer_id buf_id)
{
	return &bufs[buf_id];
}

static struct sm_buffer *nrfx_backend_get_trailing_buffer(void)
{
	/* nrfx_saadc's public event API only reports complete buffers —
	 * there's no documented way to ask how many conversions EasyDMA
	 * has written into the in-flight ping-pong buffer before it's
	 * done. Always reports zero samples, so sm_deliver_if_nonempty()
	 * correctly never delivers it. */
	static struct sm_buffer trailing;

	trailing.meta.window_type  = active_cfg.window;
	trailing.meta.window_seq   = window_seq;
	trailing.meta.rate_hz      = active_cfg.rate_hz;
	trailing.meta.sample_count = 0;
	memset(trailing.meta.per_channel_skew_us, 0, sizeof(trailing.meta.per_channel_skew_us));
	trailing.meta.complete = true;
	trailing.raw = NULL;
	return &trailing;
}

static const struct sampling_source_api nrfx_backend_api = {
	.start = nrfx_backend_start,
	.stop = nrfx_backend_stop,
	.get_buffer = nrfx_backend_get_buffer,
	.get_trailing_buffer = nrfx_backend_get_trailing_buffer,
};

/* ── One-time hardware bring-up ──────────────────────────────────────── */

void sampling_source_nrfx_install(void)
{
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST,
		    nrfx_saadc_irq_handler, 0, 0);

	int err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);

	if (err != 0) {
		LOG_ERR("nrfx_saadc_init: %d", err);
		return;
	}

	err = nrfx_saadc_channels_config(nrfx_backend_channels, SM_ADC_NUM_CHANNELS);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_channels_config: %d", err);
		return;
	}

	/* Advanced mode, external-timer-triggered (internal_timer_cc = 0);
	 * hardware start-on-end is wired separately via
	 * gppi_start_on_end_handle below, so start_on_end stays false here
	 * (avoids a redundant software trigger). */
	nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

	adv_config.internal_timer_cc = 0;
	adv_config.start_on_end = false;

	uint32_t channel_mask = nrfx_saadc_channels_configured_get();

	err = nrfx_saadc_advanced_mode_set(channel_mask, NRFX_BACKEND_RESOLUTION,
					    &adv_config, saadc_evt_handler);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_advanced_mode_set: %d", err);
		return;
	}

	uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);

	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	err = nrfx_timer_init(&timer_inst, &timer_config, timer_handler);
	if (err != 0) {
		LOG_ERR("nrfx_timer_init: %d", err);
		return;
	}

	/* Allocate both DPPI connections once; enabled/disabled per
	 * session rather than allocated/freed each time. */
	err = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&timer_inst, NRF_TIMER_CC_CHANNEL0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
		&gppi_sampling_handle);
	if (err != 0) {
		LOG_ERR("gppi_conn_alloc (SAMPLING): %d", err);
		return;
	}

	err = nrfx_gppi_conn_alloc(
		nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START),
		&gppi_start_on_end_handle);
	if (err != 0) {
		LOG_ERR("gppi_conn_alloc (START_ON_END): %d", err);
		return;
	}

	sampling_source_register(&nrfx_backend_api);

	err = nrfx_saadc_offset_calibrate(saadc_evt_handler);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_offset_calibrate: %d", err);
	}

	LOG_INF("nrfx SAADC backend ready (channels=%u AIN0/AIN1, gain=1/6, ref=0.6V internal, "
		"HW-only placeholders)", SM_ADC_NUM_CHANNELS);
}
