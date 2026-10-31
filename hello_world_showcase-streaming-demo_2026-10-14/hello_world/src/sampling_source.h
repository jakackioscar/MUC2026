/* SPDX-License-Identifier: Apache-2.0
 *
 * Sampling source interface for the state machine's logic layer.
 *
 * Exactly one backend is active at a time (sampling_source_register()).
 * This project only ever installs sampling_source_nrfx.c (real nrfx
 * SAADC) — ported from nrf5340_firmware, which also has a synthetic
 * software backend (sampling_source_synth.c) not carried over here.
 */
#ifndef SAMPLING_SOURCE_H_
#define SAMPLING_SOURCE_H_

#include <stdint.h>
#include <stdbool.h>

#include "sm_events.h"

/* Two TENG channels (AIN0/AIN1, see sampling_source_nrfx.c), always
 * sampled interleaved [S0 S1 | S0 S1 | ...]. Was temporarily widened to
 * 6 (AIN0-AIN5) to chase an AIN0/AIN1 channel-mapping bug — see
 * sampling_source_nrfx.c's channel table comment for the actual root
 * cause found (a wrong-enum-type bug, nothing to do with channel count)
 * — narrowed back to the 2 channels this project actually uses now that
 * it's fixed and confirmed on real hardware. */
#define SM_ADC_NUM_CHANNELS 2

enum sm_window_type {
	WIN_HR,        /* PULSE_WINDOW, heart-rate config */
	WIN_BP,        /* PULSE_WINDOW, blood-pressure/PTT config */
	WIN_ACTIVITY,  /* ACTIVE_CAPTURE config */
};

enum sm_buffer_policy {
	BUF_SINGLE_BIG,   /* PULSE_WINDOW: one big buffer, whole window at once */
	BUF_PING_PONG,    /* ACTIVE_CAPTURE: two buffers, continuous */
};

/* HW-only, do not treat as validated: real rates TBD once front-end and
 * PTT-resolution requirements are confirmed on real hardware. Backed by
 * Kconfig so they're tunable without touching this file — see Kconfig
 * symbols SM_SAMPLE_RATE_{HR,BP,ACTIVITY}_HZ. */
struct sm_window_profile {
	uint32_t rate_hz;
	enum sm_buffer_policy buf_policy;
};

/* Indexed by enum sm_window_type. Defined in sampling_source.c. */
extern const struct sm_window_profile sm_window_profiles[3];

struct sampling_config {
	enum sm_window_type window;
	/* Resolved from sm_window_profiles[window] by sm_core.c; carried
	 * here too so callers/logs don't have to re-look-up the table. */
	uint32_t rate_hz;
	enum sm_buffer_policy buf_policy;
};

/* ── PTT metadata ─────────────────────────────────────────────────────────
 *
 * Every buffer handed to the (not-yet-built) processing layer carries
 * this. per_channel_skew_us is the fixed, known offset of each channel's
 * sample instant from the scan's t0 — a constant of the sampling
 * hardware/config (single SAADC scan interleaving S1/S2/S3), not
 * something measured per-buffer.
 *
 * HW-only: the real skew constant is TBD once the real nrfx scan-mode
 * backend exists.
 */
struct sm_buffer_meta {
	enum sm_window_type window_type;
	uint32_t window_seq;                /* monotonic window counter */
	uint32_t rate_hz;
	uint32_t sample_count;              /* samples per channel in this buffer */
	uint32_t per_channel_skew_us[SM_ADC_NUM_CHANNELS];
	/* true = meaningful captured data (full ping-pong chunk, full single
	 * window, or the trailing partial chunk at a normal ACTIVE_CAPTURE
	 * exit). false = reserved for a preempted half pulse-window; per the
	 * transition table that case is never actually delivered, so this
	 * mostly documents intent for logging. */
	bool complete;
	/* k_uptime_get_32() at the moment this buffer's LAST sample
	 * completed (see sampling_source_nrfx.c's NRFX_SAADC_EVT_DONE
	 * handler) — device clock, milliseconds since boot, wraps at
	 * ~49.7 days (uint32_t ms). Added specifically because assumed
	 * sample-rate-based time reconstruction (PC-side tools) has
	 * repeatedly been wrong in this project's own history (see
	 * sampling_source_nrfx.c's trigger_hz note) — this lets any
	 * consumer measure real elapsed device time between windows
	 * instead of trusting a configured/assumed rate. */
	uint32_t timestamp_ms;
};

struct sm_buffer {
	struct sm_buffer_meta meta;
	int16_t *raw;   /* sample_count * SM_ADC_NUM_CHANNELS, channel-interleaved */
};

/* ── Delivery snapshot (sm_take_delivered_buffer()) ──────────────────────
 *
 * struct sm_buffer above holds a backend-owned pointer (buf->raw) into
 * storage the backend reuses/overwrites every fill cycle — fine for
 * immediate use inside sm_core.c's dispatch, unsafe to carry across a
 * queue a consumer drains later. sm_core.c's delivery queue instead
 * copies a bounded number of raw samples BY VALUE at delivery time.
 *
 * SM_DELIVERY_RAW_CAPACITY is a transport/preview cap, not a claim about
 * how large real captured windows are — it exists so a delivery snapshot
 * has a small, fixed size. meta.sample_count still reports the window's
 * true total;
 * raw_count reports how many of raw[] below are actually valid
 * (min(sample_count, SM_DELIVERY_RAW_CAPACITY)).
 */
/* Must stay >= CONFIG_SM_ACTIVITY_CHUNK_SAMPLES or every window silently
 * truncates again — see sampling_source.c's BUILD_ASSERT. Sized to 100
 * (== the default chunk size) specifically so ACTIVE_CAPTURE windows
 * carry every sample through to BLE, not just a preview: ble_sender.c
 * fragments a whole window across multiple notify packets instead of
 * capping what gets queued here. */
#define SM_DELIVERY_RAW_CAPACITY 100

struct sm_delivered_window {
	struct sm_buffer_meta meta;
	uint8_t raw_count;
	int16_t raw[SM_DELIVERY_RAW_CAPACITY * SM_ADC_NUM_CHANNELS];
};

/* ── Sampling source driver interface ────────────────────────────────────── */

struct sampling_source_api {
	int  (*start)(const struct sampling_config *cfg);
	void (*stop)(void);
	/* Only valid to call after the matching EV_BUF_READY has been
	 * consumed, or (get_trailing_buffer) right after stop(). */
	struct sm_buffer *(*get_buffer)(enum sm_buffer_id buf_id);
	/* Best-effort partial buffer as of the moment stop() was called —
	 * used for ACTIVE_CAPTURE's silence-timeout exit (take the trailing
	 * buffer). May return NULL if there is nothing to report. */
	struct sm_buffer *(*get_trailing_buffer)(void);
};

void sampling_source_register(const struct sampling_source_api *api);
int  sampling_source_start(const struct sampling_config *cfg);
void sampling_source_stop(void);
struct sm_buffer *sampling_source_get_buffer(enum sm_buffer_id buf_id);
struct sm_buffer *sampling_source_get_trailing_buffer(void);

#endif /* SAMPLING_SOURCE_H_ */
