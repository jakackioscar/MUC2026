/* SPDX-License-Identifier: Apache-2.0
 *
 * Sampling power state machine: DEEP_IDLE / PULSE_WINDOW / ACTIVE_CAPTURE.
 * Single-threaded event consumer (see sm_thread_fn) — all transitions run
 * on one thread, so there's no locking between states. See sm_core.h for
 * the public API and the transition table.
 *
 * Ported unmodified from nrf5340_firmware (ADC->BLE pipeline subset).
 * main.c drives this into ACTIVE_CAPTURE once at boot with a single
 * EV_ACTIVITY kick and Kconfig's SM_SILENCE_TIMEOUT_MS is set high enough
 * that it never times back out to DEEP_IDLE during a normal session —
 * see main.c's comment. No code below is aware of that; it behaves
 * exactly as it does in the source project.
 */
#include "sm_core.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

LOG_MODULE_REGISTER(sm_core, LOG_LEVEL_INF);

#define SM_EVENT_QUEUE_DEPTH    8
#define SM_DELIVERY_QUEUE_DEPTH 2   /* matches ping-pong depth */
/* sm_deliver_buffer()'s local `struct sm_delivered_window snapshot` alone
 * is ~440 bytes at SM_DELIVERY_RAW_CAPACITY=100 * SM_ADC_NUM_CHANNELS=2
 * (raw[100*2] int16, see sampling_source.h). Was bumped as high as 4096
 * during a temporary 6-channel debug widening (see sampling_source.h);
 * narrowed back down now that channel count is back to 2, with the same
 * comfortable margin this size always had at 2 channels. */
#define SM_THREAD_STACK_SIZE    2048
#define SM_THREAD_PRIORITY      6

K_MSGQ_DEFINE(sm_event_q, sizeof(struct sm_event), SM_EVENT_QUEUE_DEPTH, 4);
/* Stores struct sm_delivered_window BY VALUE, not a pointer into backend
 * storage — the backend only owns 2 physical buffers that get overwritten
 * every fill cycle, so a queued pointer would go stale as soon as the next
 * fill reuses that slot. k_msgq's copy-in/copy-out gives a proper snapshot
 * for free, including a bounded number of raw samples. */
K_MSGQ_DEFINE(sm_delivery_q, sizeof(struct sm_delivered_window), SM_DELIVERY_QUEUE_DEPTH, 4);
K_SEM_DEFINE(sm_event_processed, 0, 1);

static enum sm_state current_state = SM_STATE_DEEP_IDLE;
static struct sampling_config current_cfg;
/* atomic_t: incremented from this file's own thread, read via
 * sm_get_overflow_count() from whatever wants a health check — needs to
 * be read cross-thread safely. This queue (upstream of both
 * rtt_sender.c and ble_sender.c, see sm_ble_bridge.c) overflowing means
 * the bridge thread itself fell behind, which shouldn't normally happen
 * since both its consumers are non-blocking. */
static atomic_t overflow_count;

static struct k_timer window_timer;
static struct k_timer silence_timer;

/* ── Delivery (hand-off to sm_ble_bridge.c) ──────────────────────────────
 *
 * sm_ble_bridge.c is the only consumer of sm_take_delivered_buffer().
 * Nothing else drains it, so on real hardware it will fill after the
 * first two buffers and start dropping oldest + counting if the bridge
 * thread ever falls behind — that's expected, not a bug.
 */
static void sm_deliver_buffer(struct sm_buffer *buf)
{
	if (!buf) {
		return;
	}

	struct sm_delivered_window snapshot = {
		.meta = buf->meta,
	};

	if (buf->raw) {
		snapshot.raw_count = (uint8_t)MIN(buf->meta.sample_count,
						   SM_DELIVERY_RAW_CAPACITY);
		memcpy(snapshot.raw, buf->raw,
		       (size_t)snapshot.raw_count * SM_ADC_NUM_CHANNELS * sizeof(int16_t));
	}

	if (k_msgq_put(&sm_delivery_q, &snapshot, K_NO_WAIT) != 0) {
		struct sm_delivered_window dropped;

		if (k_msgq_get(&sm_delivery_q, &dropped, K_NO_WAIT) == 0) {
			uint32_t n = (uint32_t)atomic_inc(&overflow_count) + 1;

			LOG_WRN("delivery queue full, dropped window_seq=%u (overflow #%u)",
				dropped.meta.window_seq, n);
		}
		k_msgq_put(&sm_delivery_q, &snapshot, K_NO_WAIT);
	}

	LOG_INF("delivered: window_type=%d seq=%u samples=%u complete=%d raw_count=%u",
		snapshot.meta.window_type, snapshot.meta.window_seq,
		snapshot.meta.sample_count, snapshot.meta.complete, snapshot.raw_count);
}

/* Only deliver buffers that actually have samples — an empty buffer
 * (sample_count == 0) isn't "complete data", it's nothing, and shouldn't
 * be marked/delivered as if it were. */
static void sm_deliver_if_nonempty(struct sm_buffer *buf)
{
	if (buf && buf->meta.sample_count > 0) {
		sm_deliver_buffer(buf);
	}
}

/* Drain any EV_BUF_READY still sitting in the event queue right after
 * sampling_source_stop() returns — the backend's own fill timer can race
 * the exit-triggering EV_TIMEOUT and land a buffer-ready event just after
 * it. Call AFTER stop() (no new EV_BUF_READY can appear) but BEFORE
 * enter_deep_idle() (a late one is still delivered as this session's
 * data, not silently dropped as an "ignoring event" warning). */
static void drain_stale_buf_ready(void)
{
	struct sm_event ev;

	while (k_msgq_get(&sm_event_q, &ev, K_NO_WAIT) == 0) {
		if (ev.type == EV_BUF_READY) {
			LOG_INF("draining stale EV_BUF_READY that raced session exit");
			sm_deliver_if_nonempty(sampling_source_get_buffer(ev.buf_id));
		} else {
			/* Not ours — put it back for the next state to handle
			 * normally, and stop draining (don't reorder anything
			 * behind it). Extremely unlikely at this exact point,
			 * but don't silently eat a real event either. */
			sm_post_event(&ev);
			break;
		}
	}
}

/* ── Timers ────────────────────────────────────────────────────────────
 * k_timer expiry callbacks run in ISR context in Zephyr — kept minimal,
 * only touching sm_post_event() (ISR-safe K_NO_WAIT msgq put).
 */
static void window_timer_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	struct sm_event ev = { .type = EV_TIMEOUT, .timeout_kind = SM_TIMEOUT_WINDOW_FULL };
	sm_post_event(&ev);
}

static void silence_timer_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	struct sm_event ev = { .type = EV_TIMEOUT, .timeout_kind = SM_TIMEOUT_SILENCE };
	sm_post_event(&ev);
}

/* ── State entry ──────────────────────────────────────────────────────── */

static void enter_deep_idle(void)
{
	current_state = SM_STATE_DEEP_IDLE;
	LOG_INF("state -> DEEP_IDLE");
}

static void enter_pulse_window(enum sm_pulse_kind kind)
{
	enum sm_window_type win = (kind == SM_PULSE_BP) ? WIN_BP : WIN_HR;

	current_cfg.window = win;
	current_cfg.rate_hz = sm_window_profiles[win].rate_hz;
	current_cfg.buf_policy = sm_window_profiles[win].buf_policy;

	sampling_source_start(&current_cfg);

	/* Backstop only — under normal operation this never fires, because
	 * sm_post_event() cancels it synchronously the moment the fill posts
	 * EV_BUF_READY (see sm_post_event() below). */
	k_timer_start(&window_timer, K_MSEC(CONFIG_SM_PULSE_WINDOW_MS), K_NO_WAIT);

	current_state = SM_STATE_PULSE_WINDOW;
	LOG_INF("state -> PULSE_WINDOW (window=%d rate=%uHz)", win, current_cfg.rate_hz);
}

static void enter_active_capture(void)
{
	current_cfg.window = WIN_ACTIVITY;
	current_cfg.rate_hz = sm_window_profiles[WIN_ACTIVITY].rate_hz;
	current_cfg.buf_policy = sm_window_profiles[WIN_ACTIVITY].buf_policy;

	sampling_source_start(&current_cfg);

	k_timer_start(&silence_timer, K_MSEC(CONFIG_SM_SILENCE_TIMEOUT_MS), K_NO_WAIT);

	current_state = SM_STATE_ACTIVE_CAPTURE;
	LOG_INF("state -> ACTIVE_CAPTURE");
}

/* ── Per-state event handlers ─────────────────────────────────────────── */

static void handle_deep_idle(const struct sm_event *ev)
{
	switch (ev->type) {
	case EV_PULSE_DUE:
		enter_pulse_window(ev->pulse_kind);
		break;
	case EV_ACTIVITY:
		enter_active_capture();
		break;
	default:
		LOG_WRN("DEEP_IDLE: ignoring event type=%d", ev->type);
		break;
	}
}

static void handle_pulse_window(const struct sm_event *ev)
{
	switch (ev->type) {
	case EV_TIMEOUT:
		if (ev->timeout_kind != SM_TIMEOUT_WINDOW_FULL) {
			LOG_WRN("PULSE_WINDOW: unexpected timeout_kind=%d", ev->timeout_kind);
			break;
		}
		/* Exit action, in this order: stop the backend first (no more
		 * EV_BUF_READY can appear after this), drain anything that
		 * raced the stop and is already queued, THEN deliver and
		 * change state. */
		sampling_source_stop();
		drain_stale_buf_ready();
		sm_deliver_if_nonempty(sampling_source_get_buffer(SM_BUF_0));
		enter_deep_idle();
		break;

	case EV_BUF_READY: {
		/* Stay in PULSE_WINDOW; check whether the big buffer is
		 * actually full. If so, short-circuit by self-generating the
		 * window-full timeout instead of waiting for window_timer's
		 * backstop to also expire. */
		struct sm_buffer *b = sampling_source_get_buffer(ev->buf_id);
		uint32_t expected = (uint32_t)((uint64_t)CONFIG_SM_PULSE_WINDOW_MS *
						current_cfg.rate_hz / 1000);

		if (b && b->meta.sample_count >= expected) {
			k_timer_stop(&window_timer);
			struct sm_event timeout_ev = {
				.type = EV_TIMEOUT,
				.timeout_kind = SM_TIMEOUT_WINDOW_FULL,
			};
			sm_post_event(&timeout_ev);
		}
		/* else: not full yet, stay and wait. */
		break;
	}

	case EV_ACTIVITY:
		/* Priority: activity preempts pulse. Exit action: discard the
		 * half-filled pulse window — incomplete, never delivered —
		 * and stop pulse sampling. Entry action: switch to
		 * ping-pong, start silence timer. */
		k_timer_stop(&window_timer);
		sampling_source_stop();
		drain_stale_buf_ready();
		LOG_WRN("PULSE_WINDOW preempted by EV_ACTIVITY: half-window discarded (incomplete)");
		enter_active_capture();
		break;

	default:
		LOG_WRN("PULSE_WINDOW: ignoring event type=%d", ev->type);
		break;
	}
}

static void handle_active_capture(const struct sm_event *ev)
{
	switch (ev->type) {
	case EV_TIMEOUT:
		if (ev->timeout_kind != SM_TIMEOUT_SILENCE) {
			LOG_WRN("ACTIVE_CAPTURE: unexpected timeout_kind=%d", ev->timeout_kind);
			break;
		}
		/* Exit action, in this order: stop the backend first (no more
		 * EV_BUF_READY can appear after this), drain anything that
		 * raced the stop and is already queued, THEN take the
		 * trailing buffer — only if it actually has samples — and
		 * change state. */
		sampling_source_stop();
		drain_stale_buf_ready();
		sm_deliver_if_nonempty(sampling_source_get_trailing_buffer());
		enter_deep_idle();
		break;

	case EV_ACTIVITY:
		/* Self-loop: reset (extend) the silence timeout. */
		k_timer_start(&silence_timer, K_MSEC(CONFIG_SM_SILENCE_TIMEOUT_MS), K_NO_WAIT);
		break;

	case EV_PULSE_DUE:
		/* No preemption this direction: drop, don't queue, don't
		 * backfill later. */
		LOG_INF("ACTIVE_CAPTURE: EV_PULSE_DUE ignored (activity has priority)");
		break;

	case EV_BUF_READY: {
		/* Data must not be lost: take the full buffer first. The
		 * backend keeps filling the other ping-pong slot on its own
		 * — swapping to the next buffer happens inside the backend,
		 * not the state machine, nothing else to do here. */
		struct sm_buffer *b = sampling_source_get_buffer(ev->buf_id);

		sm_deliver_if_nonempty(b);
		break;
	}

	default:
		LOG_WRN("ACTIVE_CAPTURE: ignoring event type=%d", ev->type);
		break;
	}
}

/* ── Dispatch + thread ────────────────────────────────────────────────── */

static void dispatch(const struct sm_event *ev)
{
	switch (current_state) {
	case SM_STATE_DEEP_IDLE:
		handle_deep_idle(ev);
		break;
	case SM_STATE_PULSE_WINDOW:
		handle_pulse_window(ev);
		break;
	case SM_STATE_ACTIVE_CAPTURE:
		handle_active_capture(ev);
		break;
	}
}

static void sm_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct sm_event ev;

	while (1) {
		k_msgq_get(&sm_event_q, &ev, K_FOREVER);
		dispatch(&ev);
		k_sem_give(&sm_event_processed);
	}
}

K_THREAD_DEFINE(sm_tid, SM_THREAD_STACK_SIZE, sm_thread_fn, NULL, NULL, NULL,
		 SM_THREAD_PRIORITY, 0, 0);

/* ── Public API ───────────────────────────────────────────────────────── */

int sm_core_init(void)
{
	/* Timers are init'd once here (never re-init'd on state transitions)
	 * so sm_reset_for_test() can safely stop them even before either
	 * state has ever been entered. */
	k_timer_init(&window_timer, window_timer_expiry, NULL);
	k_timer_init(&silence_timer, silence_timer_expiry, NULL);

	/* Cold start: DEEP_IDLE, no pre-sampling. sm_thread_fn is already
	 * running (K_THREAD_DEFINE auto-starts) and blocked on
	 * k_msgq_get(K_FOREVER), waiting for the first event. */
	current_state = SM_STATE_DEEP_IDLE;
	LOG_INF("state machine cold-started into DEEP_IDLE");
	return 0;
}

void sm_reset_for_test(void)
{
	k_timer_stop(&window_timer);
	k_timer_stop(&silence_timer);
	sampling_source_stop();
	current_state = SM_STATE_DEEP_IDLE;
	atomic_set(&overflow_count, 0);

	struct sm_event ev;
	while (k_msgq_get(&sm_event_q, &ev, K_NO_WAIT) == 0) {
		/* drain */
	}
	struct sm_delivered_window win;
	while (k_msgq_get(&sm_delivery_q, &win, K_NO_WAIT) == 0) {
		/* drain */
	}
	k_sem_reset(&sm_event_processed);
}

int sm_post_event(const struct sm_event *ev)
{
	if (ev->type == EV_BUF_READY) {
		/* Cancel PULSE_WINDOW's window_timer backstop synchronously,
		 * here (before the event is even queued), not later after
		 * the state-machine thread dequeues it. window_timer is
		 * armed with the same nominal duration as the backend's fill
		 * timer, so both can land on the same system tick — Zephyr's
		 * timeout list removes a timer before invoking its callback,
		 * so a same-tick k_timer_stop() called from an earlier
		 * timer's own callback reliably prevents the later one from
		 * firing at all. No-op if window_timer isn't running.
		 * k_timer_stop() is ISR-safe. */
		k_timer_stop(&window_timer);
	}
	return k_msgq_put(&sm_event_q, ev, K_NO_WAIT);
}

enum sm_state sm_get_state(void)
{
	return current_state;
}

uint32_t sm_get_overflow_count(void)
{
	return (uint32_t)atomic_get(&overflow_count);
}

int sm_take_delivered_buffer(struct sm_delivered_window *out, k_timeout_t timeout)
{
	return k_msgq_get(&sm_delivery_q, out, timeout);
}

int sm_wait_idle(k_timeout_t timeout)
{
	return k_sem_take(&sm_event_processed, timeout);
}
