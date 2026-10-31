/* SPDX-License-Identifier: Apache-2.0
 *
 * Abstract event types for the sampling power state machine. Every
 * async source (RTC, LPCOMP, SAADC END, window/silence timers)
 * translates its interrupt/expiry into one of these and posts it via
 * sm_post_event() (see sm_core.h) — nothing touches state directly
 * except the state machine's own single-threaded consumer.
 */
#ifndef SM_EVENTS_H_
#define SM_EVENTS_H_

#include <stdint.h>

enum sm_event_type {
	EV_PULSE_DUE,   /* RTC: a pulse-window capture is due */
	EV_ACTIVITY,    /* LPCOMP: activity detected */
	EV_BUF_READY,   /* a sampling buffer has been filled */
	EV_TIMEOUT,     /* a window/silence timer expired */
};

enum sm_timeout_kind {
	SM_TIMEOUT_WINDOW_FULL,  /* PULSE_WINDOW's window duration elapsed */
	SM_TIMEOUT_SILENCE,      /* ACTIVE_CAPTURE's silence timeout elapsed */
};

/* Which physical buffer is ready. SINGLE_BIG only ever uses SM_BUF_0. */
enum sm_buffer_id {
	SM_BUF_0 = 0,
	SM_BUF_1 = 1,
};

/* Which PULSE_WINDOW config an EV_PULSE_DUE requests. The transition table
 * only says "DEEP_IDLE + EV_PULSE_DUE -> PULSE_WINDOW (pulse config)" —
 * it does not say how HR vs BP gets decided (e.g. "every Nth pulse is BP").
 * That policy is deliberately NOT in sm_core.c: whoever generates
 * EV_PULSE_DUE (a real RTC scheduler later, DK-button injection for now)
 * decides and stamps it here. */
enum sm_pulse_kind {
	SM_PULSE_HR,
	SM_PULSE_BP,
};

struct sm_event {
	enum sm_event_type type;
	union {
		enum sm_timeout_kind timeout_kind; /* type == EV_TIMEOUT */
		enum sm_buffer_id buf_id;           /* type == EV_BUF_READY */
		enum sm_pulse_kind pulse_kind;       /* type == EV_PULSE_DUE */
	};
};

#endif /* SM_EVENTS_H_ */
