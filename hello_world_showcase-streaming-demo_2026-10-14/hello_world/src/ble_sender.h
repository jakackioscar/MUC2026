/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BLE_SENDER_H_
#define BLE_SENDER_H_

#include <stdint.h>

#include "sampling_source.h"

/* Initialise BLE stack, NUS service, and start advertising.
 * Returns 0 on success, negative errno on failure. */
int ble_sender_init(void);

/* Queues one delivered window for real-time transmission. Non-blocking
 * (K_NO_WAIT). If the internal queue is full, drops the oldest queued
 * entry, counts an overflow, and logs a warning — same backpressure
 * policy as sm_core.c's own delivery queue. Never blocks the caller.
 * Always returns 0 (queueing always eventually succeeds by dropping
 * something); kept as int for API-evolution headroom. */
int ble_sender_queue_sm_window(const struct sm_delivered_window *win);

/* Cumulative count of windows dropped because the internal TX queue was
 * full (BLE side of any drop investigation — see sm_core.h's
 * sm_get_overflow_count() for the other half, windows dropped before
 * ever reaching this queue). Never resets on its own. */
uint32_t ble_sender_get_overflow_count(void);

#endif /* BLE_SENDER_H_ */
