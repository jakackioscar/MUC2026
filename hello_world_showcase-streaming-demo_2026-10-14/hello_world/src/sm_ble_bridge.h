/* SPDX-License-Identifier: Apache-2.0
 *
 * Bridges sm_core.c's delivered windows to rtt_sender.c and ble_sender.c,
 * both live. See sm_ble_bridge.c.
 */
#ifndef SM_BLE_BRIDGE_H_
#define SM_BLE_BRIDGE_H_

/* K_THREAD_DEFINE already auto-starts the bridge thread at boot; this
 * just logs a "ready" line so main.c has an explicit, symmetrical call
 * site alongside the other *_init() calls. */
void sm_ble_bridge_init(void);

#endif /* SM_BLE_BRIDGE_H_ */
