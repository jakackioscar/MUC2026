/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADC -> BLE/RTT pipeline test firmware. Wiring layer only, ported/trimmed
 * from nrf5340_firmware's main.c: same ble_sender_init() / sm_core_init()
 * / sm_ble_bridge_init() call sequence, minus that project's random-forest
 * inference self-test and DK-button state-machine event injection.
 * rtt_sender_init() adds a second, wired transport (see rtt_sender.h) —
 * sm_ble_bridge.c feeds both from the same delivered windows.
 *
 * sm_core.c is carried over byte-for-byte from nrf5340_firmware (see its
 * own header) and normally starts cold in DEEP_IDLE, only leaving it on
 * an externally-posted EV_ACTIVITY/EV_PULSE_DUE — usually DK-button
 * injected there. This project doesn't port that button-injection module
 * (sm_event_injection.c), so instead: the one sm_post_event() call below
 * gives it that same kick once at boot to enter ACTIVE_CAPTURE, and
 * Kconfig's SM_SILENCE_TIMEOUT_MS is set large enough (24h) that it
 * never times back out to DEEP_IDLE during a normal session — net
 * effect, continuous ADC sampling streamed live over BLE/RTT for as
 * long as the board is powered, without touching sm_core.c's transition
 * logic at all.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_clock.h>

#include "ble_sender.h"
#include "rtt_sender.h"
#include "sm_core.h"
#include "sm_ble_bridge.h"
#include "sampling_source_nrfx.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Prints which LFCLK source is actually running (nrf_clock_lf_srccopy_get()
 * reads the source latched at LFCLKSTART, not just what's configured).
 * CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL=y (prj.conf) only pins what Zephyr
 * *asks* the clock to start as — this confirms it's actually running on
 * XTAL and not silently falling back to RC. */
static void log_lfclk_source(void)
{
	nrf_clock_lfclk_t src = nrf_clock_lf_srccopy_get(NRF_CLOCK);
	const char *name;

	switch (src) {
	case NRF_CLOCK_LFCLK_XTAL:
		name = "XTAL (LFXO, external 32.768kHz crystal) — expected";
		break;
	case NRF_CLOCK_LFCLK_RC:
		name = "RC (LFRC, internal) — NOT the configured source, check crystal/board";
		break;
	case NRF_CLOCK_LFCLK_SYNTH:
		name = "SYNTH (derived from HFCLK) — NOT the configured source";
		break;
	default:
		name = "UNKNOWN";
		break;
	}
	LOG_INF("LFCLK running source: %s (raw=0x%x)", name, (unsigned int)src);
}

int main(void)
{
	int err;

	log_lfclk_source();

	err = ble_sender_init();
	if (err) {
		LOG_ERR("ble_sender_init: %d", err);
		return err;
	}

	rtt_sender_init();

	sampling_source_nrfx_install();
	sm_core_init();

	/* See file header: the only kick this simplified pipeline needs to
	 * move sm_core.c out of its cold DEEP_IDLE start. */
	sm_post_event(&(struct sm_event){ .type = EV_ACTIVITY });

	sm_ble_bridge_init();

	LOG_INF("ADC->BLE/RTT pipeline up, advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);

	return 0;
}
