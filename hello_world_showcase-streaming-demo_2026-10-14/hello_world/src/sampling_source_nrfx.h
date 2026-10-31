/* SPDX-License-Identifier: Apache-2.0
 *
 * Real nrfx SAADC sampling_source backend — see sampling_source_nrfx.c
 * for the architecture and HW-only placeholder values.
 */
#ifndef SAMPLING_SOURCE_NRFX_H_
#define SAMPLING_SOURCE_NRFX_H_

/* Registers this backend as the active sampling_source_api implementation
 * and brings up the SAADC/TIMER/GPPI hardware (one-time: init, channel
 * config, offset calibration). Does not start sampling — that only
 * happens on sampling_source_start(). Real hardware only; do not call
 * this on native_sim (the SAADC/TIMER/GPPI peripherals it drives don't
 * exist there). */
void sampling_source_nrfx_install(void);

#endif /* SAMPLING_SOURCE_NRFX_H_ */
