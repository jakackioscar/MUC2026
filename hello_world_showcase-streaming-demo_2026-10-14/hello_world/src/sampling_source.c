/* SPDX-License-Identifier: Apache-2.0
 *
 * Shared plumbing for the sampling_source interface: the per-window-type
 * rate/policy table and dispatch through whichever backend is registered.
 * See sampling_source.h and sampling_source_nrfx.c.
 */
#include "sampling_source.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sampling_source, LOG_LEVEL_INF);

/* SM_DELIVERY_RAW_CAPACITY (sampling_source.h) must cover a whole
 * ACTIVE_CAPTURE chunk or sm_core.c's sm_deliver_buffer() silently goes
 * back to truncating windows to a preview — this is the whole point of
 * having raised it, so catch a regression (e.g. someone bumping
 * SM_ACTIVITY_CHUNK_SAMPLES past 100 without also raising the capacity)
 * at build time instead of losing data silently again. */
BUILD_ASSERT(SM_DELIVERY_RAW_CAPACITY >= CONFIG_SM_ACTIVITY_CHUNK_SAMPLES,
	     "SM_DELIVERY_RAW_CAPACITY must cover a full ACTIVE_CAPTURE chunk");

/* HW-only, do not treat as validated — see sampling_source.h and
 * Kconfig help text for each symbol. */
const struct sm_window_profile sm_window_profiles[3] = {
	[WIN_HR]       = { .rate_hz = CONFIG_SM_SAMPLE_RATE_HR_HZ,
			    .buf_policy = BUF_SINGLE_BIG },
	[WIN_BP]       = { .rate_hz = CONFIG_SM_SAMPLE_RATE_BP_HZ,
			    .buf_policy = BUF_SINGLE_BIG },
	[WIN_ACTIVITY] = { .rate_hz = CONFIG_SM_SAMPLE_RATE_ACTIVITY_HZ,
			    .buf_policy = BUF_PING_PONG },
};

static const struct sampling_source_api *active_api;

void sampling_source_register(const struct sampling_source_api *api)
{
	active_api = api;
}

int sampling_source_start(const struct sampling_config *cfg)
{
	if (!active_api) {
		LOG_ERR("no sampling_source backend registered");
		return -ENODEV;
	}
	return active_api->start(cfg);
}

void sampling_source_stop(void)
{
	if (active_api) {
		active_api->stop();
	}
}

struct sm_buffer *sampling_source_get_buffer(enum sm_buffer_id buf_id)
{
	if (!active_api) {
		return NULL;
	}
	return active_api->get_buffer(buf_id);
}

struct sm_buffer *sampling_source_get_trailing_buffer(void)
{
	if (!active_api || !active_api->get_trailing_buffer) {
		return NULL;
	}
	return active_api->get_trailing_buffer();
}
