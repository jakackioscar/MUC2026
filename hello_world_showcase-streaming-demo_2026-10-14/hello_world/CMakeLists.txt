# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

# ADC->BLE/RTT pipeline, ported from nrf5340_firmware (that project's
# state-machine/inference-only files are deliberately not carried over).
target_sources(app PRIVATE
	src/main.c
	src/ble_sender.c
	src/rtt_sender.c
	src/sm_core.c
	src/sampling_source.c
	src/sampling_source_nrfx.c
	src/sm_ble_bridge.c
)
