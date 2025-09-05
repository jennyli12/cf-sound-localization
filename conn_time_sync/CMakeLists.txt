#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(conn_time_sync)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.c src/timed_led_toggle.c src/timed_i2s.c src/buttons_leds.c)

if (CONFIG_BT_CENTRAL)
    target_sources(app PRIVATE src/central.c)
endif()

if (CONFIG_BT_PERIPHERAL)
    target_sources(app PRIVATE src/peripheral.c)
endif()

if (CONFIG_SOC_SERIES_NRF54LX)
    target_sources(app PRIVATE src/controller_time_nrf54.c)
else()
    MESSAGE(FATAL_ERROR "Unsupported series")
endif()
