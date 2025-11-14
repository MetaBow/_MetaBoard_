#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/bno08x
  ${CMAKE_CURRENT_SOURCE_DIR}/drv2605
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(metabow-firmware)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/battery_monitor.c
  src/adpcm_codec.c
  src/drv2605.c
  src/haptic_cmd.c
)

# NORDIC SDK APP END
