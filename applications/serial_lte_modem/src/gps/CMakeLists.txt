#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_include_directories(.)
target_sources_ifdef(CONFIG_SLM_GPS app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/slm_at_gps.c)
