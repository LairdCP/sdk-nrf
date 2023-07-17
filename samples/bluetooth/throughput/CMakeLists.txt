#
# Copyright (c) 2018-2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "dts.overlay")

# Generate build ID based on UTC timestamp
string(TIMESTAMP build_time "%s" UTC)
message("build time: ${build_time}")
set(BUILD_TIME ${build_time})
# This configuration file will get automatically merged.
configure_file(${CMAKE_SOURCE_DIR}/build_time.conf.in ${CMAKE_BINARY_DIR}/generated_build_time.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE
	${app_sources}
)
# NORDIC SDK APP END

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
