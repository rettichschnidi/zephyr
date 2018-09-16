# Makefile - Precision32 SDK
#
# Copyright (c) 2018, Reto Schneider
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(
  sim3u1xx
  )

zephyr_sources(
  sim3u1xx/system_sim3u1xx.c
  )
