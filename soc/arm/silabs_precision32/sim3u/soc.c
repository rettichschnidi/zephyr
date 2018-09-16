/*
 * Copyright (c) 2017, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SoC initialization for the SIM3U
 */

#include <kernel.h>
#include <init.h>
#include <soc.h>
#include <arch/cpu.h>
#include <cortex_m/exc.h>

/**
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */
static ALWAYS_INLINE void clkInit(void)
{
}

/**
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */
static int silabs_sim3u_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* Initialize system clock according to CONFIG_CMU settings */
	clkInit();

	return 0;
}

SYS_INIT(silabs_sim3u_init, PRE_KERNEL_1, 0);
