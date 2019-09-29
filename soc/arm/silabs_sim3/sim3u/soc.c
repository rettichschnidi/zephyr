/*
 * Copyright (c) 2019, Christian Taedcke
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

uint32_t SystemCoreClock = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

/**
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */
static ALWAYS_INLINE void clkInit(void)
{
	/* Wait until oscillator is no longer busy. */
	while (CLKCTRL0->CONTROL_b.OBUSYF) {
	}

//	// The following line bricked my dev-board!
//	CLKCTRL0->CONTROL_b.AHBSEL = CLKCTRL0_CONTROL_AHBSEL_100;
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

	unsigned int oldLevel; /* Old interrupt lock level */

	/* Disable interrupts */
	oldLevel = irq_lock();

	/* Disable watchdog reset source */
	RSTSRC0->RESETEN_b.WDTREN = 0;

	z_clearfaults();

	/* Initialize system clock */
	clkInit();

	/*
	 * Install default handler that simply resets the CPU if configured in
	 * the kernel, NOP otherwise.
	 */
	NMI_INIT();

	/* Restore interrupt state */
	irq_unlock(oldLevel);
	return 0;
}

SYS_INIT(silabs_sim3u_init, PRE_KERNEL_1, 0);
