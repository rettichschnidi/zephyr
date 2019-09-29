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

#ifdef CONFIG_SOC_SERIES_SIM3U_PREVENT_BRICKING
	/*
	 * If the reset pin was the source of the last reset, delay startup for
	 * 500 msec.
	 * Firmware can disable the debug port by inadvertantly setting the AHB
	 * clock source to a disabled clock. If this happens too quickly after a
	 * reset, it is not possible for a debug agent to gain control and thus
	 * not possible to reprogram the on-chip flash. Adding a delay here
	 * gives a debug agent sufficient time to connect.
	 */
	if ((RSTSRC0->RESETFLAG_b.PORRF == 0) &&
	    (RSTSRC0->RESETFLAG_b.VMONRF == 0) &&
	    (RSTSRC0->RESETFLAG_b.PINRF == 1)) {
		/*
		 * Set the SysTick timer to count down 10M ticks @ 20MHz
		 * (~500 msec)
		 */
		SysTick->LOAD = 0xA00000;
		SysTick->VAL = 0;
		SysTick->CTRL =
			SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

		/* Wait for the count down to complete */
		while (0 == (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
		}

		/* Set the SysTick timer to reset values */
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;
	}
#endif /* CONFIG_SOC_SERIES_SIM3U_PREVENT_BRICKING */

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
