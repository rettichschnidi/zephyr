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
#include <arch/arm/cortex_m/cmsis.h>

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
static ALWAYS_INLINE void clock_init(void)
{
	/* Wait until oscillator is no longer busy. */
	while (CLKCTRL0->CONTROL_b.OBUSYF) {
	}

#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0) ||                             \
	defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1)
	/* Enable APB clock to the port bank modules. */
	CLKCTRL0->APBCLKG0_b.PB0CEN = 1;
#endif /* SOC_SERIES_SIM3U_CROSSBAR_0 */

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

	/* Initialize system clock */
	clock_init();

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

/**
 * @brief Configure the cross bar(s)
 *
 * @return N/A
 *
 * Excerpt from the reference manual:
 * When configuring the crossbar, all settings should be made to the crossbar
 * and Port Bank registers before enabling the crossbar. This ensures that
 * peripherals will not shift around while each one is being enabled and Port
 * I/O pins will remain stable. The settings in PBOUTMD, PBMDSEL, or PBSKIPEN
 * will not take effect until the crossbars are enabled.
 */
static int silabs_sim3u_crossbar_init(struct device *arg)
{
	ARG_UNUSED(arg);
#ifdef CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0
	const u32_t crossbar_0_pb0 = 0
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_0_SKIP)
				     | (1U << 0)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_1_SKIP)
				     | (1U << 1)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_2_SKIP)
				     | (1U << 2)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_3_SKIP)
				     | (1U << 3)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_4_SKIP)
				     | (1U << 4)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_5_SKIP)
				     | (1U << 5)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_6_SKIP)
				     | (1U << 6)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_7_SKIP)
				     | (1U << 7)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_8_SKIP)
				     | (1U << 8)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_9_SKIP)
				     | (1U << 9)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_10_SKIP)
				     | (1U << 10)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_11_SKIP)
				     | (1U << 11)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_12_SKIP)
				     | (1U << 12)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_13_SKIP)
				     | (1U << 13)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_14_SKIP)
				     | (1U << 14)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB0_15_SKIP)
				     | (1U << 15)
#endif
		;
	PBSTD0->PBSKIPEN = crossbar_0_pb0;

	const u32_t crossbar_0_pb1 = 0
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_0_SKIP)
				     | (1U << 0)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_1_SKIP)
				     | (1U << 1)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_2_SKIP)
				     | (1U << 2)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_3_SKIP)
				     | (1U << 3)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_4_SKIP)
				     | (1U << 4)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_5_SKIP)
				     | (1U << 5)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_6_SKIP)
				     | (1U << 6)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_7_SKIP)
				     | (1U << 7)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_8_SKIP)
				     | (1U << 8)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_9_SKIP)
				     | (1U << 9)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_10_SKIP)
				     | (1U << 10)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_11_SKIP)
				     | (1U << 11)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_12_SKIP)
				     | (1U << 12)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_13_SKIP)
				     | (1U << 13)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_14_SKIP)
				     | (1U << 14)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0_PB1_15_SKIP)
				     | (1U << 15)
#endif
		;
	PBSTD1->PBSKIPEN = crossbar_0_pb1;

	PBCFG0->XBAR0H_b.XBAR0EN = 1;
#endif /* CONFIG_SOC_SERIES_SIM3U_CROSSBAR_0 */

#ifdef CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1
	const u32_t crossbar_1_pb2 = 0
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_0_SKIP)
				     | (1U << 0)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_1_SKIP)
				     | (1U << 1)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_2_SKIP)
				     | (1U << 2)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_3_SKIP)
				     | (1U << 3)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_4_SKIP)
				     | (1U << 4)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_5_SKIP)
				     | (1U << 5)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_6_SKIP)
				     | (1U << 6)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_7_SKIP)
				     | (1U << 7)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_8_SKIP)
				     | (1U << 8)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_9_SKIP)
				     | (1U << 9)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_10_SKIP)
				     | (1U << 10)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_11_SKIP)
				     | (1U << 11)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_12_SKIP)
				     | (1U << 12)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_13_SKIP)
				     | (1U << 13)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_14_SKIP)
				     | (1U << 14)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB2_15_SKIP)
				     | (1U << 15)
#endif
		;
	PBSTD2->PBSKIPEN = crossbar_1_pb2;

	const u32_t crossbar_1_pb3 = 0
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_0_SKIP)
				     | (1U << 0)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_1_SKIP)
				     | (1U << 1)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_2_SKIP)
				     | (1U << 2)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_3_SKIP)
				     | (1U << 3)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_4_SKIP)
				     | (1U << 4)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_5_SKIP)
				     | (1U << 5)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_6_SKIP)
				     | (1U << 6)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_7_SKIP)
				     | (1U << 7)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_8_SKIP)
				     | (1U << 8)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_9_SKIP)
				     | (1U << 9)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_10_SKIP)
				     | (1U << 10)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_11_SKIP)
				     | (1U << 11)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_12_SKIP)
				     | (1U << 12)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_13_SKIP)
				     | (1U << 13)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_14_SKIP)
				     | (1U << 14)
#endif
#if defined(CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1_PB3_15_SKIP)
				     | (1U << 15)
#endif
		;
	PBSTD3->PBSKIPEN = crossbar_1_pb3;

	PBCFG0->XBAR1_b.XBAR1EN = 1;
#endif /* CONFIG_SOC_SERIES_SIM3U_CROSSBAR_1 */

	return 0;
}

SYS_INIT(silabs_sim3u_crossbar_init, PRE_KERNEL_2, 0);
