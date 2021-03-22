/*
 * Copyright (c) 2019, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "gpio_sim3.h"

int gpio_sim3_pin_configure(const struct device *port, gpio_pin_t pin,
			    gpio_flags_t flags)
{
	/* Check for an invalid pin configuration */
	if ((flags & GPIO_INT_ENABLE) && (flags & GPIO_OUTPUT)) {
		return -EINVAL;
	}

	/* Interrupt on edge is not supported by the hardware */
	if ((flags & GPIO_INT_ENABLE) && (flags & GPIO_INT_EDGE)) {
		return -ENOTSUP;
	}

	/* Drive strengths flags are not implemented */
	if ((flags & GPIO_DS_LOW_MASK) || (flags & GPIO_DS_HIGH_MASK)) {
		return -ENOTSUP;
	}

	/* Enable APB clock to the PPL0 registers. */
	CLKCTRL0->APBCLKG0_b.PLL0CEN = 1;

	return 0;
}
