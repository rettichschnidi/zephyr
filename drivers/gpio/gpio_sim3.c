/*
 * Copyright (c) 2019, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <gpio.h>
#include <soc.h>

#include "gpio_sim3.h"
#include "gpio_utils.h"

#define NUMBER_OF_PORTS 5

int gpio_sim3_configure(struct device *dev, int access_op, u32_t pin, int flags)
{
	/* Enable APB clock to the PPL0 registers. */
	CLKCTRL0->APBCLKG0_b.PLL0CEN = 1;

	/* Check for an invalid pin configuration */
	if ((flags & GPIO_INT) && (flags & GPIO_DIR_OUT)) {
		return -EINVAL;
	}

	/* Interrupt on edge is not supported by the hardware */
	if ((flags & GPIO_INT) && (flags & GPIO_INT_EDGE)) {
		return -ENOTSUP;
	}

	/* Setting interrupt flags for a complete port is not implemented */
	if ((flags & GPIO_INT) && (access_op == GPIO_ACCESS_BY_PORT)) {
		return -ENOTSUP;
	}

	/* Drive strengths flags are not implemented */
	if ((flags & GPIO_DS_LOW_MASK) || (flags & GPIO_DS_HIGH_MASK)) {
		return -ENOTSUP;
	}

	/* Configuring a complete port is not implemented */
	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	return 0;
}

