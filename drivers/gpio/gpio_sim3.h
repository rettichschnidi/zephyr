/*
 * Copyright (c) 2019 Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_

#include <stddef.h>
#include <gpio.h>
#include <soc.h>

/**
 * @file header for SiM3 GPIO
 */

struct gpio_sim3_config {
	void *gpio_base;
	u8_t gpio_index;
};

int gpio_sim3_configure(struct device *dev, int access_op, u32_t pin,
			int flags);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_ */
