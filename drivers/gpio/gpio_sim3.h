/*
 * Copyright (c) 2019 Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_

#include <stddef.h>
#include <drivers/gpio.h>
#include <soc.h>

/**
 * @file header for SiM3 GPIO
 */

struct gpio_sim3_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	void *gpio_base;
	uint8_t gpio_index;
};

struct gpio_sim3_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
}

int gpio_sim3_pin_configure(const struct device *port, gpio_pin_t pin,
			    gpio_flags_t flags);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_SIM3_H_ */
