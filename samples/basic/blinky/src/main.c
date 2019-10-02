/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME	1000

void main(void)
{
	u32_t cnt = 0;
	struct device *led0;
	struct device *led1;
	struct device *led2;

	led0 = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	led1 = device_get_binding(DT_ALIAS_LED1_GPIOS_CONTROLLER);
	led2 = device_get_binding(DT_ALIAS_LED2_GPIOS_CONTROLLER);

	/* Set LED pin as output */
	gpio_pin_configure(led0, DT_ALIAS_LED0_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(led1, DT_ALIAS_LED1_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(led2, DT_ALIAS_LED2_GPIOS_PIN, GPIO_DIR_OUT);

	while (1) {
		/* Set pin to HIGH/LOW every 1 second */
		gpio_pin_write(led0, DT_ALIAS_LED0_GPIOS_PIN, cnt % 2);
		gpio_pin_write(led1, DT_ALIAS_LED1_GPIOS_PIN, cnt % 2);
		gpio_pin_write(led2, DT_ALIAS_LED2_GPIOS_PIN, cnt % 2);
		cnt++;
		k_sleep(SLEEP_TIME);
	}
}
