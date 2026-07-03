/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Reto Schneider
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_gpio_stepper_driver

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_stepper_driver, CONFIG_STEPPER_LOG_LEVEL);

struct gpio_stepper_driver_config {
	struct gpio_dt_spec en_pin;
};

struct gpio_stepper_driver_data {
	enum stepper_micro_step_resolution micro_step_res;
};

static int gpio_stepper_driver_enable(const struct device *dev)
{
	const struct gpio_stepper_driver_config *config = dev->config;

	return gpio_pin_set_dt(&config->en_pin, 1);
}

static int gpio_stepper_driver_disable(const struct device *dev)
{
	const struct gpio_stepper_driver_config *config = dev->config;

	return gpio_pin_set_dt(&config->en_pin, 0);
}

static int gpio_stepper_driver_set_micro_step_res(const struct device *dev,
						  enum stepper_micro_step_resolution micro_step_res)
{
	struct gpio_stepper_driver_data *data = dev->data;

	/* Micro-stepping is realized purely via the coil GPIO pattern of the motion controller;
	 * this driver has no register to write to and only tracks the value for get().
	 */
	data->micro_step_res = micro_step_res;

	return 0;
}

static int gpio_stepper_driver_get_micro_step_res(const struct device *dev,
						   enum stepper_micro_step_resolution *micro_step_res)
{
	const struct gpio_stepper_driver_data *data = dev->data;

	*micro_step_res = data->micro_step_res;

	return 0;
}

static int gpio_stepper_driver_init(const struct device *dev)
{
	const struct gpio_stepper_driver_config *config = dev->config;

	if (!gpio_is_ready_dt(&config->en_pin)) {
		LOG_ERR_DEVICE_NOT_READY(config->en_pin.port);
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
}

static DEVICE_API(stepper, gpio_stepper_driver_api) = {
	.enable = gpio_stepper_driver_enable,
	.disable = gpio_stepper_driver_disable,
	.set_micro_step_res = gpio_stepper_driver_set_micro_step_res,
	.get_micro_step_res = gpio_stepper_driver_get_micro_step_res,
};

#define GPIO_STEPPER_DRIVER_DEFINE(inst)							   \
	static const struct gpio_stepper_driver_config gpio_stepper_driver_config_##inst = {	   \
		.en_pin = GPIO_DT_SPEC_INST_GET(inst, en_gpios),				   \
	};											   \
	static struct gpio_stepper_driver_data gpio_stepper_driver_data_##inst = {		   \
		.micro_step_res = DT_INST_PROP(inst, micro_step_res),				   \
	};											   \
	DEVICE_DT_INST_DEFINE(inst, gpio_stepper_driver_init, NULL,				   \
			      &gpio_stepper_driver_data_##inst,				   \
			      &gpio_stepper_driver_config_##inst, POST_KERNEL,		   \
			      CONFIG_STEPPER_INIT_PRIORITY, &gpio_stepper_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_STEPPER_DRIVER_DEFINE)
