/*
 * Copyright (c) 2019, Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "gpio_utils.h"

#include "gpio_sim3.h"

#define NUMBER_OF_PORTS 4

#define DT_DRV_COMPAT silabs_sim3_gpio_port

struct gpio_sim3_pbstd_common_config {
};

struct gpio_sim3_pbstd_common_data {
	/* A list of all ports */
	const struct device *ports[NUMBER_OF_PORTS];
	size_t count;
};

struct gpio_sim3_pbstd_data {
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* pin callback routine enable flags, by pin number */
	uint32_t pin_callback_enables;
};

static const struct gpio_sim3_pbstd_common_config
	gpio_sim3_pbstd_common_config = {};

static struct gpio_sim3_pbstd_common_data gpio_sim3_pbstd_common_data;

static inline void
gpio_sim3_add_pbstd_port(struct gpio_sim3_pbstd_common_data *data,
			 const struct device *dev)
{
	__ASSERT(dev, "No port device!");
	data->ports[data->count++] = dev;
}

static int gpio_sim3_pbstd_port_set_bits_raw(const struct device *port,
					     uint32_t pins)
{
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_SET = pins;

	return 0;
}

static int gpio_sim3_pbstd_port_clear_bits_raw(const struct device *port,
					       uint32_t pins)
{
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_CLR = pins;

	return 0;
}

static int gpio_sim3_pbstd_port_get_raw(const struct device *port,
					uint32_t *value)
{
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;

	*value = gpio_base->PBPIN_b.PBPIN;

	return 0;
}

static int gpio_sim3_pbstd_port_set_masked_raw(const struct device *port,
					       uint32_t mask, uint32_t value)
{
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_MSK = (mask << 0xF) | value;

	return 0;
}

static int gpio_sim3_pbstd_port_toggle_bits(const struct device *dev,
					    uint32_t pins)
{
	const struct gpio_sim3_config *config = dev->config;
	PBSTD_Type *gpio_base = config->gpio_base;
	uint32_t key;

	key = irq_lock();
	gpio_base->PB = gpio_base->PB ^ pins;
	irq_unlock(key);

	return 0;
}

static int gpio_sim3_pbstd_pin_interrupt_configure(const struct device *port,
						   gpio_pin_t pin,
						   enum gpio_int_mode mode,
						   enum gpio_int_trig trig)
{
	struct gpio_sim3_pbstd_data *data = port->data;
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;

	if (mode == GPIO_INT_DISABLE) {
		data->pin_callback_enables &= ~BIT(pin);
		gpio_base->PMEN_CLR = (1U << pin);
	} else {
		data->pin_callback_enables |= BIT(pin);
		/* Enable pmatch for this pin */
		gpio_base->PMEN_SET = (1U << pin);
	}

	return 0;
}

static inline int gpio_sim3_pbstd_configure(const struct device *port,
					    gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_sim3_config *config = port->config;
	PBSTD_Type *gpio_base = config->gpio_base;
	const int common = gpio_sim3_pin_configure(port, pin, flags);

	if (common < 0) {
		return common;
	}

	if (flags & GPIO_INPUT) {
		/* Set the pins masked with 1's to open-drain input. */
		gpio_base->PBOUTMD_CLR = (1U << pin);
		gpio_base->PB_SET = (1U << pin);
		gpio_base->PBMDSEL_SET = (1U << pin);
		if (flags & GPIO_PULL_UP) {
			/* Only available for complete port */
			gpio_base->PBDRV_b.PBPUEN = 1;
		} else if (flags & GPIO_PULL_DOWN) {
			/* Pull down not available by hardware */
			return -ENOTSUP;
		}

		/* Only available for complete port */
		gpio_base->PBDRV_b.PBPUEN = 0;
	} else { /* GPIO_DIR_OUT */
		gpio_base->PB_CLR = (1U << pin); /* Set to 0 */
		gpio_base->PBOUTMD_SET = (1U << pin); /* push-pull */
		gpio_base->PBMDSEL_SET = (1U << pin); /* digital mode */
	}

	if (flags & GPIO_INT_ENABLE) {
		if (flags & GPIO_ACTIVE_HIGH) {
			gpio_base->PM_SET = (1U << pin);
		} else {
			gpio_base->PM_CLR = (1U << pin);
		}
	}

	return 0;
}

static int gpio_sim3_pbstd_manage_callback(const struct device *dev,
					   struct gpio_callback *callback,
					   bool set)
{
	struct gpio_sim3_pbstd_data *data = dev->data;

	gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}

/**
 * Handler for pin interrupts
 */
static void gpio_sim3_pbstd_isr(const struct device *dev)
{
	struct gpio_sim3_pbstd_common_data *data = dev->data;
	uint32_t enabled_int, int_status;
	const struct device *port_dev;
	struct gpio_sim3_pbstd_data *port_data;
	const struct gpio_sim3_config *config;
	PBSTD_Type *gpio_base;

	for (unsigned int i = 0; i < data->count; i++) {
		port_dev = data->ports[i];
		port_data = port_dev->data;
		config = port_dev->config;
		gpio_base = config->gpio_base;
		int_status = ~(gpio_base->PM ^ gpio_base->PBPIN);
		enabled_int = int_status & port_data->pin_callback_enables;
		int_status &= ~enabled_int;

		gpio_fire_callbacks(&port_data->callbacks, port_dev,
				    enabled_int);
		/* need to change pmatch? */
	}
	/* need to clear pending int? */
}
static const struct gpio_driver_api gpio_sim3_pbstd_driver_api = {
	.pin_configure = gpio_sim3_pbstd_configure,
	.port_get_raw = gpio_sim3_pbstd_port_get_raw,
	.port_set_masked_raw = gpio_sim3_pbstd_port_set_masked_raw,
	.port_set_bits_raw = gpio_sim3_pbstd_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sim3_pbstd_port_clear_bits_raw,
	.port_toggle_bits = gpio_sim3_pbstd_port_toggle_bits,
	.pin_interrupt_configure = gpio_sim3_pbstd_pin_interrupt_configure,
	.manage_callback = gpio_sim3_pbstd_manage_callback,
	.get_pending_int = NULL,
};

static const struct gpio_driver_api gpio_sim3_pbstd_common_driver_api = {
	.manage_callback = gpio_sim3_pbstd_manage_callback,
};

static int gpio_sim3_pbstd_init(const struct device *dev);

DEVICE_DT_DEFINE(DT_INST(0, silabs_sim3_gpio), gpio_sim3_pbstd_init,
		 device_pm_control_nop, &gpio_sim3_pbstd_common_data,
		 &gpio_sim3_pbstd_common_config, POST_KERNEL,
		 CONFIG_GPIO_SIM3_PBSTD_COMMON_INIT_PRIORITY,
		 &gpio_sim3_pbstd_common_driver_api);

static int gpio_sim3_pbstd_init(const struct device *dev)
{
	gpio_sim3_pbstd_common_data.count = 0;
	IRQ_CONNECT(PMATCH0_IRQn,
		    DT_IRQ_BY_NAME(DT_INST(0, silabs_sim3_gpio), pmatch0,
				   priority),
		    gpio_sim3_pbstd_isr,
		    DEVICE_DT_GET(DT_INST(0, silabs_sim3_gpio)), 0);

	irq_enable(PMATCH0_IRQn);
	return 0;
}

#ifdef CONFIG_GPIO_SIM3_PORT0
static int gpio_sim3_port0_init(const struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port0_config = {
	.gpio_base = (void *)DT_INST_REG_ADDR(0),
};

static struct gpio_sim3_pbstd_data gpio_sim3_port0_data;

DEVICE_DT_INST_DEFINE(0, gpio_sim3_port0_init, device_pm_control_nop,
		      &gpio_sim3_port0_data, &gpio_sim3_port0_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		      &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port0_init(const struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT0 */

#ifdef CONFIG_GPIO_SIM3_PORT1
static int gpio_sim3_port1_init(const struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port1_config = {
	.gpio_base = (void *)DT_INST_REG_ADDR(1),
};

static struct gpio_sim3_pbstd_data gpio_sim3_port1_data;

DEVICE_DT_INST_DEFINE(1, gpio_sim3_port1_init, device_pm_control_nop,
		      &gpio_sim3_port1_data, &gpio_sim3_port1_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		      &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port1_init(const struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT1 */

#ifdef CONFIG_GPIO_SIM3_PORT2
static int gpio_sim3_port2_init(const struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port2_config = {
	.gpio_base = (void *)DT_INST_REG_ADDR(2),
};

static struct gpio_sim3_pbstd_data gpio_sim3_port2_data;

DEVICE_DT_INST_DEFINE(2, gpio_sim3_port2_init, device_pm_control_nop,
		      &gpio_sim3_port2_data, &gpio_sim3_port2_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		      &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port2_init(const struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT2 */

#ifdef CONFIG_GPIO_SIM3_PORT3
static int gpio_sim3_port3_init(const struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port3_config = {
	.gpio_base = (void *)DT_INST_REG_ADDR(3),
};

static struct gpio_sim3_pbstd_data gpio_sim3_port3_data;

DEVICE_DT_INST_DEFINE(3, gpio_sim3_port3_init, device_pm_control_nop,
		      &gpio_sim3_port3_data, &gpio_sim3_port3_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		      &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port3_init(const struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT3 */
