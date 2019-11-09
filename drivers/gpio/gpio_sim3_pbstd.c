/*
 * Copyright (c) 2019, Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <gpio.h>
#include <soc.h>

#include "gpio_utils.h"

#include "gpio_sim3.h"

#define NUMBER_OF_PORTS 4

struct gpio_sim3_pbstd_common_config {
};

struct gpio_sim3_pbstd_common_data {
	/* A list of all ports */
	struct device *ports[NUMBER_OF_PORTS];
	size_t count;
};

struct gpio_sim3_pbstd_data {
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* pin callback routine enable flags, by pin number */
	u32_t pin_callback_enables;
};

static const struct gpio_sim3_pbstd_common_config
	gpio_sim3_pbstd_common_config = {};

static struct gpio_sim3_pbstd_common_data gpio_sim3_pbstd_common_data;

static inline void
gpio_sim3_add_pbstd_port(struct gpio_sim3_pbstd_common_data *data,
			 struct device *dev)
{
	__ASSERT(dev, "No port device!");
	data->ports[data->count++] = dev;
}

static int gpio_sim3_pbstd_write(struct device *dev, int access_op, u32_t pin,
				 u32_t value)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBSTD_Type *gpio_base = config->gpio_base;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		if (value) {
			/* Set the data output for the corresponding pin.
			 * Writing zeros to the other bits leaves the data
			 * output unchanged for the other pins.
			 */
			gpio_base->PB_SET = BIT(pin);
		} else {
			/* Clear the data output for the corresponding pin.
			 * Writing zeros to the other bits leaves the data
			 * output unchanged for the other pins.
			 */
			gpio_base->PB_CLR = BIT(pin);
		}
	} else { /* GPIO_ACCESS_BY_PORT */
		/* Write the data output for all the pins */
		gpio_base->PB_b.PB = value;
	}

	return 0;
}

static int gpio_sim3_pbstd_read(struct device *dev, int access_op, u32_t pin,
				u32_t *value)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBSTD_Type *gpio_base = config->gpio_base;

	*value = gpio_base->PBPIN_b.PBPIN;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		*value = (*value & BIT(pin)) >> pin;
	}

	/* nothing more to do for GPIO_ACCESS_BY_PORT */

	return 0;
}

static inline int gpio_sim3_pbstd_configure(struct device *dev, int access_op,
					    u32_t pin, int flags)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBSTD_Type *gpio_base = config->gpio_base;
	const int common = gpio_sim3_configure(dev, access_op, pin, flags);

	if (common < 0) {
		return common;
	}

	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
		/* Set the pins masked with 1's to open-drain input. */
		gpio_base->PBOUTMD_CLR = (1U << pin);
		gpio_base->PB_SET = (1U << pin);
		gpio_base->PBMDSEL_SET = (1U << pin);
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			/* Only available for complete port */
			gpio_base->PBDRV_b.PBPUEN = 1;
		} else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
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

	if (flags & GPIO_INT) {
		if (flags & GPIO_INT_ACTIVE_HIGH) {
			gpio_base->PM_SET = (1U << pin);
		} else {
			gpio_base->PM_CLR = (1U << pin);
		}
	}

	return 0;
}

static int gpio_sim3_pbstd_manage_callback(struct device *dev,
					   struct gpio_callback *callback,
					   bool set)
{
	struct gpio_sim3_pbstd_data *data = dev->driver_data;

	gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}

static int gpio_sim3_pbstd_enable_callback(struct device *dev, int access_op,
					   u32_t pin)
{
	struct gpio_sim3_pbstd_data *data = dev->driver_data;
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBSTD_Type *gpio_base = config->gpio_base;

	if (access_op == GPIO_ACCESS_BY_PORT) {
		return -ENOTSUP;
	}

	data->pin_callback_enables |= BIT(pin);
	gpio_base->PMEN_SET = (1U << pin); /* Enable pmatch for this pin */

	return 0;
}

static int gpio_sim3_pbstd_disable_callback(struct device *dev, int access_op,
					    u32_t pin)
{
	struct gpio_sim3_pbstd_data *data = dev->driver_data;
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBSTD_Type *gpio_base = config->gpio_base;

	if (access_op == GPIO_ACCESS_BY_PORT) {
		return -ENOTSUP;
	}

	data->pin_callback_enables &= ~BIT(pin);
	gpio_base->PMEN_CLR = (1U << pin);

	return 0;
}

/**
 * Handler for pin interrupts
 */
static void gpio_sim3_pbstd_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct gpio_sim3_pbstd_common_data *data = dev->driver_data;
	u32_t enabled_int, int_status;
	struct device *port_dev;
	struct gpio_sim3_pbstd_data *port_data;
	const struct gpio_sim3_config *config;
	PBSTD_Type *gpio_base;

	for (unsigned int i = 0; i < data->count; i++) {
		port_dev = data->ports[i];
		port_data = port_dev->driver_data;
		config = port_dev->config->config_info;
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
	.config = gpio_sim3_pbstd_configure,
	.write = gpio_sim3_pbstd_write,
	.read = gpio_sim3_pbstd_read,
	.manage_callback = gpio_sim3_pbstd_manage_callback,
	.enable_callback = gpio_sim3_pbstd_enable_callback,
	.disable_callback = gpio_sim3_pbstd_disable_callback,
};

static const struct gpio_driver_api gpio_sim3_pbstd_common_driver_api = {
	.manage_callback = gpio_sim3_pbstd_manage_callback,
	.enable_callback = gpio_sim3_pbstd_enable_callback,
	.disable_callback = gpio_sim3_pbstd_disable_callback,
};

static int gpio_sim3_pbstd_init(struct device *dev);

DEVICE_AND_API_INIT(gpio_sim3_pbstd_common, DT_GPIO_SIM3_PBSTD_COMMON_NAME,
		    gpio_sim3_pbstd_init, &gpio_sim3_pbstd_common_data,
		    &gpio_sim3_pbstd_common_config, POST_KERNEL,
		    CONFIG_GPIO_SIM3_PBSTD_COMMON_INIT_PRIORITY,
		    &gpio_sim3_pbstd_common_driver_api);

static int gpio_sim3_pbstd_init(struct device *dev)
{
	gpio_sim3_pbstd_common_data.count = 0;

	IRQ_CONNECT(PMATCH0_IRQn,
		    DT_INST_0_SILABS_SIM3_GPIO_IRQ_PMATCH0_PRIORITY,
		    gpio_sim3_pbstd_isr, DEVICE_GET(gpio_sim3_pbstd_common), 0);

	irq_enable(PMATCH0_IRQn);
	return 0;
}

#ifdef CONFIG_GPIO_SIM3_PORT0
static int gpio_sim3_port0_init(struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port0_config = {
	.gpio_base = (void *)DT_INST_0_SILABS_SIM3_GPIO_PORT_BASE_ADDRESS,
};

static struct gpio_sim3_pbstd_data gpio_sim3_port0_data;

DEVICE_AND_API_INIT(gpio_sim3_port0, DT_INST_0_SILABS_SIM3_GPIO_PORT_LABEL,
		    gpio_sim3_port0_init, &gpio_sim3_port0_data,
		    &gpio_sim3_port0_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port0_init(struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT0 */

#ifdef CONFIG_GPIO_SIM3_PORT1
static int gpio_sim3_port1_init(struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port1_config = {
	.gpio_base = (void *)DT_INST_1_SILABS_SIM3_GPIO_PORT_BASE_ADDRESS,
};

static struct gpio_sim3_pbstd_data gpio_sim3_port1_data;

DEVICE_AND_API_INIT(gpio_sim3_port1, DT_INST_1_SILABS_SIM3_GPIO_PORT_LABEL,
		    gpio_sim3_port1_init, &gpio_sim3_port1_data,
		    &gpio_sim3_port1_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port1_init(struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT1 */

#ifdef CONFIG_GPIO_SIM3_PORT2
static int gpio_sim3_port2_init(struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port2_config = {
	.gpio_base = (void *)DT_INST_2_SILABS_SIM3_GPIO_PORT_BASE_ADDRESS,
};

static struct gpio_sim3_pbstd_data gpio_sim3_port2_data;

DEVICE_AND_API_INIT(gpio_sim3_port2, DT_INST_2_SILABS_SIM3_GPIO_PORT_LABEL,
		    gpio_sim3_port2_init, &gpio_sim3_port2_data,
		    &gpio_sim3_port2_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port2_init(struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT2 */

#ifdef CONFIG_GPIO_SIM3_PORT3
static int gpio_sim3_port3_init(struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port3_config = {
	.gpio_base = (void *)DT_INST_3_SILABS_SIM3_GPIO_PORT_BASE_ADDRESS,
};

static struct gpio_sim3_pbstd_data gpio_sim3_port3_data;

DEVICE_AND_API_INIT(gpio_sim3_port3, DT_INST_3_SILABS_SIM3_GPIO_PORT_LABEL,
		    gpio_sim3_port3_init, &gpio_sim3_port3_data,
		    &gpio_sim3_port3_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_sim3_pbstd_driver_api);

static int gpio_sim3_port3_init(struct device *dev)
{
	gpio_sim3_add_pbstd_port(&gpio_sim3_pbstd_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_SIM3_PORT3 */
