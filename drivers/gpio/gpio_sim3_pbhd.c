/*
 * Copyright (c) 2019, Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "gpio_sim3.h"
#include "gpio_utils.h"

#define DT_DRV_COMPAT silabs_sim3_gpio_port

struct gpio_sim3_pbhd_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
};

static inline int gpio_sim3_pbhd_pin_configure(const struct device *port, gpio_pin_t pin,
					       gpio_flags_t flags)
{
	const struct gpio_sim3_config *config = port->config;
	PBHD_Type *gpio_base = config->gpio_base;
	const int common = gpio_sim3_pin_configure(port, pin, flags);

	if (common < 0) {
		return common;
	}

	if (flags & GPIO_INT_ENABLE) {
		return -ENOTSUP;
	}

	/* Unlock all PBPD pin except for the bank-wide ones */
	PBCFG0->PBKEY = 0xA5;
	PBCFG0->PBKEY = 0xF1;
	/* Unlock the bank-wide pins too */
	gpio_base->PBLOCK = 0x00;

	gpio_base->PBDRV_b.PBBIASEN = 1; /* enable current biasing */
	gpio_base->PBDRV_b.PBLVMD = 1; /* VIOHD is > 3.6V */
	gpio_base->PBDRV_b.PBDRVEN = PBHD4_PBDRV_PBDRVEN_Enable; /* enable driver */

	/* Configure pin as GPIO (reset value) */
	switch (pin) {
	case 0:
		gpio_base->PBFSEL_b.PB0SEL = PBHD4_PBFSEL_PB0SEL_00;
		break;
	case 1:
		gpio_base->PBFSEL_b.PB1SEL = PBHD4_PBFSEL_PB1SEL_00;
		break;
	case 2:
		gpio_base->PBFSEL_b.PB2SEL = PBHD4_PBFSEL_PB2SEL_00;
		break;
	case 3:
		gpio_base->PBFSEL_b.PB3SEL = PBHD4_PBFSEL_PB3SEL_00;
		break;
	case 4:
		gpio_base->PBFSEL_b.PB4SEL = PBHD4_PBFSEL_PB4SEL_00;
		break;
	case 5:
		gpio_base->PBFSEL_b.PB5SEL = PBHD4_PBFSEL_PB5SEL_000;
		break;
	}

	gpio_base->PBMDSEL_SET = BIT(pin); /* digital mode */
	if (flags & GPIO_INPUT) {
		gpio_base->PB_SET = BIT(pin); /* digital input */
	} else { /* GPIO_DIR_OUT */
		gpio_base->PB_CLR = BIT(pin); /* Set to 0 */
		/*  Port bank N-Channel driver enable */
		gpio_base->PBDEN_SET = BIT(pin);
		/*  Port bank P-Channel driver enable */
		gpio_base->PBDEN_SET = BIT(pin) << 16;
	}

	return 0;
}

static int gpio_sim3_pbhd_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_sim3_config *config = port->config;
	PBHD_Type *gpio_base = config->gpio_base;

	*value = gpio_base->PBPIN_b.PBPIN;

	return 0;
}

static int gpio_sim3_pbhd_port_set_masked_raw(const struct device *port, uint32_t mask,
					      uint32_t value)
{
	const struct gpio_sim3_config *config = port->config;
	PBHD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_MSK = (mask << 0xF) | value;

	return 0;
}

static int gpio_sim3_pbhd_port_set_bits_raw(const struct device *port, uint32_t pins)
{
	const struct gpio_sim3_config *config = port->config;
	PBHD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_SET = pins;

	return 0;
}

static int gpio_sim3_pbhd_port_clear_bits_raw(const struct device *port, uint32_t pins)
{
	const struct gpio_sim3_config *config = port->config;
	PBHD_Type *gpio_base = config->gpio_base;

	gpio_base->PB_CLR = pins;

	return 0;
}

static int gpio_sim3_pbhd_port_toggle_bits(const struct device *dev, uint32_t pins)
{
	const struct gpio_sim3_config *config = dev->config;
	PBHD_Type *gpio_base = config->gpio_base;
	uint32_t key;

	key = irq_lock();
	gpio_base->PB = gpio_base->PB ^ pins;
	irq_unlock(key);

	return 0;
}

static const struct gpio_driver_api gpio_sim3_pbhd_driver_api = {
	.pin_configure = gpio_sim3_pbhd_pin_configure,
	.port_get_raw = gpio_sim3_pbhd_port_get_raw,
	.port_set_masked_raw = gpio_sim3_pbhd_port_set_masked_raw,
	.port_set_bits_raw = gpio_sim3_pbhd_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sim3_pbhd_port_clear_bits_raw,
	.port_toggle_bits = gpio_sim3_pbhd_port_toggle_bits,
	.pin_interrupt_configure = NULL,
	.manage_callback = NULL,
	.get_pending_int = NULL,
};

static int gpio_sim3_port4_init(const struct device *dev)
{
	return 0;
}

static const struct gpio_sim3_config gpio_sim3_port4_config = {
	.common = {
		.port_pin_mask = 0x3F,
	},
	.gpio_base = (void *)DT_INST_REG_ADDR(4),
};

static struct gpio_sim3_pbhd_data gpio_sim3_port4_data;

DEVICE_DT_INST_DEFINE(4, gpio_sim3_port4_init, device_pm_control_nop, &gpio_sim3_port4_data,
		      &gpio_sim3_port4_config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &gpio_sim3_pbhd_driver_api);
