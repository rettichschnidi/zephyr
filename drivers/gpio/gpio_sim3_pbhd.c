/*
 * Copyright (c) 2019, Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <gpio.h>
#include <soc.h>

#include "gpio_sim3.h"
#include "gpio_utils.h"

static int gpio_sim3_pbhd_write(struct device *dev, int access_op, u32_t pin,
				u32_t value)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBHD_Type *gpio_base = config->gpio_base;

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

static int gpio_sim3_pbhd_read(struct device *dev, int access_op, u32_t pin,
			       u32_t *value)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBHD_Type *gpio_base = config->gpio_base;

	*value = gpio_base->PBPIN_b.PBPIN;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		*value = (*value & BIT(pin)) >> pin;
	}

	/* nothing more to do for GPIO_ACCESS_BY_PORT */

	return 0;
}

static inline int gpio_sim3_pbhd_configure(struct device *dev, int access_op,
					   u32_t pin, int flags)
{
	const struct gpio_sim3_config *config = dev->config->config_info;
	PBHD_Type *gpio_base = config->gpio_base;
	const int common = gpio_sim3_configure(dev, access_op, pin, flags);

	if (common < 0) {
		return common;
	}

	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
		return -ENOTSUP;
	}

	/* Unlock all PBPD pin except for the bank-wide ones */
	PBCFG0->PBKEY = 0xA5;
	PBCFG0->PBKEY = 0xF1;
	/* Unlock the bank-wide pins too */
	gpio_base->PBLOCK = 0x00;

	gpio_base->PBDRV_b.PBBIASEN = 1; /* enable current biasing */
	gpio_base->PBDRV_b.PBLVMD = 1; /* VIOHD is > 3.6V */
	gpio_base->PBDRV_b.PBDRVEN =
		PBHD4_PBDRV_PBDRVEN_Enable; /* enable driver */

	gpio_base->PB_CLR = (1U << pin); /* Set to 0 */
	gpio_base->PBMDSEL_SET = (1U << pin); /* digital mode */

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

	/* output driver enable */
	gpio_base->PBDEN_b.PBNDEN = (1U << pin);
	gpio_base->PBDEN_b.PBPDEN = (1U << pin);

	return 0;
}

static const struct gpio_driver_api gpio_sim3_pbhd_driver_api = {
	.config = gpio_sim3_pbhd_configure,
	.write = gpio_sim3_pbhd_write,
	.read = gpio_sim3_pbhd_read
};

static int gpio_sim3_port4_init(struct device *dev);

static const struct gpio_sim3_config gpio_sim3_port4_config = {
	.gpio_base = (void *)DT_INST_4_SILABS_SIM3_GPIO_PORT_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(gpio_sim3_port4, DT_INST_4_SILABS_SIM3_GPIO_PORT_LABEL,
		    gpio_sim3_port4_init, NULL, &gpio_sim3_port4_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_sim3_pbhd_driver_api);

static int gpio_sim3_port4_init(struct device *dev)
{
	return 0;
}
