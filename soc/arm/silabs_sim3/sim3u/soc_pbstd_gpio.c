/*
 * Copyright (c) 2019 Reto Schneider
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Silabs SiM3U MCU family General Purpose Input Output (GPIO) driver.
 */

#include "soc_pbstd_gpio.h"

void soc_pbstd_gpio_configure(const struct soc_pbstd_gpio_pin *pin)
{
	switch (pin->mode) {
	case SOC_PBSTD_GPIO_PIN_MODE_DIGITAL_PUSH_PULL_OUTPUT:
		pin->port->PB_CLR = (1U << pin->pin);
		pin->port->PBOUTMD_SET = (1U << pin->pin);
		break;
	case SOC_PBSTD_GPIO_PIN_MODE_DIGITAL_INPUT:
		pin->port->PBOUTMD_CLR = (1U << pin->pin);
		pin->port->PB_SET = (1U << pin->pin);
		break;
	}
	/* For GPIOs, all modes are digital. */
	pin->port->PBMDSEL_SET = (1U << pin->pin);
}
