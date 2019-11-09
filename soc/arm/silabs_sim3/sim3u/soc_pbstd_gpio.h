/*
 * Copyright (c) 2019 Reto Schneider
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Silabs SiM3U MCU family General Purpose Input Output (GPIO) driver.
 */

#ifndef _SILABS_SIM3U_SOC_PBSTD_GPIO_H_
#define _SILABS_SIM3U_SOC_PBSTD_GPIO_H_

#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

enum soc_pbstd_gpio_pin_mode {
	SOC_PBSTD_GPIO_PIN_MODE_DIGITAL_PUSH_PULL_OUTPUT,
	SOC_PBSTD_GPIO_PIN_MODE_DIGITAL_INPUT,
};

struct soc_pbstd_gpio_pin {
	PBSTD_Type *port; /** GPIO port */
	unsigned int pin; /** GPIO pin on the port */
	enum soc_pbstd_gpio_pin_mode
		mode; /** mode of the pin, e.g. Digital Push-Pull Output */
};

/**
 * @brief Configure GPIO pin
 * @param[in] pin configuration data
 */
void soc_pbstd_gpio_configure(const struct soc_pbstd_gpio_pin *pin);

#ifdef __cplusplus
}
#endif

#endif /* _SILABS_SIM3U_SOC_PBSTD_GPIO_H_ */
