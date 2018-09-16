/*
 * Copyright (c) 2017, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <uart.h>
#include <em_usart.h>
#include <em_gpio.h>
#include <em_cmu.h>
#include <soc.h>

struct uart_gecko_config {
	USART_TypeDef *base;
	CMU_Clock_TypeDef clock;
	u32_t baud_rate;
	struct soc_gpio_pin pin_rx;
	struct soc_gpio_pin pin_tx;
	unsigned int loc;
};

struct uart_gecko_data {
};

static int uart_gecko_poll_in(struct device *dev, unsigned char *c)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_StatusGet(config->base);

	if (flags & USART_STATUS_RXDATAV) {
		*c = USART_Rx(config->base);
		return 0;
	}

	return -1;
}

static unsigned char uart_gecko_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	USART_Tx(config->base, c);

	return c;
}

static int uart_gecko_err_check(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_IntGet(config->base);
	int err = 0;

	if (flags & USART_IF_RXOF) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & USART_IF_PERR) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & USART_IF_FERR) {
		err |= UART_ERROR_FRAMING;
	}

	USART_IntClear(config->base, USART_IF_RXOF |
		       USART_IF_PERR |
		       USART_IF_FERR);

	return err;
}


static void uart_gecko_init_pins(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	soc_gpio_configure(&config->pin_rx);
	soc_gpio_configure(&config->pin_tx);
#if defined(_USART_ROUTEPEN_MASK) || defined(_UART_ROUTEPEN_MASK)
	config->base->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
	config->base->ROUTELOC0 = (config->loc << _USART_ROUTELOC0_TXLOC_SHIFT) |
		(config->loc << _USART_ROUTELOC0_RXLOC_SHIFT);
	config->base->ROUTELOC1 = _USART_ROUTELOC1_RESETVALUE;
#else
	config->base->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN
		| (config->loc << 8);
#endif
}

static int uart_gecko_init(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;

	/* The peripheral and gpio clock are already enabled from soc and gpio
	 * driver
	 */

	usartInit.baudrate = config->baud_rate;

	/* Enable USART clock */
	CMU_ClockEnable(config->clock, true);

	/* Init USART */
	USART_InitAsync(config->base, &usartInit);

	/* Initialize USART pins */
	uart_gecko_init_pins(dev);

	return 0;
}

static const struct uart_driver_api uart_gecko_driver_api = {
	.poll_in = uart_gecko_poll_in,
	.poll_out = uart_gecko_poll_out,
	.err_check = uart_gecko_err_check,
};

#ifdef CONFIG_UART_GECKO_0

static const struct uart_gecko_config uart_gecko_0_config = {
	.base = UART0,
	.clock = cmuClock_UART0,
	.baud_rate = CONFIG_UART_GECKO_0_BAUD_RATE,
	.pin_rx = PIN_UART0_RXD,
	.pin_tx = PIN_UART0_TXD,
	.loc = CONFIG_UART_GECKO_0_GPIO_LOC,
};

static struct uart_gecko_data uart_gecko_0_data;

DEVICE_AND_API_INIT(uart_0, CONFIG_UART_GECKO_0_NAME,
		    &uart_gecko_init,
		    &uart_gecko_0_data, &uart_gecko_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_gecko_driver_api);

#endif /* CONFIG_UART_GECKO_0 */

#ifdef CONFIG_UART_GECKO_1

static const struct uart_gecko_config uart_gecko_1_config = {
	.base = UART1,
	.clock = cmuClock_UART1,
	.baud_rate = CONFIG_UART_GECKO_1_BAUD_RATE,
	.pin_rx = PIN_UART1_RXD,
	.pin_tx = PIN_UART1_TXD,
	.loc = CONFIG_UART_GECKO_1_GPIO_LOC,
};

static struct uart_gecko_data uart_gecko_1_data;

DEVICE_AND_API_INIT(uart_1, CONFIG_UART_GECKO_1_NAME,
		    &uart_gecko_init,
		    &uart_gecko_1_data, &uart_gecko_1_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_gecko_driver_api);

#endif /* CONFIG_UART_GECKO_1 */

#ifdef CONFIG_USART_GECKO_0

static const struct uart_gecko_config usart_gecko_0_config = {
	.base = USART0,
	.clock = cmuClock_USART0,
	.baud_rate = CONFIG_USART_GECKO_0_BAUD_RATE,
	.pin_rx = PIN_USART0_RXD,
	.pin_tx = PIN_USART0_TXD,
	.loc = CONFIG_USART_GECKO_0_GPIO_LOC,
};

static struct uart_gecko_data usart_gecko_0_data;

DEVICE_AND_API_INIT(usart_0, CONFIG_USART_GECKO_0_NAME,
		    &uart_gecko_init,
		    &usart_gecko_0_data, &usart_gecko_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_gecko_driver_api);

#endif /* CONFIG_USART_GECKO_0 */

#ifdef CONFIG_USART_GECKO_1

static const struct uart_gecko_config usart_gecko_1_config = {
	.base = USART1,
	.clock = cmuClock_USART1,
	.baud_rate = CONFIG_USART_GECKO_1_BAUD_RATE,
	.pin_rx = PIN_USART1_RXD,
	.pin_tx = PIN_USART1_TXD,
	.loc = CONFIG_USART_GECKO_1_GPIO_LOC,
};

static struct uart_gecko_data usart_gecko_1_data;

DEVICE_AND_API_INIT(usart_1, CONFIG_USART_GECKO_1_NAME,
		    &uart_gecko_init,
		    &usart_gecko_1_data, &usart_gecko_1_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_gecko_driver_api);

#endif /* CONFIG_UART_GECKO_1 */
