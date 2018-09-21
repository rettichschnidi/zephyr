/*
 * Copyright (c) 2017, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <uart.h>
#include <soc.h>
#include "SI32_UART_A_Type.h"

struct uart_precision32_config {
	u32_t baud_rate;
	unsigned int loc;
};

struct uart_precision32_data {
};

static int uart_precision32_poll_in(struct device *dev, unsigned char *c)
{
    (void)dev;
    (void)c;

    return 0;
}

static unsigned char uart_precision32_poll_out(struct device *dev, unsigned char c)
{
    (void)dev;
    (void)c;

    return 0;
}

static int uart_precision32_err_check(struct device *dev)
{
    (void)dev;

    return 0;
}

static int uart_precision32_init(struct device *dev)
{
    (void)dev;

	return 0;
}

static const struct uart_driver_api uart_precision32_driver_api = {
	.poll_in = uart_precision32_poll_in,
	.poll_out = uart_precision32_poll_out,
	.err_check = uart_precision32_err_check,
};

#ifdef CONFIG_UART_PRECISION32_0

static const struct uart_precision32_config uart_precision32_0_config = {
	.baud_rate = CONFIG_UART_PRECISION32_0_BAUD_RATE,
	.loc = CONFIG_UART_PRECISION32_0_GPIO_LOC,
};

static struct uart_precision32_data uart_precision32_0_data;

DEVICE_AND_API_INIT(uart_0, CONFIG_UART_PRECISION32_0_NAME,
		    &uart_precision32_init,
		    &uart_precision32_0_data, &uart_precision32_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_precision32_driver_api);

#endif /* CONFIG_UART_PRECISION32_0 */

#ifdef CONFIG_USART_PRECISION32_0

static const struct uart_precision32_config usart_precision32_0_config = {
	.baud_rate = CONFIG_USART_PRECISION32_0_BAUD_RATE,
	.loc = CONFIG_USART_PRECISION32_0_GPIO_LOC,
};

static struct uart_precision32_data usart_precision32_0_data;

DEVICE_AND_API_INIT(usart_0, CONFIG_USART_PRECISION32_0_NAME,
		    &uart_precision32_init,
		    &usart_precision32_0_data, &usart_precision32_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_precision32_driver_api);

#endif /* CONFIG_USART_PRECISION32_0 */

