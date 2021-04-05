/*
 * Copyright (c) 2019, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/uart.h>
#include <soc.h>

#define DT_DRV_COMPAT silabs_sim3_uart

struct uart_sim3_config {
	UART_Type *base;
	uint32_t baud_rate;
	// unsigned int loc;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_sim3_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static int uart_sim3_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_sim3_config *config = dev->config;
	uint8_t fifo_count = config->base->FIFOCN_b.RCNT;

	if (fifo_count) {
		*c = config->base->DATA.U8;
		return 0;
	}

	return -1;
}

static void uart_sim3_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_sim3_config *config = dev->config;

	/* Wait for transmitter fifo to be empty. */
	while (config->base->FIFOCN_b.TCNT)
		;

	config->base->DATA.U8 = c;
}

static int uart_sim3_err_check(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;
	uint32_t flags = config->base->CONTROL;
	int err = 0;

	if (flags & UART_CONTROL_ROREI_Msk) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & UART_CONTROL_RPARERI_Msk) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & UART_CONTROL_RFRMERI_Msk) {
		err |= UART_ERROR_FRAMING;
	}

	config->base->CONTROL_CLR =
		UART_CONTROL_RFRMERI_Msk | UART_CONTROL_RPARERI_Msk | UART_CONTROL_ROREI_Msk;

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_sim3_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_sim3_config *config = dev->config;
	uint8_t num_tx = 0U;

	while ((len - num_tx > 0) && (config->base->FIFOCN_b.TCNT == 0)) {
		config->base->DATA.U8 = tx_data[num_tx++];
	}

	return num_tx;
}

static int uart_sim3_fifo_read(const struct device *dev, uint8_t *rx_data, const int len)
{
	const struct uart_sim3_config *config = dev->config;
	uint8_t num_rx = 0U;

	while ((len - num_rx > 0) && (num_rx < config->base->FIFOCN_b.RCNT)) {
		rx_data[num_rx++] = config->base->DATA.U8;
	}

	return num_rx;
}

static void uart_sim3_irq_tx_enable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	/* Enable the transmit complete interrupt */
	config->base->CONTROL_SET = UART_CONTROL_TCPTIEN_Msk | UART_CONTROL_TDREQIEN_Msk;
}

static void uart_sim3_irq_tx_disable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	config->base->CONTROL_CLR = UART_CONTROL_TCPTIEN_Msk | UART_CONTROL_TDREQIEN_Msk;
}

static int uart_sim3_irq_tx_complete(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;
	uint32_t flags = config->base->CONTROL;

	config->base->CONTROL_CLR = UART_CONTROL_TCPTI_Msk;

	return (flags & UART_CONTROL_TCPTI_Msk) != 0;
}

static int uart_sim3_irq_tx_ready(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;
	uint32_t flags = config->base->CONTROL;

	config->base->CONTROL_CLR = UART_CONTROL_TDREQI_Msk;

	return (flags & UART_CONTROL_TDREQI_Msk) != 0;
}

static void uart_sim3_irq_rx_enable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	config->base->CONTROL_SET = UART_CONTROL_RDREQIEN_Msk;
}

static void uart_sim3_irq_rx_disable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	config->base->CONTROL_CLR = UART_CONTROL_RDREQIEN_Msk;
}

static int uart_sim3_irq_rx_full(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;
	int flag = config->base->CONTROL_b.RDREQI;

	config->base->CONTROL_CLR = UART_CONTROL_RDREQI_Msk;

	return flag;
}

static int uart_sim3_irq_rx_ready(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	return config->base->CONTROL_b.RDREQIEN && uart_sim3_irq_rx_full(dev);
}

static void uart_sim3_irq_err_enable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	config->base->CONTROL_SET = UART_CONTROL_RERIEN_Msk;
}

static void uart_sim3_irq_err_disable(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;

	config->base->CONTROL_CLR = UART_CONTROL_RERIEN_Msk;
}

static int uart_sim3_irq_is_pending(const struct device *dev)
{
	return uart_sim3_irq_tx_ready(dev) || uart_sim3_irq_rx_ready(dev);
}

static int uart_sim3_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_sim3_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_sim3_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_sim3_isr(struct device *dev)
{
	struct uart_sim3_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define N (2)
#define CALC_BAUDRATE(baudrate)                                                                    \
	(uint32_t)((uint32_t)(SystemCoreClock / (N * (uint32_t)baudrate)) - 1)

#if 0
#define CALC_BAUDRATE(baudrate) (uint32_t)((uint32_t)(20000000 / (N * (uint32_t)baudrate)) - 1)
#endif

static int uart_sim3_init(const struct device *dev)
{
	const struct uart_sim3_config *config = dev->config;
	const uint16_t baud = CALC_BAUDRATE(config->baud_rate);

	/*
	 * The peripheral and gpio clocks are already enabled from soc and gpio
	 * driver.
	 */

	/* Enable UART clock */
	switch ((uintptr_t)config->base) {
	case DT_INST_REG_ADDR(0):
		CLKCTRL0->APBCLKG0_b.UART0CEN = CLKCTRL0_APBCLKG0_UART0CEN_Enable;
		PBCFG0->XBAR0H_SET = PBCFG_XBAR0H_UART0EN_Msk;
		break;
	case DT_INST_REG_ADDR(1):
		CLKCTRL0->APBCLKG0_b.UART1CEN = CLKCTRL0_APBCLKG0_UART1CEN_Enable;
		PBCFG0->XBAR0H_SET = PBCFG_XBAR0H_UART1EN_Msk;
		break;
	}

	config->base->BAUDRATE_b.TBAUD = baud;
	config->base->BAUDRATE_b.RBAUD = baud;

	/* 8n1 is reset value. */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	/* Enable RX and TX */
	config->base->CONTROL_SET = UART_CONTROL_REN_Msk | UART_CONTROL_TEN_Msk;
	return 0;
}

static const struct uart_driver_api uart_sim3_driver_api = {
	.poll_in = uart_sim3_poll_in,
	.poll_out = uart_sim3_poll_out,
	.err_check = uart_sim3_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sim3_fifo_fill,
	.fifo_read = uart_sim3_fifo_read,
	.irq_tx_enable = uart_sim3_irq_tx_enable,
	.irq_tx_disable = uart_sim3_irq_tx_disable,
	.irq_tx_complete = uart_sim3_irq_tx_complete,
	.irq_tx_ready = uart_sim3_irq_tx_ready,
	.irq_rx_enable = uart_sim3_irq_rx_enable,
	.irq_rx_disable = uart_sim3_irq_rx_disable,
	.irq_rx_ready = uart_sim3_irq_rx_ready,
	.irq_err_enable = uart_sim3_irq_err_enable,
	.irq_err_disable = uart_sim3_irq_err_disable,
	.irq_is_pending = uart_sim3_irq_is_pending,
	.irq_update = uart_sim3_irq_update,
	.irq_callback_set = uart_sim3_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_SIM3_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_0(const struct device *dev);
#endif

static const struct uart_sim3_config uart_sim3_0_config = {
	.base = (UART_Type *)DT_INST_REG_ADDR(0),
	// .clock = cmuClock_UART0,
	.baud_rate = DT_INST_PROP(0, current_speed),
//.loc = DT_SILABS_SIM3_UART_UART_0_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_sim3_config_func_0,
#endif
};

static struct uart_sim3_data uart_sim3_0_data;

DEVICE_DT_INST_DEFINE(0, uart_sim3_init, device_pm_control_nop, &uart_sim3_0_data,
		      &uart_sim3_0_config, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_0(const struct device *dev)
{
	IRQ_CONNECT(UART0_IRQn, DT_INST_IRQ(1, priority), uart_sim3_isr, DEVICE_DT_INST_GET(0), 0);

	irq_enable(UART0_IRQn);
}
#endif

#endif /* CONFIG_UART_SIM3_0 */

#ifdef CONFIG_UART_SIM3_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_1(const struct device *dev);
#endif

static const struct uart_sim3_config uart_sim3_1_config = {
	.base = (UART_Type *)DT_INST_REG_ADDR(1),
	//	.clock = cmuClock_UART1,
	.baud_rate = DT_INST_PROP(1, current_speed),
//	.loc = DT_SILABS_SIM3_UART_UART_1_LOCATION,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_sim3_config_func_1,
#endif
};

static struct uart_sim3_data uart_sim3_1_data;

DEVICE_DT_INST_DEFINE(1, uart_sim3_init, device_pm_control_nop, &uart_sim3_1_data,
		      &uart_sim3_1_config, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &uart_sim3_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sim3_config_func_1(const struct device *dev)
{
	IRQ_CONNECT(UART1_IRQn, DT_INST_IRQ(1, priority), uart_sim3_isr, DEVICE_DT_INST_GET(1), 0);

	irq_enable(UART1_IRQn);
}
#endif

#endif /* CONFIG_UART_SIM3_1 */
