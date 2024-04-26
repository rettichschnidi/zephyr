/*
 * Copyright (c) 2024 GARDENA GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si32_dma

#include <SI32_DMACTRL_A_Type.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_si32, CONFIG_DMA_LOG_LEVEL);

void dma_si32_isr(const struct device *dev)
{
	LOG_ERR("INTERRUPT TRIGGERED");
}

static int dma_si32_init(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dma_si32_isr, DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));

	return -ENOTSUP;
}

int dma_si32_config(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	return -ENOTSUP;
}

int dma_si32_start(const struct device *dev, uint32_t channel)
{
	return -ENOTSUP;
}

int dma_si32_stop(const struct device *dev, uint32_t channel)
{
	return -ENOTSUP;
}

static const struct dma_driver_api dma_si32_driver_api = {
	.config = dma_si32_config,
	.start = dma_si32_start,
	.stop = dma_si32_stop,
};

struct dma_si32_cfg {
	SI32_DMACTRL_A_Type *base;
};

struct dma_si32_dev_data {
	/* now what!? */
};

static const struct dma_si32_cfg dma_si32_cfg = {
	.base = (SI32_DMACTRL_A_Type *)DT_INST_REG_ADDR(0)};

static struct dma_si32_dev_data dma_si32_data = {};

DEVICE_DT_INST_DEFINE(0, &dma_si32_init, NULL, &dma_si32_data, &dma_si32_cfg, POST_KERNEL,
		      CONFIG_DMA_INIT_PRIORITY, &dma_si32_driver_api);
