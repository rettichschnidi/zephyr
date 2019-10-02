/*
 * Copyright (c) 2019 Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include "board.h"
#include <gpio.h>
#include <misc/printk.h>

static int GARDENA_SGRMT1_init(struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(GARDENA_SGRMT1_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
