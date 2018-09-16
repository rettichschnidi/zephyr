/*
 * Copyright (c) 2017 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <board.h>
#include <misc/printk.h>

static int gardena_smart_radio_module_init(struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(gardena_smart_radio_module_init, PRE_KERNEL_1, 0);
