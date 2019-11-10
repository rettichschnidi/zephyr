/*
 * Copyright (c) 2019, Christian Taedcke
 * Copyright (c) 2019, Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <flash.h>
#include <soc.h>
#include <irq.h>
#include <stdbool.h>

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(flash_sim3);

struct flash_sim3_data {
	struct k_mutex mutex;
	bool locked;
};

#define DEV_NAME(dev) ((dev)->config->name)
#define DEV_DATA(dev) ((struct flash_sim3_data *const)(dev)->driver_data)

enum { FLASH_PAGE_SIZE = 1024 };

static bool write_range_is_valid(off_t offset, u32_t size);
static bool read_range_is_valid(off_t offset, u32_t size);

static void unlock_flash_multiple_op(void)
{
	FLASHCTRL0->KEY = 0xA5;
	FLASHCTRL0->KEY = 0xF2;
}

static void lock_flash(void)
{
	FLASHCTRL0->KEY = 0x5A;
}

static int flash_sim3_read(struct device *dev, off_t offset, void *data,
			   size_t size)
{
	(void)dev;

	if (!read_range_is_valid(offset, size)) {
		return -EINVAL;
	}

	if (!size) {
		return 0;
	}

	memcpy(data, (u8_t *)CONFIG_FLASH_BASE_ADDRESS + offset, size);

	return 0;
}

static int flash_sim3_write(struct device *dev, const off_t offset,
			    const void *const data, const size_t size)
{
	struct flash_sim3_data *const dev_data = DEV_DATA(dev);

	if (!write_range_is_valid(offset, size)) {
		return -EINVAL;
	}

	if (!size) {
		return 0;
	}

	const u8_t *write_base = (u8_t *)CONFIG_FLASH_BASE_ADDRESS + offset;
	const u8_t *source_base = (const u8_t *)data;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	FLASHCTRL0->CONFIG_b.ERASEEN = 0;
	const int key = irq_lock();

	unlock_flash_multiple_op();
	for (off_t write_offset = 0; write_offset < size;
	     write_offset += sizeof(u16_t)) {
		FLASHCTRL0->WRADDR = (uint32_t)(write_base + write_offset);
		FLASHCTRL0->WRDATA =
			*(const u16_t *)(source_base + write_offset);
	}
	lock_flash();
	irq_unlock(key);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

static int flash_sim3_erase(struct device *dev, off_t offset, size_t size)
{
	struct flash_sim3_data *const dev_data = DEV_DATA(dev);
	int irq_key;

	if (!write_range_is_valid(offset, size)) {
		return -EINVAL;
	}

	if ((size % FLASH_PAGE_SIZE) != 0) {
		LOG_ERR("size %x: not multiple of a page size", size);
		return -EINVAL;
	}

	if (!size) {
		return 0;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (dev_data->locked) {
		k_mutex_unlock(&dev_data->mutex);
		return -EACCES;
	}

	/* Generic code for erasing multiple flash pages. Could be simplified
	 * for single pages and would need to be adapted when running from RAM.
	 */
	FLASHCTRL0->CONFIG_b.ERASEEN = 1;
	irq_key = irq_lock();
	unlock_flash_multiple_op();
	for (off_t erase_offset = 0; erase_offset < size;
	     erase_offset += FLASH_PAGE_SIZE) {
		FLASHCTRL0->WRADDR =
			CONFIG_FLASH_BASE_ADDRESS + offset + erase_offset;
		FLASHCTRL0->WRDATA = 42; /* Any value works here. */
	}
	lock_flash();
	irq_unlock(irq_key);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

static int flash_sim3_write_protection(struct device *dev, bool enable)
{
	struct flash_sim3_data *const dev_data = DEV_DATA(dev);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* The SiM3 requires a chain of commands (i.e. enabling erase before
	 * unlocking the flash.
	 * Therefore, all we do here is to allow other function to ensure
	 * whether the write protection has been willingly disabled upfront.
	 */
	dev_data->locked = enable;

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

/* Note:
 * - A flash address to write to must be aligned to half-words.
 * - Number of bytes to write must be divisible by 2.
 */
static bool write_range_is_valid(off_t offset, u32_t size)
{
	return read_range_is_valid(offset, size) &&
	       (offset % sizeof(u16_t) == 0) && (size % 2 == 0);
}

static bool read_range_is_valid(off_t offset, u32_t size)
{
	return (offset + size) <= (CONFIG_FLASH_SIZE * 1024);
}

static int flash_sim3_init(struct device *dev)
{
	struct flash_sim3_data *const dev_data = DEV_DATA(dev);

	k_mutex_init(&dev_data->mutex);

	/* Ensure supply monitor is enabled. */
	VMON0->CONTROL_b.VMONEN = VMON0_CONTROL_VMONEN_Enable;

	/* Ensure supply monitor is configured as reset source. */
	RSTSRC0->RESETFLAG_b.VMONRF = RSTSRC0_RESETEN_VMONREN_Enable;

	/* CLKCTRL0->AHBCLKG_b.FLASHCEN is set per default */

	/* Lock the flash. */
	flash_sim3_write_protection(dev, true);

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

static const struct flash_driver_api flash_sim3_driver_api = {
	.read = flash_sim3_read,
	.write = flash_sim3_write,
	.erase = flash_sim3_erase,
	.write_protection = flash_sim3_write_protection,
	/* FLASH_WRITE_BLOCK_SIZE is extracted from device tree as flash node
	 * property 'write-block-size'.
	 */
	.write_block_size = DT_FLASH_WRITE_BLOCK_SIZE,
};

static struct flash_sim3_data flash_sim3_0_data;

DEVICE_AND_API_INIT(flash_sim3_0, DT_FLASH_DEV_NAME, flash_sim3_init,
		    &flash_sim3_0_data, NULL, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_sim3_driver_api);
