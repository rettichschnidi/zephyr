/*
 * Copyright (c) 2026 Siemens
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sitronix_st77916

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_st77916, CONFIG_DISPLAY_LOG_LEVEL);

#include "display_st77916.h"

/* Command set lock/unlock sequences */
#define ST77916_CMD2_ENABLE  0x01
#define ST77916_CMD2_UNLOCK  0x01
#define ST77916_GAM_ENABLE   0x02
#define ST77916_PAGE_DISABLE 0x00

struct st77916_config {
	const struct device *mipi_dbi;
	const struct mipi_dbi_config dbi_config;
	uint16_t width;
	uint16_t height;
	bool inverted;
	uint8_t madctl;
	uint8_t colmod;
	uint8_t vrhps[5];
	uint8_t vrhns[5];
	uint8_t vcoms[1];
	uint8_t pgc[14];
	uint8_t ngc[14];
	uint8_t frctra1[2];
	uint8_t frctra2[2];
	uint8_t frctra3[5];
	uint8_t pwrctra1[2];
	uint8_t pwrctra2[2];
	uint8_t pwrctra3[2];
	bool rgb_is_inverted;
	uint16_t x_offset;
	uint16_t y_offset;
};

static int st77916_send_cmd(const struct device *dev,
			    uint8_t cmd, const uint8_t *data, size_t len)
{
	const struct st77916_config *config = dev->config;

	return mipi_dbi_command_write(config->mipi_dbi, &config->dbi_config,
				      cmd, data, len);
}

static int st77916_set_cursor(const struct device *dev,
			      const uint16_t x, const uint16_t y,
			      const uint16_t width, const uint16_t height)
{
	const struct st77916_config *config = dev->config;
	uint16_t addr_data[2];
	int ret;

	/* Column address */
	addr_data[0] = sys_cpu_to_be16(x + config->x_offset);
	addr_data[1] = sys_cpu_to_be16(x + config->x_offset + width - 1);

	ret = st77916_send_cmd(dev, ST77916_CMD_CASET,
			       (uint8_t *)addr_data, sizeof(addr_data));
	if (ret < 0) {
		return ret;
	}

	/* Row address */
	addr_data[0] = sys_cpu_to_be16(y + config->y_offset);
	addr_data[1] = sys_cpu_to_be16(y + config->y_offset + height - 1);

	ret = st77916_send_cmd(dev, ST77916_CMD_RASET,
			       (uint8_t *)addr_data, sizeof(addr_data));

	return ret;
}

static int st77916_blanking_on(const struct device *dev)
{
	return st77916_send_cmd(dev, ST77916_CMD_DISPOFF, NULL, 0);
}

static int st77916_blanking_off(const struct device *dev)
{
	return st77916_send_cmd(dev, ST77916_CMD_DISPON, NULL, 0);
}

static int st77916_get_pixelfmt(const struct device *dev)
{
	const struct st77916_config *config = dev->config;

	if (((bool)(config->madctl & ST77916_MADCTL_BGR)) !=
	    config->rgb_is_inverted) {
		return PIXEL_FORMAT_RGB_565;
	}

	return PIXEL_FORMAT_RGB_565X;
}

static int st77916_write(const struct device *dev,
			 const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	const struct st77916_config *config = dev->config;
	int ret;
	struct display_buffer_descriptor mipi_desc;
	enum display_pixel_format pixfmt;

	ret = st77916_set_cursor(dev, x, y, desc->width, desc->height);
	if (ret < 0) {
		return ret;
	}

	mipi_desc.buf_size = desc->width * desc->height * ST77916_PIXEL_SIZE;
	mipi_desc.frame_incomplete = desc->frame_incomplete;
	mipi_desc.pitch = desc->pitch;
	mipi_desc.width = desc->width;
	mipi_desc.height = desc->height;

	ret = mipi_dbi_command_write(config->mipi_dbi,
				     &config->dbi_config, ST77916_CMD_RAMWR,
				     NULL, 0);
	if (ret < 0) {
		return ret;
	}

	pixfmt = st77916_get_pixelfmt(dev);

	return mipi_dbi_write_display(config->mipi_dbi,
				      &config->dbi_config, buf,
				      &mipi_desc, pixfmt);
}

static void st77916_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct st77916_config *config = dev->config;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->current_pixel_format = st77916_get_pixelfmt(dev);
	capabilities->supported_pixel_formats = capabilities->current_pixel_format;
	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int st77916_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	if (pixel_format == st77916_get_pixelfmt(dev)) {
		return 0;
	}

	return -ENOTSUP;
}

static int st77916_lcd_config(const struct device *dev)
{
	const struct st77916_config *config = dev->config;
	int ret;
	uint8_t param;

	/* Enable Command Table 2 */
	param = ST77916_CMD2_ENABLE;
	ret = st77916_send_cmd(dev, ST77916_CMD_CSC1, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	param = ST77916_CMD2_UNLOCK;
	ret = st77916_send_cmd(dev, ST77916_CMD_CSC2, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	/* VRHP (positive voltage regulator) */
	ret = st77916_send_cmd(dev, ST77916_CMD_VRHPS,
			       config->vrhps, sizeof(config->vrhps));
	if (ret < 0) {
		return ret;
	}

	/* VRHN (negative voltage regulator) */
	ret = st77916_send_cmd(dev, ST77916_CMD_VRHNS,
			       config->vrhns, sizeof(config->vrhns));
	if (ret < 0) {
		return ret;
	}

	/* VCOM */
	ret = st77916_send_cmd(dev, ST77916_CMD_VCOMS,
			       config->vcoms, sizeof(config->vcoms));
	if (ret < 0) {
		return ret;
	}

	/* Frame rate control */
	ret = st77916_send_cmd(dev, ST77916_CMD_FRCTRA1,
			       config->frctra1, sizeof(config->frctra1));
	if (ret < 0) {
		return ret;
	}

	ret = st77916_send_cmd(dev, ST77916_CMD_FRCTRA2,
			       config->frctra2, sizeof(config->frctra2));
	if (ret < 0) {
		return ret;
	}

	ret = st77916_send_cmd(dev, ST77916_CMD_FRCTRA3,
			       config->frctra3, sizeof(config->frctra3));
	if (ret < 0) {
		return ret;
	}

	/* Power control */
	ret = st77916_send_cmd(dev, ST77916_CMD_PWRCTRA1,
			       config->pwrctra1, sizeof(config->pwrctra1));
	if (ret < 0) {
		return ret;
	}

	ret = st77916_send_cmd(dev, ST77916_CMD_PWRCTRA2,
			       config->pwrctra2, sizeof(config->pwrctra2));
	if (ret < 0) {
		return ret;
	}

	ret = st77916_send_cmd(dev, ST77916_CMD_PWRCTRA3,
			       config->pwrctra3, sizeof(config->pwrctra3));
	if (ret < 0) {
		return ret;
	}

	/* Disable Command Table 2 */
	param = ST77916_PAGE_DISABLE;
	ret = st77916_send_cmd(dev, ST77916_CMD_CSC1, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	/* Enable gamma page */
	param = ST77916_GAM_ENABLE;
	ret = st77916_send_cmd(dev, ST77916_CMD_CSC1, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	/* Positive gamma */
	ret = st77916_send_cmd(dev, ST77916_CMD_GAMCTRP1,
			       config->pgc, sizeof(config->pgc));
	if (ret < 0) {
		return ret;
	}

	/* Negative gamma */
	ret = st77916_send_cmd(dev, ST77916_CMD_GAMCTRN1,
			       config->ngc, sizeof(config->ngc));
	if (ret < 0) {
		return ret;
	}

	/* Disable gamma page */
	param = ST77916_PAGE_DISABLE;
	ret = st77916_send_cmd(dev, ST77916_CMD_CSC1, &param, sizeof(param));

	return ret;
}

static int st77916_init(const struct device *dev)
{
	const struct st77916_config *config = dev->config;
	int ret;
	uint8_t param;

	if (!device_is_ready(config->mipi_dbi)) {
		LOG_ERR("MIPI DBI device not ready");
		return -ENODEV;
	}

	/* Hardware reset: pulse for 20us (minimum 10us per datasheet) */
	ret = mipi_dbi_reset(config->mipi_dbi, 1);
	if (ret < 0) {
		return ret;
	}
	/* Wait 120ms after reset release for voltage stabilization */
	k_msleep(120);

	/* Configure display registers (Command Table 2 + gamma) */
	ret = st77916_lcd_config(dev);
	if (ret < 0) {
		LOG_ERR("Could not set LCD configuration (%d)", ret);
		return ret;
	}

	/* Display inversion */
	if (config->inverted) {
		ret = st77916_send_cmd(dev, ST77916_CMD_INVON, NULL, 0);
	} else {
		ret = st77916_send_cmd(dev, ST77916_CMD_INVOFF, NULL, 0);
	}
	if (ret < 0) {
		return ret;
	}

	/* Pixel format */
	param = config->colmod;
	ret = st77916_send_cmd(dev, ST77916_CMD_COLMOD, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	/* Memory data access control */
	param = config->madctl;
	ret = st77916_send_cmd(dev, ST77916_CMD_MADCTL, &param, sizeof(param));
	if (ret < 0) {
		return ret;
	}

	/* Normal display mode on */
	ret = st77916_send_cmd(dev, ST77916_CMD_NORON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	/* Exit sleep */
	ret = st77916_send_cmd(dev, ST77916_CMD_SLPOUT, NULL, 0);
	if (ret < 0) {
		return ret;
	}
	/* Wait 120ms after sleep out, per datasheet */
	k_msleep(120);

	/* Display on */
	ret = st77916_send_cmd(dev, ST77916_CMD_DISPON, NULL, 0);

	return ret;
}

static DEVICE_API(display, st77916_api) = {
	.blanking_on = st77916_blanking_on,
	.blanking_off = st77916_blanking_off,
	.write = st77916_write,
	.get_capabilities = st77916_get_capabilities,
	.set_pixel_format = st77916_set_pixel_format,
};

#define ST77916_INIT(n)								\
	static const struct st77916_config st77916_config_##n = {		\
		.mipi_dbi = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		.dbi_config = {							\
			.config = MIPI_DBI_SPI_CONFIG_DT(			\
						DT_DRV_INST(n),			\
						SPI_OP_MODE_MASTER |		\
						SPI_WORD_SET(8),		\
						0),				\
			.mode = DT_INST_STRING_UPPER_TOKEN_OR(n, mipi_mode,	\
						MIPI_DBI_MODE_SPI_4WIRE),	\
			.color_coding = DT_INST_STRING_UPPER_TOKEN_OR(n,	\
						color_coding,			\
						MIPI_DBI_MODE_RGB565),		\
		},								\
		.width = DT_INST_PROP(n, width),				\
		.height = DT_INST_PROP(n, height),				\
		.inverted = DT_INST_PROP(n, color_invert),			\
		.madctl = DT_INST_PROP(n, madctl),				\
		.colmod = DT_INST_PROP(n, colmod),				\
		.vrhps = DT_INST_PROP(n, vrhps),				\
		.vrhns = DT_INST_PROP(n, vrhns),				\
		.vcoms = DT_INST_PROP(n, vcoms),				\
		.pgc = DT_INST_PROP(n, pgc),					\
		.ngc = DT_INST_PROP(n, ngc),					\
		.frctra1 = DT_INST_PROP(n, frctra1),				\
		.frctra2 = DT_INST_PROP(n, frctra2),				\
		.frctra3 = DT_INST_PROP(n, frctra3),				\
		.pwrctra1 = DT_INST_PROP(n, pwrctra1),				\
		.pwrctra2 = DT_INST_PROP(n, pwrctra2),				\
		.pwrctra3 = DT_INST_PROP(n, pwrctra3),				\
		.rgb_is_inverted = DT_INST_PROP(n, rgb_is_inverted),		\
		.x_offset = DT_INST_PROP(n, x_offset),				\
		.y_offset = DT_INST_PROP(n, y_offset),				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, st77916_init,					\
			NULL,							\
			NULL,							\
			&st77916_config_##n,					\
			POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,		\
			&st77916_api);

DT_INST_FOREACH_STATUS_OKAY(ST77916_INIT)
