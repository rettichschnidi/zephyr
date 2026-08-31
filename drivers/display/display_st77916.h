/*
 * Copyright (c) 2026 Siemens
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_ST77916_H_
#define ZEPHYR_DRIVERS_DISPLAY_ST77916_H_

/* Command Table 1 */
#define ST77916_CMD_NOP      0x00 /* No operation */
#define ST77916_CMD_SWRESET  0x01 /* Software reset */
#define ST77916_CMD_SLPIN    0x10 /* Sleep in */
#define ST77916_CMD_SLPOUT   0x11 /* Sleep out */
#define ST77916_CMD_NORON    0x13 /* Normal display mode on */
#define ST77916_CMD_INVOFF   0x20 /* Display inversion off */
#define ST77916_CMD_INVON    0x21 /* Display inversion on */
#define ST77916_CMD_DISPOFF  0x28 /* Display off */
#define ST77916_CMD_DISPON   0x29 /* Display on */
#define ST77916_CMD_CASET    0x2A /* Column address set */
#define ST77916_CMD_RASET    0x2B /* Row address set */
#define ST77916_CMD_RAMWR    0x2C /* Memory write */
#define ST77916_CMD_TEON     0x35 /* Tearing effect line on */
#define ST77916_CMD_MADCTL   0x36 /* Memory data access control */
#define ST77916_CMD_COLMOD   0x3A /* Interface pixel format */
#define ST77916_CMD_STE      0x44 /* Write tear scanline */
#define ST77916_CMD_WRDISBV  0x51 /* Write display brightness */
#define ST77916_CMD_WRCTRLD  0x53 /* Write CTRL display */

/* Page Set / Command Set Control */
#define ST77916_CMD_CSC1     0xF0 /* Command set ctrl 1 */
#define ST77916_CMD_CSC2     0xF1 /* Command set ctrl 2 */
#define ST77916_CMD_CSC3     0xF2 /* Command set ctrl 3 */
#define ST77916_CMD_CSC4     0xF3 /* Command set ctrl 4 */

/* Command Table 2 (require CSC page enable) */
#define ST77916_CMD_VRHPS    0xB0 /* VRHP set */
#define ST77916_CMD_VRHNS    0xB1 /* VRHN set */
#define ST77916_CMD_VCOMS    0xB2 /* VCOM GND set */
#define ST77916_CMD_STEP14S  0xB5 /* Step set 1 */
#define ST77916_CMD_STEP23S  0xB6 /* Step set 2 */
#define ST77916_CMD_SBSTS    0xB7 /* SVDD SVCL set */
#define ST77916_CMD_TCONS    0xBA /* TCON set */
#define ST77916_CMD_FRCTRA1  0xC0 /* Frame rate control A1, normal mode */
#define ST77916_CMD_FRCTRA2  0xC1 /* Frame rate control A2, normal mode */
#define ST77916_CMD_FRCTRA3  0xC2 /* Frame rate control A3, normal mode */
#define ST77916_CMD_PWRCTRA1 0xC6 /* Power control A1, normal mode */
#define ST77916_CMD_PWRCTRA2 0xC7 /* Power control A2, normal mode */
#define ST77916_CMD_PWRCTRA3 0xC8 /* Power control A3, normal mode */
#define ST77916_CMD_RESSET1  0xD0 /* Resolution set 1 */
#define ST77916_CMD_RESSET2  0xD1 /* Resolution set 2 */
#define ST77916_CMD_RESSET3  0xD2 /* Resolution set 3 */
#define ST77916_CMD_VCMOFSET 0xDD /* VCOM offset set */
#define ST77916_CMD_GAMCTRP1 0xE0 /* Positive voltage gamma control */
#define ST77916_CMD_GAMCTRN1 0xE1 /* Negative voltage gamma control */

/* MADCTL bit definitions */
#define ST77916_MADCTL_MY    BIT(7) /* Row address order */
#define ST77916_MADCTL_MX    BIT(6) /* Column address order */
#define ST77916_MADCTL_MV    BIT(5) /* Row/column exchange */
#define ST77916_MADCTL_ML    BIT(4) /* Vertical refresh order */
#define ST77916_MADCTL_BGR   BIT(3) /* BGR color order */
#define ST77916_MADCTL_MH    BIT(2) /* Horizontal refresh order */

/* COLMOD pixel format values */
#define ST77916_COLMOD_RGB565 0x55 /* 16-bit/pixel */
#define ST77916_COLMOD_RGB666 0x66 /* 18-bit/pixel */

/* Pixel size in bytes (16-bit color mode) */
#define ST77916_PIXEL_SIZE 2

#endif /* ZEPHYR_DRIVERS_DISPLAY_ST77916_H_ */
