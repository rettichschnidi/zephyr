/* ieee802154_si4467.h - Registers definition for TI SI4467 */

/*
 * Copyright (c) 2020 Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_SI4467_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_SI4467_H_

#include <linker/sections.h>
#include <sys/atomic.h>
#include <drivers/spi.h>

#include <drivers/ieee802154/si4467.h>

/* Note for EMK & EM adapter booster pack users:
 * SPI pins are easy, RESET as well, but when it comes to GPIO:
 * CHIP -> EM adapter
 * GPIO0 -> GPIOA
 * GPIO1 -> reserved (it's SPI MISO)
 * GPIO2 -> GPIOB
 * GPIO3 -> GPIO3
 */

enum si4467_gpio_index {
	SI4467_GPIO_IDX_GPIO0,
	SI4467_GPIO_IDX_MAX,
};

struct si4467_gpio_configuration {
	struct device *dev;
	u32_t pin;
};

/* Runtime context structure
 ***************************
 */

struct si4467_context {
	struct net_if *iface;
	/**************************/
	struct si4467_gpio_configuration gpios[SI4467_GPIO_IDX_MAX];
	struct gpio_callback rx_tx_cb;
	struct device *spi;
	struct spi_config spi_cfg;
	u8_t mac_addr[8];
	/************RF************/
	const struct si4467_rf_registers_set *rf_settings;
	/************TX************/
	struct k_sem tx_sync;
	atomic_t tx;
	atomic_t tx_start;
	/************RX************/
	K_THREAD_STACK_MEMBER(rx_stack,
			      CONFIG_IEEE802154_SI4467_RX_STACK_SIZE);
	struct k_thread rx_thread;
	struct k_sem rx_lock;
	atomic_t rx;
};

#include "ieee802154_si4467_regs.h"

/* Registers useful routines
 ***************************
 */

bool z_si4467_access_reg(struct si4467_context *ctx, bool read, u8_t addr,
			void *data, size_t length, bool extended, bool burst);

static inline u8_t si4467_read_single_reg(struct si4467_context *ctx,
					   u8_t addr, bool extended)
{
	u8_t val;

	if (z_si4467_access_reg(ctx, true, addr, &val, 1, extended, false)) {
		return val;
	}

	return 0;
}

static inline bool si4467_write_single_reg(struct si4467_context *ctx,
					    u8_t addr, u8_t val, bool extended)
{
	return z_si4467_access_reg(ctx, false, addr, &val, 1, extended, false);
}

static inline bool si4467_instruct(struct si4467_context *ctx, u8_t addr)
{
	return z_si4467_access_reg(ctx, false, addr, NULL, 0, false, false);
}

#define DEFINE_REG_READ(__reg_name, __reg_addr, __ext)			\
	static inline u8_t read_reg_##__reg_name(struct si4467_context *ctx) \
	{								\
		return si4467_read_single_reg(ctx, __reg_addr, __ext);	\
	}

#define DEFINE_REG_WRITE(__reg_name, __reg_addr, __ext)			\
	static inline bool write_reg_##__reg_name(struct si4467_context *ctx, \
						  u8_t val)		\
	{								\
		return si4467_write_single_reg(ctx, __reg_addr,	\
						val, __ext);		\
	}

DEFINE_REG_WRITE(iocfg3, SI4467_REG_IOCFG3, false)
DEFINE_REG_WRITE(iocfg2, SI4467_REG_IOCFG2, false)
DEFINE_REG_WRITE(iocfg0, SI4467_REG_IOCFG0, false)
DEFINE_REG_WRITE(pa_cfg1, SI4467_REG_PA_CFG1, false)
DEFINE_REG_WRITE(pkt_len, SI4467_REG_PKT_LEN, false)

DEFINE_REG_READ(fs_cfg, SI4467_REG_FS_CFG, false)
DEFINE_REG_READ(rssi0, SI4467_REG_RSSI0, true)
DEFINE_REG_READ(pa_cfg1, SI4467_REG_PA_CFG1, false)
DEFINE_REG_READ(num_txbytes, SI4467_REG_NUM_TXBYTES, true)
DEFINE_REG_READ(num_rxbytes, SI4467_REG_NUM_RXBYTES, true)


/* Instructions useful routines
 ******************************
 */

#define DEFINE_STROBE_INSTRUCTION(__ins_name, __ins_addr)		\
	static inline bool instruct_##__ins_name(struct si4467_context *ctx) \
	{								\
		return si4467_instruct(ctx, __ins_addr);		\
	}

DEFINE_STROBE_INSTRUCTION(sres, SI4467_INS_SRES)
DEFINE_STROBE_INSTRUCTION(sfstxon, SI4467_INS_SFSTXON)
DEFINE_STROBE_INSTRUCTION(sxoff, SI4467_INS_SXOFF)
DEFINE_STROBE_INSTRUCTION(scal, SI4467_INS_SCAL)
DEFINE_STROBE_INSTRUCTION(srx, SI4467_INS_SRX)
DEFINE_STROBE_INSTRUCTION(stx, SI4467_INS_STX)
DEFINE_STROBE_INSTRUCTION(sidle, SI4467_INS_SIDLE)
DEFINE_STROBE_INSTRUCTION(safc, SI4467_INS_SAFC)
DEFINE_STROBE_INSTRUCTION(swor, SI4467_INS_SWOR)
DEFINE_STROBE_INSTRUCTION(spwd, SI4467_INS_SPWD)
DEFINE_STROBE_INSTRUCTION(sfrx, SI4467_INS_SFRX)
DEFINE_STROBE_INSTRUCTION(sftx, SI4467_INS_SFTX)
DEFINE_STROBE_INSTRUCTION(sworrst, SI4467_INS_SWORRST)
DEFINE_STROBE_INSTRUCTION(snop, SI4467_INS_SNOP)

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_SI4467_H_ */
