/* ieee802154_si4467.c - TI SI4467 driver */

#define DT_DRV_COMPAT ti_si4467

/*
 * Copyright (c) 2020 Reto Schneider
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_si4467
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <kernel.h>
#include <arch/cpu.h>
#include <debug/stack.h>

#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <sys/byteorder.h>
#include <string.h>
#include <random/rand32.h>

#include <drivers/spi.h>
#include <drivers/gpio.h>

#include <net/ieee802154_radio.h>

#include "ieee802154_si4467.h"
#include "ieee802154_si4467_rf.h"

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
static struct spi_cs_control cs_ctrl;
#endif

/* ToDo: supporting 802.15.4g will require GPIO2
 * used as SI4467_GPIO_SIG_RXFIFO_THR
 *
 * Note: GPIO3 is unused.
 */
#define SI4467_IOCFG3	SI4467_GPIO_SIG_MARC_2PIN_STATUS_0
#define SI4467_IOCFG2	SI4467_GPIO_SIG_MARC_2PIN_STATUS_1
#define SI4467_IOCFG0	SI4467_GPIO_SIG_PKT_SYNC_RXTX

/***********************
 * Debugging functions *
 **********************/
static void si4467_print_status(u8_t status)
{
	if (status == SI4467_STATUS_IDLE) {
		LOG_DBG("Idling");
	} else if (status == SI4467_STATUS_RX) {
		LOG_DBG("Receiving");
	} else if (status == SI4467_STATUS_TX) {
		LOG_DBG("Transmitting");
	} else if (status == SI4467_STATUS_FSTXON) {
		LOG_DBG("FS TX on");
	} else if (status == SI4467_STATUS_CALIBRATE) {
		LOG_DBG("Calibrating");
	} else if (status == SI4467_STATUS_SETTLING) {
		LOG_DBG("Settling");
	} else if (status == SI4467_STATUS_RX_FIFO_ERROR) {
		LOG_DBG("RX FIFO error!");
	} else if (status == SI4467_STATUS_TX_FIFO_ERROR) {
		LOG_DBG("TX FIFO error!");
	}
}

/*********************
 * Generic functions *
 ********************/

bool z_si4467_access_reg(struct si4467_context *ctx, bool read, u8_t addr,
			void *data, size_t length, bool extended, bool burst)
{
	u8_t cmd_buf[2];
	const struct spi_buf buf[2] = {
		{
			.buf = cmd_buf,
			.len = extended ? 2 : 1,
		},
		{
			.buf = data,
			.len = length,

		}
	};
	struct spi_buf_set tx = { .buffers = buf };

	/*
	LOG_DBG("%s: addr 0x%02x - Data %p Length %u - %s, %s",
		    read ? "Read" : "Write", addr, data, length,
		    extended ? "extended" : "normal",
		    burst ? "burst" : "single");
	*/

	cmd_buf[0] = 0U;

	if (burst) {
		cmd_buf[0] |= SI4467_ACCESS_BURST;
	}

	if (extended) {
		cmd_buf[0] |= SI4467_REG_EXTENDED_ADDRESS;
		cmd_buf[1] = addr;
	} else {
		cmd_buf[0] |= addr;
	}

	if (read) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		cmd_buf[0] |= SI4467_ACCESS_RD;

		tx.count = 1;

		return (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) == 0);
	}

	/* SI4467_ACCESS_WR is 0 so no need to play with it */
	tx.count =  data ? 2 : 1;

	return (spi_write(ctx->spi, &ctx->spi_cfg, &tx) == 0);
}

static inline u8_t *get_mac(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

#if defined(CONFIG_IEEE802154_SI4467_RANDOM_MAC)
	u32_t *ptr = (u32_t *)(si4467->mac_addr + 4);

	UNALIGNED_PUT(sys_rand32_get(), ptr);

	si4467->mac_addr[7] = (si4467->mac_addr[7] & ~0x01) | 0x02;
#else
	si4467->mac_addr[4] = CONFIG_IEEE802154_SI4467_MAC4;
	si4467->mac_addr[5] = CONFIG_IEEE802154_SI4467_MAC5;
	si4467->mac_addr[6] = CONFIG_IEEE802154_SI4467_MAC6;
	si4467->mac_addr[7] = CONFIG_IEEE802154_SI4467_MAC7;
#endif

	si4467->mac_addr[0] = 0x00;
	si4467->mac_addr[1] = 0x12;
	si4467->mac_addr[2] = 0x4b;
	si4467->mac_addr[3] = 0x00;

	return si4467->mac_addr;
}

static u8_t get_status(struct si4467_context *ctx)
{
	u8_t val;

	if (z_si4467_access_reg(ctx, true, SI4467_INS_SNOP,
				&val, 1, false, false)) {
		/* See Section 3.1.2 */
		return val & SI4467_STATUS_MASK;
	}

	/* We cannot get the status, so let's assume about readyness */
	return SI4467_STATUS_CHIP_NOT_READY;
}

/******************
 * GPIO functions *
 *****************/

static inline void gpio0_int_handler(struct device *port,
				     struct gpio_callback *cb, u32_t pins)
{
	struct si4467_context *si4467 =
		CONTAINER_OF(cb, struct si4467_context, rx_tx_cb);

	if (atomic_get(&si4467->tx) == 1) {
		if (atomic_get(&si4467->tx_start) == 0) {
			atomic_set(&si4467->tx_start, 1);
		} else {
			atomic_set(&si4467->tx, 0);
		}

		k_sem_give(&si4467->tx_sync);
	} else {
		if (atomic_get(&si4467->rx) == 1) {
			k_sem_give(&si4467->rx_lock);
			atomic_set(&si4467->rx, 0);
		} else {
			atomic_set(&si4467->rx, 1);
		}
	}
}

static void enable_gpio0_interrupt(struct si4467_context *si4467, bool enable)
{
	gpio_pin_interrupt_configure(
		si4467->gpios[SI4467_GPIO_IDX_GPIO0].dev,
		si4467->gpios[SI4467_GPIO_IDX_GPIO0].pin,
		enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
}

static void setup_gpio_callback(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	gpio_init_callback(&si4467->rx_tx_cb, gpio0_int_handler,
			   BIT(si4467->gpios[SI4467_GPIO_IDX_GPIO0].pin));
	gpio_add_callback(si4467->gpios[SI4467_GPIO_IDX_GPIO0].dev,
			  &si4467->rx_tx_cb);
}

/****************
 * RF functions *
 ***************/

static u8_t get_lo_divider(struct si4467_context *ctx)
{
	/* See Table 34  */
	return FSD_BANDSELECT(read_reg_fs_cfg(ctx)) << 1;
}

static bool write_reg_freq(struct si4467_context *ctx, u32_t freq)
{
	u8_t freq_data[3];

	freq_data[0] = (u8_t)((freq & 0x00FF0000) >> 16);
	freq_data[1] = (u8_t)((freq & 0x0000FF00) >> 8);
	freq_data[2] = (u8_t)(freq & 0x000000FF);

	return z_si4467_access_reg(ctx, false, SI4467_REG_FREQ2,
				  freq_data, 3, true, true);
}


/* See Section 9.12 - RF programming
 *
 * The given formula in datasheet cannot be simply applied here, where CPU
 * limits us to unsigned integers of 32 bits. Instead, "slicing" it to
 * parts that fits in such limit is a solution which is applied below.
 *
 * The original formula being (freqoff is neglegted):
 * Freq = ( RF * Lo_Div * 2^16 ) / Xtal
 *
 * RF and Xtal are, from here, expressed in KHz.
 *
 * It first calculates the targeted RF with given ChanCenterFreq0, channel
 * spacing and the channel number.
 *
 * The calculation will slice the targeted RF by multiple of 10:
 * 10^n where n is in [5, 3]. The rest, below 1000, is taken at once.
 * Let's take the 434000 KHz RF for instance:
 * it will be "sliced" in 3 parts: 400000, 30000, 4000.
 * Or the 169406 KHz RF, 4 parts: 100000, 60000, 9000, 406.
 *
 * This permits also to play with Xtal to keep the result big enough to avoid
 * losing precision. A factor - growing as much as Xtal decrease -  is then
 * applied to get to the proper result. Which one is rounded to the nearest
 * integer, again to get a bit better precision.
 *
 * In the end, this algorithm below works for all the supported bands by SI4467.
 * User does not need to pass anything extra besides the nominal settings: no
 * pre-computed part or else.
 */
static u32_t rf_evaluate_freq_setting(struct si4467_context *ctx, u32_t chan)
{
	u32_t xtal = CONFIG_IEEE802154_SI4467_XOSC;
	u32_t mult_10 = 100000U;
	u32_t factor = 1U;
	u32_t freq = 0U;
	u32_t rf, lo_div;

	rf = ctx->rf_settings->chan_center_freq0 +
		((chan * (u32_t)ctx->rf_settings->channel_spacing) / 10U);
	lo_div = get_lo_divider(ctx);

	LOG_DBG("Calculating freq for %u KHz RF (%u)", rf, lo_div);

	while (rf > 0) {
		u32_t hz, freq_tmp, rst;

		if (rf < 1000) {
			hz = rf;
		} else {
			hz = rf / mult_10;
			hz *= mult_10;
		}

		if (hz < 1000) {
			freq_tmp = (hz * lo_div * 65536U) / xtal;
		} else {
			freq_tmp = ((hz * lo_div) / xtal) * 65536U;
		}

		rst = freq_tmp % factor;
		freq_tmp /= factor;

		if (factor > 1 && (rst/(factor/10U)) > 5) {
			freq_tmp++;
		}

		freq += freq_tmp;

		factor *= 10U;
		mult_10 /= 10U;
		xtal /= 10U;
		rf -= hz;
	}

	LOG_DBG("FREQ is 0x%06X", freq);

	return freq;
}

static bool
rf_install_settings(struct device *dev,
		    const struct si4467_rf_registers_set *rf_settings)
{
	struct si4467_context *si4467 = dev->driver_data;

	if (!z_si4467_access_reg(si4467, false, SI4467_REG_SYNC3,
				 (void *)rf_settings->registers,
				 SI4467_RF_NON_EXT_SPACE_REGS, false, true) ||
	    !z_si4467_access_reg(si4467, false, SI4467_REG_IF_MIX_CFG,
				 (u8_t *)rf_settings->registers
				 + SI4467_RF_NON_EXT_SPACE_REGS,
				 SI4467_RF_EXT_SPACE_REGS, true, true) ||
	    !write_reg_pkt_len(si4467, 0xFF)) {
		LOG_ERR("Could not install RF settings");
		return false;
	}

	si4467->rf_settings = rf_settings;

	return true;
}

static int rf_calibrate(struct si4467_context *ctx)
{
	if (!instruct_scal(ctx)) {
		LOG_ERR("Could not calibrate RF");
		return -EIO;
	}

	k_busy_wait(USEC_PER_MSEC * 5U);

	/* We need to re-enable RX as SCAL shuts off the freq synth */
	if (!instruct_sidle(ctx) ||
	    !instruct_sfrx(ctx) ||
	    !instruct_srx(ctx)) {
		LOG_ERR("Could not switch to RX");
		return -EIO;
	}

	k_busy_wait(USEC_PER_MSEC * 10U);

	si4467_print_status(get_status(ctx));

	return 0;
}

/****************
 * TX functions *
 ***************/

static inline bool write_txfifo(struct si4467_context *ctx,
				void *data, size_t length)
{
	return z_si4467_access_reg(ctx, false,
				  SI4467_REG_TXFIFO,
				  data, length, false, true);
}

/****************
 * RX functions *
 ***************/

static inline bool read_rxfifo(struct si4467_context *ctx,
			       void *data, size_t length)
{
	return z_si4467_access_reg(ctx, true,
				  SI4467_REG_RXFIFO,
				  data, length, false, true);
}

static inline u8_t get_packet_length(struct si4467_context *ctx)
{
	u8_t len;

	if (z_si4467_access_reg(ctx, true, SI4467_REG_RXFIFO,
				&len, 1, false, true)) {
		return len;
	}

	return 0;
}

static inline bool verify_rxfifo_validity(struct si4467_context *ctx,
					  u8_t pkt_len)
{
	/* packet should be at least 3 bytes as a ACK */
	if (pkt_len < 3 ||
	    read_reg_num_rxbytes(ctx) > (pkt_len + SI4467_FCS_LEN)) {
		return false;
	}

	return true;
}

static inline bool read_rxfifo_content(struct si4467_context *ctx,
				       struct net_buf *buf, u8_t len)
{

	if (!read_rxfifo(ctx, buf->data, len) ||
	    (get_status(ctx) == SI4467_STATUS_RX_FIFO_ERROR)) {
		return false;
	}

	net_buf_add(buf, len);

	return true;
}

static inline bool verify_crc(struct si4467_context *ctx, struct net_pkt *pkt)
{
	u8_t fcs[2];

	if (!read_rxfifo(ctx, fcs, 2)) {
		return false;
	}

	if (!(fcs[1] & SI4467_FCS_CRC_OK)) {
		return false;
	}

	net_pkt_set_ieee802154_rssi(pkt, fcs[0]);
	net_pkt_set_ieee802154_lqi(pkt, fcs[1] & SI4467_FCS_LQI_MASK);

	return true;
}

static void si4467_rx(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;
	struct net_pkt *pkt;
	u8_t pkt_len;

	while (1) {
		pkt = NULL;

		k_sem_take(&si4467->rx_lock, K_FOREVER);

		if (get_status(si4467) == SI4467_STATUS_RX_FIFO_ERROR) {
			LOG_ERR("Fifo error");
			goto flush;
		}

		pkt_len = get_packet_length(si4467);
		if (!verify_rxfifo_validity(si4467, pkt_len)) {
			LOG_ERR("Invalid frame");
			goto flush;
		}

		pkt = net_pkt_alloc_with_buffer(si4467->iface, pkt_len,
						AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("No free pkt available");
			goto flush;
		}

		if (!read_rxfifo_content(si4467, pkt->buffer, pkt_len)) {
			LOG_ERR("No content read");
			goto flush;
		}

		if (!verify_crc(si4467, pkt)) {
			LOG_ERR("Bad packet CRC");
			goto out;
		}

		if (ieee802154_radio_handle_ack(si4467->iface, pkt) == NET_OK) {
			LOG_DBG("ACK packet handled");
			goto out;
		}

		LOG_DBG("Caught a packet (%u)", pkt_len);

		if (net_recv_data(si4467->iface, pkt) < 0) {
			LOG_DBG("Packet dropped by NET stack");
			goto out;
		}

		log_stack_usage(&si4467->rx_thread);
		continue;
flush:
		LOG_DBG("Flushing RX");
		instruct_sidle(si4467);
		instruct_sfrx(si4467);
		instruct_srx(si4467);
out:
		if (pkt) {
			net_pkt_unref(pkt);
		}

	}
}


/********************
 * Radio device API *
 *******************/
static enum ieee802154_hw_caps si4467_get_capabilities(struct device *dev)
{
	return IEEE802154_HW_FCS | IEEE802154_HW_SUB_GHZ;
}

static int si4467_cca(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	if (atomic_get(&si4467->rx) == 0) {
		u8_t status = read_reg_rssi0(si4467);

		if (!(status & CARRIER_SENSE) &&
		    (status & CARRIER_SENSE_VALID)) {
			return 0;
		}
	}

	LOG_WRN("Busy");

	return -EBUSY;
}

static int si4467_set_channel(struct device *dev, u16_t channel)
{
	struct si4467_context *si4467 = dev->driver_data;

	/* Unlike usual 15.4 chips, si4467 is closer to a bare metal radio modem
	 * and thus does not provide any means to select a channel directly, but
	 * requires instead that one calculates and configures the actual
	 * targeted frequency for the requested channel.
	 *
	 * See rf_evaluate_freq_setting() above.
	 */

	if (atomic_get(&si4467->rx) == 0) {
		u32_t freq = rf_evaluate_freq_setting(si4467, channel);

		if (!write_reg_freq(si4467, freq) ||
		    rf_calibrate(si4467)) {
			LOG_ERR("Could not set channel %u", channel);
			return -EIO;
		}
	}

	return 0;
}

static int si4467_set_txpower(struct device *dev, s16_t dbm)
{
	struct si4467_context *si4467 = dev->driver_data;
	u8_t pa_power_ramp;

	LOG_DBG("%d dbm", dbm);

	/* See Section 7.1 */
	dbm = ((dbm + 18) * 2) - 1;
	if ((dbm <= 3) || (dbm >= 64)) {
		LOG_ERR("Unhandled value");
		return -EINVAL;
	}

	pa_power_ramp = read_reg_pa_cfg1(si4467) & ~PA_POWER_RAMP_MASK;
	pa_power_ramp |= ((u8_t) dbm) & PA_POWER_RAMP_MASK;

	if (!write_reg_pa_cfg1(si4467, pa_power_ramp)) {
		LOG_ERR("Could not proceed");
		return -EIO;
	}

	return 0;
}

static int si4467_tx(struct device *dev,
		     enum ieee802154_tx_mode mode,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
	struct si4467_context *si4467 = dev->driver_data;
	u8_t *frame = frag->data;
	u8_t len = frag->len;
	bool status = false;

	if (mode != IEEE802154_TX_MODE_DIRECT) {
		NET_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	LOG_DBG("%p (%u)", frag, len);

	/* ToDo:
	 * Supporting 802.15.4g will require to loop in pkt's frags
	 * depending on len value, this will also take more time.
	 */

	if (!instruct_sidle(si4467) ||
	    !instruct_sfrx(si4467) ||
	    !instruct_sftx(si4467) ||
	    !instruct_sfstxon(si4467)) {
		LOG_ERR("Cannot switch to TX mode");
		goto out;
	}

	if (!write_txfifo(si4467, &len, SI4467_PHY_HDR_LEN) ||
	    !write_txfifo(si4467, frame, len) ||
	    read_reg_num_txbytes(si4467) != (len + SI4467_PHY_HDR_LEN)) {
		LOG_ERR("Cannot fill-in TX fifo");
		goto out;
	}

	atomic_set(&si4467->tx, 1);
	atomic_set(&si4467->tx_start, 0);

	if (!instruct_stx(si4467)) {
		LOG_ERR("Cannot start transmission");
		goto out;
	}

	/* Wait for SYNC to be sent */
	k_sem_take(&si4467->tx_sync, K_MSEC(100));
	if (atomic_get(&si4467->tx_start) == 1) {
		/* Now wait for the packet to be fully sent */
		k_sem_take(&si4467->tx_sync, K_MSEC(100));
	}

out:
	si4467_print_status(get_status(si4467));

	if (atomic_get(&si4467->tx) == 1 &&
	    read_reg_num_txbytes(si4467) != 0) {
		LOG_ERR("TX Failed");

		atomic_set(&si4467->tx_start, 0);
		instruct_sftx(si4467);
		status = false;
	} else {
		status = true;
	}

	atomic_set(&si4467->tx, 0);

	/* Get back to RX */
	instruct_srx(si4467);

	return status ? 0 : -EIO;
}

static int si4467_start(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	if (!instruct_sidle(si4467) ||
	    !instruct_sftx(si4467) ||
	    !instruct_sfrx(si4467) ||
	    rf_calibrate(si4467)) {
		LOG_ERR("Could not proceed");
		return -EIO;
	}

	enable_gpio0_interrupt(si4467, true);

	si4467_print_status(get_status(si4467));

	return 0;
}

static int si4467_stop(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	enable_gpio0_interrupt(si4467, false);

	if (!instruct_spwd(si4467)) {
		LOG_ERR("Could not proceed");
		return -EIO;
	}

	return 0;
}

static u16_t si4467_get_channel_count(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	return si4467->rf_settings->channel_limit;
}

/******************
 * Initialization *
 *****************/

static int power_on_and_setup(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	if (!instruct_sres(si4467)) {
		LOG_ERR("Cannot reset");
		return -EIO;
	}

	if (!rf_install_settings(dev, &si4467_rf_settings)) {
		return -EIO;
	}

	if (!write_reg_iocfg3(si4467, SI4467_IOCFG3) ||
	    !write_reg_iocfg2(si4467, SI4467_IOCFG2) ||
	    !write_reg_iocfg0(si4467, SI4467_IOCFG0)) {
		LOG_ERR("Cannot configure GPIOs");
		return -EIO;
	}

	setup_gpio_callback(dev);

	return rf_calibrate(si4467);
}

static struct si4467_gpio_configuration *configure_gpios(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;
	struct device *gpio = device_get_binding(DT_INST_GPIO_LABEL(0, int_gpios));

	if (!gpio) {
		return NULL;
	}

	si4467->gpios[SI4467_GPIO_IDX_GPIO0].pin = DT_INST_GPIO_PIN(0, int_gpios);
	gpio_pin_configure(gpio, si4467->gpios[SI4467_GPIO_IDX_GPIO0].pin,
			   GPIO_INPUT | DT_INST_GPIO_FLAGS(0, int_gpios));
	si4467->gpios[SI4467_GPIO_IDX_GPIO0].dev = gpio;

	return si4467->gpios;
}

static int configure_spi(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	si4467->spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!si4467->spi) {
		LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	cs_ctrl.gpio_dev = device_get_binding(
		DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	if (!cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	cs_ctrl.delay = 0U;

	si4467->spi_cfg.cs = &cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
		DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
		DT_INST_SPI_DEV_CS_GPIOS_PIN(0));
#endif

	si4467->spi_cfg.operation = SPI_WORD_SET(8);
	si4467->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	si4467->spi_cfg.slave = DT_INST_REG_ADDR(0);

	return 0;
}

static int si4467_init(struct device *dev)
{
	struct si4467_context *si4467 = dev->driver_data;

	atomic_set(&si4467->tx, 0);
	atomic_set(&si4467->tx_start, 0);
	atomic_set(&si4467->rx, 0);
	k_sem_init(&si4467->rx_lock, 0, 1);
	k_sem_init(&si4467->tx_sync, 0, 1);

	if (!configure_gpios(dev)) {
		LOG_ERR("Configuring GPIOS failed");
		return -EIO;
	}

	if (configure_spi(dev) != 0) {
		LOG_ERR("Configuring SPI failed");
		return -EIO;
	}

	LOG_DBG("GPIO and SPI configured");

	if (power_on_and_setup(dev) != 0) {
		LOG_ERR("Configuring SI4467 failed");
		return -EIO;
	}

	k_thread_create(&si4467->rx_thread, si4467->rx_stack,
			CONFIG_IEEE802154_SI4467_RX_STACK_SIZE,
			(k_thread_entry_t)si4467_rx,
			dev, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&si4467->rx_thread, "si4467_rx");

	LOG_INF("SI4467 initialized");

	return 0;
}

static void si4467_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct si4467_context *si4467 = dev->driver_data;
	u8_t *mac = get_mac(dev);

	LOG_DBG("");

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	si4467->iface = iface;

	ieee802154_init(iface);
}

static struct si4467_context si4467_context_data;

static struct ieee802154_radio_api si4467_radio_api = {
	.iface_api.init	= si4467_iface_init,

	.get_capabilities	= si4467_get_capabilities,
	.cca			= si4467_cca,
	.set_channel		= si4467_set_channel,
	.set_txpower		= si4467_set_txpower,
	.tx			= si4467_tx,
	.start			= si4467_start,
	.stop			= si4467_stop,
	.get_subg_channel_count = si4467_get_channel_count,
};

NET_DEVICE_INIT(si4467, CONFIG_IEEE802154_SI4467_DRV_NAME,
		si4467_init, device_pm_control_nop,
		&si4467_context_data, NULL,
		CONFIG_IEEE802154_SI4467_INIT_PRIO,
		&si4467_radio_api, IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2), 125);
