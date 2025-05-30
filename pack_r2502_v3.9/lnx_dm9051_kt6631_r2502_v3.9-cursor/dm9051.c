// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>

#include "dm9051.h"

#define KERNEL_BUILD_CONF	DM9051_KERNEL_6_6
#define AARCH_OS_BITS		AARCH_OS_64

/* Operating modes */
enum
{
	MODE_POLL = 0,
	MODE_INTERRUPT = 1,
	MODE_INTERRUPT_CLKOUT = 2, /* need pi3/pi5 test verify more */
};

enum
{
	MODE_A = 0,
	MODE_B = 1,
	MODE_C = 2,
	MODE_NUM = 3
};

#define DM9051_SKB_PROTECT		  //tX 'wb' do skb protect
enum
{
	SKB_WB_OFF = 0,
	SKB_WB_ON = 1, //'wb'
};

enum
{
	BURST_MODE_ALIGN = 0,
	BURST_MODE_FULL = 1,
};

/* Driver configuration structure */
struct driver_config
{
	const char *release_version;
	int interrupt;
	int mid;
	struct mod_config
	{
		char *test_info;
		u8 encpt_mode; /* encpt_mode */
		u8 encpt_pad;  /* encpt_setted_key */
		int skb_wb_mode;
		int tx_mode;
		int checksuming;
		struct align_config
		{
			int burst_mode;
			size_t tx_blk; /* alignment, software_build_kernel_conf */
			size_t rx_blk;
		} align;
	} mod[MODE_NUM];
};

/* Default driver configuration */
const struct driver_config confdata = {
	.release_version = "lnx_dm9051_kt6631_r2502_v3.9-cursor",
	.interrupt = MODE_INTERRUPT, /* MODE_INTERRUPT or MODE_INTERRUPT_CLKOUT */
	.mid = MODE_B,
	.mod = {
		{
			.test_info = "Test in rpi5 bcm2712",
			.encpt_mode = FORCE_BUS_ENCPT_FAB_ON,
			.encpt_pad = 0x00, /* or FORCE_BUS_ENCPT_NUL_KEY */
			.skb_wb_mode = SKB_WB_ON,
			.tx_mode = FORCE_TX_CONTI_OFF,
			.checksuming = DEFAULT_CHECKSUM_OFF,
			.align = {.burst_mode = BURST_MODE_ALIGN, .tx_blk = 32, .rx_blk = 64},
		},
		{
			.test_info = "Test in rpi4 bcm2711",
			.encpt_mode = FORCE_BUS_ENCPT_FAB_ON,
			.encpt_pad = 0x00, /* or FORCE_BUS_ENCPT_NUL_KEY */
			.skb_wb_mode = SKB_WB_ON,
			.tx_mode = FORCE_TX_CONTI_OFF,
			.checksuming = DEFAULT_CHECKSUM_OFF,
			.align = {.burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
		},
		{
			.test_info = "Test in processor Cortex-A",
			.encpt_mode = FORCE_BUS_ENCPT_OFF,
			.encpt_pad = 0x00, /* or FORCE_BUS_ENCPT_NUL_KEY */
			.skb_wb_mode = SKB_WB_OFF,
			.tx_mode = FORCE_TX_CONTI_OFF,
			.checksuming = DEFAULT_CHECKSUM_OFF,
			.align = {.burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
		}},
};

/* Configuration access macros */
#define mconf (&confdata.mod[confdata.mid])
#define kconf (&confdata)
#define cint (confdata.interrupt)

/* Helper macros */
#define SCAN_BL(dw) (dw & GENMASK(7, 0))
#define SCAN_BH(dw) ((dw & GENMASK(15, 8)) >> 8)

/**
 * struct rx_ctl_mach - rx activities record
 * @status_err_counter: rx status error counter
 * @large_err_counter: rx get large packet length error counter
 * @rx_err_counter: receive packet error counter
 * @tx_err_counter: transmit packet error counter
 * @fifo_rst_counter: reset operation counter
 *
 * To keep track for the driver operation statistics
 */
struct rx_ctl_mach
{
	u16 status_err_counter;
	u16 large_err_counter;
	u16 rx_err_counter;
	u16 tx_err_counter;
	u16 fifo_rst_counter;

	u16 evaluate_rxb_counter;
	int nRxcF;
	u16 ndelayF;
};

/**
 * struct dm9051_rxctrl - dm9051 driver rx control
 * @hash_table: Multicast hash-table data
 * @rcr_all: KS_RXCR1 register setting
 * @encpt_setted_key: Encryption key from fixed code or efuse
 *
 * The settings needs to control the receive filtering
 * such as the multicast hash-filter and the receive register settings
 */
struct dm9051_rxctrl
{
	u16 hash_table[4];
	u8 rcr_all;
	u8 encpt_setted_key;
};

/**
 * struct dm9051_rxhdr - rx packet data header
 * @headbyte: lead byte equal to 0x01 notifies a valid packet
 * @status: status bits for the received packet
 * @rxlen: packet length
 *
 * The Rx packed, entered into the FIFO memory, start with these
 * four bytes which is the Rx header, followed by the ethernet
 * packet data and ends with an appended 4-byte CRC data.
 * Both Rx packet and CRC data are for check purpose and finally
 * are dropped by this driver
 */
struct dm9051_rxhdr
{
	u8 headbyte;
	u8 status;
	__le16 rxlen;
};

/**
 * struct board_info - maintain the saved data
 * @msg_enable: message level value
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @regmap_dm: regmap for register read/write
 * @regmap_dmbulk: extra regmap for bulk read/write
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @irq_workp: Work queue for polling mode
 * @pause: ethtool pause parameter structure
 * @spi_lockm: between threads lock structure
 * @reg_mutex: regmap access lock structure
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @rctl: rx control setting structure
 * @imr_all: to store operating imr value for register DM9051_IMR
 * @lcr_all: to store operating rcr value for register DM9051_LMCR
 */
struct board_info
{
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct regmap *regmap_dm;
	struct regmap *regmap_dmbulk;
	struct work_struct rxctrl_work;
	struct work_struct tx_work;
#if 1
	struct delayed_work irq_workp;
#endif
	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex reg_mutex;
	struct rx_ctl_mach bc;
	struct dm9051_rxhdr rxhdr;
	struct dm9051_rxctrl rctl;
	u8 imr_all;
	u8 lcr_all;
};

static int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb)
{
	int ret;

	ret = regmap_read(db->regmap_dm, reg, prb);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d get reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val)
{
	int ret;

	ret = regmap_write(db->regmap_dm, reg, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d set reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_update_bits(struct board_info *db, unsigned int reg, unsigned int mask,
							  unsigned int val)
{
	int ret;

	ret = regmap_update_bits(db->regmap_dm, reg, mask, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d update bits reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

static inline int dm9051_stop_mrcmd(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_ISR, ISR_STOP_MRCMD); /* to stop mrcmd */
}

/* skb buffer exhausted, just discard the received data
 */
static int dm9051_dumpblk(struct board_info *db, u8 reg, size_t count)
{
	struct net_device *ndev = db->ndev;
	unsigned int rb;
	int ret;

	/* no skb buffer,
	 * both reg and &rb must be noinc,
	 * read once one byte via regmap_read
	 */
	do
	{
		ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
		if (ret < 0)
		{
			netif_err(db, drv, ndev, "%s: error %d dumping read reg %02x\n",
					  __func__, ret, reg);
			break;
		}
	} while (--count);

	return ret;
}

static int dm9051_set_regs(struct board_info *db, unsigned int reg, const void *val,
						   size_t val_count)
{
	int ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk writing regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_get_regs(struct board_info *db, unsigned int reg, void *val,
						   size_t val_count)
{
	int ret;

	ret = regmap_bulk_read(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk reading regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff,
							size_t len)
{
	int ret;

	if (mconf->align.burst_mode)
	{ // tx
		ret = regmap_noinc_write(db->regmap_dm, reg, buff, len);
	}
	else
	{
		const u8 *p = (const u8 *)buff;
		size_t BLKTX = mconf->align.tx_blk;
		while (len >= BLKTX)
		{
			ret = regmap_noinc_write(db->regmap_dm, reg, p, BLKTX);
			p += BLKTX;
			len -= BLKTX;
			if (ret < 0)
			{
#if AARCH_OS_BITS
				netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x len %lu\n",
#else
				netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x len %u\n",
#endif
						  __func__, ret, reg, BLKTX);
				return ret;
			}
		}
		while (len--)
		{
			unsigned int val = (unsigned int)*p++;
			ret = regmap_write(db->regmap_dm, reg, val);
			if (ret < 0)
				break;
		}
	}
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen)
{
	if (db->rctl.encpt_setted_key &&
		mconf->encpt_mode != FORCE_BUS_ENCPT_OFF)
	{
		unsigned int j;
		for (j = 0; j < crlen; j++)
		{
			buff[j] ^= db->rctl.encpt_setted_key;
		}
	}

	return dm9051_write_mem(db, DM_SPI_MWCMD, buff, crlen); //'!wb'
}

static int dm9051_read_mem_rxb(struct board_info *db, unsigned int reg, void *buff,
							   size_t len)
{
	int ret;
	u8 *p = buff;
	unsigned int rb;

	while (len--)
	{
		ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
		*p++ = (u8)SCAN_BL(rb);
		if (ret < 0)
			break;
	}

	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
						   size_t len)
{
	int ret;

	if (mconf->align.burst_mode)
	{ // rx
		ret = regmap_noinc_read(db->regmap_dm, reg, buff, len);
	}
	else
	{
		u8 *p = buff;
		unsigned int rb;
		size_t BLKRX = mconf->align.rx_blk;
		while (len >= BLKRX)
		{
			ret = regmap_noinc_read(db->regmap_dm, reg, p, BLKRX);
			if (ret < 0)
			{
#if AARCH_OS_BITS
				netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x len %lu\n",
#else
				netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x len %u\n",
#endif
						  __func__, ret, reg, BLKRX);
				return ret;
			}
			p += BLKRX;
			len -= BLKRX;
		}

		while (len--)
		{
			ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
			*p++ = (u8)SCAN_BL(rb);
			if (ret < 0)
				break;
		}
	}
	if (ret < 0)
	{
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
				  __func__, ret, reg);
		return ret;
	}

	return dm9051_stop_mrcmd(db);
}

static int dm9051_read_mem_cache(struct board_info *db, unsigned int reg, u8 *buff,
								 size_t crlen)
{
	int ret = dm9051_read_mem(db, reg, buff, crlen);

	if (ret == 0 && db->rctl.encpt_setted_key &&
		mconf->encpt_mode != FORCE_BUS_ENCPT_OFF)
	{
		size_t j;
		for (j = 0; j < crlen; j++)
		{
			buff[j] ^= db->rctl.encpt_setted_key;
		}
	}
	return ret;
}

/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
void dm9051_create_tx_head(u8 *th, unsigned int len)
{
	th[0] = len & 0xff;
	; //;;todo
	th[1] = (len >> 8) & 0xff;
	th[2] = 0x00;
	th[3] = 0x80;
}

/* Get space of 3b max
 */
static unsigned int get_tx_free(struct board_info *db)
{
	int ret;
	unsigned int rb;

	ret = regmap_read(db->regmap_dm, DM9051_TXFSSR, &rb); // quick direct
	if (ret < 0)
	{
		netif_err(db, drv, db->ndev, "%s: error %d read reg %02x\n",
				  __func__, ret, DM9051_TXFSSR);
		return 0; // size now 'zero'
	}

	return (rb & 0xff) * 64;
}

static unsigned int tx_free_poll_timeout(struct board_info *db, unsigned int tx_tot,
										 u32 sleep_us, u64 timeout_us)
{
	unsigned int tx_free;
	for (;;)
	{
		tx_free = get_tx_free(db);
		if (tx_free >= tx_tot)
			return tx_tot;
		if (!sleep_us)
			break;
		if (timeout_us < sleep_us)
			break;
		timeout_us -= sleep_us;
		udelay(sleep_us);
	}
	printk("dm9051 tx -ENOMEM, req_size %u, free_size %u\n", tx_tot, tx_free);
	return 0;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
static int dm9051_nsr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
								   mval & (NSR_TX2END | NSR_TX1END),
								   1, econf->tx_timeout_us); // 2100 <- 20
	if (econf->force_monitor_tx_timeout && ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for tx end\n");
	return ret;
}

static int dm9051_epcr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_EPCR, mval,
								   !(mval & EPCR_ERRE), 100, 10000);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");
	return ret;
}

static int dm9051_ncr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
								   !(mval & NCR_RST), 10, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for ncr reset\n");
	return ret;
}

static int dm9051_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

static unsigned int dm9051_intcr_value(struct board_info *db)
{
	return (dm9051_irq_flag(db) == IRQF_TRIGGER_LOW) ? INTCR_POL_LOW : INTCR_POL_HIGH;
}

static int dm9051_set_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_set_reg(db, DM9051_FCR, fcr); // flow control / back pressure
}

static int dm9051_set_rcr(struct board_info *db)
{
	if (mconf->tx_mode == FORCE_TX_CONTI_ON)
	{
		/* or, be OK to put in dm9051_set_rcr()
		 */
		dm9051_set_reg(db, DM9051_ATCR, ATCR_TX_MODE2); /* tx continue on */
		return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all | RCR_DIS_WATCHDOG_TIMER);
	}
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all);
}

static int dm9051_set_recv(struct board_info *db)
{
	int ret;

	ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
	if (ret)
		return ret;

	return dm9051_set_rcr(db); /* enable rx */ // dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all);
}

static int dm9051_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_update_bits(db, DM9051_FCR, FCR_RXTX_BITS, fcr);
}

static int dm9051_disable_interrupt(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_IMR, IMR_PAR); /* disable int */
}

static int dm9051_enable_interrupt(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_IMR, db->imr_all); /* enable int */
}

// static int dm9051_stop_mrcmd(struct board_info *db)
//{
//	return dm9051_set_reg(db, DM9051_ISR, ISR_STOP_MRCMD); /* to stop mrcmd */
// }

static int dm9051_clear_interrupt(struct board_info *db)
{
	return dm9051_update_bits(db, DM9051_ISR, ISR_CLR_INT, ISR_CLR_INT);
}

static int dm9051_eeprom_read(struct board_info *db, int offset, u8 *to)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, to, 2);
}

static int dm9051_eeprom_write(struct board_info *db, int offset, u8 *data)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, data, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret)
		return ret;

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

static int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
	 * only fill lower 2 bytes
	 */
	*val = 0;
	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, val, 2);
}

static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, &val, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret)
		return ret;

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

// dm9051_phyread.EXTEND
static int dm9051_phyread_log_bmsr(struct board_info *db, char *head, int addr,
								   unsigned int reg, unsigned int *val)
{
	int ret = dm9051_phyread(db, reg, val);
	if (ret)
		return ret;

	/* check log */
	do
	{
		static unsigned int bmsr = 0xffff;
		if (*val != bmsr)
		{
			//printk("%s_read.bmsr.ne, phyaddr %d [BMSR] %04x\n", head, addr, *val);
			bmsr = *val;
		}
		else
		{
			if (!(*val & BIT(6)))
				printk("%s_read.bmsr.eq, phyaddr %d [BMSR] %04x\n", head, addr, *val);

			if (*val == 0xffff)
			{
				unsigned int vval;
				ret = dm9051_phyread(db, 2, &vval);
				if (ret)
					return ret;
				printk("%s_read.PHYID1[_check_], phyaddr %d phyreg(PHYID1) %04x\n", head, addr, vval);
			}
		}
	} while (0);

	return ret;
}

// dm9051_phyread.EXTEND
static int dm9051_phyread_log_reset(struct board_info *db,
									unsigned int reg)
{
	unsigned int val;
	int ret = dm9051_phyread(db, reg, &val);
	if (ret)
	{
		printk("_reset [dm9051_core_reset] dm9051_phyread(%u), ret = %d (ERROR)\n", reg, ret);
		return ret;
	}
	printk("_reset [dm9051_core_reset] dm9051_phyread(%u), %04x\n", reg, val);
	return ret;
}

// dm9051_phyread.EXTEND
static int dm9051_phyread_log_dscsr(struct board_info *db,
									unsigned int reg)
{
	unsigned int val;
	int ret = dm9051_phyread(db, reg, &val);
	if (ret)
	{
		printk("_reset [dm9051_core_reset] dm9051_phyread(dscsr), ret = %d (ERROR)\n", ret);
		return ret;
	}
	printk("_reset [dm9051_core_reset] dm9051_phyread(dscsr), %04x\n", val);

#if 1
	if ((val & 0x01f0) == 0x0010)
	{
		printk("_reset [dm9051_core_reset] PHY addr DOES 1\n");
	}
	else
	{
		printk("_reset [dm9051_core_reset] PHY addr NOT 1 ?\n");

		/* write */
		printk("_reset [dm9051_core_reset] PHY addr SETTO 1\n");
		ret = dm9051_phywrite(db, reg, 0xf210); // register 17
		if (ret)
			return ret;

		ret = dm9051_phyread(db, reg, &val);
		if (ret)
		{
			printk("_reset [dm9051_core_reset] dm9051_phyread-2(dscsr), ret = %d (ERROR)\n", ret);
			return ret;
		}
		printk("_reset [dm9051_core_reset] dm9051_phyread-2(dscsr), %04x\n", val);
		if ((val & 0x01f0) == 0x0010)
		{
			printk("_reset [dm9051_core_reset] PHY addr DOES 1 [workaround fixed]\n");
		}
		else
		{
			printk("_reset [dm9051_core_reset] PHY addr NOT 1 ? [second time]\n");
		}
	}
#else
//	if ((val & 0x1f) != 1) {
//		printk("_reset [dm9051_core_reset] PHY addr NOT 1 ?\n");

//		printk("_reset [dm9051_core_reset] PHY addr SETTO 1\n");
//		ret = dm9051_phywrite(db, reg, 0x0001); //register 17
//		if (ret)
//			return ret;

//		ret = dm9051_phyread(db, reg, &val);
//		if (ret) {
//			printk("_reset [dm9051_core_reset] dm9051_phyread-2(dscsr), ret = %d (ERROR)\n", ret);
//			return ret;
//		}
//		printk("_reset [dm9051_core_reset] dm9051_phyread-2(dscsr), %04x\n", val);
//		if ((val & 0x1f) != 1) {
//			printk("_reset [dm9051_core_reset] PHY addr NOT 1 ? [second time]\n");
//		}
//	}
#endif
	return ret;
}

static int dm9051_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	unsigned int val = 0xffff;
	int ret;

	if (addr == DM9051_PHY_ADDR)
	{
		if (regnum == 1)
			ret = dm9051_phyread_log_bmsr(db, "_mdio", addr, regnum, &val);
		else
			ret = dm9051_phyread(db, regnum, &val);
		if (ret)
			return ret;
	}

	return val;
}

static int dm9051_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;

	if (addr == DM9051_PHY_ADDR)
	{
		/* [dbg] mdio.wr BMCR */
		if (regnum == 0)
		{
			if (val & 0x800)
				printk("_[mii_bus] mdio write : power down (warn)\n");
		}
		return dm9051_phywrite(db, regnum, val);
	}

	return -ENODEV;
}

static int dm9051_ndo_set_features(struct net_device *ndev,
								   netdev_features_t features)
{
	// netdev_features_t changed = ndev->features ^ features;
	struct board_info *db = to_dm9051_board(ndev);

	if ((features & NETIF_F_RXCSUM) && (features & NETIF_F_HW_CSUM))
	{
		printk("_ops [Enabling TX/RX checksum]\n");
		dm9051_set_reg(db, 0x31, 0x7);
		dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_RXCSUM)
	{
		printk("_ops [Enabling RX checksum only]\n");
		dm9051_set_reg(db, 0x31, 0x0);
		dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_HW_CSUM)
	{
		printk("_ops [Enabling TX checksum only]\n");
		dm9051_set_reg(db, 0x31, 0x7);
		dm9051_set_reg(db, 0x32, 0x0);
	}
	else
	{
		// hardware_disable_rx_csum();
		printk("_ops [Disabling TX/RX checksum]\n");
		dm9051_set_reg(db, 0x31, 0x0);
		dm9051_set_reg(db, 0x32, 0x0);
	}

	return 0;
}

/* Function write:
 */
static void dm9051_write_crypt_word(struct board_info *db, u8 crypt_word)
{
	int ret;
	unsigned int crypt_1, crypt_2;
	ret = dm9051_get_reg(db, DM9051_PIDL, &crypt_1);
	if (ret)
		return;
	ret = dm9051_get_reg(db, DM9051_PIDH, &crypt_2);
	if (ret)
		return;
	ret = dm9051_set_reg(db, 0x49, crypt_1);
	if (ret)
		return;
	ret = dm9051_set_reg(db, 0x49, crypt_2);
	if (ret)
		return;
	ret = dm9051_set_reg(db, 0x49, crypt_word);
	if (ret)
		return;
	printk("[dm9051a_add_cryp-key] 0x%02x\n", crypt_word);
}

/* Function read:
 */
static u8 dm9051_read_crypt_word(struct board_info *db)
{
	int ret;
	unsigned int crypt_1, crypt_2, key;
	struct device *dev = &db->spidev->dev;
	ret = dm9051_get_reg(db, DM9051_PIDL, &crypt_1);
	if (ret)
		return 0;
	ret = dm9051_get_reg(db, DM9051_PIDH, &crypt_2);
	if (ret)
		return 0;
	ret = dm9051_set_reg(db, 0x49, crypt_1);
	if (ret)
		return 0;
	ret = dm9051_set_reg(db, 0x49, crypt_2);
	if (ret)
		return 0;
	ret = dm9051_get_reg(db, 0x49, &key);
	if (ret)
		return 0;
	dev_info(dev, "[Encrypt mode]= on, key 0x%02x\n", key & 0xff);
	return (u8)(key & 0xff);
}

static int dm9051_setup_crypt(struct board_info *db)
{
	if (mconf->encpt_mode == FORCE_BUS_ENCPT_FIX_ON)
	{
		dm9051_write_crypt_word(db, db->rctl.encpt_setted_key);
		db->rctl.encpt_setted_key = dm9051_read_crypt_word(db); // read to be sure!
	}
	else if (mconf->encpt_mode == FORCE_BUS_ENCPT_FAB_ON)
	{
		db->rctl.encpt_setted_key = dm9051_read_crypt_word(db);
	}

	return 0;
}

static int dm9051_core_reset(struct board_info *db)
{
	int ret;

	db->bc.fifo_rst_counter++;

	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;

	dm9051_ncr_poll(db);

	/* debug */
	do
	{
#if 1
		ret = dm9051_phyread_log_reset(db, 0);
		if (ret)
			return ret;
		ret = dm9051_phyread_log_reset(db, 3);
		if (ret)
			return ret;
		ret = dm9051_phyread_log_dscsr(db, 17);
		if (ret)
			return ret;
#else
		unsigned int val;
		ret = dm9051_phyread(db, 0, &val);
		if (ret)
		{
			printk("_reset [dm9051_core_reset] dm9051_phyread(0), ret = %d\n", ret);
			return ret;
		}
		printk("_reset [dm9051_core_reset] dm9051_phyread(0), %04x\n", val);
		ret = dm9051_phyread(db, 3, &val);
		if (ret)
		{
			printk("_reset [dm9051_core_reset] dm9051_phyread(3), ret = %d\n", ret);
			return ret;
		}
		printk("_reset [dm9051_core_reset] dm9051_phyread(3), %04x\n", val);
#endif

		ret = dm9051_phywrite(db, 0, 0x8000);
		if (ret)
			return ret;
		//printk("_reset [dm9051_core_reset] set phy MII_BMCR, %04x\n", 0x8000);

		/* dm9051 chip registers could not be accessed within 1 ms
		 * after GPR power on, delay 1 ms is essential
		 */
		msleep(1);

		// ret = dm9051_phywrite(db, 0, 0x3300);
		// if (ret)
		//	return ret;
		// printk("_reset [dm9051_core_reset] set phy MII_BMCR, %04x\n", 0x3300);
	} while (0);

	dm9051_setup_crypt(db);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, (mconf->skb_wb_mode) ? MBNDRY_WORD : MBNDRY_BYTE); /* MemBound */
	if (ret)
		return ret;
	// Spenser
	// ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_UNLIMIT); /* from PPCR_PAUSE_COUNT, Pause Count */
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, 0xF);
	if (ret)
		return ret;

	// Checksum Offload
	//.already. dm9051_ndo_set_features(db->ndev, db->ndev->features);

	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	/* Diagnostic contribute: In dm9051_enable_interrupt()
	 * (or located in the core reset subroutine is better!!)
	 */
	if (cint == MODE_INTERRUPT_CLKOUT)
	{
		printk("_reset [dm9051_core_reset] set DM9051_IPCOCR %02lx\n", IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
		ret = regmap_write(db->regmap_dm, DM9051_IPCOCR, IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
		if (ret)
			return ret;
	}

	return ret; /* return dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db)) */
}

static void dm9051_reg_lock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_lock(&db->reg_mutex);
}

static void dm9051_reg_unlock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_unlock(&db->reg_mutex);
}

static struct regmap_config regconfigdm = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
};

static struct regmap_config regconfigdmbulk = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
	.use_single_read = true,
	.use_single_write = true,
};

/* Work around for dm9051.c as dynamic load driver
 */
#define KERNEL_ROADMAP_CONF_H_
#ifdef KERNEL_ROADMAP_CONF_H_
#undef __devm_regmap_init_spi // compiler not undefined, since used in _wrapper

static int regmap_spi_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write(spi, data, count);
}
static int regmap_spi_gather_write(void *context,
								   const void *reg, size_t reg_len,
								   const void *val, size_t val_len)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = reg,
			.len = reg_len,
		},
		{
			.tx_buf = val,
			.len = val_len,
		},
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	return spi_sync(spi, &m);
}
static int regmap_spi_async_write(void *context, //.V510_COMPLEX
								  const void *reg, size_t reg_len,
								  const void *val, size_t val_len,
								  struct regmap_async *a)
{
	printk("NOT SUPPORT: regmap_spi_async_write(context,...)\n");
	return -1;
}
static struct regmap_async *regmap_spi_async_alloc(void)
{ //.V510_COMPLEX
	printk("NOT SUPPORT: regmap_spi_async_alloc(void)\n");
	return NULL;
}
static int regmap_spi_read(void *context,
						   const void *reg, size_t reg_size,
						   void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write_then_read(spi, reg, reg_size, val, val_size);
}

static const struct regmap_bus regmap_bus_dm = {
	// to refer to LXR
	.write = regmap_spi_write,
	.gather_write = regmap_spi_gather_write,
	.async_write = regmap_spi_async_write, //.V510_COMPLEX
	.async_alloc = regmap_spi_async_alloc, //.V510_COMPLEX
	.read = regmap_spi_read,
	.read_flag_mask = 0x80,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_bus *regmap_get_spi_bus(struct spi_device *spi,
												   const struct regmap_config *config)
{
	return &regmap_bus_dm;
}

struct regmap *__devm_regmap_init_spi_dm(struct spi_device *spi,
										 const struct regmap_config *config,
										 struct lock_class_key *lock_key,
										 const char *lock_name)
{
	// const struct regmap_bus *bus = &regmap_bus_dm;
	const struct regmap_bus *bus = regmap_get_spi_bus(spi, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus); /* . */

	return __devm_regmap_init(&spi->dev, bus, &spi->dev, config, lock_key, lock_name);
}

#undef devm_regmap_init_spi
#define devm_regmap_init_spi(dev, config)                        \
	__regmap_lockdep_wrapper(__devm_regmap_init_spi_dm, #config, \
							 dev, config)
#endif

static int dm9051_map_init(struct spi_device *spi, struct board_info *db)
{
	/* create two regmap instances,
	 * split read/write and bulk_read/bulk_write to individual regmap
	 * to resolve regmap execution confliction problem
	 */
	regconfigdm.lock_arg = db;
	db->regmap_dm = devm_regmap_init_spi(db->spidev, &regconfigdm);
	if (IS_ERR(db->regmap_dm))
		return PTR_ERR(db->regmap_dm);

	regconfigdmbulk.lock_arg = db;
	db->regmap_dmbulk = devm_regmap_init_spi(db->spidev, &regconfigdmbulk);
	return PTR_ERR_OR_ZERO(db->regmap_dmbulk);
}

static int dm9051_map_chipid(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	unsigned short wid;
	u8 buff[6];
	int ret;

	ret = dm9051_get_regs(db, DM9051_VIDL, buff, sizeof(buff));
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	if (wid != DM9051_ID)
	{
		dev_err(dev, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_info(dev, "chip %04x found\n", wid);
	// dev_info(dev, "Davicom: %s\n", mconf->test_info);
	dev_info(dev, "[TX mode]= %s mode\n",
			 (mconf->tx_mode == FORCE_TX_CONTI_ON) ? "continue" : "normal");
	dev_info(dev, "[Encrypt mode]= %s\n",
			 (mconf->encpt_mode != FORCE_BUS_ENCPT_OFF) ? "on" : "off"); // xxx
	// dev_info(dev, "SPI_XFER_MEM= %s\n", mconf->align.burst_mode ? "burst mode" : "alignment mode");
	// if (!mconf->align.burst_mode) { }
	// dev_info(dev, "Alignment TX: %lu\n", mconf->align.tx_blk);
	// dev_info(dev, "Alignment RX: %lu\n", mconf->align.rx_blk);
	dev_info(dev, "Check TX End: %llu\n", econf->tx_timeout_us);
	dev_info(dev, "DRVR= %s, %s\n",
			 econf->force_monitor_rxb ? "monitor rxb" : "silence rxb",
			 econf->force_monitor_tx_timeout ? "monitor tx_timeout" : "silence tx_ec");
	return 0;
}

/* Read DM9051_PAR registers which is the mac address loaded from EEPROM while power-on
 */
static int dm9051_map_etherdev_par(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	ret = dm9051_get_regs(db, DM9051_PAR, addr, sizeof(addr));
	if (ret < 0)
		return ret;

	dev_info(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 addr[0], addr[1], addr[2],
			 addr[3], addr[4], addr[5]);

	if (!is_valid_ether_addr(addr))
	{
		eth_hw_addr_random(ndev);

		/* [ndev->dev_addr[0] = 0x00; ndev->dev_addr[1] = 0x60; ndev->dev_addr[2] = 0x6e;] */
		// #if KERNEL_BUILD_CONF < DM9051_KERNEL_6_1
		//		ether_addr_copy(ndev->dev_addr, addr);
		// #else
		//		eth_hw_addr_set(ndev, addr);
		// #endif

		ether_addr_copy(addr, ndev->dev_addr);
		addr[0] = 0x00;
		addr[1] = 0x60;
		addr[2] = 0x6e;

		ret = dm9051_set_regs(db, DM9051_PAR, addr, sizeof(addr));
		if (ret < 0)
			return ret;

		dev_info(&db->spidev->dev, "Use random MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	}

#if KERNEL_BUILD_CONF < DM9051_KERNEL_6_1
	ether_addr_copy(ndev->dev_addr, addr);
#else
	eth_hw_addr_set(ndev, addr);
#endif
	return 0;
}

/* ethtool-ops
 */
static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRVNAME_9051, sizeof(info->driver));
}

static void dm9051_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->msg_enable = value;
}

static u32 dm9051_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	return db->msg_enable;
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
	return 128;
}

static int dm9051_get_eeprom(struct net_device *ndev,
							 struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2)
	{
		ret = dm9051_eeprom_read(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static int dm9051_set_eeprom(struct net_device *ndev,
							 struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2)
	{
		ret = dm9051_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static void dm9051_get_pauseparam(struct net_device *ndev,
								  struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	*pause = db->pause;
}

static int dm9051_set_pauseparam(struct net_device *ndev,
								 struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return dm9051_update_fcr(db);

	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
					  pause->autoneg);
	phy_start_aneg(db->phydev);
	return 0;
}

static void dm9051_get_strings(struct net_device *netdev, u32 sget, u8 *data)
{
	if (sget == ETH_SS_STATS)
		memcpy(data, dm9051_stats_strings, sizeof(dm9051_stats_strings));
}

static int dm9051_get_sset_count(struct net_device *netdev, int sset)
{
	return (sset == ETH_SS_STATS) ? ARRAY_SIZE(dm9051_stats_strings) : 0;
}

static void dm9051_get_ethtool_stats(struct net_device *ndev,
									 struct ethtool_stats *stats, u64 *data)
{
	struct board_info *db = to_dm9051_board(ndev);

	data[0] = ndev->stats.rx_packets;
	data[1] = ndev->stats.tx_packets;
	data[2] = ndev->stats.rx_errors = db->bc.rx_err_counter;
	data[3] = ndev->stats.tx_errors = db->bc.tx_err_counter;
	data[4] = ndev->stats.rx_bytes;
	data[5] = ndev->stats.tx_bytes;
	data[6] = db->bc.fifo_rst_counter - 1; // Subtract Initial reset
}

static const struct ethtool_ops dm9051_ethtool_ops = {
	.get_drvinfo = dm9051_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_msglevel = dm9051_get_msglevel,
	.set_msglevel = dm9051_set_msglevel,
	.nway_reset = phy_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = dm9051_get_eeprom_len,
	.get_eeprom = dm9051_get_eeprom,
	.set_eeprom = dm9051_set_eeprom,
	.get_pauseparam = dm9051_get_pauseparam,
	.set_pauseparam = dm9051_set_pauseparam,
	.get_strings = dm9051_get_strings,
	.get_sset_count = dm9051_get_sset_count,
	.get_ethtool_stats = dm9051_get_ethtool_stats,
};

static int dm9051_all_start(struct board_info *db)
{
	int ret;

	/* GPR power on of the internal phy
	 */
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	// printk("_open [dm9051_all_start] set reg DM9051_GPR, %x\n", 0);

	ret = dm9051_set_reg(db, DM9051_GPR, 0);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;

	// printk("Set dm9051_irq_flag() %d, _TRIGGER_LOW %d, _TRIGGER_HIGH %d (start)\n",
	//	   dm9051_irq_flag(db), IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);

	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
	if (ret)
		return ret;

	return dm9051_enable_interrupt(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

	/* GPR power off of the internal phy,
	 * The internal phy still could be accessed after this GPR power off control
	 */
	printk("_stop [dm9051_all_stop] set reg DM9051_GPCR, 0x%02x\n", (unsigned int)GPCR_GEP_CNTL);
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	printk("_stop [dm9051_all_stop] set reg DM9051_GPR, 0x%02x\n", (unsigned int)GPR_PHY_OFF);
	ret = dm9051_set_reg(db, DM9051_GPR, GPR_PHY_OFF);
	if (ret)
		return ret;

	ret = dm9051_phywrite(db, 0, 0x3900);
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_RCR, RCR_RX_DISABLE);
}

/* fifo reset while rx error found
 */
static int dm9051_all_restart(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret;

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;

	// printk("dm9.Set dm9051_irq_flag() %d, _TRIGGER_LOW %d, _TRIGGER_HIGH %d (restart)\n",
	//	   dm9051_irq_flag(db), IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
	printk("dm9.Set dm9051_irq_flag() %d (restart)\n",
		   dm9051_irq_flag(db));

	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
	if (ret)
		return ret;

	ret = dm9051_enable_interrupt(db);
	if (ret)
		return ret;

	printk("dm9.Show rxstatus_Er & rxlen_Er %d, RST_c %d\n",
		   db->bc.status_err_counter + db->bc.large_err_counter,
		   db->bc.fifo_rst_counter);
	netdev_dbg(ndev, " rxstatus_Er & rxlen_Er %d, RST_c %d\n",
			   db->bc.status_err_counter + db->bc.large_err_counter,
			   db->bc.fifo_rst_counter);

	ret = dm9051_set_recv(db);
	if (ret)
		return ret;

	return dm9051_set_fcr(db);
}

#define TIMES_TO_RST 10
#define DM9051_RX_BREAK(exp, yhndlr, nhndlr) \
	do                                       \
	{                                        \
		if ((exp))                           \
		{                                    \
			yhndlr;                          \
		}                                    \
		else                                 \
		{                                    \
			nhndlr;                          \
		}                                    \
	} while (0)

int trap_clr(struct board_info *db)
{
	db->bc.evaluate_rxb_counter = 0;
	return 0;
}

void monitor_rxb0(unsigned int rxbyte)
{
	if (econf->force_monitor_rxb)
	{
		static int rxbz_counter = 0;
		static unsigned int inval_rxb[TIMES_TO_RST] = {0};
		unsigned int i;
		int n = 0;
		char pbff[80];

		u8 *bf = (u8 *)&rxbyte; // tested

		if (SCAN_BL(rxbyte) == 1 && SCAN_BH(rxbyte) != 1)
			printk("-. ........ warn, spenser board ...BL %02lx BH %02lx.......... on .monitor_rxb0 %2d\n",
				   SCAN_BL(rxbyte), SCAN_BH(rxbyte), rxbz_counter);

		if (SCAN_BL(rxbyte) == 1 || SCAN_BL(rxbyte) == 0)
			return;

		printk("_.moni   bf] %02x %02x\n", bf[0], bf[1]);						// tested
		printk("_.moni rxbs] %02lx %02lx\n", SCAN_BL(rxbyte), SCAN_BH(rxbyte)); // tested

		inval_rxb[rxbz_counter] = SCAN_BL(rxbyte);
		rxbz_counter++;

		n += sprintf(pbff + n, "_.monitor_rxb0 %2d]", rxbz_counter);
		for (i = 0; i < rxbz_counter; i++)
		{
			if (i && !(i % 5))
				n += sprintf(pbff + n, " ");
			if (rxbz_counter > 5 && i < 5)
			{
				n += sprintf(pbff + n, "  .");
				continue;
			}
			n += sprintf(pbff + n, " %02x", inval_rxb[i]);
		}
		printk("%s\n", pbff);

		if (rxbz_counter >= TIMES_TO_RST)
		{
			rxbz_counter = 0;
			memset(inval_rxb, 0, sizeof(inval_rxb));
			printk("_[Less constrain of old SCAN_BL trap's, NOT dm9051_all_restart] only monitored.\n");
		}
	}
}

void monitor_rxc(struct board_info *db, int scanrr)
{
	if (econf->force_monitor_rxc && scanrr && db->bc.nRxcF < 25)
	{ // tx_mode
		db->bc.nRxcF += scanrr;
		if (mconf->align.burst_mode)
			printk("___[%s][rx/tx Burst mode] nRxc %d\n",
				   (mconf->tx_mode == FORCE_TX_CONTI_ON) ? "TX continue" : "TX normal mode",
				   db->bc.nRxcF);
		else
#if AARCH_OS_BITS
			printk("___[%s][Alignment RX %lu, Alignment TX %lu] nRxc %d\n",
				   (mconf->tx_mode == FORCE_TX_CONTI_ON) ? "TX continue" : "TX normal mode",
				   mconf->align.rx_blk,
				   mconf->align.tx_blk, db->bc.nRxcF);
#else
			printk("___[%s][Alignment RX %u, Alignment RX %u] nRxc %d\n",
				   (mconf->tx_mode == FORCE_TX_CONTI_ON) ? "TX continue" : "TX normal mode",
				   mconf->align.rx_blk,
				   mconf->align.tx_blk, db->bc.nRxcF);
#endif
	}
}

// check rxbs
// return: 0 : Still not trap
//         1 : Do reatart trap
int trap_rxb(struct board_info *db, unsigned int *prxbyte)
{
	if (SCAN_BH(*prxbyte) == 0)
		return 0;

	do
	{
		static unsigned int inval_rxb[TIMES_TO_RST] = {0};
		u8 *bf = (u8 *)prxbyte;
		int ret, n = 0;
		unsigned int i;
		char pbff[80];

		printk("_.eval   bf] %02x %02x\n", bf[0], bf[1]);
		printk("_.eval rxbs] %02lx %02lx\n", SCAN_BL(*prxbyte), SCAN_BH(*prxbyte));

		inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
		db->bc.evaluate_rxb_counter++;

		n += sprintf(pbff + n, "_[eval_rxb %2d]", db->bc.evaluate_rxb_counter);
		for (i = 0; i < db->bc.evaluate_rxb_counter; i++)
		{
			if (i && !(i % 5))
				n += sprintf(pbff + n, " ");
			if (db->bc.evaluate_rxb_counter > 5 && i < 5)
			{
				n += sprintf(pbff + n, "  .");
				continue;
			}
			n += sprintf(pbff + n, " %02x", inval_rxb[i]);
		}
		printk("%s\n", pbff);

		if (db->bc.evaluate_rxb_counter >= TIMES_TO_RST)
		{
			trap_clr(db);
			memset(inval_rxb, 0, sizeof(inval_rxb));
			ret = dm9051_all_restart(db); //...
			if (!ret)
				printk("_[dm9051_all_restart] work around done\n");
			return 1;
		}
	} while (0);
	return 0;
}

// int rx_ready(struct board_info *db)
//{
// }

int rx_break(unsigned int rxbyte, netdev_features_t features)
{
	monitor_rxb0(rxbyte);
	if (features & NETIF_F_RXCSUM)
	{
		//[ONly for if NOT discard checksum error packet, while REG32.D[0] is 0]
		// if (SCAN_BH(rxbyte) & 0xe0) {
		// if (SCAN_BH(rxbyte) & 0x20)
		//	printk("dm9.Monitor, on %d packet, IP_checksum error found\n", scanrr);
		// if (SCAN_BH(rxbyte) & 0x40)
		//	printk("dm9.Monitor, on %d packet, TCP_checksum error found\n", scanrr);
		// if (SCAN_BH(rxbyte) & 0x80)
		//	printk("dm9.Monitor, on %d packet, UDP_checksum error found\n", scanrr);
		// dm9051_dumpblk..
		//  continue
		//}
		DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), return 0, return -EINVAL);
	}
	else
		DM9051_RX_BREAK((SCAN_BH(rxbyte) == DM9051_PKT_RDY), return 0, return -EINVAL);
}

int rx_head_break(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int rxlen;

	rxlen = le16_to_cpu(db->rxhdr.rxlen);
	if (db->rxhdr.status & RSR_ERR_BITS || rxlen > DM9051_PKT_MAX)
	{
		printk("dm9.Monitor headbyte/status/rxlen %2x %2x %04x\n",
			   db->rxhdr.headbyte,
			   db->rxhdr.status,
			   db->rxhdr.rxlen);
		netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   db->rxhdr.headbyte);

		if (db->rxhdr.status & RSR_ERR_BITS)
		{
			db->bc.status_err_counter++;
			netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
		}
		else
		{
			db->bc.large_err_counter++;
			netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
		}
		return 1;
	}
	return 0;
}

/* read packets from the fifo memory
 * return value,
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data error, caller escape from rx operation
 */
static int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret, rxlen, padlen;
	unsigned int rxbyte;
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;

	do
	{
		ret = dm9051_read_mem_rxb(db, DM_SPI_MRCMDX, &rxbyte, 2);
		if (ret)
			return ret;

		if (rx_break(rxbyte, ndev->features))
		{
			if (trap_rxb(db, &rxbyte))
				return -EINVAL;
			break;
		}
		trap_clr(db);

#if 0
		monitor_rxb0(rxbyte);
		if (ndev->features & NETIF_F_RXCSUM) {
			//[ONly for if NOT discard checksum error packet, while REG32.D[0] is 0]
			//if (SCAN_BH(rxbyte) & 0xe0) {
				//if (SCAN_BH(rxbyte) & 0x20)
				//	printk("dm9.Monitor, on %d packet, IP_checksum error found\n", scanrr);
				//if (SCAN_BH(rxbyte) & 0x40)
				//	printk("dm9.Monitor, on %d packet, TCP_checksum error found\n", scanrr);
				//if (SCAN_BH(rxbyte) & 0x80)
				//	printk("dm9.Monitor, on %d packet, UDP_checksum error found\n", scanrr);
				//dm9051_dumpblk..
				//  continue
			//}
			DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), trap_clr(db), return !trap_rxb(db, &rxbyte) ? scanrr : -EINVAL);
		} else
			DM9051_RX_BREAK((SCAN_BH(rxbyte) == DM9051_PKT_RDY), trap_clr(db), return !trap_rxb(db, &rxbyte) ? scanrr : -EINVAL);

		//ret = rx_ready(db); [NOT Suitable with]
		//if (ret)
		//	return ret;
#endif

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		/* rx_head_takelen check */
		ret = rx_head_break(db);
		if (ret)
			return dm9051_all_restart(db);

#if 0
		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		if (db->rxhdr.status & RSR_ERR_BITS || rxlen > DM9051_PKT_MAX) {
			printk("dm9.Monitor headbyte/status/rxlen %2x %2x %04x\n", 
				db->rxhdr.headbyte,
				db->rxhdr.status,
				db->rxhdr.rxlen);
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   db->rxhdr.headbyte);

			if (db->rxhdr.status & RSR_ERR_BITS) {
				db->bc.status_err_counter++;
				netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
			} else {
				db->bc.large_err_counter++;
				netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
			}
			return dm9051_all_restart(db);
		}
#endif

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		padlen = (mconf->skb_wb_mode && (rxlen & 1)) ? rxlen + 1 : rxlen;
		skb = dev_alloc_skb(padlen);
		if (!skb)
		{
			ret = dm9051_dumpblk(db, DM_SPI_MRCMD, padlen);
			if (ret)
				return ret;
			break; //.return scanrr;
		}

		rdptr = skb_put(skb, rxlen - 4);
		ret = dm9051_read_mem_cache(db, DM_SPI_MRCMD, rdptr, padlen);
		if (ret)
		{
			db->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			return ret;
		}

		skb->protocol = eth_type_trans(skb, db->ndev);
		if (ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	} while (!ret);
	monitor_rxc(db, scanrr);

	return scanrr;
}

/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static int dm9051_conti_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	const unsigned int tx_xxhead = 4;
	unsigned int tx_xxbst = ((len + 3) / 4) * 4;
	u8 th[4];
	dm9051_create_tx_head(th, len);

	if (!tx_free_poll_timeout(db, tx_xxhead + tx_xxbst, 1, econf->tx_timeout_us))
	{					// 2100 <- 20
		return -ENOMEM; //-ETIMEDOUT or
	}

	ret = dm9051_write_mem(db, DM_SPI_MWCMD, th, 4);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, buff, tx_xxbst); //'tx_xxbst', DM_SPI_MWCMD
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_TCR, TCR_TXREQ);
}

static int dm9051_single_tx(struct board_info *db, u8 *buff, unsigned int buff_len, unsigned int len)
{
	int ret;

	ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, buff, buff_len); //'!wb'
	if (ret)
		return ret;

	ret = dm9051_set_regs(db, DM9051_TXPLL, &len, 2);
	if (ret < 0)
		return ret;

	return dm9051_set_reg(db, DM9051_TCR, TCR_TXREQ);
}

#ifdef DM9051_SKB_PROTECT
struct sk_buff *dm9051_skb_protect(struct sk_buff *skb)
{
	struct sk_buff *skb2;

	skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
	if (!skb2)
		printk("[WB_SUPPORT] which is on len %d, memory leak!\n", skb->len);
	dev_kfree_skb(skb);
	return skb2;
}
#endif

static int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;

	while (!skb_queue_empty(&db->txq))
	{
		struct sk_buff *skb = skb_dequeue(&db->txq);
		if (skb)
		{
			int ret;
			unsigned int len = skb->len;

			if (mconf->tx_mode == FORCE_TX_CONTI_ON)
			{
				ret = dm9051_conti_tx(db, skb->data, skb->len); //'double_wb'
			}
			else
			{
				unsigned int pad = 0; //'~wb'
				if (mconf->skb_wb_mode && (skb->len & 1))
				{
					#ifdef DM9051_SKB_PROTECT
					skb = dm9051_skb_protect(skb); /* protection */
					#endif
					pad = 1;
				}

				if (skb)
					ret = dm9051_single_tx(db, skb->data, len + pad, len); //'skb->len'
			}

			if (skb) {
				ntx++;
				dev_kfree_skb(skb);
				if (ret < 0)
				{
					db->bc.tx_err_counter++;
					return 0;
				}
				ndev->stats.tx_bytes += len;
				ndev->stats.tx_packets++;
			}
		}

		if (netif_queue_stopped(ndev) &&
			(skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

static irqreturn_t dm9051_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int result, result_tx;

	mutex_lock(&db->spi_lockm);

	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do
	{
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
	} while (result > 0);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}

static void dm9051_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int result;

	mutex_lock(&db->spi_lockm);

	result = dm9051_loop_tx(db);
	if (result < 0)
		netdev_err(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

static void dm9051_rxctl_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, rxctrl_work);
	struct net_device *ndev = db->ndev;
	int result;

	mutex_lock(&db->spi_lockm);

	result = dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
	if (result < 0)
		goto out_unlock;

	dm9051_set_recv(db);

	/* To has mutex unlock and return from this function if regmap function fail
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);
}

/* [DM_TIMER_EXPIRE2] poll extream.fast */
/* [DM_TIMER_EXPIRE1] consider not be 0, to alower and not occupy almost all CPU resource.
 * This is by CPU scheduling-poll, so is software driven!
 */
#define DM_TIMER_EXPIRE1 1
#define DM_TIMER_EXPIRE2 0
#define DM_TIMER_EXPIRE3 0

/* schedule delay work */
static void dm9051_irq_delayp(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_workp);

	dm9051_rx_threaded_irq(0, db); // 0 is no-used.

	if (db->bc.ndelayF >= csched.nTargetMaxNum)
		db->bc.ndelayF = POLL_OPERATE_INIT;

	/* redundent, but for safe */
	if (!cint)
		// schedule_delayed_work(&db->irq_workp, 0);
		schedule_delayed_work(&db->irq_workp, csched.delayF[db->bc.ndelayF++]);
}

/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;

	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */

	/* interrupt work */
	if (cint)
	{
		ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
								   dm9051_irq_flag(db) | IRQF_ONESHOT,
								   ndev->name, db);
		if (ret < 0)
		{
			netdev_err(ndev, "failed to get irq\n");
			return ret;
		}
	}

#if 0
	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;
#endif

	ret = dm9051_all_start(db);
	if (ret)
	{
		phy_stop(db->phydev);
		if (cint)
			free_irq(spi->irq, db);
		return ret;
	}

#if 1
	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;
#endif

	netif_wake_queue(ndev);

	/* schedule delay work */
	if (!cint)
		schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = dm9051_all_stop(db);
	if (ret)
		return ret;

	/* schedule delay work */
	if (!cint)
		cancel_delayed_work_sync(&db->irq_workp);

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

	/* when (threadedcfg.interrupt_supp == THREADED_INT) */
	if (cint)
	{
		free_irq(db->spidev->irq, db);
		printk("_stop [free irq %d]\n", db->spidev->irq);
	}

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	schedule_work(&db->tx_work);

	return NETDEV_TX_OK;
}

/* event: play with a schedule starter
 */
static void dm9051_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct dm9051_rxctrl rxctrl;
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;

	memset(&rxctrl, 0, sizeof(rxctrl));

	/* rx control */
	if (ndev->flags & IFF_PROMISC)
	{
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI)
	{
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	rxctrl.rcr_all = rcr;

	/* broadcast address */
	rxctrl.hash_table[0] = 0;
	rxctrl.hash_table[1] = 0;
	rxctrl.hash_table[2] = 0;
	rxctrl.hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev)
	{
		hash_val = ether_crc_le(ETH_ALEN, ha->addr) & GENMASK(5, 0);
		rxctrl.hash_table[hash_val / 16] |= BIT(0) << (hash_val % 16);
	}

	/* schedule work to do the actual set of the data if needed */
	if (memcmp(&db->rctl, &rxctrl, sizeof(rxctrl)))
	{
		memcpy(&db->rctl, &rxctrl, sizeof(rxctrl));
		schedule_work(&db->rxctrl_work);
	}
}

/* event: write into the mac registers and eeprom directly
 */
static int dm9051_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = eth_prepare_mac_addr_change(ndev, p);
	if (ret < 0)
		return ret;

	eth_commit_mac_addr_change(ndev, p);
	return dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
}

static struct net_device_stats *dm9051_get_stats(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	ndev->stats.tx_errors = db->bc.tx_err_counter;
	ndev->stats.rx_errors = db->bc.rx_err_counter;
	return &ndev->stats;
}

static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
	.ndo_set_features = dm9051_ndo_set_features,
	.ndo_get_stats = dm9051_get_stats,
};

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;

	trap_clr(db);
	db->bc.nRxcF = 0;
	db->bc.ndelayF = POLL_OPERATE_INIT;
}

static int dm9051_mdio_register(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = devm_mdiobus_alloc(&spi->dev);
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = dm9051_mdio_read;
	db->mdiobus->write = dm9051_mdio_write;
	db->mdiobus->name = "dm9051-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
			 "dm9051-%s.%u", dev_name(&spi->dev), spi->chip_select);

	ret = devm_mdiobus_register(&spi->dev, db->mdiobus);
	if (ret)
		dev_err(&spi->dev, "Could not register MDIO bus\n");

	return ret;
}

static void dm9051_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	phy_print_status(db->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link)
	{
		if (db->phydev->pause)
		{
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
		dm9051_update_fcr(db);
	}
}

/* phy connect as poll mode
 */
static int dm9051_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
			 db->mdiobus->id, DM9051_PHY_ADDR);

	db->phydev = phy_connect(db->ndev, phy_id, dm9051_handle_link_change,
							 PHY_INTERFACE_MODE_MII);
	return PTR_ERR_OR_ZERO(db->phydev);
}

static int dm9051_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	int ret;

	ndev = devm_alloc_etherdev(dev, sizeof(struct board_info));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	db = netdev_priv(ndev);

	db->msg_enable = 0;
	db->spidev = spi;
	db->ndev = ndev;

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;

	// Spenser - Setup for Checksum Offload
	/* Set default features */
	if (mconf->checksuming)
	{
		ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
		ndev->hw_features |= ndev->features;
	}

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

	if (!cint)
		/* schedule delay work */
		INIT_DELAYED_WORK(&db->irq_workp, dm9051_irq_delayp);

	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	printk("\n");
	dev_info(dev, "Davicom: %s", kconf->release_version); // kconf->driver_release_version_candidate //LNX_DM9051_RELEASE_VERSION

	/* [dbg] spi.speed */
	do
	{
		unsigned int speed;
		of_property_read_u32(spi->dev.of_node, "spi-max-frequency", &speed);
		dev_info(dev, "SPI speed from DTS: %d Hz\n", speed);

		if (cint)
		{
			unsigned int intdata[2];
			of_property_read_u32_array(spi->dev.of_node, "interrupts", &intdata[0], 2);
			dev_info(dev, "Operation: Interrupt mode/ %s\n", (cint == MODE_INTERRUPT_CLKOUT) ? "CLKOUT" : "REG39H");
			dev_info(dev, "Operation: Interrupt pin: %d\n", intdata[0]); // intpin
			dev_info(dev, "Operation: Interrupt trig type: %d\n", intdata[1]);
		}
		else
		{
			int i;
			dev_info(dev, "Operation: Polling mode\n"); //~intpin
			dev_info(dev, "Operation: Polling operate count %d\n", csched.nTargetMaxNum);
			for (i = 0; i < csched.nTargetMaxNum; i++)
			{
				dev_info(dev, "Operation: Polling operate delayF[%d]= %lu\n", i, csched.delayF[i]);
			}
		}
	} while (0);
	printk("\n");
	dev_info(dev, "Davicom: %s", mconf->test_info);
	dev_info(dev, "LXR: %s, BUILD: %s\n", linux_name[LXR_REF_CONF], linux_name[KERNEL_BUILD_CONF]);
	dev_info(dev, "SPI_XFER_MEM= %s\n", mconf->align.burst_mode ? "burst mode" : "alignment mode");
	dev_info(dev, "Alignment TX: %lu\n", mconf->align.tx_blk);
	dev_info(dev, "Alignment RX: %lu\n", mconf->align.rx_blk);

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

	ret = dm9051_map_etherdev_par(ndev, db);
	if (ret < 0)
		return ret;

	ret = dm9051_mdio_register(db);
	if (ret)
		return ret;

	ret = dm9051_phy_connect(db);
	if (ret)
		return ret;

	dm9051_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = devm_register_netdev(dev, ndev);
	if (ret)
	{
		phy_disconnect(db->phydev);
		return dev_err_probe(dev, ret, "device register failed");
	}

	return 0;
}

#if KERNEL_BUILD_CONF <= DM9051_KERNEL_5_10
static int dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	phy_disconnect(db->phydev);
	return 0;
}
#else
static void dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	phy_disconnect(db->phydev);
}
#endif

static const struct of_device_id dm9051_match_table[] = {
	{.compatible = "davicom,dm9051"},
	{}};

static const struct spi_device_id dm9051_id_table[] = {
	{"dm9051", 0},
	{}};

static struct spi_driver dm9051_driver = {
	.driver = {
		.name = DRVNAME_9051,
		.of_match_table = dm9051_match_table,
	},
	.probe = dm9051_probe,
	.remove = dm9051_drv_remove,
	.id_table = dm9051_id_table,
};
module_spi_driver(dm9051_driver);

MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_DESCRIPTION("Davicom DM9051 network SPI driver");
MODULE_LICENSE("GPL");
