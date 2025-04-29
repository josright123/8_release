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
#include <linux/utsname.h>
#include <generated/utsrelease.h> // For newer kernels
//#include <linux/utsrelease.h>      // For older kernels
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>
#define MAIN_DATA
#include <linux/ptp_clock_kernel.h>
#include "dm9051.h"
#include "dm9051_plug.h"
#include "dm9051_ptpd.h"

//#define KERNEL_BUILD_CONF	DM9051_KERNEL_6_6
const struct mod_config *dm9051_modedata = &driver_align_mode; /* Driver configuration */

/* tX 'wb' do skb protect */
#define DM9051_SKB_PROTECT

/* log */
#ifdef DMCONF_AARCH_64
#define PRINT_ALIGN_INFO(n) \
			printk("___[TX %s mode][Alignment RX %lu, Alignment TX %lu] nRxc %d\n", \
				   dmplug_tx, \
				   dm9051_modedata->align.rx_blk, \
				   dm9051_modedata->align.tx_blk, \
				   n)
#else
#define PRINT_ALIGN_INFO(n) \
			printk("___[TX %s mode][Alignment RX %u, Alignment RX %u] nRxc %d\n", \
				   dmplug_tx, \
				   dm9051_modedata->align.rx_blk, \
				   dm9051_modedata->align.tx_blk, \
				   n)
#endif

/* Helper macros */
#define SCAN_BL(dw) (dw & GENMASK(7, 0))
#define SCAN_BH(dw) ((dw & GENMASK(15, 8)) >> 8)

#if 0 //sticked fixed here is better!
struct rx_ctl_mach
{
};

struct dm9051_rxctrl
{
};

struct dm9051_rxhdr
{
};

struct board_info
{
};
#endif

/*
 * Info: 
 */

int get_dts_irqf(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

//const static char *linux_name[] = {
//        "rsrv",
//        "rsrv",
//        "rsrv",
//        "rsrv",
//        "rsrv",
//        "DM9051_KERNEL_5_10",
//        "DM9051_KERNEL_5_15",
//        "DM9051_KERNEL_6_1",
//        "DM9051_KERNEL_6_6",
//        "UNKNOW",
//};

static void SHOW_CONFIG_MODE(struct spi_device *spi)
{
	struct device *dev = &spi->dev;

	/* [dbg] spi.speed */
//	dev_info(dev, "Linux %s DM9051A\n", utsname()->release);
	do
	{
		unsigned int speed;
		of_property_read_u32(spi->dev.of_node, "spi-max-frequency", &speed);
		dev_info(dev, "SPI speed from DTS: %d Hz\n", speed);
		SHOW_MODE(spi);
	} while (0);
	printk("\n");
	dev_info(dev, "Davicom: %s", driver_align_mode.test_info);
	dev_info(dev, "LXR: %s, BUILD: %s\n", utsname()->release, utsname()->release); //dev_info(dev, "LXR: %s, BUILD: %s\n", linux_name[LXR_REF_CONF], linux_name[KERNEL_BUILD_CONF]); /* Driver configuration test_info */
	//dev_info(dev, "Kernel Version (compile-time): %s\n", UTS_RELEASE);
#ifdef DMCONF_AARCH_64
	dev_info(dev, "TX: %s blk %lu\n", dm9051_modedata->align.burst_mode_info, dm9051_modedata->align.tx_blk);
	dev_info(dev, "RX: %s blk %lu\n", dm9051_modedata->align.burst_mode_info, dm9051_modedata->align.rx_blk);
#else
	dev_info(dev, "TX: %s blk %u\n", dm9051_modedata->align.burst_mode_info, dm9051_modedata->align.tx_blk);
	dev_info(dev, "RX: %s blk %u\n", dm9051_modedata->align.burst_mode_info, dm9051_modedata->align.rx_blk);
#endif
}

static void SHOW_OPTION_MODE(struct spi_device *spi)
{
	struct device *dev = &spi->dev;

	dev_info(dev, "Check TX End: %llu, TX mode= %s mode, DRVR= %s, %s\n", econf->tx_timeout_us, dmplug_tx,
			econf->force_monitor_rxb ? "monitor rxb" : "silence rxb",
			econf->force_monitor_tx_timeout ? "monitor tx_timeout" : "silence tx_ec");
}

static void SHOW_MONITOR_RXC(struct board_info *db)
{
	if (dm9051_modedata->align.burst_mode == BURST_MODE_FULL)
		printk("___[rx/tx %s mode] nRxc %d\n",
			   dmplug_tx,
			   db->bc.nRxcF);
	else if (dm9051_modedata->align.burst_mode == BURST_MODE_ALIGN)
		PRINT_ALIGN_INFO(db->bc.nRxcF);
}

int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb)
{
	int ret;

//lockdep_assert_held(&db->spi_lockm); --to be used! --20250411
	ret = regmap_read(db->regmap_dm, reg, prb);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d get reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val)
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

int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff,
			size_t len)
{
	int ret;

	if (dm9051_modedata->align.burst_mode)
	{ // tx
		ret = regmap_noinc_write(db->regmap_dm, reg, buff, len);
	}
	else
	{
		const u8 *p = (const u8 *)buff;
		size_t BLKTX = dm9051_modedata->align.tx_blk;
		while (len >= BLKTX)
		{
			ret = regmap_noinc_write(db->regmap_dm, reg, p, BLKTX);
			p += BLKTX;
			len -= BLKTX;
			if (ret < 0)
			{
				PRINT_REGMAP_BLK_ERR("writing", ret, reg, BLKTX);
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

int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen)
{
	BUS_OPS(db, buff, crlen);
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

int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
						   size_t len)
{
	int ret;

	if (dm9051_modedata->align.burst_mode)
	{ // rx
		ret = regmap_noinc_read(db->regmap_dm, reg, buff, len);
	}
	else
	{
		u8 *p = buff;
		unsigned int rb;
		size_t BLKRX = dm9051_modedata->align.rx_blk;
		while (len >= BLKRX)
		{
			ret = regmap_noinc_read(db->regmap_dm, reg, p, BLKRX);
			if (ret < 0)
			{
				PRINT_REGMAP_BLK_ERR("reading", ret, reg, BLKRX);
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
	if (ret == 0)
		BUS_OPS(db, buff, crlen);
	return ret;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
#ifndef DMPLUG_CONTI
/*static*/ int dm9051_nsr_poll(struct board_info *db)
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
#endif

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
#ifdef DMPLUG_CONTI
	return TX_SET_CONTI(db);
#else
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all);
#endif
}

static int dm9051_set_recv(struct board_info *db)
{
	int ret;

	ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
	if (ret)
		return ret;

	return dm9051_set_rcr(db); /* enable rx */
}

static int dm9051_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		//fcr |= FCR_BKPA | FCR_FLCE; (optional)
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

//.inline dm9051_stop_mrcmd
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
	if (ret) {
		printk("timeout of dm9051_eeprom_write %d %04x\n", offset, to[0] | to[1] << 8);
		return ret;
	}

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
	if (ret) {
		printk("timeout of dm9051_eeprom_write %d %04x\n", offset, data[0] | data[1] << 8);
		return ret;
	}

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
	if (ret) {
		printk("timeout of dm9051_phyrd %d %04x\n", reg, *val);
		return ret;
	}

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
	if (ret) {
		printk("timeout of dm9051_phywrite %d %04x\n", reg, val);
		return ret;
	}

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

// dm9051_phyread.EXTEND
//static int dm9051_phyread_log_reset(struct board_info *db,
//									unsigned int reg)
//{
//	unsigned int val;
//	int ret = dm9051_phyread(db, reg, &val);
//	if (ret)
//	{
//		printk("_reset [_core_reset] dm9051_phyrd(%u), ret = %d (ERROR)\n", reg, ret);
//		return ret;
//	}
//	printk("_reset [_core_reset] dm9051_phyrd(%u), %04x\n", reg, val);
//	return ret;
//}

// dm9051_phyread.EXTEND
//static int dm9051_phyread_log_dscsr(struct board_info *db,
//									unsigned int reg)
//{
//	unsigned int val;
//	int ret = dm9051_phyread(db, reg, &val);
//	if (ret)
//	{
//		printk("_reset [_core_reset] dm9051_phyrd(dscsr), ret = %d (ERROR)\n", ret);
//		return ret;
//	}
//	printk("_reset [_core_reset] dm9051_phyrd(dscsr), %04x\n", val);

//	if ((val & 0x01f0) == 0x0010)
//	{
//		printk("_reset [_core_reset] PHY addr DOES 1\n");
//	}
//	else
//	{
//		printk("_reset [_core_reset] PHY addr NOT 1 ?\n");

//		/* write */
//		printk("_reset [_core_reset] PHY addr SETTO 1\n");
//		ret = dm9051_phywrite(db, reg, 0xf210); // register 17
//		if (ret)
//			return ret;

//		ret = dm9051_phyread(db, reg, &val);
//		if (ret)
//		{
//			printk("_reset [_core_reset] dm9051_phyrd-2(dscsr), ret = %d (ERROR)\n", ret);
//			return ret;
//		}
//		printk("_reset [_core_reset] dm9051_phyrd-2(dscsr), %04x\n", val);
//		if ((val & 0x01f0) == 0x0010)
//		{
//			printk("_reset [_core_reset] PHY addr DOES 1 [workaround fixed]\n");
//		}
//		else
//		{
//			printk("_reset [_core_reset] PHY addr NOT 1 ? [second time]\n");
//		}
//	}

//	return ret;
//}

//static int PHY_LOG(struct board_info *db)
//{
//	int ret = dm9051_phyread_log_reset(db, 0);
//	if (ret)
//		return ret;
//	ret = dm9051_phyread_log_reset(db, 3);
//	if (ret)
//		return ret;
//	ret = dm9051_phyread_log_dscsr(db, 17);
//	if (ret)
//		return ret;

//	return 0;
//}

static int PHY_RST(struct board_info *db)
{
	int ret = dm9051_phywrite(db, 0, 0x8000);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	/* Jabber function disabled refer to bench test
	 * meeting advice 20250226
	 */
	//ret = dm9051_phywrite(db, 18, 0x7000);
	//if (ret)
	//	return ret;
	return 0;
}

#ifdef DMCONF_BMCR_WR
static int dm9051_phy_reset(struct board_info *db);

/*void set_log_addr(struct board_info *db, char *p)
  {
  }*/
char *get_log_addr(struct board_info *db)
{
	if (!db->automdix_log[0][0]) {
		//db->automdix_log[0][0] = 1;
 //printk("[get log addr 0]\n");
		return &db->automdix_log[0][0]; //&db->automdix_log[0][1];
	}
	if (!db->automdix_log[1][0]) {
		//db->automdix_log[1][0] = 1;
 //printk("[get log addr 1]\n");
		return &db->automdix_log[1][0]; //&db->automdix_log[1][1];
	}
	if (!db->automdix_log[2][0]) {
		//db->automdix_log[2][0] = 1;
 //printk("[get log addr 2]\n");
		return &db->automdix_log[2][0]; //&db->automdix_log[2][1];
	}

 //printk("[get log addr 2ov]\n");
	db->automdix_log[0][0] = db->automdix_log[1][0];
	strcpy(&db->automdix_log[0][1], &db->automdix_log[1][1]);
	db->automdix_log[1][0] = db->automdix_log[2][0];
	strcpy(&db->automdix_log[1][1], &db->automdix_log[2][1]);
	
	return &db->automdix_log[2][0];
	//int i;
	//for () {
	//}
}
static void show_log_data(char *head, char *data)
{
		printk("<%s> %s\n", head, &data[1]);
}
static void show_log_addr(char *head, struct board_info *db)
{
	if (db->automdix_log[0][0]) {
		printk("\n");
		show_log_data(head, &db->automdix_log[0][0]); //printk("<%s> %s\n", head, &db->automdix_log[0][1]);
	}
	if (db->automdix_log[1][0])
		show_log_data(head, &db->automdix_log[1][0]); //printk("<%s> %s\n", head, &db->automdix_log[1][1]);
	if (db->automdix_log[2][0])
		show_log_data(head, &db->automdix_log[2][0]); //printk("<%s> %s\n", head, &db->automdix_log[2][1]);
}
void amdix_log_reset(struct board_info *db)
{
	db->n_automdix = 0; //log-reset
	db->automdix_log[0][0] = 0;
	db->automdix_log[1][0] = 0;
	db->automdix_log[2][0] = 0;

	db->stop_automdix_flag = 0;
}
#if 0
void amdix_link_change_up(struct board_info *db, unsigned int bmsr)
{
	if (!(bmsr & BIT(2)) && (db->bmsr & BIT(2))) {
		//[show]
		show_log_addr("hist", db);
		//[message]
		printk("<link_phylib. on %02u to %02u>, current lpa %04x [bmsr] %04x to %04x found reach link\n", db->_stop_automdix_flag,
			db->n_automdix, db->lpa, bmsr, db->bmsr);
		//[clear]
		printk("[link] clear log...");
		amdix_log_reset(db);
	}
}
#endif

void amdix_bmsr_change_up(struct board_info *db, unsigned int bmsr)
{
	if (!(bmsr & BIT(2))) {
		show_log_addr("hist", db);
		printk("<link_phylib. on %02u to %02u>, current lpa %04x [bmsr] %04x to %04x found reach link\n", db->stop_automdix_flag,
			db->n_automdix, db->lpa, bmsr, db->bmsr);
		printk("[link] clear log...");
		amdix_log_reset(db);
	}
}

static int amdix_bmsr_change(struct board_info *db)
{
	static unsigned int bmsr = 0x0000; //0xffff;

#if 1
	/* bmsr (change linkup) */
	if (db->bmsr & BIT(2)) {
		amdix_bmsr_change_up(db, bmsr);
	}
#endif

	/* bmsr (change) */
	if (db->bmsr != bmsr) {		
		if (db->bmsr & BIT(2))
			;
		else {
			//printk("??? [NO write obsevation]\n");
			/* NO write obsevation */
			//printk("??? <may up to down> %02u _First quicking an enter amdix ... lpa %04x bmsr from %04x to %04x\n",
			//		db->n_automdix, db->lpa, bmsr, db->bmsr);

#if 0
			/* NO write obsevation */
			/* ------------------- */
			int ret;
			db->mdi ^= 0x0020;
			printk("??? <may up to down> %02u [lpa] %04x _dm9051_phywr[_AutoMDIX_] reg %d [val %04x]",
					db->n_automdix, db->lpa, 20, db->mdi); 
			
			ret = dm9051_phywrite(db, 20, db->mdi);
			if (ret)
				printk("warn: phywrite reg %d [val %04x], fail\n", 20, db->mdi);
#endif
		}

		bmsr = db->bmsr;
		return 1;
	}
	return 0;
}

static int dm9051_phyread_nt_bmsr(struct board_info *db,
								   unsigned int *val)
{
	int ret;

	ret = dm9051_phyread(db, MII_LPA, val);
	if (ret)
		return ret;
	db->lpa = *val;

	ret = dm9051_phyread(db, MII_BMSR, val);
	if (ret)
		return ret;
	db->bmsr = *val;

	/* nt log */
#if 1
	do
	{
		if (!amdix_bmsr_change(db))
		{
			if (!(db->bmsr & BIT(2))) {

				do {
					if (db->stop_automdix_flag) {
					#if 1
						/* lpa changeto 0x0000 */
						if (!db->lpa) {
							db->stop_automdix_flag = 0;
							break;
						}
					#endif
						if (!(db->n_automdix %10)) {
							printk("%2u [lpa %04x]\n", db->n_automdix, db->lpa);
							break;
						}
						return 0;
					}
					if (db->lpa) {
						printk("<fund_phylib. on %02u to %02u, rd.bmsr %04x [lpa] %04x> STOPPING... amdix\n", db->stop_automdix_flag,
							db->n_automdix, db->bmsr, db->lpa);
						db->stop_automdix_flag = db->n_automdix;
						//printk("(STOP avoid below possible more once toggle...)\n");
						return 0; //break; //(STOP avoid below possible more once toggle...)
					}
				} while(0);

				db->n_automdix++;
				do {
			#if 1
						char *p = get_log_addr(db);
					db->mdi ^= 0x0020;

					do {
						/* store list */
						//sprintf(p, "from_phylib. %02u [lpa] %04x _mon_phywr[_AutoMDIX_] reg %d [val %04x]",
						//		db->n_automdix, db->lpa, 20, db->mdi);
						//snprintf(p, sizeof(automdix_log[0]), "from_phylib. %02u [lpa] %04x _mon_phywr[_AutoMDIX_] reg %d [val %04x]",
						//		db->n_automdix, db->lpa, 20, db->mdi);
						p[0] = db->n_automdix;
						memset(&p[1], 0, AMDIX_LOG_BUFSIZE);
						snprintf(&p[1], AMDIX_LOG_BUFSIZE-1, "in_case phylib. %02u [lpa] %04x phy[AutoMDIX] reg %d [val %04x]",
								db->n_automdix, db->lpa, 20, db->mdi);
					} while(0);

					/* NO write obsevation */
					//ret = dm9051_phywrite(db, 20, db->mdi);
					//if (ret)
					//	return ret;
			#endif

					if ((db->n_automdix % NUM_TRIGGER) <= NUM_BMSR_DOWN_SHOW) {
						if ((db->n_automdix % NUM_TRIGGER) == 1) //only first.
							printk("\n"); //printk("(re-cycle)(first).mdix\n");
//						show_log_data("bmsr down", p);
					}

			#if 1
					/* phy reset insteaded */
					if (!(db->n_automdix % NUM_TRIGGER)) {
						printk("( bmsr down per %d).phy reset insteaded: %u\n", NUM_TRIGGER, db->n_automdix);
						ret = dm9051_phy_reset(db);
						if (ret)
							return ret;
					}
			#endif
				} while(0);
			}
		}
	} while (0);
#endif

	return ret;			
}

// dm9051_phyread.EXTEND
#if 0
//static 
int dm9051_phyread_log_bmsr(struct board_info *db,
								   unsigned int *val)
{
	int ret;

	ret = dm9051_phyread(db, MII_LPA, val);
	if (ret)
		return ret;
	db->lpa = *val;

	ret = dm9051_phyread(db, MII_BMSR, val);
	if (ret)
		return ret;
	db->bmsr = *val;

	/* check log */
	do
	{
		if (!amdix_link_change(db))
		{
			if (!(db->bmsr & BIT(2))) {
				//static unsigned int n_automdix = 0;
				//static unsigned int mdi = 0x0830;
				db->n_automdix++;
				
				if (db->_stop_automdix_flag) {
#if 1
					printk("[lpa %04x]\n", db->lpa);
					break;
#else
					printk("[lpa %04x] Go ..\n", db->lpa);
					/* Too many Go .., 
					 * Want rst something ...
					 */
#endif
				}

				//ret = dm9051_phyread(db, 5, &vval);
				//if (ret)
				//	return ret;
				
				if (db->lpa) {
					printk("<fund_phylib. on %02u to %02u, _mdio_read.bmsr[lpa] %04x> STOPPING... automdix\n", db->_stop_automdix_flag,
						db->n_automdix, db->lpa);
					db->_stop_automdix_flag = db->n_automdix;
#if 1
					printk("(STOP avoid below possible more once toggle...)\n");
					break; //(STOP avoid below possible more once toggle...)
#endif
					//break; //(NOT STOP
				}

				if (!(db->n_automdix % TOGG_INTVL)) {
					char *p;
					db->mdi ^= 0x0020;

					if (db->n_automdix <= TOGG_TOT_SHOW) {
						if (db->n_automdix == TOGG_INTVL) //only first.
							printk("\n");
					}
					if (db->n_automdix <= TOGG_TOT_SHOW 
						&& !(db->bmsr & BIT(6))) printk("_mdio_read.bmsr.BIT6= 0, !MF_Preamble, phyaddr %d [BMSR] %04x\n", DM9051_PHY_ADDR, db->bmsr);

					ret = dm9051_phywrite(db, 20, db->mdi);
					if (ret)
						return ret;

					/* store to showlist */
					p = get_log_addr(db);
					sprintf(&p[1], "from_phylib. %02u [lpa] %04x _dm9051_phywr[_AutoMDIX_] reg %d [val %04x]",
							db->n_automdix, db->lpa, 20, db->mdi); //= set_log_addr(db, p, ...);
				}
				break;
			}
		}
	} while (0);

	return ret;
}
#endif //0
#endif

static int dm9051_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	unsigned int val;

	if (addr == DM9051_PHY_ADDR)
	{
		//=return dm9051_mdio_read_delay(db, addr, regnum, &val);
		int ret;

		#if MI_FIX
		mutex_lock(&db->spi_lockm);
		#endif

#ifdef DMCONF_BMCR_WR
		if (regnum == MII_BMSR)
			ret = dm9051_phyread_nt_bmsr(db, &val);
		else
#endif
			ret = dm9051_phyread(db, regnum, &val);

		#if MI_FIX
		mutex_unlock(&db->spi_lockm);
		#endif

		if (ret)
			return ret;
		return val;
	}

	return 0xffff;
}

static int dm9051_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;

	if (addr == DM9051_PHY_ADDR)
	{
		int ret;

		#if MI_FIX
		static int mdio_write_count = 0;

		if (regnum == 0x0d || regnum == 0x0e) //unknown of dm9051a
			return 0;

		mutex_lock(&db->spi_lockm);
		#endif

		/* [dbg] mdio.wr BMCR */
		do {
			/* NOT next with printk for dm9051_phywr(regnum, val) */
			if ((regnum == 0) && (val & 0x800)) {
				printk("[mdio write] phywr(%d) %04x: power down (warn)\n", regnum, val);
				//printk("\n");
				break;
			}

			if (mdio_write_count <= 9)
				printk("[count%d] phywr(%d) %04x\n", mdio_write_count++, regnum, val);
		} while(0);
		ret = dm9051_phywrite(db, regnum, val);

		#if MI_FIX
		mutex_unlock(&db->spi_lockm);
		#endif

		return ret;
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
		printk("_ndo set and write [Enabling TX/RX checksum]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_RXCSUM)
	{
		printk("_ndo set and write [Enabling RX checksum only]\n");
		db->csum_gen_val = 0x0; //dm9051_set_reg(db, 0x31, 0x0);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_HW_CSUM)
	{
		printk("_ndo set and write [Enabling TX checksum only]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x0; //dm9051_set_reg(db, 0x32, 0x0);
	}
	else
	{
		//printk("_ndo set and write [Disabling TX/RX checksum]\n");
		db->csum_gen_val = 0x0; //dm9051_set_reg(db, 0x31, 0x0);
		db->csum_rcv_val = 0x0; //dm9051_set_reg(db, 0x32, 0x0);
	}

#if MI_FIX
	mutex_lock(&db->spi_lockm);
#endif
	dm9051_set_reg(db, 0x31, db->csum_gen_val);
	dm9051_set_reg(db, 0x32, db->csum_rcv_val);
#if MI_FIX
	mutex_unlock(&db->spi_lockm);
#endif

	return 0;
}

/* Functions:
 */

static unsigned int dm9051_init_intcr_value(struct board_info *db)
{
	return (get_dts_irqf(db) == IRQF_TRIGGER_LOW || get_dts_irqf(db) == IRQF_TRIGGER_FALLING) ? INTCR_POL_LOW : INTCR_POL_HIGH;
}

static int dm9051_ncr_reset(struct board_info *db)
{
	int ret;

	printk("_ncr_reset\n");

	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;

	dm9051_ncr_poll(db);

	return 0;
}
static int dm9051_phy_reset(struct board_info *db)
{
	int ret;

	printk("_phy_reset\n");

//	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
//	if (ret)
//		return ret;

//	dm9051_ncr_poll(db);

#if 1
	/* PHY reset */
	ret = PHY_RST(db);
	if (ret)
		return ret;
#endif
	return 0;
}

static int dm9051_core_init(struct board_info *db)
{
	int ret;

//	db->bc.fifo_rst_counter++;

	ret = BUS_SETUP(db); //reserved customization
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, (dm9051_modedata->skb_wb_mode) ? MBNDRY_WORD : MBNDRY_BYTE); /* MemBound */
	if (ret)
		return ret;
	// Spenser
	// ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_UNLIMIT); /* from PPCR_PAUSE_COUNT, Pause Count */
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, 0xF);
	if (ret)
		return ret;

	// Checksum Offload
	//printk("dm9051_set [write TX/RX checksum] wr 0x31 0x%02x wr 0x32 0x%02x, in _core_reset\n",
	//	db->csum_gen_val, db->csum_rcv_val);
	ret = regmap_write(db->regmap_dm, 0x31, db->csum_gen_val);
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, 0x32, db->csum_rcv_val);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	/* Diagnostic contribute: In dm9051_enable_interrupt()
	 * (or located in the core reset subroutine is better!!)
	 */
	#if defined(DMPLUG_INT) && defined(DMPLUG_INT_CLKOUT)
		printk("_reset [_core_reset] set DM9051_IPCOCR %02lx\n", IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
		ret = regmap_write(db->regmap_dm, DM9051_IPCOCR, IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
		if (ret)
			return ret;
	#endif

	return ret; /* ~return dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db)) */
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

	//dev_info(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	//		 addr[0], addr[1], addr[2],
	//		 addr[3], addr[4], addr[5]);

	if (!is_valid_ether_addr(addr))
	{
		eth_hw_addr_random(ndev);

		//#if LINUX_VERSION_CODE < KERNEL_VERSION(6,1,0)//#endif
		//#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)//#endif
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
	eth_hw_addr_set(ndev, addr);
#else
	ether_addr_copy(ndev->dev_addr, addr);
#endif

	dev_info(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 addr[0], addr[1], addr[2],
			 addr[3], addr[4], addr[5]);
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
#if MI_FIX //ee read
		mutex_lock(&db->spi_lockm);
#endif
		ret = dm9051_eeprom_read(db, (offset + i) / 2, data + i);
#if MI_FIX //ee read
		mutex_unlock(&db->spi_lockm);
#endif
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

#if MI_FIX //ee write
	mutex_lock(&db->spi_lockm);
#endif
	for (i = 0; i < len; i += 2)
	{
		ret = dm9051_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
#if MI_FIX //ee write
	mutex_unlock(&db->spi_lockm);
#endif
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

	if (pause->autoneg == AUTONEG_DISABLE) {
		int ret;
#if MI_FIX //fcr
		mutex_lock(&db->spi_lockm);
#endif
		ret = dm9051_update_fcr(db);
#if MI_FIX //fcr
		mutex_unlock(&db->spi_lockm);
#endif
		return ret;
	}

#if MI_FIX //fcr
		mutex_lock(&db->spi_lockm);
#endif
	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
					  pause->autoneg);
	phy_start_aneg(db->phydev);
#if MI_FIX //fcr
		mutex_unlock(&db->spi_lockm);
#endif
	return 0;
}

const static char dm9051_stats_strings[][ETH_GSTRING_LEN] = {
        "rx_packets",
        "tx_packets",
        "rx_errors",
        "tx_errors",
        "rx_bytes",
        "tx_bytes",
        "fifo_rst",
};

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

static const struct ethtool_ops dm9051_ethtool_ops = { //const struct ethtool_ops dm9051_ptpd_ethtool_ops
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
/* 4 ptpc */
#if 1 //0
#ifdef DMPLUG_PTP
	.get_ts_info = dm9051_ts_info, //_15888_,
#endif
#endif
};

static int dm9051_all_start(struct board_info *db)
{
	int ret;
	
	printk("_all_start\n");

	/* GPR power on of the internal phy
	 */
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	ret = dm9051_set_reg(db, DM9051_GPR, 0);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;
	ret = dm9051_phy_reset(db);
	if (ret)
		return ret;

	return dm9051_core_init(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

	/* GPR power off of the internal phy,
	 * The internal phy still could be accessed after this GPR power off control
	 */
	//printk("_stop [dm9051_all_stop] set reg DM9051_GPCR, 0x%02x\n", (unsigned int)GPCR_GEP_CNTL);
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	//printk("_stop [dm9051_all_stop] set reg DM9051_GPR, 0x%02x\n", (unsigned int)GPR_PHY_OFF);
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
static int dm9051_all_restart(struct board_info *db) //todo
{
	int ret;
		
	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;
	ret = dm9051_phy_reset(db);
	if (ret)
		return ret;
	ret = dm9051_all_reinit(db); //head_restart
	if (ret)
		return ret;

	dm9051_all_restart_sum(db);
	return 0;
}

/* to reset while link change up
 */
#ifdef DMCONF_MRR_WR
static int dm9051_all_upstart(struct board_info *db) //todo
{
	int ret;

	printk("_all_upstart\n");

	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;

	ret = dm9051_all_reinit(db); //up_restart
	if (ret)
		return ret;

	return 0;
}
#endif //DMCONF_MRR_WR

/* all reinit while rx error found
 */
int dm9051_all_reinit(struct board_info *db)
{
	int ret;

//	mutex_unlock(&db->spi_lockm);
//	phy_stop(db->phydev);
//	mutex_lock(&db->spi_lockm);

	ret = dm9051_core_init(db);
	if (ret)
		return ret;

//	mutex_unlock(&db->spi_lockm);
//	phy_start(db->phydev);
//	phy_start_aneg(db->phydev);
//	mutex_lock(&db->spi_lockm);

	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db));
	if (ret)
		return ret;

	ret = dm9051_enable_interrupt(db);
	if (ret)
		return ret;

	return dm9051_subconcl_and_rerxctrl(db);
}

void dm9051_all_restart_sum(struct board_info *db)
{
	struct net_device *ndev = db->ndev;

	db->bc.fifo_rst_counter++;
	printk("List: dm9.Show rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	netdev_dbg(ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	//printk("_[_all_restart] rxb work around done\n");
}

int dm9051_subconcl_and_rerxctrl(struct board_info *db)
{
	int ret;

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

static int trap_clr(struct board_info *db)
{
	db->bc.evaluate_rxb_counter = 0;
	return 0;
}

static void monitor_rxb0(unsigned int rxbyte)
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
			printk("_[Less constrain of old SCAN_BL trap's, NOT _all_restart] only monitored.\n");
		}
	}
}

static void monitor_rxc(struct board_info *db, int scanrr)
{
	if (econf->force_monitor_rxc && scanrr && db->bc.nRxcF < 25)
	{
		db->bc.nRxcF += scanrr;
		SHOW_MONITOR_RXC(db);
	}
}

// check rxbs
// return: 0 : Still not trap
//         1 : Do reatart trap
static int trap_rxb(struct board_info *db, unsigned int *prxbyte)
{
	if (SCAN_BH(*prxbyte) == 0)
		return 0;

	do
	{
		static unsigned int inval_rxb[TIMES_TO_RST] = {0};
		int n = 0;
		unsigned int i;
		char pbff[80];

		//u8 *bf = (u8 *)prxbyte;
		//printk("_.eval   bf] %02x %02x\n", bf[0], bf[1]);
		//printk("_.eval rxbs] %02lx %02lx\n", SCAN_BL(*prxbyte), SCAN_BH(*prxbyte));

		inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
		db->bc.evaluate_rxb_counter++;

  if (db->bc.evaluate_rxb_counter == 1) {
	sprintf(db->bc.head, "rxb 1st %d", db->bc.evaluate_rxb_counter); 
	dm9051_dump_reg2s(db, 0x74, 0x75);
	dm9051_dump_reg2s(db, 0x24, 0x25);
  }
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
			return 1;
		}
	} while (0);
	return 0;
}

static int rx_break(unsigned int rxbyte, netdev_features_t features)
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
		DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), return 0, printk("YES checksum check\n"); return -EINVAL);
	}
	else
		DM9051_RX_BREAK((SCAN_BH(rxbyte) == DM9051_PKT_RDY), return 0, /*printk("NO checksum check\n");*/ return -EINVAL);
}

static int rx_head_break(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int rxlen;

	//_15888_
	u8 err_bits = RSR_ERR_BITS;
	
	/* 7 rxhead ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	static int before_slave_ptp_packets = 5;
//	if (db->ptp_on) 
	//err_bits &= ~(RSR_LCS | RSR_PLE | RSR_AE); //(0x93);
	err_bits &= ~RSR_PTP_BITS;
	#endif
	#endif

	rxlen = le16_to_cpu(db->rxhdr.rxlen);
	if (db->rxhdr.status & err_bits || rxlen > DM9051_PKT_MAX)
	{
		printk("Err: [dm9.Monitor headbyte/status/rxlen %2x %2x %04x]\n",
			   db->rxhdr.headbyte,
			   db->rxhdr.status,
			   db->rxhdr.rxlen);

		if (db->rxhdr.headbyte != 0 &&  db->rxhdr.headbyte != 0x01) {
			printk("Err: rxhdr-byte (%02x)\n",
					   db->rxhdr.headbyte);
			netdev_dbg(ndev, "Err: rxhdr-byte (%02x)\n",
					   db->rxhdr.headbyte);
		}

		//if (db->rxhdr.status & RSR_ERR_BITS)
		if (db->rxhdr.status & err_bits)
		{
			db->bc.status_err_counter++;
		}
		else
		//if (rxlen > DM9051_PKT_MAX)
		{
			db->bc.large_err_counter++;
		}
		
		
		if (db->rxhdr.status & err_bits) {
			printk("check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
			netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
		}
		
		if (rxlen > DM9051_PKT_MAX) {
			printk("check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
			netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
		}

		return 1;
	}

	/* -rxhead ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	if (before_slave_ptp_packets && (!db->ptp_on) && (db->rxhdr.status & RSR_PTP_BITS)) {
		printk("%d. User raw state, ptp packet received!\n", before_slave_ptp_packets--);
	}
	#endif
	#endif

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
#if 1
//.		dm9051_loop_tx(db); /* [More] and more tx better performance */
#endif
		ret = dm9051_read_mem_rxb(db, DM_SPI_MRCMDX, &rxbyte, 2);
		if (ret)
			return ret;

		if (rx_break(rxbyte, ndev->features))
		{
			if (trap_rxb(db, &rxbyte)) {
				  //if (db->bc.evaluate_rxb_counter == 1) {
					sprintf(db->bc.head, "rxb last"); //(, db->bc.evaluate_rxb_counter); 
					dm9051_dump_reg2s(db, 0x74, 0x75);
					dm9051_dump_reg2s(db, 0x24, 0x25);
				  //}
				dm9051_all_restart(db);
				return -EINVAL;
			}
			break;
		}
		trap_clr(db);

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		/* rx_head_takelen check */
		ret = rx_head_break(db);
		if (ret) {
			dm9051_all_restart(db);
			return -EINVAL;
		}
		
		/* 7.1 ptpc */
		#if 1 //0
		#ifdef DMPLUG_PTP
		/* receive rx_tstamp */
		ret = dm9051_read_ptp_tstamp_mem(db, db->rxTSbyte);
		if (ret)
			return ret;
		#endif
		#endif

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		padlen = (dm9051_modedata->skb_wb_mode && (rxlen & 1)) ? rxlen + 1 : rxlen;
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

		if (is_ptp_packet(skb->data))
		do {
			static int slave_get_ptpFrame = 9;
			static int master_get_delayReq6 = 6; //5;
			static int slave_get_ptpMisc = 9;
			//u8 message_type0 =
			//	get_ptp_message_type(skb);
			u8 message_type =
				get_ptp_message_type005(skb);
			//printk("message type, A= %X B= %X\n", message_type0, message_type);
			
			if (is_ptp_sync_packet(message_type)) {
				if (slave_get_ptpFrame)
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					printk("\n");
					printk("Slave(%d)-get-sync with tstamp. \n", --slave_get_ptpFrame);
					//sprintf(db->bc.head, "Slave-get-sync with tstamp, len= %3d", skb->len);
					//dm9051_dump_data1(db, skb->data, skb->len);
				} else {
					printk("Slave(%d)-get-sync without tstamp. \n", --slave_get_ptpFrame);
				}}
			} else
			if (message_type == PTP_MSGTYPE_FOLLOW_UP) {
				if (slave_get_ptpFrame)
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					printk("Slave(%d)-get-followup with tstamp. \n", --slave_get_ptpFrame);
				} else {
					printk("Slave(%d)-get-followup without tstamp. \n", --slave_get_ptpFrame);
				}}
			} else
			if (message_type == PTP_MSGTYPE_DELAY_RESP) {
				if (slave_get_ptpFrame)
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					printk("Slave(%d)-get-DELAY_RESP with tstamp. \n", --slave_get_ptpFrame);
				} else {
					printk("Slave(%d)-get-DELAY_RESP without tstamp. \n", --slave_get_ptpFrame);
				}}
			} else
			if (message_type == PTP_MSGTYPE_ANNOUNCE) {
				if (slave_get_ptpFrame)
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					printk("Slave(%d)-get-ANNOUNCE with tstamp. \n", --slave_get_ptpFrame);
				} else {
					printk("Slave(%d)-get-ANNOUNCE without tstamp. \n", --slave_get_ptpFrame);
				}}
			} else
			if (is_ptp_delayreq_packet(message_type)) {
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					if (master_get_delayReq6) {
						printk("Master(%d)-get-DELAY_REQ with tstamp. \n", --master_get_delayReq6);
					}
				} else {
					printk("Master-get-DELAY_REQ without tstamp.\n");
				}}
			} else
			{
				if (slave_get_ptpMisc)
				if (db->ptp_enable) {
				if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
					printk("Slave(%d) or Master get-knonw with tstamp. \n", --slave_get_ptpMisc);
				} else {
					printk("Slave(%d) or Master get-knonw without tstamp. \n", --slave_get_ptpMisc);
				}}
			}
		} while(0);

		skb->protocol = eth_type_trans(skb, db->ndev);

		/* 7.2 ptpc */
		#if 1 //0
		#ifdef DMPLUG_PTP
	//So when NOT T1/T4, we can skip tell an empty (virtual) tstamp
	//if (db->rxhdr.status & RSR_RXTS_EN) {	// Is it inserted Timestamp?
		dm9051_ptp_rx_hwtstamp(db, skb, db->rxTSbyte); //_15888_, 
		/* following, with netif_rx(skb),
		 * slave4l can parse the T1 and/or T4 rx tstamp from master
		 */
	//}
		#endif
		#endif

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

#ifndef DMPLUG_CONTI
#define EXPEND_LEN(datlen,pd)	(datlen + pd)
#define WRITE_SKB(db,p,len)		dm9051_write_mem_cache(db,p,len)
#ifdef DM9051_SKB_PROTECT
static struct sk_buff *EXPAND_SKB(struct sk_buff *skb, unsigned int pad)
{	
	struct sk_buff *skb2;
	//printk("pad skb->len %d, data_len+pad %d (pad %d)\n", skb->len, EXPEND_LEN(skb->len, pad), pad);

	skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
	if (skb2) {
		dev_kfree_skb(skb);
		//printk("new skb->len %d, data_len+pad %d (pad %d)\n", skb2->len, EXPEND_LEN(skb2->len, pad), pad);
		return skb2;
	}

	printk("[WB_SUPPORT] warn on len %d, skb_copy_expand get memory leak!\n", skb->len);
	return skb;
}
#endif

static int dm9051_single_tx(struct board_info *db, u8 *p, unsigned int data_len, unsigned int pad)
{
	int ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = WRITE_SKB(db, p, EXPEND_LEN(data_len, pad)); //'!wb'
	if (ret)
		return ret;

	return dm9051_set_regs(db, DM9051_TXPLL, &data_len, 2);
}

static int dm9051_req_tx(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_TCR, db->tcr_wr); //base with TCR_TXREQ
}
#endif

static int TX_PACKET(struct board_info *db, struct sk_buff *skb)
{
	unsigned int data_len = skb->len;
	int ret;

	/* 6 tx ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	u8 message_type = get_ptp_message_type005(skb);

	db->tcr_wr = TCR_TXREQ; // TCR register value
	//db->ptp_mode = (u8) dm9051_ptp_one_step(skb, db); //_15888_,
	//db->ptp_mode = (u8) dm9051_ptp_one_step001(skb, db); //_15888_,
	if (dm9051_ptp_frame(db, skb)) {
		//or
		//if (likely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			if (is_ptp_sync_packet(message_type)) {
				if (db->ptp_step == 2) {
					db->tcr_wr = TCR_TS_EN | TCR_TXREQ;
					//printk("SM9051 SYNC step %u, tcr = TCR_TS_EN | TCR_TXREQ\n", db->ptp_step);
				} else {
					db->tcr_wr = TCR_TS_EMIT | TCR_TXREQ;
					//printk("SM9051 SYNC step %u, tcr = TCR_TS_EN | TCR_TS_EMIT\n", db->ptp_step);
				}
			} else if (is_ptp_delayreq_packet(message_type)) //_15888_,
				db->tcr_wr = TCR_TS_EN | TCR_TS_EMIT | TCR_TXREQ;
		//}
	}
	#endif
	#endif

	do {
		//if (!DM9051_TX_CONTI()) //TX_CONTI will place into dm9051_plug.c (then eliminate dm9051_open.c)
		//{ //as below:
		//}
#ifdef DMPLUG_CONTI
		ret = TX_OPS_CONTI(db, skb); //skb->data, data_len); //'double_wb'
		if (ret)
			break;
#else
		/*
		 * if (!dm9051_modedata->skb_wb_mode) {
		 *   ret = WRITE_SKB(db, skb, skb->len);
		 *   ret = dm9051_single_tx(db, skb->len);
		 * }
		 * else {
		 *   pad = ...
		 *   skb = EXPAND_SKB(skb, pad);
		 *   ret = WRITE_SKB(db, skb, EXPEND_LEN(data_len, pad);
		 *   ret = dm9051_single_tx(db, skb, data_len, pad);
		 * }
		 * ret = dm9051_req_tx(db);
		 */
		unsigned int pad = (dm9051_modedata->skb_wb_mode && (data_len & 1)) ? 1 : 0; //'~wb'

		#ifdef DM9051_SKB_PROTECT
		if (pad)
			skb = EXPAND_SKB(skb, pad);
		#endif

		ret = dm9051_single_tx(db, skb->data, data_len, pad);
		if (ret)
			break;
#endif

		ret = dm9051_req_tx(db);
	} while(0);

	if (ret)
		db->bc.tx_err_counter++;
	else {
		struct net_device *ndev = db->ndev;
		ndev->stats.tx_bytes += data_len;
		ndev->stats.tx_packets++;
	}

	/* 6.1 tx ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	/*
	 * Change to be only 'db->ptp_sync' is true. (less report, report only essential.)
	 */
//	if (db->ptp_mode == PTP_ONE_STEP || db->ptp_mode == PTP_TWO_STEP || db->ptp_mode == PTP_NOT_SYNC) { //temp
		//u8 message_type = get_ptp_message_type005(skb);
//	}
	if ((is_ptp_sync_packet(message_type) &&
		db->ptp_step == 2) ||
		is_ptp_delayreq_packet(message_type)) //_15888_,
		dm9051_ptp_tx_hwtstamp(db, skb); //dm9051_hwtstamp_to_skb(skb, db); //_15888_,
	#endif
	#endif

	dev_kfree_skb(skb);
	return ret;
}

static int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;

	while (!skb_queue_empty(&db->txq))
	{
		struct sk_buff *skb = skb_dequeue(&db->txq);
		if (skb) {
			if (TX_PACKET(db, skb)) {
				if (netif_queue_stopped(ndev) &&
					(skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
					netif_wake_queue(ndev);
				return ntx;
			}
			ntx++;
		}

		if (netif_queue_stopped(ndev) &&
			(skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

/* schedule delay works */

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

/* start_xmit schedule delay works */

static void dm9051_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);

	mutex_lock(&db->spi_lockm);

	dm9051_loop_tx(db);
//	int result;
//	result = 
//	if (result < 0)
//		netdev_err(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

/* Common: looping rx and tx */

static int dm9051_delayp_looping_rx_tx(struct board_info *db) //.looping_rx_tx()
{
	int result; //, result_tx;

	do
	{
		result = dm9051_loop_rx(db); /* threaded rx */
		if (result < 0)
			return result; //result; //goto out_unlock;

		dm9051_loop_tx(db); /* more tx better performance */
//		result_tx = 
//		if (result_tx < 0)
//			return result_tx; //result_tx; //goto out_unlock;
	} while (result > 0);

	return 0;
}

#if 1
//static void dm9051_rx_plat_enable(struct board_info *db)
//{
//	dm9051_enable_interrupt(db);
//}
//static int dm9051_rx_plat_disable(struct board_info *db)
//{
//	int result = dm9051_disable_interrupt(db);
//	if (result)
//		return result;

//	result = dm9051_clear_interrupt(db);
//	if (result)
//		return result;
//	return result;
//}

//static void dm9051_rx_plat_loop(struct board_info *db)
//{
//	int ret;

//	ret = dm9051_delayp_looping_rx_tx(db); //.looping_rx_tx()
//	if (ret < 0)
//		return;

//	dm9051_enable_interrupt(db); //"dm9051_rx_plat_enable(struct board_info *db)"
//}

/* Interrupt: Interrupt work */

static void dm9051_rx_int2_plat(int voidirq, void *pw) //.dm9051_(macro)_rx_tx_plat()
{
	struct board_info *db = pw;
	int result; //, result_tx;

	mutex_lock(&db->spi_lockm);

#if 1 //[REAL.]	//'MI_FIX' (result = dm9051_rx_plat_disable(db);)
	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	dm9051_delayp_looping_rx_tx(db); //.looping_rx_tx()
	//result = 
	//if (result < 0)
	//	goto out_unlock;

	dm9051_enable_interrupt(db);
#else //[TEMP.]	
	//result = dm9051_clear_interrupt(db);
	//if (result)
	//	goto out_unlock;
#endif //[TEMP.]

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);
	//return IRQ_HANDLED;
}

/* !Interrupt: Poll delay work */
#ifndef DMPLUG_INT //NOT DMPLUG_INT =POLL
/* [DM_TIMER_EXPIRE2] poll extream.fast */
/* [DM_TIMER_EXPIRE1] consider not be 0, to alower and not occupy almost all CPU resource.
 * This is by CPU scheduling-poll, so is software driven!
 */
#define DM_TIMER_EXPIRE1 1
#define DM_TIMER_EXPIRE2 0
#define DM_TIMER_EXPIRE3 0

void dm9051_poll_servicep(struct work_struct *work) //.dm9051_poll_delay_plat()
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_workp);

	mutex_lock(&db->spi_lockm);

	dm9051_delayp_looping_rx_tx(db); //.looping_rx_tx()

	mutex_unlock(&db->spi_lockm);

	if (db->bc.ndelayF >= csched.nTargetMaxNum)
		db->bc.ndelayF = POLL_OPERATE_INIT;

	/* redundent, but for safe */
	//if (!dm9051_cmode_int)
	schedule_delayed_work(&db->irq_workp, csched.delayF[db->bc.ndelayF++]);
}
#endif

int thread_servicep_done = 1;
int thread_servicep_re_enter;

#ifdef INT_TWO_STEP
void dm9051_rx_irq_servicep(struct work_struct *work) //optional: INT: TWO_STEP SRVEICE
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_servicep);

	dm9051_rx_int2_plat(0, db); // 0 is no-used //.dm9051_(macro)_rx_tx_plat()
	thread_servicep_done = 1;

}

irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw) //optional: INT: TWO_STEP
{
	struct board_info *db = pw;

	if (!thread_servicep_re_enter)
		printk("_.int2   [%s] first-enter %d\n", __func__, thread_servicep_re_enter++); //function

	if (thread_servicep_done) {
		thread_servicep_done = 0;

		#if 1
		//dm9051_rx_int2_plat(voidirq, pw); //.dm9051_(macro)_rx_tx_plat()
		schedule_delayed_work(&db->irq_servicep, 0);
		#endif
	}
	else {
		if (thread_servicep_re_enter <= 10)
			printk("_.int2   [%s] re-enter %d\n", __func__, thread_servicep_re_enter++);
	}
	return IRQ_HANDLED;
}
#endif //INT_TWO_STEP

irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw)
{
	if (!thread_servicep_re_enter)
		printk("_.int   [dm9051_rx_threaded_plat] first-enter %d\n", thread_servicep_re_enter++);
		
	if (thread_servicep_done) {
		thread_servicep_done = 0;

		dm9051_rx_int2_plat(voidirq, pw); //.dm9051_(macro)_rx_tx_plat()
		thread_servicep_done = 1;
	} else {
		printk("_.int   [dm9051_rx_threaded_plat] re-enter %d\n", thread_servicep_re_enter++);
	}
	return IRQ_HANDLED;
}
#endif

/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;
	
	printk("\n");
	netdev_info(db->phydev->attached_dev, "dm9051_open\n");
	netdev_info(db->phydev->attached_dev, "Davicom: %s(%d)", dmplug_intterrpt_des, dmplug_interrupt);
	//amdix_log_reset(db); (to be determined)

	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	#ifdef DMPLUG_PTP
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_RXEN; //_15888_ //Disable discard CRC error (work around)
	#else
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	#endif
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */
//before [spi_lockm]

//use [spi_lockm]
	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	ret = dm9051_all_start(db);
	if (ret)
		goto open_end;

/* -open ptpc */
#if 1 //0
#ifdef DMPLUG_PTP
	if (db->ptp_on) {
		//_15888_ 
		u32 rate_reg = dm9051_get_rate_reg(db); //15888, dm9051_get_rate_reg(db);
		printk("Pre-RateReg value = 0x%08X\n", rate_reg);
	}
#endif
#endif

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db));
	if (ret)
		goto open_end;

	ret = dm9051_enable_interrupt(db);
	if (ret)
		goto open_end;

//.	phy_support_sym_pause(db->phydev);
//.	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	netif_wake_queue(ndev);

//	if (!DM9051_OPEN_POLLING()) //POLLING will place into dm9051_plug.c (then eliminate dm9051_open.c)
//	{ //as below:
//	}
	ret = DM9051_OPEN_REQUEST(db);
	if (ret < 0) {
		#if MI_FIX
		mutex_unlock(&db->spi_lockm);
		#endif

		phy_stop(db->phydev); //of 'dm9051_core_clear(db)' //

		#if MI_FIX
		mutex_lock(&db->spi_lockm);
		#endif
		goto open_end;
	}

open_end:
	printk("dm9051_open_end.done\n");

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif
	return ret;
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

	//printk("\n");
	dev_info(&db->spidev->dev, "dm9051_stop\n");

	phy_stop(db->phydev);

	/* schedule delay work */
	#ifdef DMPLUG_INT
	#ifdef INT_TWO_STEP
		cancel_delayed_work_sync(&db->irq_servicep); //.if (_dm9051_cmode_int)
	#endif //INT_TWO_STEP
	#else //DMPLUG_INT
		cancel_delayed_work_sync(&db->irq_workp); //.if (!_dm9051_cmode_int)
	#endif 

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);


	#ifdef DMPLUG_INT
	END_FREE_IRQ(ndev);
	#endif

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);


	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	ret = dm9051_all_stop(db);

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	return ret;
}
//static int dm9051_stop001(struct net_device *ndev)
//{
//	struct board_info *db = to_dm9051_board(ndev);
//	int ret;

//	dev_info(&db->spidev->dev, "dm9051_stop\n");
//	printk("\n");

////	mutex_lock(&db->spi_lockm);
//	ret = dm9051_all_stop(db);
////	mutex_unlock(&db->spi_lockm);
//	if (ret)
//		return ret;

//	/* schedule delay work */
//	#ifdef DMPLUG_INT
//	#ifdef INT_TWO_STEP
//		cancel_delayed_work_sync(&db->irq_servicep); //.if (_dm9051_cmode_int)
//	#endif //INT_TWO_STEP
//	#else //DMPLUG_INT
//		cancel_delayed_work_sync(&db->irq_workp); //.if (!_dm9051_cmode_int)
//	#endif 

//	flush_work(&db->tx_work);
//	flush_work(&db->rxctrl_work);

////	mutex_lock(&db->spi_lockm);
//	phy_stop(db->phydev);
////	mutex_unlock(&db->spi_lockm);

//	#ifdef DMPLUG_INT
//	END_FREE_IRQ(ndev);
//	#endif

//	netif_stop_queue(ndev);

//	skb_queue_purge(&db->txq);

//	return 0;
//}

/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

		#if 0
		//show_ptp_type(skb);	//Show PTP message type
		skb_tx_timestamp(skb);	
		//Spenser - Report software Timestamp ----- no need? v.s. skb_tstamp_tx(skb, &shhwtstamps);//Report HW Timestamp
		#endif

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
#if MI_FIX //mac
	mutex_lock(&db->spi_lockm);
#endif
	ret = dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
#if MI_FIX //mac
	mutex_unlock(&db->spi_lockm);
#endif
	return ret;
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
	/* 5 ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	.ndo_eth_ioctl = dm9051_ptp_netdev_ioctl, //_15888_
	#endif
	#endif
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

	db->csum_gen_val = 0; //disabling
	db->csum_rcv_val = 0; //disabling
	db->n_automdix = 0; //log-reset
	db->stop_automdix_flag = 0;
	db->automdix_log[0][0] = 0;
	db->automdix_log[1][0] = 0;
	db->automdix_log[2][0] = 0;
	db->mdi = 0x0830;
	
	db->tcr_wr = TCR_TXREQ; //pre-defined

	//db->ptp_step = 0;
	//db->ptp_packet = 0;
	//db->ptp_sync = 0;
	//db->tempetory_ptp_dreq = 0;
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
	db->mdiobus->phy_mask = (u32)~BIT(1); //if ((bus->phy_mask & BIT(1)) == 0) accepted
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

	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	if (db->phydev->link)
	{
printk("\n");
printk("LOCK_MUTEX\n");

		if (db->phydev->pause)
		{
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
	}

	phy_print_status(db->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link)
	{
#ifdef DMCONF_MRR_WR
		do {
			int ret = dm9051_all_upstart(db);
			if (ret)
				goto u_end;
		} while(0);
		dm9051_update_fcr(db);
u_end:
#else
		dm9051_update_fcr(db);
#endif

printk("UNLOCK_MUTEX\n");
printk("\n");
	}

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif
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
	//.by ptp4l run command
	//db->ptp_on = 1;		//Enable PTP must disable checksum_offload

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;//&dm9051_ptpd_ethtool_ops;

	/* Set default features */
	if (dm9051_modedata->checksuming)
	{
		// Spenser - Setup for Checksum Offload
	#ifdef DMPLUG_PTP
		dev_info(&db->spidev->dev, "Enable PTP must coerce to disable checksum_offload\n");
	#else
		ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	#endif
	}
	ndev->hw_features |= ndev->features;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

	#ifdef DMPLUG_INT
	#ifdef INT_TWO_STEP
		INIT_RX_INT2_DELAY_SETUP(db);
	#endif
	#else
		INIT_RX_POLL_DELAY_SETUP(db);
	#endif

	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	printk("\n");
	dev_info(dev, "Davicom: %s", confdata.release_version);
	//dev_info(dev, "Davicom: %s(%d)", dmplug_intterrpt_des, dmplug_interrupt);
	//dev_info(dev, "Davicom: confdata.interrupt= %s", confdata.interrupt);

	SHOW_CONFIG_MODE(spi);

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

//	ret = _BUS_SETUP(db); /* first, reserved customization */
//	if (ret)
//		return ret;

	SHOW_OPTION_MODE(spi);

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

	/* 2 ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	//db->ptp_on = 1;
	db->ptp_enable = 1;
	db->ptp_on = 0;
	dev_info(&db->spidev->dev, "DM9051A Driver PTP Init\n");
	dm9051_ptp_init(db); //_15888_
	#endif
	#endif

	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,0)
static int dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	phy_disconnect(db->phydev);

	/* 3 ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	dm9051_ptp_stop(db); //_15888_ todo
	#endif
	#endif
	return 0;
}
#else
static void dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	sprintf(db->bc.head, "drv remove");
	dm9051_dump_reg2s(db, DM9051_PIDL, DM9051_PIDH);
	dm9051_dump_reg2s(db, DM9051_VIDL, DM9051_VIDH);

	phy_disconnect(db->phydev);

	/* 3 ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	dm9051_ptp_stop(db); //_15888_ todo
	#endif
	#endif
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
