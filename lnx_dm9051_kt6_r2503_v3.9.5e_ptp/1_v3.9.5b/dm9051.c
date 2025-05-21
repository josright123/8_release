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
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>
#define MAIN_DATA
#include "dm9051.h"

const struct mod_config *dm9051_modedata = &driver_align_mode; /* Driver configuration */

/* Tx 'wb' do skb protect */
#define DM9051_SKB_PROTECT
#define STICK_SKB_CHG_NOTE
#define DM9051_INTR_BACKCODE

/* Helper macros */
#define SCAN_BL(dw) (dw & GENMASK(7, 0))
#define SCAN_BH(dw) ((dw & GENMASK(15, 8)) >> 8)

/* raw fake encrypt */
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty

/* fake normal tx mode */
#define dmplug_tx "normal"
#define TX_CONTI_NEW(d)

/* re-direct conti */
#ifdef DMPLUG_CONTI
#undef TX_CONTI_NEW
#define TX_CONTI_NEW(d) tx_contu_new(d)
#endif

/* fake ptpc */
#define PTP_NEW(d, n)
#define PTP_INIT_RCR(d)
#define PTP_INIT(d)
#define PTP_END(d)

/* re-direct ptpc */
#ifdef DMPLUG_PTP
#undef PTP_NEW
#define PTP_NEW(d, n) ptp_new(d, n)
#undef PTP_INIT_RCR
#define PTP_INIT_RCR(d) ptp_init_rcr(d)
#undef PTP_INIT
#define PTP_INIT(d) ptp_init(d)
#undef PTP_END
#define PTP_END(d) ptp_end(d)
#endif

/* raw(fake) bmsr_wr */
#define PHY_READ(d, n, av) dm9051_phyread(d, n, av)

/* re-direct bmsr_wr */
#ifdef DMCONF_BMCR_WR
#undef PHY_READ
#define PHY_READ(d, n, av) dm9051_phyread_nt_bmsr(d, n, av)
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

#define dmplug_rx_mach "interrupt direct edge mode"
#if defined(INT_CLKOUT)
#undef dmplug_rx_mach
#define dmplug_rx_mach "interrupt periodic clkout mode"
#endif
#if !defined(DMPLUG_INT)
#undef dmplug_rx_mach
#define dmplug_rx_mach "poll mode"
#endif

#define dmplug_intterrpt2 "interrupt direct step"
#if defined(INT_TWO_STEP)
#undef dmplug_intterrpt2
#define dmplug_intterrpt2 "interrupt two step"
#endif 

//#if defined(DMPLUG_INT)
//  #if !defined(INT_CLKOUT)
//  #define dmplug_rx_mach "interrupt polarity ctrl mode"
//  #else
//  #define dmplug_rx_mach "interrupt clkout mode"
//  #endif

//  #if !defined(INT_TWO_STEP)
//  #define dmplug_intterrpt2 "interrupt direct step"
//  #else
//  #define dmplug_intterrpt2 "interrupt two step"
//  #endif
//#else
//#define dmplug_rx_mach "poll mode"
//#endif

/* log */
#define DEV_INFO_RX_ALIGN(dev) \
		dev_warn(dev, "RX: %s blk %u\n", \
			dm9051_modedata->align.burst_mode_info, \
			dm9051_modedata->align.rx_blk)
#define DEV_INFO_TX_ALIGN(dev) \
		dev_warn(dev, "TX: %s blk %u\n", \
			dm9051_modedata->align.burst_mode_info, \
			dm9051_modedata->align.tx_blk)
#define PRINT_ALIGN_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[TX %s mode][Alignment RX %u, Alignment RX %u] nRxc %d\n", \
			dmplug_tx, \
			dm9051_modedata->align.rx_blk, \
			dm9051_modedata->align.tx_blk, \
			n)
#define PRINT_BURST_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[rx/tx %s mode] nRxc %d\n", \
			dmplug_tx, \
			n)
#define PRINT_REGMAP_BLK_ERR(evtstr, ret, r, n) \
		netif_err(db, drv, db->ndev, "%s: error %d noinc %s regs %02x len %u\n", \
			__func__, ret, \
			evtstr, \
			r, \
			n)

static void SHOW_DEVLOG_REFER_BEGIN(struct device *dev, struct board_info *db)
{
	printk("\n");
#if 1	//[Test]
	dev_info(dev, "dev_info Version\n");
	dev_notice(dev, "dev_notice Version\n");
	//dev_alert(dev, "dev_alert Version\n"); //(available)
	//dev_emerg(dev, "dev_emerg Version\n");
	dev_warn(dev, "dev_warn Version\n");
	dev_crit(dev, "dev_crit Version\n");
	dev_err(dev, "dev_err Version\n");
	printk("\n");
#endif
	/* conti */
	TX_CONTI_NEW(db);
	/* 2.0 ptpc */
	if (db->ptp_enable) {
		dev_info(&db->spidev->dev, "DMPLUG PTP Version\n");
		dev_info(&db->spidev->dev, "Enable PTP must COERCE to disable checksum_offload\n");
	}
}

static void SHOW_LOG_REFER_BEGIN(struct board_info *db)
{
#if 1
	/* [.] netif_msg_drv
	 */
	printk("\n");
	netif_info(db, probe, db->ndev, "netif_info Version\n");
	netif_notice(db, probe, db->ndev, "netif_notice Version\n");
	//netif_alert(db, probe, db->ndev, "netif_alert Version\n"); //(available)
	//netif_emerg(db, probe, db->ndev, "netif_emerg Version\n");
	netif_warn(db, probe, db->ndev, "netif_warn Version\n");
	netif_crit(db, probe, db->ndev, "netif_crit Version\n");
	netif_err(db, probe, db->ndev, "netif_err Version\n");
//	netif_dbg(db, probe, ndev, "netif_dbg Version\n");
//	netdev_dbg(ndev, "netdev_dbg Version\n");
#endif
}

void SHOW_DRIVER(struct device *dev)
{
	/* driver version log */
	printk("\n");
#if defined(__x86_64__) || defined(__aarch64__)
	// 64-bit code
	#ifdef CONFIG_64BIT
	// 64-bit specific code
	dev_warn(dev, "Davicom: %s (64 bit)", confdata.release_version);
	#else
	// 32-bit specific code
	dev_crit(dev, "Davicom: %s (64 bit, warn: but kernel config has no CONFIG_64BIT?)", confdata.release_version);
	#endif
#else
	// 32-bit code
	#ifdef CONFIG_64BIT
	// 64-bit specific code
	dev_crit(dev, "Davicom: %s (32 bit, warn: but kernel config has CONFIG_64BIT?))", confdata.release_version);
	#else
	// 32-bit specific code
	dev_warn(dev, "Davicom: %s (32 bit)", confdata.release_version);
	#endif
#endif
}

void SHOW_DTS_SPEED(struct device *dev)
{
	/* [dbg] spi.speed */
	unsigned int speed;
	of_property_read_u32(dev->of_node, "spi-max-frequency", &speed);
	dev_info(dev, "SPI speed from DTS: %d Hz\n", speed);
}

void SHOW_RX_MATCH_MODE(struct device *dev)
{
	dev_info(dev, "Davicom: %s", dmplug_rx_mach);
}

void SHOW_RX_INT_MODE(struct device *dev)
{
	#if defined(DMPLUG_INT)
	dev_info(dev, "Davicom: %s", dmplug_intterrpt2);
	#endif
}

void SHOW_DTS_INT(struct device *dev)
{
	#if defined(DMPLUG_INT)
	unsigned int intdata[2];
	of_property_read_u32_array(dev->of_node, "interrupts", &intdata[0], 2);
	dev_info(dev, "Operation: Interrupt pin: %d\n", intdata[0]); // intpin
	dev_info(dev, "Operation: Interrupt trig type: %d\n", intdata[1]);
	#endif
}

static void SHOW_CONFIG_MODE(struct device *dev)
{
	printk("\n");
	//dev_info(dev, "Davicom: %s", driver_align_mode.test_info);
#if defined(__x86_64__) || defined(__aarch64__)
	// 64-bit code
	dev_info(dev, "LXR: %s, BUILD: %s __aarch64__ (64 bit)\n", utsname()->release, utsname()->release); //(compile-time)
	
	#ifdef CONFIG_64BIT
	// 64-bit specific code
	dev_info(dev, "LXR: %s, BUILD: %s CONFIG_64BIT (64 bit)\n", utsname()->release, utsname()->release); //(compile-time): "%s\n", UTS_RELEASE);
	#else
	// 32-bit specific code
	dev_err(dev, "LXR: %s, BUILD: %s CONFIG_32BIT (32 bit, warn: because config has no CONFIG_64BIT ?)\n", utsname()->release, utsname()->release);
	#endif
#else
	// 32-bit code
	dev_info(dev, "LXR: %s, BUILD: %s __aarch32__ (32 bit)\n", utsname()->release, utsname()->release); //(compile-time)

	#ifdef CONFIG_64BIT
	// 64-bit specific code
	dev_err(dev, "LXR: %s, BUILD: %s (64 bit, warn: CONFIG_64BIT contrary to __aarch32__ ?)\n", utsname()->release, utsname()->release);
	#else
	// 32-bit specific code
	dev_info(dev, "LXR: %s, BUILD: %s CONFIG_32BIT (32 bit)\n", utsname()->release, utsname()->release); //(compile-time): "%s\n", UTS_RELEASE);
	#endif
#endif
}

static void SHOW_DEVLOG_MODE(struct device *dev)
{
	SHOW_DRIVER(dev);

	SHOW_DTS_SPEED(dev);
	SHOW_RX_MATCH_MODE(dev);
	SHOW_RX_INT_MODE(dev);
	SHOW_DTS_INT(dev);

	SHOW_CONFIG_MODE(dev);
	DEV_INFO_TX_ALIGN(dev);
	DEV_INFO_RX_ALIGN(dev);
}

static void SHOW_PLAT_MODE(struct device *dev)
{
	dev_info(dev, "Davicom: %s", driver_align_mode.test_info);
}

int SHOW_MAP_CHIPID(struct device *dev, unsigned short wid)
{
	if (wid != DM9051_ID)
	{
		dev_err(dev, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_warn(dev, "chip %04x found\n", wid);
	return 0;
}

static void SHOW_OPTION_MODE(struct device *dev)
{
	dev_info(dev, "Check TX End: %llu, TX mode= %s mode, DRVR= %s, %s\n", econf->tx_timeout_us, dmplug_tx,
			econf->force_monitor_rxb ? "monitor rxb" : "silence rxb",
			econf->force_monitor_tx_timeout ? "monitor tx_timeout" : "silence tx_ec");
}

void SHOW_MAC(struct board_info *db, u8 *addr)
{
	dev_warn(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 addr[0], addr[1], addr[2],
			 addr[3], addr[4], addr[5]);
}

static void SHOW_MONITOR_RXC(struct board_info *db)
{
	if (dm9051_modedata->align.burst_mode == BURST_MODE_FULL)
		PRINT_BURST_INFO(db->bc.nRxcF);
	else if (dm9051_modedata->align.burst_mode == BURST_MODE_ALIGN)
		PRINT_ALIGN_INFO(db->bc.nRxcF);
}

static void SHOW_OPEN(struct board_info *db)
{
	printk("\n");
	netif_info(db, drv, db->ndev, "dm9051_open\n");
	netif_info(db, drv, db->ndev, "Davicom: %s", dmplug_rx_mach);
	#if defined(DMPLUG_INT)
	netif_info(db, drv, db->ndev, "Davicom: %s", dmplug_intterrpt2);
	#endif
	/* amdix_log_reset(db); */ //(to be determined)
}

void dm9051_dump_data1(struct board_info *db, u8 *packet_data, int packet_len)
{
	int i, j, rowsize = 32;
	int splen; //index of start row
	int rlen; //remain/row length 
	char line[120];

	netif_info(db, pktdata, db->ndev, "%s\n", db->bc.head);
	for (i = 0; i < packet_len; i += rlen) {
		//rlen = print_line(packet_data+i, min(rowsize, skb->len - i)); ...
		rlen =  packet_len - i;
		if (rlen >= rowsize) rlen = rowsize;

		splen = 0;
		splen += sprintf(line + splen, " %3d", i);
		for (j = 0; j < rlen; j++) {
			if (!(j % 8)) splen += sprintf(line + splen, " ");
			if (!(j % 16)) splen += sprintf(line + splen, " ");
			splen += sprintf(line + splen, " %02x", packet_data[i+j]);
		}
		netif_info(db, pktdata, db->ndev, "%s\n", line);
	}
}

static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2)
{
	unsigned int v1, v2;

	dm9051_get_reg(db, reg1, &v1);
	dm9051_get_reg(db, reg2, &v2);
	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, reg1, v1, reg2, v2);
}

void dm9051_headlog_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2)
{
	memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, head);
	dm9051_dump_reg2s(db, reg1, reg2);
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
		u32 BLKTX = dm9051_modedata->align.tx_blk;
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
		u32 BLKRX = dm9051_modedata->align.rx_blk;
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

/* waiting tx-end rather than tx-req
 * got faster
 */
#if !defined(DMPLUG_CONTI) || defined(DMPLUG_PTP)
int dm9051_nsr_poll(struct board_info *db)
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
#if !defined(DMPLUG_CONTI)
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all);
#else
	return TX_MOTE2_CONTI_RCR(db);
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
		netif_info(db, timer, db->ndev, "timeout of dm9051_eeprom_write %d %04x\n", offset, to[0] | to[1] << 8);
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
		netif_info(db, timer, db->ndev, "timeout of dm9051_eeprom_write %d %04x\n", offset, data[0] | data[1] << 8);
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
		netif_info(db, timer, db->ndev, "timeout of dm9051_phyrd %d %04x\n", reg, *val);
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
		netif_info(db, timer, db->ndev, "timeout of dm9051 phy write %d %04x\n", reg, val);
		return ret;
	}

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}


/* dm9051_phyread.EXTEND for 'link'
 */
static int dm9051_phyread_headlog(char *head, struct board_info *db,
									unsigned int reg)
//=static int dm9051_phyread_log_reset(struct board_info *db,
//									unsigned int reg)
{
	unsigned int val;
	int ret = dm9051_phyread(db, reg, &val);
	if (ret) {
		netif_err(db, link, db->ndev, "%s dm9051_phyrd(%u), ret = %d (ERROR)\n", head, reg, ret);
		return ret;
	}
	netif_warn(db, link, db->ndev, "%s %04x\n", head, val);
	return ret;
}

//static int PHY_LOG(struct board_info *db)
//{
//	int ret = dm9051_phyread_log_reset(db, 0);
//	if (ret)
//		return ret;
//	ret = dm9051_phyread_log_reset(db, 3);
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

static int dm9051_phy_reset(struct board_info *db)
{
	int ret;
#if 1
	/* PHY reset */
	printk("_phy_reset\n");
	ret = PHY_RST(db);
	if (ret)
		return ret;
#endif
	return 0;
}

//#ifdef DMCONF_BMCR_WR
//PHY_READ/dm9051_phyread_nt_bmsr
//#endif

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

		ret = PHY_READ(db, regnum, &val);

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
	//struct device *dev1 = &db->spidev->dev; //= &spi->dev;

	if (addr == DM9051_PHY_ADDR)
	{
		static int mdio_write_count = 0;
		int ret;

		#if MI_FIX
		if (regnum == 0x0d || regnum == 0x0e) //unknown of dm9051a
			return 0;

		mutex_lock(&db->spi_lockm);
		#endif

		/* [dbg] mdio.wr BMCR */
		do {
			/* NOT next with k for dm9051_phywr(regnum, val) */
			if ((regnum == 0) && (val & 0x800)) {
				netif_crit(db, link, db->ndev, "[mdio phywr] %d %04x: power down (warn)\n", regnum, val);
				break;
			}

			if (mdio_write_count <= 9)
				netif_info(db, link, db->ndev, "[count%d] mdio phywr %d %04x\n", mdio_write_count++, regnum, val);
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
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling TX/RX checksum]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_RXCSUM)
	{
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling RX checksum only]\n");
		db->csum_gen_val = 0x0; //dm9051_set_reg(db, 0x31, 0x0);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_HW_CSUM)
	{
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling TX checksum only]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x0; //dm9051_set_reg(db, 0x32, 0x0);
	}
	else
	{
		//netif_info(db, drv, db->ndev, "_ndo set and write [Disabling TX/RX checksum]\n");
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

static int dm9051_core_init(struct board_info *db)
{
	int ret = BUS_SETUP(db); //reserved, customization, raw empty or overlay
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
	//k("dm9051_set [write TX/RX checksum] wr 0x31 0x%02x wr 0x32 0x%02x, in _core_reset\n",
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
	#ifdef INT_CLKOUT
		netif_info(db, intr, db->ndev, "_reset [_core_reset] set DM9051_IPCOCR %02lx\n", IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
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
	struct device *dev = context;
	dev_warn(dev, "NOT SUPPORT: regmap_spi_async_write(context,...)\n");
	return -1;
}
static struct regmap_async *regmap_spi_async_alloc(void)
{ //.V510_COMPLEX
	//struct device *dev = context;
	//dev_warn(dev, "NOT SUPPORT: regmap_spi_async_alloc(void)\n");
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

	printk("\n");
	SHOW_PLAT_MODE(dev);
	SHOW_OPTION_MODE(dev);

	ret = dm9051_get_regs(db, DM9051_VIDL, buff, sizeof(buff));
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	return SHOW_MAP_CHIPID(dev, wid);
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
	SHOW_MAC(db, addr);
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
		mutex_lock(&db->spi_lockm); //.ee
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
	mutex_lock(&db->spi_lockm); //.
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
		mutex_lock(&db->spi_lockm); //.fcr
#endif
		ret = dm9051_update_fcr(db);
#if MI_FIX //fcr
		mutex_unlock(&db->spi_lockm);
#endif
		return ret;
	}

	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
					  pause->autoneg);
	phy_start_aneg(db->phydev);
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
        "dump BMSR register",
        "dump BMSR register",
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
	unsigned int val;

	data[0] = ndev->stats.rx_packets;
	data[1] = ndev->stats.tx_packets;
	data[2] = ndev->stats.rx_errors = db->bc.rx_err_counter;
	data[3] = ndev->stats.tx_errors = db->bc.tx_err_counter;
	data[4] = ndev->stats.rx_bytes;
	data[5] = ndev->stats.tx_bytes;
	data[6] = db->bc.fifo_rst_counter - 1; // Subtract Initial reset
	
	/* ethtool -S eth1, this is the extra dump parts */
#if MI_FIX //ee read
	mutex_lock(&db->spi_lockm);
#endif
	netif_info(db, tx_done, db->ndev, "rx_psckets: %llu\n", data[0]);
	dm9051_headlog_regs("dump rcr registers:", db, DM9051_RCR, DM9051_RCR);
	dm9051_headlog_regs("dump wdr registers:", db, 0x24, 0x25);
	dm9051_headlog_regs("dump mrr registers:", db, DM9051_MRRL, DM9051_MRRH);

	netif_info(db, tx_done, db->ndev, "%6d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in,
		db->xmit_in, db->xmit_tc, db->xmit_zc);
	netif_info(db, tx_done, db->ndev, "%6d [_THrd0] run %u Pkt %u zero-in %u, on-THrd %u Pkt %u\n", db->xmit_thrd0,
		db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd0, db->xmit_ttc0);
	netif_info(db, tx_done, db->ndev, "%6d [_THrd] run %u Pkt %u zero-in %u, on-THrd %u Pkt %u\n", db->xmit_thrd,
		db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd, db->xmit_ttc);

	/*PHY_LOG*/
	dm9051_phyread_headlog("bcr00", db, 0);
	dm9051_phyread_headlog("adv04", db, 4);
	dm9051_phyread_headlog("lpa05", db, 5);
	dm9051_phyread_headlog("phy17", db, 17);
	dm9051_phyread_headlog("phy20", db, 20);
	/*BMSR*/
	/*dm9051_phyread_headlog("bmsr", db, MII_BMSR);*/
	dm9051_phyread(db, MII_BMSR, &val);
	netif_warn(db, link, db->ndev, "bmsr %04x\n", val);
	data[7] = val;
	dm9051_phyread(db, MII_BMSR, &val);
	netif_warn(db, link, db->ndev, "bmsr %04x\n", val);
	data[8] = val;

	#if defined(DMPLUG_INT) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "dm9051 INT"); //#pragma message("dm9051 INT")
	#endif
	#if !defined(DMPLUG_INT) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "dm9051 POL"); //#pragma message("dm9051 POL")
	#endif
	#if defined(INT_CLKOUT) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "INT: INT_CLKOUT"); //#warning "INT: INT_CLKOUT"
	#endif
	#if defined(INT_TWO_STEP) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "INT: TWO_STEP"); //#warning "INT: TWO_STEP"
	#endif
	
	#if defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "WORKROUND: BMCR_WR"); //#pragma message("WORKROUND: BMCR_WR")
	#endif
	#if defined(DMCONF_MRR_WR) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "WORKROUND: MRR_WR"); //#pragma message("WORKROUND: MRR_WR")
	#endif
	
	#if defined(DMPLUG_CONTI) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "dm9051 CONTI"); //#pragma message("dm9051 CONTI")
	#endif

	#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "dm9051 PTP"); //#pragma message("dm9051 PTP")
	#endif
	#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
	netif_info(db, drv, db->ndev, "dm9051 PPS"); //#warning "dm9051 PPS"
	#endif

#if MI_FIX //ee write
	mutex_unlock(&db->spi_lockm);
#endif
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
	//k("_stop [dm9051_all_stop] set reg DM9051_GPCR, 0x%02x\n", (unsigned int)GPCR_GEP_CNTL);
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	//k("_stop [dm9051_all_stop] set reg DM9051_GPR, 0x%02x\n", (unsigned int)GPR_PHY_OFF);
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
#if 0
static int dm9051_all_upstart(struct board_info *db) //todo
{
	int ret;

	printk("_all_upstart\n"); //NOT to .netif_crit(db, rx_err, db->ndev, "_all_upstart\n");

	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;

	ret = dm9051_all_reinit(db); //up_restart
	if (ret)
		return ret;

	return 0;
}
#endif
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
	//struct net_device *ndev = db->ndev;

	db->bc.fifo_rst_counter++;
	netif_warn(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	netif_crit(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	//k("_[_all_restart] rxb work around done\n");
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
	do {	\
		if ((exp)) {	\
			yhndlr;	\
		} else {	\
			nhndlr;	\
		}	\
	} while(0)

static int trap_clr(struct board_info *db)
{
	db->bc.evaluate_rxb_counter = 0;
	return 0;
}

static void monitor_rxb0(struct board_info *db, unsigned int rxbyte)
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
			netif_crit(db, rx_status, db->ndev, "-. ........ warn, spenser board ...BL %02lx BH %02lx.......... on .%s %2d\n",
				   SCAN_BL(rxbyte), SCAN_BH(rxbyte), __func__, rxbz_counter);

		if (SCAN_BL(rxbyte) == 1 || SCAN_BL(rxbyte) == 0)
			return;

		netif_crit(db, rx_status, db->ndev, "_.moni   bf] %02x %02x\n", bf[0], bf[1]);						// tested
		netif_crit(db, rx_status, db->ndev, "_.moni rxbs] %02lx %02lx\n", SCAN_BL(rxbyte), SCAN_BH(rxbyte)); // tested

		inval_rxb[rxbz_counter] = SCAN_BL(rxbyte);
		rxbz_counter++;

		n += sprintf(pbff + n, "_.%s %2d]", __func__, rxbz_counter);
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
		netif_crit(db, rx_status, db->ndev, "%s\n", pbff);

		if (rxbz_counter >= TIMES_TO_RST)
		{
			rxbz_counter = 0;
			memset(inval_rxb, 0, sizeof(inval_rxb));
			netif_err(db, rx_status, db->ndev, "_[Less constrain of old SCAN_BL trap's, NOT _all_restart] only monitored.\n");
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

		inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
		db->bc.evaluate_rxb_counter++;

		if (db->bc.evaluate_rxb_counter == 1) {
			char head[HEAD_LOG_BUFSIZE];
			sprintf(head, "rxb 1st %d", db->bc.evaluate_rxb_counter); 
			dm9051_headlog_regs(head, db, 0x74, 0x75);
			dm9051_headlog_regs(head, db, 0x24, 0x25);
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
		netif_warn(db, rx_status, db->ndev, "%s\n", pbff);

		if (db->bc.evaluate_rxb_counter >= TIMES_TO_RST)
		{
			trap_clr(db);
			memset(inval_rxb, 0, sizeof(inval_rxb));
			return 1;
		}
	} while (0);
	return 0;
}

static int rx_break(struct board_info *db, unsigned int rxbyte, netdev_features_t features)
{
	monitor_rxb0(db, rxbyte);
	if (features & NETIF_F_RXCSUM)
	{
		//DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), return 0, 
		//	netif_warn(db, rx_status, db->ndev, "YES checksum check\n"); return -EINVAL);
		
		do {	\
			if (((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY)) {	\
				return 0;	\
			} else {	\
				netif_warn(db, rx_status, db->ndev, "YES checksum check\n"); return -EINVAL;	\
			}	\
		} while(0);
	}
	else
		DM9051_RX_BREAK((SCAN_BH(rxbyte) == DM9051_PKT_RDY), return 0,
			/*k("NO checksum check\n");*/ return -EINVAL);
}

static int rx_head_break(struct board_info *db)
{
	//struct net_device *ndev = db->ndev;
	int rxlen;

	u8 err_bits = RSR_ERR_BITS;
	
	/* 7 rxhead ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	static int before_slave_ptp_packets = 5;
	if (db->ptp_enable) {
		err_bits &= ~RSR_PTP_BITS; //_15888_ //To allow support "Enable PTP" must disable checksum_offload
	}
	#endif
	#endif

	rxlen = le16_to_cpu(db->rxhdr.rxlen);
	if (db->rxhdr.status & err_bits || rxlen > DM9051_PKT_MAX)
	{
		netif_warn(db, rx_status, db->ndev, "Err: [dm9.Monitor headbyte/status/rxlen %2x %2x %04x]\n",
			   db->rxhdr.headbyte,
			   db->rxhdr.status,
			   db->rxhdr.rxlen);

		if (db->rxhdr.headbyte != 0 &&  db->rxhdr.headbyte != 0x01) {
			netif_warn(db, rx_status, db->ndev, "Err: rxhdr-byte (%02x)\n",
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
			netif_warn(db, rx_status, db->ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
		}

		if (rxlen > DM9051_PKT_MAX) {
			netif_warn(db, rx_status, db->ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
		}

		return 1;
	}

	/* -rxhead ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	if (before_slave_ptp_packets && (!db->ptp_on) && (db->rxhdr.status & RSR_PTP_BITS)) {
		netif_warn(db, hw, db->ndev, "%d. On ptp_on is 0, ptp packet received!\n", before_slave_ptp_packets--);
	}
	#endif
	#endif
	return 0;
}

static int dm9051_loop_tx(struct board_info *db);

/* read packets from the fifo memory
 * return value,
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data invalide(NOT an error), caller escape from rx operation
 */
//static 
int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret, rxlen, padlen;
	unsigned int rxbyte;
	struct sk_buff *skb;
	int ntx;
	u8 *rdptr;
	int scanrr = 0;

	do
	{
#if 1
		/* In rx-loop
		 */
		sprintf(db->bc.head, "_THrd0");
		db->bc.mode = TX_THREAD0;
		ntx = dm9051_loop_tx(db); /* [More] and more tx better performance */
		if (ntx) {
			db->xmit_thrd0++;
			db->xmit_ttc0 += ntx;
			if (db->xmit_thrd0 <= 9) {
				netif_warn(db, tx_queued, db->ndev, "%2d [_THrd0] run %u Pkt %u zero-in %u, on-THrd %u Pkt %u\n", db->xmit_thrd0,
					db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd0, db->xmit_ttc0);
			}
		}
#endif
		ret = dm9051_read_mem_rxb(db, DM_SPI_MRCMDX, &rxbyte, 2);
		if (ret)
			return ret;

		if (rx_break(db, rxbyte, ndev->features))
		{
			if (trap_rxb(db, &rxbyte)) {
				dm9051_headlog_regs("rxb last", db, 0x74, 0x75);
				dm9051_headlog_regs("rxb last", db, 0x24, 0x25);
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

		/* 7.2dbg ptpc */
		#if 1 //0
		#ifdef DMPLUG_PTP
		dm9051_ptp_rx_packet_monitor(db, skb);
		#endif
		#endif

		skb->protocol = eth_type_trans(skb, db->ndev);

		/* 7.2 ptpc */
		#if 1 //0
		#ifdef DMPLUG_PTP
		if (db->rxhdr.status & RSR_RXTS_EN) {
			//if T1/T4,
		}

		//So when NOT T1/T4, we can skip tell tstamp (just an empty (virtual) one)
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

#if 1
	/* Ending rx-loop
	 */
	//int ntx;
	sprintf(db->bc.head, "_THrd");
	db->bc.mode = TX_THREAD;
	ntx = dm9051_loop_tx(db); /* more tx better performance */
	if (ntx) {
		db->xmit_thrd++;
		db->xmit_ttc += ntx;
		if (db->xmit_thrd <= 9) {
			netif_warn(db, tx_queued, db->ndev, "%2d [_THrd] run %u Pkt %u zero-in %u, on-THrd %u Pkt %u\n", db->xmit_thrd,
				db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd, db->xmit_ttc);
		}
	}
#endif
	return scanrr;
}

#if !defined(DMPLUG_CONTI)
#ifdef DM9051_SKB_PROTECT
static struct sk_buff *EXPAND_SKB(struct sk_buff *skb, unsigned int pad)
{	
	struct sk_buff *skb2;

	skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
	if (skb2) {
		dev_kfree_skb(skb);
		return skb2;
	}

	//.netif_warn(db, tx_queued, db->ndev, "[WB_SUPPORT] warn on len %d, skb_copy_expand get memory leak!\n", skb->len);
	return skb;
}
#endif

/* particulars, wb mode*/
struct sk_buff *dm9051_pad_txreq(struct board_info *db, struct sk_buff *skb)
{
//#if !defined(_DMPLUG_CONTI)
	db->data_len = skb->len;
	db->pad = (dm9051_modedata->skb_wb_mode && (skb->len & 1)) ? 1 : 0; //'~wb'
#ifdef DM9051_SKB_PROTECT
	if (db->pad)
		skb = EXPAND_SKB(skb, db->pad);
#endif
//#endif
	return skb;
}

static int dm9051_single_tx(struct board_info *db, u8 *p)
{
	int ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, p, db->data_len + db->pad); //'!wb'
	if (ret)
		return ret;

	return dm9051_set_regs(db, DM9051_TXPLL, &db->data_len, 2); //address of structure's field
}

static int dm9051_req_tx(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_TCR, db->tcr_wr); //base with TCR_TXREQ
}

static int TX_SEND(struct board_info *db, struct sk_buff *skb)
{
	int ret;

	/*
	 * #define EXPEND_LEN(datlen,pd) (datlen + pd)
	 * #define WRITE_SKB(db,p,len) dm9051_write_mem_cache(db,p,len)
	 * if (!dm9051_modedata->skb_wb_mode) {
	 *   ret = WRITE_SKB(db, skb, skb->len);
	 *   ret = dm9051_single_tx(db, skb->len);
	 * }
	 * else {
	 *   pad = ...
	 *   skb = _EXPAND_SKB(skb, pad);
	 *   ret = WRITE_SKB(db, skb, EXPEND_LEN(data_len, pad);
	 *   ret = dm9051_single_tx(db, skb, data_len, pad);
	 * }
	 * ret = dm9051_req_tx(db);
	 */
	do {
		ret = dm9051_single_tx(db, skb->data);
		if (ret)
			break;

		ret = dm9051_req_tx(db);
	} while(0);

	if (!ret) {
		struct net_device *ndev = db->ndev;
		ndev->stats.tx_bytes += db->data_len;
		ndev->stats.tx_packets++;
	}

	return ret;
}
#endif

int TX_SENDC(struct board_info *db, struct sk_buff *skb)
{
	int ret;

#if !defined(DMPLUG_CONTI)
#if defined(STICK_SKB_CHG_NOTE)
	skb = dm9051_pad_txreq(db, skb);
#endif
#endif

	/* 6 tx ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	//u8 message_type = 
	dm9051_ptp_txreq(db, skb);
	#endif
	#endif

#if !defined(DMPLUG_CONTI)
	ret = TX_SEND(db, skb);
#else
	ret = TX_MODE2_CONTI_TCR(db, skb, econf->tx_timeout_us);
#endif
	
	if ((db->bc.mode == TX_DELAY && db->xmit_in <=9) || 
		(db->bc.mode == TX_THREAD  && db->xmit_thrd <= 9) ||
		(db->bc.mode == TX_THREAD0  && db->xmit_thrd0 <= 9)) {
		netif_info(db, tx_queued, db->ndev, "%s. tx_send end_wr %02x\n", db->bc.head, db->tcr_wr);
	}

	/* 6.1 tx ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	dm9051_ptp_txreq_hwtstamp(db, skb);
	#endif
	#endif

#if defined(STICK_SKB_CHG_NOTE)
	dev_kfree_skb(skb);
#endif
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
			if (TX_SENDC(db, skb)) {
				db->bc.tx_err_counter++;
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

/* threaded_irq */

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
	int ntx;

	mutex_lock(&db->spi_lockm);

	sprintf(db->bc.head, "_dely");
	db->bc.mode = TX_DELAY;
	ntx = dm9051_loop_tx(db);

	db->xmit_tc += ntx;
	db->xmit_zc += ntx ? 0 : 1;
	if (ntx) {
		db->xmit_in++;
		if (db->xmit_in <= 9) {
			netif_info(db, tx_queued, db->ndev, "%2d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in,
				db->xmit_in, db->xmit_tc, db->xmit_zc);
		}
	}

//	int result;
//	result = 
//	if (result < 0)
//		n(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

/* (looping rx and tx) send packets and read packets from the fifo memory
 * return value,
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data invalide(NOT an error), caller escape from rx operation
 */
//static int dm9051_delayp_looping_rx_tx(struct board_info *db) //.looping_rx_tx()
//{
//	/*
//	 * is:
//	 *     while ((result = dm9051_loop_rx(db)) > 0) ;
//	 *     return result;
//	 */
//	int result;

//	do {
//		result = dm9051_loop_rx(db); /* threaded rx */
//	} while(result > 0);

//	return result;
//}

#if 1
//static void dm9051_rx_xplat_enable(struct board_info *db)
//{
//	dm9051_enable_interrupt(db);
//}
//static void dm9051_rx_xplat_loop(struct board_info *db)
//{
//	int ret;
//	dm9051_delayp_looping_rx_tx(db); //.looping_rx_tx()
//.	ret =
//.	if (ret < 0)
//.		return;
//	dm9051_enable_interrupt(db); //"dm9051_rx_xplat_enable(struct board_info *db)"
//}

//static int dm9051_rx_xplat_disable(struct board_info *db)
//{
//	int result = dm9051_disable_interrupt(db);
//	if (result)
//		return result;

//	result = dm9051_clear_interrupt(db);
//	if (result)
//		return result;
//	return result;
//}

/* Interrupt: Interrupt work */
/* dm9051 rx_threaded irq */

void dm9051_thread_irq(void *pw) //.(macro)_rx_tx_plat() //dm9051_rx_threaded_irq()
{
	struct board_info *db = pw;
	int result;

	mutex_lock(&db->spi_lockm);

	//[REAL.'MI_FIX'] //(result is as 'dm9051_rx_xplat_disable'(db))
	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	/* return value from _dm9051_delayp_looping_rx_tx(),
	 *    0 - no error, caller need stop further rx operation
	 *  -EBUSY - read data invalide(NOT an error), caller escape from rx operation
	 */
	while (dm9051_loop_rx(db) > 0) ; //dm9051_delayp_looping_rx_tx(db);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);
	//_thread_servicep_done = 1;
	//return IRQ_HANDLED;
}

/* threaded_irq */

int thread_servicep_done = 1;
int thread_servicep_re_enter = 0;

irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw)
{
	struct board_info *db = pw;

	if (thread_servicep_done) {
		thread_servicep_done = 0;
		if (!thread_servicep_re_enter)
			netif_crit(db, intr, db->ndev, "_.PLAT.WARN   [%s] this-first-enter %d\n", __func__, thread_servicep_re_enter++);
		dm9051_thread_irq(db); //(voidirq, pw) //.(macro)_rx_tx_plat()
		thread_servicep_done = 1;
	} else {
		//.if (thread_servicep_re_enter <= 9)
		netif_err(db, intr, db->ndev, "_.PLAT.WARN   [%s] re-enter %d\n", __func__, thread_servicep_re_enter++);
	}
	return IRQ_HANDLED;
}
#endif

static int dm9051_all_start_init(struct board_info *db)
{
//before [spi_lockm]
//use [spi_lockm]
	int ret;

	#if MI_FIX
	mutex_lock(&db->spi_lockm); //.open
	#endif

	ret = dm9051_all_start(db);
	if (ret)
		return ret;

	/* -open ptpc */
	#if 1 //0
	#ifdef DMPLUG_PTP
	if (db->ptp_on) {
		//_15888_ 
		u32 rate_reg = dm9051_get_rate_reg(db); //15888, dm9051_get_rate_reg(db);
		netif_warn(db, hw, db->ndev, "Pre-RateReg value = 0x%08X\n", rate_reg);
	}
	#endif
	#endif

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	return ret;
}

static int dm9051_all_start_intr(struct board_info *db)
{
	int ret;

	#if MI_FIX
	mutex_lock(&db->spi_lockm);//.open's
	#endif

	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db));
	if (ret)
		goto intr_unlck;

	ret = dm9051_enable_interrupt(db);

intr_unlck:
	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif
	
	return ret;
}

static int dm9051_all_stop_mlock(struct board_info *db)
{
	int ret;

	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	ret = dm9051_all_stop(db);

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	return ret;
}

#if defined(DMPLUG_INT)
/*
 * Interrupt: 
 */
static int dm9051_req_irq(struct board_info *db, irq_handler_t handler)
{
	struct spi_device *spi = db->spidev;
	int ret;

	netif_crit(db, intr, db->ndev, "request_irq(INT_THREAD)\n");
	thread_servicep_re_enter = 0; //used in 'dm9051_rx_threaded_plat'
	ret = request_threaded_irq(spi->irq, NULL, handler, //'dm9051_rx_threaded_plat'
							   get_dts_irqf(db) | IRQF_ONESHOT,
							   db->ndev->name, db);
	if (ret < 0)
		netif_err(db, intr, db->ndev, "failed to rx request threaded irq setup\n");
	return ret;
}
#endif

static int dm9051_threaded_irq(struct board_info *db, irq_handler_t handler)
{
#ifndef DMPLUG_INT
	if (dm9051_poll_supp())
		return dm9051_poll_sch(db);
	return 0;
#endif

#if defined(DMPLUG_INT)
	if (dm9051_int2_supp())
		return dm9051_int2_irq(db, dm9051_rx_int2_delay);

	return dm9051_req_irq(db, handler);
#endif
}

#if defined(DMPLUG_INT)
static void dm9051_thread_irq_free(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	#ifdef INT_TWO_STEP
	cancel_delayed_work_sync(&db->irq_servicep);
	#endif //_INT_TWO_STEP

	free_irq(db->spidev->irq, db);
	netif_err(db, intr, ndev, "_[stop] remove: free irq %d\n", db->spidev->irq);
}
#endif

static void dm9051_free_irqworks(struct board_info *db)
{
	/* schedule delay work */
	#if defined(DMPLUG_INT)
	/*
	 * Interrupt: 
	 */
	dm9051_thread_irq_free(db->ndev);
	#else
	/*
	 * Polling: 
	 */
	cancel_delayed_work_sync(&db->irq_workp);
	#endif
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

	SHOW_OPEN(db);

	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	PTP_INIT_RCR(db);
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */

	ret = dm9051_all_start_init(db); /* such as all start */
	if (ret)
		return ret;

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	netif_wake_queue(ndev);

	ret = dm9051_threaded_irq(db, dm9051_rx_threaded_plat); /* near the bottom */
	if (ret < 0) {
		phy_stop(db->phydev); //of 'dm9051_core_clear(db)' //
		return ret;
	}

	ret = dm9051_all_start_intr(db); /* near the bottom */
	if (ret) {
		phy_stop(db->phydev);
		dm9051_free_irqworks(db);
		return ret;
	}

	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
////static int dm9051_stop001(struct net_device *ndev)
////{
//	mutex_lock(&db->spi_lockm);
//	ret = dm9051_all_stop(db);
//	mutex_unlock(&db->spi_lockm);
//	if (ret)
//		return ret;
////	mutex_lock(&db->spi_lockm);
//	phy_stop(db->phydev);
////	mutex_unlock(&db->spi_lockm);
////}
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	netif_err(db, probe, ndev, "dm9051_stop\n"); //as 'probe' type, original dev_info()

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

	dm9051_free_irqworks(db);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return dm9051_all_stop_mlock(db);
}

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
	skb_tx_timestamp(skb); //Spenser - Report software Timestamp v.s. - Report HW Timestamp
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
		netif_crit(db, hw, db->ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI)
	{
		rcr |= RCR_ALL;
		netif_crit(db, hw, db->ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
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
#ifdef DMPLUG_PTP
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
	.ndo_do_ioctl = dm9051_ptp_netdev_ioctl, //_15888_
#else
	.ndo_eth_ioctl = dm9051_ptp_netdev_ioctl, //_15888_
#endif
#endif
};

static void CHKSUM_PTP_NDEV(struct board_info *db, struct net_device *ndev)
{
	/* Set default features */
	if (dm9051_modedata->checksuming)
		ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;

	/* 2 ptpc */
	PTP_NEW(db, ndev);
	if (db->ptp_enable)
		ndev->features &= ~(NETIF_F_HW_CSUM | NETIF_F_RXCSUM); //"Run PTP must COERCE to disable checksum_offload"

	ndev->hw_features |= ndev->features;
}

static void DM9051_PROBE_DLYSETUP(struct board_info *db)
{
	#if defined(DMPLUG_INT)
	/*
	 * Interrupt: 
	 */
	#ifdef INT_TWO_STEP
		PROBE_INT2_DLY_SETUP(db);
	#endif
	#else
	/*
	 * Polling: 
	 */
	PROBE_POLL_SETUP(db);
	#endif
}

static void MRR_WR_FCR_UPSTART(struct board_info *db)
{
	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif
#ifdef DMCONF_MRR_WR
	//int ret = dm9051_all_upstart(db);
	//if (ret)
	//	goto dnf_end;
	printk("_all_upstart\n"); //NOT to .netif_crit(db, rx_err, db->ndev, "_all_upstart\n");
	do {
		int ret = dm9051_ncr_reset(db);
		if (ret)
			goto dnf_end;
		
	//=	ret = dm9051_all_reinit(db); //up_restart
	#if 1		
		ret = dm9051_core_init(db);
		if (ret)
			goto dnf_end;
	#endif
		ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db));
		if (ret)
			goto dnf_end;

		ret = dm9051_enable_interrupt(db);
		if (ret)
			goto dnf_end;

		ret = dm9051_subconcl_and_rerxctrl(db);
		if (ret)
			goto dnf_end;
	} while(0);
	dm9051_update_fcr(db);
dnf_end:
	netif_crit(db, rx_err, db->ndev, "DMCONF_MRR_WR operation done!\n");
#else //DMCONF_MRR_WR
	dm9051_update_fcr(db);
	netif_crit(db, rx_err, db->ndev, "DMCONF_MRR_WR operation not applied!\n");
#endif
	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif
}

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;

	trap_clr(db);
	db->bc.nRxcF = 0;

	db->csum_gen_val = 0; //disabling
	db->csum_rcv_val = 0; //disabling

	db->n_automdix = 0; //log-reset
	db->stop_automdix_flag = 0;
	db->automdix_log[0][0] = 0;
	db->automdix_log[1][0] = 0;
	db->automdix_log[2][0] = 0;
	db->mdi = 0x0830;
	
	db->tcr_wr = TCR_TXREQ; //pre-defined
	
	db->xmit_in = 0;
	db->xmit_tc = 0;
	db->xmit_zc = 0;
	db->xmit_thrd0 = 0;
	db->xmit_ttc0 = 0;
	db->xmit_thrd = 0;
	db->xmit_ttc = 0;
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

	if (db->phydev->link)
	{
		printk("\n");
		printk("LOCK_MUTEX\n");
	}

	phy_print_status(db->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link)
	{
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
		MRR_WR_FCR_UPSTART(db);

		printk("UNLOCK_MUTEX\n");
		printk("\n");
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

	db->spidev = spi;
	db->ndev = ndev;

	CHKSUM_PTP_NDEV(db, ndev);

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;

	/* version log */
	SHOW_DEVLOG_REFER_BEGIN(dev, db);

	//[NETIF_MSG_HW is play for phylib...]
	//db->msg_enable = 0;
	db->msg_enable = NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP | 
		NETIF_MSG_RX_ERR | NETIF_MSG_TX_ERR | NETIF_MSG_INTR | NETIF_MSG_RX_STATUS | NETIF_MSG_PKTDATA | NETIF_MSG_HW /*| NETIF_MSG_HW*/; //0;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

	DM9051_PROBE_DLYSETUP(db);

	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	SHOW_DEVLOG_MODE(dev);

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

	ret = dm9051_map_etherdev_par(ndev, db);
	if (ret < 0)
		return ret;

	ret = dm9051_mdio_register(db);
	if (ret)
		return ret;

	dm9051_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = devm_register_netdev(dev, ndev);
	if (ret)
	{
		//phy_disconnect(db->phydev);
		return dev_err_probe(dev, ret, "device register failed");
	}
	
	//if (netif_running(ndev)) ..
	SHOW_LOG_REFER_BEGIN(db);

	/* 2.1 ptpc */
	PTP_INIT(db);

	return dm9051_phy_connect(db); /* phy connect in the bottom */
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,0) || \
	LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
static int dm9051_drv_remove(struct spi_device *spi)
#else
static void dm9051_drv_remove(struct spi_device *spi)
#endif
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	printk("\n");
	netif_warn(db, probe, db->ndev, "_[phy] remove: disconnect\r\n"); //as 'probe' type
	phy_disconnect(db->phydev);

	/* 3 ptpc */
	PTP_END(db);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,0) || \
	LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
	return 0;
#endif
}

static const struct of_device_id dm9051_match_table[] = {
	{.compatible = "davicom,dm9051"},
	{}};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
MODULE_DEVICE_TABLE(of, dm9051_match_table);
#endif

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
