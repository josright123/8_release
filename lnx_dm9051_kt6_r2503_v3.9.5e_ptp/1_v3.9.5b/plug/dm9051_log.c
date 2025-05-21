// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
 
/*
 * User notice:
 *   To add log function, 
 *   And add this file to project
 *   And modify Makefile to insert this file 
 *   into the project build.
 * 
 * In Makefile
 * Change
 *   dm9051a-objs := dm9051.o dm9051_plug.o
 * to
 *   dm9051a-objs := dm9051.o dm9051_plug.o dm9051_log.o
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
 
//#include <linux/etherdevice.h>
//#include <linux/ethtool.h>
//#include <linux/interrupt.h>
//#include <linux/iopoll.h>
//#include <linux/irq.h>
//#include <linux/mii.h>
//#include <linux/module.h>
//#include <linux/netdevice.h>
//#include <linux/phy.h>
//#include <linux/regmap.h>
//#include <linux/skbuff.h>
//#include <linux/spinlock.h>
//#include <linux/spi/spi.h>
//#include <linux/types.h>
//#include <linux/of.h>
#define SECOND_MAIN //(sec)
#include "../dm9051.h"

//#warning "DMPLUG: dm9051 plug-in log function"

void SHOW_DEVLOG_REFER_BEGIN(struct device *dev, struct board_info *db)
{
	printk("\n");
	dev_info(dev, "dev_info Version\n");
	dev_notice(dev, "dev_notice Version\n");
	dev_warn(dev, "dev_warn Version\n");
	dev_crit(dev, "dev_crit Version\n");
	dev_err(dev, "dev_err Version\n");
	printk("\n");

	/* conti */
	TX_CONTI_NEW(db);
	/* 2.0 ptpc */
	if (db->ptp_enable) {
		dev_info(&db->spidev->dev, "DMPLUG PTP Version\n");
		dev_info(&db->spidev->dev, "Enable PTP must COERCE to disable checksum_offload\n");
	}
}

void SHOW_LOG_REFER_BEGIN(struct board_info *db)
{
	printk("\n");
	netif_info(db, probe, db->ndev, "netif_info Version\n");
	netif_notice(db, probe, db->ndev, "netif_notice Version\n");
	netif_warn(db, probe, db->ndev, "netif_warn Version\n");
	netif_crit(db, probe, db->ndev, "netif_crit Version\n");
	netif_err(db, probe, db->ndev, "netif_err Version\n");
//	netif_dbg(db, probe, ndev, "netif_dbg Version\n");
//	netdev_dbg(ndev, "netdev_dbg Version\n");
}

static void USER_CONFIG(struct device *dev, struct board_info *db, char *str)
{
	if (dev)
		dev_warn(dev, "%s\n", str);
	else if (db)
		netif_info(db, drv, db->ndev, "%s\n", str);
}

void SHOW_ALL_USER_CONFIG(struct device *dev, struct board_info *db)
{
	#if defined(DMPLUG_INT) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "dm9051 INT"); //netif_info(db, drv, db->ndev, "dm9051 INT"); //#pragma message("dm9051 INT")
	#endif
	#if !defined(DMPLUG_INT) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "dm9051 POL"); //netif_info(db, drv, db->ndev, "dm9051 POL"); //#pragma message("dm9051 POL")
	#endif
	#if defined(INT_CLKOUT) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "INT: INT_CLKOUT"); //netif_info(db, drv, db->ndev, "INT: INT_CLKOUT"); //#warning "INT: INT_CLKOUT"
	#endif
	#if defined(INT_TWO_STEP) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "INT: TWO_STEP"); //netif_info(db, drv, db->ndev, "INT: TWO_STEP"); //#warning "INT: TWO_STEP"
	#endif
	
	#if defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "WORKROUND: BMCR_WR"); //netif_info(db, drv, db->ndev, "WORKROUND: BMCR_WR"); //#pragma message("WORKROUND: BMCR_WR")
	#endif
	#if defined(DMCONF_MRR_WR) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "WORKROUND: MRR_WR"); //netif_info(db, drv, db->ndev, "WORKROUND: MRR_WR"); //#pragma message("WORKROUND: MRR_WR")
	#endif
	
	#if defined(DMPLUG_CONTI) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "dm9051 CONTI"); //netif_info(db, drv, db->ndev, "dm9051 CONTI"); //#pragma message("dm9051 CONTI")
	#endif

	#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "dm9051 PTP"); //netif_info(db, drv, db->ndev, "dm9051 PTP"); //#pragma message("dm9051 PTP")
	#endif
	#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
	USER_CONFIG(dev, db, "dm9051 PPS"); //netif_info(db, drv, db->ndev, "dm9051 PPS"); //#warning "dm9051 PPS"
	#endif
}

/*
 * Driver logs: 
 */
static void SHOW_DRIVER(struct device *dev)
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

static void SHOW_DTS_SPEED(struct device *dev)
{
	/* [dbg] spi.speed */
	unsigned int speed;
	of_property_read_u32(dev->of_node, "spi-max-frequency", &speed);
	dev_info(dev, "SPI speed from DTS: %d Hz\n", speed);
}

static void SHOW_DTS_INT(struct device *dev)
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

#define DEV_INFO_TX_ALIGN(dev) \
		dev_warn(dev, "TX: %s blk %u\n", \
			dm9051_modedata->align.burst_mode_info, \
			dm9051_modedata->align.tx_blk)
#define DEV_INFO_RX_ALIGN(dev) \
		dev_warn(dev, "RX: %s blk %u\n", \
			dm9051_modedata->align.burst_mode_info, \
			dm9051_modedata->align.rx_blk)

static void SHOW_ENG_OPTION_MODE(struct device *dev)
{
	dev_info(dev, "Check TX End: %llu, TX mode= %s mode, DRVR= %s, %s\n", econf->tx_timeout_us, dmplug_tx,
			econf->force_monitor_rxb ? "monitor rxb" : "silence rxb",
			econf->force_monitor_tx_timeout ? "monitor tx_timeout" : "silence tx_ec");
}

void SHOW_DEVLOG_MODE(struct device *dev)
{
	SHOW_DRIVER(dev);

	SHOW_DTS_SPEED(dev);
	//SHOW_ALL_USER_CONFIG(dev, NULL);
//	SHOW_RX_MATCH_MODE(dev);
//	SHOW_RX_INT_MODE(dev);
	SHOW_DTS_INT(dev);

	SHOW_CONFIG_MODE(dev);
	DEV_INFO_TX_ALIGN(dev);
	DEV_INFO_RX_ALIGN(dev);
	SHOW_ENG_OPTION_MODE(dev);
}

//-
void SHOW_PLAT_MODE(struct device *dev)
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

void SHOW_MAC(struct board_info *db, u8 *addr)
{
	dev_warn(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 addr[0], addr[1], addr[2],
			 addr[3], addr[4], addr[5]);
}

void SHOW_OPEN(struct board_info *db)
{
	printk("\n");
	netif_warn(db, drv, db->ndev, "dm9051_open\n");
	SHOW_ALL_USER_CONFIG(NULL, db);
//	netif_info(db, drv, db->ndev, "Davicom: %s", dmplug_rx_mach);
//	#if defined(DMPLUG_INT)
//	netif_info(db, drv, db->ndev, "Davicom: %s", dmplug_intterrpt2);
//	#endif
	/* amdix_log_reset(db); */ //(to be determined)
}

void SHOW_MONITOR_RXC(struct board_info *db, int scanrr)
{
#define PRINT_BURST_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[rx/tx %s mode] nRxc %d\n", \
			dmplug_tx, \
			n)
#define PRINT_ALIGN_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[TX %s mode][Alignment RX %u, Alignment RX %u] nRxc %d\n", \
			dmplug_tx, \
			dm9051_modedata->align.rx_blk, \
			dm9051_modedata->align.tx_blk, \
			n)

	if (econf->force_monitor_rxc && scanrr && db->bc.nRxcF < 25)
	{
		db->bc.nRxcF += scanrr;
		
		if (dm9051_modedata->align.burst_mode == BURST_MODE_FULL)
			PRINT_BURST_INFO(db->bc.nRxcF);
		else if (dm9051_modedata->align.burst_mode == BURST_MODE_ALIGN)
			PRINT_ALIGN_INFO(db->bc.nRxcF);
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

/* dm9051_phyread.EXTEND for 'link'
 */
int dm9051_phyread_headlog(char *head, struct board_info *db, unsigned int reg)
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

void monitor_rxb0(struct board_info *db, unsigned int rxbyte)
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

MODULE_DESCRIPTION("Davicom DM9051 driver, Log-function");
MODULE_LICENSE("GPL");
