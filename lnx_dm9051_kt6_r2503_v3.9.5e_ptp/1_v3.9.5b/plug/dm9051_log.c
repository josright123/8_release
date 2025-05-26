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
 
//#define SECOND_MAIN //(sec)
#include "../dm9051.h"

extern const struct plat_cnf_info *plat_cnf;
extern const struct param_config *param;

#define dmplug_tx "normal"
#define tx_conti_ver(b)

#if defined(DMPLUG_CONTI)
#define dmplug_tx "continue"
#define tx_conti_ver(b) tx_continue_ver(b)
void tx_continue_ver(struct board_info *db);
#endif

//#warning "DMPLUG: dm9051 plug-in log function"
struct driver_rel_info {
	const char *release_version;
};
const struct driver_rel_info driver_info = {
	.release_version = "lnx_dm9051_kt6631_r2502_v3.9.1",
};

void show_dev_begin(struct device *dev, struct board_info *db) //.SHOW_DEVLOG_REFER_BEGIN
{
	printk("\n");
	dev_info(dev, "dev_info Version\n");
	dev_notice(dev, "dev_notice Version\n");
	dev_warn(dev, "dev_warn Version\n");
	dev_crit(dev, "dev_crit Version\n");
	dev_err(dev, "dev_err Version\n");
	printk("\n");

	/* conti */
	tx_conti_ver(db);
	/* 2.0 ptpc */
	#ifdef DMPLUG_PTP
	if (db->ptp_enable) {
		dev_info(&db->spidev->dev, "DMPLUG PTP Version\n");
		dev_info(&db->spidev->dev, "Enable PTP must COERCE to disable checksum_offload\n");
	}
	#endif
}

void show_log(struct board_info *db) //.SHOW_LOG_REFER_BEGIN
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
	dev_warn(dev, "Davicom: %s (64 bit)", driver_info.release_version);
	#else
	// 32-bit specific code
	dev_crit(dev, "Davicom: %s (64 bit, warn: but kernel config has no CONFIG_64BIT?)", driver_info.release_version);
	#endif
#else
	// 32-bit code
	#ifdef CONFIG_64BIT
	// 64-bit specific code
	dev_crit(dev, "Davicom: %s (32 bit, warn: but kernel config has CONFIG_64BIT?))", driver_info.release_version);
	#else
	// 32-bit specific code
	dev_warn(dev, "Davicom: %s (32 bit)", driver_info.release_version);
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
			plat_cnf->align.burst_mode_info, \
			plat_cnf->align.tx_blk)
#define DEV_INFO_RX_ALIGN(dev) \
		dev_warn(dev, "RX: %s blk %u\n", \
			plat_cnf->align.burst_mode_info, \
			plat_cnf->align.rx_blk)

static void SHOW_ENG_OPTION_MODE(struct device *dev)
{
	dev_info(dev, "Check TX End: %llu, TX mode= %s mode, DRVR= %s, %s\n", param->tx_timeout_us, dmplug_tx,
			param->force_monitor_rxb ? "monitor rxb" : "silence rxb",
			param->force_monitor_tx_timeout ? "monitor tx_timeout" : "silence tx_ec");
}

void show_mode(struct device *dev) //.SHOW_DEVLOG_MODE
{
	SHOW_DRIVER(dev);

	SHOW_DTS_SPEED(dev);
//	SHOW_RX_MATCH_MODE(dev);
//	SHOW_RX_INT_MODE(dev);
	SHOW_DTS_INT(dev);

	SHOW_CONFIG_MODE(dev);
	DEV_INFO_TX_ALIGN(dev);
	DEV_INFO_RX_ALIGN(dev);
	SHOW_ENG_OPTION_MODE(dev);
}

//-
void show_pmode(struct device *dev) //.SHOW_PLAT_MODE
{
	dev_info(dev, "Davicom: %s", plat_cnf->test_info);
}

void show_mac(struct board_info *db, u8 *addr) //.SHOW_MAC
{
	dev_warn(&db->spidev->dev, "Power-on chip MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 addr[0], addr[1], addr[2],
			 addr[3], addr[4], addr[5]);
}

void show_rxc(struct board_info *db, int scanrr) //.SHOW_MONITOR_RXC
{
#define PRINT_BURST_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[rx/tx %s mode] nRxc %d\n", \
			dmplug_tx, \
			n)
#define PRINT_ALIGN_INFO(n) \
		netif_warn(db, rx_status, db->ndev, "___[TX %s mode][Alignment RX %u, Alignment RX %u] nRxc %d\n", \
			dmplug_tx, \
			plat_cnf->align.rx_blk, \
			plat_cnf->align.tx_blk, \
			n)

	if (param->force_monitor_rxc && scanrr && db->bc.nRxcF < 25)
	{
		db->bc.nRxcF += scanrr;
		
		if (plat_cnf->align.burst_mode == BURST_MODE_FULL)
			PRINT_BURST_INFO(db->bc.nRxcF);
		else if (plat_cnf->align.burst_mode == BURST_MODE_ALIGN)
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

static void show_log_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2) //.dm9051_headlog_regs
{
	memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, head);
	dm9051_dump_reg2s(db, reg1, reg2);
}
void dm9051_log_rxptr(char *head, struct board_info *db)
{
	show_log_regs(head, db, DM9051_MRRL, DM9051_MRRH);
	show_log_regs(head, db, 0x24, 0x25);
}

/* dm9051_phyread.EXTEND for 'link'
 */
static int show_log_phy(char *head, struct board_info *db, unsigned int reg) //.dm9051_phyread_headlog
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
void dm9051_log_phy(struct board_info *db)
{
	show_log_phy("bcr00", db, 0);
	show_log_phy("adv04", db, 4);
	show_log_phy("lpa05", db, 5);
	show_log_phy("phy17", db, 17);
	show_log_phy("phy20", db, 20);
}

void dump_data(struct board_info *db, u8 *packet_data, int packet_len) //.dm9051_dump_data1
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

void show_rxb(struct board_info *db, unsigned int rxbyte) //.monitor_rxb0
{
	if (param->force_monitor_rxb)
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
