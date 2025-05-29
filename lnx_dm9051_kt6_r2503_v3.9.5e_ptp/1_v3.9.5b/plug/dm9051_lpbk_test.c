// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
 
/*
 * User notice:
 *   To add loop test function, 
 *   Add 
 * 	#define DMPLUG_LPBK_TST
 *      in dm9051.h
 *   And add this file to project
 *   And modify Makefile to insert this file 
 *   into the project build.
 * 
 * In Makefile
 * Change
 *   dm9051a-objs := dm9051.o
 * to
 *   dm9051a-objs := dm9051.o plug/dm9051_lpbk_test.o
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
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>
#include "../dm9051.h"

//#warning "DMPLUG: dm9051 mac loopback test implement source file - dm9051_lpbk_test.c"
#pragma message("DMPLUG: dm9051 mac loopback test implement source file - dm9051_lpbk_test.c")

extern const struct plat_cnf_info *plat_cnf;

/*
 * mac loopback test: 
 */

//static int dm9051_single_tx(struct board_info *db, u8 *p);
//static int dm9051_req_tx(struct board_info *db);
//static int rx_break(struct board_info *db, unsigned int rxbyte, netdev_features_t features);
//static int trap_rxb(struct board_info *db, unsigned int *prxbyte);
//static int trap_clr(struct board_info *db);
//static int rx_head_break(struct board_info *db);
static int amdix_bmsr_change(struct board_info *db, unsigned int *val)
{
	static unsigned int bmsr = 0x0000; //0xffff;

	int ret = dm9051_phyread(db, MII_LPA, val);
	if (ret)
		return ret;
	db->lpa = *val;

	ret = dm9051_phyread(db, MII_BMSR, val);
	if (ret)
		return ret;
	db->bmsr = *val;

	if (db->bmsr != bmsr) {
		bmsr = db->bmsr;
		return 1;
	}
	return 0;
}
static void dm9051_phyread_bmsr_loop(struct board_info *db, unsigned int reg, unsigned int *val) //reg, not used!
{
	int i = 0;
	while (1) {
		while (amdix_bmsr_change(db, val))
			;

		//while (!amdix_bmsr_change(db)) {		
		//}
		if (db->bmsr & BIT(2)) {
			netif_warn(db, link, db->ndev, "<fund_phylib. rd.bmsr %04x [lpa] %04x> DONE...\n",
						db->bmsr, db->lpa);
			break;
		}

		msleep(1);
		if (i++ > 1000) {
			netif_warn(db, link, db->ndev, "<fund_phylib. rd.bmsr %04x [lpa] %04x> TimeOut...\n",
						db->bmsr, db->lpa);
			break;
		}
	}

	//return ret;	
}
void dump_data_001(struct board_info *db, u8 *packet_data, int packet_len) //.dm9051_dump_data1
{
	int i, j, rowsize = 32;
	int splen; //index of start row
	int rlen; //remain/row length 
	char line[120];
	
	//[rlen = packet_len > 16 ? 16 : packet_len;]

	netif_info(db, pktdata, db->ndev, "%s\n", db->bc.head);
	for (i = 0; i < packet_len; i += rlen) {
		//rlen = print_line(packet_data+i, min(rowsize, skb->len - i)); ...
		rlen =  packet_len - i;
		if (rlen >= rowsize) rlen = rowsize;

		splen = 0;
		splen += sprintf(line + splen, "%03d", i);
		for (j = 0; j < rlen; j++) {
			if (!(j % 8)) splen += sprintf(line + splen, " ");
			if (!(j % 16)) splen += sprintf(line + splen, " ");
			splen += sprintf(line + splen, " %02x", packet_data[i+j]);
		}
		netif_info(db, pktdata, db->ndev, "%s\n", line);
	}
}
int dump_regs(struct board_info *db)
{
	//get_regs
	int i, j;
//	u8 buff[128-16];
	int splen; //index of start row
	int row_count, reg_count = 128;
	unsigned int val;
	char mark[128];
	char line[120];
	int ret;

//	ret = dm9051_get_regs(db, 0, buff, sizeof(buff));
//	if (ret < 0)
//		return ret;

//	printk("[1]\n");
//	for (i = 0; i < sizeof(buff); i += 16)
//		printk("%08x  %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n", i,
//			buff[i], buff[i+1], buff[i+2], buff[i+3],
//			buff[i+4], buff[i+5], buff[i+6], buff[i+7],
//			buff[i+8], buff[i+9], buff[i+10], buff[i+11],
//			buff[i+12], buff[i+13], buff[i+14], buff[i+15]
//			);

//	printk("[2]\n");
	memset(mark, 1, sizeof(mark));
	mark[0x71] = mark[0x72] = 0; //mark[113] = mark[114] = 0;
	for (i = 0; i < reg_count; i += row_count) {
		row_count = reg_count - i;
		if (row_count >= 16) row_count = 16;

		splen = sprintf(line + 0, "%08x", i);
		for (j = 0; j < row_count; j++) {
			if (!(j % 8)) splen += sprintf(line + splen, " ");

			if (mark[i+j]) {
				ret = dm9051_get_reg(db, i + j, &val);
				if (ret < 0)
					break;
				splen += sprintf(line + splen, " %02x", val);
			} else
				splen += sprintf(line + splen, " --");
		}
		netif_warn(db, pktdata, db->ndev, "%s\n", line); //netif_info
	}

	return ret;
}

int dm9051_single_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret, rxlen, padlen;
	unsigned int rxbyte;
	u8 buf[4 + 128];
	
	ret = dm9051_read_mem_rxb(db, DM_SPI_MRCMDX, &rxbyte, 2);
	if (ret)
		return ret;

	if (rx_break(db, rxbyte, ndev->features))
	{
		if (trap_rxb(db, &rxbyte)) {
			DMPLUG_LOG_RXPTR("rxb last", db);
			//dm9051_all_restart(db);
			printk("One rx error found!\n");
			return -EINVAL;
		}

		//printk("Once rx NO data~\n");
		return 0; //return -EINVAL;
	}
	trap_clr(db);

	ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
	if (ret)
		return ret;

	/* rx_head_takelen check */
	ret = rx_head_break(db);
	if (ret) {
		//dm9051_all_restart(db);
		printk("One rx_head error found!\n");
		return -EINVAL;
	}

	/* 7.1 ptpc */
	ret = DMPLUG_RX_TS_MEM(db);
	if (ret)
		return ret;

	rxlen = le16_to_cpu(db->rxhdr.rxlen);
	padlen = (plat_cnf->skb_wb_mode && (rxlen & 1)) ? rxlen + 1 : rxlen;
	
	db->bc.nRxcF++;
	printk("\n");
#ifdef DMPLUG_PTP
	if (db->ptp_enable && is_ptp_rxts_enable(db))
		printk("recv packet %d, rx tstamp %d bytes and %d bytes\n", db->bc.nRxcF, 8, rxlen);
	else
#endif
		printk("recv packet %d, rx %d bytes\n", db->bc.nRxcF, rxlen);

	ret = dm9051_read_mem_cache(db, DM_SPI_MRCMD, buf, padlen);
	if (ret)
	{
		return ret;
	}

	//SHOW_ptp_rx_packet_monitor(db, skb);
#ifdef DMPLUG_PTP
	if (db->ptp_enable) {
	if (is_ptp_rxts_enable(db)) {	// Inserted Timestamp
		sprintf(db->bc.head, "dump rx-tstamp %d", 8);
		dump_data_001(db, db->rxTSbyte, 8);
	}}
#endif
	
	sprintf(db->bc.head, "dump rx-len %d", rxlen);
	dump_data_001(db, buf, rxlen);
	return 0;
}

int test_loop_back(struct board_info *db)
{
	int ret;
	unsigned int val;
	u16 b;
	int i;
	unsigned int data_len[2] = {64, 128};
	u8 buf[256]; // = {0,0,0,0, 1,2,3,4,5,6,7,8,9,10, };

	printk("test_loop_back\n");
	msleep(1);
	SHOW_BMSR(db);
	SHOW_BMSR(db);

	printk("set_loop_back\n");

#if 1
	//ret = dm9051_set_reg(db, DM9051_NCR, 0x0A);
	ret = dm9051_set_reg(db, DM9051_NCR, 0x02);
	
	//ret = dm9051_set_reg(db, DM9051_NCR, 0x04);
	if (ret)
		return ret;
#endif
	dm9051_phyread_bmsr_loop(db, MII_BMSR, &val);
	netif_warn(db, link, db->ndev, "<dm9051_phyread_bmsr_loop rd.bmsr %04x [lpa] %04x> End...\n",
				db->bmsr, db->lpa);

	msleep(1);
	SHOW_BMSR(db);
	SHOW_BMSR(db);
	
	//[rx enable, etc.]
	//tail of _upstart
	ret = dm9051_all_start_intr(db);
	if (ret)
		return ret;

	db->rctl.rcr_all |= RCR_PRMSC; //RCR_DIS_LONG | RCR_RXEN; //particular, dm9051_set_rx_mode
	ret = dm9051_subconcl_and_rerxctrl(db);
	if (ret)
		return ret;
	
printk("Rdy\n");
	dump_regs(db);

	//SENDC
#if 0
//	printk("send_bytes 10\n"); //dm9.Monitor
//buf[0] = 0x01;
//buf[1] = 0x40;
//buf[2] = 0x0a;
//buf[3] = 0x00;
//	db->pad = 0;
//	db->data_len = 10;
//	ret = dm9051_single_tx(db, buf);
//	if (ret)
//		return ret;

//	db->tcr_wr = TCR_TXREQ; //pre-defined
//	ret = dm9051_req_tx(db);
//	if (ret)
//		return ret;

//	printk("poll nsr\n");
//	ret = dm9051_nsr_poll(db);
//	if (ret)
//		return ret;

//	dump_regs(db);
#endif
	
	//SENDC
#if 0
//for (b = 1; b <= 256; b++) {
//	buf[b-1] = (u8) b;
//}

//	db->pad = 0;
//	db->data_len = 128; //10; //64;
//	printk("Send_bytes %u\n", db->data_len);
//	dm9051_write_mem(db, DM_SPI_MWCMD, buf, db->data_len); //'!wb'
//	if (ret)
//		return ret;
//	ret = dm9051_set_regs(db, DM9051_TXPLL, &db->data_len, 2);
//	if (ret)
//		return ret;
////	ret = dm9051_single_tx(db, buf);
////	if (ret)
////		return ret;

//	ret = dm9051_set_reg(db, DM9051_TCR, TCR_TXREQ); //base with TCR_TXREQ
//	if (ret)
//		return ret;
////	db->tcr_wr = TCR_TXREQ; //pre-defined
////	ret = dm9051_req_tx(db);
////	if (ret)
////		return ret;

//	//printk("Poll nsr\n");
//	ret = dm9051_nsr_poll(db);
//	if (ret)
//		return ret;

//	dump_regs(db);
#endif
	
#if 1
for (b = 1; b <= 256; b++) {
	buf[b-1] = (u8) b;
}

for (i = 0; i < 2; i++) {
	db->pad = 0;
	db->data_len = data_len[i]; //66;
	db->tcr_wr = TCR_TXREQ; //pre-defined
	printk("Send_bytes %u\n", db->data_len);
	ret = dm9051_single_tx(db, buf);
	if (ret)
		return ret;

	ret = dm9051_req_tx(db);
	if (ret)
		return ret;

	ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	dump_regs(db);
}
#endif
	
	db->bc.nRxcF = 0;
	while(1) {
		int sRxcF = db->bc.nRxcF;
		ret = dm9051_single_rx(db);
		if (ret)
			break; //return ret;
		if (sRxcF != db->bc.nRxcF) {
			dump_regs(db);
		} else
			break;
	}
	printk("Total rx %d packets\n", db->bc.nRxcF);
	printk("\n");
	db->bc.nRxcF = 0;
	
//	//printk("_single_rx(db).1\n");
//	ret = _dm9051_single_rx(db);
//	if (ret)
//		return ret;
//	dump_regs(db);
//	printk("\n");

//	//printk("_single_rx(db).2\n");
//	ret = _dm9051_single_rx(db);
//	if (ret)
//		return ret;
//	dump_regs(db);
//	printk("\n");

	return ret;
}

/*
 * mac loopback test: 
 */
int test_loop_test(struct board_info *db)
{
	int ret = dm9051_all_start(db);
	if (ret)
		return ret;

	do {
		u8 save = db->rctl.rcr_all;
		ret = test_loop_back(db);
		db->rctl.rcr_all = save;
	} while(0);

	return ret;
}

MODULE_DESCRIPTION("Davicom DM9051 driver, mac loopback test!");
MODULE_LICENSE("GPL");
