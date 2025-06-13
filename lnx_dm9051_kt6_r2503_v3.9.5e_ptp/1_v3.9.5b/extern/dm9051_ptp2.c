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
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/ipv6.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>

//#include "dm9051_ptpd.h"
#include "../dm9051.h"
/*#include extern/extern.h */ //(extern/)
//#include "extern.h"
#include "dm9051_ptp1.h" /* 0.1 ptpc */

#ifdef DMPLUG_PTP_SW
void dm9051_ptp_tx_swtstamp(struct sk_buff *skb) //SKBTX_SW_TSTAMP
{
	if (skb_shinfo(skb)->tx_flags & SKBTX_SW_TSTAMP) {
		skb_tx_timestamp(skb); // Add SW_TSTAMP
	}
}
#endif

#ifdef DMPLUG_PTP
int ptp_9051_adjfine(struct ptp_clock_info *caps, long scaled_ppm)
{
//struct aq_ptp_s *aq_ptp = container_of(ptp, struct aq_ptp_s, ptp_info);
//    struct board_info *db = container_of(caps, struct board_info, ptp_caps);
//	struct board_info *db = container_of(caps, struct board_info, pbi.ptp_caps /*pbi*/ /*pbi.ptp_caps*/ /*ptp_caps*/);
//	ptp_board_info_t *pbi = &db->pbi;
	struct ptp_board_info *pbi = container_of(caps, struct ptp_board_info, ptp_caps);
	struct board_info *db = pbi->db;

	s64 ppm;
	s64 s64_adj;
	s64 subrate;
	u32 rate;
	// u16 hi, lo;
	u8 s_ppm[4];
	int i;
	int neg_adj = 0;
	static int adjfine5 = 5;

	/* 將scaled_ppm轉換為實際ppm值 */
#if 1
	if (scaled_ppm < 0) {
		ppm = ((s64)(-scaled_ppm) * 1000) / 65536;
		ppm = -ppm;
	} else {
		ppm = ((s64)scaled_ppm * 1000) / 65536;
	}
	/* 計算調整值 */
	s64_adj = (ppm * 171797) / 1000;  // base = 171.79
#else
	/* 計算調整值 */
#if 0
	ppm = (s64)((scaled_ppm >= 0) ? scaled_ppm : -scaled_ppm);
	ppm = div_s64(ppm * 1000, 65536);
	s64_adj = (ppm * 171797) / 1000;  // base = 171.79
	if (scaled_ppm < 0)
		s64_adj = -s64_adj;
#else
	ppm = (s64)((scaled_ppm >= 0) ? scaled_ppm : -scaled_ppm);
	ppm = div_s64(ppm * 1000, 65536);
	s64_adj = div_s64(ppm * 171797, 1000); // base = 171.79
	if (scaled_ppm < 0)
		s64_adj = -s64_adj;
#endif
#endif

	/* 保護寄存器訪問 */
	mutex_lock(&db->spi_lockm);

	/* 計算與上次調整的差值 */
	subrate = s64_adj - pbi->pre_rate;

	/* 處理正負值 */
	if (subrate < 0) {
		rate = (u32)(-subrate);
		neg_adj = 1;
	} else {
		rate = (u32)subrate;
		neg_adj = 0;
	}

	/* 溢出保護 */
	//if (rate > 0xffffffff)
	//    rate = 0xffffffff;
	/* 準備寄存器數據 */
	// hi = (rate >> 16);
	// lo = rate & 0xffff;
	// s_ppm[0] = lo & 0xff;
	// s_ppm[1] = (lo >> 8) & 0xff;
	// s_ppm[2] = hi & 0xFF;
	// s_ppm[3] = (hi >> 8) & 0xff;// 將32位rate值直接拆分為4個8位字節
	s_ppm[0] = rate & 0xff;           // 第1個字節（最低有效字節）
	s_ppm[1] = (rate >> 8) & 0xff;    // 第2個字節
	s_ppm[2] = (rate >> 16) & 0xff;   // 第3個字節
	s_ppm[3] = (rate >> 24) & 0xff;   // 第4個字節（最高有效字節）

	if (adjfine5) {
		printk("%d. Ent 0x%lX offset_pps %llX, pre_rat %llX, s64_delta_rat= 0x%llX, u32_rat= %X, sign= %d\n",
		       adjfine5--, scaled_ppm, s64_adj, pbi->pre_rate, subrate, rate, neg_adj);
	}

	/* 重置PTP時鐘控制寄存器 */
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);

	/* 寫入4字節調整數據 */
	for (i = 0; i < 4; i++) {
		dm9051_set_reg(db, DM9051_1588_TS, s_ppm[i]);
	}

	/* 根據正負值設置不同的控制位 */
	if (neg_adj == 1) {
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
			       DM9051_CCR_RATE_CTL | DM9051_CCR_PTP_RATE);
	} else {
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_RATE);
	}

	mutex_unlock(&db->spi_lockm);

	/* 存儲當前調整值供下次使用 */
	pbi->pre_rate = s64_adj;

	return 0;
}

/* phyter seems to miss the mark by 16 ns */
#define ADJTIME_FIX	16

int ptp_9051_adjtime(struct ptp_clock_info *caps, s64 delta)
{
	//remark1-slave
	//printk("...ptp_9051_adjtime\n");

//	struct board_info *db = container_of(caps, struct board_info,
//					     ptp_caps);
//	struct board_info *db = container_of(caps, struct board_info,
//						pbi.ptp_caps);
	struct ptp_board_info *pbi = container_of(caps, struct ptp_board_info, ptp_caps);
	struct board_info *db = pbi->db;

	//ptp_board_info_t *pbi = &db->pbi;

	struct timespec64 ts;
	int sign = 1;
	int i;
	//int err;
	u8 temp[8];

	//Spenser - Reset Rate register, write 0x60 bit0=1, then write bit0=0
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00); //Enable PTP function Register offset 0x60, value 0x00


	//remark1-slave
	//printk("+++00111+++++ [in %s] delta = %lld+++++++++\n", __FUNCTION__ ,delta);

	//printk("@@@1 ptp_dm8806_adjtime delta %llx \n", delta);

	delta += ADJTIME_FIX;
	//printk("@@@2 ptp_dm8806_adjtime delta %llx \n", delta);

	if (delta < 0) {
		sign = 0;
		delta = -delta;

		printk("delta less han zero.. \n");

	}

	//mutex_lock(&clock->extreg_lock); //ADDR_LOCK_HEAD_ESSENTIAL(db); //mutex_lock

	//printk("@@@2-1 ptp_dm8806_adjtime delta %llx \n", delta);
	ts.tv_sec = div_s64(delta, 0x3b9aca00);

	//printk("@@@2-2 ptp_dm8806_adjtime delta 0x%llx sec 0x%llx \n", delta, ts.tv_sec);

	ts.tv_nsec = (delta - (ts.tv_sec * 0x3b9aca00)) & 0xffffffff;


	//printk("@@@3 ptp_dm8806_adjtime delta %llx  nsec=%lx  \n", delta, ts.tv_nsec);

	if (sign == 0) {	// delta less han zero
		if (ts.tv_sec == 0) {
			printk("adjtime - delta.tv_sec = 0\n");
			ts.tv_sec++;
		}
		ts.tv_sec  = (0x100000000-ts.tv_sec);
		ts.tv_nsec = (1000000000 - ts.tv_nsec);
	}


	temp[0] = ts.tv_nsec & 0xff;
	temp[1] = (ts.tv_nsec & 0xffff) >> 8;
	temp[2] = (ts.tv_nsec >> 16) & 0xff;
	temp[3] = ts.tv_nsec >> 24;
	temp[4] = ts.tv_sec & 0xff;
	temp[5] = (ts.tv_sec & 0xffff) >> 8;
	temp[6] = (ts.tv_sec >> 16) & 0xff;
	temp[7] = ts.tv_sec >> 24;

	//dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x2);

//Spenser - Add to PTP Clock
	mutex_lock(&db->spi_lockm);

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	for (i = 0; i < 8; i++) {
		dm9051_set_reg(db, DM9051_1588_TS, temp[i] & 0xff);
	}
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_ADD);

	mutex_unlock(&db->spi_lockm);

	//remark1-slave
	//printk(" ptp_9051_adjtime hwtstamp = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
	//       temp[0], temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7]);


	//remark1-slave
	//printk("### sign %d delta %llx ts.tv_sec =  %lld, ts.tv_nsec = %ld  ###\n", sign, delta, ts.tv_sec, ts.tv_nsec);

	//mutex_unlock(&clock->extreg_lock);
	//ADDR_LOCK_TAIL_ESSENTIAL(db); //mutex_unlock

	printk("ptp_9051_adjtime...\n");

	return 0;

}

int ptp_9051_gettime(struct ptp_clock_info *caps,
		     struct timespec64 *ts)
{
//struct board_info *db = container_of(caps, struct board_info,
//			 ptp_caps);
//struct board_info *db = container_of(caps, struct board_info,
//					pbi.ptp_caps);
	struct ptp_board_info *pbi = container_of(caps, struct ptp_board_info, ptp_caps);
	struct board_info *db = pbi->db;

//ptp_board_info_t *pbi = &db->pbi;
	unsigned int temp[8];
	int i;
	unsigned int uIntTemp;

	printk("DM9051A ...ptp_9051_gettime\n");

// tom: from stone's doc. write 0x84 to reg 0x61 is enough,
// bit 0 PTP_EN has been set in ptp_init
	mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
		       DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);

	for (i = 0; i < 8; i++) {
		regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
		temp[i] = (u8)(uIntTemp & 0xFF);
	}
	mutex_unlock(&db->spi_lockm);
	/*
	dm9051_read_mem(db, DM9051_1588_TS, temp, DM9051_1588_TS_BULK_SIZE);

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x80);	// Reset Register 68H Index
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01
	*/
//regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, &temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H

// tom: re-write the upper statements
	ts->tv_nsec = ((uint32_t)temp[3] << 24) | ((uint32_t)temp[2] << 16) |
		      ((uint32_t)temp[1] << 8) | (uint32_t)temp[0];
	ts->tv_sec  = ((uint32_t)temp[7] << 24) | ((uint32_t)temp[6] << 16) |
		      ((uint32_t)temp[5] << 8) | (uint32_t)temp[4];


//printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ptp_dm9051_gettime sec=%llx nsec=%lx \n", ts->tv_sec, ts->tv_nsec);

	return 0;
}

int ptp_9051_settime(struct ptp_clock_info *caps,
		     const struct timespec64 *ts)
{

//struct board_info *db = container_of(caps, struct board_info,
//			 ptp_caps);
//struct board_info *db = container_of(caps, struct board_info,
//			pbi.ptp_caps);
	struct ptp_board_info *pbi = container_of(caps, struct ptp_board_info, ptp_caps);
	struct board_info *db = pbi->db;

//ptp_board_info_t *pbi = &db->pbi;
	mutex_lock(&db->spi_lockm);
	printk("...ptp_9051_settime\n");

	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_nsec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 24) & 0xff));     // Write register 0x68

	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_sec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 24) & 0xff));     // Write register 0x68

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_WRITE);
	mutex_unlock(&db->spi_lockm);


	return 0;
}

int ptp_9051_feature_enable(struct ptp_clock_info *caps,
			    struct ptp_clock_request *rq, int on)
{
	printk("...ptp_9051_feature_enable\n");
	return 0;
}
int ptp_9051_verify_pin(struct ptp_clock_info *caps, unsigned int pin,
			enum ptp_pin_function func, unsigned int chan)
{
	printk("!!! 1. ptp_9051_verify_pin in\n");
	return 0;
}

static struct ptp_clock_info dm9051a_ptp_info = {
	.owner = THIS_MODULE,
	//.name = "DM9051A PTP",
	.max_adj = 50000000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0, //n_periodic_outputs
	//.n_pins = 0, //1; //n_programable_pins
	.pps = 0, //1, //0,
	.adjfine = ptp_9051_adjfine,
	.adjtime = ptp_9051_adjtime,
	.gettime64 = ptp_9051_gettime,
	.settime64 = ptp_9051_settime,
	.enable = ptp_9051_feature_enable,
	.verify = ptp_9051_verify_pin,

};

static void dm9051_ptp_core_init(struct board_info *db)
{
//Spenser
	/*
		dm9051_set_reg(db, DM9051_1588_RX_CONF1,
			       DM9051A_RC_SLAVE | DM9051A_RC_RX_EN | DM9051A_RC_RX2_EN);
	*/
	dm9051_set_reg(db, DM9051_1588_RX_CONF1, 0x12);		//enable 8 bytes timestamp & multicast filter

	//Spenser - Reset Rate register, write 0x60 bit0=1, then write bit0=0
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00); //Enable PTP function Register offset 0x60, value 0x00

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x01); //Enable PTP clock function Register offset 0x61, value 0x01

	//Setup GP1 to edge trigger output!
	//Register 0x60 to 0x0 (GP page (bit 1), PTP Function(bit 0))
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00);

	//Register 0x6A to 0x06 (interrupt disable(bit 2), trigger or event enable(bit 1), trigger output(bit 0))
	dm9051_set_reg(db, DM9051_1588_GPIO_CONF, 0x06);

	//Register 0x6B to 0x02(trigger out type: edge output(bit 3:2),  triger output active high(bit 1))
	dm9051_set_reg(db, DM9051_1588_GPIO_TE_CONF, 0x02);

	//Stone add for 1588 Read 0x68 in one SPI cycle enable (register 0x63 bit 6 0:enable, 1:disable => 0x40)
	//Stone add for 1588 TX 1-Step checksum enable (register 0x63 bit 7 0:enable, 1:disable => 0x80)
	dm9051_set_reg(db, DM9051_1588_1_STEP_CHK, 0x00);

#ifdef DMPLUG_PPS_CLKOUT
	dm9051_set_reg(db, 0x3C, 0xB0);
#endif
}

//static u32 dm9051_get_rate_reg(struct board_info *db);
//void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);

/* ptpc - support functions-2 */
static u32 dm9051_get_rate_reg(struct board_info *db)   /*s64*/
{
	u8 mRate[4];
	u32 pre_rate;
	//_mutex_lock(&db->_spi_lockm);
	dm9051_set_reg(db, 0x69, 0x01);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	regmap_noinc_read(db->regmap_dm, 0x68, mRate, 4);
	pre_rate = ((uint32_t)mRate[3] << 24) | ((uint32_t)mRate[2] << 16) |
		   ((uint32_t)mRate[1] << 8) | (uint32_t)mRate[0];
	//_mutex_unlock(&db->_spi_lockm);
	//printk("Pre-RateReg value = 0x%08X\n", pre_rate);

	return pre_rate;
}

static void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamps;
	//u64 regval;
	u8 temp[9];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	u32 sec;
	u64 ns;
	//int i;
	//unsigned int uIntTemp = 0;


	memset(&temp, 0, sizeof(temp));

	//printk("==================================> TX_hwtstamp FROM in\r\n");

	//Spenser - Read TX timestamp from REG_68H
	//remark6-slave

//.	_mutex_lock(&db->_spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// Reset Register 68H Index
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// dummy reset to workaround unsync
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01

	regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// for (i=0; i< 8; i++) {
	// 	regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
	// 	temp[i] = (u8)(uIntTemp & 0xFF);
	// }
//.	_mutex_unlock(&db->_spi_lockm);

#if 0
	if (!uIntTemp) {
		printk("HW Timestamp read fail\n");
	} else {
		printk("HW Timestamp read OK\n");
	}
#endif
#ifdef _DE_TIMESTAMP
	printk(" TXTXTXTXTX hwtstamp 0x68 = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \r\n",
	       temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
#endif
	//printk("==================================> TX_hwtstamp FROM OUT\r\n");
	ns_lo = temp[0] | (temp[1] << 8);
	ns_hi = temp[2] | (temp[3] << 8);

	s_lo = temp[4] | (temp[5] << 8);
	s_hi = temp[6] | (temp[7] << 8);

	sec = s_lo;
	sec |= s_hi << 16;

	ns = ns_lo;
	ns |= ns_hi  << 16;

#ifdef DE_TIMESTAMP
	//remark4-slave
	//printk(" TXTXTXTXTX hwtstamp sec = %x, ns = %x \r\n", sec, (u32)ns);

#endif


	ns += ((u64)sec) * 1000000000ULL;


	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ns_to_ktime(ns);
	//skb_complete_tx_timestamp(skb, &shhwtstamps);
	//remark5-slave
	//printk("---Report TX HW Timestamp\n");
	skb_tstamp_tx(skb, &shhwtstamps); //For report T3 HW tx tstamp
	//remark5-slave
	//printk("Report TX HW Timestamp---\n");

//Spenser - doesn't trigger GP1 this time.
#if 0
	if (skb->len == 86) {
		if (skb->data[42] == 0x00) {
			printk("++++++ master => Sync or Follow_up packet  (slave => Delay Reqest packet sequestid = %x %x )!!!!! +++++ \r\n", skb->data[73], skb->data[74]);
			printk(" TXTXTXTXTX hwtstamp 0x68 = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \r\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
			//dm9051_GP1_setup(db, temp);
			schedule_work(&db->ptp_extts0_work);
		}

	}
#endif
}

void on_core_init_ptp_rate(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	if (pbi->ptp_on) { /* all_start, all_upstart, all_restart */
		u32 rate_reg = dm9051_get_rate_reg(db); //15888, dm9051_get_rate_reg(db);
		netif_warn(db, hw, db->ndev, "dm9051.on.Pre-RateReg value = 0x%08X\n", rate_reg);
	}
}

// SKBTX_HW_TSTAMP = 1 << 0,
// SKBTX_SW_TSTAMP = 1 << 1,
// SKBTX_IN_PROGRESS = 1 << 2,
// ...
// SKBTX_SCHED_TSTAMP = 1 << 6,
static void dm9051_ptp_tx_in_progress(struct board_info *db, struct sk_buff *skb)
{
	db->pbi.ptp_skp_hw_tstamp = 0;
	
#if 0
	/* Use: enum hwtstamp_tx_types
	 */

	/* When S/W TSTAMP, from ever (H/W TSTAMP) and (S/W TSTAMP) not clear this 'tx_type', then do , will hurt !!!
	 */
	if (db->pbi.tstamp_config.tx_type &
	    (HWTSTAMP_TX_ON | HWTSTAMP_TX_ONESTEP_SYNC)) {
		.................... //can do
	}

	//	if (!db->pbi.tstamp_config.tx_type)
	//		return;
#endif
	
	/* Use: skb->tx_flags, is better~
	 */
	if (!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		return;

	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		db->pbi.ptp_skp_hw_tstamp = 1;
		//return 1;
	}
	//return 0;
}

//SKBTX_HW_TSTAMP
static void dm9051_ptp_tcr_2wr(struct board_info *db, struct sk_buff *skb)
{
	db->tcr_wr = TCR_TXREQ; // TCR register value
	db->pbi.ptp_chip_push_tstamp = 0;

	do {
		ptp_board_info_t *pbi = &db->pbi;
		struct ptp_header *ptp_hdr;

		//	if (!pbi->tstamp_config.tx_type)
		//		return;

		/* Use: skb->tx_flags, is better~
		 */
		if (!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
			return;

		ptp_hdr = get_ptp_header(skb);
		if (ptp_hdr) {
			//db->ptp_step = dm9051_ptp_step(db, skb);
			//if (db->ptp_step) {
			u8 message_type = get_ptp_message_type005(ptp_hdr); //for tx send
			//if (dm9051_ptp_frame(db, skb)) {
			//if (likely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			if (is_ptp_sync_packet(message_type)) {
				pbi->ptp_step = (u8)(ptp_hdr->flag_field[0] & PTP_FLAG_TWOSTEP) ? PTP_TWO_STEP : PTP_ONE_STEP;
				if (pbi->ptp_step == 2) {
	#if defined(DMPLUG_PTP_TWO_STEP)
					db->tcr_wr = TCR_TSEN_CAP | TCR_TXREQ;
					db->pbi.ptp_chip_push_tstamp = 1;
	#else
	#warning "dm9051 NOT Add H/W PTP TWO STEP.."
	#endif
				} else {
					db->tcr_wr = TCR_TS1STEP_EMIT | TCR_TXREQ;
				}
			}
			else if (is_ptp_delayreq_packet(message_type))
				db->tcr_wr = TCR_TSEN_CAP | TCR_TS1STEP_EMIT | TCR_TXREQ;
			else if (is_peer_delayreq_packet(message_type))
				db->tcr_wr = TCR_TSEN_CAP | TCR_TS1STEP_EMIT | TCR_TXREQ;
			//}
			//}
			//return message_type;
			//}
		}
	} while(0);
}

//SKBTX_HW_TSTAMP
static void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	//ptp_board_info_t *pbi = &db->pbi;
	//	if (!pbi->tstamp_config.tx_type)
	//		return;

	/* Use: skb->tx_flags, is better~
	 */
	if (!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		return;

//	if ((is_ptp_sync_packet(message_type) &&
//		db->ptp_step == 2) ||
//		is_ptp_delayreq_packet(message_type) ||
//		is_peer_delayreq_packet(message_type)) { //_15888_,

//.	if (db->pbi.ptp_chip_push_tstamp) //tobe
	if (db->tcr_wr & TCR_TSEN_CAP) {
		int ret;

		/* Poll for TX completion */
		ret = dm9051_nsr_poll(db);
		if (ret) {
			netdev_err(db->ndev, "ptp TX hwtstamp completion polling timeout\n");
			//.return ret; //.only can be less hurt
		}

		dm9051_ptp_tx_hwtstamp(db, skb); //dm9051_hwtstamp_to_skb(skb, db); //_15888_,
	}
	if (db->pbi.ptp_skp_hw_tstamp) { //.(flags & SKBTX_IN_PROGRESS)
		static int flags_count = 0; //to debug show
		flags_count++;
		netif_crit(db, hw, db->ndev, "Yes, %05d dm9051_nsr_poll\n", flags_count);
		netif_info(db, hw, db->ndev, "Yes, %05d skb_tstamp_tx\n", flags_count);
	}

//	}
}

int dm9051_ptp_single_tx(struct board_info *db, struct sk_buff *skb)
{
	int ret;
	/* 6 tx ptpc */
	dm9051_ptp_tx_in_progress(db, skb); //DMPLUG_PTP_TX_IN_PROGRESS(db, skb); //tom tell, 20250522 //Or using for two step ?
	dm9051_ptp_tcr_2wr(db, skb); //DMPLUG_PTP_TX_PRE(db, skb);
	LEN_TX(db, skb);
	PAD_TX(db, skb);
	ret = MODE_TX(db, skb);
	if (ret == 0) {
		dm9051_ptp_txreq_hwtstamp(db, skb); //DMPLUG_TX_EMIT_TS(db, skb); /* 6.1 tx ptpc */
		SHOW_DEVLOG_TCR_WR(db);
	}
	dev_kfree_skb(skb);
	return ret;
}

static u64 rx_extract_ts(u8 *rxTSbyte)
{
	//u8 temp[12];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	//u32 prttsyn_stat, hi, lo,
	u32 sec;
	u64 ns;

#if 0
	printk(" REAL RX TSTAMP hwtstamp= %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
	       rxTSbyte[0], rxTSbyte[1], rxTSbyte[2], rxTSbyte[3], rxTSbyte[4], rxTSbyte[5], rxTSbyte[6], rxTSbyte[7]);
#endif

	//dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x02); //Read RX Time Stamp Clock Register offset 0x62, value 0x02

	ns_lo = rxTSbyte[7] | (rxTSbyte[6] << 8);
	ns_hi = rxTSbyte[5] | (rxTSbyte[4] << 8);

	s_lo = rxTSbyte[3] | (rxTSbyte[2] << 8);
	s_hi = rxTSbyte[1] | (rxTSbyte[0] << 8);

	sec = s_lo;
	sec |= s_hi << 16;

	ns = ns_lo;
	ns |= ns_hi  << 16;

	ns += ((u64)sec) * 1000000000ULL;
	//printk("_dm9051_ptp_rx_hwtstamp ns_lo=%x, ns_hi=%x s_lo=%x s_hi=%x \r\n", ns_lo, ns_hi, s_lo, s_hi);
	return ns;
}

void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	ptp_board_info_t *pbi = &db->pbi;

#if 0
	/* Use: enum hwtstamp_rx_filters
	 */
	/* Even S/W TSTAMP, on (H/W TSTAMP) do , will be not hurt !!
	 */
	if (pbi->tstamp_config.rx_filter &
	    (HWTSTAMP_FILTER_PTP_V2_EVENT | HWTSTAMP_FILTER_ALL)) {
		.................... //can do
	}
#endif
#if 0
	/* Even S/W TSTAMP, do shhwtstamps->hwtstamp = ns_to_ktime(ns); (H/W TSTAMP) will be not hurt !!
	 */
	if (!pbi->tstamp_config.rx_filter) //[wait further test..]
		return;
#endif

#if 0 //[wait further test..]
	if (is_ptp_rxts_en(db)) //if T1/T4, // Is it inserted Timestamp? //[wait further test..]
#endif
	{
		//So when NOT T1/T4, we can skip tell tstamp (just an empty (virtual) one)

#if 0
			= original.dm9051_ptp_rx_hwtstamp(db, skb /*, db->rxTSbyte*/); //_15888_,
#endif
		/* following, with netif_rx(skb),
		 * slave4l can parse the T1 and/or T4 rx tstamp from master
		 */
		if (pbi->ptp_on) { //NOT by db->ptp-enable
			//printk("==> dm9051_ptp_rx_hwtstamp in\r\n");
			/* Since we cannot turn off the Rx timestamp logic if the device is
			 * doing Tx timestamping, check if Rx timestamping is configured.
			 */
			u64 ns = rx_extract_ts(pbi->rxTSbyte);
			/* Use skb_hwtstamps(skb) get 'skb_shared_hwtstamps' and then copy to ->hwtstamp
			 * We can also use skb_complete_rx_timestamp() to make the same result.
			 */
			do {
				struct skb_shared_hwtstamps *shhwtstamps =
					skb_hwtstamps(skb); //for pass T2 the HW rx tstamp
				memset(shhwtstamps, 0, sizeof(*shhwtstamps));
				shhwtstamps->hwtstamp = ns_to_ktime(ns);
			} while (0);

			//printk("Report RX Timestamp to skb = %lld\n", shhwtstamps->hwtstamp);
			//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x08); //Clear RX Time Stamp Clock Register offset 0x60, value 0x08
			//printk("<== dm9051_ptp_rx_hwtstamp out\r\n");
		}
	}
}

/* receive rx_tstamp */
int dm9051_read_ptp_tstamp_mem(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	//_15888_
	//if (db->ptp_on) { //Even NOT ptp_on, need do.
	if (pbi->ptp_enable) {
		if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
			int ret;
			//printk("Had RX Timestamp... rxstatus = 0x%x\n", db->rxhdr.status);
			if (db->rxhdr.status & RSR_RXTS_LEN) {	// 8 bytes Timestamp
				ret = dm9051_read_mem(db, DM_SPI_MRCMD, pbi->rxTSbyte, 8);
				if (ret) {
					netif_err(db, hw, db->ndev, "Read TimeStamp8 error: %02x\n", ret);
					return ret;
				}
			} else {	// 4 bytes Timestamp
				/* 4bytes, dm9051a NOT supported, Will only support for OASPI function chip.
				 */
				ret = dm9051_read_mem(db, DM_SPI_MRCMD, pbi->rxTSbyte, 4);
				if (ret) {
					netif_err(db, hw, db->ndev, "Read TimeStamp4 error: %02x\n", ret);
					return ret;
				}
			}
		}
	}
	//}
	return 0;
}

static void dm9051_ptp_register(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	printk("\n");
	netif_info(db, hw, db->ndev, "DM9051A Driver PTP Init\n");

	pbi->db = db;
	pbi->ptp_caps = dm9051a_ptp_info; //.name = "DM9051A PTP",
	strncpy(pbi->ptp_caps.name, "DM9051A PTP", sizeof(pbi->ptp_caps.name));

	pbi->ptp_clock = ptp_clock_register(&pbi->ptp_caps /*&db->pbi.ptp_caps*/ /*&db->ptp_caps*/,
					    &db->ndev->dev);
	if (IS_ERR(pbi->ptp_clock)) {
		pbi->ptp_clock = NULL;
		dev_err(&db->spidev->dev, "ptp_clock_register failed\n");
	}  else if (pbi->ptp_clock) {
		netif_warn(db, hw, db->ndev, "ptp_clock_register added PHC, index %d on %s\n",
			   ptp_clock_index(pbi->ptp_clock), db->ndev->name);

	}
	//db->ptp_flags |= IGB_PTP_ENABLED;	// Spenser - no used
}

static void dm9051_ptp_unregister(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	/* Disable PTP for if switch to standard version from PLUG_PTP version*/
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x02); //Disable PTP clock function Register offset 0x61, value 0x02

	if (pbi->ptp_clock) {
		ptp_clock_unregister(pbi->ptp_clock);
		pbi->ptp_clock = NULL;
		//printk("_[ptp] remove: PTP clock!!!\r\n");
		netif_err(db, hw, db->ndev, "_[ptp] remove: PTP clock!!!\r\n");
	}
}

void ptp_init(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	/* Turn on by ptp4l run command
	 * db->ptp_on = 1; */
	pbi->ptp_on = 0;
	dm9051_ptp_register(db); //_15888_
	dm9051_ptp_core_init(db); //only by _probe [for further functionality test, do eliminate here, put to _open, and further _core_init]
}
void ptp_end(struct board_info *db)
{
	dm9051_ptp_unregister(db); //_15888_ todo
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp2"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
