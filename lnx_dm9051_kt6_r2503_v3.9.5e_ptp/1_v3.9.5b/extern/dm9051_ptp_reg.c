// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
//#include <stdint.h>
//#include <inttypes.h>
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

#include "../dm9051.h"
#include "dm9051_ptp1.h" /* 0.1 ptpc */

#if 0
// OF ptp_9051_gettime()
// v.s. dm9051_read_mem(db, DM_SPI_MRCMD, pbi->rxTSbyte, 8);
// THIS get ts, got wrong ts : to be verified!
// (TO MAKE IT RIGHT!)
int dm9051_get_clk_ts(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;
	unsigned int uIntTemp;
	u8 *temp = &pbi->clkTSbyte[0];
	int i;

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
		       DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);

	for (i = 0; i < 8; i++) {
		regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
		temp[i] = (u8)(uIntTemp & 0xFF); //this is ok
	}
	
	do {
		struct timespec64 t;
		t.tv_nsec = ((uint32_t)temp[3] << 24) | ((uint32_t)temp[2] << 16) |
		      ((uint32_t)temp[1] << 8) | (uint32_t)temp[0];
		      
		t.tv_sec = ((uint32_t)temp[7] << 24) | ((uint32_t)temp[6] << 16) |
		      ((uint32_t)temp[5] << 8) | (uint32_t)temp[4];

//.		printk("DM9051A ...ptp_9051_gettime / %p vs %p\n", temp, &pbi->clkTSbyte[0]);
//.		printk("DM9051A ...ptp_9051_gettime  %llu s, %lu ns\n", t.tv_sec, t.tv_nsec);
		//printk("clkTSbyte %l\u s\n", t.tv_sec);
		printk("clkTSbyte ...get_clk_ts %llu s\n", t.tv_sec);
	} while(0);
	return 0;
}
#endif

#if 0
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
#endif

#if 0
void on_core_init_ptp_rate(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	if (pbi->ptp_on) { /* all_start, all_upstart, all_restart */
		u32 rate_reg = dm9051_get_rate_reg(db); //15888, dm9051_get_rate_reg(db);
		netif_warn(db, hw, db->ndev, "dm9051.on.Pre-RateReg value = 0x%08X\n", rate_reg);
	}
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp reg"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
