/*
 * drivers/net/ethernet/davicom/dm9051_ptp.c
 * Copyright 2021 Davicom Semiconductor,Inc.
 *
 * 	This program is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU General Public License
 * 	as published by the Free Software Foundation; either version 2
 * 	of the License, or (at your option) any later version.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *	http://www.davicom.com.tw/
 *	Stone SHYR <stone_shyr@davicom.com.tw>
 */

#include <linux/crc32.h>
//#include <linux/kernel.h>
#include <linux/if_vlan.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_classify.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/math64.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/math64.h>
#include <linux/time64.h>
#include <linux/net_tstamp.h>  // for HWTSTAMP_TX_*
#include "asm-generic/errno-base.h"
#include "dm9051.h"
#include "dm9051_ptp.h"
#include "linux/ethtool.h"
#include "linux/timekeeping.h"
#include "linux/workqueue.h"
#include <uapi/linux/ptp_clock.h>

static int ptp_9051_gettime(struct ptp_clock_info *ptp,
			    struct timespec64 *ts);

static int ptp_9051_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts);
// read 9051 clock
static int ptp_9051_read_clock(struct board_info *db, u8 *timestamp);
// read 9051 time stamp from GP1, GP2 or TXTSTAMP
static int ptp_9051_read_ts(struct board_info* db, u8* timestamp, TS_SOURCE ts_src);

// debug print time stamp messages

#define MAX_RXTS	64
/* phyter seems to miss the mark by 16 ns */
#define ADJTIME_FIX	16
//#define SKB_TIMESTAMP_TIMEOUT	2 /* jiffies */
//#define SKB_TIMESTAMP_TIMEOUT	1 /* jiffies */
#define SKB_TIMESTAMP_TIMEOUT	0 /* jiffies */

#define PSF_PTPVER	2
#define PSF_EVNT	0x4000
#define PSF_RX		0x2000
#define PSF_TX		0x1000
#define EXT_EVENT	1
#define CAL_EVENT	7
#define CAL_TRIGGER	1

/* a list of clocks and a mutex to protect it */
static LIST_HEAD(phyter_clocks);
static DEFINE_MUTEX(phyter_clocks_lock);

/* flags controlling PTP/1588 function */

#define IGB_PTP_OVERFLOW_CHECK	BIT(1)

//Stone add for GP1 trigger setup !
int trigger_ff = 0x01;
int trigger_70 = 0x01;
int tx_1588 = 0x00;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)

int dm9051_get_ts_info(struct net_device *net_dev,
                       struct kernel_ethtool_ts_info *info)
#else

	int dm9051_get_ts_info(struct net_device *net_dev,
			       struct ethtool_ts_info *info)
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)

{
	struct board_info *db = netdev_priv(net_dev);
	dm_printk("[in %s] *dm9051_ts_info*\n", __FUNCTION__);

	info->phc_index = db->ptp_clock ? ptp_clock_index(db->ptp_clock) : -1;
	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types =
		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
		BIT(HWTSTAMP_TX_OFF) |
		BIT(HWTSTAMP_TX_ON);

	info->rx_filters =
		BIT(HWTSTAMP_FILTER_NONE) |
		BIT(HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
		BIT(HWTSTAMP_FILTER_PTP_V2_EVENT) |
		BIT(HWTSTAMP_FILTER_ALL);

	dm_printk("[%s] tx_types %x, rx_filters %x", __func__, info->tx_types, info->rx_filters);

	return 0;
}

static int ptp_9051_getcrosststamp(struct ptp_clock_info *ptp,
                                  struct system_device_crosststamp *cts)
{
	struct board_info *db = container_of(ptp, struct board_info, ptp_caps);
	u8 cur_ts[8];
	struct system_time_snapshot tmp_snapshot1;
	struct system_time_snapshot tmp_snapshot2;

	ktime_get_snapshot(&tmp_snapshot1);
	ptp_9051_read_clock(db, cur_ts);
	ktime_get_snapshot(&tmp_snapshot2);

	// 取得 snapshot1 與 snapshot2 的中間值
	cts->sys_realtime = ktime_add_ns(tmp_snapshot1.real,
					 ktime_to_ns(ktime_sub(tmp_snapshot2.real, tmp_snapshot1.real)) / 2);
	cts->sys_monoraw = ktime_add_ns(tmp_snapshot1.raw,
					 ktime_to_ns(ktime_sub(tmp_snapshot2.raw, tmp_snapshot1.raw)) / 2);

	// convert cur_ts[8] to ktime_t
	struct timespec64 ts;
	ts.tv_nsec = ((uint32_t)cur_ts[3] << 24) | ((uint32_t)cur_ts[2] << 16) |
		((uint32_t)cur_ts[1] << 8) | (uint32_t)cur_ts[0];
	ts.tv_sec = ((uint32_t)cur_ts[7] << 24) | ((uint32_t)cur_ts[6] << 16) |
		((uint32_t)cur_ts[5] << 8) | (uint32_t)cur_ts[4];
	cts->device = ktime_set(ts.tv_sec, ts.tv_nsec);

	return 0;
}


static int write_rate_reg(struct board_info *db, u64 rate, int neg_adj)
{

	u8 s_ppm[4];
	u16 hi, lo;

	// clamp rate
	if (rate > 0xFFFFFFFF) {
		rate = 0xFFFFFFFF;
	}

	hi = (rate >> 16);
	lo = rate & 0xffff;

        s_ppm[0] = lo & 0xff;
	s_ppm[1] = (lo >> 8) & 0xff;
	s_ppm[2] = hi & 0xFF;
	s_ppm[3] = (hi >> 8) & 0xff;

	mutex_lock(&db->tsreg_lock);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST); //R61 W80
	dm9051_set_reg(db, DM9051_1588_TS, s_ppm[0]);  //Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, s_ppm[1]);    //Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, s_ppm[2]);  //Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, s_ppm[3]);    //Write register 0x68

        if (neg_adj) {
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_RATE_CTL | DM9051_CCR_PTP_RATE);
	} else {
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_RATE);
	}
	mutex_unlock(&db->tsreg_lock);
        return 0;
}


static int ptp_9051_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	const s32 RATE_BASE = 172;
 	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
#ifdef DE_TIMESTAMP
	printk("+++00112+++++ [in %s] scaled_ppm = %ld, db.last_rate = %lld+++++++++\n",
	       __FUNCTION__ ,scaled_ppm, db->last_rate);
#endif

	s64 signed_rate = scaled_ppm_to_ppb(scaled_ppm) * RATE_BASE;
	s64 diff_rate = signed_rate - db->last_rate;
	int diff_rate_sign = 0;
	db->last_rate = signed_rate;
	if (diff_rate < 0) {
		diff_rate = -diff_rate;
		diff_rate_sign = 1;
	}

	write_rate_reg(db, diff_rate, diff_rate_sign);
	return 0;
}



static int ptp_9051_adjtime(struct ptp_clock_info *ptp, s64 delta)
{

	struct timespec64 diff = ns_to_timespec64(delta);
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	mutex_lock(&db->tsreg_lock);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(diff.tv_nsec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_nsec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_nsec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_nsec >> 24) & 0xff));     // Write register 0x68

        dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(diff.tv_sec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_sec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_sec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((diff.tv_sec >> 24) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_ADD);
	mutex_unlock(&db->tsreg_lock);
	dm_printk("[%s()], sec %lld, ns %ld", __func__, diff.tv_sec, diff.tv_nsec);
	return 0;
}

static int ptp_9051_read_clock(struct board_info* db, u8* timestamp)
{
	mutex_lock(&db->tsreg_lock);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
		       DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);

        // bug fix: dm9051a 無法接受連續讀取
	// ret = dm9051_read_mem(db, DM9051_1588_TS, cur_ts, DM9051_1588_TS_BULK_SIZE);

	for (int i=0; i<8; i++) {
		dm9051_get_reg8(db, DM9051_1588_TS, timestamp + i);
	}

        mutex_unlock(&db->tsreg_lock);
	return 0;
}


static int ptp_9051_read_ts(struct board_info* db, u8* ts, TS_SOURCE ts_src)
{
	memset(ts, 0, DM9051_1588_TS_BULK_SIZE);
	// tsreg_lock protect
	mutex_lock(&db->tsreg_lock);
	// reset ptp clock index
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	// read TX Time Stamp Clock Register 0x62
	// 這個 register 是 read/write clear, 先讀再寫會不小心 clear 其他 bits
	// 直接 programming 才對
	if (ts_src == TS_SOURCE_GP2) {
		dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, DM9051_GPTXRX_GP2_TE);
	} else if (ts_src == TS_SOURCE_GP1) {
		dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, DM9051_GPTXRX_GP1_TE);
	} else {
		dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, DM9051_GPTXRX_RD_TS);
	}
	// read 8 bytes timestamp
	for (int i = 0; i < DM9051_1588_TS_BULK_SIZE; i++)
		dm9051_get_reg8(db, DM9051_1588_TS, ts + i);

	mutex_unlock(&db->tsreg_lock);
#ifdef DE_TIMESTAMP
	dm_printk(" TXTXTXTXTX hwtstamp 0x68 = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \r\n",
		  ts[0], ts[1],ts[2],ts[3],ts[4],ts[5],ts[6],ts[7]);
#endif
	return 0;
}


static int ptp_9051_gettime(struct ptp_clock_info *ptp,
			    struct timespec64 *ts)
{
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	u8 cur_ts[8];

        ptp_9051_read_clock(db, cur_ts);

	ts->tv_nsec = ((uint32_t)cur_ts[3] << 24) | ((uint32_t)cur_ts[2] << 16) |
		((uint32_t)cur_ts[1] << 8) | (uint32_t)cur_ts[0];
	ts->tv_sec = ((uint32_t)cur_ts[7] << 24) | ((uint32_t)cur_ts[6] << 16) |
		((uint32_t)cur_ts[5] << 8) | (uint32_t)cur_ts[4];

#ifdef DE_TIMESTAMP
	dm_printk("[%s] sec=%llx nsec=%lx", __func__, ts->tv_sec, ts->tv_nsec);
#endif
	return 0;
}


static int ptp_9051_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	mutex_lock(&db->tsreg_lock);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_nsec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 24) & 0xff));     // Write register 0x68

        dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_sec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 24) & 0xff));     // Write register 0x68

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_WRITE);
	mutex_unlock(&db->tsreg_lock);

	dm_printk("[%s] ts->tv_sec =  %llx, ts->tv_nsec = %lx", __func__, ts->tv_sec, ts->tv_nsec);
	return 0;
}

#if 0
//Stone add for 1588 GP1 trigger edge output enale !
#define DM9051_PTP_SEC_DELAY 60
#define DEBUG_PPS            0
static int dm9051_pps_configure(struct ptp_clock_info *ptp, struct ptp_clock_request *rq)
{
	struct timespec64 now;
	//u64 ns;
	unsigned int temp[9];
	int ret = 0;
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	//Read dm9051 ptp clock now!
	ptp_9051_gettime(ptp, &now);
#if DEBUG_PPS
        printk("[%s] now.tv_sec = 0x%llx, now.tv_nsec = 0x%lx 111", __func__,
               now.tv_sec, now.tv_nsec);
#endif
        // calculate for next sec
	timespec64_add_ns(&now, NSEC_PER_SEC);
#if DEBUG_PPS
        printk("[%s] trigger target sec = 0x%llx", __func__,
               now.tv_sec);
#endif
	temp[0]= 0x00;
	temp[1]= 0x00;
	temp[2]= 0x00;
	temp[3]= 0x00;
	temp[4]= now.tv_nsec & 0xff;
	temp[5]= (now.tv_sec>>8) & 0xff;
	temp[6]= (now.tv_sec>>16) & 0xff;
	temp[7]= (now.tv_sec>>24) & 0xff;

#undef GP1_TEST
#ifdef GP1_TEST //===================================
	//Setup GP1 to edge trigger output!
	//Register 0x60 to 0x0 (GP page (bit 1), PTP Function(bit 0))
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00);

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_ST_GPIO, &temp[8]);
	printk("Register 0x60 (0x00) = 0x%x \r\n", temp[8]);
#endif

	//Register 0x6A to 0x06 (interrupt enable(bit 2), trigger or event enable(bit 1), trigger output(bit 0))
	dm9051_set_reg(db, DM9051_1588_GPIO_CR, 0x06);

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_GPIO_CR, &temp[8]);
	printk("Register 0x6A (0x06) = 0x%x \r\n", temp[8]);
#endif

	//Register 0x6B to 0x02(trigger out type: edge output(bit 3:2),  triger output active high(bit 1))
	dm9051_set_reg(db, DM9051_1588_GPIO_TECR, 0x02);

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_GPIO_TECR, &temp[8]);
	printk("Register 0x6B (0x02) = 0x%x \r\n", temp[8]);
#endif
#endif // DEBUT_PPS
	mutex_lock(&db->tsreg_lock);
	//Register 0x61 to 0x80, clear Reg 0x68
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x80);

#if DEBUG_PPS
	ret = dm9051_get_reg(db, DM9051_1588_CLK_CTRL, &temp[8]);
	printk("Register 0x61 (0x80) = 0x%x \r\n", temp[8]);
#endif
	//Register 0x68 to now.tv_nsec & now.tv_sec !
	for (int i=0; i<8 ;i++)
		dm9051_set_reg(db, DM9051_1588_TS, temp[i]);
	mutex_unlock(&db->tsreg_lock);

	//Register 0x62 to 0x10 (GP 1 trigger load or event Read)
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x10);

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_GP_TXRX_CTRL, &temp[8]);
	printk("Register 0x62 (0x10) = 0x%x \r\n", temp[8]);
#endif

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_ST_GPIO, &temp[8]);
	printk("Register 0x60 (0x00) = 0x%x \r\n", temp[8]);
#endif

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_GPIO_CR, &temp[8]);
	printk("Register 0x6A (0x06) = 0x%x \r\n", temp[8]);
#endif

#if DEBUG_PPS
	dm9051_get_reg(db, DM9051_1588_GPIO_TECR, &temp[8]);
	//temp[8] = ior(db, DM9051_1588_GPIO_TECR);
	printk("Register 0x6B (0x02) = 0x%x \r\n", temp[8]);

	printk("temp[0-7] = %x %x %x %x %x %x %x %x \r\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
#endif

	printk("@@@ dm9051_pps_configure trigger time now.tv_sec = 0x%llx, now.tv_nsec = 0x%lx @@@\r\n", now.tv_sec, now.tv_nsec);
	return ret;
}
#endif


// calculate the pulse assert or deassert time
// then split it into high and low part of our register
//      ratio_inv: assert or deassert ratio of the period,
//                 since float is not supported in driver,
//                 the effect ratio is rate_numerator/rate_denominator

static void setup_pulse(struct board_info *db,
			u64 duration_ns, u8 reg_hi, u8 reg_lo)
{
	u64 high, low;
	u16 unit;
	if (duration_ns == 0) {
		dm_printk("[%s] input duration is 0!", __func__);
		return;
	}

	// calculate high and low duration
	if (duration_ns < (1<<13)*120) {
		unit = duration_ns / 120; // 120 ns unit
		high = (0 << 6) | (unit >> 8); // high duration
		low = unit & 0xff; // low duration
	} else if (duration_ns < (1<<13)*1000) {
		unit = duration_ns / 1000; // 1 us unit
		high = (1 << 6) | (unit >> 8); // high duration
		low = unit & 0xff; // low duration
	} else if (duration_ns < (u64)(1<<13)*1000000) {
		unit = duration_ns / 1000000; // 1ms unit
		high = (2 << 6) | (unit >> 8); // high duration
		low = unit & 0xff; // low duration
	} else {
		dm_printk("[%s] duration_ns %llu is out of H/W capability!", __func__, duration_ns);
		return;
	};
        dm9051_set_reg(db, reg_hi, high);
	dm9051_set_reg(db, reg_lo, low);
	dm_printk("[%s] duration_ns=%llu, high=0x%llx, low=0x%llx",
		  __func__, duration_ns, high, low);
}

static int dm9051_perout(struct ptp_clock_info *ptp, struct ptp_clock_request *rq, int on)
{
	struct board_info *db = container_of(ptp, struct board_info, ptp_caps);

	mutex_lock(&db->spi_lockm);
	// if want to see register write debug info, uncomment the following line
	// db->debug_flags |= DP_REG_WRITE;;
        u8 gpio_cr = 0;
        u8 st_gpio = 0;
        u8 gpio_tecr = 0;
	u8 txrx_ctrl = 0;
	int ret = 0;
        const struct ptp_perout_request *perout = &rq->perout;
	struct ptp_pin_desc *pin;
	int chan = 0;

        // 驗證 index 是否合理
	if (rq->perout.index >= ptp->n_pins) {
		dm_printk("[%s] INVALID rq->perout.index (%d) >= ptp->n_pins (%d)", __func__, rq->perout.index, ptp->n_pins);
		ret = -EINVAL;
		goto FINISH_PEROUT;
	}

        // 取得 pin 資訊
	pin = &ptp->pin_config[rq->perout.index];
	chan = pin->chan;

	if (chan > N_CHANNELS) {
		dm_printk("[%s] Channel out of range(%d) on pin: %d, chan: %d", __func__, N_CHANNELS, pin->index, chan);
		ret =  -EINVAL;
		goto FINISH_PEROUT;
	}
	if (pin->func != PTP_PF_PEROUT) {
		dm_printk("[%s] Invalid pin function: %d on pin: %d, expected %d", __func__, pin->func, pin->index,  PTP_PF_PEROUT);
		ret = -EINVAL;
		goto FINISH_PEROUT;
	}

        if (((chan == 0) && (db->extts_mask_gpio & EXTTS_MASK_GP1)) ||
	    ((chan == 1) && (db->extts_mask_gpio & EXTTS_MASK_GP2))) {
		dm_printk("[%s] !!! Pin: %d, requested function: %d channel: %d, in use as EXTTS",
			  __func__, pin->index, pin->func, chan);
		ret = -EINVAL;
                goto FINISH_PEROUT;
	}

	// 印出目前 pin 的參數
	dm_printk("[%s] VALID pin=%d, flags=0x%x", __func__, pin->index, rq->extts.flags);

	// 設定 GPIO status & page reg(0x60)
	dm9051_get_reg8(db, DM9051_1588_ST_GPIO, &st_gpio);
	if (chan == 0) {
		st_gpio &= ~DM9051_1588_GP_PAGE; // select GP1 page
		st_gpio |= DM9051_1588_GP1_ST; // GP1 clear status (write to clear)
	} else {
		st_gpio |= DM9051_1588_GP_PAGE; // select GP2 page
		st_gpio	|= DM9051_1588_GP2_ST; // GP2 clear status (write to clear)
	}

	dm9051_set_reg(db, DM9051_1588_ST_GPIO, st_gpio);
	dm_printk("[%s] Set GPIO (0x60) to 0x%x", __func__, st_gpio);

        // disable trigger and interrupt
	if (!on) {
		dm_printk("[%s] ON is 0, pin=%d, flags=0x%x",
			  __func__, pin->index, rq->extts.flags);
		dm9051_get_reg8(db, DM9051_1588_GPIO_CR, &gpio_cr);
		gpio_cr = gpio_cr & ~(DM9051_GPIO_CR_TRIG_EN | DM9051_GPIO_CR_GP_INT_EN);
		dm9051_set_reg(db, DM9051_1588_GPIO_CR, gpio_cr);
		dm_printk("[%s] Set GPIO (0x6A) to 0x%x", __func__, st_gpio);

		goto FINISH_PEROUT;
	}


	/* 啟用 GPIO 輸出模式 reg(0x6A) */
	dm9051_get_reg8(db, DM9051_1588_GPIO_CR, &gpio_cr);
	gpio_cr &= ~DM9051_GPIO_CR_GP_TYPE; // 設定為輸出
	gpio_cr |=DM9051_GPIO_CR_GP_ST | // 清除 status
		DM9051_GPIO_CR_GP_INT_EN | // 啟用 GPIO 中斷
		DM9051_GPIO_CR_TRIG_EN;   // 啟用觸發
	dm9051_set_reg(db, DM9051_1588_GPIO_CR, gpio_cr);
	dm_printk("[%s] Set GPIO Reg (0x6A) to 0x%x", __func__, gpio_cr);

	/* 設定觸發屬性 reg(0x6B) */
	dm9051_get_reg8(db, DM9051_1588_GPIO_TECR, &gpio_tecr);
	if (db->gpio_polarity[chan] == 0)
		gpio_tecr &= ~DM9051_GPIO_TECR_TRIG_POR; // low active
	else
		gpio_tecr |= DM9051_GPIO_TECR_TRIG_POR; // high active

	// 清除原本的設定
	gpio_tecr &= ~0x0c;
        // 選擇週期或脈衝模式
	if (perout->flags & PTP_PEROUT_DUTY_CYCLE) {
                // 啟用週期性觸發
		gpio_tecr |= 0x0c;
	} else if (perout->flags & PTP_PEROUT_ONE_SHOT) {
		// 啟用單一脈衝觸發
		gpio_tecr |= 0x08;
	}
	dm9051_set_reg(db, DM9051_1588_GPIO_TECR, gpio_tecr);
	dm_printk("[%s] Set GPIO Reg (0x6B) to 0x%x", __func__, gpio_tecr);

	/* 設定觸發波形 (0x6C-0x6F) */
	u64 period_ns, on_time_ns, off_time_ns;
	period_ns = perout->period.sec * NSEC_PER_SEC + perout->period.nsec;

	if (perout->flags & PTP_PEROUT_DUTY_CYCLE) {
		on_time_ns = perout->on.sec * NSEC_PER_SEC + perout->on.nsec;
		if (on_time_ns >= period_ns)
			off_time_ns = 0;
		else
			off_time_ns = period_ns - on_time_ns;
	} else {
		on_time_ns = period_ns / 2;
		off_time_ns = period_ns - on_time_ns;
	}
	dm_printk("[%s] period_ns = %lld, on_time_ns = %lld, off_time_ns = %lld, flags = 0x%x",
		  __func__, period_ns, on_time_ns, off_time_ns, perout->flags);
	// assert
	setup_pulse(db, on_time_ns, DM9051_1588_GPIO_TA_H, DM9051_1588_GPIO_TA_L);
	// deassert
	setup_pulse(db, off_time_ns, DM9051_1588_GPIO_TPDA_H, DM9051_1588_GPIO_TPDA_L);

        /* 啟動輸出 GPIO TxRx Control (0x62) */
	dm9051_get_reg8(db, DM9051_1588_GP_TXRX_CTRL, &txrx_ctrl);
	// 設定 GP1 或 GP2 的觸發輸出
	if (chan == 0) {
		txrx_ctrl |= DM9051_GPTXRX_GP1_TE; // GP1
	} else {
		txrx_ctrl |= DM9051_GPTXRX_GP2_TE; // GP2
	}

	// 如果有 PTP_PEROUT_PHASE, 起始時間就用現在時間加上 perout->phase,
	// 不然則使用 perout->start
	u64 sec; // 起始時間 sec  = perout->start.sec;
	u64 nsec; // 起始時間 ns = perout->start.nsec;
	u8 tsbuf[8];
	if (perout->flags & PTP_PEROUT_PHASE) {
		// 從kernel api 取得時間戳
		struct timespec64 ts;
		ktime_get_real_ts64(&ts);
		// 計算 起始時間
		u64 phase_nsec = perout->phase.sec * NSEC_PER_SEC + perout->phase.nsec;
		nsec = (ts.tv_nsec + phase_nsec) % NSEC_PER_SEC;
		sec = ts.tv_sec + (ts.tv_nsec + phase_nsec) / NSEC_PER_SEC;
	} else {
		nsec = perout->start.nsec % NSEC_PER_SEC;
		sec = perout->start.sec + perout->start.nsec / NSEC_PER_SEC;
	}

	tsbuf[0] = nsec & 0xff;
	tsbuf[1] = (nsec >> 8) & 0xff;
	tsbuf[2] = (nsec >> 16) & 0xff;
	tsbuf[3] = (nsec >> 24) & 0xff;
	tsbuf[4] = sec & 0xff;
	tsbuf[5] = (sec >> 8) & 0xff;
	tsbuf[6] = (sec >> 16) & 0xff;
	tsbuf[7] = (sec >> 24) & 0xff;

	mutex_lock(&db->tsreg_lock);

	// reset index of timestamp register
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	// 寫入 start timestamp
	for (int i = 0; i < 8; i++)
		dm9051_set_reg(db, DM9051_1588_TS, tsbuf[i]);
        mutex_unlock(&db->tsreg_lock);

	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, txrx_ctrl); // trigger load
	dm_printk("[%s] Set GPIO Reg (0x62) to 0x%x", __func__, txrx_ctrl);

	dm_printk("[%s]: PEROUT, start=%lld.%09llu edge=%s\n",
		  __func__, sec, nsec,
		  (perout->flags & PTP_FALLING_EDGE) ? "falling" : "rising");

FINISH_PEROUT:
	db->debug_print &= ~DP_REG_WRITE;
	mutex_unlock(&db->spi_lockm);
	return ret;
}


inline void RUN_EXTTS_WORK(struct board_info *db)
{
	if (unlikely(db->extts_work_count == 0)) {
		dm_printk("!!! start RUN_EXTTS_WORK");
	}
	db->extts_work_count++;
        if (unlikely((db->extts_work_count % 1000000) == 0))
		dm_printk("!!! RUN_EXTTS_WORK count=%d", db->extts_work_count);

        schedule_work(&db->ptp_extts_work);
}

inline void CANCEL_EXTTS_WORK(struct board_info *db)
{
	cancel_work_sync(&db->ptp_extts_work);
	db->extts_work_count = 0;
	dm_printk("!!! CANCEL_EXTTS");
}


static int dm9051_extts(struct ptp_clock_info *ptp, struct ptp_clock_request *rq, int on)
{
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);

	struct ptp_pin_desc *pin;
	u8 st_gpio, gpio_cr, gpio_tecr, txrx_ctrl;
	bool rising = false, falling = false;
	int ret = 0;
	int chan = 0;

        mutex_lock(&db->spi_lockm);

	// 驗證 index 是否合理
	if (rq->extts.index >= ptp->n_pins) {
		dm_printk("[%s] INVALID rq->extts.index >= ptp->n_pins", __func__);
		ret = -EINVAL;
		goto FINISH_EXTTS;
	}

	// 取得 pin 資訊
	pin = &ptp->pin_config[rq->extts.index];
	chan = pin->chan;
	if (chan > N_CHANNELS) {
		dm_printk("[%s] Invalid channel on pin: %d, input chan: %d", __func__, pin->index, chan);
		ret =  -EINVAL;
		goto FINISH_EXTTS;
	}
	if (pin->func != PTP_PF_EXTTS) {
		dm_printk("[%s] Invalid pin function: %d on pin: %d, expected PTP_PF_EXTTS",
			  __func__, pin->func, pin->index);
		ret = -EINVAL;
		goto FINISH_EXTTS;
	}

	// 印出目前 pin 的參數
	dm_printk("[%s] VALID pin=%d, flags=0x%x, on=%d, chan=%d",
		  __func__, pin->index, rq->extts.flags, on, chan);

	// 設定 GPIO status & page reg(0x60)
	dm9051_get_reg8(db, DM9051_1588_ST_GPIO, &st_gpio);
	if (chan == 0) {
		st_gpio &= ~DM9051_1588_GP_PAGE; // select GP1 page
		st_gpio |= DM9051_1588_GP1_ST; // GP1 clear status (write to clear)
	} else {
		st_gpio |= DM9051_1588_GP_PAGE; // select GP2 page
		st_gpio	|= DM9051_1588_GP2_ST; // GP2 clear status (write to clear)
	}
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, st_gpio);
	dm_printk("[%s] Set GPIO (0x60) to 0x%x", __func__, st_gpio);

	// 若未設定 on，代表要關閉功能
	if (!on) {
		dm_printk("[%s] Disabled external timestamping on pin %d", __func__,
			  pin->index);
		dm9051_get_reg8(db, DM9051_1588_GPIO_CR, &gpio_cr);
		// 關閉中斷與觸發設定
		gpio_cr &= ~(DM9051_GPIO_CR_TRIG_EN | DM9051_GPIO_CR_GP_INT_EN);
		dm9051_set_reg(db, DM9051_1588_GPIO_CR, gpio_cr);
		dm_printk("[%s] Set GPIO (0x6A) to 0x%x", __func__, gpio_cr);
		if (chan==0)
			db->extts_mask_gpio &= ~EXTTS_MASK_GP1;
		else
			db->extts_mask_gpio &= ~EXTTS_MASK_GP2;
                ret = 0;
		goto FINISH_EXTTS;
	}

	// 驗證 edge flag，並處理 STRICT 判斷
	rising = rq->extts.flags & PTP_RISING_EDGE;
	falling = rq->extts.flags & PTP_FALLING_EDGE;
	db->extts_flags[chan] = rq->extts.flags;

	if (!rising && !falling) {
		dm_printk("[%s] No rising or falling edge specified", __func__);
		ret = -EINVAL;
		goto FINISH_EXTTS;
	}

	// 設定 GPIO Control (0x6A), 輸入方向 + interrupt enable
	dm9051_get_reg8(db, DM9051_1588_GPIO_CR, &gpio_cr);
	gpio_cr = DM9051_GPIO_CR_TRIG_EN |
		DM9051_GPIO_CR_GP_INT_EN |
		DM9051_GPIO_CR_GP_TYPE; // 設定為輸入
	dm9051_set_reg(db, DM9051_1588_GPIO_CR, gpio_cr);
	dm_printk("[%s] Set GPIO (0x6A) to 0x%x", __func__, gpio_cr);

	// Event Control (0x6B) 設定上升與下降緣
	dm9051_get_reg8(db, DM9051_1588_GPIO_TECR, &gpio_tecr);
	// lock first event
	gpio_tecr |= DM9051_GPIO_TECR_EVENT_LCK;
	if (rising)
		gpio_tecr |= DM9051_GPIO_TECR_GP_R_EVT;
	if (falling)
		gpio_tecr |= DM9051_GPIO_TECR_GP_F_EVT;
	dm9051_set_reg(db, DM9051_1588_GPIO_TECR, gpio_tecr);
	dm_printk("[%s] Set GPIO (0x6B) to 0x%x", __func__, gpio_tecr);

	dm_printk("[%s] Enabled external timestamping on pin %d, rising=%d, falling=%d",
		  __func__, pin->index, rising, falling);

	// 啟動輸出 GPIO TxRx Control (0x62)
	dm9051_get_reg8(db, DM9051_1588_GP_TXRX_CTRL, &txrx_ctrl);
	// 設定 GP1 或 GP2 的觸發輸出
	if (chan == 0) {
		txrx_ctrl |= DM9051_GPTXRX_GP1_TE; // GP1
	} else {
		txrx_ctrl |= DM9051_GPTXRX_GP2_TE; // GP2
	}
	// trigger load
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, txrx_ctrl);

	// if work has not been started, start it
	if (!(db->extts_mask_gpio & EXTTS_MASK_ALL))
		RUN_EXTTS_WORK(db); // 啟動工作以處理外部時間戳記

        // 設定外部時間戳記的 mask
	if (chan == 0)
		db->extts_mask_gpio |= EXTTS_MASK_GP1;
	else
		db->extts_mask_gpio |= EXTTS_MASK_GP2;


FINISH_EXTTS:
	mutex_unlock(&db->spi_lockm);
        return 0;
}


static int ptp_9051_feature_enable(struct ptp_clock_info *ptp,
				   struct ptp_clock_request *rq, int on)
{
	int ret = 0;
	printk("!!! 1. [%s] in rq->type = %d,  on = %d", __func__,  rq->type, on);

	switch (rq->type) {
		case PTP_CLK_REQ_EXTTS:
			printk("2. PTP_CLK_REQ_EXTTS \n");
			printk("   rq->extts.index = %d, rq->extts.flags = 0x%x, rq->extts.rsv[0] = 0x%x, rq->extts.rsv[1] = 0x%x \r\n", rq->extts.index, rq->extts.flags, rq->extts.rsv[0], rq->extts.rsv[1]);
			//func = PTP_PF_EXTTS;
			//chan = rq->extts.index;
			ret = dm9051_extts(ptp, rq, on); //Stone add for enable GP2 event input!
			break;
		case PTP_CLK_REQ_PEROUT:
			printk("3. PTP_CLK_REQ_PEROUT \n");
                        printk("   rq->perout.index = %d, rq->perout.start.sec = %lld, rq->perout.start.nsec = %d, rq->perout.period.sec = %lld, rq->perout.period.nsec = 0%d",
			       rq->perout.index, rq->perout.start.sec, rq->perout.start.nsec, rq->perout.period.sec, rq->perout.period.nsec);
			//func = PTP_PF_PEROUT;
			//chan = rq->perout.index;
			ret = dm9051_perout(ptp, rq, on); //Stone add for read GP2 status, and read PTP clock data to ptp4l!
			break;
		case PTP_CLK_REQ_PPS:
			printk("4. ptp_9051_feature_enable PTP_CLK_REQ_PPS do nothing\n");
			//return i40e_pps_configure(ptp, rq, on);
			//Stone add for 1588 pps enable "echo 1 > /sys/class/ptp/ptp0/pps_enable"
			//if (on){
			//	ret = dm9051_pps_configure(ptp, rq);
			//}
			break;
		default:
			ret = 0;;
	}

	//Stone add for 1588 pps
	return ret;

}



// process_extts(int gpio)
//   Helper function to processing external timestamp event
//   @db: board information
//   @chan: 0 for GP1, 1 for GP2
//   @st_gpio: current status of GPIO register

inline void process_extts(struct board_info *db, int chan, u8 st_gpio)
{
	u8 ts[8];
	struct ptp_clock_event event;

	// 先分析 input status register，判斷是上升或下降緣
        int type = (chan == 0) ? DM9051_1588_GP1_TYPE : DM9051_1588_GP2_TYPE;
	int rising_edge = st_gpio & type;
	char *gp_name = (chan == 0) ? "GP1" : "GP2";

	mutex_lock(&db->spi_lockm);
	int read_ts_source = (chan == 0) ? TS_SOURCE_GP1 : TS_SOURCE_GP2;
	ptp_9051_read_ts(db, ts, read_ts_source);
	st_gpio &= 0x03; // 保留非 staus 的設定
	if (chan == 0)
		st_gpio |= DM9051_1588_GP1_ST; // write clear GP1 status
	else
		st_gpio |= DM9051_1588_GP2_ST; // write clear GP2 status
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, st_gpio);
	mutex_unlock(&db->spi_lockm);
	// print timestamp
	u32* p = (u32*)ts;
	if ((rising_edge && db->extts_flags[chan] & PTP_RISING_EDGE) ||
	    (!rising_edge && db->extts_flags[chan] & PTP_FALLING_EDGE)) {
		dm_printk("%s event : %d sec %09d ns, edge = %s", gp_name, p[1], p[0],
			  (rising_edge) ? "rising":"falling");
		event.type = PTP_CLOCK_EXTTS;
		event.index = ptp_find_pin(db->ptp_clock, PTP_PF_EXTTS, chan);
		event.timestamp = p[1] * NSEC_PER_SEC + p[0];
		ptp_clock_event(db->ptp_clock, &event);
	}
}

static void dm9051_ptp_extts_work(struct work_struct *work)
{
	u8 st_gpio;
        struct board_info *db = container_of(work, struct board_info, ptp_extts_work);

        // read PTP Status Reg (0x60)
	mutex_lock(&db->spi_lockm);
	dm9051_get_reg8(db, DM9051_1588_ST_GPIO, &st_gpio);
	mutex_unlock(&db->spi_lockm);
	// GP1 has status
        if ((db->extts_mask_gpio & EXTTS_MASK_GP1) &&
	    (st_gpio & DM9051_1588_GP1_ST)) {
		process_extts(db, 0, st_gpio);
	}
	// GP2 has status
	if ((db->extts_mask_gpio & EXTTS_MASK_GP2) &&
	    (st_gpio & DM9051_1588_GP2_ST)) {
		process_extts(db, 1, st_gpio);
	}
        // 重新啟動工作以便下次事件處理
	if (db->extts_mask_gpio & EXTTS_MASK_ALL) {
		RUN_EXTTS_WORK(db);
	} else {
		dm_printk("!!! EXTTS work finished !!!");
	}
}



static int ptp_9051_verify_pin(struct ptp_clock_info *ptp, unsigned int pin,
			       enum ptp_pin_function func, unsigned int chan)
{
	//struct board_info *db = container_of(ptp, struct board_info,
	//			   	         ptp_caps);

	printk("!!! 1. ptp_9051_verify_pin in\n");

	return 0;
}

/**
 * dm9051_ptp_get_ts_config - get hardware time stamping config
 * @netdev:
 * @ifreq:
 *
 * Get the hwtstamp_config settings to return to the user. Rather than attempt
 * to deconstruct the settings from the registers, just return a shadow copy
 * of the last known settings.
 **/

int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	struct hwtstamp_config *config = &db->tstamp_config;
        dm_printk("[in %s()]", __FUNCTION__);
	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;

}

/**
 *  dm9051_ptp_set_timestamp_mode - setup hardware for timestamping
 *  @adapter: our device structure
 *  @config: hwtstamp configuration
 *
 *  Outgoing time stamping can be enabled and disabled. Play nice and
 *  disable it when requested, although it shouldn't case any overhead
 *  when no packet needs it. At most one packet in the queue may be
 *  marked for time stamping, otherwise it would be impossible to tell
 *  for sure to which packet the hardware time stamp belongs.
 *
 *  Incoming time stamping has to be configured via the hardware
 *  filters. Not all combinations are supported, in particular event
 *  type has to be specified. Matching the kind of event packet is
 *  not supported, with the exception of "all V2 events regardless of
 *  level 2 or 4".
 */

static int dm9051_ptp_set_timestamp_mode(struct board_info *db,
					 struct hwtstamp_config *config)
{
	u32 rx_ctl = 0;
	u32 tx_ctl = 0;
	u32 rx_cfg = 0;
	bool is_l4 = false;
	bool is_l2 = false;
	int ret = 0;


	/* reserved for future extensions */
	//if (config->flags)
	//	return -EINVAL;

	if (config->tx_type < __HWTSTAMP_TX_CNT) {
		tx_ctl = (1 << config->tx_type);
	} else {
		ret = -ERANGE;
		goto end_func;
	}

	switch (config->rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			rx_ctl = 0;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			rx_ctl |= DM9051_RXCTL_TYPE_L4_V1;
			rx_cfg = DM9051_RXCFG_PTP_V1_SYNC_MESSAGE;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			rx_ctl |= DM9051_RXCTL_TYPE_L4_V1;
			rx_cfg = DM9051_RXCFG_PTP_V1_DELAY_REQ_MESSAGE;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V2_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			rx_ctl |= DM9051_RXCTL_TYPE_EVENT_V2;
			config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			is_l2 = true;
			is_l4 = true;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		case HWTSTAMP_FILTER_ALL:
			/*
			 * timestamp all packets, which it needs to do to
			 * support both V1 Sync and Delay_Req messages
			 */
			rx_ctl |= DM9051_RXCTL_TYPE_ALL;
			config->rx_filter = HWTSTAMP_FILTER_ALL;
			break;
		default:
			config->rx_filter = HWTSTAMP_FILTER_NONE;
			ret = -ERANGE;
			goto end_func;
	}

	/*
	 * Per-packet timestamping only works if all packets are
	 * timestamped, so enable timestamping in all packets as
	 * long as one rx filter was configured.
	 */
	config->rx_filter = HWTSTAMP_FILTER_ALL;
	is_l2 = true;
	is_l4 = true;
end_func:
	db->rx_ctl = rx_ctl;
	db->tx_ctl = tx_ctl;
	db->rx_cfg = rx_cfg;
	db->is_l2 = is_l2;
	db->is_l4 = is_l4;
        dm_printk("[in %s()] config fields: tx_type = %X, rx_filter = %X, flags = %X, return = %d",
		  __FUNCTION__, config->tx_type, config->rx_filter, config->flags, ret);
	return ret;
}

/**
 * dm9051_ptp_set_ts_config - set hardware time stamping config
 * @netdev:
 * @ifreq:
 *
 **/
int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	struct hwtstamp_config config;
	int err;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	err = dm9051_ptp_set_timestamp_mode(db, &config);
	if (err) {
		dm_printk("[in %s()] return err = %d", __FUNCTION__, err);
		return err;
	}

	/* save these settings for future reference */
	memcpy(&db->tstamp_config, &config,
	       sizeof(db->tstamp_config));

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

inline int dm9051_get_reg8(struct board_info *db, u8 reg, u8 *val)
{
	unsigned int intval;
	int ret = dm9051_get_reg(db, reg, &intval);
	*val = intval & 0xff;
	return ret;
}

// Read Captured Time Stamp and report back
void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamps;
	u8 temp[8];
	ptp_9051_read_ts(db, temp, TS_SOURCE_TXTSTAMP);
  	u64 ns;
        u32* p = (u32*)temp;

#ifdef DE_TIMESTAMP
	if (db->ptp_msgtype == PTP_MSGTYPE_PDELAY_REQ) {
		dm_printk(" TXTXTXTX PDELAY_REQ hwtstamp sec = %d, ns = %d", p[1], p[0]);
	} else if (db->ptp_msgtype == PTP_MSGTYPE_PDELAY_RESP) {
		dm_printk(" TXTXTXTX PDELAY_RESP hwtstamp sec = %d, ns = %d", p[1], p[0]);
	}
#endif
        ns = p[1] * 1000000000ULL + p[0];
	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &shhwtstamps);
}


//
// dm9051_ptp_rx_hwtstamp():
//     parse 8 bytes RX timestamp read from chip packet header, then put in skb
//
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte)
{
	struct skb_shared_hwtstamps *shhwtstamps = NULL;

	// timestamp read from dm9051a memory is big-endian
	// x86-pc & raspberry pi is little-endian
	u32 sec = be32_to_cpu(*(u32*)rxTSbyte);
	u64 ns = be32_to_cpu(*(u32*)(rxTSbyte + 4));

	ns += sec * 1000000000ULL;
	shhwtstamps = skb_hwtstamps(skb);
	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);

	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x08); //Clear RX Time Stamp Clock Register offset 0x60, value 0x08

}


static void ptp_9051_tx_work(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info,
					     ptp_tx_work);
	int tsynctxctl;
	printk("==> ptp_9051_tx_work in \r\n");
	if (!db->ptp_tx_skb)
		return;

#if 0  //Stone add
	if (time_is_before_jiffies(db->ptp_tx_start +
				   IGB_PTP_TX_TIMEOUT)) {
		dev_kfree_skb_any(db->ptp_tx_skb);
		db->ptp_tx_skb = NULL;
		clear_bit_unlock(__IGB_PTP_TX_IN_PROGRESS, &db->state);
		db->tx_hwtstamp_timeouts++;
		dev_warn(&db->pdev->dev, "clearing Tx timestamp hang\n");
		return;
	}
#endif

	//tsynctxctl = ior(db, DM9051_1588_ST_GPIO);        //Read register 0x60 bit 2
	dm9051_get_reg(db, DM9051_1588_ST_GPIO, &tsynctxctl);
        //printk("ptp_9051_tx_work register 0x60 = %x \r\n", tsynctxctl);
#if 0
	if (tsynctxctl & 0x04)   //
		dm9051_ptp_tx_hwtstamp(db);
	else
		/* reschedule to check later */
		schedule_work(&db->ptp_tx_work);
#endif
	printk("<== ptp_9051_tx_work out \r\n");
}

const char dm9051_ptp_driver_name[] = "DM9051 PTP";

void dm9051_ptp_init(struct board_info *db)
{
	// default pin description, user can override this with IOCTL(fd, PTP_PIN_SET_DESC)
	struct ptp_pin_desc  default_pin_desc[N_PINS] = {
		{ .index = 0, .name = GP1_EXTTS, .func = PTP_PF_EXTTS, .chan = 0 },
		{ .index = 1, .name = GP1_PEROUT, .func = PTP_PF_PEROUT, .chan = 0 },
		{ .index = 2, .name = GP2_EXTTS, .func = PTP_PF_EXTTS, .chan = 1 },
		{ .index = 3, .name = GP2_PEROUT, .func = PTP_PF_PEROUT, .chan = 1 },
	};

        strncpy(db->ptp_caps.name, dm9051_ptp_driver_name, sizeof(db->ptp_caps.name));
	memcpy(db->pin_desc, default_pin_desc, sizeof(default_pin_desc));

	db->ptp_caps.owner = THIS_MODULE;
	db->ptp_caps.max_adj = 50000000;
	db->ptp_caps.n_ext_ts = N_EXT_TS;
        db->ptp_caps.n_per_out = N_PER_OUT;
	db->ptp_caps.n_pins = N_PINS;
	// setup pin configuration
	db->ptp_caps.pin_config = db->pin_desc;
	db->ptp_caps.pps = 1;  //Stone add for 1588 pps

	db->ptp_caps.adjfine = ptp_9051_adjfine;
	db->ptp_caps.adjtime = ptp_9051_adjtime;
	db->ptp_caps.gettime64 = ptp_9051_gettime;
	db->ptp_caps.settime64 = ptp_9051_settime;
	db->ptp_caps.enable = ptp_9051_feature_enable;
	db->ptp_caps.verify = ptp_9051_verify_pin;
	db->ptp_caps.getcrosststamp = ptp_9051_getcrosststamp;
	db->last_rate = 0;

	INIT_WORK(&db->ptp_tx_work, ptp_9051_tx_work);

        db->tstamp_config.flags = 0;
	db->tstamp_config.rx_filter =
		(1 << HWTSTAMP_FILTER_ALL) |
		(1 << HWTSTAMP_FILTER_SOME) |
		(1 << HWTSTAMP_FILTER_NONE);
	db->tstamp_config.tx_type =
		(1 << HWTSTAMP_TX_ONESTEP_SYNC) |
		(1 << HWTSTAMP_TX_ON) |
		(1 << HWTSTAMP_TX_OFF);
	// gpio polarity: 0 for active low, 1 for active high
	db->gpio_polarity[0] = DM9051_DEFAULT_POLARITY_GP1;
	db->gpio_polarity[1] = DM9051_DEFAULT_POLARITY_GP2;

        printk("ptp_clock_register in \r\n");
	db->ptp_clock = ptp_clock_register(&db->ptp_caps,
					   &db->ndev->dev);
        printk("ptp_clock_register end \r\n");
	if (IS_ERR(db->ptp_clock)) {
		db->ptp_clock = NULL;
		dev_err(&db->spidev->dev, "ptp_clock_register failed\n");
	}  else if (db->ptp_clock) {
		printk("added PHC on %s\n",
		       db->ndev->name);
	}
	db->debug_print = 0;

	INIT_WORK(&db->ptp_extts_work, dm9051_ptp_extts_work);

	// Enable h/w capture rx timestamp
	dm9051_set_reg(db, DM9051_1588_RX_CONF1,
		       DM9051A_RC_SLAVE | DM9051A_RC_RXTS_EN | DM9051A_RC_RX2_EN);
        // Enable PTP h/w functions
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_EN);

	//Stone add for one-step Sync packet insert time stamp! 2024-08-14!
	//Stone add for 1588 Read 0x68 in one SPI cycle enable (register 0x63 bit 6 0:enable, 1:disable => 0x40)
	//Stone add for 1588 TX 1-Step checksum enable (register 0x63 bit 7 0:enable, 1:disable => 0x80)
	dm9051_set_reg(db, DM9051_1588_1_STEP_CHK, 0x00);

	// DA1082S_E1_design_report_part1.pdf datasheet.
	// 實際測試 dm9051a_ptp_pps_set GP2 enable, mac reg[0x3C] = 0xA0, bit 7~4 = 0xA, bit 3~0 = 0x0
	// Set MAC REG_3CH = 0xa0
	// GP1: 80ns pulse per 1 sec
	// GP2: toggle per 1 sec
	// dm9051_set_reg(db, 0x3C, 0xA0);

	// Set MAC REG_3CH = 0xb0
	// LNKLED: 80ns pulse per 1 sec
	// SPDLED: toggle per 1 sec
	// dm9051_set_reg(db, 0x3C, 0xB0);
}

static void dump_ptp_pins(struct board_info *db)
{
	printk("PTP Pins:\n");
	for (int i = 0; i < db->ptp_caps.n_pins; i++) {
		struct ptp_pin_desc *pin = &db->ptp_caps.pin_config[i];
		printk("Pin %d: name=%s, func=%d, chan=%d\n",
		       pin->index, pin->name, pin->func, pin->chan);
	}
}

void dm9051_ptp_stop(struct board_info *db)
{
	cancel_work_sync(&db->ptp_extts_work);
	if (db->ptp_clock) {
		dump_ptp_pins(db);
		ptp_clock_unregister(db->ptp_clock);
		db->ptp_clock = NULL;
		printk("dm9051_ptp_stop remove PTP clock!!!\r\n");
	}
}

inline void dm9051_setup_1_step_ts_offset(struct board_info *db)
{
	switch (db->ptp_class & PTP_CLASS_PMASK) {
		case PTP_CLASS_IPV4:
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_OFFSET,
				       DM9051_1588_1_STEP_ADDR_OFFSET_IPV4);
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_IPV4);
			break;
		case PTP_CLASS_IPV6:
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_OFFSET,
				       DM9051_1588_1_STEP_ADDR_OFFSET_IPV6);
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_IPV6);
			break;
		case PTP_CLASS_L2:
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_OFFSET,
				       DM9051_1588_1_STEP_ADDR_OFFSET_L2);
			dm9051_set_reg(db,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET,
				       DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_L2);
			break;
		default:
			dm_printk("[%s] Unsupported PTP class: 0x%x",
				  __func__, db->ptp_class);
			break;
	}

}

inline void dm9051_ptp_setup_before_tx(struct board_info *db, struct sk_buff* skb)
{
	db->ptp_tx_flags = skb_shinfo(skb)->tx_flags;
        if (db->ptp_tx_flags) {
		// 這個封包需要硬體層的 timestamp，請稍後在傳送完成時回填
		if (db->ptp_tx_flags & SKBTX_HW_TSTAMP)
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

		db->ptp_class = ptp_classify_raw(skb);
		dm9051_setup_1_step_ts_offset(db);
		//dm_printk("[%s] PTP Class: 0x%x", __func__, db->ptp_class);
		db->ptp_hdr = ptp_parse_header(skb, db->ptp_class);
		db->ptp_msgtype = ptp_get_msgtype(db->ptp_hdr, db->ptp_class);
		// 在 skb 的 tx_flags 是 SKBTX_SW_TSTAMP 的情形下,
                // kernel 會發送/回報 timestamp, 但是 SKBTX_HW_TSTAMP
                // 就留給硬體處理(dm9051_ptp_tx_hwtstamp())
		skb_tx_timestamp(skb);
	}
}

inline bool use_hwtstamp(struct board_info *db)
{
	return db->tstamp_config.tx_type == HWTSTAMP_TX_ON;
}

// dm9051_ptp_process_after_tx():
//     讀取時間戳記並回報

inline void dm9051_ptp_process_after_tx(struct board_info *db, struct sk_buff* skb)
{
	int will_report_ts = 0;
	// 考慮需要硬體時間戳記的情形
	if (unlikely(db->ptp_tx_flags & SKBTX_HW_TSTAMP)) {
		if (is_ptp_two_step(db)) {
			// Two-step 都不需要在 TCR 設定時間戳記 (TCR_DIS_JABBER_TIMER)
			switch (db->ptp_msgtype) {
				case PTP_MSGTYPE_SYNC:
					// two-step SYNC 需要 capture timestamp, 這是 t1
					will_report_ts = 1;
					break;
				case PTP_MSGTYPE_DELAY_REQ:
					// two-step DELAY_REQ 需要 capture timestamp, 這是 t3
					will_report_ts = 1;
					break;
				case PTP_MSGTYPE_PDELAY_REQ:
					// two-step PDelay_Req 需要 capture timestamp, 這是 t1'
					will_report_ts = 1;
					break;
				case PTP_MSGTYPE_PDELAY_RESP:
					// two-step PDelay_Resp 需要 capture timestamp, 這是 t3'
					will_report_ts = 1;
					break;
			}
		}
		if (is_ptp_one_step(db)) {
			switch(db->ptp_msgtype) {
				case PTP_MSGTYPE_SYNC:
					// one-step SYNC 不需要 capture timestamp 回報
					will_report_ts = 0;
					break;
				case PTP_MSGTYPE_DELAY_REQ:
					// one-step DELAY_REQ 需要 capture timestamp 回報爲
					// t3
					will_report_ts = 1;
					break;
				case PTP_MSGTYPE_PDELAY_REQ:
					// one-step PDelay_Req 需要 capture timestamp
					// 回報爲 t1'
					will_report_ts = 1;
					break;
				case PTP_MSGTYPE_PDELAY_RESP:
					// one-step PDelay_Resp 需要 capture timestamp
					// 回報爲 t3'
					will_report_ts = 1;
					break;
			}
		}
	}
	if (will_report_ts) {
		// 需要回報 timestamp
		dm9051_ptp_tx_hwtstamp(db, skb);
	}
}

// dm9051_ptp_tcr_flags():
//     根據 PTP 設定和訊息類型，決定是否需要在 TCR 寫入時間戳記相關的 flags

inline u8 dm9051_ptp_tcr_flags(struct board_info *db)
{
	u8 tcr_flags = 0;

	// 如果需要硬體時間戳記，則設定 TCR 的時間戳記相關 flags
	if (unlikely(db->ptp_tx_flags & SKBTX_HW_TSTAMP)) {
		// Two step 情況下 SYNC, DELAY_REQ
		// 還有 PDelay_Req, PDelay_Resp
		// 不需要在 TCR 設定時間戳記 flags
		if (is_ptp_two_step(db)) {
			// Two-step 都不需要在 TCR 設定時間戳記 (TCR_DIS_JABBER_TIMER)
			switch (db->ptp_msgtype) {
				case PTP_MSGTYPE_SYNC:
					// two-step SYNC 需要 capture timestamp, 這是 t1
					tcr_flags = TCR_TS_EN;
					break;
				case PTP_MSGTYPE_DELAY_REQ:
					// two-step DELAY_REQ 需要 capture timestamp, 這是 t3
					tcr_flags = TCR_TS_EN;
					break;
				case PTP_MSGTYPE_PDELAY_REQ:
					// two-step PDelay_Req 需要 capture timestamp, 這是 t1'
					tcr_flags = TCR_TS_EN;
					break;
				case PTP_MSGTYPE_PDELAY_RESP:
					// two-step PDelay_Resp 需要 capture timestamp, 這是 t3'
					tcr_flags = TCR_TS_EN;
					break;
			}
		}
		if (is_ptp_one_step(db)) {
			switch(db->ptp_msgtype) {
				case PTP_MSGTYPE_SYNC:
					// one-step SYNC 不需要 capture timestamp 回報爲
					// t1, 但是要送出 timestamp
					tcr_flags = TCR_DIS_JABBER_TIMER;
					break;
				case PTP_MSGTYPE_DELAY_REQ:
					// one-step DELAY_REQ 需要 capture timestamp 回報爲
					// t3, 不需要送出 timestamp
					tcr_flags = TCR_TS_EN;
					break;
				case PTP_MSGTYPE_PDELAY_REQ:
					// one-step PDelay_Req 需要 capture timestamp
					// 回報爲 t1', 也要送出 timestamp
					tcr_flags = TCR_TS_EN | TCR_DIS_JABBER_TIMER;
					break;
				case PTP_MSGTYPE_PDELAY_RESP:
					// one-step PDelay_Resp 需要 capture timestamp
					// 回報爲 t3', 也要送出 timestamp
					tcr_flags = TCR_TS_EN | TCR_DIS_JABBER_TIMER;
					break;
			}
		}
	}

	return tcr_flags;
}



inline bool is_ptp_two_step(struct board_info *db)
{
	switch (db->tstamp_config.tx_type) {
		case HWTSTAMP_TX_ON:
			// two-step 模式：送出後由軟體/驅動讀回 timestamp
			return true;
		case HWTSTAMP_TX_ONESTEP_SYNC:
		case HWTSTAMP_TX_ONESTEP_P2P:
			// one-step 模式：timestamp 直接寫入封包中
			return false;
		case HWTSTAMP_TX_OFF:
		default:
			// 沒有啟用 timestamping，視為 non two-step
			return false;
	}
}
inline bool is_ptp_one_step(struct board_info *db)
{
	switch (db->tstamp_config.tx_type) {
		case HWTSTAMP_TX_ON:
			// two-step 模式：送出後由軟體/驅動讀回 timestamp
			return false;
		case HWTSTAMP_TX_ONESTEP_SYNC:
		case HWTSTAMP_TX_ONESTEP_P2P:
			// one-step 模式：timestamp 直接寫入封包中
			return true;
		case HWTSTAMP_TX_OFF:
		default:
			// 沒有啟用 timestamping，視為 non one-step
			return false;
	}
}

MODULE_DESCRIPTION("Davicom DM9051 1588 driver");
MODULE_LICENSE("GPL");
