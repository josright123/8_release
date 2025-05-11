#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_

/* Macro domain
 */
/*#define DMPLUG_CONTI */ //(conti)
/*#define DMPLUG_CRYPT */ //(crypt)
/*#define DMPLUG_PTP */ //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */ //(ptp1588 pps)

/* Macro for already known platforms
 */ 
//#define PLUG_MODEN
#ifdef PLUG_MODEN
#define DMPLUG_CONTI //(conti)
#endif

//#define PLUG_CUSTOMIZE_CRYP
#ifdef PLUG_CUSTOMIZE_CRYP
#define DMPLUG_CRYPT //(crypt)
#endif

#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)

  #define PLUG_PTP_PPS
  #ifdef PLUG_PTP_PPS
  #define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
  #endif
#endif

#ifdef MAIN_DATA
/*
 * MAIN Data: 
 */
const struct driver_config confdata = {
	.release_version = "lnx_dm9051_kt6631_r2502_v3.9.1",
};
const struct eng_config engdata = {
	.force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,
	.sched = {
		.delayF = {0, 1, 0, 0, 1}, 
		.nTargetMaxNum = POLL_OPERATE_NUM},
	.tx_timeout_us = 210000, //2100,
};
const struct eng_config *econf = &engdata;
const struct eng_sched csched = engdata.sched;
#endif

/*
 * Engineering Verification
 */
#ifdef MAIN_DATA
//#ifdef DMCONF_AARCH_64
//#pragma message("dm9051 AARCH_64")
////#warning "dm9051 AARCH_64"
//#else
//#pragma message("dm9051 AARCH_32")
////#warning "dm9051 AARCH_32"
//#endif

//#ifdef DMCONF_DIV_HLPR_32
//#pragma message("dm9051 DIV_HLPR_32")
////#warning "dm9051 DIV_HLPR_32"
//#endif

#ifdef DMPLUG_CONTI
#warning "dm9051 CONTI"
#endif

#ifdef DMPLUG_CRYPT
#warning "dm9051 CRYPT"
#endif

#ifdef DMPLUG_PTP
#pragma message("dm9051 PTP")

#ifdef DMPLUG_PPS_CLKOUT
#pragma message("dm9051 PPS")
//#warning "dm9051 PPS"
#endif
#endif
#endif //MAIN_DATA

//inline
#ifdef DMPLUG_CRYPT
int BUS_SETUP(struct board_info *db);
void BUS_OPS(struct board_info *db, u8 *buff, unsigned int crlen);
#else
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty
#endif

/*
 * Conti: 
 */
#ifdef DMPLUG_CONTI
int TX_MOTE2_CONTI_RCR(struct board_info *db);
int TX_MODE2_CONTI_TCR(struct board_info *db, struct sk_buff *skb);
#endif

/* Log definitions */
#ifdef DMPLUG_CONTI
#define dmplug_tx "continue"
#else
#define dmplug_tx "normal"
#endif

/* 0.1 ptpc */
#if 1 //0
#ifdef DMPLUG_PTP
//=
//of #include "dm9051_ptpd.h"
#include <linux/ptp_classify.h>
#include <linux/ip.h>
#include <linux/udp.h>
// bits defines
// 06H RX Status Reg
// BIT(5),PTP use the same bit, timestamp is available
// BIT(3),PTP use the same bit, this is odd parity rx TimeStamp
// BIT(2),PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_EN		BIT(5)
#define RSR_RXTS_PARITY		BIT(3)
#define RSR_RXTS_LEN		BIT(2)
#define	RSR_PTP_BITS		(RSR_RXTS_EN | RSR_RXTS_PARITY | RSR_RXTS_LEN)
/* ethtool_ops
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info); //kt612
#else
int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info); //kt66
#endif
/* netdev_ops
 */
int dm9051_ptp_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);
void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb);
int dm9051_read_ptp_tstamp_mem(struct board_info *db, u8 *rxTSbyte);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);
void dm9051_ptp_txreq(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb);
u32 dm9051_get_rate_reg(struct board_info *db);
void dm9051_ptp_init(struct board_info *db);
void dm9051_ptp_stop(struct board_info *db);
#endif
#endif

#endif //_DM9051_PLUG_H_
