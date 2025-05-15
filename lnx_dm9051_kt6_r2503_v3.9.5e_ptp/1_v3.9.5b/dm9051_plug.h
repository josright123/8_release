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

/*
 * Engineering Verification
 */
//#ifdef _MAIN_DATA
 //#ifdef DMCONF_AARCH_64
 //#pragma message("dm9051 AARCH_64")
 //#else
 //#pragma message("dm9051 AARCH_32")
 //#endif

 //#ifdef DMCONF_DIV_HLPR_32
 //#pragma message("dm9051 DIV_HLPR_32")
 //#endif
//#endif //_MAIN_DATA

#if defined(DMPLUG_INT)
#ifdef INT_CLKOUT
#endif
#ifdef INT_TWO_STEP
void INIT_RX_INT2_DELAY_SETUP(struct board_info *db);
void dm9051_rx_irq_servicep(struct work_struct *work);
irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw);
#endif
#else
void dm9051_poll_servicep(struct work_struct *work);
void INIT_RX_POLL_DELAY_SETUP(struct board_info *db);
void INIT_RX_POLL_SCHED_DELAY(struct board_info *db);
#endif

#ifdef DMCONF_BMCR_WR
int dm9051_phyread_nt_bmsr(struct board_info *db, unsigned int reg, unsigned int *val);
#endif

#ifdef DMCONF_MRR_WR
#endif

/*
 * Conti: 
 */
#ifdef DMPLUG_CONTI
/* Log definitions */
#define dmplug_tx "continue"
void tx_contu_new(struct board_info *db);
int TX_MOTE2_CONTI_RCR(struct board_info *db);
int TX_MODE2_CONTI_TCR(struct board_info *db, struct sk_buff *skb, u64 tx_timeout_us);
#endif

//[overlay]
#ifdef DMPLUG_CRYPT
//overlay by plug/
#undef BUS_SETUP
#define BUS_SETUP(db) bus_setup(struct board_info *db)
#undef BUS_OPS
#define BUS_OPS(db, buff, crlen) bus_ops(struct board_info *db, u8 *buff, unsigned int crlen)
//implement in plug/
int bus_setup(struct board_info *db);
void bus_ops(struct board_info *db, u8 *buff, unsigned int crlen);
#endif

/* 0.0 ptpc */
#ifdef DMPLUG_PTP
//=
//of #include "dm9051_ptpd.h"
#include <linux/ptp_classify.h>
#include <linux/ip.h>
#include <linux/udp.h>
//implement in ptpd
void ptp_new(struct board_info *db, struct net_device *ndev);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
#endif

/* 0.1 ptpc */
#ifdef DMPLUG_PTP
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
#endif //_DM9051_PLUG_H_
