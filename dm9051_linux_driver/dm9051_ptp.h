/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

#ifndef _DM9051_PTP_H_
#define _DM9051_PTP_H_
#include <linux/bits.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/version.h>  // 確保使用 LINUX_VERSION_CODE 和 KERNEL_VERSION

struct board_info;
#define N_EXT_TS	4
#define N_PER_OUT       4
#define N_PINS          4
#define N_CHANNELS      2

// pin names
#define GP1_EXTTS       "GP1_EXTTS"
#define GP1_PEROUT      "GP1_PEROUT"
#define GP2_EXTTS       "GP2_EXTTS"
#define GP2_PEROUT      "GP2_PEROUT"

// Latch Timestamp Register (0x68) from GP2, GP1 or Immediately
#define TS_SOURCE                     int
#define	TS_SOURCE_GP2                 2
#define	TS_SOURCE_GP1                 1
#define	TS_SOURCE_TXTSTAMP            0
#define DM9051_DEFAULT_POLARITY_GP1   1 // 1: active high, 0: active low
#define DM9051_DEFAULT_POLARITY_GP2   1 // 1: active high, 0: active low

#undef TT9052
#define DM9051A
#undef DE_TIMESTAMP

#define check_TT9052 0 //Stone add for DM9051A one-step Sync packet insert time stamp! 2024-08-14! (1:disable (TT9052 mode), 0:enable)

#ifdef TT9052
#define DM9051_1588_ST_GPIO 0x60
#define DM9051_1588_CLK_CTRL 0x61
#define DM9051_1588_GP_TXRX_CTRL x62
#define DM9051_1588_TX_CONF 0x63
#define DM9051_1588_RX_CONF1 0x64
#define DM9051_1588_RX_CONF2 0x65
#define DM9051_1588_RX_CONF3 0x66
#define DM9051_1588_CLK_P 0x67
#define DM9051_1588_TS 0x68
#define DM9051_1588_AUTO 0x69
#define DM9051_1588_GPIO_CR 0x6A
#define DM9051_1588_GPIO_TECR x6B
#define DM9051_1588_GPIO_TA_L 0x6C
#define DM9051_1588_GPIO_TA_H 0x6D
#define DM9051_1588_GPIO_DTA_L 0x6E
#define DM9051_1588_GPIO_DTA_H 0x6F
#endif // TT9052


#define DM9051_RXCTL_TYPE_L2_V2	0x00
#define DM9051_RXCTL_TYPE_L4_V1	0x02
#define DM9051_RXCTL_TYPE_L2_L4_V2	0x04
#define DM9051_RXCTL_TYPE_ALL	0x08
#define DM9051_RXCTL_TYPE_EVENT_V2	0x0A
#define DM9051_RXCTL_SYSCFI		0x00000020 /* Sys clock frequency */

#define DM9051_RXCFG_PTP_V1_CTRLT_MASK		0x000000FF
#define DM9051_RXCFG_PTP_V1_SYNC_MESSAGE		0x00
#define DM9051_RXCFG_PTP_V1_DELAY_REQ_MESSAGE	0x01
#define DM9051_RXCFG_PTP_V1_FOLLOWUP_MESSAGE	0x02
#define DM9051_RXCFG_PTP_V1_DELAY_RESP_MESSAGE	0x03
#define DM9051_RXCFG_PTP_V1_MANAGEMENT_MESSAGE	0x04

#define DM9051_RXCFG_PTP_V2_MSGID_MASK		0x00000F00
#define DM9051_RXCFG_PTP_V2_SYNC_MESSAGE		0x0000
#define DM9051_RXCFG_PTP_V2_DELAY_REQ_MESSAGE	0x0100
#define DM9051_RXCFG_PTP_V2_PATH_DELAY_REQ_MESSAGE	0x0200
#define DM9051_RXCFG_PTP_V2_PATH_DELAY_RESP_MESSAGE	0x0300
#define DM9051_RXCFG_PTP_V2_FOLLOWUP_MESSAGE	0x0800
#define DM9051_RXCFG_PTP_V2_DELAY_RESP_MESSAGE	0x0900
#define DM9051_RXCFG_PTP_V2_PATH_DELAY_FOLLOWUP_MESSAGE 0x0A00
#define DM9051_RXCFG_PTP_V2_ANNOUNCE_MESSAGE	0x0B00
#define DM9051_RXCFG_PTP_V2_SIGNALLING_MESSAGE	0x0C00
#define DM9051_RXCFG_PTP_V2_MANAGEMENT_MESSAGE	0x0D00



#ifdef DM9051A
#define DM9051_1588_ST_GPIO 0x60
#define DM9051_1588_CLK_CTRL 0x61
#define DM9051_1588_GP_TXRX_CTRL 0x62

#define DM9051_1588_1_STEP_CHK 0x63
#define DM9051_1588_RX_CONF1 0x64
#define DM9051_1588_1_STEP_ADDR_OFFSET 0x65 // Address Offset Register
#define DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET 0x66 // Address Offset Register
#define DM9051_1588_CLK_P 0x67 // Clock Period Register
#define DM9051_1588_TS 0x68 // Timestamp Register
//#define DM9051_1588_AUTO 0x69
#define DM9051_1588_MNTR 0x69 // Monitor Register

#define DM9051_1588_GPIO_CR 0x6A // GPIO Control Register
#define DM9051_1588_GPIO_TECR 0x6B // GPIO Trigger Event Control Register

#define DM9051_1588_GPIO_TA_L 0x6C // GPIO Trigger Assert Pulse Low Register
#define DM9051_1588_GPIO_TA_H 0x6D // GPIO Trigger Assert Pulse High Register
#define DM9051_1588_GPIO_TPDA_L 0x6E // GPIO Trigger Periodic De-asserted Pulse Low Register
#define DM9051_1588_GPIO_TPDA_H 0x6F // GPIO Trigger Periodic De-asserted Pulse High Register

// bits defines
// 60H
#define DM9051_1588_GP2_ST BIT(7)
#define DM9051_1588_GP2_TYPE BIT(6) // 0: falling edge, 1: rising edge
#define DM9051_1588_GP1_ST BIT(5)
#define DM9051_1588_GP1_TYPE BIT(4)
#define DM9051_1588_GP_PAGE BIT(1) // 0: GP1, 1: GP2
#define DM9051_1588_PTP_RST BIT(0)

// 61H Clock Control Reg
#define DM9051_CCR_IDX_RST BIT(7)
#define DM9051_CCR_RATE_CTL BIT(6)
#define DM9051_CCR_PTP_RATE BIT(5)
#define DM9051_CCR_PTP_ADD BIT(4)
#define DM9051_CCR_PTP_WRITE BIT(3)
#define DM9051_CCR_PTP_READ BIT(2)
#define DM9051_CCR_PTP_DIS BIT(1)
#define DM9051_CCR_PTP_EN BIT(0)

// 62H
#define DM9051_GPTXRX_INT_MASK BIT(7)
// bit 6 reserved
#define DM9051_GPTXRX_GP2_TE   BIT(5)
#define DM9051_GPTXRX_GP1_TE   BIT(4)
// bit 3:1 reserved
// read tx tstamp clock
#define DM9051_GPTXRX_RD_TS    BIT(0)

// 64H
#define DM9051A_RC_SLAVE BIT(7)
#define DM9051A_RC_RXTS_EN BIT(4)
#define DM9051A_RC_RX2_EN BIT(3)
#define DM9051A_RC_FLTR_MASK 0x3
#define DM9051A_RC_FLTR_ALL_PKTS 0
#define DM9051A_RC_FLTR_MCAST_PKTS 1
#define DM9051A_RC_FLTR_DA 2
#define DM9051A_RC_FLTR_DA_SPICIFIED 3



// 65H
/*
  originTimestamp = Ethernet Header(14) +
    IP Header (IPV4: 20, IPV6: 40) +
    UDP Header (8) +
    PTP Header Offset (usually 34)
    Davicom use 8 of 10 bytes to store the timestamp, so shift 2 bytes

    IPv4	14 + 20 + 8 + 34 = 76	0x4C -> Davicom start from 0x4E
    IPv6	14 + 40 + 8 + 34 = 96	0x60 -> Davicom start from 0x62
    L2          14 + 0 + 0 + 34 = 48	0x30 -> Davicom start from 0x32
*/

#define DM9051_1588_1_STEP_ADDR_OFFSET_IPV4 0x4E
#define DM9051_1588_1_STEP_ADDR_OFFSET_IPV6 0x62
#define DM9051_1588_1_STEP_ADDR_OFFSET_L2 0x32

// 66H
/*
   from <linux/ptp_classify.h>
   offset of ptp_header.reserved2 (i.e. 16 offset, 4 bytes size)
   <-- DM9051A use 4 of 2 bytes to store the checksum compansation

   reserved2 = Ethernet Header(14) +
   IP Header (IPV4: 20, IPV6: 40) +
   UDP Header (8) +
   reserved2 offset in ptp_header(16)

   IPV4        14 + 20 + 8 + 16 = 58	0x3A -> Davicom start from 0x3C
   IPV6        14 + 40 + 8 + 16 = 78	0x4E -> Davicom start from 0x50
   L2          14 + 0 + 0 + 16 = 30	0x1E -> Davicom start from 0x20
 */
#define DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_IPV4 0x3C
#define DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_IPV6 0x50
#define DM9051_1588_1_STEP_ADDR_CHKSUM_OFFSET_L2 0x20

// 69H
#define DM9051A_MNTR_IDX_68H 0xF0
#define DM9051A_MNTR_RD_RAT BIT(0)

// 6AH
#define DM9051_GPIO_CR_GP_ST BIT(7) // 0: GP2 disable, 1: GP2 ready
#define DM9051_GPIO_CR_GP_RISING BIT(6) // 0: GP event is falling, 1: GP event is rising
#define DM9051_GPIO_CR_GP_INT_EN BIT(2) // 0: disable, 1: enable
#define DM9051_GPIO_CR_TRIG_EN BIT(1) // 0: trigger & event disable, 1: enable
#define DM9051_GPIO_CR_GP_TYPE BIT(0) // 0: trigger output, 1: event input

// 6BH
#define DM9051_GPIO_TECR_EVENT_LCK BIT(6) // 0: lock 1st event, 1: overwrite
#define DM9051_GPIO_TECR_GP_R_EVT BIT(5) // 1: detect rising edge, 0: no detect
#define DM9051_GPIO_TECR_GP_F_EVT BIT(4) // 1: detect falling edge, 0: no detect
#define DM9051_GPIO_TECR_TRIG_POR BIT(1) // 0: trigger output active low, 1: active high



#endif //DM9051A


#define DM9051_1588_TS_BULK_SIZE 8
#define DM9051_RC_RXTS_EN BIT(4)
#define DM9051_RC_RXTS_FLTR_MASK GENMASK(1, 0)
// board_info.extts_mask_gpio, set & cleared in extts(), and checked by the extts work thread
#define EXTTS_MASK_GP1 BIT(0)
#define EXTTS_MASK_GP2 BIT(1)
#define EXTTS_MASK_ALL (EXTTS_MASK_GP1 | EXTTS_MASK_GP2)


void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);
void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_init(struct board_info *db);
void dm9051_ptp_stop(struct board_info *db);
int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr);
int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr);
inline u8 dm9051_ptp_tcr_flags(struct board_info *db);
inline void dm9051_ptp_setup_before_tx(struct board_info *db, struct sk_buff* skb);
inline void dm9051_ptp_process_after_tx(struct board_info *db, struct sk_buff* skb);
inline bool is_ptp_two_step(struct board_info *db);
inline bool is_ptp_one_step(struct board_info *db);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)
int dm9051_get_ts_info(struct net_device *net_dev,
                       struct kernel_ethtool_ts_info *ts_info);
#else
int dm9051_get_ts_info(struct net_device *net_dev,
                       struct ethtool_ts_info *ts_info);
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(6, 7, 0)


#endif // _DM9051_PTP_H_
