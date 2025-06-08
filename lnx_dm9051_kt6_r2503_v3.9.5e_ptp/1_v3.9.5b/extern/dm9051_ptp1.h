/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PTPC_H_
#define _DM9051_PTPC_H_
// #include <linux/ptp_clock_kernel.h>
// #include <linux/ptp_classify.h>
// #include <linux/ip.h>
// #include <linux/udp.h>

// #ifdef DMPLUG_PTP .. #endif

/*
 * ptp 1588:
 */
#define DM9051_1588_ST_GPIO			 0x60
#define DM9051_1588_CLK_CTRL		 0x61
#define DM9051_1588_GP_TXRX_CTRL	 0x62
// #define DM9051_1588_TX_CONF 0x63
#define DM9051_1588_1_STEP_CHK		 0x63
#define DM9051_1588_RX_CONF1		 0x64
// #define DM9051_1588_RX_CONF2 0x65
#define DM9051_1588_1_STEP_ADDR		 0x65
// #define DM9051_1588_RX_CONF3 0x66
#define DM9051_1588_1_STEP_ADDR_CHK	 0x66
#define DM9051_1588_CLK_P			 0x67
#define DM9051_1588_TS				 0x68
// #define DM9051_1588_AUTO 0x69
#define DM9051_1588_MNTR			 0x69
#define DM9051_1588_GPIO_CONF		 0x6A
#define DM9051_1588_GPIO_TE_CONF	 0x6B
#define DM9051_1588_GPIO_TA_L		 0x6C
#define DM9051_1588_GPIO_TA_H		 0x6D
#define DM9051_1588_GPIO_DTA_L		 0x6E
#define DM9051_1588_GPIO_DTA_H		 0x6F

// 61H Clock Control Reg
#define DM9051_CCR_IDX_RST			 BIT(7)
#define DM9051_CCR_RATE_CTL			 BIT(6)
#define DM9051_CCR_PTP_RATE			 BIT(5)
#define DM9051_CCR_PTP_ADD			 BIT(4)
#define DM9051_CCR_PTP_WRITE		 BIT(3)
#define DM9051_CCR_PTP_READ			 BIT(2)
#define DM9051_CCR_PTP_DIS			 BIT(1)
#define DM9051_CCR_PTP_EN			 BIT(0)

// 64H
#define DM9051A_RC_SLAVE			 BIT(7)
#define DM9051A_RC_RX_EN			 BIT(4)
#define DM9051A_RC_RX2_EN			 BIT(3)
#define DM9051A_RC_FLTR_MASK		 0x3
#define DM9051A_RC_FLTR_ALL_PKTS	 0
#define DM9051A_RC_FLTR_MCAST_PKTS	 1
#define DM9051A_RC_FLTR_DA			 2
#define DM9051A_RC_FLTR_DA_SPICIFIED 3

#define DM9051_1588_TS_BULK_SIZE	 8

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 10, 11)
  /* PTP header flag fields */
  #define PTP_FLAG_TWOSTEP BIT(1)
#endif

/* PTP message type constants */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 10, 11)
  #define PTP_MSGTYPE_SYNC		0x0
  #define PTP_MSGTYPE_DELAY_REQ 0x1
#endif

// #define PTP_MSGTYPE_PDELAY_REQ     0x2
// #define PTP_MSGTYPE_PDELAY_RESP    0x3
#define PTP_MSGTYPE_FOLLOW_UP			  0x8
#define PTP_MSGTYPE_DELAY_RESP			  0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP 0xA
#define PTP_MSGTYPE_ANNOUNCE			  0xB
#define PTP_MSGTYPE_SIGNALING			  0xC
#define PTP_MSGTYPE_MANAGEMENT			  0xD

// PTP FIELD
#define PTP_ETHERTYPE					  0x88F7 // Layer 2 PTP
#define PTP_EVENT_PORT					  319	 // UDP PTP EVENT
#define PTP_GENERAL_PORT				  320	 // UDP PTP GENERAL

/* 0.1 ptpc */
// bits defines
// 06H RX Status Reg
// BIT(5),PTP use the same bit, timestamp is available
// BIT(3),PTP use the same bit, this is odd parity rx TimeStamp
// BIT(2),PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_EN						  BIT(5)
#define RSR_RXTS_PARITY					  BIT(3)
#define RSR_RXTS_LEN					  BIT(2)
#define RSR_PTP_BITS					  (RSR_RXTS_EN | RSR_RXTS_PARITY | RSR_RXTS_LEN)

/* PTP message type classification */
enum ptp_sync_type
{
  // PTP_NOT_PTP = 0,      /* Not a PTP packet or no timestamp involved */
  PTP_ONE_STEP = 1, /* One-step sync message */
  PTP_TWO_STEP = 2, /* Two-step sync message */
};

int				   is_ptp_rxts_enable(struct board_info *db);
struct ptp_header *get_ptp_header(struct sk_buff *skb);
u8				   get_ptp_message_type005(struct ptp_header *ptp_hdr);
int				   is_ptp_sync_packet(u8 msgtype);
int				   is_ptp_delayreq_packet(u8 msgtype);

// typedef struct ptp_board_info {
// } ptp_board_info_t;

#endif //_DM9051_PTPC_H_
