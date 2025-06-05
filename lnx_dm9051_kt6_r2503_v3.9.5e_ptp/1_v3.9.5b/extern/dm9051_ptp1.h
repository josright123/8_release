/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PTPC_H_
#define _DM9051_PTPC_H_
//#include <linux/ptp_clock_kernel.h>
//#include <linux/ptp_classify.h>
//#include <linux/ip.h>
//#include <linux/udp.h>

/*#define DMPLUG_PTP */ //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */ //(ptp1588 pps)
/*#define DMPLUG_PTP_TWO_STEP */ //(ptp1588 two step)

/* Capabilities:
 *        hardware-transmit
 *        hardware-receive
 *        hardware-raw-clock
 */
#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)

#define PLUG_PTP_PPS
#ifdef PLUG_PTP_PPS
#define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
#endif

/* "dm9051 PTP TWO STEP"
 * Always essential (Mandartory recommanded)
 * (if not support, after the master send sync, NO follow up can be available to send.)
 */
#define PLUG_PTP_TWO_STEP //(always essential)
#ifdef PLUG_PTP_TWO_STEP
#define DMPLUG_PTP_TWO_STEP //(always essential)(HW Two step support)
#endif
#endif

/*Capabilities:
 *        software-transmit
 *        software-receive
 *        software-system-clock
 *PTP Hardware Clock: none
 *Hardware Transmit Timestamp Modes: none
 *Hardware Receive Filter Modes: none
 */
#define PLUG_PTP_1588_SW
#ifdef PLUG_PTP_1588_SW
#define DMPLUG_PTP_SW //(ptp 1588 S/W)
#endif

/* pragma
 */
#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
//#warning "dm9051 PTP"
#pragma message("dm9051: H/W PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
//#warning "dm9051 PPS"
#pragma message("dm9051: H/W PPS")
#endif
#if defined(DMPLUG_PTP_TWO_STEP) && defined(MAIN_DATA)
//#warning "dm9051 PTP TWO STEP"
#pragma message("dm9051: H/W PTP TWO STEP")
#endif

#if defined(DMPLUG_PTP_SW) && defined(MAIN_DATA)
#pragma message("dm9051: S/W PTP (TWO STEP)")
#endif

//#ifdef DMPLUG_PTP .. #endif

/*
 * ptp 1588:
 */
#define DM9051_1588_ST_GPIO 0x60
#define DM9051_1588_CLK_CTRL 0x61
#define DM9051_1588_GP_TXRX_CTRL 0x62
//#define DM9051_1588_TX_CONF 0x63
#define DM9051_1588_1_STEP_CHK 0x63
#define DM9051_1588_RX_CONF1 0x64
//#define DM9051_1588_RX_CONF2 0x65
#define DM9051_1588_1_STEP_ADDR 0x65
//#define DM9051_1588_RX_CONF3 0x66
#define DM9051_1588_1_STEP_ADDR_CHK 0x66
#define DM9051_1588_CLK_P 0x67
#define DM9051_1588_TS 0x68
//#define DM9051_1588_AUTO 0x69
#define DM9051_1588_MNTR 0x69
#define DM9051_1588_GPIO_CONF 0x6A
#define DM9051_1588_GPIO_TE_CONF 0x6B
#define DM9051_1588_GPIO_TA_L 0x6C
#define DM9051_1588_GPIO_TA_H 0x6D
#define DM9051_1588_GPIO_DTA_L 0x6E
#define DM9051_1588_GPIO_DTA_H 0x6F

// 61H Clock Control Reg
#define DM9051_CCR_IDX_RST BIT(7)
#define DM9051_CCR_RATE_CTL BIT(6)
#define DM9051_CCR_PTP_RATE BIT(5)
#define DM9051_CCR_PTP_ADD BIT(4)
#define DM9051_CCR_PTP_WRITE BIT(3)
#define DM9051_CCR_PTP_READ BIT(2)
#define DM9051_CCR_PTP_DIS BIT(1)
#define DM9051_CCR_PTP_EN BIT(0)

// 64H
#define DM9051A_RC_SLAVE BIT(7)
#define DM9051A_RC_RX_EN BIT(4)
#define DM9051A_RC_RX2_EN BIT(3)
#define DM9051A_RC_FLTR_MASK 0x3
#define DM9051A_RC_FLTR_ALL_PKTS 0
#define DM9051A_RC_FLTR_MCAST_PKTS 1
#define DM9051A_RC_FLTR_DA 2
#define DM9051A_RC_FLTR_DA_SPICIFIED 3

#define DM9051_1588_TS_BULK_SIZE 8

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
/* PTP header flag fields */
#define PTP_FLAG_TWOSTEP	BIT(1)
#endif

/* PTP message type constants */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
#define PTP_MSGTYPE_SYNC             0x0
#define PTP_MSGTYPE_DELAY_REQ        0x1
#endif
//#define PTP_MSGTYPE_PDELAY_REQ     0x2
//#define PTP_MSGTYPE_PDELAY_RESP    0x3
#define PTP_MSGTYPE_FOLLOW_UP        0x8
#define PTP_MSGTYPE_DELAY_RESP       0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP 0xA
#define PTP_MSGTYPE_ANNOUNCE         0xB
#define PTP_MSGTYPE_SIGNALING        0xC
#define PTP_MSGTYPE_MANAGEMENT       0xD

// PTP FIELD
#define PTP_ETHERTYPE 0x88F7    // Layer 2 PTP
#define PTP_EVENT_PORT 319      // UDP PTP EVENT
#define PTP_GENERAL_PORT 320    // UDP PTP GENERAL

/* 0.1 ptpc */
// bits defines
// 06H RX Status Reg
// BIT(5),PTP use the same bit, timestamp is available
// BIT(3),PTP use the same bit, this is odd parity rx TimeStamp
// BIT(2),PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_EN		BIT(5)
#define RSR_RXTS_PARITY		BIT(3)
#define RSR_RXTS_LEN		BIT(2)
#define	RSR_PTP_BITS		(RSR_RXTS_EN | RSR_RXTS_PARITY | RSR_RXTS_LEN)

/* PTP message type classification */
enum ptp_sync_type {
	//PTP_NOT_PTP = 0,      /* Not a PTP packet or no timestamp involved */
	PTP_ONE_STEP = 1,     /* One-step sync message */
	PTP_TWO_STEP = 2,     /* Two-step sync message */
};

//typedef struct ptp_board_info {
//} ptp_board_info_t;

int is_ptp_rxts_enable(struct board_info *db);

struct ptp_header *get_ptp_header(struct sk_buff *skb);
u8 get_ptp_message_type005(struct ptp_header *ptp_hdr);
int is_ptp_sync_packet(u8 msgtype);
int is_ptp_delayreq_packet(u8 msgtype);

/* macro fakes
 */
#if 1
//struct board_info;

#define PTP_VER(b)
#define PTP_VER_SOFTWARE(b)

#define PTP_NEW(d)				0
#define PTP_INIT_RCR(d)
#define PTP_INIT(d)
#define PTP_END(d)
#define PTP_ETHTOOL_INFO(s)
#define PTP_STATUS_BITS(b)			RSR_ERR_BITS
#define PTP_NETDEV_IOCTL(s)
#define PTP_AT_RATE(b)

/* ptp2 */
#define DMPLUG_RX_TS_MEM(b)		0
#define DMPLUG_RX_HW_TS_SKB(b,s)
#define SHOW_ptp_rx_packet_monitor(b,s)
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b)

#define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	//0
#define DMPLUG_PTP_TX_PRE(b,s)
#define DMPLUG_TX_EMIT_TS(b,s)

/* ptp sw */
#define DMPLUG_PTP_TX_TIMESTAMPING_SW(s)
#endif

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
//int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info); //kt612
//#else
//int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info); //kt66
//#endif

/* ethtool_ops
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
static inline int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info)
#else
static inline int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
#endif
{
	struct board_info *db = netdev_priv(net_dev);
	ptp_board_info_t *pbi = &db->pbi;

//Spenser - get phc_index
	//info->phc_index = -1;
	info->phc_index = pbi->ptp_clock ? ptp_clock_index(pbi->ptp_clock) : -1;

	info->so_timestamping = 0;
#if 1
#if defined(DMPLUG_PTP_SW)
	/* .software ts */
	info->so_timestamping |=
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE;
#endif
#endif
#if defined(DMPLUG_PTP)
	info->so_timestamping |=
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
#endif

#if defined(DMPLUG_PTP)
	info->tx_types =
		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
		BIT(HWTSTAMP_TX_OFF) |
		BIT(HWTSTAMP_TX_ON);
#endif

#if defined(DMPLUG_PTP)
	info->rx_filters =
		BIT(HWTSTAMP_FILTER_NONE) |
		BIT(HWTSTAMP_FILTER_ALL);
#endif
	return 0;
}

/* netdev_ops
 */
int dm9051_ptp_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);

/* functions re-construct
 */
/* CO1, */
/*#define CO1*/ //(Coerce)

/* ptp */
#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
#undef PTP_ETHTOOL_INFO
#define PTP_ETHTOOL_INFO(s)		s = dm9051_ts_info,
#undef PTP_NETDEV_IOCTL
#define PTP_NETDEV_IOCTL(s)		s = dm9051_ptp_netdev_ioctl,
#endif

#if defined(DMPLUG_PTP) /*&& defined(MAIN_DATA) && defined(CO1)*/
/* re-direct ptpc */
#undef PTP_VER
#undef PTP_NEW
#undef PTP_INIT_RCR
#undef PTP_INIT
#undef PTP_END
#undef PTP_STATUS_BITS
#undef PTP_AT_RATE

#define PTP_VER(b)		ptp_ver(b)
#define PTP_NEW(d) 				ptp_new(d)
#define PTP_INIT_RCR(d) 		ptp_init_rcr(d)
#define PTP_INIT(d) 			ptp_init(d)
#define PTP_END(d) 				ptp_end(d)
#define PTP_STATUS_BITS(b)			ptp_status_bits(db)
#define PTP_AT_RATE(b)	on_core_init_ptp_rate(b)

#undef DMPLUG_RX_TS_MEM
#undef DMPLUG_RX_HW_TS_SKB
#undef SHOW_ptp_rx_packet_monitor
#undef DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER

#define DMPLUG_RX_TS_MEM(b)		dm9051_read_ptp_tstamp_mem(b)
#define DMPLUG_RX_HW_TS_SKB(b,s) dm9051_ptp_rx_hwtstamp(b,s)
#define SHOW_ptp_rx_packet_monitor(b,s) dm9051_ptp_rx_packet_monitor(b,s)
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b) \
		dm9051_ptp_rxc_from_master(b)

#undef DMPLUG_PTP_TX_IN_PROGRESS
#undef DMPLUG_PTP_TX_PRE
#undef DMPLUG_TX_EMIT_TS

#define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	dm9051_ptp_tx_in_progress(b,s)
#define DMPLUG_PTP_TX_PRE(b,s)	dm9051_ptp_txreq(b,s)
#define DMPLUG_TX_EMIT_TS(b,s)	dm9051_ptp_txreq_hwtstamp(b,s)
#endif

void ptp_ver(struct board_info *db);
int ptp_new(struct board_info *db);
void ptp_init_rcr(struct board_info *db);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
u8 ptp_status_bits(struct board_info *db);
void on_core_init_ptp_rate(struct board_info *db);

int dm9051_read_ptp_tstamp_mem(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rxc_from_master(struct board_info *db);

void dm9051_ptp_tx_in_progress(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_txreq(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb);

#if defined(DMPLUG_PTP_SW)
/* re-direct ptp sw */
#undef PTP_VER_SOFTWARE
#define PTP_VER_SOFTWARE(b)	ptp_ver_software(b)

#undef DMPLUG_PTP_TX_TIMESTAMPING_SW
#define DMPLUG_PTP_TX_TIMESTAMPING_SW(s)	dm9051_ptp_tx_swtstamp(s)
#endif

void ptp_ver_software(struct board_info *db);
void dm9051_ptp_tx_swtstamp(struct sk_buff *skb);

/* ptp, clkout, 2step */
#if defined(DMPLUG_PTP)
#undef INFO_PTP
#define INFO_PTP(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT)
#undef INFO_PPS
#define INFO_PPS(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PPS")
#endif
#if defined(DMPLUG_PTP_TWO_STEP)
#undef INFO_PTP2S
#define INFO_PTP2S(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PTP TWO STEP")
#endif

#if defined(DMPLUG_PTP_SW)
#undef INFO_PTP_SW_2S
#define INFO_PTP_SW_2S(dev, db)				USER_CONFIG(dev, db, "dm9051: S/W PTP (TWO STEP)")
#endif

#endif //_DM9051_PTPC_H_
