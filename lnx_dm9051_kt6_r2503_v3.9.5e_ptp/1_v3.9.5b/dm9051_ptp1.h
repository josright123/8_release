/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PTPC_H_
#define _DM9051_PTPC_H_

#ifdef DMPLUG_PTP
struct board_info;
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

void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);

//implement in ptpd
//static void dm9051_ptp_core_init(struct board_info *db);

void ptp_new(struct board_info *db, struct net_device *ndev);
void ptp_init_rcr(struct board_info *db);
u8 ptp_status_bits(struct board_info *db);
int is_ptp_rxts_enable(struct board_info *db);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
#endif

//#define NOT_REQUEST_SUPPORTTED	0x0
//#define VOID_REQUEST_FUNCTION		-9
//#define REQUEST_SUPPORTTED		1 //REQUEST_SUPPORTTED (1)

/* Optional functions declaration const */
enum dm_req_not_support {
	VOID_REQUEST_FUNCTION =		-9,
	NOT_REQUEST_SUPPORTTED =	0,
};
enum dm_req_support {
	REQUEST_SUPPORTTED =		1,
};

/* Optional functions and dadicated function */
//#define SECOND_MAIN //(sec)
#define FAK //(fake)

//[fake.1]
#if defined(FAK) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
//#ifdef _MAIN_DATA
/* raw fake encrypt */
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty

/* fake raw tx mode */
#define dmplug_tx "normal"
#define TX_CONTI_NEW(d)

/* poll fake */
#define dm9051_poll_supp()		NOT_REQUEST_SUPPORTTED
#define dm9051_poll_sch(d)		VOID_REQUEST_FUNCTION

#define dm9051_int2_supp()		NOT_REQUEST_SUPPORTTED
#define dm9051_int2_irq(d,h)		VOID_REQUEST_FUNCTION

/* raw(fake) bmsr_wr */
#define PHY_READ(d, n, av) dm9051_phyread(d, n, av)

//[fake]
#define SHOW_DEVLOG_REFER_BEGIN(d, b)
#define SHOW_LOG_REFER_BEGIN(b)
#define SHOW_DEVLOG_MODE(d)

#define SHOW_PLAT_MODE(d)
#define SHOW_MAC(b, a)
#define SHOW_MONITOR_RXC(b, n)

#define dm9051_headlog_regs(h, b, r1, r2)
#define dm9051_phyread_headlog(h, b, r)	(void)0
#define dm9051_dump_data1(b, p, l)
#define monitor_rxb0(b, rb)
//#endif //_MAIN_DATA
#endif

//[fake.2]
#if defined(FAK) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
/* fake ptpc */
#define PTP_NEW(d, n)			d->ptp_enable = 0
#define PTP_INIT_RCR(d)
#define PTP_INIT(d)
#define PTP_END(d)
#endif

/* CO, re-direct */
#define CO //(Coerce)

/* re-direct ptpc */
#if defined(CO) && defined(DMPLUG_PTP) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef PTP_NEW
#define PTP_NEW(d, n) ptp_new(d, n)
#undef PTP_INIT_RCR
#define PTP_INIT_RCR(d) ptp_init_rcr(d)
#undef PTP_INIT
#define PTP_INIT(d) ptp_init(d)
#undef PTP_END
#define PTP_END(d) ptp_end(d)
#endif

#endif //_DM9051_PTPC_H_
