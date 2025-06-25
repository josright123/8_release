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

// #ifdef _DMPLUG_PTP .. #endif
/* pragma
 */
#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
    #pragma message("dm9051: H/W PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
    #pragma message("dm9051: H/W PPS")
#endif
#if defined(DMPLUG_PTP_TWO_STEP) && defined(MAIN_DATA)
    #pragma message("dm9051: H/W PTP TWO STEP")
#endif

/* ptp, clkout, 2step
 */
#if defined(DMPLUG_PTP)
    #undef INFO_PTP
    #define INFO_PTP(dev, db) USER_CONFIG(dev, db, "dm9051: H/W PTP")
	
	#if defined(DMPLUG_PPS_CLKOUT)
    #undef INFO_PPS
    #define INFO_PPS(dev, db) USER_CONFIG(dev, db, "dm9051: H/W PPS")
	#endif
	#if defined(DMPLUG_PTP_TWO_STEP)
    #undef INFO_PTP2S
    #define INFO_PTP2S(dev, db) USER_CONFIG(dev, db, "dm9051: H/W PTP TWO STEP")
	#endif
#endif

	/* ptp */
	#if defined(DMPLUG_PTP) /*&& defined(MAIN_DATA) && defined(CO1) (re-direct ptpc) */
		#undef PTP_VER
		#undef PTP_SETUP
		#undef PTP_CHECKSUM_LIMIT
		// #undef PTP_NEW
		#undef PTP_INIT
		#undef PTP_END
		#undef PTP_STATUS_BITS
		#undef PTP_AT_RATE
		#define PTP_VER(b)                ptp_ver(b)
		#define PTP_SETUP(b)              ptp_operation_extern(b)
		#define PTP_CHECKSUM_LIMIT(b, nd) ptp_checksum_limit(b, nd)
		// #define PTP_NEW(d)			  ptp_new(d)
		#define PTP_INIT(d)               ptp_init(d)
		#define PTP_END(d)                ptp_end(d)
		#define PTP_STATUS_BITS(b)        ptp_status_bits(db)
		#define PTP_AT_RATE(b)            	on_core_init_ptp_rate(b)

		#undef PTP_CONSTRAIN
		#define PTP_CONSTRAIN(n, f) 		dm9051_ptp_fix_features(n, f)
		#undef DMPLUG_RX_TS_MEM
		#undef DMPLUG_RX_HW_TS_SKB
		#define DMPLUG_RX_TS_MEM(b)       dm9051_read_ptp_tstamp_mem(b)
		#define DMPLUG_RX_HW_TS_SKB(b, s) dm9051_ptp_rx_hwtstamp(b, s)
		#undef DMPLUG_SHOW_ptp_rx_packet_monitor
		#define DMPLUG_SHOW_ptp_rx_packet_monitor(b, s) dm9051_ptp_rx_packet_monitor(b, s)
		#undef DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER
		#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b) dm9051_ptp_rxc_from_master(b)
		#undef SINGLE_TX // udef
		#define SINGLE_TX(b, s) dm9051_ptp_single_tx(b, s)
		// #undef DMPLUG_PTP_TX_IN_PROGRESS
		// #undef DMPLUG_PTP_TX_PRE
		// #undef DMPLUG_TX_EMIT_TS
		// #define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	dm9051_ptp_tx_in_progress(b,s)
		// #define DMPLUG_PTP_TX_PRE(b,s)			dm9051_ptp_tcr_2wr(b,s)
		// #define DMPLUG_TX_EMIT_TS(b,s)			dm9051_ptp_txreq_hwtstamp(b,s)
	#endif


/* ~(ptp sw ||) final global ptp */
	#if defined(DMPLUG_PTP) /* || defined(_DMPLUG_PTP_SW)*/
		#undef INIT_RCR
		#define INIT_RCR(b)           	  b->rctl.rcr_all = (RCR_ALL | RCR_DIS_LONG | RCR_RXEN) //ptp_init_rcr(d)
	#endif

/*
 * ptp 1588:
 */
#define DM9051_1588_ST_GPIO          0x60
#define DM9051_1588_CLK_CTRL         0x61
#define DM9051_1588_GP_TXRX_CTRL     0x62
// #define DM9051_1588_TX_CONF 0x63
#define DM9051_1588_1_STEP_CHK       0x63
#define DM9051_1588_RX_CONF1         0x64
// #define DM9051_1588_RX_CONF2 0x65
#define DM9051_1588_1_STEP_ADDR      0x65
// #define DM9051_1588_RX_CONF3 0x66
#define DM9051_1588_1_STEP_ADDR_CHK  0x66
#define DM9051_1588_CLK_P            0x67
#define DM9051_1588_TS               0x68
// #define DM9051_1588_AUTO 0x69
#define DM9051_1588_MNTR             0x69
#define DM9051_1588_GPIO_CONF        0x6A
#define DM9051_1588_GPIO_TE_CONF     0x6B
#define DM9051_1588_GPIO_TA_L        0x6C
#define DM9051_1588_GPIO_TA_H        0x6D
#define DM9051_1588_GPIO_DTA_L       0x6E
#define DM9051_1588_GPIO_DTA_H       0x6F

// 02H TX Control Reg
#define TCR_TSEN_CAP                 TCR_RSV_BIT7
#define TCR_TS1STEP_EMIT             TCR_DIS_JABBER_TIMER

// 61H Clock Control Reg
#define DM9051_CCR_IDX_RST           BIT(7)
#define DM9051_CCR_RATE_CTL          BIT(6)
#define DM9051_CCR_PTP_RATE          BIT(5)
#define DM9051_CCR_PTP_ADD           BIT(4)
#define DM9051_CCR_PTP_WRITE         BIT(3)
#define DM9051_CCR_PTP_READ          BIT(2)
#define DM9051_CCR_PTP_DIS           BIT(1)
#define DM9051_CCR_PTP_EN            BIT(0)

// 64H
#define DM9051A_RC_SLAVE             BIT(7)
#define DM9051A_RC_RX_EN             BIT(4)
#define DM9051A_RC_RX2_EN            BIT(3)
#define DM9051A_RC_FLTR_MASK         0x3
#define DM9051A_RC_FLTR_ALL_PKTS     0
#define DM9051A_RC_FLTR_MCAST_PKTS   1
#define DM9051A_RC_FLTR_DA           2
#define DM9051A_RC_FLTR_DA_SPICIFIED 3

#define DM9051_1588_TS_BULK_SIZE     8

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 10, 11)
    /* PTP header flag fields */
    #define PTP_FLAG_TWOSTEP BIT(1)
#endif

/* PTP message type constants */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 10, 11)
    #define PTP_MSGTYPE_SYNC      0x0
    #define PTP_MSGTYPE_DELAY_REQ 0x1
#endif

#define PTP_MSGTYPE_SYNC_pri                  0x0
#define PTP_MSGTYPE_DELAY_REQ_pri             0x1
#define PTP_MSGTYPE_PDELAY_REQ_pri            0x2 // #define PTP_MSGTYPE_PDELAY_REQ     0x2
#define PTP_MSGTYPE_PDELAY_RESP_pri           0x3 // #define PTP_MSGTYPE_PDELAY_RESP    0x3
#define PTP_MSGTYPE_DELAY_RESP_pri            0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri 0xA

#define PTP_MSGTYPE_FOLLOW_UP                 0x8
#define PTP_MSGTYPE_DELAY_RESP                0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP     0xA
#define PTP_MSGTYPE_ANNOUNCE                  0xB
#define PTP_MSGTYPE_SIGNALING                 0xC
#define PTP_MSGTYPE_MANAGEMENT                0xD

// PTP FIELD
#define PTP_ETHERTYPE                         0x88F7 // Layer 2 PTP
#define PTP_EVENT_PORT                        319    // UDP PTP EVENT
#define PTP_GENERAL_PORT                      320    // UDP PTP GENERAL

/* 0.1 ptpc */
// bits defines
// 06H RX Status Reg
// BIT(5),PTP use the same bit, timestamp is available
// BIT(3),PTP use the same bit, this is odd parity rx TimeStamp
// BIT(2),PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_EN                           BIT(5)
#define RSR_RXTS_PARITY                       BIT(3)
#define RSR_RXTS_LEN                          BIT(2)
#define RSR_PTP_BITS                          (RSR_RXTS_EN | RSR_RXTS_PARITY | RSR_RXTS_LEN)

/* PTP message type classification */
enum ptp_sync_type
{
    // PTP_NOT_PTP = 0,      /* Not a PTP packet or no timestamp involved */
    PTP_ONE_STEP = 1, /* One-step sync message */
    PTP_TWO_STEP = 2, /* Two-step sync message */
};

int is_ptp_rxts_en(struct board_info *db);

struct ptp_header *get_ptp_header(struct sk_buff *skb);

struct ptp_header *dm9051_rx_ptp_hdr_monitor(struct board_info *db);
u8  get_ptp_message_type005(struct ptp_header *ptp_hdr);
int is_ptp_announce_packet(u8 msgtype);
int is_ptp_sync_packet(u8 msgtype);
int is_ptp_delayreq_packet(u8 msgtype);
int is_ptp_delayresp_packet(u8 msgtype);
int is_peer_delayreq_packet(u8 msgtype);
int is_peer_delayresp_packet(u8 msgtype);

// typedef struct ptp_board_info {
// } ptp_board_info_t;

#endif //_DM9051_PTPC_H_
