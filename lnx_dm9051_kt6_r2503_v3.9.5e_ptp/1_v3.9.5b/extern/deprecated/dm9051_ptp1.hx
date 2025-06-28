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
    #pragma message("dm9051: PTP (H/W ONE STEP)")
#endif
#if defined(DMPLUG_PTP_TWO_STEP) && defined(MAIN_DATA)
    #pragma message("dm9051: PTP (H/W TWO STEP)")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
    #pragma message("dm9051: PTP (H/W PPS)")
#endif

/* ptp, clkout, 2step
 */
#if defined(DMPLUG_PTP)
    #undef INFO_PTP
    #define INFO_PTP(dev, db) USER_CONFIG(dev, db, "dm9051: PTP (H/W ONE STEP)")
	
	#if defined(DMPLUG_PTP_TWO_STEP)
    #undef INFO_PTP2S
    #define INFO_PTP2S(dev, db) USER_CONFIG(dev, db, "dm9051: PTP (H/W TWO STEP)")
	#endif
	#if defined(DMPLUG_PPS_CLKOUT)
    #undef INFO_PPS
    #define INFO_PPS(dev, db) USER_CONFIG(dev, db, "dm9051: PTP (H/W PPS)")
	#endif
#endif

/* ptp implementation
 */

/* ptp */
int  dm9051_get_clk_ts(struct board_info *db);
void on_core_init_ptp_rate(struct board_info *db);

void ptp_ver(struct board_info *db);
void ptp_operation_extern(struct board_info *db);
void ptp_checksum_limit(struct board_info *db, struct net_device *ndev);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
u8   ptp_status_bits(struct board_info *db);
void dm9051_ptp_rxc_from_master(struct board_info *db);
int  dm9051_read_ptp_tstamp_mem(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb);
int  dm9051_ptp_tx_packet_monitor(struct board_info *db, struct sk_buff *skb);
int  dm9051_ptp_single_tx(struct board_info *db, struct sk_buff *skb);
netdev_features_t dm9051_ptp_fix_features(struct net_device *ndev, netdev_features_t features);

/* ptp casted, used in 'dm9051.c'
 */
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

/* PTP message type classification */
enum ptp_sync_type
{
    // PTP_NOT_PTP = 0,      /* Not a PTP packet or no timestamp involved */
    PTP_ONE_STEP = 1, /* One-step sync message */
    PTP_TWO_STEP = 2, /* Two-step sync message */
};

int is_ptp_rxts_en(struct board_info *db);
struct ptp_header *get_ptp_header(struct sk_buff *skb);
u8  get_ptp_message_type005(struct ptp_header *ptp_hdr);
struct ptp_header *dm9051_rx_ptp_hdr_monitor(struct board_info *db);

int is_ptp_announce_packet(u8 msgtype);
int is_ptp_sync_packet(u8 msgtype);
int is_ptp_delayreq_packet(u8 msgtype);
int is_ptp_delayresp_packet(u8 msgtype);
int is_peer_delayreq_packet(u8 msgtype);
int is_peer_delayresp_packet(u8 msgtype);

// typedef struct ptp_board_info {
// } ptp_board_info_t;

#endif //_DM9051_PTPC_H_
