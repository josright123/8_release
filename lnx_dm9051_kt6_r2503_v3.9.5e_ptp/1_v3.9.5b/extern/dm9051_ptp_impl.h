/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PTP_IMPL_H_
#define _DM9051_PTP_IMPL_H_

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

/* ptp
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

/* fakes dm9051_log */
	#define SHOW_DEVLOG_TCR_WR(b)

/* ptp casted, used in 'dm9051.c'
 */
//#if defined(DMPLUG_PTP) /*&& defined(CO1) */
//#endif
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
	#define PTP_CONSTRAIN(n, f) 		dm9051_ptp_constrain_features(n, f)
	#undef DMPLUG_RX_TS_MEM
	#undef DMPLUG_RX_HW_TS_SKB
	#define DMPLUG_RX_TS_MEM(b)       dm9051_read_ptp_tstamp_mem(b)
	#define DMPLUG_RX_HW_TS_SKB(b, s) dm9051_ptp_rx_hwtstamp(b, s)
	#undef DMPLUG_SHOW_ptp_rx_packet_monitor
	#define DMPLUG_SHOW_ptp_rx_packet_monitor(b, s) dm9051_ptp_rx_packet_monitor(b, s)
	#undef DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER
	#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b) dm9051_ptp_rxc_from_master(b)

	#undef LEN_TX
	#define LEN_TX(b, s)          dm9051_tx_len(b, s)
	#undef PAD_TX
	#define PAD_TX(b, s)          // empty
	#undef MODE_TX
	#define MODE_TX(b, s)         dm9051_mode_tx(b, s) //~wd, i.e. bd (byte mode)
	#undef SINGLE_TX
	#define SINGLE_TX(b, s) dm9051_ptp_single_tx(b, s)
	// #undef DMPLUG_PTP_TX_IN_PROGRESS
	// #undef DMPLUG_PTP_TX_PRE
	// #undef DMPLUG_TX_EMIT_TS
	// #define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	dm9051_ptp_tx_in_progress(b,s)
	// #define DMPLUG_PTP_TX_PRE(b,s)			dm9051_ptp_tcr_2wr(b,s)
	// #define DMPLUG_TX_EMIT_TS(b,s)			dm9051_ptp_txreq_hwtstamp(b,s)

/* ~(ptp sw ||) final global ptp */
//#if defined(DMPLUG_PTP) /* || defined(_DMPLUG_PTP_SW)*/
//#endif
	#undef INIT_RCR
	#define INIT_RCR(b)           	  b->rctl.rcr_all = (RCR_ALL | RCR_DIS_LONG | RCR_RXEN) //ptp_init_rcr(d)

#endif //_DM9051_PTP_IMPL_H_
