/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_EXTERN_H_
#define _DM9051_EXTERN_H_
/*#define DMPLUG_PTP */ //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */ //(ptp1588 pps)
/*#define DMCONF_BMCR_WR */ //(bmcr-work around)
/*#define DMCONF_MRR_WR */ //(mrr-work around, when link change to up)
/*#define DMPLUG_LOG */ //(debug log)

#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)

  #define PLUG_PTP_PPS
  #ifdef PLUG_PTP_PPS
  #define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
  #endif
#endif

/* pragma
 */
#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
//#warning "dm9051 PTP"
#pragma message("dm9051 PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
//#warning "dm9051 PPS"
#pragma message("dm9051 PPS")
#endif

//#define PLUG_BMCR
#ifdef PLUG_BMCR
#define DMCONF_BMCR_WR //(bmcr-work around)
#endif

//#define PLUG_MRR
#ifdef PLUG_MRR
#define DMCONF_MRR_WR //(mrr-work around)
#endif

//#define PLUG_LOG
#ifdef PLUG_LOG
#define DMPLUG_LOG //(debug log, extra-print-log for detail observation!)
#endif

/* pragma
 */
#if defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
#pragma message("WORKROUND: BMCR_WR")
#endif
#if defined(DMCONF_MRR_WR) && defined(MAIN_DATA)
#pragma message("WORKROUND: MRR_WR")
#endif

#if defined(DMPLUG_LOG) && defined(MAIN_DATA)
#pragma message("DEBUG: LOG")
#endif

/* functions re-construct
 */
/* CO1, */
/*#define CO1*/ //(Coerce)
#if defined(DMPLUG_PTP) /*&& defined(MAIN_DATA) && defined(CO1)*/
/* re-direct ptpc */
#undef PTP_VER
#define PTP_VER(b)		ptp_ver(b)
#undef PTP_NEW
#define PTP_NEW(d) 				ptp_new(d)
#undef PTP_INIT_RCR
#define PTP_INIT_RCR(d) 		ptp_init_rcr(d)
#undef PTP_INIT
#define PTP_INIT(d) 			ptp_init(d)
#undef PTP_END
#define PTP_END(d) 				ptp_end(d)
#undef PTP_ETHTOOL_INFO
#define PTP_ETHTOOL_INFO(s)		s = dm9051_ts_info,
#undef PTP_STATUS_BITS
#define PTP_STATUS_BITS(b)			ptp_status_bits(db)
#undef PTP_NETDEV_IOCTL
#define PTP_NETDEV_IOCTL(s)	s = dm9051_ptp_netdev_ioctl,
#undef PTP_AT_RATE
#define PTP_AT_RATE(b)	on_core_init_ptp_rate(b)

#undef DMPLUG_RX_TS_MEM
#define DMPLUG_RX_TS_MEM(b)		dm9051_read_ptp_tstamp_mem(b)
#undef DMPLUG_RX_HW_TS_SKB
#define DMPLUG_RX_HW_TS_SKB(b,s) dm9051_ptp_rx_hwtstamp(b,s)
#undef SHOW_ptp_rx_packet_monitor
#define SHOW_ptp_rx_packet_monitor(b,s) dm9051_ptp_rx_packet_monitor(b,s)
#undef DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b) \
		dm9051_ptp_rxc_from_master(b)

#undef DMPLUG_PTP_TX_IN_PROGRESS
#define DMPLUG_PTP_TX_IN_PROGRESS(s)	dm9051_ptp_tx_in_progress(s)
#undef DMPLUG_PTP_TX_PRE
#define DMPLUG_PTP_TX_PRE(b,s)	dm9051_ptp_txreq(b,s)
#undef DMPLUG_TX_EMIT_TS
#define DMPLUG_TX_EMIT_TS(b,s)	dm9051_ptp_txreq_hwtstamp(b,s)
#endif
 
#endif //_DM9051_EXTERN_H_
