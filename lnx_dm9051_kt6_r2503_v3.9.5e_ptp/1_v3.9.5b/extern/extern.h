/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_EXTERN_H_
#define _DM9051_EXTERN_H_
/*#define DMCONF_BMCR_WR */ //(bmcr-work around)
/*#define DMCONF_MRR_WR */ //(mrr-work around, when link change to up)
/*#define DMPLUG_LOG */ //(debug log)

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

/* ptp */
#if defined(DMPLUG_PTP) /*&& defined(MAIN_DATA) && defined(CO1)*/
/* re-direct ptpc */
#undef INFO_PTP

#undef PTP_VER
#undef PTP_NEW
#undef PTP_INIT_RCR
#undef PTP_INIT
#undef PTP_END
#undef PTP_ETHTOOL_INFO
#undef PTP_NETDEV_IOCTL
#undef PTP_STATUS_BITS
#undef PTP_AT_RATE

#define INFO_PTP(dev, db)	USER_CONFIG(dev, db, "dm9051 PTP")

#define PTP_VER(b)		ptp_ver(b)
#define PTP_NEW(d) 				ptp_new(d)
#define PTP_INIT_RCR(d) 		ptp_init_rcr(d)
#define PTP_INIT(d) 			ptp_init(d)
#define PTP_END(d) 				ptp_end(d)
#define PTP_ETHTOOL_INFO(s)		s = dm9051_ts_info,
#define PTP_STATUS_BITS(b)			ptp_status_bits(db)
#define PTP_NETDEV_IOCTL(s)	s = dm9051_ptp_netdev_ioctl,
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

#define DMPLUG_PTP_TX_IN_PROGRESS(s)	dm9051_ptp_tx_in_progress(s)
#define DMPLUG_PTP_TX_PRE(b,s)	dm9051_ptp_txreq(b,s)
#define DMPLUG_TX_EMIT_TS(b,s)	dm9051_ptp_txreq_hwtstamp(b,s)
#endif

/* ptp clkout */
#if defined(DMPLUG_PPS_CLKOUT)
#undef INFO_PPS

#define INFO_PPS(dev, db)					USER_CONFIG(dev, db, "dm9051 PPS")
#endif

#if defined(DMPLUG_LOG)
#undef INFO_LOG

#define INFO_LOG(dev, db)					USER_CONFIG(dev, db, "dm9051 LOG")
#endif
 
#endif //_DM9051_EXTERN_H_
