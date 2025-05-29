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

#if defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
#pragma message("WORKROUND: BMCR_WR")
#endif
#if defined(DMCONF_MRR_WR) && defined(MAIN_DATA)
#pragma message("WORKROUND: MRR_WR")
#endif

#if defined(DMPLUG_LOG) && defined(MAIN_DATA)
#pragma message("DEBUG: LOG")
#endif
#endif //_DM9051_EXTERN_H_
