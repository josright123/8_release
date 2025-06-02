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

/* ECO, */
#define ECO //(Coerce)

/* re-direct bmsr_wr */
#if defined(ECO) && defined(DMCONF_BMCR_WR) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef PHY_READ
#define PHY_READ(d, n, av) dm9051_phyread_nt_bmsr(d, n, av)
#endif

#if defined(ECO) && defined(DMCONF_MRR_WR) && defined(MAIN_DATA)
#undef LINKCHG_UPSTART
#define LINKCHG_UPSTART(b) dm9051_all_upstart(b)
#endif

#if defined(ECO) && defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
int dm9051_phyread_nt_bmsr(struct board_info *db, unsigned int reg, unsigned int *val);
#endif

/* re-direct log */
#if defined(ECO) && defined(DMPLUG_LOG) && (defined(SECOND_MAIN) || defined(MAIN_DATA))

#undef SHOW_DEVLOG_REFER_BEGIN
#undef SHOW_LOG_REFER_BEGIN
#undef SHOW_DEVLOG_MODE
#undef SHOW_DEVLOG_XMIT_THRD0
#undef SHOW_DEVLOG_XMIT_THRD
#undef SHOW_DEVLOG_XMIT_IN
#undef SHOW_DEVLOG_TCR_WR

#undef SHOW_PLAT_MODE
#undef SHOW_MAC
#undef SHOW_MONITOR_RXC

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);
#undef DMPLUG_LOG_RXPTR //#undef dm9051_headlog_regs
#undef DMPLUG_LOG_PHY //#undef dm9051_phyread_headlog

#undef dm9051_dump_data1
#undef monitor_rxb0

#define SHOW_DEVLOG_REFER_BEGIN(d,b) show_dev_begin(d,b)
#define SHOW_LOG_REFER_BEGIN(b) show_log(b)
#define SHOW_DEVLOG_MODE(d) show_mode(d)
#define SHOW_DEVLOG_XMIT_THRD0(b) show_xmit_thrd0(b)
#define SHOW_DEVLOG_XMIT_THRD(b) show_xmit_thrd(b)
#define SHOW_DEVLOG_XMIT_IN(b) show_xmit_in(b)
#define SHOW_DEVLOG_TCR_WR(b) show_tcr_wr(b)

#define SHOW_PLAT_MODE(d) show_pmode(d)
#define SHOW_MAC(b,a) show_mac(b,a)
#define SHOW_MONITOR_RXC(b,n) show_rxc(b,n)

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);
#define DMPLUG_LOG_RXPTR(h,b) dm9051_log_rxptr(h,b) //#define dm9051_headlog_regs(h,b,r1,r2) show_log_regs(h,b,r1,r2)
#define DMPLUG_LOG_PHY(b) dm9051_log_phy(b) //#define dm9051_phyread_headlog(h,b,r) show_log_phy(h,b,r)

#define dm9051_dump_data1(b,p,n) dump_data(b,p,n)
#define monitor_rxb0(b,rb) show_rxb(b,rb)

void show_dev_begin(struct device *dev, struct board_info *db);
void show_log(struct board_info *db);
void show_mode(struct device *dev);
void show_tcr_wr(struct board_info *db);

void show_pmode(struct device *dev);
void show_mac(struct board_info *db, u8 *addr);
void show_rxc(struct board_info *db, int scanrr);

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);

void dm9051_log_rxptr(char *head, struct board_info *db); //static void show_log_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2);
void dm9051_log_phy(struct board_info *db); //static int show_log_phy(char *head, struct board_info *db, unsigned int reg);

void dump_data(struct board_info *db, u8 *packet_data, int packet_len);
void show_rxb(struct board_info *db, unsigned int rxbyte);
#endif

#if defined(DMPLUG_LOG)
#undef INFO_LOG

#define INFO_LOG(dev, db)					USER_CONFIG(dev, db, "dm9051 LOG")
#endif
 
#endif //_DM9051_EXTERN_H_
