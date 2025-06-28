/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_EXTERN_H_
#define _DM9051_EXTERN_H_

// #ifdef _DMPLUG_LOG .. #endif
/*#define DMCONF_BMCR_WR */ //(extern, bmcr-work around)
/*#define DMPLUG_MRR_WR */ //(extern, mrr-work around, when link change to up)

//#define PLUG_BMCR
#ifdef PLUG_BMCR
#define DMCONF_BMCR_WR //(extern, bmcr-work around)
#endif

//#define PLUG_MRR
#ifdef PLUG_MRR
#define DMPLUG_MRR_WR //(extern, mrr-work around)
#endif

/* pragma
 */
#if defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
#pragma message("EXTERN-WORKROUND: BMCR_WR")
#endif
#if defined(DMPLUG_MRR_WR) && defined(MAIN_DATA)
#pragma message("EXTERN-WORKROUND: MRR_WR")
#endif

/* USER_CONFIG, show for starting
 */
#if defined(DMCONF_BMCR_WR)
#undef INFO_BMCR_WR
#define INFO_BMCR_WR(dev, db)				USER_CONFIG(dev, db, "EXTERN-WORKROUND: BMCR_WR")
#endif

#if defined(DMPLUG_MRR_WR)
#undef INFO_MRR_WR
#define INFO_MRR_WR(dev, db) 				USER_CONFIG(dev, db, "EXTERN-WORKROUND: MRR_WR")
#endif

/* ECO, */
#define ECO //(Coerce)

/* re-direct bmsr_wr */
#if defined(ECO) && defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
#undef BMSR_OPERATION_CLEAR
#define BMSR_OPERATION_CLEAR(b) dm9051_bmsr_init(b)

#undef MDIO_PHY_READ
#define MDIO_PHY_READ(d, n, av) dm9051_phyread_nt_bmsr(d, n, av)
#endif

#if defined(ECO) && defined(DMPLUG_MRR_WR) && defined(MAIN_DATA)
#undef LINKCHG_UPSTART
#define LINKCHG_UPSTART(b) dm9051_all_upstart(b)
#endif

#if defined(ECO) && defined(DMCONF_BMCR_WR) && defined(MAIN_DATA)
int dm9051_phyread_nt_bmsr(struct board_info *db, unsigned int reg, unsigned int *val);
#endif

/* re-direct log */
#if defined(ECO) && defined(DMPLUG_LOG) && defined(MAIN_DATA)

#undef SHOW_BEGIN_LOG
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

#undef monitor_rxb0

#define SHOW_BEGIN_LOG(d,b) show_dev_begin(d,b)
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
#define DMPLUG_LOG_RXPTR(h,b) dm9051_log_rxptr(h,b) //#define dm9051_headlog_regs(h,b,r1,r2) 
#define DMPLUG_LOG_PHY(b) dm9051_log_phy(b) //#define dm9051_phyread_headlog(h,b,r) show_log_phy(h,b,r)

#define monitor_rxb0(b,rb) show_rxb(b,rb)

void show_dev_begin(struct device *dev, struct board_info *db);
void show_log(struct board_info *db);
void show_mode(struct device *dev);
void show_tcr_wr(struct board_info *db);

void show_pmode(struct device *dev);
void show_mac(struct board_info *db, u8 *addr);
void show_rxc(struct board_info *db, int scanrr);

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);

void dm9051_log_rxptr(char *head, struct board_info *db);
void dm9051_log_phy(struct board_info *db); //static int show_log_phy(char *head, struct board_info *db, unsigned int reg);

void show_rxb(struct board_info *db, unsigned int rxbyte);
#endif

#endif //_DM9051_EXTERN_H_
