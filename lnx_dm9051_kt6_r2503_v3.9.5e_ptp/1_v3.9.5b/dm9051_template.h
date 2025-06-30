/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_TMPLTE_H_
#define _DM9051_TMPLTE_H_

/* -----------------
 * Main Block.
 * Macro domain
 * -----------------
 */

/*#define DMPLUG_INT */         //(INT39)
/*#define INT_CLKOUT */         //(INT39 ClkOut)
/*#define INT_TWO_STEP */       //(INT39 two_step)
/*#define DMPLUG_WD */          //(wd mode)
/*#define DMPLUG_SKB_PROTECT */ //(wd mode skb protect)
/*#define DMPLUG_MI_FIX */      //(driver config)
/*#define DMPLUG_PTP_SW */      //(ptp1588 software)

/* Macro for already known platforms
 */
#define PLUG_ENABLE_INT
#ifdef PLUG_ENABLE_INT
    #define DMPLUG_INT //(INT39)

    // #define PLUG_INT_CLKOUT
    #ifdef PLUG_INT_CLKOUT
        #define INT_CLKOUT //(INT39_CLKOUT)
    #endif

    // #define PLUG_INT_2STEP
    #ifdef PLUG_INT_2STEP
        #define INT_TWO_STEP //(INT39_TWO_STEP)
    #endif
#endif

// #define PLUG_ENABLE_WD
#ifdef PLUG_ENABLE_WD
    #define DMPLUG_WD //(wd mode)

    #define PLUG_SKB_PROTECT
    #ifdef PLUG_SKB_PROTECT
        #define DMPLUG_SKB_PROTECT // (skb protect)
    #endif
#endif

//[#define MI_FIX  1] //(driver config)
#define PLUG_MI_FIX
#ifdef PLUG_MI_FIX
    #define DMPLUG_MI_FIX //(driver config)
#endif                    //(driver config)

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
    #define DMPLUG_PTP_SW //(ptp S/W)
#endif                    //(ptp S/W)

/* ---------------------------
 * Extension Block.
 * Second Particular Functions
 * ---------------------------
 */

/*#define DMPLUG_DUMP_FUNC */    //(debug dump data)
/*#define DMPLUG_PTP */          //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */   //(ptp1588 pps)
/*#define DMPLUG_PTP_TWO_STEP */ //(ptp1588 two step)

#define PLUG_LOG
#ifdef PLUG_LOG
    #define DMPLUG_DUMP_FUNC //(extern, debug log, extra-print-log for detail observation!)
#endif

/* Capabilities:
 *        hardware-transmit
 *        hardware-receive
 *        hardware-raw-clock
 */
#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
    #define DMPLUG_PTP        //(ptp)

    #define PLUG_PTP_TWO_STEP //(always essential mandartory)(if not support, master NO follow up send)
    #ifdef PLUG_PTP_TWO_STEP
        #define DMPLUG_PTP_TWO_STEP //(HW Two step)
    #endif

    #define PLUG_PTP_PPS
    #ifdef PLUG_PTP_PPS
        #define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
    #endif
#endif //(ptp)

//.#define _MAIN_DATA
//.#include "dm9051.h"
//.#include "template.h" //= (Only for dm9051.c C source code)(fakes (castable functions))

//.#if defined(MAIN_DATA)
#include "template.h" //= (Only for dm9051.c C source code)(fakes (castable functions))
#include "dm9051_main_data.h" /* main_data */

//#if defined(_MAIN_DATA)
//  ...
//#endif //_MAIN_DATA

/* log: Put here after all included header files
 *      So conditional USER_CONFIG strings could be exactly correct
 */
static inline int SHOW_ALL_USER_CONFIG(char *head, struct device *dev, struct board_info *db)
{
    db->ucfg_count = 0;
    printk("\n");
    netif_warn(db, drv, db->ndev, "%s", head);
    INFO_INT(dev, db);
    INFO_INT_CLKOUT(dev, db);
    INFO_INT_TWOSTEP(dev, db);
    INFO_WD(dev, db);
    INFO_SKB_PROT(dev, db);
    INFO_PTP_SW_2S(dev, db);
    INFO_PTP(dev, db);
    INFO_PTP2S(dev, db);
    INFO_PPS(dev, db);
    INFO_MI_FIX(dev, db);
    INFO_DUMP_FUNC(dev, db);
    INFO_DUMP_RX_CNT(dev, db); // msg_enable
    INFO_BMCR_WR(dev, db);
    INFO_MRR_WR(dev, db);
    INFO_BUSWORK(dev, db);
    INFO_CONTI(dev, db);
    INFO_LPBK_TST(dev, db);
    INFO_CPU_BITS(dev, db);
    INFO_CPU_MIS_CONF(dev, db);
    INFO_KERNEL_VER(dev, db);
    INFO_MSG_ENABLE(dev, db); // msg_enable
    return db->ucfg_count;
}

static inline int SHOW_MAP_CHIPID(struct device *dev, unsigned short wid)
{
    if (wid != DM9051_ID)
    {
        dev_err(dev, "chipid error as %04x !\n", wid);
        return -ENODEV;
    }

    dev_warn(dev, "probe %04x found\n", wid);
    return 0;
}

//"So does NOT (all_start(open))"
//"So does NOT (all_restart(err_fnd))"
//"dm9051.on.(all_upstart(link_chg))"
static inline void show_core_reset(struct board_info *db)
{
    netif_crit(db, hw, db->ndev, "dm9051.on.(all_start(open)) [or]");
    netif_crit(db, hw, db->ndev, "dm9051.on.(all_restart(err_fnd))");
}

static inline void show_all_upfcr(struct board_info *db)
{
    netif_crit(db, link, db->ndev, "all_upfcr != core_reset (~DMPLUG_MRR_WR)");
    netif_crit(db, link, db->ndev, "So does NOT (all_upstart(link_chg))");
}

static inline void SHOW_OPEN(struct board_info *db)
{
    SHOW_ALL_USER_CONFIG("dm9051_open", NULL, db);
    /* amdix_log_reset(db); */ //(to be determined)
}

static inline void SHOW_RESTART_SHOW_STATIISTIC(struct board_info *db)
{
    netif_warn(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d, RST_up %d\n", //'netif_crit'
               db->bc.status_err_counter + db->bc.large_err_counter, db->bc.fifo_rst_counter, db->bc.up_rst_counter);
}

static inline void SHOW_XMIT_ANALYSIS(struct board_info *db)
{
    printk("\n");
    netif_info(db, tx_done, db->ndev, "%6d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in, db->xmit_in, db->xmit_tc,
               db->xmit_zc);
    netif_info(db, tx_done, db->ndev, "%6d [_THrd-in] on-THrd-in %u Pkt %u\n", db->xmit_thrd0, db->xmit_thrd0,
               db->xmit_ttc0);
    netif_info(db, tx_done, db->ndev, "%6d [_THrd-end] on-THrd-end %u Pkt %u\n", db->xmit_thrd, db->xmit_thrd,
               db->xmit_ttc);
}

static inline void dm9051_log_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2) //.show_log_regs
{
    unsigned int v1, v2;

    memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
    snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, head);
    dm9051_get_reg(db, reg1, &v1);
    dm9051_get_reg(db, reg2, &v2);
    netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, reg1, v1, reg2,
               v2);
}

static inline void SHOW_RX_CTRLS(struct board_info *db)
{
    dm9051_log_regs("dump rcr registers:", db, DM9051_RCR, DM9051_RCR);
    dm9051_log_regs("dump wdr registers:", db, 0x24, 0x25);
    dm9051_log_regs("dump mrr registers:", db, DM9051_MRRL, DM9051_MRRH);

    //.dm9051_headlog_regs("dump rcr registers:", db, DM9051_RCR, DM9051_RCR);
    //.dm9051_headlog_regs("dump wdr registers:", db, 0x24, 0x25);
    //.dm9051_headlog_regs("dump mrr registers:", db, DM9051_MRRL, DM9051_MRRH);

    //	unsigned int v1, v2;
    //	//memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
    //	dm9051_get_reg(db, DM9051_RCR, &v1);
    //	dm9051_get_reg(db, DM9051_RCR, &v2); //reg1 = DM9051_RCR; reg2 = DM9051_RCR;
    //	//snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump rcr registers:");
    //	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, DM9051_RCR,
    //v1, DM9051_RCR, v2); 	dm9051_get_reg(db, 0x24, &v1); 	dm9051_get_reg(db, 0x25, &v2); //reg1 = 0x24; reg2 = 0x25;
    //	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump wdr registers:");
    //	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, 0x24, v1,
    //0x25, v2); 	dm9051_get_reg(db, DM9051_MRRL, &v1); 	dm9051_get_reg(db, DM9051_MRRH, &v2); //reg1 = DM9051_MRRL; reg2
    //= DM9051_MRRH; 	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump mrr registers:"); 	netif_info(db, rx_status,
    //db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, DM9051_MRRL, v1, DM9051_MRRH, v2);
}

static inline void SHOW_ETH_MAC(struct board_info *db)
{
    struct net_device *ndev = db->ndev;

    printk("\n");
    netif_warn(db, hw, db->ndev, "MAC %02x %02x %02x %02x %02x %02x", ndev->dev_addr[0], ndev->dev_addr[1],
               ndev->dev_addr[2], ndev->dev_addr[3], ndev->dev_addr[4], ndev->dev_addr[5]);
}

static inline unsigned int SHOW_BMSR(struct board_info *db)
{
    unsigned int val;

    INTERN_PHY_READ(db, MII_BMSR, &val); /*.dm9051_phyread_headlog("bmsr", db, MII_BMSR);*/
    netif_warn(db, link, db->ndev, "bmsr %04x\n", val);
    return val;
}

static inline void SHOW_ETH_BMSR(struct board_info *db)
{
    db->st_bmsr1 = SHOW_BMSR(db);
    db->st_bmsr2 = SHOW_BMSR(db);
}
//.#endif //_MAIN_DATA

/* template for casting 
 * template for coercing
 */

#if defined(DMPLUG_PTP)
    #include "extern/dm9051_ptp_impl.h" /* #include "extern/dm9051_ptp1.h" */ /* ptp */
#endif

#if defined(DMPLUG_DUMP_FUNC)
	#include "extern/dump.h" /* log */
#endif

#if defined(DMPLUG_PTP_SW)
	#include "extern/software_ptp.h" /* log */
#endif

/* Extended support header files
 * #include "extern/extern.h" // extern
 * #include "plugs/plugs.h" // plug
 */

#endif //_DM9051_TMPLTE_H_
