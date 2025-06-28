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

/* main fakes (castable functions)
 */
#include "template.h" //=
#define INFO_CPU_BITS(dev, db)     USER_CONFIG(dev, db, "platform: __aarch64__")
#define INFO_KERNEL_VER(dev, db)   USER_CONFIG(dev, db, "Linux: " UTS_RELEASE)
#define INFO_INT(dev, db)          USER_CONFIG(dev, db, "dm9051: POL")
#define INFO_WD(dev, db)           USER_CONFIG(dev, db, "dm9051: BD")
#define INFO_MSG_ENABLE(dev, db)   MACRO_MSG_CONFIG(dev, db)
#define INFO_CPU_MIS_CONF(dev, db) // silence conditionally
#define INFO_INT_CLKOUT(dev, db)
#define INFO_INT_TWOSTEP(dev, db)
#define INFO_SKB_PROT(dev, db)
#define INFO_MI_FIX(dev, db)
#define INFO_DUMP_FUNC(dev, db)
#define INFO_DUMP_RX_CNT(dev, db)
#define INFO_BMCR_WR(dev, db)
#define INFO_MRR_WR(dev, db)
#define INFO_BUSWORK(dev, db)
#define INFO_CONTI(dev, db)
#define INFO_LPBK_TST(dev, db)
#define INFO_PTP(dev, db)
#define INFO_PPS(dev, db)
#define INFO_PTP2S(dev, db)
#define INFO_PTP_SW_2S(dev, db)
#define dm9051_dump_data1(b, p, l)
/* int fakes */
#define DM9051_STOP_FREEIRQ(b)    // empty
#define DM9051_STOP_CANCELDLY2(b) // empty
#define DM9051_PROBE_DLYSETUP(b)  // empty
/* fake clkout */
#define INT_SET_CLKOUT(db)        0 // empty(NoError)
/* poll fakes */
enum dm_req_not_support
{
    VOID_REQUEST_FUNCTION  = -9,
    NOT_REQUEST_SUPPORTTED = 0,
};
enum dm_req_support
{
    REQUEST_SUPPORTTED = 1,
};
#define dm9051_int2_supp()    NOT_REQUEST_SUPPORTTED
#define dm9051_int2_irq(d, h) VOID_REQUEST_FUNCTION
#define dm9051_poll_supp()    NOT_REQUEST_SUPPORTTED
#define dm9051_poll_sch(d)    VOID_REQUEST_FUNCTION
/* wd fakes */
#define BOUND_CONF_BIT        MBNDRY_BYTE
#define PAD_LEN(len)          len
#define PAD_TX(b, s)          // empty
#define CHG_SKB_TX(b, s)      // empty
/* mi fix fakes */
#define MI_MUTEX_LOCK(b)      // empty
#define MI_MUTEX_UNLOCK(b)    // empty
/* fakes (ptp sw) */
#define PTP_VER_SOFTWARE(b)   // empty (impl in dm9051_log.c)
#define DMPLUG_PTP_TX_TIMESTAMPING_SW(s)
/* final global fakes (ptp) */
/* In struct board_info; */
#define INIT_RCR(b)           b->rctl.rcr_all = (RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN)
/* fakes and ptp sw */
#define PTP_ETHTOOL_INFO(s)
#define PTP_NETDEV_IOCTL(s)
/* fakes(default) raw tx mode */
#define LEN_TX(b, s)          dm9051_tx_len(b, s)
#define MODE_TX(b, s)         dm9051_mode_tx(b, s) //~wd, i.e. bd (byte mode)
/* fakes dm9051_log */
#define SHOW_DEVLOG_TCR_WR(b)

/* ptp raw fake (used in 'dm9051.c')
 */
#define PTP_VER(b)
#define PTP_SETUP(b)                b->pbi.ptp_enable = 0 // dm9051_operation_clear_extern(b)
#define PTP_CHECKSUM_LIMIT(b, nd)
// #define PTP_NEW(d)				0
#define PTP_INIT(d)
#define PTP_END(d)
#define PTP_STATUS_BITS(b)          RSR_ERR_BITS
#define PTP_CONSTRAIN(n, f)         f
#define PTP_AT_RATE(b)

int dm9051_eth_ioctl(struct net_device *ndev, struct ifreq *rq,
                     int cmd); /* implement in "extern/dm9051_ptp1.c", "dm9051.c" */

/* ptp2 */
#define DMPLUG_RX_TS_MEM(b)         0
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b)
#define DMPLUG_RX_HW_TS_SKB(b, s)
#define SINGLE_TX(b, s)             dm9051_single_tx(b, s)

#define DMPLUG_SHOW_ptp_rx_packet_monitor(b, s)
#define LOG_RX_PACKET_DUMP(b, s)

//#include "dm9051.h"
//#define MAIN_DATA

#if defined(DMPLUG_DUMP_FUNC)
	#include "extern/dump.h" /* log */
#endif
#if defined(DMPLUG_PTP)
    #include "extern/dm9051_ptp_impl.h" /* #include "extern/dm9051_ptp1.h" */ /* ptp */
#endif
/* Extended support header files
 * #include "extern/extern.h" // extern
 * #include "plugs/plugs.h" // plug
 */
#include "dm9051_main_data.h" /* main_data */

#if defined(MAIN_DATA)
const struct plat_cnf_info *plat_cnf = &plat_misc_mode; //'&plat_align_mode'; /* Driver configuration */

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

static int SHOW_MAP_CHIPID(struct device *dev, unsigned short wid)
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
static void show_core_reset(struct board_info *db)
{
    netif_crit(db, hw, db->ndev, "dm9051.on.(all_start(open)) [or]");
    netif_crit(db, hw, db->ndev, "dm9051.on.(all_restart(err_fnd))");
}

static void show_all_upfcr(struct board_info *db)
{
    netif_crit(db, link, db->ndev, "all_upfcr != core_reset (~DMPLUG_MRR_WR)");
    netif_crit(db, link, db->ndev, "So does NOT (all_upstart(link_chg))");
}

static void SHOW_OPEN(struct board_info *db)
{
    SHOW_ALL_USER_CONFIG("dm9051_open", NULL, db);
    /* amdix_log_reset(db); */ //(to be determined)
}

static void SHOW_PLAT_CONF(struct board_info *db)
{
    netif_crit(db, hw, db->ndev, "plat_cnf->test_device: %s", plat_cnf->test_device);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.mode: %s", plat_cnf->align.mode);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.txsize: %d", plat_cnf->align.tx_blk);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.rxsize: %d", plat_cnf->align.rx_blk);
    netif_crit(db, hw, db->ndev, "plat_cnf->checksuming: %d", plat_cnf->checksuming);
}

static void SHOW_RESTART_SHOW_STATIISTIC(struct board_info *db)
{
    netif_warn(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d, RST_up %d\n", //'netif_crit'
               db->bc.status_err_counter + db->bc.large_err_counter, db->bc.fifo_rst_counter, db->bc.up_rst_counter);
}

static void SHOW_XMIT_ANALYSIS(struct board_info *db)
{
    printk("\n");
    netif_info(db, tx_done, db->ndev, "%6d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in, db->xmit_in, db->xmit_tc,
               db->xmit_zc);
    netif_info(db, tx_done, db->ndev, "%6d [_THrd-in] on-THrd-in %u Pkt %u\n", db->xmit_thrd0, db->xmit_thrd0,
               db->xmit_ttc0);
    netif_info(db, tx_done, db->ndev, "%6d [_THrd-end] on-THrd-end %u Pkt %u\n", db->xmit_thrd, db->xmit_thrd,
               db->xmit_ttc);
}

void dm9051_log_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2) //.show_log_regs
{
    unsigned int v1, v2;

    memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
    snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, head);
    dm9051_get_reg(db, reg1, &v1);
    dm9051_get_reg(db, reg2, &v2);
    netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, reg1, v1, reg2,
               v2);
}

static void SHOW_RX_CTRLS(struct board_info *db)
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

void SHOW_ETH_MAC(struct board_info *db)
{
    struct net_device *ndev = db->ndev;

    printk("\n");
    netif_warn(db, hw, db->ndev, "MAC %02x %02x %02x %02x %02x %02x", ndev->dev_addr[0], ndev->dev_addr[1],
               ndev->dev_addr[2], ndev->dev_addr[3], ndev->dev_addr[4], ndev->dev_addr[5]);
}

unsigned int SHOW_BMSR(struct board_info *db)
{
    unsigned int val;

    INTERN_PHY_READ(db, MII_BMSR, &val); /*.dm9051_phyread_headlog("bmsr", db, MII_BMSR);*/
    netif_warn(db, link, db->ndev, "bmsr %04x\n", val);
    return val;
}

void SHOW_ETH_BMSR(struct board_info *db)
{
    db->st_bmsr1 = SHOW_BMSR(db);
    db->st_bmsr2 = SHOW_BMSR(db);
}
#endif //MAIN_DATA

#endif //_DM9051_TMPLTE_H_
