/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _TEMPLATE_H_
#define _TEMPLATE_H_

/* -----------------
 * Template Block.
 * -----------------
 */

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

#define LEN_TX(b, s)          dm9051_tx_len(b, s)
#define PAD_TX(b, s)          // empty
#define MODE_TX(b, s)         dm9051_mode_tx(b, s) //~wd, i.e. bd (byte mode)

#undef SINGLE_TX //(also used in "dm9051_ptp_impl.h")
#define SINGLE_TX(b, s)             dm9051_single_tx(b, s)

#define DMPLUG_SHOW_ptp_rx_packet_monitor(b, s)
#define LOG_RX_PACKET_DUMP(b, s)

#endif //_TEMPLATE_H_
