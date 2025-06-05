// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

/*
 * User notice:
 *   To add log function,
 *   And add this file to project
 *   And modify Makefile to insert this file
 *   into the project build.
 *
 * In Makefile
 * Change
 *   dm9051a-objs := dm9051.o
 * to
 *   dm9051a-objs := dm9051.o dm9051_wd.o
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h> // For newer kernels
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>

// #define SECOND_MAIN //(sec)
#include "dm9051.h"
/*#include extern/extern.h */ //(extern/)
#include "extern/extern.h"
#include "extern/dm9051_ptp1.h" /* 0.1 ptpc */

/* Tx 'wb' do skb protect */
#define DM9051_SKB_PROTECT

// #ifdef DMPLUG_WD
// #endif

#ifdef DM9051_SKB_PROTECT
static struct sk_buff *EXPAND_SKB(struct sk_buff *skb, unsigned int pad)
{
    struct sk_buff *skb2;

    skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
    if (skb2)
    {
        dev_kfree_skb(skb);
        return skb2;
    }

    //.netif_warn(db, tx_queued, db->ndev, "[WB_SUPPORT] warn on len %d, skb_copy_expand get memory leak!\n", skb->len);
    return skb;
}
#endif

/* particulars, wb mode*/
struct sk_buff *dm9051_pad_txreq(struct board_info *db, struct sk_buff *skb)
{
    db->data_len = skb->len;

    db->pad = (skb->len & 1) ? 1 : 0; // db->pad = (plat_cnf->skb_wb_mode && (skb->len & 1)) ? 1 : 0; //'~wb'

#ifdef DM9051_SKB_PROTECT
    if (db->pad)
        skb = EXPAND_SKB(skb, db->pad);
#endif
    return skb;
}

MODULE_DESCRIPTION("Davicom DM9051 driver, wd-function");
MODULE_LICENSE("GPL");
