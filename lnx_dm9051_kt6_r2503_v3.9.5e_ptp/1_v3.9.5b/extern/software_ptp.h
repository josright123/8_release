/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_SOFTWARE_PTP_H_
#define _DM9051_SOFTWARE_PTP_H_

/* pragma
 */
#if defined(DMPLUG_PTP_SW) && defined(MAIN_DATA)
    #pragma message("dm9051: PTP (S/W TWO STEP)")
#endif

#if defined(DMPLUG_PTP_SW)
    #undef INFO_PTP_SW_2S
    #define INFO_PTP_SW_2S(dev, db) USER_CONFIG(dev, db, "dm9051: PTP (S/W TWO STEP)")
#endif

/* ptp sw */
#if defined(DMPLUG_PTP_SW)
    /* re-direct ptp sw */
    #undef PTP_VER_SOFTWARE
    #define PTP_VER_SOFTWARE(b) ptp_ver_software(b) /* impl in dm9051_log.c */
    #undef DMPLUG_PTP_TX_TIMESTAMPING_SW
    #define DMPLUG_PTP_TX_TIMESTAMPING_SW(s) dm9051_ptp_tx_swtstamp(s)
#endif

#if defined(DMPLUG_PTP_SW) && defined(MAIN_DATA)
void ptp_ver_software(struct board_info *db)
{
    dev_info(&db->spidev->dev, "DMPLUG PTP Software Version\n");
}

void dm9051_ptp_tx_swtstamp(struct sk_buff *skb) // SKBTX_SW_TSTAMP (on 'dm9051_start_xmit')
{
    if (skb_shinfo(skb)->tx_flags & SKBTX_SW_TSTAMP)
    {
        skb_tx_timestamp(skb); // Add SW_TSTAMP
    }
}
#endif

#endif //_DM9051_SOFTWARE_PTP_H_
