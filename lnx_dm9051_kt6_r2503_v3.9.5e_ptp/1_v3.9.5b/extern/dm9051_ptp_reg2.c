// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
//#include <stdint.h>
//#include <inttypes.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/utsname.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/ipv6.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>

#include "../dm9051.h"
#include "dm9051_ptp1.h" /* 0.1 ptpc */

#if 0
//u8 *gpacket_data;
//int gpacket_len;

extern int slave_get_ptpFrame;

static u64 rx_extract_ts(u8 *rxTSbyte)
{
	//u8 temp[12];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	//u32 prttsyn_stat, hi, lo,
	u32 sec;
	u64 ns;

#if 0
	printk(" REAL RX TSTAMP hwtstamp= %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
	       rxTSbyte[0], rxTSbyte[1], rxTSbyte[2], rxTSbyte[3], rxTSbyte[4], rxTSbyte[5], rxTSbyte[6], rxTSbyte[7]);
#endif

	//dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x02); //Read RX Time Stamp Clock Register offset 0x62, value 0x02

	ns_lo = rxTSbyte[7] | (rxTSbyte[6] << 8);
	ns_hi = rxTSbyte[5] | (rxTSbyte[4] << 8);

	s_lo = rxTSbyte[3] | (rxTSbyte[2] << 8);
	s_hi = rxTSbyte[1] | (rxTSbyte[0] << 8);

	sec = s_lo;
	sec |= s_hi << 16;

	ns = ns_lo;
	ns |= ns_hi  << 16;
	
	//.printk("Slave(%d)-DM9051A ...extract_ts  %llu s, %llu ns\n", slave_get_ptpFrame, sec, ns);

	ns += ((u64)sec) * 1000000000ULL;
	//printk("_dm9051_ptp_rx_hwtstamp ns_lo=%x, ns_hi=%x s_lo=%x s_hi=%x \r\n", ns_lo, ns_hi, s_lo, s_hi);
	return ns;
}
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	ptp_board_info_t *pbi = &db->pbi;

#if 0
	/* Use: enum hwtstamp_rx_filters
	 */
	/* Even S/W TSTAMP, on (H/W TSTAMP) do , will be not hurt !!
	 */
	if (pbi->tstamp_config.rx_filter &
	    (HWTSTAMP_FILTER_PTP_V2_EVENT | HWTSTAMP_FILTER_ALL)) {
		.................... //can do
	}
#endif
#if 0
	/* Even S/W TSTAMP, do shhwtstamps->hwtstamp = ns_to_ktime(ns); (H/W TSTAMP) will be not hurt !!
	 */
	if (!pbi->tstamp_config.rx_filter) //[wait further test..]
		return;
#endif

#if 1 //[wait further test..]
	//[Now]
	if (is_ptp_rxts_en(db)) //if T1/T4, // Is it inserted Timestamp? //[wait further test..]
#endif
	{
		//So when NOT T1/T4, we can skip tell tstamp (just an empty (virtual) one)

#if 0
			= original.dm9051_ptp_rx_hwtstamp(db, skb); //_15888_,
#endif
		if (dm9051_rx_ptp_hdr_monitor(db)) {
			/* following, with netif_rx(skb),
			 * slave4l can parse the T1 and/or T4 rx tstamp from master
			 */
			if (pbi->ptp_on) { //NOT by db->ptp-enable
				//printk("==> dm9051_ptp_rx_hwtstamp in\r\n");
				/* Since we cannot turn off the Rx timestamp logic if the device is
				 * doing Tx timestamping, check if Rx timestamping is configured.
				 */
				u64 ns;

				if (slave_get_ptpFrame || is_peer_delayreq_packet(pbi->ptp_rx_msgtype)) {
					u8 *rxTSbyte = pbi->rxTSbyte;
					u16 ns_hi, ns_lo, s_hi, s_lo;
					u32 sec;
					//u64 ns;

					ns_lo = rxTSbyte[7] | (rxTSbyte[6] << 8); //pbi->rxTSbyte[7] | (pbi->rxTSbyte[6] << 8);
					ns_hi = rxTSbyte[5] | (rxTSbyte[4] << 8); //pbi->rxTSbyte[5] | (pbi->rxTSbyte[4] << 8);

					s_lo = rxTSbyte[3] | (rxTSbyte[2] << 8); //pbi->rxTSbyte[3] | (pbi->rxTSbyte[2] << 8);
					s_hi = rxTSbyte[1] | (rxTSbyte[0] << 8); //pbi->rxTSbyte[1] | (pbi->rxTSbyte[0] << 8);

					sec = s_lo;
					sec |= s_hi << 16;

					ns = ns_lo;
					ns |= ns_hi  << 16;

					#if 0 //chk OK
					if (is_peer_delayreq_packet(pbi->ptp_rx_msgtype)) 
						printk("Peer get-pdly_Req.sec.ns: (frame %d) ts bytes %d: %u sec\n", 
							pbi->total_ptp_frames, pbi->ptp_ts_bytes, sec);
					#endif
					if (slave_get_ptpFrame) {
						//printk("Slave(%d)-DM9051A ...ptp_rxts_en  %llu s, %" PRIu64 " ns\n", sec, ns);
						//printk("Slave(%d)-DM9051A ...ptp_rxts_en  %llu s, %llu ns\n", slave_get_ptpFrame, sec, ns);
						if (is_ptp_sync_packet(pbi->ptp_rx_msgtype)) {
							printk("Slave %u s\n", sec);

							/* Slave test when if no master emit sync */
							//if (1) {
							//	dump_data(db, gpacket_data, gpacket_len); //dump_data(), used directly.
							//}

						}
						else if (pbi->ptp_rx_msgtype == PTP_MSGTYPE_PDELAY_REQ_pri ||
								pbi->ptp_rx_msgtype == PTP_MSGTYPE_PDELAY_RESP_pri ||
								pbi->ptp_rx_msgtype == PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri)
							; //skip
						else
							printk("Slave(!) rx msgtype %u, %u s\n", pbi->ptp_rx_msgtype, sec);
						slave_get_ptpFrame--;
					}
				}

				ns = rx_extract_ts(pbi->rxTSbyte);
				/* Use skb_hwtstamps(skb) get 'skb_shared_hwtstamps' and then copy to ->hwtstamp
				 * We can also use skb_complete_rx_timestamp() to make the same result.
				 */
				do {
					struct skb_shared_hwtstamps *shhwtstamps =
						skb_hwtstamps(skb); //for pass T2 the HW rx tstamp
					memset(shhwtstamps, 0, sizeof(*shhwtstamps));
					shhwtstamps->hwtstamp = ns_to_ktime(ns);
				} while (0);

				//printk("Report RX Timestamp to skb = %lld\n", shhwtstamps->hwtstamp);
				//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x08); //Clear RX Time Stamp Clock Register offset 0x60, value 0x08
				//printk("<== dm9051_ptp_rx_hwtstamp out\r\n");
			}
		} //dm9051_rx_ptp_hdr_monitor
	}
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp reg2"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
