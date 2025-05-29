// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
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
//_15888_
//#include <linux/ptp_clock_kernel.h>
//#include <linux/ptp_classify.h>

//#include "dm9051_ptp1.h"
#include "../dm9051.h"
#include "dm9051_ptp1.h" /* 0.1 ptpc */
//#include "dm9051_ptpd.h"
#define DMCONF_DIV_HLPR_32 //(32-bit division helper, __aeabi_ldivmod())

#ifdef DMCONF_DIV_HLPR_32
/* Implement the missing ARM EABI division helper */
long long __aeabi_ldivmod(long long numerator, long long denominator)
{
    long long res = 0;
    long long d = denominator;
    int sign = 1;

    if (numerator < 0) {
        numerator = -numerator;
        sign = -sign;
    }

    if (d < 0) {
        d = -d;
        sign = -sign;
    }

    if (d != 0) {
        /* Use the kernel's division helper */
        res = div64_s64(numerator, d);
        if (sign < 0)
            res = -res;
    }

    return res;
}
#endif

int is_ptp_sync_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_SYNC) ? 1 : 0;
}
int is_ptp_delayreq_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_DELAY_REQ) ? 1 : 0;
}

static struct ptp_header *get_ptp_header(struct sk_buff *skb) {
    u8 *p = skb->data;
    struct ethhdr *eth = (struct ethhdr *)p;
    u8 *ptp_hdr;
    u16 proto;

    // Skip Ethernet header
    p += ETH_HLEN;
    proto = ntohs(eth->h_proto);

    // Check for Layer 2 PTP
    if (proto == PTP_ETHERTYPE) {
	return (struct ptp_header *) p;
        //ptp_hdr = p;
        //return ptp_hdr[0] & 0x0f;
    }

    // Handle IPv4
    if (proto == ETH_P_IP) {
        struct iphdr *ip = (struct iphdr *)p;
        if (ip->protocol == IPPROTO_UDP) {
            struct udphdr *udp = (struct udphdr *)(p + sizeof(struct iphdr));
            if (ntohs(udp->dest) == PTP_EVENT_PORT || ntohs(udp->dest) == PTP_GENERAL_PORT) {
                ptp_hdr = (u8 *)udp + sizeof(struct udphdr);
                return (struct ptp_header *) ptp_hdr;
		//return ptp_hdr[0] & 0x0f;
            }
        }
    }
    // Handle IPv6
    else if (proto == ETH_P_IPV6) {
        struct ipv6hdr *ip6 = (struct ipv6hdr *)p;
        if (ip6->nexthdr == IPPROTO_UDP) {
            struct udphdr *udp = (struct udphdr *)(p + sizeof(struct ipv6hdr));
            if (ntohs(udp->dest) == PTP_EVENT_PORT || ntohs(udp->dest) == PTP_GENERAL_PORT) {
                ptp_hdr = (u8 *)udp + sizeof(struct udphdr);
		return (struct ptp_header *) ptp_hdr;
                //return ptp_hdr[0] & 0x0f;
            }
        }
    }

    return NULL;
    //return 0; // Not a PTP packet
}

u8 get_ptp_message_type005(struct ptp_header *ptp_hdr) {
	//struct ptp_header *ptp_hdr = get_ptp_header(skb);

	//if (!ptp_hdr)
	//	return 0;

	//return ptp_hdr[0] & 0x0f;
	return ptp_hdr->tsmt & 0x0f;
}

void dm9051_ptp_txreq(struct board_info *db, struct sk_buff *skb)
{
	struct ptp_header *ptp_hdr;

	db->tcr_wr = TCR_TXREQ; // TCR register value

	ptp_hdr = get_ptp_header(skb);
	if (ptp_hdr) {
		//db->ptp_step = dm9051_ptp_step(db, skb);
		//if (db->ptp_step) {
			u8 message_type = get_ptp_message_type005(ptp_hdr); //for tx send
			//if (dm9051_ptp_frame(db, skb)) {
				//if (likely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
				if (is_ptp_sync_packet(message_type)) {
					db->ptp_step = (u8)(ptp_hdr->flag_field[0] & PTP_FLAG_TWOSTEP) ? PTP_TWO_STEP : PTP_ONE_STEP;
					if (db->ptp_step == 2) {
						db->tcr_wr = TCR_TS_EN | TCR_TXREQ;
					} else {
						db->tcr_wr = TCR_TS_EMIT | TCR_TXREQ;
					}
				} else if (is_ptp_delayreq_packet(message_type)) //_15888_,
					db->tcr_wr = TCR_TS_EN | TCR_TS_EMIT | TCR_TXREQ;
				//}
			//}
			//return message_type;
		//}
	}
}

void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
//	if ((is_ptp_sync_packet(message_type) &&
//		db->ptp_step == 2) ||
//		is_ptp_delayreq_packet(message_type)) { //_15888_,
	if (db->tcr_wr & TCR_TS_EN) {
		int ret;

		/* Poll for TX completion */
		ret = dm9051_nsr_poll(db);
		if (ret) {
			netdev_err(db->ndev, "ptp TX hwtstamp completion polling timeout\n");
			//.return ret; //.only can be less hurt
		}

		dm9051_ptp_tx_hwtstamp(db, skb); //dm9051_hwtstamp_to_skb(skb, db); //_15888_,
	}
//	}
}

void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb)
{
	struct ptp_header *ptp_hdr = get_ptp_header(skb);
	if (ptp_hdr) //is_ptp_packet(skb->data)
	{
		static int slave_get_ptpFrame = 9;
		static int slave_get_ptpFrameResp3 = 3;
		static int master_get_delayReq6 = 6; //5;
		static int slave_get_ptpMisc = 9;
		u8 message_type = get_ptp_message_type005(ptp_hdr); //for rx monitor
		
		if (is_ptp_sync_packet(message_type)) {
			if (slave_get_ptpFrame)
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("\n");
				printk("Slave(%d)-get-sync with tstamp. \n", --slave_get_ptpFrame);
				//sprintf(db->bc.head, "Slave-get-sync with tstamp, len= %3d", skb->len);
				//dm9051_dump_data1(db, skb->data, skb->len);
			} else {
				printk("Slave(%d)-get-sync without tstamp. \n", --slave_get_ptpFrame);
			}}
		} else
		if (message_type == PTP_MSGTYPE_FOLLOW_UP) {
			if (slave_get_ptpFrame)
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-followup with tstamp. \n", --slave_get_ptpFrame);
			} else {
				printk("Slave(%d)-get-followup without tstamp. \n", --slave_get_ptpFrame);
			}}
		} else
		if (message_type == PTP_MSGTYPE_DELAY_RESP) {
			if (slave_get_ptpFrameResp3)
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-DELAY_RESP with tstamp. \n", --slave_get_ptpFrameResp3);
			} else {
				printk("Slave(%d)-get-DELAY_RESP without tstamp. \n", --slave_get_ptpFrameResp3);
			}}
		} else
		if (message_type == PTP_MSGTYPE_ANNOUNCE) {
			if (slave_get_ptpFrame)
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-ANNOUNCE with tstamp. \n", --slave_get_ptpFrame);
			} else {
				printk("Slave(%d)-get-ANNOUNCE without tstamp. \n", --slave_get_ptpFrame);
			}}
		} else
		if (is_ptp_delayreq_packet(message_type)) {
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				if (master_get_delayReq6) {
					printk("Master(%d)-get-DELAY_REQ with tstamp. \n", --master_get_delayReq6);
				}
			} else {
				printk("Master-get-DELAY_REQ without tstamp.\n");
			}}
		} else
		{
			if (slave_get_ptpMisc)
			if (db->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d) or Master get-knonw with tstamp. \n", --slave_get_ptpMisc);
			} else {
				printk("Slave(%d) or Master get-knonw without tstamp. \n", --slave_get_ptpMisc);
			}}
		}
	}
}

/* ptpc - support functions-2 */
/*s64*/
u32 dm9051_get_rate_reg(struct board_info *db) {
	u8 mRate[4];
	u32 pre_rate;
	//_mutex_lock(&db->_spi_lockm);
	dm9051_set_reg(db, 0x69, 0x01);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	regmap_noinc_read(db->regmap_dm, 0x68, mRate, 4);
	pre_rate = ((uint32_t)mRate[3] << 24) | ((uint32_t)mRate[2] << 16) |
		((uint32_t)mRate[1] << 8) | (uint32_t)mRate[0];
	//_mutex_unlock(&db->_spi_lockm);
	//printk("Pre-RateReg value = 0x%08X\n", pre_rate);

	return pre_rate;
}

int dm9051_read_ptp_tstamp_mem(struct board_info *db, u8 *rxTSbyte)
{
	//_15888_
	//if (db->ptp_on) { //Even NOT ptp_on, need do.
	if (db->ptp_enable) {
	if (is_ptp_rxts_enable(db)) {	// Inserted Timestamp
		struct net_device *ndev = db->ndev;
		int ret;
		//printk("Had RX Timestamp... rxstatus = 0x%x\n", db->rxhdr.status);
		if(db->rxhdr.status & RSR_RXTS_LEN) {	// 8 bytes Timestamp
			ret = dm9051_read_mem(db, DM_SPI_MRCMD, rxTSbyte, 8);
			if (ret) {
				netdev_dbg(ndev, "Read TimeStamp error: %02x\n", ret);
				return ret;
			}
		}else{
			/* 4bytes, dm9051a NOT supported 
			 */
			ret = dm9051_read_mem(db, DM_SPI_MRCMD, rxTSbyte, 4);
			if (ret) {
				netdev_dbg(ndev, "Read TimeStamp error: %02x\n", ret);
				return ret;
			}
		}			
	}}
	//}
	return 0;
}

//static 
void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamps;
	//u64 regval;
	u8 temp[9];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	u32 sec;
	u64 ns;
	//int i;
	//unsigned int uIntTemp = 0;


	memset(&temp, 0, sizeof(temp));

	//printk("==================================> TX_hwtstamp FROM in\r\n");

	//Spenser - Read TX timestamp from REG_68H
	//remark6-slave

//.	_mutex_lock(&db->_spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// Reset Register 68H Index
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// dummy reset to workaround unsync
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01

	regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// for (i=0; i< 8; i++) {
	// 	regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
	// 	temp[i] = (u8)(uIntTemp & 0xFF);
	// }
//.	_mutex_unlock(&db->_spi_lockm);

#if 0
	if (!uIntTemp) {
		printk("HW Timestamp read fail\n");
	}else{
		printk("HW Timestamp read OK\n");
	}
#endif
#ifdef _DE_TIMESTAMP
	printk(" TXTXTXTXTX hwtstamp 0x68 = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \r\n",
	       temp[0], temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7]);
#endif
	//printk("==================================> TX_hwtstamp FROM OUT\r\n");
	ns_lo = temp[0] | (temp[1] << 8);
	ns_hi = temp[2] | (temp[3] << 8);

	s_lo = temp[4] | (temp[5] << 8);
	s_hi = temp[6] | (temp[7] << 8);

	sec = s_lo;
	sec |= s_hi << 16;

	ns = ns_lo;
	ns |= ns_hi  << 16;

#ifdef DE_TIMESTAMP
	//remark4-slave
	//printk(" TXTXTXTXTX hwtstamp sec = %x, ns = %x \r\n", sec, (u32)ns);

#endif


	ns += ((u64)sec) * 1000000000ULL;


	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ns_to_ktime(ns);
	//skb_complete_tx_timestamp(skb, &shhwtstamps);
	//remark5-slave
	//printk("---Report TX HW Timestamp\n");
	skb_tstamp_tx(skb, &shhwtstamps); //For report T3 HW tx tstamp
	//remark5-slave
	//printk("Report TX HW Timestamp---\n");

//Spenser - doesn't trigger GP1 this time.
#if 0
	if(skb->len == 86){
		if (skb->data[42] == 0x00){
			printk("++++++ master => Sync or Follow_up packet  (slave => Delay Reqest packet sequestid = %x %x )!!!!! +++++ \r\n", skb->data[73], skb->data[74]);
			printk(" TXTXTXTXTX hwtstamp 0x68 = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \r\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
			//dm9051_GP1_setup(db, temp);
			schedule_work(&db->ptp_extts0_work);
		}

	}
#endif
}

u64 rx_extract_ts(u8 *rxTSbyte)
{
	//u8 temp[12];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	//u32 prttsyn_stat, hi, lo,
	u32 sec;
	u64 ns;

	#if 0
	printk(" REAL RX TSTAMP hwtstamp= %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
	       rxTSbyte[0], rxTSbyte[1],rxTSbyte[2],rxTSbyte[3],rxTSbyte[4],rxTSbyte[5],rxTSbyte[6],rxTSbyte[7]);
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

	ns += ((u64)sec) * 1000000000ULL;
	//printk("dm9051_ptp_rx_hwtstamp ns_lo=%x, ns_hi=%x s_lo=%x s_hi=%x \r\n", ns_lo, ns_hi, s_lo, s_hi);
	return ns;
}

void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte)
{
	if(db->ptp_on) { //NOT by db->ptp-enable
		//printk("==> dm9051_ptp_rx_hwtstamp in\r\n");
		/* Since we cannot turn off the Rx timestamp logic if the device is
		 * doing Tx timestamping, check if Rx timestamping is configured.
		 */
		u64 ns = rx_extract_ts(rxTSbyte);
		do {
			struct skb_shared_hwtstamps *shhwtstamps =
				skb_hwtstamps(skb); //for pass T2 the HW rx tstamp
			memset(shhwtstamps, 0, sizeof(*shhwtstamps));
			shhwtstamps->hwtstamp = ns_to_ktime(ns);
		} while(0);

		//printk("Report RX Timestamp to skb = %lld\n", shhwtstamps->hwtstamp);
		//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x08); //Clear RX Time Stamp Clock Register offset 0x60, value 0x08
		//printk("<== dm9051_ptp_rx_hwtstamp out\r\n");
	}
}

static int lan_ptp_get_ts_ioctl(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *adb = netdev_priv(netdev);
	struct hwtstamp_config *config = &adb->tstamp_config;
        
	/* copy from db tstamp_config, to user */
	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;
}

static int lan743x_ptp_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct board_info *adb = netdev_priv(netdev);
	struct hwtstamp_config config;
	int ret = 0;

	if (!ifr) {
		netif_err(adb, hw, adb->ndev,
			  "SIOCSHWTSTAMP, ifr == NULL\n");
		return -EINVAL;
	}

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	if (config.flags) {
		netif_warn(adb, hw, adb->ndev,
			   "ignoring hwtstamp_config.flags == 0x%08X, expected 0\n",
			   config.flags);
	}

	switch (config.tx_type) {
		case HWTSTAMP_TX_OFF:
			//dev_info(&adb->spidev->dev, "IOCtl - Now db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, false)\n", adb->ptp_on);
			netif_info(adb, hw, adb->ndev, "IOCtl - Now db->ptp_on %d, NOTE: Stop tx sync !\n", adb->ptp_on);
			//lan743x_ptp_set_sync_ts_insert(adapter, false);
			break;
		case HWTSTAMP_TX_ONESTEP_SYNC:
	//.		db->ptp_onestep = true;
			adb->ptp_on = 1;
			//dev_info(&adb->spidev->dev, "IOCtl - Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, true)\n", adb->ptp_on);
			netif_info(adb, hw, adb->ndev, "IOCtl: Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, true)\n", adb->ptp_on);
			//gem_ptp_set_one_step_sync(bp, 1);
			//lan743x_ptp_set_sync_ts_insert(adapter, true);
			break;
		case HWTSTAMP_TX_ON:
	//.		db->ptp_onestep = false;
			adb->ptp_on = 1;
			netif_info(adb, hw, adb->ndev, "IOCtl - Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, false)\n", adb->ptp_on);
			//gem_ptp_set_one_step_sync(bp, 0);
			//lan743x_ptp_set_sync_ts_insert(adapter, false);
			break;
		case HWTSTAMP_TX_ONESTEP_P2P:
			//ret = -ERANGE;
			netif_warn(adb, hw, adb->ndev, "IOCtl - Now db->ptp_on %d, Error Range!\n", adb->ptp_on);
			return -ERANGE;
			//break;
		default:
			netif_warn(adb, hw, adb->ndev,
				   "  tx_type = %d, UNKNOWN\n", config.tx_type);
			return -EINVAL;
			//ret = -EINVAL;
			//break;
	}

	switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			break;
		case HWTSTAMP_FILTER_PTP_V2_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		case HWTSTAMP_FILTER_PTP_V2_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			//dev_info(&adb->spidev->dev, "config->rx_filter - to be, HWTSTAMP_FILTER_PTP_V2_EVENT\n"); //~ db->ptp_on = 1;
			netif_info(adb, hw, adb->ndev, "config->rx_filter: Master.Slave.Has, to be HWTSTAMP_FILTER_PTP_V2_EVENT\n");
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		case HWTSTAMP_FILTER_ALL:
			//db->ptp_on = 1;
			netif_info(adb, hw, adb->ndev, "config->rx_filter - to be, HWTSTAMP_FILTER_ALL\n");
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			break;
		default:
			netif_warn(adb, hw, adb->ndev,
					   "  rx_filter = %d, UNKNOWN\n", config.rx_filter);
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			return -ERANGE;
	}

//	switch (config.tx_type) {
//	case HWTSTAMP_TX_OFF:
//		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
//			index++)
//			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
//							 false, false);
//		lan743x_ptp_set_sync_ts_insert(adapter, false);
//		break;
//	case HWTSTAMP_TX_ON:
//		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
//			index++)
//			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
//							 true, false);
//		lan743x_ptp_set_sync_ts_insert(adapter, false);
//		break;
//	case HWTSTAMP_TX_ONESTEP_SYNC:
//		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
//			index++)
//			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
//							 true, true);

//		lan743x_ptp_set_sync_ts_insert(adapter, true);
//		break;
//	case HWTSTAMP_TX_ONESTEP_P2P:
//		ret = -ERANGE;
//		break;
//	default:
//		netif_warn(adapter, drv, adapter->netdev,
//			   "  tx_type = %d, UNKNOWN\n", config.tx_type);
//		ret = -EINVAL;
//		break;
//	}

	if (!ret) {
		/* copy to db tstamp_config */
		memcpy(&adb->tstamp_config, &config,
		       sizeof(adb->tstamp_config));

		/* copy to user */
		return copy_to_user(ifr->ifr_data, &config,
			sizeof(config)) ? -EFAULT : 0;
	}

	return ret;
}

/* ethtool_ops
 * tell timestamp info and types */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info)
#else
int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
#endif
{
	struct board_info *db = netdev_priv(net_dev);
	
//Spenser - get phc_index	
	//info->phc_index = -1;
	info->phc_index = db->ptp_clock ? ptp_clock_index(db->ptp_clock) : -1;


	info->so_timestamping =
#if 1
#if 0
		/* .software ts */
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
#endif
#endif
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types =
		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
		BIT(HWTSTAMP_TX_OFF) |
		BIT(HWTSTAMP_TX_ON);

	info->rx_filters =
		BIT(HWTSTAMP_FILTER_NONE) |
		BIT(HWTSTAMP_FILTER_ALL);


	return 0;
}

/* netdev_ops 
 * tell support ptp */
int dm9051_ptp_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	//struct board_info *db = to_dm9051_board(ndev);
    //struct hwtstamp_config config;
	if (!netif_running(ndev))
		return -EINVAL;

	switch(cmd) {
		case SIOCGHWTSTAMP:
			//printk("Process SIOCGHWTSTAMP\n");
			//db->ptp_on = 1;
			//return dm9051_ptp_get_ts_config(ndev, rq);
			return lan_ptp_get_ts_ioctl(ndev, rq);
		case SIOCSHWTSTAMP:
			//printk("Process SIOCSHWTSTAMP\n");
			//db->ptp_on = 1;
			//return dm9051_ptp_set_ts_config(ndev, rq);
			return lan743x_ptp_ioctl(ndev, rq, cmd);
		case SIOCBONDINFOQUERY:
			printk("dm9051_netdev_ioctl SIOCBONDINFOQUERY = cmd 0x%X. NOT support\n", cmd);
			return -EOPNOTSUPP;
		default:
			//printk("dm9051_netdev_ioctl cmd = 0x%X\n", cmd);
			//db->ptp_on = 0;
			//return -EOPNOTSUPP;
			break;
	}

	printk("dm9051_netdev_ioctl phy_mii_ioctl, cmd = 0x%X\n", cmd);
	return phy_mii_ioctl(ndev->phydev, rq, cmd); //'rq' is ifr
}

/* APIs */
int ptp_new(struct board_info *db) {
	db->ptp_enable = 1; // Enable PTP - For the driver whole operations
	return 1;
}
void ptp_init_rcr(struct board_info *db) {
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_RXEN; //_15888_ //Disable discard CRC error (work around)
}
u8 ptp_status_bits(struct board_info *db) {
	u8 err_bits = RSR_ERR_BITS;

	if (db->ptp_enable) {
		err_bits &= ~RSR_PTP_BITS; //_15888_ //To allow support "Enable PTP" must disable checksum_offload
	}
	return err_bits;
}
int is_ptp_rxts_enable(struct board_info *db) {
	return (db->rxhdr.status & RSR_RXTS_EN) ? 1 : 0; //if T1/T4, // Is it inserted Timestamp?
}

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp1"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
