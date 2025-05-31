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

#include "dm9051_ptp1.h" /* 0.1 ptpc */
//#include "dm9051_ptpd.h"
#include "../dm9051.h"
/*#include extern/extern.h */ //(extern/)
#include "extern.h"
#ifdef DMPLUG_PTP

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

int dm9051_ptp_tx_in_progress(struct sk_buff *skb)
{
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		return 1;
	}
	return 0;
}

void dm9051_ptp_txreq(struct board_info *db, struct sk_buff *skb)
{
	ptp_board_info_t *pbi = &db->pbi;
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
					pbi->ptp_step = (u8)(ptp_hdr->flag_field[0] & PTP_FLAG_TWOSTEP) ? PTP_TWO_STEP : PTP_ONE_STEP;
					if (pbi->ptp_step == 2) {
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

void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb)
{
	ptp_board_info_t *pbi = &db->pbi;
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
			if (pbi->ptp_enable) {
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
			if (pbi->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-followup with tstamp. \n", --slave_get_ptpFrame);
			} else {
				printk("Slave(%d)-get-followup without tstamp. \n", --slave_get_ptpFrame);
			}}
		} else
		if (message_type == PTP_MSGTYPE_DELAY_RESP) {
			if (slave_get_ptpFrameResp3)
			if (pbi->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-DELAY_RESP with tstamp. \n", --slave_get_ptpFrameResp3);
			} else {
				printk("Slave(%d)-get-DELAY_RESP without tstamp. \n", --slave_get_ptpFrameResp3);
			}}
		} else
		if (message_type == PTP_MSGTYPE_ANNOUNCE) {
			if (slave_get_ptpFrame)
			if (pbi->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d)-get-ANNOUNCE with tstamp. \n", --slave_get_ptpFrame);
			} else {
				printk("Slave(%d)-get-ANNOUNCE without tstamp. \n", --slave_get_ptpFrame);
			}}
		} else
		if (is_ptp_delayreq_packet(message_type)) {
			if (pbi->ptp_enable) {
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
			if (pbi->ptp_enable) {
			if (db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
				printk("Slave(%d) or Master get-knonw with tstamp. \n", --slave_get_ptpMisc);
			} else {
				printk("Slave(%d) or Master get-knonw without tstamp. \n", --slave_get_ptpMisc);
			}}
		}
	}
}

void dm9051_ptp_rxc_from_master(struct board_info *db)
{
	do {
	/* show that received ptp packets, while ptp_on, but ptp4l still NOT ran.
	 */
	//	static int before_slave_ptp_packets = 5;
	//	if (before_slave_ptp_packets && (!db->ptp_on) && (db->rxhdr.status & RSR_PTP_BITS)) {
	//		netif_warn(db, hw, db->ndev, "%d. On ptp_on is 0, ptp packet received!\n", before_slave_ptp_packets--);
	//	}
	} while(0);
}

static int lan_ptp_get_ts_ioctl(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	ptp_board_info_t *pbi = &db->pbi;
	struct hwtstamp_config *config = &pbi->tstamp_config;
        
	/* copy from db tstamp_config, to user */
	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;
}

static int lan743x_ptp_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct board_info *db = netdev_priv(netdev);
	ptp_board_info_t *pbi = &db->pbi;
	struct hwtstamp_config config;
	int ret = 0;

	if (!ifr) {
		netif_err(db, hw, db->ndev,
			  "SIOCSHWTSTAMP, ifr == NULL\n");
		return -EINVAL;
	}

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	if (config.flags) {
		netif_warn(db, hw, db->ndev,
			   "ignoring hwtstamp_config.flags == 0x%08X, expected 0\n",
			   config.flags);
	}

	switch (config.tx_type) {
		case HWTSTAMP_TX_OFF:
			//dev_info(&adb->spidev->dev, "IOCtl - Now db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, false)\n", adb->ptp_on);
			netif_info(db, hw, db->ndev, "IOCtl - Now db->ptp_on %d, NOTE: Stop tx sync !\n", pbi->ptp_on);
			//lan743x_ptp_set_sync_ts_insert(adapter, false);
			break;
		case HWTSTAMP_TX_ONESTEP_SYNC:
	//.		db->ptp_onestep = true;
			pbi->ptp_on = 1;
			//dev_info(&adb->spidev->dev, "IOCtl - Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, true)\n", adb->ptp_on);
			netif_info(db, hw, db->ndev, "IOCtl: Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, true)\n", pbi->ptp_on);
			//gem_ptp_set_one_step_sync(bp, 1);
			//lan743x_ptp_set_sync_ts_insert(adapter, true);
			break;
		case HWTSTAMP_TX_ON:
	//.		db->ptp_onestep = false;
			pbi->ptp_on = 1;
			netif_info(db, hw, db->ndev, "IOCtl - Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, false)\n", pbi->ptp_on);
			//gem_ptp_set_one_step_sync(bp, 0);
			//lan743x_ptp_set_sync_ts_insert(adapter, false);
			break;
		case HWTSTAMP_TX_ONESTEP_P2P:
			//ret = -ERANGE;
			netif_warn(db, hw, db->ndev, "IOCtl - Now db->ptp_on %d, Error Range!\n", pbi->ptp_on);
			return -ERANGE;
			//break;
		default:
			netif_warn(db, hw, db->ndev,
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
			netif_info(db, hw, db->ndev, "config->rx_filter: Master.Slave.Has, to be HWTSTAMP_FILTER_PTP_V2_EVENT\n");
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		case HWTSTAMP_FILTER_ALL:
			//db->ptp_on = 1;
			netif_info(db, hw, db->ndev, "config->rx_filter - to be, HWTSTAMP_FILTER_ALL\n");
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			break;
		default:
			netif_warn(db, hw, db->ndev,
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
		memcpy(&pbi->tstamp_config, &config,
		       sizeof(pbi->tstamp_config));

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
	ptp_board_info_t *pbi = &db->pbi;
	
//Spenser - get phc_index	
	//info->phc_index = -1;
	info->phc_index = pbi->ptp_clock ? ptp_clock_index(pbi->ptp_clock) : -1;

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
void ptp_ver(struct board_info *db) {
	ptp_board_info_t *pbi = &db->pbi;

	if (pbi->ptp_enable) {
		dev_info(&db->spidev->dev, "DMPLUG PTP Version\n");
		dev_info(&db->spidev->dev, "Enable PTP must COERCE to disable checksum_offload\n");
	}
}
int ptp_new(struct board_info *db) {
	ptp_board_info_t *pbi = &db->pbi;

	pbi->ptp_enable = 1; // Enable PTP - For the driver whole operations
	return 1;
}
void ptp_init_rcr(struct board_info *db) {
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_RXEN; //_15888_ //Disable discard CRC error (work around)
}
u8 ptp_status_bits(struct board_info *db) {
	return RSR_ERR_BITS & ~RSR_PTP_BITS;
//	u8 err_bits = RSR_ERR_BITS;

//	if (db->ptp_enable) {
//		err_bits &= ~RSR_PTP_BITS; //_15888_ //To allow support "Enable PTP" must disable checksum_offload
//	}
//	return err_bits;
}

int is_ptp_rxts_enable(struct board_info *db) {
	return (db->rxhdr.status & RSR_RXTS_EN) ? 1 : 0; //if T1/T4, // Is it inserted Timestamp?
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp1"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
