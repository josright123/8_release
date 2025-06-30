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
#include <generated/utsrelease.h> // For newer kernels
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

#include "../dm9051.h"
#include "../dm9051_template.h"
//#include "dm9051_ptp1.h" /* 0.1 ptpc */

/* PTP message type constants */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 10, 11)
    #define PTP_MSGTYPE_SYNC      0x0
    #define PTP_MSGTYPE_DELAY_REQ 0x1
#endif

#define PTP_MSGTYPE_SYNC_pri                  0x0
#define PTP_MSGTYPE_DELAY_REQ_pri             0x1
#define PTP_MSGTYPE_PDELAY_REQ_pri            0x2 // #define PTP_MSGTYPE_PDELAY_REQ     0x2
#define PTP_MSGTYPE_PDELAY_RESP_pri           0x3 // #define PTP_MSGTYPE_PDELAY_RESP    0x3
#define PTP_MSGTYPE_DELAY_RESP_pri            0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri 0xA

#define PTP_MSGTYPE_FOLLOW_UP                 0x8
#define PTP_MSGTYPE_DELAY_RESP                0x9
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP     0xA
#define PTP_MSGTYPE_ANNOUNCE                  0xB
#define PTP_MSGTYPE_SIGNALING                 0xC
#define PTP_MSGTYPE_MANAGEMENT                0xD

// PTP FIELD
#define PTP_ETHERTYPE                         0x88F7 // Layer 2 PTP
#define PTP_EVENT_PORT                        319    // UDP PTP EVENT
#define PTP_GENERAL_PORT                      320    // UDP PTP GENERAL

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

/* ethtool_ops
 * tell timestamp info and types */

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
//int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info)
//#else
//int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
//#endif
//{
//	struct board_info *db = netdev_priv(net_dev);
//	ptp_board_info_t *pbi = &db->pbi;
//
////Spenser - get phc_index
//	//info->phc_index = -1;
//	info->phc_index = pbi->ptp_clock ? ptp_clock_index(pbi->ptp_clock) : -1;

//	info->so_timestamping =
//#if 1
//#if 0
//		/* .software ts */
//		SOF_TIMESTAMPING_TX_SOFTWARE |
//		SOF_TIMESTAMPING_RX_SOFTWARE |
//		SOF_TIMESTAMPING_SOFTWARE |
//#endif
//#endif
//		SOF_TIMESTAMPING_TX_HARDWARE |
//		SOF_TIMESTAMPING_RX_HARDWARE |
//		SOF_TIMESTAMPING_RAW_HARDWARE;

//	info->tx_types =
//		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
//		BIT(HWTSTAMP_TX_OFF) |
//		BIT(HWTSTAMP_TX_ON);

//	info->rx_filters =
//		BIT(HWTSTAMP_FILTER_NONE) |
//		BIT(HWTSTAMP_FILTER_ALL);


//	return 0;
//}

//#if defined(DMPLUG_PTP)
netdev_features_t dm9051_ptp_constrain_features(struct net_device *ndev,
	netdev_features_t features)
{
	struct board_info *db = netdev_priv(ndev);

	if (db->pbi.ptp_enable) {
		if (features & (NETIF_F_HW_CSUM | NETIF_F_RXCSUM))
			netif_crit(db, hw, db->ndev, "dm9051a: while ptp_enable, checksum offload is NOT allow!!\n");
		features &= ~(NETIF_F_HW_CSUM | NETIF_F_RXCSUM);
	}

	return features;
}
//#endif

//#ifdef DMPLUG_PTP
/* Sync
 * Delay Request
 * Peer Delay Request
 * Peer Delay Response
 */
int is_ptp_announce_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_ANNOUNCE) ? 1 : 0;
}
int is_ptp_sync_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_SYNC) ? 1 : 0;
}
int is_ptp_delayreq_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_DELAY_REQ) ? 1 : 0;
}
int is_ptp_delayresp_packet(u8 msgtype)
{
	return (msgtype ==  PTP_MSGTYPE_DELAY_RESP) ? 1 : 0;
}
int is_peer_delayreq_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_PDELAY_REQ_pri) ? 1 : 0;
}
int is_peer_delayresp_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_PDELAY_RESP_pri) ? 1 : 0;
}
int is_peer_delayresp_followup_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri) ? 1 : 0;
}

struct ptp_header *get_ptp_header(struct sk_buff *skb)
{
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

u8 get_ptp_message_type005(struct ptp_header *ptp_hdr)
{
	//struct ptp_header *ptp_hdr = get_ptp_header(skb);

	//if (!ptp_hdr)
	//	return 0;

	//return ptp_hdr[0] & 0x0f;
	return ptp_hdr->tsmt & 0x0f;
}

int dm9051_ptp_tx_packet_monitor(struct board_info *db, struct sk_buff *skb)
{
	struct ptp_header *ptp_hdr = get_ptp_header(skb);
	if (ptp_hdr) {
		u8 message_type = get_ptp_message_type005(ptp_hdr); //for tx monitor
		db->pbi.ptp_tx_msgtype = message_type;
		if (is_ptp_sync_packet(message_type))
			; //printk("Master() - sync in SKBTX_IN_PROGRESS.\n");
		
		else if (message_type == PTP_MSGTYPE_FOLLOW_UP)
			printk("Master() - FOLLOW_UP in SKBTX_IN_PROGRESS.\n");
		else if (is_ptp_delayresp_packet(message_type))
			printk("Master() - delayRESP in SKBTX_IN_PROGRESS.\n");
		else if (is_ptp_announce_packet(message_type))
			; //printk("Master() - announce in SKBTX_IN_PROGRESS.\n");
		else if (is_ptp_delayreq_packet(message_type))
			; //printk("PTP() - delayREQ in SKBTX_IN_PROGRESS.\n");

		else if (is_peer_delayreq_packet(message_type))
			; //printk("PTP() - peerDelayREQ in SKBTX_IN_PROGRESS.\n");
		else if (is_peer_delayresp_packet(message_type))
			; //printk("PTP() - peerDelayRESP in SKBTX_IN_PROGRESS.\n");
		else if (message_type == PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri)
			; //..
		else
			printk("PTP() UNKNOW (msg type %u) in SKBTX_IN_PROGRESS.\n", message_type);
		return 1;
	}
	return 0;
}

int slave_get_ptpFrame = 109;

extern u8 *gpacket_data;
extern int gpacket_len;

struct ptp_header *dm9051_rx_ptp_hdr_monitor(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	return pbi->ptp_hdr_rx;
}

void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb)
{
	ptp_board_info_t *pbi = &db->pbi;
	struct ptp_header *ptp_hdr;
	
	gpacket_data = skb->data;
	gpacket_len = skb->len;
	
	pbi->ptp_hdr_rx = ptp_hdr = get_ptp_header(skb);
	if (ptp_hdr) { //is_ptp_packet(skb->data)
		static int slave_get_ptpFrameResp3 = 3;
		static int master_get_delayReq6 = 6; //5;
		//static int master_get_pdelayReq6 = 6; //5;
		static int master_get_pdelayResp6 = 6;
		static int slave_get_ptpMisc = 9;
		//static int total_ptp_frames = 0;
		u8 message_type = get_ptp_message_type005(ptp_hdr); //for rx monitor
		
		pbi->total_ptp_frames++;
		pbi->ptp_rx_msgtype = message_type;

		if (is_ptp_sync_packet(message_type)) {
			if (slave_get_ptpFrame)
				if (pbi->ptp_enable) {
					if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
						printk("\n");
						printk("get-sync\n"); //printk("Slave(%d)-get-sync with tstamp. \n", slave_get_ptpFrame);
						/* THIS get ts, v.s. AS dm9051_read_mem(db, DM_SPI_MRCMD, pbi->rxTSbyte, 8)
						 * THIS get ts, Pls don't store into pbi->rxTSbyte[]
						 * where had already ts data by dm9051_read_mem(db, DM_SPI_MRCMD, pbi->rxTSbyte, 8)
						 */
						dm9051_get_clk_ts(db);
						//sprintf(db->bc.head, "Slave-get-sync with tstamp, len= %3d", skb->len);
						//dm9051_dump_data1(db, skb->data, skb->len);
					} else {
						printk("Slave(%d)-get-sync without tstamp. \n", slave_get_ptpFrame);
					}
				}
		} else if (message_type == PTP_MSGTYPE_FOLLOW_UP) {
			if (slave_get_ptpFrame)
				if (pbi->ptp_enable) {
					if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
						printk("Slave(%d)-get-followup with tstamp. \n", slave_get_ptpFrame);
					} else {
						printk("Slave(%d)-get-followup without tstamp. \n", slave_get_ptpFrame);
					}
				}
		} else if (is_ptp_delayresp_packet(message_type)) { //= (message_type == PTP_MSGTYPE_DELAY_RESP)
			if (slave_get_ptpFrameResp3)
				if (pbi->ptp_enable) {
					if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
						printk("Slave(%d)-get-DELAY_RESP with tstamp. \n", --slave_get_ptpFrameResp3);
					} else {
						printk("Slave(%d)-get-DELAY_RESP without tstamp. \n", --slave_get_ptpFrameResp3);
					}
				}
		} else if (is_ptp_announce_packet(message_type)) {
			if (slave_get_ptpFrame)
				if (pbi->ptp_enable) {
					if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
						printk("Slave(%d)-get-ANNOUNCE with tstamp. \n", slave_get_ptpFrame);
					} else {
						printk("Slave(%d)-get-ANNOUNCE without tstamp. \n", slave_get_ptpFrame);
					}
				}
		} else if (is_ptp_delayreq_packet(message_type)) { //skip is_peer_delayreq_packet();
			if (pbi->ptp_enable) {
				if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
					if (master_get_delayReq6) {
						printk("Master(%d)-get-DELAY_REQ with tstamp. \n", --master_get_delayReq6);
					}
				} else {
					printk("Master-get-DELAY_REQ without tstamp.\n");
				}
			}
		} else if (is_peer_delayreq_packet(message_type)) {
			//if (pbi->ptp_enable) {
				if (is_ptp_rxts_en(db)) {	// Inserted Timestamp //skip
					//if (master_get_pdelayReq6)
					//	printk("PEER(%d)-get-PEER_DELAY_REQ with tstamp. ts bytes %d\n",
					//		--master_get_pdelayReq6, pbi->ptp_ts_bytes);
				}
				else {
					dm9051_get_clk_ts(db);
					printk("PEER-get-PEER_DELAY_REQ without tstamp. CHIP_WRONG_CONDITION !!\n");
				}
			//}
		} else if (is_peer_delayresp_packet(message_type)) {
				if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
					if (master_get_pdelayResp6) //skip
						; //printk("PEER(%d)-get-PEER_DELAY_RESP: (frame %d) tstamp ts bytes %d\n", 
							// --master_get_pdelayResp6, pbi->total_ptp_frames, pbi->ptp_ts_bytes);
				}
				else {
					printk("PEER-get-PEER_DELAY_RESP without tstamp. CHIP_WRONG_CONDITION !!\n");
				}
		} else if (message_type == PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP_pri) {
			//skip
		} else {
			if (slave_get_ptpMisc)
				if (pbi->ptp_enable) {
					if (is_ptp_rxts_en(db)) {	// Inserted Timestamp
						printk("Slave(%d) or Master get msgtype - %d W/ tstamp. \n", --slave_get_ptpMisc, message_type);
					} else {
						printk("Slave(%d) or Master get msgtype - %d W/O tstamp. \n", --slave_get_ptpMisc, message_type);
					}
				}
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
	} while (0);
}

/* APIs */
void ptp_ver(struct board_info *db)
{
	ptp_board_info_t *pbi = &db->pbi;

	if (pbi->ptp_enable) {
		dev_info(&db->spidev->dev, "DMPLUG PTP HW Version\n");
		dev_info(&db->spidev->dev, "Enable PTP HW must COERCE to disable checksum_offload\n");
	}
}

//int ptp_new(struct board_info *db)
//{
//	ptp_board_info_t *pbi = &db->pbi;

//	pbi->ptp_enable = 1; // Enable PTP - For the driver whole operations
//	return 1;
//}

void ptp_operation_extern(struct board_info *db)
{
	db->pbi.ptp_enable = 1;
}

void ptp_checksum_limit(struct board_info *db, struct net_device *ndev)
{
	if (db->pbi.ptp_enable) //(PTP_NEW(db))
		ndev->features &= ~(NETIF_F_HW_CSUM | NETIF_F_RXCSUM); //"Run PTP must COERCE to disable checksum_offload"
}

//void ptp_init_rcr(struct board_info *db)
//{
//	db->rctl.rcr_all = RCR_DIS_LONG | RCR_RXEN; //_15888_ //Disable discard CRC error (work around)
//#if 1 //[ptp p2p]
//	db->rctl.rcr_all |= RCR_ALL;
//#endif
//}

//#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptp1"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
