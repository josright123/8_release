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

#include "dm9051_ptp1.h"
#include "dm9051.h"
//#include "dm9051_ptpd.h"
#define DMCONF_DIV_HLPR_32 //(32-bit division helper, __aeabi_ldivmod())

u8 get_ptp_message_type005(struct ptp_header *ptp_hdr);

int is_ptp_sync_packet(u8 msgtype);
int is_ptp_delayreq_packet(u8 msgtype);

void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);

int ptp_9051_adjfine(struct ptp_clock_info *caps, long scaled_ppm);
int ptp_9051_adjtime(struct ptp_clock_info *caps, s64 delta);
int ptp_9051_gettime(struct ptp_clock_info *caps,
	struct timespec64 *ts);
int ptp_9051_settime(struct ptp_clock_info *caps,
	const struct timespec64 *ts);
int ptp_9051_feature_enable(struct ptp_clock_info *caps,
	struct ptp_clock_request *rq, int on);
int ptp_9051_verify_pin(struct ptp_clock_info *caps, unsigned int pin,
    enum ptp_pin_function func, unsigned int chan);

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

/* ptpc - support functions-1 */
#if 1 //1 //0
#ifdef DMPLUG_PTP
//u8 get_ptp_message_type(struct sk_buff *skb) {
//    struct udphdr *p_udp_hdr;
//    u8 *ptp_hdr;

//    p_udp_hdr = udp_hdr(skb);
//    ptp_hdr = (u8 *)p_udp_hdr + sizeof(struct udphdr);

//    return ptp_hdr[0] & 0x0f;
//}

//static int is_ptp_packet(const u8 *packet) {
//    struct ethhdr *eth = (struct ethhdr *)packet;

//    // 檢查 Layer 2 PTP
//    if (ntohs(eth->h_proto) == PTP_ETHERTYPE) {
//        return 1;
//    }

//    // 檢查 Layer 4 UDP PTP
//    if (ntohs(eth->h_proto) == ETH_P_IP) {
//        struct iphdr *ip = (struct iphdr *)(packet + ETH_HLEN);
//        if (ip->protocol == IPPROTO_UDP) {
//            struct udphdr *udp = (struct udphdr *)(packet + ETH_HLEN + sizeof(struct iphdr));
//            if (ntohs(udp->dest) == PTP_EVENT_PORT || ntohs(udp->dest) == PTP_GENERAL_PORT) {
//                return 1;
//            }
//        }
//    }

//    return 0;
//}

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

//u8 get_ptp_message_type0050(struct sk_buff *skb) {
//    u8 *p = skb->data;
//    struct udphdr *p_udp_hdr; //= p + 14 + 20;
//    u8 *ptp_hdr;
//p += 14 + 20;
//p_udp_hdr = (struct udphdr *) p;
//	
//    ptp_hdr = (u8 *)p_udp_hdr + sizeof(struct udphdr);

//    return ptp_hdr[0] & 0x0f;
//}
//printk("B.udp %x, ptp %x\n", (unsigned int) p_udp_hdr, (unsigned int) ptp_hdr);

#if 0 //Saved
//bool is_ptp_packet(struct sk_buff *skb) {
//    struct udphdr *p_udp_hdr;
//    struct iphdr *p_ip_hdr;

//    if (skb->protocol != htons(ETH_P_IP))
//        return false;

//    p_ip_hdr = ip_hdr(skb);
//    if (p_ip_hdr->protocol != IPPROTO_UDP)
//        return false;

//    p_udp_hdr = udp_hdr(skb);
//    if (ntohs(p_udp_hdr->dest) == 319 || ntohs(p_udp_hdr->dest) == 320) {
//        return true;
//    }
//    return false;
//}
#endif

// show ptp message typee
//static void types_log(char *head, u8 message_type) {
//    switch (message_type) {
//	case 0:
//		printk("%s: PTP Sync message\n", head);
//		break;
//	case 1:
//		printk("%s: PTP Delay_Req message\n", head);
//		break;
//	case 2:
//		printk("%s: PTP Path Delay_Req message\n", head);
//		break;
//	case 3:
//		printk("%s: PTP Path Delay_Resp message\n", head);
//		break;
//	case 8:
//		printk("%s: PTP Follow_Up message\n", head);
//		break;
//	case 9:
//		printk("%s: PTP Delay_Resp message\n", head);
//		break;
//	case 0xA:
//		printk("%s: PTP Path Delay_Resp_Follow_Up message\n", head);
//		break;
//	case 0xB:
//		printk("%s: PTP Announce message\n", head);
//		break;
//	case 0xC:
//		printk("%s: PTP Signaling message\n", head);
//		break;
//	case 0xD:
//		printk("%s: PTP Management message\n", head);
//		break;
//        default:
//		printk(KERN_INFO "%s: Unknown PTP Message Type: 0x%02X\n", head, message_type);
//		break;
//    }
//}

//void show_ptp_types_log(char *head, struct sk_buff *skb)
//{
//    types_log(head, get_ptp_message_type(skb));
//}

#if 0
//#define PP_HTONS(x) ((u16)((((x) & (u16)0x00ffU) << 8) | (((x) & (u16)0xff00U) >> 8)))
//u16 lwip_htons(u16 n) {
//  return PP_HTONS(n);
//}

//static void dump_ptp_packet0(struct board_info *db, struct sk_buff *skb, u8 message_type, int count)
//{
//	//u8 message_type = get_ptp_message_type(skb);
//	struct udphdr *p_udp_hdr;
//	u8 *ptp_hdr;

//	p_udp_hdr = udp_hdr(skb);
//	ptp_hdr = (u8 *) p_udp_hdr + sizeof(struct udphdr);
//	//[.ptp .general event/or .message event]
//	if (lwip_htons(p_udp_hdr->dest) == 319 || lwip_htons(p_udp_hdr->dest) == 320) {
//		printk("\n");
//		printk("%d udp src/dst port %d / %d, %s\n", 
//			count, lwip_htons(p_udp_hdr->source), lwip_htons(p_udp_hdr->dest),
//			is_ptp_sync_packet(message_type) ? "sync" : 
//			is_ptp_delayreq_packet(message_type) ? "delayReq" :
//			"otherPtpPacket");
//		sprintf(db->bc.head, " TX LEN= %3d", skb->len);
//		dm9051_dump_data0(db, skb->data, skb->len);
//	}
//	else {
//		printk("No [udp src 319, dst 320]\n");
//		printk("count %d msg_type is %02x\n", count, message_type);
//		sprintf(db->bc.head, " TX LEN= %3d", skb->len);
//		printk("%s\n", db->bc.head);
//	}
//}

//static void dump_ptp_packet1(struct board_info *db, struct sk_buff *skb)
//{
//	struct udphdr *p_udp_hdr;
//	u8 *ptp_hdr;

//	p_udp_hdr = udp_hdr(skb);
//	ptp_hdr = (u8 *) p_udp_hdr + sizeof(struct udphdr);

//	//[.ptp .general event/or .message event]
//	if (lwip_htons(p_udp_hdr->dest) == 319 || lwip_htons(p_udp_hdr->dest) == 320) {
//		printk("\n");
//		sprintf(db->bc.head, " TX LEN= %3d", skb->len);
//		dm9051_dump_data1(db, skb->data, skb->len);
//	} else {
//		printk("\n");
//		printk("Not [udp src 319, dst 320]\n");
//		sprintf(db->bc.head, " TX LEN= %3d", skb->len);
//		dm9051_dump_data1(db, skb->data, skb->len);
//	}
//}
#endif //0
#endif //
#endif // 1

#ifdef DMPLUG_PTP
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
#endif

/* ptpc - support functions-2 */
#ifdef DMPLUG_PTP
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

#if 0
//static int dm9051_ptp_set_timestamp_mode(struct board_info *db,
//					 struct hwtstamp_config *config);
static int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr);
static int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr);
#endif
#if 0
static int dm9051_ptp_set_timestamp_mode(struct board_info *db,
					 struct hwtstamp_config *config)
{
	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
//.		db->ptp_onestep = true;
		db->ptp_on = 1;
		netif_info(db, hw, db->ndev, "Enable PTP - hw tstamp tx one step, Driver support, Set db->ptp_on %d\n", db->ptp_on);
		//gem_ptp_set_one_step_sync(bp, 1);
		//tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	case HWTSTAMP_TX_ON:
//.		db->ptp_onestep = false;
		db->ptp_on = 1;
		netif_info(db, hw, db->ndev, "Enable PTP - hw tstamp tx on, Driver support, Set db->ptp_on %d\n", db->ptp_on);
		//gem_ptp_set_one_step_sync(bp, 0);
		//tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	default:
		return -ERANGE;
	}

	switch (config->rx_filter) {
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
		db->ptp_on = 1;
		dev_info(&db->spidev->dev, "Enable PTP - filter v2, Driver support, Set db->ptp_on %d\n", db->ptp_on);
		config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;

		//rx_bd_control =  TSTAMP_ALL_PTP_FRAMES;
		//tstamp_config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		//regval = macb_readl(bp, NCR);
		//macb_writel(bp, NCR, (regval | MACB_BIT(SRTSM)));
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_ALL:
		db->ptp_on = 1;
		dev_info(&db->spidev->dev, "Enable PTP - filter all, Driver support, Set db->ptp_on %d\n", db->ptp_on);
		config->rx_filter = HWTSTAMP_FILTER_ALL;

		//rx_bd_control = TSTAMP_ALL_FRAMES;
		//tstamp_config->rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		config->rx_filter = HWTSTAMP_FILTER_NONE;
		return -ERANGE;
	}

	return 0;
}

/**
 * dm9051_ptp_get_ts_config - get hardware time stamping config
 * @netdev:
 * @ifreq:
 *
 * Get the hwtstamp_config settings to return to the user. Rather than attempt
 * to deconstruct the settings from the registers, just return a shadow copy
 * of the last known settings.
 **/

static int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	struct hwtstamp_config *config = &db->tstamp_config;
        
	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;

}

/**
 * dm9051_ptp_set_ts_config - set hardware time stamping config
 * @netdev:
 * @ifreq:
 *
 **/
static int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	struct hwtstamp_config config;
	int err;

        //dm_printk("[in %s()]", __FUNCTION__);

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	err = dm9051_ptp_set_timestamp_mode(db, &config);
	if (err)
		return err;

	/* save these settings for future reference */
	memcpy(&db->tstamp_config, &config,
	       sizeof(db->tstamp_config));

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}
#endif

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

#if 0
int lan743x_ptp_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct lan743x_adapter *adapter = netdev_priv(netdev);
	struct hwtstamp_config config;
	int ret = 0;
	int index;

	... }

static int lan743x_netdev_ioctl(struct net_device *netdev,
				struct ifreq *ifr, int cmd)
{
	if (!netif_running(netdev))
		return -EINVAL;
	if (cmd == SIOCSHWTSTAMP)
		return lan743x_ptp_ioctl(netdev, ifr, cmd);
	return phy_mii_ioctl(netdev->phydev, ifr, cmd);
}
#endif

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

#if 1
#if 0
//static unsigned int ptp_packet_classify(struct sk_buff *skb)
//{
//	unsigned int ptp_class;

//	/* Early return if no hardware timestamp is involved */
//	if (!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
//		//printk("dm9051_ptp, no hardware timestamp involved\n");
//		return PTP_CLASS_NONE; //PTP_NOT_PTP;
//	}
// 
//	/* Validate skb length */
//	if (skb->len < sizeof(struct ptp_header)) {
//		printk("dm9051_ptp, packet too short for PTP header\n");
//		return PTP_CLASS_NONE; //PTP_NOT_PTP;
//	}
//    
//	/* Classify and parse PTP packet */
//	ptp_class = ptp_classify_raw(skb);
//	if (ptp_class == PTP_CLASS_NONE)
//		return PTP_CLASS_NONE; //PTP_NOT_PTP;

//	return ptp_class;
//}

//static struct ptp_header *dm9051_ptp_udphdr(struct sk_buff *skb)
//{
//#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
//	//get_ptp_header(); ...
//#else
//	unsigned int ptp_class = ptp_packet_classify(skb);

//	if (ptp_class == PTP_CLASS_NONE)
//		return NULL;

//	return ptp_parse_header(skb, ptp_class);
//#endif
//}

//static u8 dm9051_ptp_step(struct board_info *db, struct sk_buff *skb)
//{
//	struct ptp_header *hdr = dm9051_ptp_udphdr(skb);

//	if (hdr)
//		return (u8)(hdr->flag_field[0] & PTP_FLAG_TWOSTEP) ? PTP_TWO_STEP : PTP_ONE_STEP;
//	return 0;
//}

//u8 dm9051_ptp_frame(struct board_info *db, struct sk_buff *skb)
//{
//	struct ptp_header *hdr = dm9051_ptp_udphdr(skb);

//	if (!hdr) {
//		db->ptp_packet = 0;
//		db->ptp_step = 0;
//	} else {
//		db->ptp_packet = 1;
//		db->ptp_step = (u8)(hdr->flag_field[0] & PTP_FLAG_TWOSTEP) ? PTP_TWO_STEP : PTP_ONE_STEP;
//	}
//	return db->ptp_packet;
//}
#endif

/* PTP message type classification */
enum ptp_sync_type {
    //PTP_NOT_PTP = 0,      /* Not a PTP packet or no timestamp involved */
    PTP_ONE_STEP = 1,     /* One-step sync message */
    PTP_TWO_STEP = 2,     /* Two-step sync message */
};

int is_ptp_sync_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_SYNC) ? 1 : 0;
}
int is_ptp_delayreq_packet(u8 msgtype)
{
	return (msgtype == PTP_MSGTYPE_DELAY_REQ) ? 1 : 0;
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
#endif

#if 0
/**
 * dm9051_hwtstamp_to_skb - Process hardware timestamp for a PTP packet
 * @skb: The socket buffer containing the PTP packet
 * @db: The board info structure
 *
 * Returns: 0 on success, negative error code on failure
 */
int dm9051_hwtstamp_to_skb(struct sk_buff *skb, struct board_info *db)
{
    int ret;
    static int sync5 = 3; //5;
    static int delayReq5 = 3; //5;
    u8 message_type;
    struct net_device *ndev = db->ndev;

    if (!db->ptp_on)
        return 0;

    /* Poll for TX completion */
    ret = dm9051_nsr_poll(db);
    if (ret) {
        netdev_err(ndev, "TX completion polling timeout\n");
        return ret;
    }

    /* Check if hardware timestamp is enabled */
    if (!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
        netdev_dbg(ndev, "No hardware timestamp requested\n");
        printk("No hardware timestamp requested\n");
	//dump_ptp_packet1(db, skb);
        return -EINVAL;
    }

    /* Get and validate PTP message type */
    message_type = get_ptp_message_type(skb);
    if (message_type > PTP_MSGTYPE_MANAGEMENT) {
        netdev_dbg(ndev, "Invalid PTP message type: 0x%02x\n", message_type);
        printk("Invalid PTP message type: 0x%02x\n", message_type);
	//dump_ptp_packet1(db, skb);
        return -EINVAL;
    }

    /* Process PTP message based on type */
    /* TxSync one step chip-insert-tstamp, and NOT report HW tstamp to master4l 
     * (master do)
     * TxSync two step chip-NOT-tstamp, but report HW timestamp to master4l 
     * (master do/ master4l go ahead to do followup)
     * TxDelayReq one-step/two-step chip-NOT-insert-tstamp 
     * (slave do)
     * TxDelayReq one-step/two-step report HW timestamp to slave4l 
     * (slave do)
     * TxDelayResp CAN had T4 on recv DelayReq to be added with DelayResp feedback 
     * to slave4l 
     * (master do)
     */
    switch (message_type) {
        case PTP_MSGTYPE_SYNC:
            if (sync5) {
                netdev_dbg(ndev, "TX Sync Timestamp (%d disp)\n", sync5);
		dump_ptp_packet0(db, skb, PTP_MSGTYPE_SYNC, sync5);
		sync5--;
            }
            /* Only report HW timestamp for two-step sync */
            if (db->ptp_mode == PTP_TWO_STEP) {
                dm9051_ptp_tx_hwtstamp(db, skb);
            break;

        case PTP_MSGTYPE_DELAY_REQ:
            if (delayReq5) {
                netdev_dbg(ndev, "TX Delay_Req Timestamp (%d disp)\n", delayReq5);
		dump_ptp_packet0(db, skb, PTP_MSGTYPE_DELAY_REQ, delayReq5);
		delayReq5--;
            }
            dm9051_ptp_tx_hwtstamp(db, skb);
            break;

        default:
            netdev_dbg(ndev, "Unhandled PTP message type: 0x%02x\n", message_type);
            break;
    }

    return ret;
}
#endif //0

static struct ptp_clock_info dm9051a_ptp_info = {
    .owner = THIS_MODULE,
    //.name = "DM9051A PTP",
    .max_adj = 50000000,
    .n_alarm = 0,
    .n_ext_ts = 0,
    .n_per_out = 0, //n_periodic_outputs
    //.n_pins = 0, //1; //n_programable_pins	
    .pps = 0, //1, //0,
    .adjfine = ptp_9051_adjfine,
    .adjtime = ptp_9051_adjtime,
    .gettime64 = ptp_9051_gettime,
    .settime64 = ptp_9051_settime,
    .enable = ptp_9051_feature_enable,
    .verify = ptp_9051_verify_pin,
 
};

#if 1
int ptp_9051_adjfine(struct ptp_clock_info *caps, long scaled_ppm)
{
//struct aq_ptp_s *aq_ptp = container_of(ptp, struct aq_ptp_s, ptp_info);
    struct board_info *db = container_of(caps, struct board_info, ptp_caps);
    s64 ppm;
    s64 s64_adj;
    s64 subrate;
    u32 rate;
    // u16 hi, lo;
    u8 s_ppm[4];
    int i;
    int neg_adj = 0;
static int adjfine5 = 5;

    /* 將scaled_ppm轉換為實際ppm值 */
#if 1
    if (scaled_ppm < 0) {
        ppm = ((s64)(-scaled_ppm) * 1000) / 65536;
        ppm = -ppm;
    } else {
        ppm = ((s64)scaled_ppm * 1000) / 65536;
    }
    /* 計算調整值 */
    s64_adj = (ppm * 171797) / 1000;  // base = 171.79
#else
    /* 計算調整值 */
    #if 0
		ppm = (s64) ((scaled_ppm >= 0) ? scaled_ppm : -scaled_ppm);
    ppm = div_s64(ppm * 1000, 65536);
    s64_adj = (ppm * 171797) / 1000;  // base = 171.79
    if (scaled_ppm < 0)
        s64_adj = -s64_adj;
    #else
		ppm = (s64) ((scaled_ppm >= 0) ? scaled_ppm : -scaled_ppm);
    ppm = div_s64(ppm * 1000, 65536);
    s64_adj = div_s64(ppm * 171797, 1000); // base = 171.79
    if (scaled_ppm < 0)
        s64_adj = -s64_adj;
    #endif
#endif

    /* 保護寄存器訪問 */
    mutex_lock(&db->spi_lockm);

    /* 計算與上次調整的差值 */
    subrate = s64_adj - db->pre_rate;

    /* 處理正負值 */
    if (subrate < 0) {
        rate = (u32)(-subrate);
        neg_adj = 1;
    } else {
        rate = (u32)subrate;
        neg_adj = 0;
    }

    /* 溢出保護 */
    //if (rate > 0xffffffff)
    //    rate = 0xffffffff;
    /* 準備寄存器數據 */
    // hi = (rate >> 16);
    // lo = rate & 0xffff;
    // s_ppm[0] = lo & 0xff;
    // s_ppm[1] = (lo >> 8) & 0xff;
    // s_ppm[2] = hi & 0xFF;
    // s_ppm[3] = (hi >> 8) & 0xff;// 將32位rate值直接拆分為4個8位字節
	s_ppm[0] = rate & 0xff;           // 第1個字節（最低有效字節）
	s_ppm[1] = (rate >> 8) & 0xff;    // 第2個字節
	s_ppm[2] = (rate >> 16) & 0xff;   // 第3個字節
	s_ppm[3] = (rate >> 24) & 0xff;   // 第4個字節（最高有效字節）

if (adjfine5) {
printk("%d. Ent 0x%lX offset_pps %llX, pre_rat %llX, s64_delta_rat= 0x%llX, u32_rat= %X, sign= %d\n",
	adjfine5--, scaled_ppm, s64_adj, db->pre_rate, subrate, rate, neg_adj);
}

    /* 重置PTP時鐘控制寄存器 */
    dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
    
    /* 寫入4字節調整數據 */
    for (i = 0; i < 4; i++) {
        dm9051_set_reg(db, DM9051_1588_TS, s_ppm[i]);
    }

    /* 根據正負值設置不同的控制位 */
    if (neg_adj == 1) {
        dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
            DM9051_CCR_RATE_CTL | DM9051_CCR_PTP_RATE);
    } else {
        dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_RATE);
    }

    mutex_unlock(&db->spi_lockm);

    /* 存儲當前調整值供下次使用 */
    db->pre_rate = s64_adj;

    return 0;
}
#endif

/* phyter seems to miss the mark by 16 ns */
#define ADJTIME_FIX	16

int ptp_9051_adjtime(struct ptp_clock_info *caps, s64 delta)
{
	//remark1-slave
	//printk("...ptp_9051_adjtime\n");
	
	struct board_info *db = container_of(caps, struct board_info,
					     ptp_caps);
	struct timespec64 ts;
	int sign = 1;
	int i;
	//int err;
	u8 temp[8];
	
	//Spenser - Reset Rate register, write 0x60 bit0=1, then write bit0=0 
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00); //Enable PTP function Register offset 0x60, value 0x00
	

	//remark1-slave
	//printk("+++00111+++++ [in %s] delta = %lld+++++++++\n", __FUNCTION__ ,delta);

	//printk("@@@1 ptp_dm8806_adjtime delta %llx \n", delta);

	delta += ADJTIME_FIX;
	//printk("@@@2 ptp_dm8806_adjtime delta %llx \n", delta);

	if (delta < 0) {
		sign = 0;
		delta = -delta;

		printk("delta less han zero.. \n");

	}

	//mutex_lock(&clock->extreg_lock);
	//ADDR_LOCK_HEAD_ESSENTIAL(db); //mutex_lock

	//printk("@@@2-1 ptp_dm8806_adjtime delta %llx \n", delta);
	ts.tv_sec = div_s64(delta, 0x3b9aca00);

	//printk("@@@2-2 ptp_dm8806_adjtime delta 0x%llx sec 0x%llx \n", delta, ts.tv_sec);

	ts.tv_nsec = (delta - (ts.tv_sec * 0x3b9aca00))& 0xffffffff;
	

	//printk("@@@3 ptp_dm8806_adjtime delta %llx  nsec=%lx  \n", delta, ts.tv_nsec);

	if (sign == 0)	// delta less han zero
	{
		if (ts.tv_sec == 0) {
			printk("adjtime - delta.tv_sec = 0\n");
			ts.tv_sec++;
		}
		ts.tv_sec  = (0x100000000-ts.tv_sec);
		ts.tv_nsec = (1000000000 - ts.tv_nsec);
	}


	temp[0] = ts.tv_nsec & 0xff;
	temp[1] = (ts.tv_nsec & 0xffff) >> 8;
	temp[2] = (ts.tv_nsec >> 16) & 0xff;
	temp[3] = ts.tv_nsec >> 24;
	temp[4] = ts.tv_sec & 0xff;
	temp[5] = (ts.tv_sec & 0xffff) >> 8;
	temp[6] = (ts.tv_sec >> 16) & 0xff;
	temp[7] = ts.tv_sec >> 24;

	//dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x2);

//Spenser - Add to PTP Clock
	mutex_lock(&db->spi_lockm);

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	for (i=0; i<8; i++)
	{
		dm9051_set_reg(db, DM9051_1588_TS, temp[i] & 0xff);
	}
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_ADD);
	
	mutex_unlock(&db->spi_lockm);
	
	//remark1-slave
	//printk(" ptp_9051_adjtime hwtstamp = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
	//       temp[0], temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7]);


	//remark1-slave
	//printk("### sign %d delta %llx ts.tv_sec =  %lld, ts.tv_nsec = %ld  ###\n", sign, delta, ts.tv_sec, ts.tv_nsec);

	//mutex_unlock(&clock->extreg_lock);
	//ADDR_LOCK_TAIL_ESSENTIAL(db); //mutex_unlock

	printk("ptp_9051_adjtime...\n");
	
	return 0;

}

int ptp_9051_gettime(struct ptp_clock_info *caps,
	struct timespec64 *ts)
{
struct board_info *db = container_of(caps, struct board_info,
			 ptp_caps);
unsigned int temp[8];
int i;
unsigned int uIntTemp;

printk("DM9051A ...ptp_9051_gettime\n");

// tom: from stone's doc. write 0x84 to reg 0x61 is enough,
// bit 0 PTP_EN has been set in ptp_init
mutex_lock(&db->spi_lockm);
dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
   DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);
   
for (i=0; i< 8; i++) {
regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
temp[i] = (u8)(uIntTemp & 0xFF);
}
mutex_unlock(&db->spi_lockm);
/*
dm9051_read_mem(db, DM9051_1588_TS, temp, DM9051_1588_TS_BULK_SIZE);

dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x80);	// Reset Register 68H Index
dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01
*/	
//regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, &temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H

// tom: re-write the upper statements
ts->tv_nsec = ((uint32_t)temp[3] << 24) | ((uint32_t)temp[2] << 16) |
((uint32_t)temp[1] << 8) | (uint32_t)temp[0];
ts->tv_sec  = ((uint32_t)temp[7] << 24) | ((uint32_t)temp[6] << 16) |
((uint32_t)temp[5] << 8) | (uint32_t)temp[4];


//printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ptp_dm9051_gettime sec=%llx nsec=%lx \n", ts->tv_sec, ts->tv_nsec);

return 0;
}

int ptp_9051_settime(struct ptp_clock_info *caps,
	const struct timespec64 *ts)
{

struct board_info *db = container_of(caps, struct board_info,
			 ptp_caps);
mutex_lock(&db->spi_lockm);
printk("...ptp_9051_settime\n");

dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_nsec & 0xff));             // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 8) & 0xff));      // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 16) & 0xff));     // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 24) & 0xff));     // Write register 0x68

dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_sec & 0xff));             // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 8) & 0xff));      // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 16) & 0xff));     // Write register 0x68
dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 24) & 0xff));     // Write register 0x68

dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_WRITE);
mutex_unlock(&db->spi_lockm);


return 0;
}

int ptp_9051_feature_enable(struct ptp_clock_info *caps,
	struct ptp_clock_request *rq, int on)
{
	printk("...ptp_9051_feature_enable\n");
	return 0;
}
int ptp_9051_verify_pin(struct ptp_clock_info *caps, unsigned int pin,
			       enum ptp_pin_function func, unsigned int chan)
{
	printk("!!! 1. ptp_9051_verify_pin in\n");
	return 0;
}

static void dm9051_ptp_register(struct board_info *db)
{
	printk("\n");
	netif_info(db, hw, db->ndev, "DM9051A Driver PTP Init\n");

	db->ptp_caps = dm9051a_ptp_info; //.name = "DM9051A PTP",
	strncpy(db->ptp_caps.name, "DM9051A PTP", sizeof(db->ptp_caps.name));

	db->ptp_clock = ptp_clock_register(&db->ptp_caps,
					   &db->ndev->dev);
	if (IS_ERR(db->ptp_clock)) {
		db->ptp_clock = NULL;
		dev_err(&db->spidev->dev, "ptp_clock_register failed\n");
	}  else if (db->ptp_clock) {
		netif_warn(db, hw, db->ndev, "ptp_clock_register added PHC, index %d on %s\n",
		       ptp_clock_index(db->ptp_clock), db->ndev->name);
		
	}
	//db->ptp_flags |= IGB_PTP_ENABLED;	// Spenser - no used
}

static void dm9051_ptp_unregister(struct board_info *db)
{
	/* Disable PTP for if switch to standard version from PLUG_PTP version*/
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x02); //Disable PTP clock function Register offset 0x61, value 0x02

	if (db->ptp_clock) {
		ptp_clock_unregister(db->ptp_clock);
		db->ptp_clock = NULL;
		//printk("_[ptp] remove: PTP clock!!!\r\n");
		netif_info(db, hw, db->ndev, "_[ptp] remove: PTP clock!!!\r\n");
	}
}
#endif

#ifdef DMPLUG_PTP
/* APIs */
void ptp_new(struct board_info *db, struct net_device *ndev) {
	db->ptp_enable = 1; // Enable PTP - For the driver whole operations
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

void ptp_init(struct board_info *db) {
	/* Turn on by ptp4l run command
	 * db->ptp_on = 1; */
	db->ptp_on = 0;
	dm9051_ptp_register(db); //_15888_
	dm9051_ptp_core_init(db); //only by _probe [for further functionality test, do eliminate here, put to _open, and further _core_init]
}
void ptp_end(struct board_info *db) {
	dm9051_ptp_unregister(db); //_15888_ todo
}

void dm9051_ptp_core_init(struct board_info *db)
{
//Spenser
/*	
	dm9051_set_reg(db, DM9051_1588_RX_CONF1,
		       DM9051A_RC_SLAVE | DM9051A_RC_RX_EN | DM9051A_RC_RX2_EN);
*/
	dm9051_set_reg(db, DM9051_1588_RX_CONF1, 0x12);		//enable 8 bytes timestamp & multicast filter
	
	//Spenser - Reset Rate register, write 0x60 bit0=1, then write bit0=0 
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x01); //Disable PTP function Register offset 0x60, value 0x01
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00); //Enable PTP function Register offset 0x60, value 0x00
	
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, 0x01); //Enable PTP clock function Register offset 0x61, value 0x01
	
	//Setup GP1 to edge trigger output!
	//Register 0x60 to 0x0 (GP page (bit 1), PTP Function(bit 0))
	dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x00);

	//Register 0x6A to 0x06 (interrupt disable(bit 2), trigger or event enable(bit 1), trigger output(bit 0))
	dm9051_set_reg(db, DM9051_1588_GPIO_CONF, 0x06);
	
	//Register 0x6B to 0x02(trigger out type: edge output(bit 3:2),  triger output active high(bit 1))
	dm9051_set_reg(db, DM9051_1588_GPIO_TE_CONF, 0x02);
	
	//Stone add for 1588 Read 0x68 in one SPI cycle enable (register 0x63 bit 6 0:enable, 1:disable => 0x40)
	//Stone add for 1588 TX 1-Step checksum enable (register 0x63 bit 7 0:enable, 1:disable => 0x80)
	dm9051_set_reg(db, DM9051_1588_1_STEP_CHK, 0x00);

#ifdef DMPLUG_PPS_CLKOUT 
	dm9051_set_reg(db, 0x3C, 0xB0);
#endif
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, ptpd"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
