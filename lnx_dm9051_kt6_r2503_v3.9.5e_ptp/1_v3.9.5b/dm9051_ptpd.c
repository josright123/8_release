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
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>

//_15888_
#include <linux/ptp_clock_kernel.h>

#include "dm9051.h"
#include "dm9051_plug.h"
#include "dm9051_ptpd.h"

static u8 get_ptp_message_type(struct sk_buff *skb) {
    struct udphdr *p_udp_hdr;
    u8 *ptp_hdr;

    p_udp_hdr = udp_hdr(skb);
    ptp_hdr = (u8 *)p_udp_hdr + sizeof(struct udphdr);

    return ptp_hdr[0];
}

int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
{
	struct board_info *db = netdev_priv(net_dev);
	
//Spenser - get phc_index	
	//info->phc_index = -1;
	info->phc_index = db->ptp_clock ? ptp_clock_index(db->ptp_clock) : -1;


	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
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

/*
 * return -
 * 0: not PTP packet
 * 1: one-step
 * 2: two-step 
 * 3: Not Sync packet
*/
int dm9051_ptp_one_step(struct sk_buff *skb)
{
	struct ptp_header *hdr;
	unsigned int ptp_class;
	u8 msgtype;

	/* No need to parse packet if PTP TS is not involved */
	if (likely(!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))) {
		return 0;	// Not PTP packet
	}else{
		//printk("Check PTP Message\n");
		/* Identify and return whether PTP one step sync is being processed */
		ptp_class = ptp_classify_raw(skb);
		if (ptp_class == PTP_CLASS_NONE)
			goto no;

		hdr = ptp_parse_header(skb, ptp_class);
		if (!hdr)
			goto no;
		
		msgtype = ptp_get_msgtype(hdr, ptp_class);
		if (msgtype == PTP_MSGTYPE_SYNC) {
			
			if (hdr->flag_field[0] & PTP_FLAG_TWOSTEP) {
				//printk("two-step TX Sync Message\n");
				return 2;
			}else {
				//printk("onestep TX Sync Message\n");
				return 1;
			}
		}else{
			//printk("Not Sync Message\n");
			return 3;
		}
		
	}
no:
	return 0;
}

int dm9051_hwtstamp_to_skb(struct sk_buff *skb, struct board_info *db)
{
	int ret = 0;
	u8 message_type;

	if (db->ptp_on) {
		ret = dm9051_nsr_poll(db);	//TX completed
		if (ret){
			printk("nsr_polling timeout\n");
			return ret;
		}

		message_type = get_ptp_message_type(skb); //_15888_, 

		switch(message_type) {
			case 0:	//Sync
				//remark3-slave - none sync
				printk("TX Sync Timestamp\n");
				/*Spenser - Don't report HW timestamp to skb if one-step,
				 * otherwise master role will be not continue send Sync Message.
				*/
				if (db->ptp_mode == 2)	//two-step
					dm9051_ptp_tx_hwtstamp(db, skb); //_15888_ // Report HW Timestamp
				break;
			case 1:	//Delay Req
				//remark6-slave
				//printk("Tx Delay_Req Timestamp\n");
				
				dm9051_ptp_tx_hwtstamp(db, skb); //_15888_ // Report HW Timestamp
				//printk("Tx Delay_Req Timestamp...\n");
				break;
			default:
				break;
		}
	}

	return ret;
}

unsigned int dm9051_tcr_wr(struct sk_buff *skb, struct board_info *db)
{
	unsigned int tcr = TCR_TXREQ; // TCR register value
	db->ptp_tx_flags = skb_shinfo(skb)->tx_flags;

	if (db->ptp_tx_flags) {
		switch(db->ptp_mode){
			case 1:
				//printk("One Step...\n");
				//Stone add for one-step Sync packet insert time stamp! 2024-08-14!
				tcr = (TCR_TS_EN | TCR_TXREQ | TCR_DIS_JABBER_TIMER);
				break;
			case 2:
			case 3:
				tcr = (TCR_TS_EN | TCR_TXREQ);
				break;
			default:
				//printk("Not PTP packet\n");
				break;
		}
	}
	return tcr;
}

void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb)
{
	//struct sk_buff *skb = db->ptp_tx_skb;
	struct skb_shared_hwtstamps shhwtstamps;
	//u64 regval;
	u8 temp[9];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	u32 sec;
	u64 ns;
	int i;
	unsigned int uIntTemp = 0;


	memset(&temp, 0, sizeof(temp));

	//printk("==================================> TX_hwtstamp FROM in\r\n");

	//Spenser - Read TX timestamp from REG_68H
	//remark6-slave

//.	mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// Reset Register 68H Index
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// dummy reset to workaround unsync
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01

	regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// for (i=0; i< 8; i++) {
	// 	regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
	// 	temp[i] = (u8)(uIntTemp & 0xFF);
	// }
//.	mutex_unlock(&db->spi_lockm);

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
	skb_tstamp_tx(skb, &shhwtstamps);	//Report HW Timestamp
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


	//printk("<== dm9051_ptp_tx_hwtstamp out\r\n");
}

void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte)
{
	//u8 temp[12];
	u16 ns_hi, ns_lo, s_hi, s_lo;
	//u32 prttsyn_stat, hi, lo,
	u32 sec;
	u64 ns;
	//int i;
	struct skb_shared_hwtstamps *shhwtstamps = NULL;

	//printk("==> dm9051_ptp_rx_hwtstamp in\r\n");
	/* Since we cannot turn off the Rx timestamp logic if the device is
	 * doing Tx timestamping, check if Rx timestamping is configured.
	 */


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

	shhwtstamps = skb_hwtstamps(skb);
	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);

	//printk("Report RX Timestamp to skb = %lld\n", shhwtstamps->hwtstamp);
	//dm9051_set_reg(db, DM9051_1588_ST_GPIO, 0x08); //Clear RX Time Stamp Clock Register offset 0x60, value 0x08
	//printk("<== dm9051_ptp_rx_hwtstamp out\r\n");
}

s64 dm9051_get_rate_reg(struct board_info *db) {
	return 0;
}
