/*
*/

#include "dm9051.h"
//#include "dm9051_ptp.h"

#if 0
#define DE_TIMESTAMP

/* phyter seems to miss the mark by 16 ns */
#define ADJTIME_FIX	16

//Spenser - Check PTP message Type
bool is_ptp_packet(struct sk_buff *skb) {
    struct udphdr *p_udp_hdr;
    struct iphdr *p_ip_hdr;

    if (skb->protocol != htons(ETH_P_IP))
        return false;

    p_ip_hdr = ip_hdr(skb);
    if (p_ip_hdr->protocol != IPPROTO_UDP)
        return false;

    p_udp_hdr = udp_hdr(skb);
    if (ntohs(p_udp_hdr->dest) == 319 || ntohs(p_udp_hdr->dest) == 320)
        return true;

    return false;
}

u8 get_ptp_message_type(struct sk_buff *skb) {
    struct udphdr *p_udp_hdr;
    u8 *ptp_hdr;

    p_udp_hdr = udp_hdr(skb);
    ptp_hdr = (u8 *)p_udp_hdr + sizeof(struct udphdr);

    return ptp_hdr[0];
}

// show ptp message typee
void show_ptp_type(struct sk_buff *skb) {
    //if (!is_ptp_packet(skb))
    //    return;

    u8 message_type = get_ptp_message_type(skb);

    switch (message_type) {
				case 0:
					printk("PTP Sync message\n");
					break;
				case 1:
					printk("PTP Delay_Req message\n");
					break;
				case 2:
					printk("PTP Path Delay_Req message\n");
					break;
				case 3:
					printk("PTP Path Delay_Resp message\n");
					break;
				case 8:
					printk("PTP Follow_Up message\n");
					break;
				case 9:
					printk("PTP Delay_Resp message\n");
					break;
				case 0xA:
					printk("PTP Path Delay_Resp_Follow_Up message\n");
					break;
				case 0xB:
					printk("PTP Announce message\n");
					break;
				case 0xC:
					printk("PTP Signaling message\n");
					break;
				case 0xD:
					printk("PTP Management message\n");
					break;
        default:
            printk(KERN_INFO "Unknown PTP Message Type: 0x%02X\n", message_type);
            break;
    }
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

	mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// Reset Register 68H Index
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);	// dummy reset to workaround unsync
	dm9051_set_reg(db, DM9051_1588_GP_TXRX_CTRL, 0x01); //Read TX Time Stamp Clock Register offset 0x62, value 0x01

	regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// for (i=0; i< 8; i++) {
	// 	regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
	// 	temp[i] = (u8)(uIntTemp & 0xFF);
	// }
	mutex_unlock(&db->spi_lockm);

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
	u8 mRate[4];
	u32 pre_rate;
	//mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, 0x69, 0x01);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	regmap_noinc_read(db->regmap_dm, 0x68, mRate, 4);
	pre_rate = ((uint32_t)mRate[3] << 24) | ((uint32_t)mRate[2] << 16) |
		((uint32_t)mRate[1] << 8) | (uint32_t)mRate[0];
	//mutex_unlock(&db->spi_lockm);
	//printk("Pre-RateReg value = 0x%08X\n", pre_rate);

	return pre_rate;
}

static int ptp_9051_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	//remark2-slave
	//printk("...ptp_9051_adjfine\n");

 	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	//struct phy_device *phydev = clock->chosen->phydev;
	s32 rate;  //, rate_test;
	//u32 test;
	int neg_adj = 0;
	//int temp = 0;
	u8 s_ppm[4];
	u16 hi, lo;
	int i;
	//int neg_dir;
	s64 s64_adj;
	s64 ppm = (scaled_ppm * 1000) / 65536;

	//remark2-slave
	//printk("+++00112+++++ [in %s] scaled_ppm = %ld +++++++++\n", __FUNCTION__ ,scaled_ppm);


	//s64_adj =  (ppm * 170797) / 1000;		//base = 171.79
	s64_adj =  (ppm * 171797) / 1000;		//base = 171.79
	//s64_adj =  (ppm * 1719696) / 10000;		//Freq=80373
	//s64_adj =  (ppm * 1729696) / 10000;		//Freq= 79675
	//s64_adj =  (ppm * 1759696) / 10000;		//Freq= 78475
	//s64_adj =  (ppm * 1859696) / 10000;		//Freq= 74343
	//s64_adj =  (ppm * 2859696) / 10000;		//Freq= 48394
	//s64_adj =  (ppm * 3059696) / 10000;		//Freq= 45535, offset>600

	//s64_adj =  (ppm * 1659696) / 10000;		//Freq=83373,  linreg
	//s64_adj =  (ppm * 1629696) / 10000;		//Freq=84373,  linreg
	//s64_adj =  (ppm * 1619696) / 10000;		//Freq=84373,  linreg
	//s64_adj =  (ppm * 1609696) / 10000;		//Freq=85373,  linreg offset<300

	//s64_adj =  (ppm * 1589696) / 10000;		//Not use, Freq=86673,  linreg offset<300, pi not Sync

	//printk("Before Writing pre_rate = 0x%llX\n", db->pre_rate);

	s64 subrate = s64_adj - db->pre_rate;
	if (subrate < 0) {
		rate = (s32)-subrate;
		neg_adj = 1;

	}else{
		rate = (s32)subrate;
		neg_adj = 0;

	}
	db->pre_rate = s64_adj;	//store value of rate register

	//printk("After Caculated pre_rate = 0x%llX, subrate = 0x%llX, rate = 0x%X, sign = %d\n", db->pre_rate, subrate, rate, neg_adj);

	hi = (rate >> 16);
	lo = rate & 0xffff;

	s_ppm[0] = lo & 0xff;
	s_ppm[1] = (lo >> 8) & 0xff;
	s_ppm[2] = hi & 0xFF;
	s_ppm[3] = (hi >> 8) & 0xff;

//Spenser - Update PTP Clock Rate
	//mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST); //R61 W80

	for (i = 0; i < 4; i++) {
		dm9051_set_reg(db, DM9051_1588_TS, s_ppm[i]);
		//printk("s_ppm_%d = 0x%X\n", i, s_ppm[i]);
	}

	if (neg_adj == 1) {
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL,
			       DM9051_CCR_RATE_CTL | DM9051_CCR_PTP_RATE);
	}else{
		dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_RATE);
	}
	//mutex_unlock(&db->spi_lockm);

#if 0	//SPenser - Read rate Register for check

	u32 rate_reg = dm9051_get_rate_reg(db);
	printk("RateReg value = 0x%08X\n", rate_reg);
#endif

	//printk("ptp_9051_adjfine...\n");

	return 0;
}


static int ptp_9051_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	//remark1-slave
	//printk("...ptp_9051_adjtime\n");

	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	struct timespec64 ts;
	int sign = 1;
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
	for (int i=0; i<8; i++)
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

static int ptp_9051_gettime(struct ptp_clock_info *ptp,
			    struct timespec64 *ts)
{
	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);
	u8 temp[8];
	int i;
	unsigned int uIntTemp;

	printk("DM9051A ...ptp_9051_gettime\n");

	// tom: from stone's doc. write 0x84 to reg 0x61 is enough,
	// bit 0 PTP_EN has been set in ptp_init
	mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL,	DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);

	regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	printk("ptp_9051_gettime temp = 0x%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n", temp[7], temp[6],temp[5],temp[4],temp[3],temp[2],temp[1],temp[0]);

	// dm9051_set_reg(db, DM9051_1588_CLK_CTRL,	DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);
	// regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, &temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// printk("ptp_9051_gettime &temp = 0x%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n", temp[7], temp[6],temp[5],temp[4],temp[3],temp[2],temp[1],temp[0]);

	// dm9051_set_reg(db, DM9051_1588_CLK_CTRL,	DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);
	// regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, &temp[0], 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	// printk("ptp_9051_gettime &temp[0] = 0x%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n", temp[7], temp[6],temp[5],temp[4],temp[3],temp[2],temp[1],temp[0]);

	// dm9051_set_reg(db, DM9051_1588_CLK_CTRL,	DM9051_CCR_IDX_RST | DM9051_CCR_PTP_READ);
	// for (i=0; i< 8; i++) {
	// 	regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
	// 	temp[i] = (u8)(uIntTemp & 0xFF);
	// }
	// printk("ptp_9051_gettime for = 0x%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n", temp[7], temp[6],temp[5],temp[4],temp[3],temp[2],temp[1],temp[0]);
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


static int ptp_9051_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	printk("...ptp_9051_settime\n");

	struct board_info *db = container_of(ptp, struct board_info,
					     ptp_caps);

	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_nsec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_nsec >> 24) & 0xff));     // Write register 0x68

    dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)(ts->tv_sec & 0xff));             // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 8) & 0xff));      // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 16) & 0xff));     // Write register 0x68
	dm9051_set_reg(db, DM9051_1588_TS, (uint8_t)((ts->tv_sec >> 24) & 0xff));     // Write register 0x68

	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_PTP_WRITE);


	return 0;
}

static int ptp_9051_feature_enable(struct ptp_clock_info *ptp,
				   struct ptp_clock_request *rq, int on)
{
	printk("...ptp_9051_feature_enable\n");
	return 0;
}

static int ptp_9051_verify_pin(struct ptp_clock_info *ptp, unsigned int pin,
			       enum ptp_pin_function func, unsigned int chan)
{
	//struct board_info *db = container_of(ptp, struct board_info,
	//			   	         ptp_caps);

	printk("!!! 1. ptp_9051_verify_pin in\n");

	return 0;
}

#if 0
int dm9051_get_hwtst(struct net_device *dev, struct ifreq *rq)
{
	struct hwtstamp_config *tstamp_config;
	struct board_info *db = netdev_priv(dev);

	printk("...dm9051_get_hwtst\n");
	tstamp_config = &db->tstamp_config;
	return -EOPNOTSUPP;
#if 0
	if ((bp->hw_dma_cap & HW_DMA_CAP_PTP) == 0)
		return -EOPNOTSUPP;

	if (copy_to_user(rq->ifr_data, tstamp_config, sizeof(*tstamp_config)))
		return -EFAULT;
	else
		return 0;
#endif
}

int dm9051_set_hwtst(struct net_device *dev, struct ifreq *ifr, int cmd)
{
#if 0
	enum macb_bd_control tx_bd_control = TSTAMP_DISABLED;
	enum macb_bd_control rx_bd_control = TSTAMP_DISABLED;
	struct hwtstamp_config *tstamp_config;
	struct macb *bp = netdev_priv(dev);
	u32 regval;
#endif

	//struct board_info *db = netdev_priv(dev);


	printk("...dm9051_set_hwtst\n");
	return -EOPNOTSUPP;

#if 0
	tstamp_config = &bp->tstamp_config;
	if ((bp->hw_dma_cap & HW_DMA_CAP_PTP) == 0)
		return -EOPNOTSUPP;

	if (copy_from_user(tstamp_config, ifr->ifr_data,
			   sizeof(*tstamp_config)))
		return -EFAULT;

	switch (tstamp_config->tx_type) {
	case HWTSTAMP_TX_OFF:
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
		gem_ptp_set_one_step_sync(bp, 1);
		tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	case HWTSTAMP_TX_ON:
		gem_ptp_set_one_step_sync(bp, 0);
		tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	default:
		return -ERANGE;
	}

	switch (tstamp_config->rx_filter) {
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
		rx_bd_control =  TSTAMP_ALL_PTP_FRAMES;
		tstamp_config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		regval = macb_readl(bp, NCR);
		macb_writel(bp, NCR, (regval | MACB_BIT(SRTSM)));
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_ALL:
		rx_bd_control = TSTAMP_ALL_FRAMES;
		tstamp_config->rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		tstamp_config->rx_filter = HWTSTAMP_FILTER_NONE;
		return -ERANGE;
	}

	if (gem_ptp_set_ts_mode(bp, tx_bd_control, rx_bd_control) != 0)
		return -ERANGE;

	if (copy_to_user(ifr->ifr_data, tstamp_config, sizeof(*tstamp_config)))
		return -EFAULT;
	else
		return 0;
#endif
}
#endif

static struct ptp_clock_info ptp_dm9051a_info = {
    .owner = THIS_MODULE,
    .name = "DM9051A PTP",
    .max_adj = 50000000,
    .n_alarm = 0,
    .n_ext_ts = 0,
    .n_per_out = 0,
    .pps = 0,
    .adjfine = ptp_9051_adjfine,
    .adjtime = ptp_9051_adjtime,
    .gettime64 = ptp_9051_gettime,
    .settime64 = ptp_9051_settime,
    .enable = ptp_9051_feature_enable,
    .verify = ptp_9051_verify_pin,

};

void dm9051_ptp_init(struct board_info *db)
{

	db->ptp_caps = ptp_dm9051a_info;

#if 0
	db->tstamp_config.flags = 0;
	db->tstamp_config.rx_filter =
		(1 << HWTSTAMP_FILTER_ALL) |
		(1 << HWTSTAMP_FILTER_SOME) |
		(1 << HWTSTAMP_FILTER_NONE);

	db->tstamp_config.tx_type =
		(1 << HWTSTAMP_TX_ON) |
		(1 << HWTSTAMP_TX_OFF);
#endif

	db->ptp_clock = ptp_clock_register(&db->ptp_caps,
					   &db->ndev->dev);
	if (IS_ERR(db->ptp_clock)) {
		db->ptp_clock = NULL;
		dev_err(&db->spidev->dev, "ptp_clock_register failed\n");
	}  else if (db->ptp_clock) {
		printk("added PHC on %s\n",
		       db->ndev->name);

	}
	//db->ptp_flags |= IGB_PTP_ENABLED;	// Spenser - no used

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

  // GP1, GP2 pps, toggle per 1 sec
  // Set MAC REG_3CH = 0xa0
  // GP1: 80ns pulse per 1 sec
  // GP2: toggle per 1 sec
  // cspi_write_reg(0x3C, 0xA0);

  // Set MAC REG_3CH = 0xb0
  // LNKLED: 80ns pulse per 1 sec
  // SPDLED: toggle per 1 sec
  // cspi_write_reg(0x3C, 0xB0);
  dm9051_set_reg(db, 0x3C, 0xB0);

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

int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct board_info *db = netdev_priv(netdev);
	struct hwtstamp_config *config = &db->tstamp_config;

	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;

}

static int dm9051_ptp_set_timestamp_mode(struct board_info *db,
					 struct hwtstamp_config *config)
{
#if 0
	//dm_printk("[in %s()] XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXconfig->tx_type = %X, config->rx_filter = %X, config->flags = %X", __FUNCTION__, config->tx_type, config->rx_filter, config->flags);

	config->tx_type =
		(1 << HWTSTAMP_TX_OFF);
	config->rx_filter =
		(1 << HWTSTAMP_FILTER_SOME) |
		/* PTP v1, UDP, any kind of event packet */
		(1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT);

	//dm_printk("!!! 2. dm9051_ptp_set_timestamp_mode in config->tx_type=%x, config->rx_filter=%x\n", config->tx_type, config->rx_filter);
#endif

#if 0
	config->tx_type =
		(1 << HWTSTAMP_TX_ON);

	config->rx_filter =
		(1 << HWTSTAMP_FILTER_ALL) |
		(1 << HWTSTAMP_FILTER_SOME);
#endif

	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
//.		db->ptp_onestep = true;
		db->ptp_on = 1;
		//gem_ptp_set_one_step_sync(bp, 1);
		//tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	case HWTSTAMP_TX_ON:
//.		db->ptp_onestep = false;
		db->ptp_on = 1;
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
		config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;

		//rx_bd_control =  TSTAMP_ALL_PTP_FRAMES;
		//tstamp_config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		//regval = macb_readl(bp, NCR);
		//macb_writel(bp, NCR, (regval | MACB_BIT(SRTSM)));
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_ALL:
		db->ptp_on = 1;
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
 * dm9051_ptp_set_ts_config - set hardware time stamping config
 * @netdev:
 * @ifreq:
 *
 **/
int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr)
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

void dm9051_ptp_stop(struct board_info *db)
{

	if (db->ptp_clock) {
		ptp_clock_unregister(db->ptp_clock);
		db->ptp_clock = NULL;
		printk("dm9051_ptp_stop remove PTP clock!!!\r\n");
	}
}
#endif

//EXPORT_SYMBOL(dm9051_ptp_init);
MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
