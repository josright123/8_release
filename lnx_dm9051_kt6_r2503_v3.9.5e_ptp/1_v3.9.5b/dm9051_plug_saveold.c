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
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/ptp_clock_kernel.h>
#include "dm9051.h"
#include "dm9051_plug.h"

#ifdef DMCONF_AARCH_64
#warning "dm9051 AARCH_64"
#else
#warning "dm9051 AARCH_32"
#endif

#ifdef DMCONF_DIV_HLPR_32
#warning "dm9051 DIV_HLPR_32"
#endif

#ifdef DMPLUG_CONTI
#warning "dm9051 CONTI"
#endif

#ifdef DMPLUG_PTP
#warning "dm9051 PTP"
#endif

/* INT and INT two_step */
#ifdef DMPLUG_INT
#warning "dm9051 INT"
#ifdef INT_TWO_STEP
#warning "INT: TWO_STEP"
#endif
#endif

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

/*
 * Interrupt: 
 */

void SHOW_INT_MODE(struct spi_device *spi)
{
	//if (cint)
	//{
		unsigned int intdata[2];
		of_property_read_u32_array(spi->dev.of_node, "interrupts", &intdata[0], 2);
		dev_info(&spi->dev, "Operation: Interrupt pin: %d\n", intdata[0]); // intpin
		dev_info(&spi->dev, "Operation: Interrupt trig type: %d\n", intdata[1]);
		#ifdef INT_TWO_STEP
		dev_info(&spi->dev, "Interrupt: Two_step\n");
		#endif
	//}
}

void SHOW_POLL_MODE(struct spi_device *spi)
{
	//if (!cint)
	//{
		int i;
		dev_info(&spi->dev, "Operation: Polling mode\n"); //~intpin
		dev_info(&spi->dev, "Operation: Polling operate count %d\n", csched.nTargetMaxNum);
		for (i = 0; i < csched.nTargetMaxNum; i++)
		{
			dev_info(&spi->dev, "Operation: Polling operate delayF[%d]= %lu\n", i, csched.delayF[i]);
		}
	//}
}

static int dm9051_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

//static 
unsigned int dm9051_intcr_value(struct board_info *db)
{
	return (dm9051_irq_flag(db) == IRQF_TRIGGER_LOW || dm9051_irq_flag(db) == IRQF_TRIGGER_FALLING) ? INTCR_POL_LOW : INTCR_POL_HIGH;
}

void INIT_RX_INT2_DELAY_SETUP(struct board_info *db)
{
	#ifdef INT_TWO_STEP
	//if (cint)
	INIT_DELAYED_WORK(&db->irq_servicep, dm9051_rx_irq_servicep);
	#endif //INT_TWO_STEP
}

int INIT_REQUEST_IRQ(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;
	//if (cint) {
	#ifdef INT_TWO_STEP
		ret = request_threaded_irq(ndev->irq, NULL, dm9051_rx_int2_delay,
									dm9051_irq_flag(db) | IRQF_ONESHOT,
									ndev->name, db);
		//ret = request_irq(ndev->irq, dm9051_rx_int2_delay,
		//							dm9051_irq_flag(db) | IRQF_ONESHOT,
		//							ndev->name, db);
		if (ret < 0)
			netdev_err(ndev, "failed to rx request irq setup\n");
	#else //INT_TWO_STEP
		ret = request_threaded_irq(ndev->irq, NULL, /*dm9051_rx_threaded_plat*/ /*dm9051_rx_int2_delay*/ dm9051_rx_threaded_plat,
		 						   dm9051_irq_flag(db) | IRQF_ONESHOT,
		 						   ndev->name, db);
		if (ret < 0)
			netdev_err(ndev, "failed to rx request threaded irq setup\n");
	#endif //INT_TWO_STEP
	//}
	return ret;
}

void END_FREE_IRQ(struct net_device *ndev)
{
	//if (cint)
	//{
	struct board_info *db = to_dm9051_board(ndev);
	free_irq(db->spidev->irq, db);
	printk("_stop [free irq %d]\n", db->spidev->irq);
	//}
}

void INIT_RX_POLL_DELAY_SETUP(struct board_info *db)
{
	//if (cpoll)
	/* schedule delay work */
	INIT_DELAYED_WORK(&db->irq_workp, dm9051_irq_delayp); //.dm9051_poll_delay_plat()
}

void INIT_RX_POLL_SCHED_DELAY(struct board_info *db)
{
	//if (cpoll)
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
}

/*
 * Conti: 
 */

#ifdef DMPLUG_CONTI
/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static void dm9051_create_tx_head(u8 *th, unsigned int len)
{
	th[0] = len & 0xff;
	; //;;todo
	th[1] = (len >> 8) & 0xff;
	th[2] = 0x00;
	th[3] = 0x80;
}

/* Get space of 3b max
 */
static unsigned int get_tx_free(struct board_info *db)
{
	int ret;
	unsigned int rb;

	ret = regmap_read(db->regmap_dm, DM9051_TXFSSR, &rb); // quick direct
	if (ret < 0)
	{
		netif_err(db, drv, db->ndev, "%s: error %d read reg %02x\n",
				  __func__, ret, DM9051_TXFSSR);
		return 0; // size now 'zero'
	}

	return (rb & 0xff) * 64;
}

static unsigned int tx_free_poll_timeout(struct board_info *db, unsigned int tx_tot,
										 u32 sleep_us, u64 timeout_us)
{
	unsigned int tx_free;
	for (;;)
	{
		tx_free = get_tx_free(db);
		if (tx_free >= tx_tot)
			return tx_tot;
		if (!sleep_us)
			break;
		if (timeout_us < sleep_us)
			break;
		timeout_us -= sleep_us;
		udelay(sleep_us);
	}
	printk("dm9051 tx -ENOMEM, req_size %u, free_size %u\n", tx_tot, tx_free);
	return 0;
}

int TX_SET_CONTI(struct board_info *db)
{
	/* or, be OK to put in dm9051_set_rcr()
		 */
	dm9051_set_reg(db, DM9051_ATCR, ATCR_TX_MODE2); /* tx continue on */
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all | RCR_DIS_WATCHDOG_TIMER);
}

/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
int TX_OPS_CONTI(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	const unsigned int tx_xxhead = 4;
	unsigned int tx_xxbst = ((len + 3) / 4) * 4;
	u8 th[4];
	dm9051_create_tx_head(th, len);

	if (!tx_free_poll_timeout(db, tx_xxhead + tx_xxbst, 1, econf->tx_timeout_us))
	{					// 2100 <- 20
		return -ENOMEM; //-ETIMEDOUT or
	}

	ret = dm9051_write_mem(db, DM_SPI_MWCMD, th, 4);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, buff, tx_xxbst); //'tx_xxbst'
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_TCR, db->tcr_wr); //TCR_TXREQ
}
#endif

/*
 * ptp 1588 chip control: 
 */
#if 0
// show ptp message typee
static void types_log(char *head, u8 message_type) {
    switch (message_type) {
	case 0:
		printk("%s: PTP Sync message\n", head);
		break;
	case 1:
		printk("%s: PTP Delay_Req message\n", head);
		break;
	case 2:
		printk("%s: PTP Path Delay_Req message\n", head);
		break;
	case 3:
		printk("%s: PTP Path Delay_Resp message\n", head);
		break;
	case 8:
		printk("%s: PTP Follow_Up message\n", head);
		break;
	case 9:
		printk("%s: PTP Delay_Resp message\n", head);
		break;
	case 0xA:
		printk("%s: PTP Path Delay_Resp_Follow_Up message\n", head);
		break;
	case 0xB:
		printk("%s: PTP Announce message\n", head);
		break;
	case 0xC:
		printk("%s: PTP Signaling message\n", head);
		break;
	case 0xD:
		printk("%s: PTP Management message\n", head);
		break;
        default:
		printk(KERN_INFO "%s: Unknown PTP Message Type: 0x%02X\n", head, message_type);
		break;
    }
}

void show_ptp_types_log(char *head, struct sk_buff *skb)
{
    types_log(head, get_ptp_message_type(skb));
}

bool is_ptp_packet(struct sk_buff *skb) {
    struct udphdr *p_udp_hdr;
    struct iphdr *p_ip_hdr;

    if (skb->protocol != htons(ETH_P_IP))
        return false;

    p_ip_hdr = ip_hdr(skb);
    if (p_ip_hdr->protocol != IPPROTO_UDP)
        return false;

    p_udp_hdr = udp_hdr(skb);
    if (ntohs(p_udp_hdr->dest) == 319 || ntohs(p_udp_hdr->dest) == 320) {
//#if 0
//        show_ptp_type(skb);
//#endif
        return true;
    }

    return false;
}

u8 get_ptp_message_type(struct sk_buff *skb) {
    struct udphdr *p_udp_hdr;
    u8 *ptp_hdr;

    p_udp_hdr = udp_hdr(skb);
    ptp_hdr = (u8 *)p_udp_hdr + sizeof(struct udphdr);

	printk("ptp_hdr[0] is %02x\n", ptp_hdr[0]);
    return ptp_hdr[0];
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
	
	return pre_rate;
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
	
	//regmap_noinc_read(db->regmap_dm, DM9051_1588_TS, &temp, 8);	//Spenser -  Read HW Timestamp from DM9051A REG_68H
	for (i=0; i< 8; i++) {
		regmap_read(db->regmap_dm, DM9051_1588_TS, &uIntTemp);
		temp[i] = (u8)(uIntTemp & 0xFF);
	}
	mutex_unlock(&db->spi_lockm);

	//printk("==================================> TX_hwtstamp FROM OUT\r\n");
	ns_lo = temp[0] | (temp[1] << 8);
	ns_hi = temp[2] | (temp[3] << 8);

	s_lo = temp[4] | (temp[5] << 8);
	s_hi = temp[6] | (temp[7] << 8);

	sec = s_lo;
	sec |= s_hi << 16;

	ns = ns_lo;
	ns |= ns_hi  << 16;


	ns += ((u64)sec) * 1000000000ULL;


	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ns_to_ktime(ns);
	//skb_complete_tx_timestamp(skb, &shhwtstamps);
	//remark5-slave
	//printk("---Report TX HW Timestamp\n");
	skb_tstamp_tx(skb, &shhwtstamps);	//Report HW Timestamp
	//remark5-slave
	//printk("Report TX HW Timestamp---\n");
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

/*
 * ptp 1588 driver netdev ioctrl: 
 */


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

        //dm_printk("[in %s()]", __FUNCTION__);

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/*int err = dm9051_ptp_set_timestamp_mode(db, &config); if (err) return err;
	 */
	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
		db->ptp_onestep = true;
		db->ptp_on = 1;
		//gem_ptp_set_one_step_sync(bp, 1);
		//tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	case HWTSTAMP_TX_ON:
		db->ptp_onestep = false;
		db->ptp_on = 1;
		//gem_ptp_set_one_step_sync(bp, 0);
		//tx_bd_control = TSTAMP_ALL_FRAMES;
		break;
	default:
		return -ERANGE;
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
		db->ptp_on = 1;
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		
		//rx_bd_control =  TSTAMP_ALL_PTP_FRAMES;
		//tstamp_config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		//regval = macb_readl(bp, NCR);
		//macb_writel(bp, NCR, (regval | MACB_BIT(SRTSM)));
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_ALL:
		db->ptp_on = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		
		//rx_bd_control = TSTAMP_ALL_FRAMES;
		//tstamp_config->rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		return -ERANGE;
	}

	/* save these settings for future reference */
	memcpy(&db->tstamp_config, &config,
	       sizeof(db->tstamp_config));

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
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

/*
 * ptp 1588 driver ops: 
 */

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

#if 1
	s64 ppm = (scaled_ppm * 1000) / 65536;
	//s64_adj =  (ppm * 170797) / 1000;		//base = 171.79
	s64_adj =  (ppm * 171797) / 1000;		//base = 171.79
#else
	s64 ppm = scaled_ppm;
	
//	ppm *= 1000;
//	ppm /= 65536;
	
//	ppm = ppm / 1000;
//	ppm = ppm * 171797;
	ppm = ppm * 171797;
	ppm >>= 16;
	
	s64_adj = ppm;
#endif
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
	
	return 0;
}


static int ptp_9051_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
/* phyter seems to miss the mark by 16 ns */
#define ADJTIME_FIX	16

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
#if 1
	ts.tv_sec = div_s64(delta, 0x3b9aca00);
#else
	#if 1
	delta /= 0x3b9aca00;
	#endif
	ts.tv_sec = delta;
#endif
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
	printk("!!! 1. ptp_9051_verify_pin in\n");

	return 0;
}

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

MODULE_DESCRIPTION("Davicom DM9051 driver, Plug-in"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
