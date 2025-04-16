#ifndef _DM9051_PTPD_H_
#define _DM9051_PTPD_H_

/*
 * ptp 1588: 
 */

#define DM9051_1588_ST_GPIO 0x60
#define DM9051_1588_CLK_CTRL 0x61
#define DM9051_1588_GP_TXRX_CTRL 0x62
//#define DM9051_1588_TX_CONF 0x63
#define DM9051_1588_1_STEP_CHK 0x63
#define DM9051_1588_RX_CONF1 0x64
//#define DM9051_1588_RX_CONF2 0x65
#define DM9051_1588_1_STEP_ADDR 0x65
//#define DM9051_1588_RX_CONF3 0x66
#define DM9051_1588_1_STEP_ADDR_CHK 0x66
#define DM9051_1588_CLK_P 0x67
#define DM9051_1588_TS 0x68
//#define DM9051_1588_AUTO 0x69
#define DM9051_1588_MNTR 0x69
#define DM9051_1588_GPIO_CONF 0x6A
#define DM9051_1588_GPIO_TE_CONF 0x6B
#define DM9051_1588_GPIO_TA_L 0x6C
#define DM9051_1588_GPIO_TA_H 0x6D
#define DM9051_1588_GPIO_DTA_L 0x6E
#define DM9051_1588_GPIO_DTA_H 0x6F

// bits defines
// 61H Clock Control Reg
#define DM9051_CCR_IDX_RST BIT(7)
#define DM9051_CCR_RATE_CTL BIT(6)
#define DM9051_CCR_PTP_RATE BIT(5)
#define DM9051_CCR_PTP_ADD BIT(4)
#define DM9051_CCR_PTP_WRITE BIT(3)
#define DM9051_CCR_PTP_READ BIT(2)
#define DM9051_CCR_PTP_DIS BIT(1)
#define DM9051_CCR_PTP_EN BIT(0)

// 64H
#define DM9051A_RC_SLAVE BIT(7)
#define DM9051A_RC_RX_EN BIT(4)
#define DM9051A_RC_RX2_EN BIT(3)
#define DM9051A_RC_FLTR_MASK 0x3
#define DM9051A_RC_FLTR_ALL_PKTS 0
#define DM9051A_RC_FLTR_MCAST_PKTS 1
#define DM9051A_RC_FLTR_DA 2
#define DM9051A_RC_FLTR_DA_SPICIFIED 3

#define DM9051_1588_TS_BULK_SIZE 8

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info); //kt612
#else
int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info); //kt66
#endif
int dm9051_ptp_set_timestamp_mode(struct board_info *db,
					 struct hwtstamp_config *config);
int dm9051_ptp_one_step(struct sk_buff *skb);
int dm9051_hwtstamp_to_skb(struct sk_buff *skb, struct board_info *db);
//extern const struct ethtool_ops dm9051_ptpd_ethtool_ops;

unsigned int dm9051_tcr_wr(struct sk_buff *skb, struct board_info *db);

void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);

/*s64*/ u32 dm9051_get_rate_reg(struct board_info *db);

int ptp_9051_adjfine(struct ptp_clock_info *ptp, long scaled_ppm);
int ptp_9051_adjtime(struct ptp_clock_info *ptp, s64 delta);
int ptp_9051_gettime(struct ptp_clock_info *ptp,
	struct timespec64 *ts);
int ptp_9051_settime(struct ptp_clock_info *ptp,
	const struct timespec64 *ts);
int ptp_9051_feature_enable(struct ptp_clock_info *ptp,
	struct ptp_clock_request *rq, int on);
int ptp_9051_verify_pin(struct ptp_clock_info *ptp, unsigned int pin,
    enum ptp_pin_function func, unsigned int chan);
void dm9051_ptp_init(struct board_info *db);

#endif //_DM9051_PTPD_H_
