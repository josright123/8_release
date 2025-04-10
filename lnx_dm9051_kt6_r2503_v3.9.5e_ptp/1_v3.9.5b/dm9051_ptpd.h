#ifndef _DM9051_PTPD_H_
#define _DM9051_PTPD_H_

int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info);
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
