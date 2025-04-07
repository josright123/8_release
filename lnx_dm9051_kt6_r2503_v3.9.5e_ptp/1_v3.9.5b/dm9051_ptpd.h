#ifndef _DM9051_PTPD_H_
#define _DM9051_PTPD_H_

int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info);
int dm9051_ptp_one_step(struct sk_buff *skb);
int dm9051_hwtstamp_to_skb(struct sk_buff *skb, struct board_info *db);
/*static*/ extern const struct ethtool_ops dm9051_ptpd_ethtool_ops;

unsigned int dm9051_tcr_wr(struct sk_buff *skb, struct board_info *db);

void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);

s64 dm9051_get_rate_reg(struct board_info *db);

#endif //_DM9051_PTPD_H_
