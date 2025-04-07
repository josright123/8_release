
#ifndef _DM9051A_PTP_H_
#define _DM9051A_PTP_H_
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/ptp_classify.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
//#include <linux/if.h>

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

struct board_info;
void dm9051_ptp_init(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);
void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_stop(struct board_info *db);
int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr);
int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr);

bool is_ptp_packet(struct sk_buff *skb);
u8 get_ptp_message_type(struct sk_buff *skb);
void show_ptp_type(struct sk_buff *skb);
s64 dm9051_get_rate_reg(struct board_info *db);

#endif
