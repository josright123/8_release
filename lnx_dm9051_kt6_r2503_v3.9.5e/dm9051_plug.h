#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_
#include <linux/module.h>
#include <linux/netdevice.h>
//#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/ptp_classify.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
//#include <linux/if.h>

/* Macro domain
 */ 
/*#define DMCONF_AARCH_64 */ //(64-bit OS)
/*#define DMCONF_DIV_HLPR_32 */	//(32-bit division helper, __aeabi_ldivmod())

/*#define DMPLUG_INT */ //(INT 39)
/*#define DMPLUG_CONTI */ //(conti)
/*#define DMPLUG_CRYPT */ //(crypt)
/*#define DMPLUG_PTP */ //(ptp 1588)

/*#define INT_TWO_STEP */ //(interrupt two_step)

/* Macro for already known platforms
 */ 
//#define PLUG_CFG_HLPR
#ifdef PLUG_CFG_HLPR
#define DMCONF_DIV_HLPR_32 //(32-bit division helper, __aeabi_ldivmod())
#endif

#define PLUG_CFG64
#ifdef PLUG_CFG64
#define DMCONF_AARCH_64 //(64-bit OS)
#endif

#define PLUG_MODEN
#ifdef PLUG_MODEN
#define DMPLUG_INT //(INT 39)
#define DMPLUG_CONTI //(conti)
#endif

//#define PLUG_CUSTOMIZE_CRYP
#ifdef PLUG_CUSTOMIZE_CRYP
#define DMPLUG_CRYPT //(crypt)
#endif

#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)
#endif

//#define PLUG_INT_2STEP
#ifdef PLUG_INT_2STEP
#define INT_TWO_STEP
#endif

/*
 * MAIN Data: 
 */

enum
{
	SKB_WB_OFF = 0,
	SKB_WB_ON = 1, //'wb'
};

enum
{
	BURST_MODE_ALIGN = 0,
	BURST_MODE_FULL = 1,
};

#ifdef MAIN_DATA
const struct eng_config engdata = {
	.force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,
	.sched = {
		.delayF = {0, 1, 0, 0, 1}, 
		.nTargetMaxNum = POLL_OPERATE_NUM},
	.tx_timeout_us = 2100,
};
const static char *linux_name[] = {
        "rsrv",
        "rsrv",
        "rsrv",
        "rsrv",
        "rsrv",
        "DM9051_KERNEL_5_10",
        "DM9051_KERNEL_5_15",
        "DM9051_KERNEL_6_1",
        "DM9051_KERNEL_6_6",
        "UNKNOW",
};

const static char dm9051_stats_strings[][ETH_GSTRING_LEN] = {
        "rx_packets",
        "tx_packets",
        "rx_errors",
        "tx_errors",
        "rx_bytes",
        "tx_bytes",
        "fifo_rst",
};
#else
extern const struct eng_config engdata;
#endif

#define econf   (&engdata)
#define csched  (engdata.sched)

/*
 * SPI sync: 
 */

int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb); //used in the plug section
int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val); //to used in the plug section

/*
 * Interrupt: 
 */

enum
{
	MODE_POLL = 0,
	MODE_INTERRUPT = 1,
	MODE_INTERRUPT_CLKOUT = 2, /* need pi3/pi5 test and verify more */
};

void SHOW_POLL_MODE(int cint, struct spi_device *spi);
void SHOW_INT_MODE(int cint, struct spi_device *spi);

unsigned int dm9051_intcr_value(struct board_info *db);
void INIT_RX_DELAY_SETUP(int cint, struct board_info *db);
int INIT_RX_REQUEST_SETUP(int cint, struct net_device *ndev);
void END_RX_REQUEST_FREE(int cint, struct net_device *ndev);

void INIT_RX_POLL_DELAY_SETUP(int cpoll, struct board_info *db);
void INIT_RX_POLL_SCHED_DELAY(int cpoll, struct board_info *db);

/*
 * Conti: 
 */

int TX_SET_CONTI(struct board_info *db);
int TX_OPS_CONTI(struct board_info *db, u8 *buff, unsigned int len);

/*
 * Encrypt Protection Driver version: 
 */

#define	FORCE_BUS_ENCPT_OFF		0
#define	FORCE_BUS_ENCPT_CUST_ON		1

#define ENCPT_MODE                      FORCE_BUS_ENCPT_OFF
#define FORCE_BUS_ENCPT_FIX_KEY		0x95 //for fix selected     

//inline
#ifdef DMPLUG_CRYPT
int BUS_SETUP(struct board_info *db);
void BUS_OPS(struct board_info *db, u8 *buff, unsigned int crlen);
#else
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty
#endif

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

struct board_info;
void dm9051_ptp_init(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);
void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_stop(struct board_info *db);
int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr);
int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr);

bool is_ptp_packet(struct sk_buff *skb);
void show_ptp_types_log(char *head, struct sk_buff *skb);

u8 get_ptp_message_type(struct sk_buff *skb);
void show_ptp_type(struct sk_buff *skb);
s64 dm9051_get_rate_reg(struct board_info *db);

#endif //_DM9051_PLUG_H_
