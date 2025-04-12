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

/*#define DMPLUG_INT_CLKOUT */ //(INT 39)
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

//#define PLUG_MODEN
#ifdef PLUG_MODEN
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

#define PLUG_ENABLE_INT
#ifdef PLUG_ENABLE_INT
#define DMPLUG_INT //(INT 39)
#endif

//#define PLUG_INT_CLKOUT
#ifder PLUG_INT_CLKOUT
#define DMPLUG_INT_CLKOUT
#endif

//#define PLUG_INT_2STEP
#ifdef PLUG_INT_2STEP
#define INT_TWO_STEP
#endif

enum
{
	MODE_POLL = 0,
	MODE_INTERRUPT = 1,
	MODE_INTERRUPT_CLKOUT = 2, /* need pi3/pi5 test and verify more */
};

#ifdef DMPLUG_INT
  #ifdef DMPLUG_INT_CLKOUT
  #define dmplug_interrupt MODE_INTERRUPT_CLKOUT
  #define dmplug_intterrpt_des "interrupt clkout mode"
  #else
  #define dmplug_interrupt MODE_INTERRUPT
  #define dmplug_intterrpt_des "interrupt mode"
  #endif

  #ifdef INT_TWO_STEP
  #define dmplug_intterrpt2 "interrupt two step"
  #else
  #define dmplug_intterrpt2 "interrupt direct trigger"
  #endif
  //".DMPLUG_INT"
#else
#define dmplug_interrupt MODE_POLL
#define dmplug_intterrpt_des "poll mode"
#endif

/* Log definitions */
#ifdef DMPLUG_CONTI
#define dmplug_tx "continue"
#else
#define dmplug_tx "normal"
#endif

#ifdef DMCONF_AARCH_64
#define PRINT_REGMAP_BLK_ERR(pstr, ret, reg, BLKLEN) \
			netif_err(db, drv, db->ndev, "%s: error %d noinc %s regs %02x len %lu\n", \
				   __func__, ret, pstr, reg, BLKLEN)
#else
#define PRINT_REGMAP_BLK_ERR(pstr, ret, reg, BLKLEN) \
			netif_err(db, drv, db->ndev, "%s: error %d noinc %s regs %02x len %u\n", \
				   __func__, ret, pstr, reg, BLKLEN)
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
struct driver_config
{
	const char *release_version;
	int interrupt;
};
const struct driver_config confdata = {
	.release_version = "lnx_dm9051_kt6631_r2502_v3.9.1",
	.interrupt = dmplug_interrupt, //as 
	 /* dmplug_interrupt =
	  * MODE_POLL, 
	  * MODE_INTERRUPT or 
	  * MODE_INTERRUPT_CLKOUT */
};
const struct driver_config *drvdata = &confdata;
#else
//.
#endif

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
const struct eng_config *econf = &engdata;
const struct eng_sched csched = engdata.sched;
#else
extern const struct eng_config engdata;
extern const struct eng_config *econf;
extern const struct eng_sched csched;
#endif

//#define econf   (&engdata)
//#define csched  (engdata.sched)

/*
 * SPI sync: 
 */

int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb); //used in the plug section
int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val); //to used in the plug section

/*
 * Interrupt: 
 */

void SHOW_POLL_MODE(int cint, struct spi_device *spi);
void SHOW_INT_MODE(int cint, struct spi_device *spi);

unsigned int dm9051_intcr_value(struct board_info *db);
void INIT_RX_DELAY_SETUP(int cint, struct board_info *db);
int INIT_REQUEST_IRQ(int cint, struct net_device *ndev);
void END_FREE_IRQ(int cint, struct net_device *ndev);

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

//struct board_info;
//void dm9051_ptp_init(struct board_info *db);
//void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb, u8 *rxTSbyte);
//void dm9051_ptp_tx_hwtstamp(struct board_info *db, struct sk_buff *skb);
//void dm9051_ptp_stop(struct board_info *db);
//int dm9051_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr);
//int dm9051_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr);

//bool is_ptp_packet(struct sk_buff *skb);
//void show_ptp_types_log(char *head, struct sk_buff *skb);

// //.u8 get_ptp_message_type(struct sk_buff *skb);
//void show_ptp_type(struct sk_buff *skb);
// //s64 dm9051_get_rate_reg(struct board_info *db);

#endif //_DM9051_PLUG_H_
