#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_

/* Macro domain
 */
/*#define DMPLUG_CONTI */ //(conti)
/*#define DMPLUG_CRYPT */ //(crypt)
/*#define DMPLUG_PTP */ //(ptp1588)

/* Macro for already known platforms
 */ 
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

#if 1
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
	};
	const struct driver_config confdata = {
		.release_version = "lnx_dm9051_kt6631_r2502_v3.9.1",
	};
	const struct eng_config engdata = {
		.force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
		.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
		.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,
		.sched = {
			.delayF = {0, 1, 0, 0, 1}, 
			.nTargetMaxNum = POLL_OPERATE_NUM},
		.tx_timeout_us = 210000, //2100,
	};
	const struct eng_config *econf = &engdata;
	const struct eng_sched csched = engdata.sched;
	#else
	extern const struct eng_config engdata;
	extern const struct eng_config *econf;
	extern const struct eng_sched csched;
	#endif

	/*
	 * SPI sync: 
	 */

	int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb); //used in the plug section
	int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val); //to used in the plug section
	int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
							   size_t len);

	void SHOW_MODE(struct spi_device *spi);
	int DM9051_OPEN_REQUEST(struct board_info *db);

	/*
	 * Polling: 
	 */
	#ifdef DMPLUG_INT
	void dm9051_rx_irq_servicep(struct work_struct *work);
	irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw);
	#else //~DMPLUG_INT =POLL
	void dm9051_poll_servicep(struct work_struct *work); //.dm9051_poll_delay_plat()

	void INIT_RX_POLL_DELAY_SETUP(struct board_info *db);
	void INIT_RX_POLL_SCHED_DELAY(struct board_info *db);
	#endif

	/*
	 * Interrupt: 
	 */

	void INIT_RX_INT2_DELAY_SETUP(struct board_info *db);
	int INIT_REQUEST_IRQ(struct net_device *ndev);
	void END_FREE_IRQ(struct net_device *ndev);

	/*
	 * Conti: 
	 */

	int TX_SET_CONTI(struct board_info *db);
	int TX_OPS_CONTI(struct board_info *db, struct sk_buff *skb); //, u8 *buff, unsigned int len);
	int TX_SEND_CONTI(struct board_info *db, struct sk_buff *skb);

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
#endif

/*
 * Engineering Verification
 */
#ifdef MAIN_DATA
//#ifdef DMCONF_AARCH_64
//#pragma message("dm9051 AARCH_64")
////#warning "dm9051 AARCH_64"
//#else
//#pragma message("dm9051 AARCH_32")
////#warning "dm9051 AARCH_32"
//#endif

//#ifdef DMCONF_DIV_HLPR_32
//#pragma message("dm9051 DIV_HLPR_32")
////#warning "dm9051 DIV_HLPR_32"
//#endif

#ifdef DMPLUG_CONTI
#warning "dm9051 CONTI"
#endif

#ifdef DMPLUG_CRYPT
#warning "dm9051 CRYPT"
#endif

#ifdef DMPLUG_PTP
#pragma message("dm9051 PTP")
//#warning "dm9051 PTP"
#endif
#endif

#endif //_DM9051_PLUG_H_
