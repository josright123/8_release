#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_

/* Macro domain
 */
/*#define DMPLUG_CONTI */ //(conti)
/*#define DMPLUG_CRYPT */ //(crypt)
/*#define DMPLUG_PTP */ //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */ //(ptp1588 pps)

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

#if defined(DMPLUG_CONTI) && defined(MAIN_DATA)
#pragma message("dm9051 CONTI")
#endif
#if defined(DMPLUG_CRYPT) && defined(MAIN_DATA)
#pragma message("dm9051 CRYPT")
#endif

#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)

  #define PLUG_PTP_PPS
  #ifdef PLUG_PTP_PPS
  #define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
  #endif
#endif

#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
#pragma message("dm9051 PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
#warning "dm9051 PPS"
#endif

/*
 * Engineering Verification
 */
//#ifdef _MAIN_DATA
 //#ifdef DMCONF_AARCH_64
 //#pragma message("dm9051 AARCH_64")
 //#else
 //#pragma message("dm9051 AARCH_32")
 //#endif

 //#ifdef DMCONF_DIV_HLPR_32
 //#pragma message("dm9051 DIV_HLPR_32")
 //#endif
//#endif //_MAIN_DATA

#if defined(DMPLUG_INT)
#ifdef INT_CLKOUT
#endif
#ifdef INT_TWO_STEP
void PROBE_INT2_DLY_SETUP(struct board_info *db);
void dm9051_rx_irq_servicep(struct work_struct *work);
irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw);
#endif
#endif

#ifndef  DMPLUG_INT
#undef dm9051_poll_sch
#define dm9051_poll_sch(d) DM9051_POLL_SCHED(d)

void dm9051_poll_servicep(struct work_struct *work);
void PROBE_POLL_SETUP(struct board_info *db);
void OPEN_POLL_SCHED(struct board_info *db);
int DM9051_POLL_SCHED(struct board_info *db);
#endif

#ifdef DMCONF_BMCR_WR
int dm9051_phyread_nt_bmsr(struct board_info *db, unsigned int reg, unsigned int *val);
#endif

#ifdef DMCONF_MRR_WR
#endif

/*
 * Conti: 
 */
#ifdef DMPLUG_CONTI
/* Log definitions */
#define dmplug_tx "continue"
void tx_contu_new(struct board_info *db);
int TX_MOTE2_CONTI_RCR(struct board_info *db);
int TX_MODE2_CONTI_TCR(struct board_info *db, struct sk_buff *skb, u64 tx_timeout_us);
#endif

//[overlay]
#ifdef DMPLUG_CRYPT
//overlay by plug/
#undef BUS_SETUP
#define BUS_SETUP(db) bus_setup(struct board_info *db)
#undef BUS_OPS
#define BUS_OPS(db, buff, crlen) bus_ops(struct board_info *db, u8 *buff, unsigned int crlen)
//implement in plug/
int bus_setup(struct board_info *db);
void bus_ops(struct board_info *db, u8 *buff, unsigned int crlen);
#endif
#endif //_DM9051_PLUG_H_
