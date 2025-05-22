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

/* CO, re-direct bmsr_wr */
#define CO //(Coerce)
#if defined(CO) && defined(DMCONF_BMCR_WR) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef PHY_READ
#define PHY_READ(d, n, av) dm9051_phyread_nt_bmsr(d, n, av)
#endif

/* re-direct conti */
#if defined(CO) && defined(DMPLUG_CONTI) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef TX_CONTI_NEW
#define TX_CONTI_NEW(d) tx_contu_new(d)
#endif

/* re-direct ptpc */
#if defined(CO) && defined(DMPLUG_PTP) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef PTP_NEW
#define PTP_NEW(d, n) ptp_new(d, n)
#undef PTP_INIT_RCR
#define PTP_INIT_RCR(d) ptp_init_rcr(d)
#undef PTP_INIT
#define PTP_INIT(d) ptp_init(d)
#undef PTP_END
#define PTP_END(d) ptp_end(d)
#endif

/* re-direct log */
#if defined(CO) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
#undef SHOW_DEVLOG_REFER_BEGIN
#undef SHOW_LOG_REFER_BEGIN
#undef SHOW_DEVLOG_MODE

#undef SHOW_PLAT_MODE
#undef SHOW_MAC
#undef SHOW_MONITOR_RXC

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);
#undef dm9051_headlog_regs
#undef dm9051_phyread_headlog
#undef dm9051_dump_data1
#undef monitor_rxb0
void SHOW_DEVLOG_REFER_BEGIN(struct device *dev, struct board_info *db);
void SHOW_LOG_REFER_BEGIN(struct board_info *db);
void SHOW_DEVLOG_MODE(struct device *dev);

void SHOW_PLAT_MODE(struct device *dev);
void SHOW_MONITOR_RXC(struct board_info *db, int scanrr);

//static void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);
void dm9051_headlog_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2);
int dm9051_phyread_headlog(char *head, struct board_info *db, unsigned int reg);
void dm9051_dump_data1(struct board_info *db, u8 *packet_data, int packet_len);
void monitor_rxb0(struct board_info *db, unsigned int rxbyte);
#endif

#endif //_DM9051_PLUG_H_
