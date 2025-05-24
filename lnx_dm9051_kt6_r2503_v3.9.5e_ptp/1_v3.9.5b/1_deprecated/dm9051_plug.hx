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

//#define PLUG_PTP_1588
//#ifdef PLUG_PTP_1588
//#define DMPLUG_PTP //(ptp 1588)

//  #define PLUG_PTP_PPS
//  #ifdef PLUG_PTP_PPS
//  #define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
//  #endif
//#endif

//#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
//#pragma message("dm9051 PTP")
//#endif
//#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
//#warning "dm9051 PPS"
//#endif

/* CO, */
#define CO //(Coerce)

/* re-direct ptpc */
//#if defined(CO) && defined(DMPLUG_PTP) && (defined(SECOND_MAIN) || defined(MAIN_DATA))
//#undef PTP_NEW
//#define PTP_NEW(d, n) ptp_new(d, n)
//#undef PTP_INIT_RCR
//#define PTP_INIT_RCR(d) ptp_init_rcr(d)
//#undef PTP_INIT
//#define PTP_INIT(d) ptp_init(d)
//#undef PTP_END
//#define PTP_END(d) ptp_end(d)
//#endif

#endif //_DM9051_PLUG_H_
