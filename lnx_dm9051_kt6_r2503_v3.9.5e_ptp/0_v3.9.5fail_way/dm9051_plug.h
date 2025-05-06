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

//inline
#ifdef DMPLUG_CRYPT
int BUS_SETUP(struct board_info *db);
void BUS_OPS(struct board_info *db, u8 *buff, unsigned int crlen);
#else
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty
#endif

/*
 * Conti: 
 */
#ifdef DMPLUG_CONTI
int TX_SET_CONTI(struct board_info *db);
int TX_OPS_CONTI(struct board_info *db, struct sk_buff *skb); //, u8 *buff, unsigned int len);
int TX_SEND_CONTI(struct board_info *db, struct sk_buff *skb);
#endif

#endif //_DM9051_PLUG_H_
