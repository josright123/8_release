/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_
/* reserved-rsrv
 */
/*#define DMPLUG_CRYPT */ //(crypt plug-in)

//#define PLUG_CRYPT
//#ifdef PLUG_CRYPT
//#define DMPLUG_CRYPT //(crypt plug-in)
//#endif

//[overlay rsrv]
#if defined(PCO) && defined(DMPLUG_CRYPT) && defined(MAIN_DATA)
//overlay by plug/
#undef BUS_SETUP
#define BUS_SETUP(db) bus_setup(struct board_info *db)
#undef BUS_OPS
#define BUS_OPS(db, buff, crlen) bus_ops(struct board_info *db, u8 *buff, unsigned int crlen)
	
//implement in plug/
int bus_setup(struct board_info *db);
void bus_ops(struct board_info *db, u8 *buff, unsigned int crlen);
#endif

/* plug-in
 */
/*#define DMPLUG_CONTI */ //(tx conti)
/*#define DMPLUG_LPBK_TST */ //(mac loopback test)

//#define PLUG_CONTI
#ifdef PLUG_CONTI
#define DMPLUG_CONTI //(tx conti)
#endif

//#define PLUG_LOOPBACK_TEST
#ifdef PLUG_LOOPBACK_TEST
#define DMPLUG_LPBK_TST //(loopback test, extra-run a mac loopback test before driver finish initialize, still can work for networking!)
#endif

#if defined(DMPLUG_CONTI) && defined(MAIN_DATA)
#pragma message("TX: CONTI Plug-in")
#endif

#if defined(DMPLUG_LPBK_TST) && defined(MAIN_DATA)
#pragma message("TEST: MAC_LOOPBACK")
#endif

#if defined(DMPLUG_CONTI)
#undef INFO_CONTI
#define INFO_CONTI(dev, db) 				USER_CONFIG(dev, db, "dm9051 CONTI")
#endif

#if defined(DMPLUG_LPBK_TST)
#undef INFO_LPBK_TST
#define INFO_LPBK_TST(dev, db)				USER_CONFIG(dev, db, "dm9051 MAC Loopback Test")
#endif

/* PCO, */
#define PCO //(Coerce)

#if defined(PCO) && defined(DMPLUG_CONTI) && defined(MAIN_DATA)
#undef TX_PAD
#define TX_PAD(b,s)							s //~wd~bd, cause by tc-conti 
#endif

#if defined(PCO) && defined(DMPLUG_CONTI) && defined(MAIN_DATA)
#undef SET_RCR
#define SET_RCR(b) TX_MOTE2_CONTI_RCR(b)

#undef TX_SEND
#define TX_SEND(b,s) TX_MODE2_CONTI_TCR(b,s, param->tx_timeout_us /* _us is global */

//implement in plug/
int TX_MOTE2_CONTI_RCR(struct board_info *db);
int TX_MODE2_CONTI_TCR(struct board_info *db, struct sk_buff *skb, u64 tx_timeout_us);
#endif

#if defined(DMPLUG_LPBK_TST)
#undef dmplug_loop_test
#define dmplug_loop_test(b)	test_loop_test(b)

//implement in plug/
int test_loop_test(struct board_info *db); //implement in plug/dm9051_lpbk_test.c
#endif

#endif //_DM9051_PLUG_H_
