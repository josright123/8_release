/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_
/*#define DMPLUG_LPBK_TST */ //(mac loopback test)

//#define PLUG_LOOPBACK_TEST
#ifdef PLUG_LOOPBACK_TEST
#define DMPLUG_LPBK_TST //(loopback test, extra-run a mac loopback test before driver finish initialize, still can work for networking!)
#endif

#if defined(DMPLUG_LPBK_TST) && defined(MAIN_DATA)
#pragma message("TEST: MAC_LOOPBACK")
#endif

#if defined(DMPLUG_LPBK_TST)
#undef INFO_LPBK_TST

#define INFO_LPBK_TST(dev, db)				USER_CONFIG(dev, db, "dm9051 MAC Loopback Test")
#endif

int test_loop_test(struct board_info *db); //implement in plug/dm9051_lpbk_test.c

#if defined(DMPLUG_LPBK_TST)
#undef dmplug_loop_test

#define dmplug_loop_test(b)	test_loop_test(b)
#endif
#endif //_DM9051_PLUG_H_
