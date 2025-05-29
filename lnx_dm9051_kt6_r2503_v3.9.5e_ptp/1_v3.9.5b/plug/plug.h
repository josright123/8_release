/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_PLUG_H_
#define _DM9051_PLUG_H_
/*#define DMPLUG_LPBK_TST */ //(mac loopback test)

#define PLUG_LOOPBACK_TEST
#ifdef PLUG_LOOPBACK_TEST
#define DMPLUG_LPBK_TST //(loopback test, extra-run a mac loopback test before driver finish initialize, still can work for networking!)
#endif

#if defined(DMPLUG_LPBK_TST) && defined(MAIN_DATA)
#pragma message("TEST: MAC_LOOPBACK")
#endif
#endif //_DM9051_PLUG_H_
