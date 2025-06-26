/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_DUMP_H_
#define _DM9051_DUMP_H_

/* pragma
 */
//#if defined(DMPLUG_LOG) && defined(MAIN_DATA)
//    #pragma message("dm9051-DBG: LOG")
//#endif

//#if defined(DMPLUG_LOG)
//    #undef INFO_LOG
//    #define INFO_LOG(dev, db) USER_CONFIG(dev, db, "dm9051-DBG: LOG")
//	#undef INFO_MSG_DBGRXC
//	#define INFO_MSG_DBGRXC(dev, db) macro_msg_dbgrxc(dev, db)
//#endif

//#undef dm9051_dump_data1
//#define dm9051_dump_data1(b, p, n) dump_data(b, p, n)

//#undef LOG_RX_PACKET_DUMP
//#define LOG_RX_PACKET_DUMP(b, s) dm9051_rx_packet_dump(b, s)

#endif //_DM9051_DUMP_H_
