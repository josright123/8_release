// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
 
/*
 * User notice:
 *   To add log function, 
 *   And add this file to project
 *   And modify Makefile to insert this file 
 *   into the project build.
 * 
 * In Makefile
 * Change
 *   dm9051a-objs := dm9051.o
 * to
 *   dm9051a-objs := dm9051.o dm9051_wd.o
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h> // For newer kernels
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>
 
//#define SECOND_MAIN //(sec)
#include "extern/dm9051_ptp1.h" /* 0.1 ptpc */
#include "dm9051.h"
/*#include extern/extern.h */ //(extern/)
#include "extern/extern.h"


MODULE_DESCRIPTION("Davicom DM9051 driver, wd-function");
MODULE_LICENSE("GPL");
