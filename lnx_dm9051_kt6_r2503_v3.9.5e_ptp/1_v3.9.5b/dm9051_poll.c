// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/ptp_clock_kernel.h>
#include "dm9051.h"
#include "dm9051_plug.h"

void INIT_RX_POLL_DELAY_SETUP(struct board_info *db)
{
	/* schedule delay work */
	INIT_DELAYED_WORK(&db->irq_workp, dm9051_poll_servicep); //.dm9051_poll_delay_plat()
}

void INIT_RX_POLL_SCHED_DELAY(struct board_info *db)
{
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
}

MODULE_DESCRIPTION("Davicom DM9051 driver, Poll"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
