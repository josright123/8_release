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
#include "dm9051.h"

#ifndef DMPLUG_INT

#define POLL_TABLE_NUM			5
#define POLL_OPERATE_INIT		0
#define POLL_OPERATE_NUM		1

struct eng_sched {
	unsigned long delayF[POLL_TABLE_NUM];
	u16 nTargetMaxNum;
};

struct eng_sched sched = {
	.delayF = {0, 1, 0, 0, 1},
	.nTargetMaxNum = POLL_OPERATE_NUM
};

/* !Interrupt: Poll delay work */
/* [DM_TIMER_EXPIRE2] poll extream.fast */
/* [DM_TIMER_EXPIRE1] consider not be 0, to alower and not occupy almost all CPU resource.
 * This is by CPU scheduling-poll, so is software driven!
 */
#define DM_TIMER_EXPIRE1 1
#define DM_TIMER_EXPIRE2 0
#define DM_TIMER_EXPIRE3 0

void dm9051_threaded_poll(struct work_struct *work) //.dm9051_poll_servicep()
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_workp);

	mutex_lock(&db->spi_lockm);

	while (dm9051_loop_rx(db) > 0) ; //dm9051_delayp_looping_rx_tx(db);

	mutex_unlock(&db->spi_lockm);

	if (db->bc.ndelayF >= sched.nTargetMaxNum)
		db->bc.ndelayF = POLL_OPERATE_INIT;

	schedule_delayed_work(&db->irq_workp, sched.delayF[db->bc.ndelayF++]);
}

/*
 * Polling:
 */
//static void DM9051_PROBE_DLYSETUP(struct board_info *db)
//{
//	#if defined(DMPLUG_INT)
//	/*
//	 * Interrupt:
//	 */
//	#ifdef INT_TWO_STEP
//		PROBE_INT2_DLY_SETUP(db);
//	#endif
//	#else
//	/*
//	 * Polling:
//	 */
//	PROBE_POLL_SETUP(db);
//	#endif
//}
void PROBE_POLL_SETUP(struct board_info *db)
{
	/* schedule delay work */
	db->bc.ndelayF = POLL_OPERATE_INIT;
	INIT_DELAYED_WORK(&db->irq_workp, dm9051_threaded_poll); //.dm9051_poll_servicep()
}

void OPEN_POLL_SCHED(struct board_info *db)
{
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
}

int DM9051_POLL_SCHED(struct board_info *db)
{
	netif_warn(db, intr, db->ndev, "schedule_delay(POLL MODE)\n");
	OPEN_POLL_SCHED(db);
	return 0;
}

//static void dm9051_free_irqworks(struct board_info *db)
//{
//	/* schedule delay work */
//	#if defined(DMPLUG_INT)
//	/*
//	 * Interrupt:
//	 */
//	dm9051_thread_irq_free(db->ndev);
//	#else
//	/*
//	 * Polling:
//	 */
//	cancel_delayed_work_sync(&db->irq_workp);
//	#endif
//}

//	#ifdef INT_TWO_STEP
//	cancel_delayed_work_sync(&db->irq_servicep);
//	#endif //_INT_TWO_STEP
#endif
