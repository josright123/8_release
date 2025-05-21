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

//extern int _thread_servicep_done;
//extern int _thread_servicep_re_enter;
//extern int _thread_servicep_doneII;
//extern int _thread_servicep_re_enterII;
int thread_servicep_doneII = 1;
int thread_servicep_re_enterII = 0;

#ifdef INT_TWO_STEP
void PROBE_INT2_DLY_SETUP(struct board_info *db)
{
	
	INIT_DELAYED_WORK(&db->irq_servicep, dm9051_rx_irq_servicep);
}

void dm9051_rx_irq_servicep(struct work_struct *work) //optional: INT: TWO_STEP SRVEICE
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_servicep);

	dm9051_rx_threaded_plat(db); //dm9051_thread_irq(db); // 0 is no-used //.(macro)_rx_tx_plat()
	thread_servicep_doneII = 1;

}

irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw) //optional: INT: TWO_STEP
{
	struct board_info *db = pw;

	if (thread_servicep_doneII) {
		thread_servicep_doneII = 0;
		if (!thread_servicep_re_enterII)
			netif_crit(db, intr, db->ndev, "_.INT2.HANDLED   [%s] first-enter %d\n", __func__, thread_servicep_re_enterII++);
		schedule_delayed_work(&db->irq_servicep, 0); //dm9051_rx_int2-plat(voidirq, pw);
	}
	else {
		if (thread_servicep_re_enterII <= 9)
			netif_err(db, intr, db->ndev, "_.INT2.HANDLED   [%s] re-enter %d\n", __func__, thread_servicep_re_enterII++);
	}
	return IRQ_HANDLED;
}

int DM9051_INT2_REQUEST(struct board_info *db, irq_handler_t handler)
{
	struct spi_device *spi = db->spidev;
	int ret;

	netif_crit(db, intr, db->ndev, "request_irq(INT TWO_STEP)\n");
	thread_servicep_re_enterII = 0; //used in 'dm9051_rx_int2_delay'
	ret = request_threaded_irq(spi->irq, NULL, handler,
								get_dts_irqf(db) | IRQF_ONESHOT,
								db->ndev->name, db);
	//ret = request_irq(ndev->irq, handdler,
	//							get_dts_irqf(db) | IRQF_ONESHOT,
	//							ndev->name, db);
	if (ret < 0)
		netif_err(db, intr, db->ndev, "failed to rx request irq setup\n");
	return ret;
}
#endif
