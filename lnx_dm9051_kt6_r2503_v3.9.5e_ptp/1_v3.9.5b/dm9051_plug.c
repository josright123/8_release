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

#ifdef DMCONF_AARCH_64
#warning "dm9051 AARCH_64"
#else
#warning "dm9051 AARCH_32"
#endif

#ifdef DMCONF_DIV_HLPR_32
#warning "dm9051 DIV_HLPR_32"
#endif

#ifdef DMPLUG_CONTI
#warning "dm9051 CONTI"
#endif

#ifdef DMPLUG_PTP
#warning "dm9051 PTP"
#endif

/* INT and INT two_step */
#ifdef DMPLUG_INT
#warning "dm9051 INT"
#ifdef INT_TWO_STEP
#warning "INT: TWO_STEP"
#endif
#endif

static int dm9051_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

//static 
unsigned int dm9051_intcr_value(struct board_info *db)
{
	return (dm9051_irq_flag(db) == IRQF_TRIGGER_LOW || dm9051_irq_flag(db) == IRQF_TRIGGER_FALLING) ? INTCR_POL_LOW : INTCR_POL_HIGH;
}

#ifdef DMCONF_DIV_HLPR_32
/* Implement the missing ARM EABI division helper */
long long __aeabi_ldivmod(long long numerator, long long denominator)
{
    long long res = 0;
    long long d = denominator;
    int sign = 1;

    if (numerator < 0) {
        numerator = -numerator;
        sign = -sign;
    }

    if (d < 0) {
        d = -d;
        sign = -sign;
    }

    if (d != 0) {
        /* Use the kernel's division helper */
        res = div64_s64(numerator, d);
        if (sign < 0)
            res = -res;
    }

    return res;
}
#endif

/*
 * Interrupt: 
 */

void SHOW_INT_MODE(struct spi_device *spi)
{
	unsigned int intdata[2];
	of_property_read_u32_array(spi->dev.of_node, "interrupts", &intdata[0], 2);
	dev_info(&spi->dev, "Operation: Interrupt pin: %d\n", intdata[0]); // intpin
	dev_info(&spi->dev, "Operation: Interrupt trig type: %d\n", intdata[1]);
	#ifdef INT_TWO_STEP
	dev_info(&spi->dev, "Interrupt: Two_step\n");
	#endif
}

void SHOW_POLL_MODE(struct spi_device *spi)
{
	int i;
	dev_info(&spi->dev, "Operation: Polling mode\n"); //~intpin
	dev_info(&spi->dev, "Operation: Polling operate count %d\n", csched.nTargetMaxNum);
	for (i = 0; i < csched.nTargetMaxNum; i++)
	{
		dev_info(&spi->dev, "Operation: Polling operate delayF[%d]= %lu\n", i, csched.delayF[i]);
	}
}

void INIT_RX_POLL_DELAY_SETUP(struct board_info *db)
{
	/* schedule delay work */
	INIT_DELAYED_WORK(&db->irq_workp, dm9051_irq_delayp); //.dm9051_poll_delay_plat()
}

void INIT_RX_POLL_SCHED_DELAY(struct board_info *db)
{
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
}

void INIT_RX_INT2_DELAY_SETUP(struct board_info *db)
{
	#ifdef INT_TWO_STEP
	INIT_DELAYED_WORK(&db->irq_servicep, dm9051_rx_irq_servicep);
	#endif //INT_TWO_STEP
}

int INIT_REQUEST_IRQ(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;
	#ifdef INT_TWO_STEP
		ret = request_threaded_irq(ndev->irq, NULL, dm9051_rx_int2_delay,
									dm9051_irq_flag(db) | IRQF_ONESHOT,
									ndev->name, db);
		//ret = request_irq(ndev->irq, dm9051_rx_int2_delay,
		//							dm9051_irq_flag(db) | IRQF_ONESHOT,
		//							ndev->name, db);
		if (ret < 0)
			netdev_err(ndev, "failed to rx request irq setup\n");
	#else //INT_TWO_STEP
		ret = request_threaded_irq(ndev->irq, NULL, /*dm9051_rx_threaded_plat*/ /*dm9051_rx_int2_delay*/ dm9051_rx_threaded_plat,
		 						   dm9051_irq_flag(db) | IRQF_ONESHOT,
		 						   ndev->name, db);
		if (ret < 0)
			netdev_err(ndev, "failed to rx request threaded irq setup\n");
	#endif //INT_TWO_STEP
	return ret;
}

void END_FREE_IRQ(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	free_irq(db->spidev->irq, db);
	printk("_stop [free irq %d]\n", db->spidev->irq);
}

/*
 * Conti: 
 */

#ifdef DMPLUG_CONTI
/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static void dm9051_create_tx_head(u8 *th, unsigned int len)
{
	th[0] = len & 0xff;
	; //;;todo
	th[1] = (len >> 8) & 0xff;
	th[2] = 0x00;
	th[3] = 0x80;
}

/* Get space of 3b max
 */
static unsigned int get_tx_free(struct board_info *db)
{
	int ret;
	unsigned int rb;

	ret = regmap_read(db->regmap_dm, DM9051_TXFSSR, &rb); // quick direct
	if (ret < 0)
	{
		netif_err(db, drv, db->ndev, "%s: error %d read reg %02x\n",
				  __func__, ret, DM9051_TXFSSR);
		return 0; // size now 'zero'
	}

	return (rb & 0xff) * 64;
}

static unsigned int tx_free_poll_timeout(struct board_info *db, unsigned int tx_tot,
										 u32 sleep_us, u64 timeout_us)
{
	unsigned int tx_free;
	for (;;)
	{
		tx_free = get_tx_free(db);
		if (tx_free >= tx_tot)
			return tx_tot;
		if (!sleep_us)
			break;
		if (timeout_us < sleep_us)
			break;
		timeout_us -= sleep_us;
		udelay(sleep_us);
	}
	printk("dm9051 tx -ENOMEM, req_size %u, free_size %u\n", tx_tot, tx_free);
	return 0;
}

int TX_SET_CONTI(struct board_info *db)
{
	/* or, be OK to put in dm9051_set_rcr()
		 */
	dm9051_set_reg(db, DM9051_ATCR, ATCR_TX_MODE2); /* tx continue on */
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all | RCR_DIS_WATCHDOG_TIMER);
}

/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
int TX_OPS_CONTI(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	const unsigned int tx_xxhead = 4;
	unsigned int tx_xxbst = ((len + 3) / 4) * 4;
	u8 th[4];
	dm9051_create_tx_head(th, len);

	if (!tx_free_poll_timeout(db, tx_xxhead + tx_xxbst, 1, econf->tx_timeout_us))
	{					// 2100 <- 20
		return -ENOMEM; //-ETIMEDOUT or
	}

	ret = dm9051_write_mem(db, DM_SPI_MWCMD, th, 4);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, buff, tx_xxbst); //'tx_xxbst'
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_TCR, db->tcr_wr); //TCR_TXREQ
}
#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, Plug-in"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
