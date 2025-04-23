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

void dm9051_dump_data(struct board_info *db, u8 *packet_data, int packet_len)
{
	int i, j, rowsize = 32;
	int splen; //index of start row
	int rlen; //remain/row length 
	char line[120];

	printk("%s\n", db->bc.head);
	for (i = 0; i < packet_len; i += rlen) {
		//rlen = print_line(packet_data+i, min(rowsize, skb->len - i)); ...
		rlen =  packet_len - i;
		if (rlen >= rowsize) rlen = rowsize;

		splen = 0;
		splen += sprintf(line + splen, " %3d", i);
		for (j = 0; j < rlen; j++) {
			if (!(j % 8)) splen += sprintf(line + splen, " ");
			if (!(j % 16)) splen += sprintf(line + splen, " ");
			splen += sprintf(line + splen, " %02x", packet_data[i+j]);
		}
		printk("%s\n", line);
	}
}

//static void dm9051_dump_reg(struct board_info *db, unsigned int reg)
//{
//	unsigned int val;

//	dm9051_get_reg(db, reg, &val);
//	printk("dm9051_get_reg(%02x)= %02x\n", reg, val);
//}

void dm9051_dump_reg2(struct board_info *db, unsigned int reg1, unsigned int reg2)
{
	unsigned int v1, v2;

	dm9051_get_reg(db, reg1, &v1);
	dm9051_get_reg(db, reg2, &v2);
	printk("dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", reg1, v1, reg2, v2);
}
void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2)
{
	unsigned int v1, v2;

	dm9051_get_reg(db, reg1, &v1);
	dm9051_get_reg(db, reg2, &v2);
	printk("%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, reg1, v1, reg2, v2);
}

void dm9051_dump_reg3(struct board_info *db, unsigned int reg1, unsigned int reg2, unsigned int reg3)
{
	unsigned int v1, v2, v3;

	dm9051_get_reg(db, reg1, &v1);
	dm9051_get_reg(db, reg2, &v2);
	dm9051_get_reg(db, reg3, &v3);
	printk("dm9051_get reg(%02x)= %02x  reg(%02x)= %02x  reg(%02x)= %02x\n", reg1, v1, reg2, v2, reg3, v3);
}
void dm9051_dump_registers(struct board_info *db)
{
	static int dumpc = 0;
	
	printk("dumpc %d\n", ++dumpc);
	dm9051_dump_reg3(db, 0x01, 0x05, 0x0a);
//	dm9051_dump_reg(db, 0x01);
//	dm9051_dump_reg(db, 0x05);
//	dm9051_dump_reg(db, 0x0a);
	
	//dm9051_dump_reg(db, 0x10);
	//dm9051_dump_reg(db, 0x11);
	//dm9051_dump_reg(db, 0x12);
	//dm9051_dump_reg(db, 0x13);
	//dm9051_dump_reg(db, 0x14);
	//dm9051_dump_reg(db, 0x15);

	dm9051_dump_reg2(db, 0x24, 0x25);
//	dm9051_dump_reg(db, 0x24);
//	dm9051_dump_reg(db, 0x25);
#if 0
	dm9051_dump_reg2(db, 0x4a, 0x53);
////	dm9051_dump_reg(db, 0x4a);
////	dm9051_dump_reg(db, 0x53);
#endif
	dm9051_dump_reg2(db, 0x74, 0x75);
//	dm9051_dump_reg(db, 0x74);
//	dm9051_dump_reg(db, 0x75);

	dm9051_dump_reg2(db, 0x7e, 0x7f);
//	dm9051_dump_reg(db, 0x7e);
//	dm9051_dump_reg(db, 0x7f);
}

#ifdef DMPLUG_INT
/*
 * Interrupt: 
 */

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
		printk("request_threaded_irq(INT_TWO_STEP)\n");
		ret = request_threaded_irq(ndev->irq, NULL, dm9051_rx_int2_delay,
									get_dts_irqf(db) | IRQF_ONESHOT,
									ndev->name, db);
		//ret = request_irq(ndev->irq, dm9051_rx_int2_delay,
		//							get_dts_irqf(db) | IRQF_ONESHOT,
		//							ndev->name, db);
		if (ret < 0)
			netdev_err(ndev, "failed to rx request irq setup\n");
	#else //INT_TWO_STEP
		printk("request_threaded_irq(INT_THREAD)\n");
		ret = request_threaded_irq(ndev->irq, NULL, /*dm9051_rx_threaded_plat*/ /*dm9051_rx_int2_delay*/ dm9051_rx_threaded_plat,
		 						   get_dts_irqf(db) | IRQF_ONESHOT,
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
#endif

#ifndef DMPLUG_INT
/*
 * Polling: 
 */

void INIT_RX_POLL_DELAY_SETUP(struct board_info *db)
{
	/* schedule delay work */
	INIT_DELAYED_WORK(&db->irq_workp, dm9051_poll_servicep); //.dm9051_poll_delay_plat()
}

void INIT_RX_POLL_SCHED_DELAY(struct board_info *db)
{
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
}
#endif

int DM9051_OPEN_REQUEST(struct board_info *db)
{
	#ifdef DMPLUG_INT
	/* interrupt work */
	return INIT_REQUEST_IRQ(db->ndev);
	#else
	/* Or schedule delay work */
	INIT_RX_POLL_SCHED_DELAY(db);
	return 0;
	#endif
}

void SHOW_MODE(struct spi_device *spi) //SHOW_INT_MODE
{
	#ifdef DMPLUG_INT
		unsigned int intdata[2];
		of_property_read_u32_array(spi->dev.of_node, "interrupts", &intdata[0], 2);
		dev_info(&spi->dev, "Davicom: %s(%d)", dmplug_intterrpt_des, dmplug_interrupt);
		dev_info(&spi->dev, "Operation: Interrupt pin: %d\n", intdata[0]); // intpin
		dev_info(&spi->dev, "Operation: Interrupt trig type: %d\n", intdata[1]);
		#ifdef INT_TWO_STEP
			dev_info(&spi->dev, "Interrupt: Two_step\n");
		#endif
	#else
		dev_info(&spi->dev, "Davicom: %s(%d)", dmplug_intterrpt_des, dmplug_interrupt);
		//int i;
		//dev_info(&spi->dev, "Operation: Polling operate count %d\n", csched.nTargetMaxNum);
		//for (i = 0; i < csched.nTargetMaxNum; i++)
		//{
		//	dev_info(&spi->dev, "Operation: Polling operate delayF[%d]= %lu\n", i, csched.delayF[i]);
		//}
	#endif
}

MODULE_DESCRIPTION("Davicom DM9051 network SPI driver"); //MODULE_DESCRIPTION("Davicom DM9051A Open");
MODULE_LICENSE("GPL");
