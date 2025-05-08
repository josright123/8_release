// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
 
/*
 * User notice:
 *   To add encryption function, 
 *   Add 
 * 	#define PLUG_CUSTOMIZE_CRYP
 *      in dm9051_plug.h
 *   And add this file to project
 *   And modify Makefile to insert this file 
 *   into the project build.
 * 
 * In Makefile
 * Change
 *   dm9051a-objs := dm9051.o dm9051_plug.o
 * to
 *   dm9051a-objs := dm9051.o dm9051_plug.o dm9051_crypt.o
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

#warning "DMPLUG: dm9051 plug-in encrypt bus work"

/*
 * Encrypt Protection Driver version: 
 */

#define	FORCE_BUS_ENCPT_OFF		0
#define	FORCE_BUS_ENCPT_CUST_ON		1

#define ENCPT_MODE                      FORCE_BUS_ENCPT_CUST_ON
#define FORCE_BUS_ENCPT_FIX_KEY		0x95 //for fix selected

#define DM9051_BUS_WORK(exp, yhndlr) 	\
	do                                  \
	{                                   \
		if ((exp))                      \
		{                               \
			yhndlr;                     \
		}                               \
	} while (0)

static int dm9051_setup_bus_work(struct board_info *db)
{
	int ret;
	unsigned int crypt_1, crypt_2, key;

	//db->rctl.bus_word = FORCE_BUS_ENCPT_FIX_KEY;
	db->rctl.bus_word = 0;
	ret = dm9051_get_reg(db, DM9051_PIDL, &crypt_1);
	if (ret)
		return ret;
	ret = dm9051_get_reg(db, DM9051_PIDH, &crypt_2);
	if (ret)
		return ret;
	ret = dm9051_set_reg(db, 0x49, crypt_1);
	if (ret)
		return ret;
	ret = dm9051_set_reg(db, 0x49, crypt_2);
	if (ret)
		return ret;
	ret = dm9051_get_reg(db, 0x49, &key);
	if (ret)
		return ret;
	//dev_info(&db->spidev->dev, "[bus word]= on, word 0x%02x\n", key & 0xff);
	db->rctl.bus_word = (u8)(key & 0xff);
	return 0;
}

int BUS_SETUP(struct board_info *db)
{
	return dm9051_setup_bus_work(db);
}

static void bus_work(u8 bus_word, u8 *buff, unsigned int crlen)
{
	unsigned int j;
	for (j = 0; j < crlen; j++)
	{
		buff[j] ^= bus_word;
	}
}

void BUS_OPS(struct board_info *db, u8 *buff, unsigned int crlen)
{
	DM9051_BUS_WORK(ENCPT_MODE && db->rctl.bus_word, bus_work(db->rctl.bus_word,buff,crlen));
}

MODULE_DESCRIPTION("Davicom DM9051 driver, Crypt-in");
MODULE_LICENSE("GPL");
