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

//#define TOGG_INTVL	1
//#define TOGG_TOT_SHOW	5
#define NUM_TRIGGER			25
#define	NUM_BMSR_DOWN_SHOW		5

#ifdef DMCONF_BMCR_WR
char *get_log_addr(struct board_info *db)
{
	if (!db->automdix_log[0][0]) {
		//db->automdix_log[0][0] = 1;
 //("[get log addr 0]\n");
		return &db->automdix_log[0][0]; //&db->automdix_log[0][1];
	}
	if (!db->automdix_log[1][0]) {
		//db->automdix_log[1][0] = 1;
 //("[get log addr 1]\n");
		return &db->automdix_log[1][0]; //&db->automdix_log[1][1];
	}
	if (!db->automdix_log[2][0]) {
		//db->automdix_log[2][0] = 1;
 //("[get log addr 2]\n");
		return &db->automdix_log[2][0]; //&db->automdix_log[2][1];
	}

 //("[get log addr 2ov]\n");
	db->automdix_log[0][0] = db->automdix_log[1][0];
	strcpy(&db->automdix_log[0][1], &db->automdix_log[1][1]);
	db->automdix_log[1][0] = db->automdix_log[2][0];
	strcpy(&db->automdix_log[1][1], &db->automdix_log[2][1]);
	
	return &db->automdix_log[2][0];
}
static void show_log_data(char *head, char *data, struct board_info *db)
{
	//struct device *dev1 = &db->spidev->dev; //= &spi->dev;
	//struct device *dev2 = db->ndev->dev.parent; //= ndev->dev.parent;

	netif_crit(db, link, db->ndev, "<%s> %s\n", head, &data[1]);
}
static void show_log_addr(char *head, struct board_info *db)
{
	if (db->automdix_log[0][0]) {
		printk("\n");
		show_log_data(head, &db->automdix_log[0][0], db); //("<%s> %s\n", head, &db->automdix_log[0][1]);
	}
	if (db->automdix_log[1][0])
		show_log_data(head, &db->automdix_log[1][0], db); //("<%s> %s\n", head, &db->automdix_log[1][1]);
	if (db->automdix_log[2][0])
		show_log_data(head, &db->automdix_log[2][0], db); //("<%s> %s\n", head, &db->automdix_log[2][1]);
}
void amdix_log_reset(struct board_info *db)
{
	db->n_automdix = 0; //log-reset
	db->automdix_log[0][0] = 0;
	db->automdix_log[1][0] = 0;
	db->automdix_log[2][0] = 0;

	db->stop_automdix_flag = 0;
}
#if 0
//void amdix_link_change_up(struct board_info *db, unsigned int bmsr)
//{
//	if (!(bmsr & BIT(2)) && (db->bmsr & BIT(2))) {
//		//[show]
//		show_log_addr("hist", db);
//		//[message]
//		k("<link_phylib. on %02u to %02u>, current lpa %04x [bmsr] %04x to %04x found reach link\n", db->_stop_automdix_flag,
//			db->n_automdix, db->lpa, bmsr, db->bmsr);
//		//[clear]
//		k("[link] clear log...");
//		amdix_log_reset(db);
//	}
//}
#endif

void amdix_bmsr_change_up(struct board_info *db, unsigned int bmsr)
{
	//struct device *dev1 = &db->spidev->dev; //= &spi->dev;
	//struct device *dev2 = db->ndev->dev.parent; //= ndev->dev.parent;

	if (!(bmsr & BIT(2))) {
		show_log_addr("hist", db);
		//dev_crit(dev1, "<link_phylib. on %02u to %02u>, current lpa %04x [bmsr] %04x to %04x found reach link\n", db->stop_automdix_flag,
		//	db->n_automdix, db->lpa, bmsr, db->bmsr);
		netif_crit(db, link, db->ndev, "<phylib BMSR> lpa %04x [bmsr] %04x\n",
			db->lpa, db->bmsr);
		netif_crit(db, link, db->ndev, "[link] clear log...");
		amdix_log_reset(db);
	}
}

static int amdix_bmsr_change(struct board_info *db)
{
	static unsigned int bmsr = 0x0000; //0xffff;

	/* bmsr (change linkup) */
	if (db->bmsr & BIT(2)) {
		amdix_bmsr_change_up(db, bmsr);
	}

	/* bmsr (change) */
	if (db->bmsr != bmsr) {
		if (db->bmsr & BIT(2))
			;
		else {
			//netif_warn(db, link, db->ndev, "??? [NO write obsevation]\n");
			/* NO write obsevation */
			//netif_warn(db, link, db->ndev, "??? <may up to down> %02u _First quicking an enter amdix ... lpa %04x bmsr from %04x to %04x\n",
			//		db->n_automdix, db->lpa, bmsr, db->bmsr);

#if 0
			/* NO write obsevation */
			/* ------------------- */
			int ret;
			db->mdi ^= 0x0020;
			netif_warn(db, link, db->ndev, "??? <may up to down> %02u [lpa] %04x _dm9051_phywr[_AutoMDIX_] reg %d [val %04x]",
					db->n_automdix, db->lpa, 20, db->mdi); 
			
//			ret = dm9051_phywrite(db, 20, db->mdi);
			if (ret)
				netif_warn(db, link, db->ndev, "warn: phywrite reg %d [val %04x], fail\n", 20, db->mdi);
#endif
		}

		bmsr = db->bmsr;
		return 1;
	}
	return 0;
}

static int dm9051_phyread_bmsr_wr(struct board_info *db, unsigned int reg, unsigned int *val) //reg, not used!
{
	/* overlay implement */
	int ret = dm9051_phyread(db, MII_LPA, val);
	if (ret)
		return ret;
	db->lpa = *val;

	ret = dm9051_phyread(db, MII_BMSR, val);
	if (ret)
		return ret;
	db->bmsr = *val;

	/* nt log */
	#if 1
	//do
	//{
		if (!amdix_bmsr_change(db))
		{
			if (!(db->bmsr & BIT(2))) {

				do {
					if (db->stop_automdix_flag) {
					#if 1
						/* lpa changeto 0x0000 */
						if (!db->lpa) {
							db->stop_automdix_flag = 0;
							break;
						}
					#endif
						if (!(db->n_automdix %10)) {
							netif_warn(db, link, db->ndev, "%2u [lpa %04x]\n", db->n_automdix, db->lpa);
							break;
						}
						return 0;
					}
					if (db->lpa) {
						netif_warn(db, link, db->ndev, "<fund_phylib. on %02u to %02u, rd.bmsr %04x [lpa] %04x> STOPPING... amdix\n", db->stop_automdix_flag,
							db->n_automdix, db->bmsr, db->lpa);
						db->stop_automdix_flag = db->n_automdix;
						//k("(STOP avoid below possible more once toggle...)\n");
						return 0; //break; //(STOP avoid below possible more once toggle...)
					}
				} while(0);

				db->n_automdix++;
				do {
			#if 1
					char *p = get_log_addr(db);
					db->mdi ^= 0x0020;

					do {
						/* store list */
						//sprintf(p, "from_phylib. %02u [lpa] %04x _mon_phywr[_AutoMDIX_] reg %d [val %04x]",
						//		db->n_automdix, db->lpa, 20, db->mdi);
						//snprintf(p, sizeof(automdix_log[0]), "from_phylib. %02u [lpa] %04x _mon_phywr[_AutoMDIX_] reg %d [val %04x]",
						//		db->n_automdix, db->lpa, 20, db->mdi);
						p[0] = db->n_automdix;
						memset(&p[1], 0, AMDIX_LOG_BUFSIZE);
						snprintf(&p[1], AMDIX_LOG_BUFSIZE-1, "in_case phylib. %02u [lpa] %04x phy[AutoMDIX] reg %d [val %04x]",
								db->n_automdix, db->lpa, 20, db->mdi);
					} while(0);

					/* NO write obsevation */
					//ret = dm9051_phywrite(db, 20, db->mdi);
					//if (ret)
					//	return ret;
			#endif

					if ((db->n_automdix % NUM_TRIGGER) <= NUM_BMSR_DOWN_SHOW) {
						if ((db->n_automdix % NUM_TRIGGER) == 1) //only first.
							printk("\n"); //printk("(re-cycle)(first).mdix\n");
//						show_log_data("bmsr down", p);
					}

			#if 1
					/* phy reset insteaded */
					if (!(db->n_automdix % NUM_TRIGGER)) {
						netif_warn(db, link, db->ndev, "( bmsr down per %d).phy reset insteaded: %u\n", NUM_TRIGGER, db->n_automdix);
						ret = dm9051_phy_reset(db);
						if (ret)
							return ret;
					}
			#endif
				} while(0);
			}
		}
	//} while (0);
	#endif

	return ret;	
}

int dm9051_phyread_nt_bmsr(struct board_info *db, unsigned int reg, unsigned int *val)
{
	if (reg == MII_BMSR)
		return dm9051_phyread_bmsr_wr(db, MII_BMSR, val);

	return dm9051_phyread(db, reg, val);		
}

// dm9051_phyread.EXTEND
#if 0
//int dm9051_phyread_log_bmsr(struct board_info *db, unsigned int *val)
//{
//	int ret;

//	ret = dm9051_phyread(db, MII_LPA, val);
//	if (ret)
//		return ret;
//	db->lpa = *val;

//	ret = dm9051_phyread(db, MII_BMSR, val);
//	if (ret)
//		return ret;
//	db->bmsr = *val;

//	/* check log */
//	do
//	{
//		if (!amdix_link_change(db))
//		{
//			if (!(db->bmsr & BIT(2))) {
//				//static unsigned int n_automdix = 0;
//				//static unsigned int mdi = 0x0830;
//				db->n_automdix++;
//				
//				if (db->_stop_automdix_flag) {
//#if 1
//					k("[lpa %04x]\n", db->lpa);
//					break;
//#endif
//				}

//				//ret = dm9051_phyread(db, 5, &vval);
//				//if (ret)
//				//	return ret;
//				
//				if (db->lpa) {
//					k("<fund_phylib. on %02u to %02u, _mdio_read.bmsr[lpa] %04x> STOPPING... automdix\n", db->_stop_automdix_flag,
//						db->n_automdix, db->lpa);
//					db->_stop_automdix_flag = db->n_automdix;
//#if 1
//					k("(STOP avoid below possible more once toggle...)\n");
//					break; //(STOP avoid below possible more once toggle...)
//#endif
//				}

//				if (!(db->n_automdix % TOGG_INTVL)) {
//					char *p;
//					db->mdi ^= 0x0020;

//					if (db->n_automdix <= TOGG_TOT_SHOW) {
//						if (db->n_automdix == TOGG_INTVL) //only first.
//							k("\n");
//					}
//					if (db->n_automdix <= TOGG_TOT_SHOW 
//						&& !(db->bmsr & BIT(6))) k("_mdio_read.bmsr.BIT6= 0, !MF_Preamble, phyaddr %d [BMSR] %04x\n", DM9051_PHY_ADDR, db->bmsr);

//					ret = dm9051_phywrite(db, 20, db->mdi);
//					if (ret)
//						return ret;

//					/* store to showlist */
//					p = get_log_addr(db);
//					sprintf(&p[1], "from_phylib. %02u [lpa] %04x _dm9051_phywr[_AutoMDIX_] reg %d [val %04x]",
//							db->n_automdix, db->lpa, 20, db->mdi); //= set_log_addr(db, p, ...);
//				}
//				break;
//			}
//		}
//	} while (0);

//	return ret;
//}
#endif //0
#endif
