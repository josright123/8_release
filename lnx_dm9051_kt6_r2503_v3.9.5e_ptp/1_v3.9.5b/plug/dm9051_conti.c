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
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>
#include "dm9051.h"

extern const struct param_config *param;

/*
 * Conti:
 */
#ifdef DMPLUG_CONTI
void tx_continue_ver(struct board_info *db)
{
	dev_info(&db->spidev->dev, "DMPLUG CONTI Version\n");
}

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
	if (ret < 0) {
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
	for (;;) {
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

int TX_MOTE2_CONTI_RCR(struct board_info *db)
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
static int TX_OPS_CONTI(struct board_info *db, struct sk_buff *skb, u64 tx_timeout_us)
{
	//u8 *buff = skb->data;
	//unsigned int len = skb->len;
	u8 th[4];
	int ret;

	dm9051_create_tx_head(th, skb->len);

	do {
		const unsigned int tx_xxhead = 4;
		unsigned int tx_xxbst = ((skb->len + 3) / 4) * 4;

		if (!tx_free_poll_timeout(db, tx_xxhead + tx_xxbst, 1, tx_timeout_us)) { // 2100 <- 20
			ret = -ENOMEM; //-ETIMEDOUT or
			break;
		}

		ret = dm9051_write_mem(db, DM_SPI_MWCMD, th, 4);
		if (ret)
			break;

		ret = dm9051_write_mem_cache(db, skb->data, tx_xxbst); //'tx_xxbst'
	} while (0);

	return ret;
}

int TX_MODE2_CONTI_TCR(struct board_info *db, struct sk_buff *skb, u64 tx_timeout_us)
{
	int ret;

	do {
		ret = TX_OPS_CONTI(db, skb, tx_timeout_us); //skb->data, data_len); //'double_wb'
		if (ret)
			break;

		ret = dm9051_req_tx(db);
	} while (0);

	if (!ret) {
		struct net_device *ndev = db->ndev;
		ndev->stats.tx_bytes += skb->len;
		ndev->stats.tx_packets++;
	}

	return ret;
}


int dm9051_single_tx_conti(struct board_info *db, struct sk_buff *skb)
{
	int ret = TX_MODE2_CONTI_TCR(db, skb, param->tx_timeout_us);

	dev_kfree_skb(skb); //skb from TX_PAD() to dev_kfree_skb(), MUST free the updatest skb.
	return ret;
}

#endif

MODULE_DESCRIPTION("Davicom DM9051 driver, TX conti plug-in"); //MODULE_DESCRIPTION("Davicom DM9051A 1588 driver");
MODULE_LICENSE("GPL");
