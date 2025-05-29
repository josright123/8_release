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
#define MAIN_DATA
#include "dm9051.h"

const struct plat_cnf_info *plat_cnf = &plat_align_mode; /* Driver configuration */

/* Tx 'wb' do skb protect */
#define DM9051_SKB_PROTECT
#define STICK_SKB_CHG_NOTE

#define DM9051_INTR_BACKUP // #ifdef DM9051_INTR_BACKUP .. #endif //instead more backup.
#define DM9051_NORM_BACKUP_TX // 

int get_dts_irqf(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

static unsigned int dm9051_init_intcr_value(struct board_info *db)
{
	return (get_dts_irqf(db) == IRQF_TRIGGER_LOW || get_dts_irqf(db) == IRQF_TRIGGER_FALLING) ? INTCR_POL_LOW : INTCR_POL_HIGH;
}

/*
 * log: 
 */
void USER_CONFIG(struct device *dev, struct board_info *db, char *str)
{
	if (dev)
		dev_warn(dev, "%s\n", str);
	else if (db)
		netif_info(db, drv, db->ndev, "%s\n", str);
}

static void SHOW_ALL_USER_CONFIG(struct device *dev, struct board_info *db)
{
	INFO_CPU_BITS(dev, db);
	INFO_CPU_MIS_CONF(dev, db);

	INFO_INT(dev, db);
	INFO_INT_CLKOUT(dev, db);
	INFO_INT_TWOSTEP(dev, db);
	INFO_BMCR_WR(dev, db);
	INFO_MRR_WR(dev, db);
	INFO_CONTI(dev, db);
	INFO_PTP(dev, db);
	INFO_PPS(dev, db);
}

static int SHOW_MAP_CHIPID(struct device *dev, unsigned short wid)
{
	if (wid != DM9051_ID)
	{
		dev_err(dev, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_warn(dev, "probe %04x found\n", wid);
	return 0;
}

static void on_core_reset_show(struct board_info *db)
{
	netif_crit(db, hw, db->ndev, "dm9051.on.(all_start(open), all_upstart(link_chg), all_restart(err_fnd))\n");
}

static void SHOW_OPEN(struct board_info *db)
{
	printk("\n");
	netif_warn(db, drv, db->ndev, "dm9051_open\n");
	SHOW_ALL_USER_CONFIG(NULL, db);
	/* amdix_log_reset(db); */ //(to be determined)
}

static void SHOW_RESTART_SHOW_STATIISTIC(struct board_info *db)
{
	netif_warn(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	netif_crit(db, rx_status, db->ndev, "List: rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	   db->bc.status_err_counter + db->bc.large_err_counter,
	   db->bc.fifo_rst_counter);
	//netif_err(db, rx_status, db->ndev, "_[_all_restart] rxb work around done\n");
}

static void SHOW_XMIT_ANALYSIS(struct board_info *db)
{
	netif_info(db, tx_done, db->ndev, "%6d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in,
		db->xmit_in, db->xmit_tc, db->xmit_zc);
	netif_info(db, tx_done, db->ndev, "%6d [_THrd-in] on-THrd-in %u Pkt %u\n", db->xmit_thrd0,
		db->xmit_thrd0, db->xmit_ttc0);
	netif_info(db, tx_done, db->ndev, "%6d [_THrd-end] on-THrd-end %u Pkt %u\n", db->xmit_thrd,
		db->xmit_thrd, db->xmit_ttc);
}

static void SHOW_RX_CTRLS(struct board_info *db)
{
	unsigned int v1, v2;

	memset(db->bc.head, 0, HEAD_LOG_BUFSIZE);
	//.dm9051_headlog_regs("dump rcr registers:", db, DM9051_RCR, DM9051_RCR);
	//.dm9051_headlog_regs("dump wdr registers:", db, 0x24, 0x25);
	//.dm9051_headlog_regs("dump mrr registers:", db, DM9051_MRRL, DM9051_MRRH);
	//unsigned int reg1, reg2;
	//reg1 = DM9051_RCR; reg2 = DM9051_RCR;
	dm9051_get_reg(db, DM9051_RCR, &v1); dm9051_get_reg(db, DM9051_RCR, &v2);
	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump rcr registers:");
	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, DM9051_RCR, v1, DM9051_RCR, v2);
	//reg1 = 0x24; reg2 = 0x25;
	dm9051_get_reg(db, 0x24, &v1); dm9051_get_reg(db, 0x25, &v2);
	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump wdr registers:");
	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, 0x24, v1, 0x25, v2);
	//reg1 = DM9051_MRRL; reg2 = DM9051_MRRH;
	dm9051_get_reg(db, DM9051_MRRL, &v1); dm9051_get_reg(db, DM9051_MRRH, &v2);
	snprintf(db->bc.head, HEAD_LOG_BUFSIZE - 1, "dump mrr registers:");
	netif_info(db, rx_status, db->ndev, "%s dm9051_get reg(%02x)= %02x  reg(%02x)= %02x\n", db->bc.head, DM9051_MRRL, v1, DM9051_MRRH, v2);
}

unsigned int SHOW_BMSR(struct board_info *db)
{
	unsigned int val;

	/*.dm9051_phyread_headlog("bmsr", db, MII_BMSR);*/
	dm9051_phyread(db, MII_BMSR, &val);
	netif_warn(db, link, db->ndev, "bmsr %04x\n", val);
	return val;
}

int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb)
{
	int ret;

	ret = regmap_read(db->regmap_dm, reg, prb);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d get reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val)
{
	int ret;

	ret = regmap_write(db->regmap_dm, reg, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d set reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_update_bits(struct board_info *db, unsigned int reg, unsigned int mask,
							  unsigned int val)
{
	int ret;

	ret = regmap_update_bits(db->regmap_dm, reg, mask, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d update bits reg %02x\n",
				  __func__, ret, reg);
	return ret;
}

static inline int dm9051_stop_mrcmd(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_ISR, ISR_STOP_MRCMD); /* to stop mrcmd */
}

/* skb buffer exhausted, just discard the received data
 */
static int dm9051_dumpblk(struct board_info *db, u8 reg, size_t count)
{
	struct net_device *ndev = db->ndev;
	unsigned int rb;
	int ret;

	/* no skb buffer,
	 * both reg and &rb must be noinc,
	 * read once one byte via regmap_read
	 */
	do
	{
		ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
		if (ret < 0)
		{
			netif_err(db, drv, ndev, "%s: error %d dumping read reg %02x\n",
					  __func__, ret, reg);
			break;
		}
	} while (--count);

	return ret;
}

static int dm9051_set_regs(struct board_info *db, unsigned int reg, const void *val,
						   size_t val_count)
{
	int ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk writing regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_get_regs(struct board_info *db, unsigned int reg, void *val,
						   size_t val_count)
{
	int ret;

	ret = regmap_bulk_read(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk reading regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff,
			size_t len)
{
	int ret;

	if (plat_cnf->align.burst_mode)
	{ // tx
		ret = regmap_noinc_write(db->regmap_dm, reg, buff, len);
	}
	else
	{
		const u8 *p = (const u8 *)buff;
		u32 BLKTX = plat_cnf->align.tx_blk;
		while (len >= BLKTX)
		{
			ret = regmap_noinc_write(db->regmap_dm, reg, p, BLKTX);
			p += BLKTX;
			len -= BLKTX;
			if (ret < 0)
			{
				netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x len %u\n",
					__func__, ret, reg, BLKTX);
				return ret;
			}
		}
		while (len--)
		{
			unsigned int val = (unsigned int)*p++;
			ret = regmap_write(db->regmap_dm, reg, val);
			if (ret < 0)
				break;
		}
	}
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen)
{
	BUS_OPS(db, buff, crlen);
	return dm9051_write_mem(db, DM_SPI_MWCMD, buff, crlen); //'!wb'
}

int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
						   size_t len)
{
	int ret;

	if (plat_cnf->align.burst_mode)
	{ // rx
		ret = regmap_noinc_read(db->regmap_dm, reg, buff, len);
	}
	else
	{
		u8 *p = buff;
		unsigned int rb;
		u32 BLKRX = plat_cnf->align.rx_blk;
		while (len >= BLKRX)
		{
			ret = regmap_noinc_read(db->regmap_dm, reg, p, BLKRX);
			if (ret < 0)
			{
				netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x len %u\n",
					__func__, ret, reg, BLKRX);
				return ret;
			}
			p += BLKRX;
			len -= BLKRX;
		}

		while (len--)
		{
			ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
			*p++ = (u8)SCAN_BL(rb);
			if (ret < 0)
				break;
		}
	}
	if (ret < 0)
	{
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
				  __func__, ret, reg);
		return ret;
	}

	return dm9051_stop_mrcmd(db);
}

int dm9051_read_mem_cache(struct board_info *db, unsigned int reg, u8 *buff,
								 size_t crlen)
{
	int ret = dm9051_read_mem(db, reg, buff, crlen);
	if (ret == 0)
		BUS_OPS(db, buff, crlen);
	return ret;
}

int dm9051_read_mem_rxb(struct board_info *db, unsigned int reg, void *buff,
						size_t len)
{
	int ret;
	u8 *p = buff;
	unsigned int rb;

	while (len--)
	{
		ret = regmap_read(db->regmap_dm, reg, &rb); // quick direct
		*p++ = (u8)SCAN_BL(rb);
		if (ret < 0)
			break;
	}

	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
				  __func__, ret, reg);
	return ret;
}

static int dm9051_ncr_reset(struct board_info *db)
{
	int ret;

	printk("_ncr_reset\n");

	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;

	dm9051_ncr_poll(db);

	return 0;
}

int dm9051_ncr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
								   !(mval & NCR_RST), 10, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for ncr reset\n");
	return ret;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
int dm9051_nsr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
								   mval & (NSR_TX2END | NSR_TX1END),
								   1, param->tx_timeout_us); // 2100 <- 20
	if (param->force_monitor_tx_timeout && ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for tx end\n");
	return ret;
}

static int dm9051_epcr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_EPCR, mval,
								   !(mval & EPCR_ERRE), 100, 10000);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");
	return ret;
}

static int dm9051_set_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_set_reg(db, DM9051_FCR, fcr); // flow control / back pressure
}

static int dm9051_set_rcr(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all);
}

static int dm9051_set_recv(struct board_info *db)
{
	int ret;

	ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
	if (ret)
		return ret;

	return SET_RCR(db); /* enable rx */
}

/* Set DM9051_IPCOCR in case of int clkout
 * DUTY_LEN 1 is for 40.96 us
 */
int dm9051_int_clkout(struct board_info *db)
{
	netif_info(db, intr, db->ndev, "_reset [_core_reset] set DM9051_IPCOCR %02lx\n", IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
	return regmap_write(db->regmap_dm, DM9051_IPCOCR, IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
}

static int dm9051_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		//fcr |= FCR_BKPA | FCR_FLCE; (optional)
		fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_update_bits(db, DM9051_FCR, FCR_RXTX_BITS, fcr);
}

static int dm9051_disable_interrupt(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_IMR, IMR_PAR); /* disable int */
}

static int dm9051_enable_interrupt(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_IMR, db->imr_all); /* enable int */
}

//.inline dm9051_stop_mrcmd
static int dm9051_clear_interrupt(struct board_info *db)
{
	return dm9051_update_bits(db, DM9051_ISR, ISR_CLR_INT, ISR_CLR_INT);
}

static int dm9051_eeprom_read(struct board_info *db, int offset, u8 *to)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret) {
		netif_info(db, timer, db->ndev, "timeout of dm9051_eeprom_write %d %04x\n", offset, to[0] | to[1] << 8);
		return ret;
	}

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, to, 2);
}

static int dm9051_eeprom_write(struct board_info *db, int offset, u8 *data)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, data, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret) {
		netif_info(db, timer, db->ndev, "timeout of dm9051_eeprom_write %d %04x\n", offset, data[0] | data[1] << 8);
		return ret;
	}

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret) {
		netif_info(db, timer, db->ndev, "timeout of dm9051_phyrd %d %04x\n", reg, *val);
		return ret;
	}

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
	 * only fill lower 2 bytes
	 */
	*val = 0;
	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, val, 2);
}

static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, &val, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll(db);
	if (ret) {
		netif_info(db, timer, db->ndev, "timeout of dm9051 phy write %d %04x\n", reg, val);
		return ret;
	}

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

static int dm9051_phy_reset(struct board_info *db)
{
	int ret;

	/* PHY reset */
	printk("_phy_reset: [internal] mdio phywr %d %04x\n", 0, 0x8000); //netif_info(db, link, db->ndev, ..

	ret = dm9051_phywrite(db, 0, 0x8000);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	/* Jabber function disabled refer to bench test
	 * meeting advice 20250226
	 */
	//printk("_phy_write: [internal] mdio phywr %d %04x for jabber disable\n", 18, 0x7000);
	//ret = dm9051_phywrite(db, 18, 0x7000);
	//if (ret)
	//	return ret;
	return 0;
}

static int dm9051_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	unsigned int val;

	if (addr == DM9051_PHY_ADDR)
	{
		int ret;

		#if MI_FIX
		mutex_lock(&db->spi_lockm);
		#endif

		ret = PHY_READ(db, regnum, &val);

		#if MI_FIX
		mutex_unlock(&db->spi_lockm);
		#endif

		if (ret)
			return ret;
		return val;
	}

	return 0xffff;
}

static int dm9051_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;

	if (addr == DM9051_PHY_ADDR)
	{
		static int mdio_write_count = 0;
		int ret;

		#if MI_FIX
		if (regnum == 0x0d || regnum == 0x0e) //unknown of dm9051a
			return 0;

		mutex_lock(&db->spi_lockm);
		#endif

		do {
			/* [dbg] Wr BMCR to power-down */
			if ((regnum == 0) && (val & BIT(11))) { //BIT(11) = 0x800
				netif_crit(db, link, db->ndev, "[mdio phywr] %d %04x: power down (warn)\n", regnum, val);
				break;
			}

			/* [dbg] Wr BMCR of first 9 wr */
			if (mdio_write_count <= 9)
				netif_info(db, link, db->ndev, "[count%d] mdio phywr %d %04x\n", mdio_write_count++, regnum, val);
		} while(0);
		ret = dm9051_phywrite(db, regnum, val);

		#if MI_FIX
		mutex_unlock(&db->spi_lockm);
		#endif

		return ret;
	}

	return -ENODEV;
}

/* Functions: Core init
 */
static int dm9051_core_reset(struct board_info *db)
{
	int ret = BUS_SETUP(db); /* customization */
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, (plat_cnf->skb_wb_mode) ? MBNDRY_WORD : MBNDRY_BYTE); /* MemBound */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_ADVCOUNT); /* Pause Count */
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, 0x31, db->csum_gen_val); /* Checksum Offload */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, 0x32, db->csum_rcv_val); /* Checksum Offload */
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	/* Diagnostic contribute: In dm9051_enable_interrupt()
	 * (or located in the core reset subroutine is better!!)
	 */
	ret = INT_CLOCK(db); /* clock out */
	if (ret)
		return ret;

	on_core_reset_show(db); /* core init (all_start(open), all_upstart(link_chg), all_restart(err_fnd)) -open ptpc */
	DMPLUG_PTP_AT_RATE(db);

	return ret; /* ~return dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db)) */
}

static void dm9051_reg_lock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_lock(&db->reg_mutex);
}

static void dm9051_reg_unlock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_unlock(&db->reg_mutex);
}

static struct regmap_config regconfigdm = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
};

static struct regmap_config regconfigdmbulk = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
	.use_single_read = true,
	.use_single_write = true,
};

/* Work around for dm9051.c as dynamic load driver
 */
#define KERNEL_ROADMAP_CONF_H_
#ifdef KERNEL_ROADMAP_CONF_H_
#undef __devm_regmap_init_spi // compiler not undefined, since used in _wrapper

static int regmap_spi_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write(spi, data, count);
}
static int regmap_spi_gather_write(void *context,
								   const void *reg, size_t reg_len,
								   const void *val, size_t val_len)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = reg,
			.len = reg_len,
		},
		{
			.tx_buf = val,
			.len = val_len,
		},
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	return spi_sync(spi, &m);
}
static int regmap_spi_async_write(void *context, //.V510_COMPLEX
								  const void *reg, size_t reg_len,
								  const void *val, size_t val_len,
								  struct regmap_async *a)
{
	struct device *dev = context;
	dev_warn(dev, "NOT SUPPORT: regmap_spi_async_write(context,...)\n");
	return -1;
}
static struct regmap_async *regmap_spi_async_alloc(void)
{ //.V510_COMPLEX
	//struct device *dev = context;
	//dev_warn(dev, "NOT SUPPORT: regmap_spi_async_alloc(void)\n");
	return NULL;
}
static int regmap_spi_read(void *context,
						   const void *reg, size_t reg_size,
						   void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write_then_read(spi, reg, reg_size, val, val_size);
}

static const struct regmap_bus regmap_bus_dm = {
	// to refer to LXR
	.write = regmap_spi_write,
	.gather_write = regmap_spi_gather_write,
	.async_write = regmap_spi_async_write, //.V510_COMPLEX
	.async_alloc = regmap_spi_async_alloc, //.V510_COMPLEX
	.read = regmap_spi_read,
	.read_flag_mask = 0x80,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_bus *regmap_get_spi_bus(struct spi_device *spi,
												   const struct regmap_config *config)
{
	return &regmap_bus_dm;
}

struct regmap *__devm_regmap_init_spi_dm(struct spi_device *spi,
										 const struct regmap_config *config,
										 struct lock_class_key *lock_key,
										 const char *lock_name)
{
	// const struct regmap_bus *bus = &regmap_bus_dm;
	const struct regmap_bus *bus = regmap_get_spi_bus(spi, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus); /* . */

	return __devm_regmap_init(&spi->dev, bus, &spi->dev, config, lock_key, lock_name);
}

#undef devm_regmap_init_spi
#define devm_regmap_init_spi(dev, config)                        \
	__regmap_lockdep_wrapper(__devm_regmap_init_spi_dm, #config, \
							 dev, config)
#endif

static int dm9051_map_init(struct spi_device *spi, struct board_info *db)
{
	/* create two regmap instances,
	 * split read/write and bulk_read/bulk_write to individual regmap
	 * to resolve regmap execution confliction problem
	 */
	regconfigdm.lock_arg = db;
	db->regmap_dm = devm_regmap_init_spi(db->spidev, &regconfigdm);
	if (IS_ERR(db->regmap_dm))
		return PTR_ERR(db->regmap_dm);

	regconfigdmbulk.lock_arg = db;
	db->regmap_dmbulk = devm_regmap_init_spi(db->spidev, &regconfigdmbulk);
	return PTR_ERR_OR_ZERO(db->regmap_dmbulk);
}

static int dm9051_map_chipid(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	unsigned short wid;
	u8 buff[6];
	int ret;

	printk("\n");
	SHOW_PLAT_MODE(dev);

	ret = dm9051_get_regs(db, DM9051_VIDL, buff, sizeof(buff));
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	return SHOW_MAP_CHIPID(dev, wid);
}

static void dm90951_get_random(struct net_device *ndev, u8 *addr)
{
		eth_hw_addr_random(ndev);
		ether_addr_copy(addr, ndev->dev_addr);
		addr[0] = 0x00;
		addr[1] = 0x60;
		addr[2] = 0x6e;
}

/* Read DM9051_PAR registers which is the mac address loaded from EEPROM while power-on
 */
static int dm9051_map_etherdev_par(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	ret = dm9051_get_regs(db, DM9051_PAR, addr, sizeof(addr));
	if (ret < 0)
		return ret;

	if (!is_valid_ether_addr(addr))
	{
		dm90951_get_random(ndev, addr);

		ret = dm9051_set_regs(db, DM9051_PAR, addr, sizeof(addr));
		if (ret < 0)
			return ret;

		dev_info(&db->spidev->dev, "Use random MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
	eth_hw_addr_set(ndev, addr);
#else
	ether_addr_copy(ndev->dev_addr, addr);
#endif
	SHOW_MAC(db, addr);
	return 0;
}

/* ethtool-ops
 */
static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRVNAME_9051, sizeof(info->driver));
}

static void dm9051_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->msg_enable = value;
}

static u32 dm9051_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	return db->msg_enable;
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
	return 128;
}

static int dm9051_get_eeprom(struct net_device *ndev,
							 struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2)
	{
#if MI_FIX //ee read
		mutex_lock(&db->spi_lockm); //.ee
#endif
		ret = dm9051_eeprom_read(db, (offset + i) / 2, data + i);
#if MI_FIX //ee read
		mutex_unlock(&db->spi_lockm);
#endif
		if (ret)
			break;
	}
	return ret;
}

static int dm9051_set_eeprom(struct net_device *ndev,
							 struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

#if MI_FIX //ee write
	mutex_lock(&db->spi_lockm); //.
#endif
	for (i = 0; i < len; i += 2)
	{
		ret = dm9051_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
#if MI_FIX //ee write
	mutex_unlock(&db->spi_lockm);
#endif
	return ret;
}

static void dm9051_get_pauseparam(struct net_device *ndev,
								  struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	*pause = db->pause;
}

static int dm9051_set_pauseparam(struct net_device *ndev,
								 struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE) {
		int ret;
#if MI_FIX //fcr
		mutex_lock(&db->spi_lockm); //.fcr
#endif
		ret = dm9051_update_fcr(db);
#if MI_FIX //fcr
		mutex_unlock(&db->spi_lockm);
#endif
		return ret;
	}

	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
					  pause->autoneg);
	phy_start_aneg(db->phydev);
	return 0;
}

const static char dm9051_stats_strings[][ETH_GSTRING_LEN] = {
        "rx_packets",
        "tx_packets",
        "rx_errors",
        "tx_errors",
        "rx_bytes",
        "tx_bytes",
        "fifo_rst",
        "dump BMSR register",
        "dump BMSR register",
};

static void dm9051_get_strings(struct net_device *netdev, u32 sget, u8 *data)
{
	if (sget == ETH_SS_STATS)
		memcpy(data, dm9051_stats_strings, sizeof(dm9051_stats_strings));
}

static int dm9051_get_sset_count(struct net_device *netdev, int sset)
{
	return (sset == ETH_SS_STATS) ? ARRAY_SIZE(dm9051_stats_strings) : 0;
}

static void dm9051_get_ethtool_stats(struct net_device *ndev,
									 struct ethtool_stats *stats, u64 *data)
{
	struct board_info *db = to_dm9051_board(ndev);

	/* ethtool -S eth1, this is the extra dump parts */
	data[0] = ndev->stats.rx_packets;
	data[1] = ndev->stats.tx_packets;
	data[2] = ndev->stats.rx_errors = db->bc.rx_err_counter;
	data[3] = ndev->stats.tx_errors = db->bc.tx_err_counter;
	data[4] = ndev->stats.rx_bytes;
	data[5] = ndev->stats.tx_bytes;
	data[6] = db->bc.fifo_rst_counter - 1; //Subtract Initial reset

	printk("\n");
	SHOW_XMIT_ANALYSIS(db);

#if MI_FIX //ee read
	mutex_lock(&db->spi_lockm);
#endif
	/*PHY_LOG*/
	DMPLUG_LOG_PHY(db);
	/*rx-state and BMSRs*/
	SHOW_RX_CTRLS(db);
	data[7] = SHOW_BMSR(db);
	data[8] = SHOW_BMSR(db);
#if MI_FIX //ee write
	mutex_unlock(&db->spi_lockm);
#endif

	SHOW_ALL_USER_CONFIG(NULL, db);
}

static const struct ethtool_ops dm9051_ethtool_ops = { //const struct ethtool_ops dm9051_ptpd_ethtool_ops
	.get_drvinfo = dm9051_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_msglevel = dm9051_get_msglevel,
	.set_msglevel = dm9051_set_msglevel,
	.nway_reset = phy_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = dm9051_get_eeprom_len,
	.get_eeprom = dm9051_get_eeprom,
	.set_eeprom = dm9051_set_eeprom,
	.get_pauseparam = dm9051_get_pauseparam,
	.set_pauseparam = dm9051_set_pauseparam,
	.get_strings = dm9051_get_strings,
	.get_sset_count = dm9051_get_sset_count,
	.get_ethtool_stats = dm9051_get_ethtool_stats,
	/* 4 ptpc */
	DMPLUG_PTP_INFO(.get_ts_info) //_15888_
};

/* all reinit while rx error found
 */
//int dm9051_all_reinit(struct board_info *db)
//{
//	int ret;

////	mutex_unlock(&db->spi_lockm);
////	phy_stop(db->phydev);
////	mutex_lock(&db->spi_lockm);

//	ret = _dm9051_core_reset(db);
//	if (ret)
//		return ret;

////	mutex_unlock(&db->spi_lockm);
////	phy_start(db->phydev);
////	phy_start_aneg(db->phydev);
////	mutex_lock(&db->spi_lockm);

//	ret = dm9051_all_start_intr(db);
//	if (ret)
//		return ret;

//	return dm9051_subconcl_and_rerxctrl(db);
//}

int dm9051_all_start(struct board_info *db)
{
	int ret;
	
	printk("_all_start\n");

	/* GPR power on of the internal phy
	 */
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	ret = dm9051_set_reg(db, DM9051_GPR, 0);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;

	ret = dm9051_phy_reset(db);
	if (ret)
		return ret;

	return dm9051_core_reset(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

	/* GPR power off of the internal phy,
	 * The internal phy still could be accessed after this GPR power off control
	 */
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;

	ret = dm9051_set_reg(db, DM9051_GPR, GPR_PHY_OFF);
	if (ret)
		return ret;

	printk("_phy_power_down: [internal] mdio phywr %d %04x\n", 0, 0x3900); //netif_info(db, link, db->ndev, ..
	netif_crit(db, hw, db->ndev, "netif_crit IsExtra-phy-power-down-redundent!?\n");
	ret = dm9051_phywrite(db, 0, 0x3900);
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_RCR, RCR_RX_DISABLE);
}

static int dm9051_all_start_mlock(struct board_info *db)
{
	int ret;

	#if MI_FIX
	mutex_lock(&db->spi_lockm); //.open
	#endif
	
	ret = dmplug_loop_test(db); //.DMPLUG_LPBK_TST
	if (ret)
		return ret;

	ret = dm9051_all_start(db);
	if (ret)
		return ret;

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	return ret;
}

static int dm9051_all_stop_mlock(struct board_info *db)
{
	int ret;

	#if MI_FIX
	mutex_lock(&db->spi_lockm);
	#endif

	ret = dm9051_all_stop(db);

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	return ret;
}

/* fifo reset while rx error found
 */
static int dm9051_all_restart(struct board_info *db) //todo
{
	int ret;
		
	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;
	ret = dm9051_phy_reset(db);
	if (ret)
		return ret;

	//=	ret = dm9051_all_reinit(db); //up_restart
	#if 1		
		ret = dm9051_core_reset(db);
		if (ret)
			return ret;

		ret = dm9051_all_start_intr(db);
		if (ret)
			return ret;

		ret = dm9051_subconcl_and_rerxctrl(db);
		if (ret)
			return ret;
	#endif

	db->bc.fifo_rst_counter++;
	SHOW_RESTART_SHOW_STATIISTIC(db);
	return 0;
}

/* to re-write while link change up
 */
int dm9051_all_upstart(struct board_info *db)
{
	int ret;

	printk("_all_upstart\n"); //NOT to .netif_crit(db, rx_err, db->ndev, "_all_upstart\n");
	do {
		ret = dm9051_ncr_reset(db);
		if (ret)
			goto dnf_end;
		
	//ret = dm9051_all_reinit(db); //up_restart
	#if 1		
		ret = dm9051_core_reset(db);
		if (ret)
			goto dnf_end;

		ret = dm9051_all_start_intr(db);
		if (ret)
			goto dnf_end;

		ret = dm9051_subconcl_and_rerxctrl(db);
		//if (ret)
		//	goto dnf_end;
	#endif
	} while(0);
	/* Commented: Just done by dm9051_set_fcr(db),
	 *            No need update.
	 * ret = dm9051_update_fcr(db);
	 */
dnf_end:
	netif_crit(db, rx_err, db->ndev, "DMCONF_MRR_WR operation done!\n");
	return ret;
}

int dm9051_all_upfcr(struct board_info *db)
{
	netif_crit(db, link, db->ndev, "DMCONF_MRR_WR operation not applied!\n");
	netif_crit(db, link, db->ndev, "So does (all_start(open), all_upstart(link_chg), all_restart(err_fnd)) not applied!\n");
	return dm9051_update_fcr(db);
}

int dm9051_all_start_intr(struct board_info *db)
{
	int ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_init_intcr_value(db));
	if (ret)
		return ret;

	return dm9051_enable_interrupt(db);
}

int dm9051_subconcl_and_rerxctrl(struct board_info *db)
{
	int ret;

	ret = dm9051_set_recv(db);
	if (ret)
		return ret;

	return dm9051_set_fcr(db);
}

#define DM9051_RX_BREAK(exp, yhndlr, nhndlr) \
	do {	\
		if ((exp)) {	\
			yhndlr;	\
		} else {	\
			nhndlr;	\
		}	\
	} while(0)

int trap_clr(struct board_info *db)
{
	db->bc.evaluate_rxb_counter = 0;
	return 0;
}

// check rxbs
// return: 0 : Still not trap
//         1 : Do reatart trap
int trap_rxb(struct board_info *db, unsigned int *prxbyte)
{
	if (SCAN_BH(*prxbyte) == 0)
		return 0;

	do
	{
		static unsigned int inval_rxb[TIMES_TO_RST] = {0};
		int n = 0;
		unsigned int i;
		char pbff[80];

		inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
		db->bc.evaluate_rxb_counter++;

		if (db->bc.evaluate_rxb_counter == 1) {
			char head[HEAD_LOG_BUFSIZE];
			sprintf(head, "rxb 1st %d", db->bc.evaluate_rxb_counter);
			DMPLUG_LOG_RXPTR(head, db);
		}

		n += sprintf(pbff + n, "_[eval_rxb %2d]", db->bc.evaluate_rxb_counter);
		for (i = 0; i < db->bc.evaluate_rxb_counter; i++)
		{
			if (i && !(i % 5))
				n += sprintf(pbff + n, " ");
			if (db->bc.evaluate_rxb_counter > 5 && i < 5)
			{
				n += sprintf(pbff + n, "  .");
				continue;
			}
			n += sprintf(pbff + n, " %02x", inval_rxb[i]);
		}
		netif_warn(db, rx_status, db->ndev, "%s\n", pbff);

		if (db->bc.evaluate_rxb_counter >= TIMES_TO_RST)
		{
			trap_clr(db);
			memset(inval_rxb, 0, sizeof(inval_rxb));
			return 1;
		}
	} while (0);
	return 0;
}

int rx_break(struct board_info *db, unsigned int rxbyte, netdev_features_t features)
{
	monitor_rxb0(db, rxbyte);
	if (features & NETIF_F_RXCSUM)
	{
		//DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), return 0, 
		//	netif_warn(db, rx_status, db->ndev, "YES checksum check\n"); 
		//	return -EINVAL);
		
		do {	\
			if (((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY)) {	\
				return 0;	\
			} else {	\
				netif_warn(db, rx_status, db->ndev, "Oops checksum check\n"); \
				return -EINVAL;	\
			}	\
		} while(0);
	}
	else
		DM9051_RX_BREAK((SCAN_BH(rxbyte) == DM9051_PKT_RDY), return 0,
			return -EINVAL);
}

int rx_head_break(struct board_info *db)
{
	int rxlen;
	//u8 err_bits = RSR_ERR_BITS;
	//#ifdef DMPLUG_PTP
	//err_bits = ptp_status_bits(db);
	//#endif
	u8 err_bits = GET_RSR_BITS(db); /* 7 rxhead ptpc, when REG60H.D[0]=0, PTP Function enable, or REG61H.D[0]=1, and D[1]=0, then re-defined RSR */

	rxlen = le16_to_cpu(db->rxhdr.rxlen);
	if (db->rxhdr.status & err_bits || rxlen > DM9051_PKT_MAX)
	{
		netif_warn(db, rx_status, db->ndev, "Err: [dm9.Monitor headbyte/status/rxlen %2x %2x %04x]\n",
			   db->rxhdr.headbyte,
			   db->rxhdr.status,
			   db->rxhdr.rxlen);

		if (db->rxhdr.headbyte != 0 &&  db->rxhdr.headbyte != 0x01) {
			netif_warn(db, rx_status, db->ndev, "Err: rxhdr-byte (%02x)\n",
					   db->rxhdr.headbyte);
		}

		//if (db->rxhdr.status & RSR_ERR_BITS)
		if (db->rxhdr.status & err_bits)
		{
			db->bc.status_err_counter++;
		}
		else
		//if (rxlen > DM9051_PKT_MAX)
		{
			db->bc.large_err_counter++;
		}

		if (db->rxhdr.status & err_bits) {
			netif_warn(db, rx_status, db->ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
		}

		if (rxlen > DM9051_PKT_MAX) {
			netif_warn(db, rx_status, db->ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
		}

		return 1;
	}

	/* -rxhead ptpc */
	DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(db);
	return 0;
}

/* read packets from the fifo memory
 * return value,
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data invalide(NOT an error), caller escape from rx operation
 */
int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret, rxlen, padlen;
	unsigned int rxbyte;
	struct sk_buff *skb;
	int ntx;
	u8 *rdptr;
	int scanrr = 0;

	do
	{
#if 1
		/* In rx-loop
		 */
		sprintf(db->bc.head, "_THrd-in");
		db->bc.mode = TX_THREAD0;
		ntx = dm9051_loop_tx(db); /* [More] and more tx better performance */
		if (ntx) {
			db->xmit_thrd0++;
			db->xmit_ttc0 += ntx;
			#ifdef DMPLUG_LOG
			if (db->xmit_thrd0 <= 9) {
				netif_warn(db, tx_queued, db->ndev, "%2d [_THrd-in] run %u Pkt %u zero-in %u, on-THrd-in %u Pkt %u\n", db->xmit_thrd0,
					db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd0, db->xmit_ttc0);
			}
			#endif
		}
#endif
		ret = dm9051_read_mem_rxb(db, DM_SPI_MRCMDX, &rxbyte, 2);
		if (ret)
			return ret;

		if (rx_break(db, rxbyte, ndev->features))
		{
			if (trap_rxb(db, &rxbyte)) {
				DMPLUG_LOG_RXPTR("rxb last", db);
				dm9051_all_restart(db);
				return -EINVAL;
			}
			break;
		}
		trap_clr(db);

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		/* rx_head_takelen check */
		ret = rx_head_break(db);
		if (ret) {
			dm9051_all_restart(db);
			return -EINVAL;
		}

		/* receive rx_tstamp */
		/* 7.1 ptpc */
		ret = DMPLUG_RX_TS_MEM(db);
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		padlen = (plat_cnf->skb_wb_mode && (rxlen & 1)) ? rxlen + 1 : rxlen;
		skb = dev_alloc_skb(padlen);
		if (!skb)
		{
			ret = dm9051_dumpblk(db, DM_SPI_MRCMD, padlen);
			if (ret)
				return ret;
			break; //.return scanrr;
		}

		rdptr = skb_put(skb, rxlen - 4);
		ret = dm9051_read_mem_cache(db, DM_SPI_MRCMD, rdptr, padlen);
		if (ret)
		{
			db->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			return ret;
		}

		/* 7.2dbg ptpc */
		SHOW_ptp_rx_packet_monitor(db, skb);

		skb->protocol = eth_type_trans(skb, db->ndev);

		/* 7.2 ptpc */
		DMPLUG_RX_HW_TS_SKB(db,skb);

		if (ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	} while (!ret);
	SHOW_MONITOR_RXC(db, scanrr);

#if 1
	/* Ending rx-loop
	 */
	//int ntx;
	sprintf(db->bc.head, "_THrd");
	db->bc.mode = TX_THREAD;
	ntx = dm9051_loop_tx(db); /* more tx better performance */
	if (ntx) {
		db->xmit_thrd++;
		db->xmit_ttc += ntx;
		#ifdef DMPLUG_LOG
		if (db->xmit_thrd <= 9) {
			netif_warn(db, tx_queued, db->ndev, "%2d [_THrd-end] run %u Pkt %u zero-in %u, on-THrd-end %u Pkt %u\n", db->xmit_thrd,
				db->xmit_in, db->xmit_tc, db->xmit_zc, db->xmit_thrd, db->xmit_ttc);
		}
		#endif
	}
#endif
	return scanrr;
}

#if defined(DM9051_NORM_BACKUP_TX) // -#if !defined(_DMPLUG_CONTI) -#endif
#ifdef DM9051_SKB_PROTECT
static struct sk_buff *EXPAND_SKB(struct sk_buff *skb, unsigned int pad)
{	
	struct sk_buff *skb2;

	skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
	if (skb2) {
		dev_kfree_skb(skb);
		return skb2;
	}

	//.netif_warn(db, tx_queued, db->ndev, "[WB_SUPPORT] warn on len %d, skb_copy_expand get memory leak!\n", skb->len);
	return skb;
}
#endif

/* particulars, wb mode*/
static struct sk_buff *dm9051_pad_txreq(struct board_info *db, struct sk_buff *skb)
{
	db->data_len = skb->len;
	db->pad = (plat_cnf->skb_wb_mode && (skb->len & 1)) ? 1 : 0; //'~wb'
#ifdef DM9051_SKB_PROTECT
	if (db->pad)
		skb = EXPAND_SKB(skb, db->pad);
#endif
	return skb;
}

int dm9051_single_tx(struct board_info *db, u8 *p)
{
	int ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = dm9051_write_mem_cache(db, p, db->data_len + db->pad); //'!wb'
	if (ret)
		return ret;

	return dm9051_set_regs(db, DM9051_TXPLL, &db->data_len, 2); //address of structure's field
}

int dm9051_req_tx(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_TCR, db->tcr_wr); //base with TCR_TXREQ
}

static int dm9051_tx_send(struct board_info *db, struct sk_buff *skb)
{
	int ret;

	/*
	 * #define EXPEND_LEN(datlen,pd) (datlen + pd)
	 * #define WRITE_SKB(db,p,len) dm9051_write_mem_cache(db,p,len)
	 * if (!plat_cnf->skb_wb_mode) {
	 *   ret = WRITE_SKB(db, skb, skb->len);
	 *   ret = dm9051_single_tx(db, skb->len);
	 * }
	 * else {
	 *   pad = ...
	 *   skb = _EXPAND_SKB(skb, pad);
	 *   ret = WRITE_SKB(db, skb, EXPEND_LEN(data_len, pad);
	 *   ret = dm9051_single_tx(db, skb, data_len, pad);
	 * }
	 * ret = dm9051_req_tx(db);
	 */
	do {
		ret = dm9051_single_tx(db, skb->data);
		if (ret)
			break;

		ret = dm9051_req_tx(db);
	} while(0);

	if (!ret) {
		struct net_device *ndev = db->ndev;
		ndev->stats.tx_bytes += db->data_len;
		ndev->stats.tx_packets++;
	}

	return ret;
}
#endif

int TX_SENDC(struct board_info *db, struct sk_buff *skb)
{
	int ret;

#if defined(STICK_SKB_CHG_NOTE)
#if !defined(DMPLUG_CONTI)
	skb = dm9051_pad_txreq(db, skb);
#endif
#endif

	/* 6.0 tx ptpc */
	if (DMPLUG_PTP_TX_IN_PROGRESS(skb)) //tom tell, 20250522
		netdev_dbg(db->ndev, "Yes, This is a hardware timestamp requested\n");

	/* 6 tx ptpc */
	DMPLUG_PTP_TX_PRE(db, skb);

	ret = TX_SEND(db, skb);

	/* 6.1 tx ptpc */
	DMPLUG_TX_EMIT_TS(db, skb);

#if defined(STICK_SKB_CHG_NOTE)
	dev_kfree_skb(skb);
#endif

	#ifdef DMPLUG_LOG /* only config enable _log */
	if ((db->bc.mode == TX_DELAY && db->xmit_in <=9) || 
		(db->bc.mode == TX_THREAD  && db->xmit_thrd <= 9) ||
		(db->bc.mode == TX_THREAD0  && db->xmit_thrd0 <= 9)) {
		netif_info(db, tx_queued, db->ndev, "%s. tx_send end_wr %02x\n", db->bc.head, db->tcr_wr);
	}
	#endif
	return ret;
}

int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;

	while (!skb_queue_empty(&db->txq))
	{
		struct sk_buff *skb = skb_dequeue(&db->txq);
		if (skb) {
			if (TX_SENDC(db, skb)) {
				db->bc.tx_err_counter++;
				if (netif_queue_stopped(ndev) &&
					(skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
					netif_wake_queue(ndev);
				return ntx;
			}
			ntx++;
		}

		if (netif_queue_stopped(ndev) &&
			(skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

/* threaded_irq */
/* schedule delay works */

static void dm9051_rxctl_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, rxctrl_work);
	struct net_device *ndev = db->ndev;
	int result;

	mutex_lock(&db->spi_lockm);

	result = dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
	if (result < 0)
		goto out_unlock;

	dm9051_set_recv(db);

	/* To has mutex unlock and return from this function if regmap function fail
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);
}

/* start_xmit schedule delay works */

static void dm9051_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int ntx;

	mutex_lock(&db->spi_lockm);

	sprintf(db->bc.head, "_dely");
	db->bc.mode = TX_DELAY;
	ntx = dm9051_loop_tx(db);

	db->xmit_tc += ntx;
	db->xmit_zc += ntx ? 0 : 1;
	if (ntx) {
		db->xmit_in++;
		#ifdef DMPLUG_LOG
		if (db->xmit_in <= 9) {
			netif_info(db, tx_queued, db->ndev, "%2d [_dely] run %u Pkt %u zero-in %u\n", db->xmit_in,
				db->xmit_in, db->xmit_tc, db->xmit_zc);
		}
		#endif
	}

	mutex_unlock(&db->spi_lockm);
}

#if 1
//static void dm9051_rx_xplat_enable(struct board_info *db)
//{
//	_dm9051_enable_interrupt(db);
//}
//static void dm9051_rx_xplat_loop(struct board_info *db)
//{
//	int ret;
//	dm9051_delayp_looping_rx_tx(db); //.looping_rx_tx()
//.	ret =
//.	if (ret < 0)
//.		return;
//	_dm9051_enable_interrupt(db); //"dm9051_rx_xplat_enable(struct board_info *db)"
//}

//static int dm9051_rx_xplat_disable(struct board_info *db)
//{
//	int result = dm9051_disable_interrupt(db);
//	if (result)
//		return result;

//	result = dm9051_clear_interrupt(db);
//	if (result)
//		return result;
//	return result;
//}

/* Interrupt: Interrupt work */
/* dm9051 rx_threaded irq */

void dm9051_thread_irq(void *pw) //.(macro)_rx_tx_plat() //dm9051_rx_threaded_irq()
{
	struct board_info *db = pw;
	int result;

	mutex_lock(&db->spi_lockm);

	//[REAL.'MI_FIX'] //(result is as 'dm9051_rx_xplat_disable'(db))
	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	/* return value from _dm9051_delayp_looping_rx_tx(),
	 *    0 - no error, caller need stop further rx operation
	 *  -EBUSY - read data invalide(NOT an error), caller escape from rx operation
	 */
	while (dm9051_loop_rx(db) > 0) ; //dm9051_delayp_looping_rx_tx(db);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);
	//_thread_servicep_done = 1;
	//return IRQ_HANDLED;
}

/* threaded_irq */

int thread_servicep_done = 1;
int thread_servicep_re_enter = 0;

irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw)
{
	struct board_info *db = pw;

	if (thread_servicep_done) {
		thread_servicep_done = 0;
		if (!thread_servicep_re_enter)
			netif_notice(db, intr, db->ndev, "INT: [%s] this-first-enter %d\n", __func__, thread_servicep_re_enter++);
		dm9051_thread_irq(db); //(voidirq, pw) //.(macro)_rx_tx_plat()
		thread_servicep_done = 1;
	} else {
		//.if (thread_servicep_re_enter <= 9)
		netif_err(db, intr, db->ndev, "_.PLAT.WARN   [%s] re-enter %d\n", __func__, thread_servicep_re_enter++);
	}
	return IRQ_HANDLED;
}
#endif

#if defined(DM9051_INTR_BACKUP) // -#if defined(DMPLUG_INT) -#endif
/*
 * Interrupt: 
 */
static int dm9051_req_irq(struct board_info *db, irq_handler_t handler)
{
	struct spi_device *spi = db->spidev;
	int ret;

	netif_warn(db, intr, db->ndev, "request_irq(INT MODE)\n");
	thread_servicep_re_enter = 0; //used in 'dm9051_rx_threaded_plat'
	ret = request_threaded_irq(spi->irq, NULL, handler, //'dm9051_rx_threaded_plat'
							   get_dts_irqf(db) | IRQF_ONESHOT,
							   db->ndev->name, db);
	if (ret < 0)
		netif_err(db, intr, db->ndev, "failed to rx request threaded irq setup\n");
	return ret;
}
#endif

static int dm9051_threaded_irq(struct board_info *db, irq_handler_t handler)
{
#ifndef DMPLUG_INT
	if (dm9051_poll_supp())
		return dm9051_poll_sch(db);
	return 0;
#endif

#if defined(DMPLUG_INT)
	if (dm9051_int2_supp())
		return dm9051_int2_irq(db, dm9051_rx_int2_delay);

	return dm9051_req_irq(db, handler);
#endif
}

#if defined(DMPLUG_INT)
static void dm9051_thread_irq_free(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	#ifdef INT_TWO_STEP
	cancel_delayed_work_sync(&db->irq_servicep);
	#endif //_INT_TWO_STEP

	free_irq(db->spidev->irq, db);
	netif_err(db, intr, ndev, "_[stop] remove: free irq %d\n", db->spidev->irq);
}
#endif

static void dm9051_free_irqworks(struct board_info *db)
{
	/* schedule delay work */
	#if defined(DMPLUG_INT)
	/*
	 * Interrupt: 
	 */
	dm9051_thread_irq_free(db->ndev);
	#else
	/*
	 * Polling: 
	 */
	cancel_delayed_work_sync(&db->irq_workp);
	#endif
}

/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;

	SHOW_OPEN(db);
	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	PTP_INIT_RCR(db);
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */

	ret = dm9051_all_start_mlock(db); /* such as all start */
	if (ret)
		return ret;

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	netif_wake_queue(ndev);

	ret = dm9051_threaded_irq(db, dm9051_rx_threaded_plat); /* near the bottom */
	if (ret < 0) {
		phy_stop(db->phydev); //of 'dm9051_core_clear(db)' //
		return ret;
	}


	#if MI_FIX
	mutex_lock(&db->spi_lockm);//.open's
	#endif

	ret = dm9051_all_start_intr(db); /* near the bottom */

	#if MI_FIX
	mutex_unlock(&db->spi_lockm);
	#endif

	if (ret) {
		phy_stop(db->phydev);
		dm9051_free_irqworks(db);
		return ret;
	}

	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
////static int dm9051_stop001(struct net_device *ndev)
////{
//	mutex_lock(&db->spi_lockm);
//	ret = dm9051_all_stop(db);
//	mutex_unlock(&db->spi_lockm);
//	if (ret)
//		return ret;
////	mutex_lock(&db->spi_lockm);
//	phy_stop(db->phydev);
////	mutex_unlock(&db->spi_lockm);
////}
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	netif_err(db, probe, ndev, "dm9051_stop\n"); //as 'probe' type, original dev_info()

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

	dm9051_free_irqworks(db);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return dm9051_all_stop_mlock(db);
}

/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	#if 0
	//show_ptp_type(skb);	//Show PTP message type
	skb_tx_timestamp(skb); //Spenser - Report software Timestamp v.s. - Report HW Timestamp
	#endif

	schedule_work(&db->tx_work);

	return NETDEV_TX_OK;
}

/* event: play with a schedule starter
 */
static void dm9051_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct dm9051_rxctrl rxctrl;
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;

	memset(&rxctrl, 0, sizeof(rxctrl));

	/* rx control */
	if (ndev->flags & IFF_PROMISC)
	{
		rcr |= RCR_PRMSC;
		netif_crit(db, hw, db->ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI)
	{
		rcr |= RCR_ALL;
		netif_crit(db, hw, db->ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	rxctrl.rcr_all = rcr;

	/* broadcast address */
	rxctrl.hash_table[0] = 0;
	rxctrl.hash_table[1] = 0;
	rxctrl.hash_table[2] = 0;
	rxctrl.hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev)
	{
		hash_val = ether_crc_le(ETH_ALEN, ha->addr) & GENMASK(5, 0);
		rxctrl.hash_table[hash_val / 16] |= BIT(0) << (hash_val % 16);
	}

	/* schedule work to do the actual set of the data if needed */
	if (memcmp(&db->rctl, &rxctrl, sizeof(rxctrl)))
	{
		memcpy(&db->rctl, &rxctrl, sizeof(rxctrl));
		schedule_work(&db->rxctrl_work);
	}
}

/* event: write into the mac registers and eeprom directly
 */
static int dm9051_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = eth_prepare_mac_addr_change(ndev, p);
	if (ret < 0)
		return ret;

	eth_commit_mac_addr_change(ndev, p);

	#if MI_FIX //mac
	mutex_lock(&db->spi_lockm);
	#endif
	ret = dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
	#if MI_FIX //mac
	mutex_unlock(&db->spi_lockm);
	#endif
	return ret;
}

static int dm9051_ndo_set_features(struct net_device *ndev,
								   netdev_features_t features)
{
	// netdev_features_t changed = ndev->features ^ features;
	struct board_info *db = to_dm9051_board(ndev);

	if ((features & NETIF_F_RXCSUM) && (features & NETIF_F_HW_CSUM))
	{
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling TX/RX checksum]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_RXCSUM)
	{
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling RX checksum only]\n");
		db->csum_gen_val = 0x0; //dm9051_set_reg(db, 0x31, 0x0);
		db->csum_rcv_val = 0x3; //dm9051_set_reg(db, 0x32, 0x3);
	}
	else if (features & NETIF_F_HW_CSUM)
	{
		netif_info(db, drv, db->ndev, "_ndo set and write [Enabling TX checksum only]\n");
		db->csum_gen_val = 0x7; //dm9051_set_reg(db, 0x31, 0x7);
		db->csum_rcv_val = 0x0; //dm9051_set_reg(db, 0x32, 0x0);
	}
	else
	{
		//netif_info(db, drv, db->ndev, "_ndo set and write [Disabling TX/RX checksum]\n");
		db->csum_gen_val = 0x0; //dm9051_set_reg(db, 0x31, 0x0);
		db->csum_rcv_val = 0x0; //dm9051_set_reg(db, 0x32, 0x0);
	}

#if MI_FIX
	mutex_lock(&db->spi_lockm);
#endif
	dm9051_set_reg(db, 0x31, db->csum_gen_val);
	dm9051_set_reg(db, 0x32, db->csum_rcv_val);
#if MI_FIX
	mutex_unlock(&db->spi_lockm);
#endif

	return 0;
}

static struct net_device_stats *dm9051_get_stats(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	ndev->stats.tx_errors = db->bc.tx_err_counter;
	ndev->stats.rx_errors = db->bc.rx_err_counter;
	return &ndev->stats;
}

static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
	.ndo_set_features = dm9051_ndo_set_features,
	.ndo_get_stats = dm9051_get_stats,
	/* 5 ptpc */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
	DMPLUG_PTP_TS_INFO(.ndo_do_ioctl) //.ndo_do_ioctl = dm9051_ptp_netdev_ioctl, //_15888_
#else
	DMPLUG_PTP_TS_INFO(.ndo_eth_ioctl) //.ndo_eth_ioctl = dm9051_ptp_netdev_ioctl, //_15888_
#endif
};

static void CHKSUM_PTP_NDEV(struct board_info *db, struct net_device *ndev)
{
	/* Set default features */
	if (plat_cnf->checksuming)
		ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;

	/* 2 ptpc */
	if (PTP_NEW(db))
		ndev->features &= ~(NETIF_F_HW_CSUM | NETIF_F_RXCSUM); //"Run PTP must COERCE to disable checksum_offload"

	ndev->hw_features |= ndev->features;
}

static void DM9051_PROBE_DLYSETUP(struct board_info *db)
{
	#if defined(DMPLUG_INT)
	/*
	 * Interrupt: 
	 */
	#ifdef INT_TWO_STEP
		PROBE_INT2_DLY_SETUP(db);
	#endif
	#else
	/*
	 * Polling: 
	 */
	PROBE_POLL_SETUP(db);
	#endif
}

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;

	trap_clr(db);
	db->bc.nRxcF = 0;

	db->csum_gen_val = 0; //disabling
	db->csum_rcv_val = 0; //disabling

	db->n_automdix = 0; //log-reset
	db->stop_automdix_flag = 0;
	db->automdix_log[0][0] = 0;
	db->automdix_log[1][0] = 0;
	db->automdix_log[2][0] = 0;
	db->mdi = 0x0830;
	
	db->tcr_wr = TCR_TXREQ; //pre-defined
	
	db->xmit_in = 0;
	db->xmit_tc = 0;
	db->xmit_zc = 0;
	db->xmit_thrd0 = 0;
	db->xmit_ttc0 = 0;
	db->xmit_thrd = 0;
	db->xmit_ttc = 0;
}

static int dm9051_mdio_register(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = devm_mdiobus_alloc(&spi->dev);
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = dm9051_mdio_read;
	db->mdiobus->write = dm9051_mdio_write;
	db->mdiobus->name = "dm9051-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1); //if ((bus->phy_mask & BIT(1)) == 0) accepted
	db->mdiobus->parent = &spi->dev;
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
			 "dm9051-%s.%u", dev_name(&spi->dev), spi->chip_select);

	ret = devm_mdiobus_register(&spi->dev, db->mdiobus);
	if (ret)
		dev_err(&spi->dev, "Could not register MDIO bus\n");

	return ret;
}

//static void FCR_UPSTART_MRR_WR(struct board_info *db)
//{
//	#if MI_FIX
//	mutex_lock(&db->spi_lockm);
//	#endif
//	
//	_LINKCHG_UPSTART(db);

//	#if MI_FIX
//	mutex_unlock(&db->spi_lockm);
//	#endif
//}

static void dm9051_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	if (db->phydev->link)
	{
		printk("\n");
		printk("LOCK_MUTEX\n");
	}

	phy_print_status(db->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link)
	{
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}

		#if MI_FIX
		mutex_lock(&db->spi_lockm);
		LINKCHG_UPSTART(db);
		mutex_unlock(&db->spi_lockm);
		#endif

		printk("UNLOCK_MUTEX\n");
		printk("\n");
	}
}

/* phy connect as poll mode
 */
static int dm9051_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
			 db->mdiobus->id, DM9051_PHY_ADDR);

	db->phydev = phy_connect(db->ndev, phy_id, dm9051_handle_link_change,
							 PHY_INTERFACE_MODE_MII);
	return PTR_ERR_OR_ZERO(db->phydev);
}

static int dm9051_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	int ret;

	ndev = devm_alloc_etherdev(dev, sizeof(struct board_info));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	db = netdev_priv(ndev);

	db->spidev = spi;
	db->ndev = ndev;

	CHKSUM_PTP_NDEV(db, ndev);

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;

	/* version log */
	SHOW_DEVLOG_REFER_BEGIN(dev, db);

	//[NETIF_MSG_HW is play for phylib...]
	//db->msg_enable = 0;
	db->msg_enable = NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP |
		NETIF_MSG_RX_ERR | NETIF_MSG_TX_ERR | NETIF_MSG_TX_QUEUED | NETIF_MSG_INTR | NETIF_MSG_TX_DONE |
		NETIF_MSG_RX_STATUS | NETIF_MSG_PKTDATA | NETIF_MSG_HW /*| 0*/;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

	DM9051_PROBE_DLYSETUP(db);

	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	SHOW_DEVLOG_MODE(dev);

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

	ret = dm9051_map_etherdev_par(ndev, db);
	if (ret < 0)
		return ret;

	ret = dm9051_mdio_register(db);
	if (ret)
		return ret;

	dm9051_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = devm_register_netdev(dev, ndev);
	if (ret)
	{
		//phy_disconnect(db->phydev);
		return dev_err_probe(dev, ret, "device register failed");
	}
	
	//if (netif_running(ndev)) ..
	SHOW_LOG_REFER_BEGIN(db);

	/* 2.1 ptpc */
	PTP_INIT(db);

	return dm9051_phy_connect(db); /* phy connect in the bottom */
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,0) || \
	LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
static int dm9051_drv_remove(struct spi_device *spi)
#else
static void dm9051_drv_remove(struct spi_device *spi)
#endif
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	printk("\n");
	netif_err(db, probe, db->ndev, "_[phy] remove: disconnect\r\n"); //as 'probe' type
	phy_disconnect(db->phydev);

	/* 3 ptpc */
	PTP_END(db);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,0) || \
	LINUX_VERSION_CODE <= KERNEL_VERSION(5,10,11)
	return 0;
#endif
}

static const struct of_device_id dm9051_match_table[] = {
	{.compatible = "davicom,dm9051"},
	{}};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
MODULE_DEVICE_TABLE(of, dm9051_match_table);
#endif

static const struct spi_device_id dm9051_id_table[] = {
	{"dm9051", 0},
	{}};

static struct spi_driver dm9051_driver = {
	.driver = {
		.name = DRVNAME_9051,
		.of_match_table = dm9051_match_table,
	},
	.probe = dm9051_probe,
	.remove = dm9051_drv_remove,
	.id_table = dm9051_id_table,
};
module_spi_driver(dm9051_driver);

MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_DESCRIPTION("Davicom DM9051 network SPI driver");
MODULE_LICENSE("GPL");
