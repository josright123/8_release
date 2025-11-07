// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "dm9051.h"

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
	if (unlikely(db->debug_print & DP_REG_WRITE)) {
		dm_printk("[dbg] %s(): reg 0x%02x, val 0x%02x\n", __func__, reg, val);
	}

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
	do {
		ret = regmap_read(db->regmap_dm, reg, &rb);
		if (ret < 0) {
			netif_err(db, drv, ndev, "%s: error %d dumping read reg %02x\n",
				  __func__, ret, reg);
			break;
		}
	} while (--count);

	return ret;
}

#if BOARD_CONF < DM9051_KERNEL_6_6
static int regmap_noinc_write_kt61(struct regmap *regmap, unsigned int reg,
				   const void *buff, size_t len)
{
	const u8 *p = (const u8 *) buff;
	unsigned int val;
	int ret;

	while (len--) {
		val = (unsigned int) *p++;
		ret = regmap_write(regmap, reg, val);
		if (ret < 0)
			break;
	}

	return ret;
}

static int regmap_noinc_read_kt61(struct regmap *regmap, unsigned int reg,
				  void *buff, size_t len)
{
	u8 *p = buff;
	unsigned int rb;
	int ret;

	while (len--) {
		ret = regmap_read(regmap, reg, &rb);
		*p++ = (u8) (rb & GENMASK(7, 0));
		if (ret < 0)
			break;
	}

	return ret;
}
#endif

int dm9051_set_regs(struct board_info *db, unsigned int reg, const void *val,
		    size_t val_count)
{
	int ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk writing regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

int dm9051_get_regs(struct board_info *db, unsigned int reg, void *val,
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

#if BOARD_CONF >= DM9051_KERNEL_6_6
	//ret = regmap_noinc_write(db->regmap_dm, reg, buff, len);
#if 1
	//[dbg] bilk.wr
	const u8 *p = (const u8 *) buff;
	//[dbg] bilk.wr
#define CBLKTX	32 //256.NG //32 //64 //1024 //32(OK)
	size_t BLKTX = CBLKTX; //size_t BLKTX = 64;
	while (len >= BLKTX) {
		ret = regmap_noinc_write(db->regmap_dm, reg, p, BLKTX);
		p += BLKTX;
		len -= BLKTX;
		if (ret < 0) {
			netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x len %lu\n",
				  __func__, ret, reg, BLKTX);
			return ret;
		}
	}
	//[dbg] bilk.wr
	while (len--) {
		unsigned int val = (unsigned int) *p++;
		ret = regmap_write(db->regmap_dm, reg, val);
		if (ret < 0)
			break;
	}
#endif
#else
	ret = regmap_noinc_write_kt61(db->regmap_dm, reg, buff, len);
#endif
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
		    size_t len)
{
	int ret = 0;

#if BOARD_CONF >= DM9051_KERNEL_6_6
	//ret = regmap_noinc_read(db->regmap_dm, reg, buff, len);
#if 1
	//[dbg] bulk.read
	u8 *p = buff;
	unsigned int rb;
	//[dbg] bulk.read
	//
#define CBLKRX	64 //32 //256.NG //32 //1 //32(OK)
	size_t BLKRX = CBLKRX; //64; //128.NG; //64(OK) //64(OK)
	while (len >= BLKRX) {
		ret = regmap_noinc_read(db->regmap_dm, reg, p, BLKRX);
		if (ret < 0) {
			netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x len %lu\n",
				  __func__, ret, reg, BLKRX);
			return ret;
		}
		p += BLKRX;
		len -= BLKRX;
	}

	while (len--) {
		ret = regmap_read(db->regmap_dm, reg, &rb);
		*p++ = (u8) (rb & GENMASK(7, 0));
		if (ret < 0)
			break;
	}
#endif
#else
	ret = regmap_noinc_read_kt61(db->regmap_dm, reg, buff, len);
#endif
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
static int dm9051_nsr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
				       mval & (NSR_TX2END | NSR_TX1END), 1, 20);
	if (ret == -ETIMEDOUT)
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

static int dm9051_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

static unsigned int dm9051_intcr_value(struct board_info *db)
{
#if 0
	//printk("dm9051_irq_flag(db) %d\n", dm9051_irq_flag(db));
	//printk("Set Reg, IRQF_TRIGGER_LOW %d, IRQF_TRIGGER_HIGH %d\n", IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
#endif

	return (dm9051_irq_flag(db) == IRQF_TRIGGER_LOW) ?
		INTCR_POL_LOW : INTCR_POL_HIGH;
}

static int dm9051_set_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_set_reg(db, DM9051_FCR, fcr);
}

static int dm9051_set_recv(struct board_info *db)
{
	int ret;

	ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all); /* enable rx */
}

static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val);

static int dm9051_ncr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
				       !(mval & NCR_RST), 10, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for ncr reset\n");
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

static int PHY_RST(struct board_info *db)
{
	printk("_phy_reset\n");
	int ret = dm9051_phywrite(db, 0, 0x8000);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	/* Jabber function disabled refer to bench test
	 * meeting advice 20250226
	 */
	//ret = dm9051_phywrite(db, 18, 0x7000);
	//if (ret)
	//	return ret;
	return 0;
}

static int dm9051_core_reset(struct board_info *db)
{
	int ret;

	db->bc.fifo_rst_counter++;

//	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
//	if (ret)
//		return ret;
	ret = dm9051_ncr_reset(db);
	if (ret)
		return ret;

	ret = PHY_RST(db);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, MBNDRY_BYTE); /* MemBound */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_COUNT); /* Pause Count */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	return ret; //return dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
}

static int dm9051_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
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

static int dm9051_stop_mrcmd(struct board_info *db)
{
	return dm9051_set_reg(db, DM9051_ISR, ISR_STOP_MRCMD); /* to stop mrcmd */
}

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
	if (ret)
		return ret;

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
	if (ret)
		return ret;

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

//static int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
//{
//	struct board_info *db = context;
//	int ret;

//	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
//	if (ret)
//		return ret;

//	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
//	if (ret)
//		return ret;

//	ret = dm9051_epcr_poll(db);
//	if (ret)
//		return ret;

//	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
//	if (ret)
//		return ret;

//	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
//	 * only fill lower 2 bytes
//	 */
//	*val = 0;
//	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, val, 2);
//}

static int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

//	mutex_lock(&db->phy_lockm);

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret)
		goto unlock;

	ret = dm9051_epcr_poll(db);
	if (ret)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		goto unlock;

	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
	 * only fill lower 2 bytes
	 */
	*val = 0;
	ret = regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, val, 2);

	//printk("phyread: reg= 0x%x, val= 0x%x\n", reg, *val);

unlock:
//	mutex_unlock(&db->phy_lockm);

	return ret;
}

//static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
//{
//	struct board_info *db = context;
//	int ret;

//	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
//	if (ret)
//		return ret;

//	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, &val, 2);
//	if (ret < 0)
//		return ret;

//	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
//	if (ret)
//		return ret;

//	ret = dm9051_epcr_poll(db);
//	if (ret)
//		return ret;

//	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
//}

static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	//printk("phywrite: reg= 0x%x, val= 0x%x\n",reg, val);

//	mutex_lock(&db->phy_lockm);

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		goto unlock;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, &val, 2);
	if (ret < 0)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret)
		goto unlock;

	ret = dm9051_epcr_poll(db);
	if (ret)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
unlock:
//	mutex_unlock(&db->phy_lockm);

	return ret;
}

static int dm9051_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	unsigned int val = 0xffff;
	int ret;

	if (addr == DM9051_PHY_ADDR) {
		ret = dm9051_phyread(db, regnum, &val);
		if (ret)
			return ret;
	}

	return val;
}

static int dm9051_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;

	if (addr == DM9051_PHY_ADDR) {
		//[dbg] mdio.wr BMCR
		if (regnum == 0) {
			//printk("_[mii_bus] mdio write 0x%x, 0x%04x\n", regnum, val);
			if (val & 0x800)
				printk("_[mii_bus] mdio write : power down (warn)\n");
		}
		return dm9051_phywrite(db, regnum, val);
	}

	return -ENODEV;
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
#define	KERNEL_ROADMAP_CONF_H_
#ifdef	KERNEL_ROADMAP_CONF_H_
#undef __devm_regmap_init_spi //compiler not undefined, since used in _wrapper

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
	struct spi_transfer t[2] = { { .tx_buf = reg, .len = reg_len, },
				     { .tx_buf = val, .len = val_len, }, };

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	return spi_sync(spi, &m);
}
static int regmap_spi_async_write(void *context, //.V510_COMPLEX
				  const void *reg, size_t reg_len,
				  const void *val, size_t val_len,
				  struct regmap_async *a) {
	printk("NOT SUPPORT: regmap_spi_async_write(context,...)\n");
	return -1;
}
static struct regmap_async *regmap_spi_async_alloc(void) { //.V510_COMPLEX
	printk("NOT SUPPORT: regmap_spi_async_alloc(void)\n");
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
						   const struct regmap_config *config) {
	return &regmap_bus_dm;
}

static struct regmap *__devm_regmap_init_spi_dm(struct spi_device *spi,
						const struct regmap_config *config,
						struct lock_class_key *lock_key,
						const char *lock_name)
{
	//const struct regmap_bus *bus = &regmap_bus_dm;
	const struct regmap_bus *bus = regmap_get_spi_bus(spi, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus); /* . */

	return __devm_regmap_init(&spi->dev, bus, &spi->dev, config, lock_key, lock_name);
}

#undef devm_regmap_init_spi
#define devm_regmap_init_spi(dev, config)				\
	__regmap_lockdep_wrapper(__devm_regmap_init_spi_dm, #config,	\
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

	ret = dm9051_get_regs(db, DM9051_VIDL, buff, sizeof(buff));
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	if ((wid != DM9058_ID) && (wid != DM9051_ID)) {
		dev_err(dev, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_info(dev, "chip %04x found\n", wid);
	return 0;
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

	if (!is_valid_ether_addr(addr)) {
		eth_hw_addr_random(ndev);

		ret = dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
		if (ret < 0)
			return ret;

		dev_dbg(&db->spidev->dev, "Use random MAC address\n");
		return 0;
	}

	eth_hw_addr_set(ndev, addr);
	return 0;
}

/* ethtool-ops
 */
static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRVNAME_9051, sizeof(info->driver));
	strscpy(info->version, DRIVER_VERSION, sizeof(info->version));
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

	for (i = 0; i < len; i += 2) {
		ret = dm9051_eeprom_read(db, (offset + i) / 2, data + i);
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

	for (i = 0; i < len; i += 2) {
		ret = dm9051_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
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

	if (pause->autoneg == AUTONEG_DISABLE)
		return dm9051_update_fcr(db);

	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
			  pause->autoneg);
	phy_start_aneg(db->phydev);
	return 0;
}

static const struct ethtool_ops dm9051_ethtool_ops = {
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
	.get_ts_info = dm9051_get_ts_info,
};

static int dm9051_all_start(struct board_info *db)
{
	int ret;

	/* GPR power on of the internal phy
	 */
//[dbg] pwr
	//printk("_[dm9051_all_start] set reg DM9051_GPCR, 0x%02x\n", (unsigned int) GPCR_GEP_CNTL);
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;
//[dbg] pwr
	//printk("_[dm9051_all_start] set reg DM9051_GPR, 0x%02x\n", 0);
	ret = dm9051_set_reg(db, DM9051_GPR, 0);
	if (ret)
		return ret;
//[dbg] pwr.phy.up
	//printk("_[dm9051_all_start] BMCR 0x3100: power up (new)\n");
//	ret = dm9051_phywrite(db, 0, 0x3100);
//	if (ret)
//		return ret;
	//printk("_[dm9051_all_start] OK\n");

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;
#if 1
	printk("Set dm9051_irq_flag() %d, _TRIGGER_LOW %d, _TRIGGER_HIGH %d (start)\n",
	       dm9051_irq_flag(db), IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
	if (ret)
		return ret;
#endif
	return dm9051_enable_interrupt(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

	/* GPR power off of the internal phy,
	 * The internal phy still could be accessed after this GPR power off control
	 */
//[dbg] pwr
	printk("_[dm9051_all_stop] set reg DM9051_GPCR, 0x%02x\n", (unsigned int) GPCR_GEP_CNTL);
	ret = dm9051_set_reg(db, DM9051_GPCR, GPCR_GEP_CNTL);
	if (ret)
		return ret;
//[dbg] pwr
	printk("_[dm9051_all_stop] set reg DM9051_GPR, 0x%02x\n", (unsigned int) GPR_PHY_OFF);
	ret = dm9051_set_reg(db, DM9051_GPR, GPR_PHY_OFF);
	if (ret)
		return ret;
//[dbg] pwr.phy.down
	//[dbg] mdio.wr BMCR
	printk("_[dm9051_all_stop] BMCR 0x3900: power down (warn)\n");
	ret = dm9051_phywrite(db, 0, 0x3900);
	if (ret)
		return ret;
	printk("_[dm9051_all_stop] OK\n");

	return dm9051_set_reg(db, DM9051_RCR, RCR_RX_DISABLE);
}

/* fifo reset while rx error found
 */
static int dm9051_all_restart(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret;

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;

#if 1
	printk("dm9.Set dm9051_irq_flag() %d, _TRIGGER_LOW %d, _TRIGGER_HIGH %d (restart)\n",
	       dm9051_irq_flag(db), IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
	ret = dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
	if (ret)
		return ret;
#endif

	ret = dm9051_enable_interrupt(db);
	if (ret)
		return ret;

	printk("dm9.Show rxstatus_Er & rxlen_Er %d, RST_c %d\n",
	       db->bc.status_err_counter + db->bc.large_err_counter,
	       db->bc.fifo_rst_counter);
	netdev_dbg(ndev, " rxstatus_Er & rxlen_Er %d, RST_c %d\n",
		   db->bc.status_err_counter + db->bc.large_err_counter,
		   db->bc.fifo_rst_counter);

	ret = dm9051_set_recv(db);
	if (ret)
		return ret;

	return dm9051_set_fcr(db);
}

#define TIMES_TO_RST	10
#define DM9051_RX_BREAK(exp, hndlr)		\
	do {					\
		if ((exp)) {			\
			hndlr;			\
		}				\
	} while(0)

#if 0
int env_evaluate_rxb(struct board_info *db, unsigned int rxbyte)
{
	//[dbg] rxb
	int ret;
	int n = 0;
	char pbff[80];
	unsigned int i;
	static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };
	inval_rxb[db->bc.evaluate_rxb_counter] = rxbyte;
	db->bc.evaluate_rxb_counter++;

	n += sprintf(pbff+n, "_[env_evaluate_rxb %2d]", db->bc.evaluate_rxb_counter);
	for (i = 0; i < db->bc.evaluate_rxb_counter; i++) {
		if (i && !(i%5))
			n += sprintf(pbff+n, " ");
		if (db->bc.evaluate_rxb_counter > 5 && i < 5) {
			n += sprintf(pbff+n, "  .");
			continue;
		}
		n += sprintf(pbff+n, " %02x", inval_rxb[i]);
	}
	printk("%s\n", pbff);
	if (db->bc.evaluate_rxb_counter >= TIMES_TO_RST) {
		db->bc.evaluate_rxb_counter = 0;
		memset(inval_rxb, 0, sizeof(inval_rxb));
		ret = dm9051_all_restart(db); //...
		if (!ret)
			printk("_[dm9051_all_restart] work around done\n");
	}
	return -EINVAL;
}
#endif

static void monitor_rxb0(unsigned int rxbyte)
{
	static int rxbz_counter = 0;
	static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };
	unsigned int i;
	int n = 0;
	char pbff[80];

	if (((rxbyte & GENMASK(7, 0)) == 1) || ((rxbyte & GENMASK(7, 0)) == 0))
		return;

	inval_rxb[rxbz_counter] = rxbyte & GENMASK(7, 0);
	rxbz_counter++;

	n += sprintf(pbff+n, "_[monitor_rxb0 %2d]", rxbz_counter);
	for (i = 0; i < rxbz_counter; i++) {
		if (i && !(i%5))
			n += sprintf(pbff+n, " ");
		if (rxbz_counter > 5 && i < 5) {
			n += sprintf(pbff+n, "  .");
			continue;
		}
		n += sprintf(pbff+n, " %02x", inval_rxb[i]);
	}
	printk("%s\n", pbff);

	if (rxbz_counter >= TIMES_TO_RST) {
		rxbz_counter = 0;
		memset(inval_rxb, 0, sizeof(inval_rxb));
		//ret = dm9051_all_restart(db); //...
		printk("_[Less constrain of old trap's dm9051_all_restart] only monitored.\n");
	}
}

// check rxbs
// return: 0 : Invalid
//         1 : Pkt_rdy
static int eval_rxb(struct board_info *db, unsigned int *prxbyte)
{
	u8 *bf = (u8 *) prxbyte;
	int n = 0;
	char pbff[80];
	unsigned int i;
	int ret;
	static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };

	if ((*prxbyte & GENMASK(15, 8)) == 0x0100)
		return 1;

	printk("_[  bf] %02x %02x\n", bf[0], bf[1]);
	printk("_[rxbs] %02lx %02lx\n", *prxbyte & GENMASK(7, 0), (*prxbyte & GENMASK(15, 8)) >> 8);

	inval_rxb[db->bc.evaluate_rxb_counter] = (*prxbyte & GENMASK(15, 8)) >> 8;
	db->bc.evaluate_rxb_counter++;

	n += sprintf(pbff+n, "_[eval_rxb %2d]", db->bc.evaluate_rxb_counter);
	for (i = 0; i < db->bc.evaluate_rxb_counter; i++) {
		if (i && !(i%5))
			n += sprintf(pbff+n, " ");
		if (db->bc.evaluate_rxb_counter > 5 && i < 5) {
			n += sprintf(pbff+n, "  .");
			continue;
		}
		n += sprintf(pbff+n, " %02x", inval_rxb[i]);
	}
	printk("%s\n", pbff);

	if (db->bc.evaluate_rxb_counter >= TIMES_TO_RST) {
		db->bc.evaluate_rxb_counter = 0;
		memset(inval_rxb, 0, sizeof(inval_rxb));
		ret = dm9051_all_restart(db); //...
		if (!ret)
			printk("_[dm9051_all_restart] work around done\n");
	}
	return 0;
}

/* read packets from the fifo memory
 * return value,
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data error, caller escape from rx operation
 */
static int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	unsigned int rxbyte;
	int ret, rxlen;
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;
	static int nloop_rx = 0;
        u8 rxTSbyte[8]; // 用於存儲 1588 Time Stamp

	do {
		//[dbg].redundent.rxb.restore.[dbg]

		ret = dm9051_read_mem(db, DM_SPI_MRCMDX, &rxbyte, 2);
		monitor_rxb0(rxbyte);
		DM9051_RX_BREAK(((rxbyte & GENMASK(15, 8)) == 0), return scanrr);
		DM9051_RX_BREAK(!eval_rxb(db, &rxbyte), return -EINVAL);
		//[dbg] rxb
		db->bc.evaluate_rxb_counter = 0;

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		ret = dm9051_stop_mrcmd(db);
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		uint clk_ctrl = 0;

		dm9051_get_reg(db, DM9051_1588_CLK_CTRL, &clk_ctrl);

		if (clk_ctrl & DM9051_CCR_PTP_EN) {
			// 9051A PTP 打開後的 status 定義
			if (db->rxhdr.status & RSR_ERR_BITS_PTP || rxlen > DM9051_PKT_MAX) {
				dm_printk("dm9.Monitor headbyte/status/rxlen %2x %2x %04x\n",
					  db->rxhdr.headbyte,
					  db->rxhdr.status,
					  db->rxhdr.rxlen);

				if (db->rxhdr.status & RSR_ERR_BITS) {
					db->bc.status_err_counter++;
					dm_printk( "check rxstatus-error (%02x)\n",
						   db->rxhdr.status);
				} else {
					db->bc.large_err_counter++;
					dm_printk( "check rxlen large-error (%d > %d)\n",
						   rxlen, DM9051_PKT_MAX);
				}
				return dm9051_all_restart(db);
			}
			// bug fix: 只判斷 PTP 的 status 仍有一般的 status error (原因不明)
			// 加上 ptp_on 判斷可以避免
			if (db->rxhdr.status & RSR_RXTS_EN) {

				// 讀取 1588 Time Stamp
				ret = dm9051_read_mem(db, DM_SPI_MRCMD, &rxTSbyte[0], 8);
				if (ret) {
					netdev_dbg(ndev, "Read TimeStamp error: %02x\n", ret);
					return ret;
				}
				// print Time Stamp
				//dm_printk("[1588 Time Stamp] RX Packet Timestamp = %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
				//	  rxTSbyte[0], rxTSbyte[1], rxTSbyte[2], rxTSbyte[3],
				//	  rxTSbyte[4], rxTSbyte[5], rxTSbyte[6], rxTSbyte[7]);
			}

		} else {
			// 用原本的 status 定義
			if (db->rxhdr.status & RSR_ERR_BITS || rxlen > DM9051_PKT_MAX) {
				dm_printk("dm9.Monitor headbyte/status/rxlen %2x %2x %04x\n",
					  db->rxhdr.headbyte,
					  db->rxhdr.status,
					  db->rxhdr.rxlen);

				if (db->rxhdr.status & RSR_ERR_BITS) {
					db->bc.status_err_counter++;
					dm_printk( "check rxstatus-error (%02x)\n",
						   db->rxhdr.status);
				} else {
					db->bc.large_err_counter++;
					dm_printk( "check rxlen large-error (%d > %d)\n",
						   rxlen, DM9051_PKT_MAX);
				}
				return dm9051_all_restart(db);
			}
		}

                skb = dev_alloc_skb(rxlen);
		if (!skb) {
			ret = dm9051_dumpblk(db, DM_SPI_MRCMD, rxlen);
			if (ret)
				return ret;
			return scanrr;
		}

		rdptr = skb_put(skb, rxlen - 4);
		ret = dm9051_read_mem(db, DM_SPI_MRCMD, rdptr, rxlen);
		if (ret) {
			db->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			return ret;
		}

		ret = dm9051_stop_mrcmd(db);
		if (ret) {
			dev_kfree_skb(skb);
			return ret;
		}

		skb->protocol = eth_type_trans(skb, db->ndev);
		if (db->ptp_on && (db->rxhdr.status & RSR_RXTS_EN)) {
			dm9051_ptp_rx_hwtstamp(db, skb, rxTSbyte);
		}
		if (db->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	} while (!ret);

	//[dbg] nRx
	nloop_rx += scanrr;
	if (nloop_rx > 125) {
		dm_printk("_[nloop_rx CBLKRX %d, CBLKTX %d] %d\n", CBLKRX, CBLKTX, nloop_rx);
		nloop_rx = 0;
	}

	return scanrr;
}

/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 *
 *  will reference PTP class & PTP packet type,
 *  call dm9051_ptp_setup_before_tx()
 *  before this function call,
 */
static int dm9051_single_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = dm9051_write_mem(db, DM_SPI_MWCMD, buff, len);
	if (ret)
		return ret;

	ret = dm9051_set_regs(db, DM9051_TXPLL, &len, 2);
	if (ret < 0)
		return ret;

	u8 tcr = TCR_TXREQ;

        tcr |=  dm9051_ptp_tcr_flags(db);

	return dm9051_set_reg(db, DM9051_TCR, tcr);
}



static int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;
	int ret;

	while (!skb_queue_empty(&db->txq)) {
		struct sk_buff *skb;
		unsigned int len;
		unsigned int skb_single_len;
		unsigned int tmp, tmp_0, i;

		skb = skb_dequeue(&db->txq);
		dm9051_get_reg(db, DM9051_NSR, &tmp);
		if (skb) {
			ntx++;
			dm9051_ptp_setup_before_tx(db, skb);
			ret = dm9051_single_tx(db, skb->data, skb->len);
			skb_single_len = len = skb->len;
			//=================================================
			dm9051_get_reg(db, DM9051_NSR, &tmp_0);
			//dm_printk("0.1 register 0x01 = %x \r\n", tmp_0);
			if (tmp != tmp_0){
				i = 0;
				// check TX complete?
				do{
					dm9051_get_reg(db, DM9051_NSR, &tmp);
					//dm_printk("register 0x01 = %x \r\n", tmp);
					tmp_0 = tmp & 0x0c;
					i++;
				}while (tmp_0 != 0x0c);
				//dm_printk("i = 0x%x, register 0x01 = %x \r\n", i, tmp);
			}
			dm9051_ptp_process_after_tx(db, skb);
			dev_kfree_skb(skb);
			if (ret < 0) {
				db->bc.tx_err_counter++;
				return 0;
			}
			ndev->stats.tx_bytes += len;
			ndev->stats.tx_packets++;
		}

		if (netif_queue_stopped(ndev) &&
		    (skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

static irqreturn_t dm9051_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int result, result_tx;

	mutex_lock(&db->spi_lockm);

	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do {
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
	} while (result > 0);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}

static void dm9051_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int result;

	mutex_lock(&db->spi_lockm);

	result = dm9051_loop_tx(db);
	if (result < 0)
		netdev_err(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

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

#if 1
#define DM_TIMER_EXPIRE1	1
#define DM_TIMER_EXPIRE2	0
#define DM_TIMER_EXPIRE3	0

static void dm9051_irq_delayp(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_workp);

//	if (db->poll_count == 0)
//		rx_pointers_monitor(db);
//	db->poll_count++;

	dm9051_rx_threaded_irq(0, db); // 0 is no-used.
	/* consider not be 0, to alower and not occupy almost all CPU resource.
	 * This is by CPU scheduling-poll, so is software driven!
	 */
	//[dbg].poll.extream.fast.[dbg]
	schedule_delayed_work(&db->irq_workp, DM_TIMER_EXPIRE2);
	//schedule_delayed_work(&db->irq_workp, DM_TIMER_EXPIRE1);
}
#endif

/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;

	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */
#if 0
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
				   dm9051_irq_flag(db) | IRQF_ONESHOT,
				   ndev->name, db);
	if (ret < 0) {
		netdev_err(ndev, "failed to get irq\n");
		return ret;
	}
	//printk("dm9051_irq_flag(db) %d\n", dm9051_irq_flag(db));
	//printk("request_irq, irqno %d, IRQF_TRIGGER_LOW %d, IRQF_TRIGGER_HIGH %d\n", spi->irq, IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
#elif 1
	//~printk("dm9051_irq_flag(db) %d\n", dm9051_irq_flag(db));
	//~printk("request_irq, irqno %d, IRQF_TRIGGER_LOW %d, IRQF_TRIGGER_HIGH %d\n", spi->irq, IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
#endif

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	ret = dm9051_all_start(db);
	if (ret) {
		phy_stop(db->phydev);
		free_irq(spi->irq, db);
		return ret;
	}

	netif_wake_queue(ndev);

#if 1
	//if (threadedcfg.interrupt_supp == THREADED_POLL)
	schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
#endif
	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = dm9051_all_stop(db);
	if (ret)
		return ret;

#if 1
	cancel_delayed_work_sync(&db->irq_workp);
#endif

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

#if 0
	free_irq(db->spidev->irq, db);
#else
	printk("free_irq, irqno %d\n", db->spidev->irq);
#endif

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

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
	if (ndev->flags & IFF_PROMISC) {
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	rxctrl.rcr_all = rcr;

	/* broadcast address */
	rxctrl.hash_table[0] = 0;
	rxctrl.hash_table[1] = 0;
	rxctrl.hash_table[2] = 0;
	rxctrl.hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = ether_crc_le(ETH_ALEN, ha->addr) & GENMASK(5, 0);
		rxctrl.hash_table[hash_val / 16] |= BIT(0) << (hash_val % 16);
	}

	/* schedule work to do the actual set of the data if needed */

	if (memcmp(&db->rctl, &rxctrl, sizeof(rxctrl))) {
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
	return dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
}

static int dm9051_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	//struct board_info *db = to_dm9051_board(ndev);
	struct mii_ioctl_data *data = if_mii(rq);


#if 1 //[JJ: programming on 20230221]
	//[int result = 0;]
	switch(cmd) {
		case SIOCGMIIPHY:
			dm_printk("[in %s()] *SIOCGMIIPHY*\n", __FUNCTION__);
			data->phy_id = DM9051_PHY_ADDR;
                        break;

/*Stone add for 1588
  case SIOCGMIIREG:
  if (phycfg.setup_phy & PHYCFG_LIB)
  data->val_out = dm9051_mdio_read(db->mdiobus,
  DM9051_PHY_ADDR, //phydev->mdio.addr
  data->reg_num & 0x1f);
  else
  do {
  unsigned int val;
  dm9051_phyread(db, data->reg_num & 0x1f, &val);
  data->val_out = val;
  } while(0);
  break;

  case SIOCSMIIREG:

  if (phycfg.setup_phy & PHYCFG_LIB)
  dm9051_mdio_write(db->mdiobus,
  DM9051_PHY_ADDR, //phydev->mdio.addr
  data->reg_num & 0x1f,
  data->val_in);
  else

  dm9051_phywrite(db, data->reg_num & 0x1f, data->val_in);

  break;
*/

			//Stone add for 1588
		case SIOCGHWTSTAMP:
			//Stone add for 1588 debug message 2024-04-19
			dm_printk("[in %s()] *SIOCGHWTSTAMP*\n", __FUNCTION__);
			return dm9051_ptp_get_ts_config(ndev, rq);

		case SIOCSHWTSTAMP:
			dm_printk("[in %s()] *SIOCSHWTSTAMP*\n", __FUNCTION__);
			return dm9051_ptp_set_ts_config(ndev, rq);

		default:
			dm_printk("[in %s() cmd %x \r\n", __FUNCTION__, cmd);
			return -EOPNOTSUPP;
	}
#endif
	return 0;
}

static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
	.ndo_eth_ioctl = dm9051_netdev_ioctl,
};

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;

	db->bc.evaluate_rxb_counter = 0;
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
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
		 "dm9051-%s.%u", dev_name(&spi->dev), spi_get_chipselect(spi,0)); //KT6631 with 'spi_get_chipselect(spi, 0)'

	ret = devm_mdiobus_register(&spi->dev, db->mdiobus);
	if (ret)
		dev_err(&spi->dev, "Could not register MDIO bus\n");

	return ret;
}

static void dm9051_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	phy_print_status(db->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link) {
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
		dm9051_update_fcr(db);
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

	db->msg_enable = 0;
	db->spidev = spi;
	db->ndev = ndev;

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;

	// regmap use this lock
	mutex_init(&db->spi_lockm);
	// big lock
	mutex_init(&db->reg_mutex);
	// lock for time stamp
	mutex_init(&db->tsreg_lock);
//	mutex_init(&db->phy_lockm);

	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

	INIT_DELAYED_WORK(&db->irq_workp, dm9051_irq_delayp); //='dm9051_continue_poll'

	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

	ret = dm9051_map_etherdev_par(ndev, db);
	if (ret < 0)
		return ret;

	ret = dm9051_mdio_register(db);
	if (ret)
		return ret;

	ret = dm9051_phy_connect(db);
	if (ret)
		return ret;

	dm9051_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = devm_register_netdev(dev, ndev);
	if (ret) {
		phy_disconnect(db->phydev);
		return dev_err_probe(dev, ret, "device register failed");
	}

        db->ptp_on = 1;
	if (db->ptp_on)
		dm9051_ptp_init(db);

	return 0;
}

static void dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);
	if (db->ptp_on) {
		dm9051_ptp_stop(db);
	}
	phy_disconnect(db->phydev);
}

static const struct of_device_id dm9051_match_table[] = {
	{ .compatible = "davicom,dm9051" },
	{}
};

static const struct spi_device_id dm9051_id_table[] = {
	{ "dm9051", 0 },
	{}
};

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

EXPORT_SYMBOL(dm9051_set_reg);
EXPORT_SYMBOL(dm9051_get_reg);
EXPORT_SYMBOL(dm9051_set_regs);
EXPORT_SYMBOL(dm9051_get_regs);
EXPORT_SYMBOL(dm9051_write_mem);
EXPORT_SYMBOL(dm9051_read_mem);

MODULE_DESCRIPTION("Davicom DM9051 network SPI driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
