// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

/*
 * Spenser Driver version: 
 * V1.0
 * 		- Added Checksum Offload feature.
 * V1.1
 * 		- Add ethtool statistics report
 * 		- Disable discard CRC error
 * V1.2
 * 		- Add interrupt mode
 * V1.3
 * 		- Fixed Xiaomi Switch
 * V1.4
 * 		- Supports PTP software timestamp
 * V1.5
 * 		- Supports PTP hardware timestamp
 * 		- UDPV4 Master/Slave Roles, one/two step ready.
 * 
 * 
 */
 
#include <linux/etherdevice.h>
//#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/mii.h>
//#include <linux/module.h>
//#include <linux/netdevice.h>
#include <linux/phy.h>
//#include <linux/regmap.h>
//#include <linux/skbuff.h>
#include <linux/spinlock.h>
//#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/of.h>

//Spenser
#include <linux/timekeeping.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>


#include "dm9051.h"
#include "dm9051_ptp.h"

#define DRVNAME_9051	"dm9051"

//#define	LNX_DM9051_RELEASE_VERSION	"lnx_dm9051_kt6631_r2412_v3.5_xiamomi"
#define	LNX_DM9051_RELEASE_VERSION	"dm9051a_spenser_v1.5"
#define	force_monitor_rxb	1

#define SCAN_BL(dw)		(dw & GENMASK(7, 0))
#define SCAN_BH(dw)		((dw & GENMASK(15, 8)) >> 8)

#if 0
/**
 * struct rx_ctl_mach - rx activities record
 * @status_err_counter: rx status error counter
 * @large_err_counter: rx get large packet length error counter
 * @rx_err_counter: receive packet error counter
 * @tx_err_counter: transmit packet error counter
 * @fifo_rst_counter: reset operation counter
 *
 * To keep track for the driver operation statistics
 */
struct rx_ctl_mach {
	u16				status_err_counter;
	u16				large_err_counter;
	u16				rx_err_counter;
	u16				tx_err_counter;
	u16				fifo_rst_counter;
	
	u16				evaluate_rxb_counter;
};

/**
 * struct dm9051_rxctrl - dm9051 driver rx control
 * @hash_table: Multicast hash-table data
 * @rcr_all: KS_RXCR1 register setting
 *
 * The settings needs to control the receive filtering
 * such as the multicast hash-filter and the receive register settings
 */
struct dm9051_rxctrl {
	u16				hash_table[4];
	u8				rcr_all;
};

/**
 * struct dm9051_rxhdr - rx packet data header
 * @headbyte: lead byte equal to 0x01 notifies a valid packet
 * @status: status bits for the received packet
 * @rxlen: packet length
 *
 * The Rx packed, entered into the FIFO memory, start with these
 * four bytes which is the Rx header, followed by the ethernet
 * packet data and ends with an appended 4-byte CRC data.
 * Both Rx packet and CRC data are for check purpose and finally
 * are dropped by this driver
 */
struct dm9051_rxhdr {
	u8				headbyte;
	u8				status;
	__le16				rxlen;
};

/**
 * struct board_info - maintain the saved data
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @regmap_dm: regmap for register read/write
 * @regmap_dmbulk: extra regmap for bulk read/write
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @pause: ethtool pause parameter structure
 * @spi_lockm: between threads lock structure
 * @reg_mutex: regmap access lock structure
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @rctl: rx control setting structure
 * @msg_enable: message level value
 * @imr_all: to store operating imr value for register DM9051_IMR
 * @lcr_all: to store operating rcr value for register DM9051_LMCR
 *
 * The saved data variables, keep up to date for retrieval back to use
 */
struct board_info {
	u32				msg_enable;
	struct spi_device		*spidev;
	struct net_device		*ndev;
	struct mii_bus			*mdiobus;
	struct phy_device		*phydev;
	struct sk_buff_head		txq;
	struct regmap			*regmap_dm;
	struct regmap			*regmap_dmbulk;
	struct work_struct		rxctrl_work;
	struct work_struct		tx_work;
	struct work_struct		irq_workp;
#if 0
  struct delayed_work		irq_workp;
#endif
	struct ethtool_pauseparam	pause;
	struct mutex			spi_lockm;
	struct mutex			reg_mutex;
	struct mutex	phy_lockm;
	struct rx_ctl_mach		bc;
	struct dm9051_rxhdr		rxhdr;
	struct dm9051_rxctrl		rctl;
	u8				imr_all;
	u8				lcr_all;
	// Spenser - ptp
	struct ptp_clock                *ptp_clock;
	struct ptp_clock_info 			ptp_info;
};

#endif

//[dbg].
#if 0
#if 1
int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb)
{
	int ret;

	ret = regmap_read(db->regmap_dm, reg, prb);
	if (ret < 0)
			netif_err(db, drv, db->ndev, "%s: error %d get reg %02x\n",
				  __func__, ret, reg);
	return ret;
}
#endif
#endif

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
		*p++ = (u8) SCAN_BL(rb);
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
	int ret;

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
		*p++ = (u8) SCAN_BL(rb);
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
//Spenser  100, 210000
	//printk("dm9051_nsr_poll...\n");
	
	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
				       mval & (NSR_TX2END | NSR_TX1END), 2, 210000); //<- 20
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

static int dm9051_ncr_poll(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
				       !(mval & NCR_RST), 10, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "timeout in checking for ncr reset\n");

/*	
//Spenser
		regmap_read(db->regmap_dm, 0x03, &mval);
		if(mval && 0xFF)
		{
			printk("TX Status Register I = 0x%02X\n", mval);
			db->bc.tx_err_counter++;
		}
		regmap_read(db->regmap_dm, 0x04, &mval);
		if(mval && 0xFF)
		{
			printk("TX Status Register II = 0x%02X\n", mval);
			db->bc.tx_err_counter++;
		}
*/

	return ret;
}

static int dm9051_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	//printk("Private interrupt type = %d\n", irq_type);
	
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
	
	//return ((dm9051_irq_flag(db) == IRQF_TRIGGER_LOW) || (dm9051_irq_flag(db) == IRQF_TRIGGER_FALLING	)) ?
	//	INTCR_POL_LOW : INTCR_POL_HIGH;
	
	if ((dm9051_irq_flag(db) == IRQF_TRIGGER_LOW) || (dm9051_irq_flag(db) == IRQF_TRIGGER_FALLING	)) {
		printk("INTCR_POL_LOW...\n");
		return INTCR_POL_LOW;
	}else{
		return INTCR_POL_HIGH;
	}
}

static int dm9051_set_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return dm9051_set_reg(db, DM9051_FCR, fcr); //flow control / back pressure
}

static int dm9051_set_recv(struct board_info *db)
{
	int ret;

	ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all); /* enable rx */
}

static int dm9051_ndo_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	//netdev_features_t changed = ndev->features ^ features;
	struct board_info *db = to_dm9051_board(ndev);
#if 0
	if (db->ptp_on) {
		printk("PTP enable must be disable TX/RX checksum\n");
        //hardware_disable_rx_csum();
        dm9051_set_reg(db, 0x31, 0x0);
        dm9051_set_reg(db, 0x32, 0x0);
	}else{
#endif		
		if ((features & NETIF_F_RXCSUM) && (features & NETIF_F_HW_CSUM)) {
			printk("Enabling TX/RX checksum\n");
			dm9051_set_reg(db, 0x31, 0x7);
            dm9051_set_reg(db, 0x32,0x3);
		} else if (features & NETIF_F_RXCSUM) {	//  && (features & NETIF_F_HW_CSUM)
            printk("Enabling RX checksum only\n");            
            dm9051_set_reg(db, 0x31, 0x0);
            dm9051_set_reg(db, 0x32,0x3);
		} else if(features & NETIF_F_HW_CSUM) {
			printk("Enabling TX checksum only\n");
			dm9051_set_reg(db, 0x31, 0x7);
			dm9051_set_reg(db, 0x32,0x0);
		} else {
            printk("Disabling TX/RX checksum\n");
            //hardware_disable_rx_csum();
            dm9051_set_reg(db, 0x31, 0x0);
            dm9051_set_reg(db, 0x32,0x0);
		}
	//}
    
	return 0;
}



static int dm9051_core_reset(struct board_info *db)
{
	int ret;
	//unsigned int val;
	//unsigned int mval, mval1;

	db->bc.fifo_rst_counter++;

	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;
	
	dm9051_ncr_poll(db);

/*		
	//Spenser - Interrupt debug
	printk("dm9051_core_reset...after NCR reset\n");
	regmap_read(db->regmap_dm, 0x39, &val);
	printk("ICR0x39 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x54, &val);
	printk("IPCOC0x54 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("ISR0x7E = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("IMR0x7F = 0x%x\n", val); 
*/
	
	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, MBNDRY_BYTE); /* MemBound */
	if (ret)
		return ret;

//Spenser	- Setup Interrupt Mode	
	//ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_UNLIMIT); /* from PPCR_PAUSE_COUNT, Pause Count */
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, 0xF);
	if (ret)
		return ret;
	//Spenser - interrupt work run
	dm9051_set_reg(db, 0x54, 0xC1);
	
	//Enable Checksum Offload
	/*
	ret = regmap_write(db->regmap_dm, 0x31, 0x7);
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, 0x32, 0x2);
	if (ret)
		return ret;
	*/	
	/*
	dm9051_set_reg(db, 0x31, 0x7);
	dm9051_set_reg(db, 0x32,0x3);
	printk("Enable TX/RX checksum offload\n");
	*/
	
	dm9051_ndo_set_features(db->ndev, db->ndev->features);
		
		
/*		
	ret = regmap_write(db->regmap_dm, 0x09, 0x8A);
	if (ret)
		return ret;	
*/	
	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;
		
//Spenser - Save 1588 Rate register value

	u32 rate_reg = 0; //15888, dm9051_get_rate_reg(db);
	printk("Pre-RateReg value = 0x%08X\n", rate_reg);

#if 0
	u8 mRate[4];
	//mutex_lock(&db->spi_lockm);
	dm9051_set_reg(db, 0x69, 0x01);
	dm9051_set_reg(db, DM9051_1588_CLK_CTRL, DM9051_CCR_IDX_RST);
	regmap_noinc_read(db->regmap_dm, 0x68, mRate, 4);
	db->pre_rate = ((uint32_t)mRate[3] << 24) | ((uint32_t)mRate[2] << 16) |
		((uint32_t)mRate[1] << 8) | (uint32_t)mRate[0];	
	//mutex_unlock(&db->spi_lockm);
	printk("Pre-RateReg value = %u\n", db->pre_rate);
#endif
	return ret; //return dm9051_set_reg(db, DM9051_INTCR, dm9051_intcr_value(db));
}


static int dm9051_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
	//Spenser
		//fcr |= FCR_BKPA | FCR_BKPM | FCR_FLCE;
		fcr |= FCR_BKPA | FCR_FLCE;
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
	//Spenser - interrupt work run
	//regmap_write(db->regmap_dm, 0x54, 0x88);

//Spenser
#if 0	
	return dm9051_set_reg(db, DM9051_IMR, 0x83);
#else
	return dm9051_set_reg(db, DM9051_IMR, db->imr_all); /* enable int */
#endif	
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

	mutex_lock(&db->spi_lockm);
	
	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret) 
		goto unlock;
		

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR);
	if (ret)
		goto unlock;

	ret = dm9051_epcr_poll(db);
	if (ret)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		goto unlock;
		
	ret = regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, to, 2);
	
unlock:
	mutex_unlock(&db->spi_lockm);
	
	return ret;
	
}

static int dm9051_eeprom_write(struct board_info *db, int offset, u8 *data)
{
	int ret;

	mutex_lock(&db->spi_lockm);
	
	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		goto unlock;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, data, 2);
	if (ret < 0)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret)
		goto unlock;

	ret = dm9051_epcr_poll(db);
	if (ret)
		goto unlock;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
unlock:
	mutex_unlock(&db->spi_lockm);
	
	return ret;
	

}

static int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

	mutex_lock(&db->phy_lockm);
	
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
	mutex_unlock(&db->phy_lockm);

	return ret;
}

static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	//printk("phywrite: reg= 0x%x, val= 0x%x\n",reg, val);
	
	mutex_lock(&db->phy_lockm);
		
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
	mutex_unlock(&db->phy_lockm);
	
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
/*		
//Spenser - workaround, didn't need			
		if(regnum == 0x04) {
			if ((val & 0x2000) || !(val & 0x1)) {	
				printk("Read Remote Fault or Protocol selection = 0x%X\n", val);
				val = 0x5e1;
				printk("Fake ");							
				dm9051_phywrite(db, regnum, val);
			}
		}
*/
		unsigned int tmp;
		if(regnum == 0x05) {
			dm9051_phyread(db, 0x04, &tmp); 
		}
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
/*
//Spenser - 		
		else if(regnum == 0x04) {
			if (val & 0x2000) {				
				printk("Write Remote Fault = 0x%X\n", val);
				return dm9051_phywrite(db, regnum, 0x5e1);
			}else{
				return dm9051_phywrite(db, regnum, val);
			}
		}
*/
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
		
		struct regmap *__devm_regmap_init_spi_dm(struct spi_device *spi,
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
	if (wid != DM9051_ID) {
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
	//printk("dm9051_get_drvinfo...\n");
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

static const char dm9051_stats_strings[][ETH_GSTRING_LEN] = {
    "rx_packets",
    "tx_packets",
    "rx_errors",
    "tx_errors",
    "rx_bytes",
    "tx_bytes",
    "fifo_rst",
};

static void dm9051_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
    if (stringset == ETH_SS_STATS)
        memcpy(data, dm9051_stats_strings, sizeof(dm9051_stats_strings));
}

static int dm9051_get_sset_count(struct net_device *netdev, int sset)
{
    if (sset == ETH_SS_STATS)
        return ARRAY_SIZE(dm9051_stats_strings);
    return 0;
}

static void dm9051_get_ethtool_stats(struct net_device *ndev,
                                     struct ethtool_stats *stats, u64 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	
	ndev->stats.tx_errors = db->bc.tx_err_counter;
	ndev->stats.rx_errors = db->bc.status_err_counter;
	
    data[0] = ndev->stats.rx_packets;
    data[1] = ndev->stats.tx_packets;
    data[2] = ndev->stats.rx_errors;
    data[3] = ndev->stats.tx_errors;
    data[4] = ndev->stats.rx_bytes;
    data[5] = ndev->stats.tx_bytes;
    data[6] = db->bc.fifo_rst_counter - 1;	//Subtract Initial reset
}

int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
{
	struct board_info *db = netdev_priv(net_dev);
	
//Spenser - get phc_index	
	//info->phc_index = -1;
	info->phc_index = db->ptp_clock ? ptp_clock_index(db->ptp_clock) : -1;


	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types =
		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
		BIT(HWTSTAMP_TX_OFF) |
		BIT(HWTSTAMP_TX_ON);

	info->rx_filters =
		BIT(HWTSTAMP_FILTER_NONE) |
		BIT(HWTSTAMP_FILTER_ALL);


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
	.get_strings		= dm9051_get_strings,
	.get_sset_count		= dm9051_get_sset_count,
	.get_ethtool_stats	= dm9051_get_ethtool_stats,
	.get_ts_info = dm9051_ts_info,
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
	ret = dm9051_phywrite(db, 0, 0x3100);
	if (ret)
		return ret;
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
		
//Spenser - start PHY 
	//dm9051_phywrite(db, 0x04, 0x1e1);	//only for test
	phy_support_sym_pause(db->phydev);
	//printk("phy_start...\n");
	phy_start(db->phydev);
	//printk("phy_start-->\n");
	
#endif

/*
//Spenser
	ret = dm9051_set_recv(db);
	if (ret)
		return ret;
*/
		
	return dm9051_enable_interrupt(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

//Spenser
	dm9051_clear_interrupt(db);
	
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
	//printk("_[dm9051_all_stop] BMCR 0x3900: power down (warn)\n");
	ret = dm9051_phywrite(db, 0, 0x3900);
	if (ret)
		return ret;
	//printk("_[dm9051_all_stop] OK\n");
	
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
	
	//Spenser - interrupt work run
	//dm9051_set_reg(db, 0x54, 0x88);
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
#define DM9051_RX_BREAK(exp, yhndlr, nhndlr) \
	do {	\
		if ((exp)) {	\
			yhndlr;	\
		} else {	\
			nhndlr;	\
		}	\
	} while(0)

void trap_clr(struct board_info *db)
{
	db->bc.evaluate_rxb_counter = 0;
}

#if 0
/*int env_evaluate_rxb(struct board_info *db, unsigned int rxbyte)
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
}*/
#endif

// check rxbs
// return: 0 : Invalid 
//         1 : Pkt_rdy
#if 0
/*int eval_rxbz(struct board_info *db, unsigned int *prxbyte)
{
	static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };
	u8 *bf = (u8 *) prxbyte;
	int n = 0;
	char pbff[80];
	unsigned int i;
	int ret;

	if (SCAN_BH(*prxbyte) == 0x01)
		return 1;

	printk("_[  bf] %02x %02x\n", bf[0], bf[1]);
	printk("_[rxbs] %02lx %02lx\n", SCAN_BL(*prxbyte), SCAN_BH(*prxbyte));

	inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
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
}*/

//-----------------
/*void test(void) {
		#if 1
		//DM9051_RX_BREAK0(((rxbyte & GENMASK(15, 8)) == 0), return scanrr);
		//DM9051_RX_BREAK0(!eval_rxbz(db, &rxbyte), return -EINVAL);
		#else
			//[dbg] rxb
			#if 1
			static int tst_code = 0x33;
			#endif
			//[dbg] rxb
			ret = dm9051_get_reg(db, DM_SPI_MRCMDX, &rxbyte); //regmap_write
			if (ret)
				return ret;
			ret = dm9051_get_reg(db, DM_SPI_MRCMDX, &rxbyte);
			if (ret)
				return ret;
			#if 1
			if (tst_code) {
				rxbyte = tst_code--;
				if (tst_code == 0x30)
					tst_code = 0;
			}
			#endif
			//[dbg] rxb
			DM9051_RX_BREAK0(((rxbyte & GENMASK(7, 0)) != 1) && ((rxbyte & GENMASK(7, 0)) != 0),
							return env_evaluate_rxb(db, rxbyte & GENMASK(7, 0)));
			DM9051_RX_BREAK0(((rxbyte & GENMASK(7, 0)) == 0),
							return scanrr);
		#endif
}*/
#endif

void monitor_rxb0(unsigned int rxbyte)
{
#if force_monitor_rxb
	static int rxbz_counter = 0;
	static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };
	unsigned int i;
	int n = 0;
	char pbff[80];
	
	u8 *bf = (u8 *) &rxbyte; //tested

	if (SCAN_BL(rxbyte) == 1 && SCAN_BH(rxbyte) != 1)
		printk("-. ........ warn, spenser board ...BL %02lx BH %02lx.......... on .monitor_rxb0 %2d\n",
			SCAN_BL(rxbyte), SCAN_BH(rxbyte), rxbz_counter);
	
	if (SCAN_BL(rxbyte) == 1 || SCAN_BL(rxbyte) == 0)
		return;

	printk("_.moni   bf] %02x %02x\n", bf[0], bf[1]); //tested
	printk("_.moni rxbs] %02lx %02lx\n", SCAN_BL(rxbyte), SCAN_BH(rxbyte)); //tested

	inval_rxb[rxbz_counter] = SCAN_BL(rxbyte);
	rxbz_counter++;

	n += sprintf(pbff+n, "_.monitor_rxb0 %2d]", rxbz_counter);
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
		printk("_[Less constrain of old SCAN_BL trap's, NOT dm9051_all_restart] only monitored.\n");
		//ret = dm9051_all_restart(db); //
	}
#endif
}

// check rxbs
// return: 0 : Still not trap 
//         1 : Do reatart trap
int trap_rxb(struct board_info *db, unsigned int *prxbyte)
{
	if (SCAN_BH(*prxbyte) == 0)
		return 0;

	do {
		static unsigned int inval_rxb[TIMES_TO_RST] = { 0 };
		u8 *bf = (u8 *) prxbyte;
		int ret, n = 0;
		unsigned int i;
		char pbff[80];

		printk("_.eval   bf] %02x %02x\n", bf[0], bf[1]);
		printk("_.eval rxbs] %02lx %02lx\n", SCAN_BL(*prxbyte), SCAN_BH(*prxbyte));

		inval_rxb[db->bc.evaluate_rxb_counter] = SCAN_BH(*prxbyte);
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
			trap_clr(db);
			memset(inval_rxb, 0, sizeof(inval_rxb));
			ret = dm9051_all_restart(db); //...
			if (!ret)
				printk("_[dm9051_all_restart] work around done\n");
			return 1;
		}
	} while(0);
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
	u8 rxTSbyte[8]; // Store 1588 Time Stamp
	//unsigned int mval;

	do {
		//[dbg].redundent.rxb.restore.[dbg]
		ret = dm9051_read_mem(db, DM_SPI_MRCMDX, &rxbyte, 2);
//Spenser
/*
		if (rxbyte == 0) {
			//printk("RX loop : %X\n", rxbyte);
			regmap_read(db->regmap_dm, 0x06, &mval);
			printk("RX Status Register: 0x%X\n", mval);
		}
*/
		//monitor_rxb0(rxbyte);
		DM9051_RX_BREAK(((SCAN_BH(rxbyte) & 0x03) == DM9051_PKT_RDY), trap_clr(db), return !trap_rxb(db, &rxbyte) ? scanrr : -EINVAL);

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		ret = dm9051_stop_mrcmd(db);
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);

//Spenser - check rx error when PTP
// Don't care RSR_LCS, RSR_PLE & RSR_AE	
			if (db->rxhdr.status & PTP_RSR_ERR_BITS || rxlen > DM9051_PKT_MAX) {
				printk("dm9.Monitor headbyte/status/rxlen %2x %2x %04x\n", 
					db->rxhdr.headbyte,
					db->rxhdr.status,
					db->rxhdr.rxlen);
				netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
					   db->rxhdr.headbyte);

				if (db->rxhdr.status & PTP_RSR_ERR_BITS) {
					db->bc.status_err_counter++;
					netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
						   db->rxhdr.status);
				} else {
					db->bc.large_err_counter++;
					netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n",
						   rxlen, DM9051_PKT_MAX);
				}
				return dm9051_all_restart(db);
			}
		
		
//Spenser - 1588
		
		if(db->rxhdr.status & RSR_RXTS_EN) {	// Inserted Timestamp
			//printk("Had RX Timestamp... rxstatus = 0x%x\n", db->rxhdr.status);
			if(db->rxhdr.status & RSR_RXTS_LEN) {	// 8 bytes Timestamp
				ret = dm9051_read_mem(db, DM_SPI_MRCMD, &rxTSbyte[0], 8);
				if (ret) {
					netdev_dbg(ndev, "Read TimeStamp error: %02x\n", ret);
					return ret;
				}
			}else{
				ret = dm9051_read_mem(db, DM_SPI_MRCMD, &rxTSbyte[0], 4);
				if (ret) {
					netdev_dbg(ndev, "Read TimeStamp error: %02x\n", ret);
					return ret;
				}
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
		//printk("skb->protocol = 0x%X\n", skb->protocol);
//Spenser - PTP
#if 0
		if (is_ptp_packet(skb)) {
			printk("RX PTP Packet\n");
			show_ptp_type(skb);	//Show PTP message type
			//skb_tx_timestamp(skb);	//Spenser - Report software Timestamp
		}
#endif		
		//if(db->rxhdr.status & RSR_RXTS_EN) {
		//if(db->ptp_on) {
		//	show_ptp_type(skb);	//Show PTP message type
			
			//15888, dm9051_ptp_rx_hwtstamp(db, skb, rxTSbyte);
		//}
		
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
		printk("_[nloop_rx CBLKRX %d, CBLKTX %d] %d\n", CBLKRX, CBLKTX, nloop_rx);
		nloop_rx = 0;
	}

	return scanrr;
}

#if 1
/*
 * return -
 * 0: not PTP packet
 * 1: one-step
 * 2: two-step 
 * 3: Not Sync packet
*/
static int dm9051_ptp_one_step(struct sk_buff *skb)
{
	struct ptp_header *hdr;
	unsigned int ptp_class;
	u8 msgtype;

	/* No need to parse packet if PTP TS is not involved */
	if (likely(!(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))) {
		return 0;	// Not PTP packet
	}else{
		//printk("Check PTP Message\n");
		/* Identify and return whether PTP one step sync is being processed */
		ptp_class = ptp_classify_raw(skb);
		if (ptp_class == PTP_CLASS_NONE)
			goto no;

		hdr = ptp_parse_header(skb, ptp_class);
		if (!hdr)
			goto no;
		
		msgtype = ptp_get_msgtype(hdr, ptp_class);
		if (msgtype == PTP_MSGTYPE_SYNC) {
			
			if (hdr->flag_field[0] & PTP_FLAG_TWOSTEP) {
				//printk("two-step TX Sync Message\n");
				return 2;
			}else {
				//printk("onestep TX Sync Message\n");
				return 1;
			}
		}else{
			//printk("Not Sync Message\n");
			return 3;
		}
		
	}
no:
	return 0;
}
#endif


/* transmit a packet,
 * return value,
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static int dm9051_single_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;
	//unsigned int mval, mval1;

	ret = dm9051_nsr_poll(db);
	if (ret)
		return ret;

	ret = dm9051_write_mem(db, DM_SPI_MWCMD, buff, len);
	if (ret)
		return ret;

	ret = dm9051_set_regs(db, DM9051_TXPLL, &len, 2);
	if (ret < 0)
		return ret;
//Spenser
	//Check checksum-offload registers
	//regmap_read(db->regmap_dm, 0x31, &mval);	
	//regmap_read(db->regmap_dm, 0x32, &mval1);
	//printk("0x31 = 0x%x, 0x32 = 0x%x\n", mval, mval1);
	
	u8 tcr = TCR_TXREQ; // TCR register value
	if (db->ptp_tx_flags) {
	switch(db->ptp_mode){
		case 1:
			//printk("One Step...\n");
			//Stone add for one-step Sync packet insert time stamp! 2024-08-14!
			tcr = (TCR_TS_EN | TCR_TXREQ | TCR_DIS_JABBER_TIMER);
			break;
		case 2:
		case 3:
			tcr = (TCR_TS_EN | TCR_TXREQ);
			break;
		default:
			//printk("Not PTP packet\n");
			break;
	}
	}
#if 0	
	if (db->ptp_onestep & db->ptp_tx_flags) {
		printk("One Step...\n");
		//Stone add for one-step Sync packet insert time stamp! 2024-08-14!
		tcr = (TCR_TS_EN | TCR_TXREQ | TCR_DIS_JABBER_TIMER);
	}else{	//Spenser - twostep
		tcr = (TCR_TS_EN | TCR_TXREQ);
	}
#endif
	
	return dm9051_set_reg(db, DM9051_TCR, tcr);
	//return dm9051_set_reg(db, DM9051_TCR, TCR_TXREQ);
}

static int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;
	int ret;

	while (!skb_queue_empty(&db->txq)) {
		struct sk_buff *skb;
		unsigned int len;
		//printk("dm9051_loop_tx inside while\n");
		skb = skb_dequeue(&db->txq);
		if (skb) {
			//printk("dm9051_loop_tx inside skb\n");
#if 0	//Spenser - remove these
//Spenser - ptp
			/* Check socket TX Timestamp */
			struct skb_shared_hwtstamps shhwtstamps;
			if (skb_shinfo(skb)->tx_flags & SKBTX_SW_TSTAMP) {
				//printk("PTP TX packet...tx_flags = 0x%x\n", skb_shinfo(skb)->tx_flags);
				ktime_t now = ktime_get_real(); // Get current time
				memset(&shhwtstamps, 0, sizeof(shhwtstamps));
				shhwtstamps.hwtstamp = now;
        
				/* insert timestamp */
				//skb_tstamp_tx(skb, &shhwtstamps);	//report hardware timestamp
				skb_tx_timestamp(skb);	// report software timestamp
			}
 // 
 #endif
   
			ntx++;
#if 0			
			if (dm9051_ptp_one_step_sync(skb)) {
				//printk("One Step...\n");
				db->ptp_onestep = true;
			}else{
				//printk("Two Step...\n");
				db->ptp_onestep = false;
			}
#endif			
			db->ptp_mode = dm9051_ptp_one_step(skb);
			
			ret = dm9051_single_tx(db, skb->data, skb->len);
			if (ret < 0)
				printk(" Fail: dm9051_single_tx()\n");
			len = skb->len;

//Spenser	
#if 0		
			//check TX packet H/W time stamp
			if (db->ptp_tx_flags & SKBTX_HW_TSTAMP) {
				printk("PTP message type = %X\n", skb->data[42]);
				
				ret = dm9051_nsr_poll(db);	//TX completed
				if (ret){
					printk("nsr_polling timeout\n");
					return ret;
				}
				//printk("PTP TX packet...tx_flags = 0x%x\n", skb_shinfo(skb)->tx_flags);
#endif				
				//Spenser - check ptp sync & delay req
				if (1) /*15888, (is_ptp_packet(skb))*/ {
					ret = dm9051_nsr_poll(db);	//TX completed
					if (ret){
						printk("nsr_polling timeout\n");
						return ret;
					}
					
					u8 message_type = (15888 & 0xff); //15888, get_ptp_message_type(skb);
					switch(message_type) {
						case 0:	//Sync
							//remark3-slave - none sync
							printk("TX Sync Timestamp\n");
							/*Spenser - Don't report HW timestamp to skb if one-step,
							 * otherwise master role will be not continue send Sync Message.
							*/
							if (db->ptp_mode == 2)	//two-step
								; //15888, dm9051_ptp_tx_hwtstamp(db, skb);	// Report HW Timestamp
							break;
						case 1:	//Delay Req
							//remark6-slave
							//printk("Tx Delay_Req Timestamp\n");
							
							//15888, dm9051_ptp_tx_hwtstamp(db, skb);	// Report HW Timestamp
							//printk("Tx Delay_Req Timestamp...\n");
							break;
						default:
							break;
					}
				}else{
					//printk("TX is not PTP packet..");
				}
				
				
				//dm9051_ptp_tx_hwtstamp(db, skb);	// Report HW Timestamp
				
			//}

			
			
			dev_kfree_skb(skb);
			if (ret < 0) {
				db->bc.tx_err_counter++;
				return 0;
			}
			ndev->stats.tx_bytes += len;
			ndev->stats.tx_packets++;
		}else{
			printk("skb null...\n");
		}

		if (netif_queue_stopped(ndev) &&
		    (skb_queue_len(&db->txq) < DM9051_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

//Spenser
#if 0
static irqreturn_t dm9051_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int result, result_tx;

	//printk("RX ISR...\n");
	
	mutex_lock(&db->spi_lockm);


//Spenser
	unsigned int val;
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("RX ISR ISR0x7E = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("RX ISR IMR0x7F = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x05, &val);
	printk("RX ISR RCR0x05 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x06, &val);
	printk("RX ISR RSR0x06 = 0x%x, ", val);

	
	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do {
		//printk("Go into loop_rx...\n");
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;

#if 1			
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
		// Spenser
		//printk("TX/RX loop...\n");
#endif		
	} while (result > 0);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}
#endif

static void dm9051_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int result;

	//printk("...dm9051_tx_delay()\n");
	//mutex_lock(&db->spi_lockm);

	result = dm9051_loop_tx(db);
	if (result < 0)
		netdev_err(db->ndev, "transmit packet error\n");

	//mutex_unlock(&db->spi_lockm);
}

//Spenser - marked for interrupt operation
#if 1
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
#endif

//Spenser - marked for interrupt operation
#if 0
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
	//Spenser - marked for interrupt operation
	//schedule_delayed_work(&db->irq_workp, DM_TIMER_EXPIRE2);
	
	//schedule_delayed_work(&db->irq_workp, DM_TIMER_EXPIRE1);
}
#endif

//Spenser
static void dm9051_irq_workp(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, irq_workp);
	int result;
	//int result_tx;
	
	//mutex_lock(&db->spi_lockm);
	dm9051_disable_interrupt(db);
	do {
		//printk("Go into loop_rx...\n");
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;

#if 0		
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
#endif		
		// Spenser
		//printk("TX/RX loop...\n");
		
	} while (result > 0);

out_unlock:
	dm9051_clear_interrupt(db);
	dm9051_enable_interrupt(db);
	//mutex_unlock(&db->spi_lockm);

}

static irqreturn_t dm9051_rx_irq_handler(int irq, void *pw)
{
	struct board_info *db = pw;
	int result;
	//int result_tx;
	//printk("RX ISR...\n");
	
	mutex_lock(&db->spi_lockm);

//Spenser
#if 0
	unsigned int val;
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("Before RX ISR ISR0x7E = 0x%x, ", val);

	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("RX ISR IMR0x7F = 0x%x, ", val);
	
	regmap_read(db->regmap_dm, 0x05, &val);
	printk("RX ISR RCR0x05 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x06, &val);
	printk("RX ISR RSR0x06 = 0x%x, ", val);
#endif

#if 1	
	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;
#endif
		
#if 1
	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;
#endif

#if 0	
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("After RX ISR ISR0x7E = 0x%x, ", val);
#endif		
	schedule_work(&db->irq_workp);
	
#if 0
	do {
		//printk("Go into loop_rx...\n");
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;

			
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
		// Spenser
		//printk("TX/RX loop...\n");
		
	} while (result > 0);
	
#endif

	dm9051_enable_interrupt(db);

#if 0
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("Leave RX ISR ISR0x7E = 0x%x, ", val);
#endif
	
	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;	
	
	
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
	//unsigned int val;

	
	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
//Spenser	
	//db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_RXEN;		//Disable discard CRC error
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	ndev->irq = spi->irq; /* by dts */

/*
//Spenser - Interrupt operation
	printk("dm9051_open...before request_threaded_irq\n");
	regmap_read(db->regmap_dm, 0x39, &val);
	printk("ICR0x39 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x54, &val);
	printk("IPCOC0x54 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("ISR0x7E = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("IMR0x7F = 0x%x\n", val); 
*/
#if 1

//Spenser -  interrupt
#if 0
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
				   dm9051_irq_flag(db) | IRQF_ONESHOT,
				   ndev->name, db);	// IRQF_ONESHOT
#endif

#if 1
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_irq_handler,
				   dm9051_irq_flag(db) | IRQF_ONESHOT,
				   ndev->name, db);	// IRQF_ONESHOT
#endif				   
	
	if (ret < 0) {
		printk("request_threaded_irq() return error code: %d irq = %d\n", ret, spi->irq);
		netdev_err(ndev, "failed to get irq\n");
		return ret;
	}
	//printk("dm9051_irq_flag(db) %d\n", dm9051_irq_flag(db));
	//printk("request_irq, irqno %d, IRQF_TRIGGER_LOW %d, IRQF_TRIGGER_HIGH %d\n", spi->irq, IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
//#elif 1
	//~printk("dm9051_irq_flag(db) %d\n", dm9051_irq_flag(db));
	//~printk("request_irq, irqno %d, IRQF_TRIGGER_LOW %d, IRQF_TRIGGER_HIGH %d\n", spi->irq, IRQF_TRIGGER_LOW, IRQF_TRIGGER_HIGH);
#endif

//Spenser move to all_start
/*
	phy_support_sym_pause(db->phydev);
	printk("phy_start...\n");
	phy_start(db->phydev);
*/

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	ret = dm9051_all_start(db);
	//printk("dm9051_all_start--> %d\n", ret);
	
	if (ret) {
		//printk("-->phy_stop\n");
		phy_stop(db->phydev);
		free_irq(spi->irq, db);
		return ret;
	}

/*
//Spenser - Interrupt debug
	printk("dm9051_open...after dm9051_all_start\n");
	regmap_read(db->regmap_dm, 0x39, &val);
	printk("ICR0x39 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x54, &val);
	printk("IPCOC0x54 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("ISR0x7E = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("IMR0x7F = 0x%x\n", val); 
*/	
	netif_wake_queue(ndev);

//Spenser - marked for interrupt operation
#if 0
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
	//unsigned int val;

	printk("-->dm9051_all_stop\n");
	
	ret = dm9051_all_stop(db);
	if (ret)
		return ret;

//Spenser - marked for interrupt operation		
#if 0
  cancel_delayed_work_sync(&db->irq_workp);
#endif

	flush_work(&db->tx_work);
	
	//Marked for Interrupt operation
	//flush_work(&db->rxctrl_work);

	//printk("-->phy_stop\n");
	phy_stop(db->phydev);

//Spenser - Interrupt operation
#if 1
	free_irq(db->spidev->irq, db);

/*	
	//Spenser - interrupt debug
	dm9051_set_reg(db, 0x39, 0);
	dm9051_set_reg(db, 0x7F, 0);
*/

#else
	printk("free_irq, irqno %d\n", db->spidev->irq);
#endif

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

/*
//Spenser - Interrupt debug
	printk("dm9051_open...before request_threaded_irq\n");
	regmap_read(db->regmap_dm, 0x39, &val);
	printk("ICR0x39 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x54, &val);
	printk("IPCOC0x54 = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7E, &val);
	printk("ISR0x7E = 0x%x, ", val);
	regmap_read(db->regmap_dm, 0x7F, &val);
	printk("IMR0x7F = 0x%x\n", val); 	
*/	

	return 0;
}


/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	
//Spenser
	//printk("dm9051_start_xmit...\n");
	db->ptp_tx_flags = skb_shinfo(skb)->tx_flags;
	if (db->ptp_tx_flags & SKBTX_HW_TSTAMP)
		db->ptp_tx_flags |= SKBTX_IN_PROGRESS;
	
	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

//Spenser - PTP	

	if (1) /*15888, (is_ptp_packet(skb))*/ {
		//remark3-slave
		//show_ptp_type(skb);	//Show PTP message type
		skb_tx_timestamp(skb);	//Spenser - Report software Timestamp
	}
	
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
		//Spenser - Marked for Interrupt operation
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

static struct net_device_stats *dm9051_get_stats(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	
	//db->bc.tx_err_counter = 1;
	
	ndev->stats.tx_errors = db->bc.tx_err_counter;
	ndev->stats.rx_errors = db->bc.status_err_counter + db->bc.large_err_counter;
	return &ndev->stats;
}

#if 0
static int configure_hardware_timestamping(struct net_device *dev, struct hwtstamp_config *config)
{
	struct board_info *db = to_dm9051_board(dev);
	
    // Configure the hardware according to config->tx_type and config->rx_filter.
    if (config->tx_type == HWTSTAMP_TX_ON) {
        // Enable hardware to send timestamps
        priv->hw->enable_tx_timestamping();
    } else {
        // Disable hardware send timestamps
        priv->hw->disable_tx_timestamping();
    }

    if (config->rx_filter == HWTSTAMP_FILTER_ALL) {
        // Enable hardware to receive timestamps
        priv->hw->enable_rx_timestamping();
    } else {
        // Disable hardware to receive timestamps
        priv->hw->disable_rx_timestamping();
    }

    return 0;
}
#endif

static int dm9051_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	//struct board_info *db = to_dm9051_board(ndev);
    //struct hwtstamp_config config;

	
	
	switch(cmd) {
		case SIOCGHWTSTAMP:
			//printk("Process SIOCGHWTSTAMP\n");
			//db->ptp_on = 1;
			return 0; //15888, dm9051_ptp_get_ts_config(ndev, rq);
		case SIOCSHWTSTAMP:
			//printk("Process SIOCSHWTSTAMP\n");
			//db->ptp_on = 1;
			return 0; //15888, dm9051_ptp_set_ts_config(ndev, rq);
		default:
			printk("dm9051_netdev_ioctl cmd = 0x%X\n", cmd);
			//db->ptp_on = 0;
			return -EOPNOTSUPP;
	}
	
#if 0	
    if (cmd == SIOCSHWTSTAMP) {
		printk("Process SIOCSHWTSTAMP\n");
		
        if (copy_from_user(&config, rq->ifr_data, sizeof(config)))
            return -EFAULT;

        // Check if the configuration is supported
        if (config.flags) // flags are not supported
            return -EINVAL;


        switch (config.tx_type) {
        case HWTSTAMP_TX_OFF:
        case HWTSTAMP_TX_ON:
            break;
        default:
            return -ERANGE;
        }

        switch (config.rx_filter) {
        case HWTSTAMP_FILTER_NONE:
        case HWTSTAMP_FILTER_ALL:
            break;
        default:
            return -ERANGE;
        }

#if 0
        // Configuring Hardware Timestamps
        if (configure_hardware_timestamping(dev, &config) < 0)
            return -EOPNOTSUPP;

        // Returning Configuration to Userspace
        if (copy_to_user(ifr->ifr_data, &config, sizeof(config)))
            return -EFAULT;
#endif
		
//Spenser
		db->tstamp_config = config;	// Store config to privae data
		
        return 0;
    }
	printk("No support SIOCSHWTSTAMP\n");
    return -EOPNOTSUPP;
#endif


}

static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
	.ndo_set_features	= dm9051_ndo_set_features,
	.ndo_get_stats		= dm9051_get_stats,
	.ndo_eth_ioctl = dm9051_netdev_ioctl,
};

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;
	
	trap_clr(db); //db->bc._evaluate_rxb_counter = 0;
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
//Spenser	
	//db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->phy_mask = 0x01;
	db->mdiobus->parent = &spi->dev;
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
		 "dm9051-%s.%u", dev_name(&spi->dev), spi->chip_select); //KT6631 with 'spi_get_chipselect(spi, 0)'

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
	db->ptp_on = 1;		//Enable PTP must disable checksum_offload

	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;


//Spenser - Setup for Checksum Offload
	/* Set default features */
	//ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	/* don't need if supports software timestamp only */
	// Enable Timestamp function
	//ndev->hw_features |= NETIF_F_HW_TS;
	//ndev->features |= NETIF_F_HW_TS;
	if (!db->ptp_on) {
		ndev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	}
	ndev->hw_features |= ndev->features;
	
	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);
	mutex_init(&db->phy_lockm);

//Spenser
	INIT_WORK(&db->rxctrl_work, dm9051_rxctl_delay);
	INIT_WORK(&db->tx_work, dm9051_tx_delay);

//Spenser - Marked for Interrupt operation
#if 0
  INIT_DELAYED_WORK(&db->irq_workp, dm9051_irq_delayp); //='dm9051_continue_poll'
#endif
	INIT_WORK(&db->irq_workp, dm9051_irq_workp); 
	
	ret = dm9051_map_init(spi, db);
	if (ret)
		return ret;

	printk("\n");
	dev_info(dev, "Davicom: %s", LNX_DM9051_RELEASE_VERSION);
#if 1
	//[dbg].[rel] spi.speed
	do {
		unsigned int speed;
		of_property_read_u32(spi->dev.of_node, "spi-max-frequency", &speed);
		dev_info(dev, "SPI speed from DTS: %d Hz\n", speed);
		//printk("_[probe] SPI speed from DTS: %d Hz\n", speed);
	} while(0);
#endif

	ret = dm9051_map_chipid(db);
	if (ret)
		return ret;

	ret = dm9051_map_etherdev_par(ndev, db);
	if (ret < 0)
		return ret;

	ret = dm9051_mdio_register(db);
	if (ret)
		return ret;
//Spenser - set PHY Reg04 = 0x5e1
	//printk("Initil PHY REG04 to 0x5e1\n");
	//dm9051_phywrite(db, 0x04, 0x5e1);
	
	ret = dm9051_phy_connect(db);
	if (ret){
		printk("PHY connect fail...\n");
		return ret;
	}else{
		printk("PHY connected...\n");
	}
	
	dm9051_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = devm_register_netdev(dev, ndev);
	if (ret) {
		phy_disconnect(db->phydev);
		return dev_err_probe(dev, ret, "device register failed");
	}

//Spenser - ptp initial
    db->ptp_on = 1;
	if (db->ptp_on)
		; //dm9051_ptp_init(db); //15888

	
	return 0;
}

static void dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);
	if(db->ptp_clock)
		; //dm9051_ptp_stop(db); //15888
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
//EXPORT_SYMBOL(dm9051_get_reg);
EXPORT_SYMBOL(dm9051_set_regs);
EXPORT_SYMBOL(dm9051_get_regs);
EXPORT_SYMBOL(dm9051_write_mem);
EXPORT_SYMBOL(dm9051_read_mem);


MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_DESCRIPTION("Davicom DM9051 network SPI driver");
MODULE_LICENSE("GPL");
