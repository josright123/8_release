/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_H_
#define _DM9051_H_
/* Standard Linux kernel headers */
#include <linux/bits.h>
#include <linux/netdevice.h>
#include <linux/types.h>

#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_classify.h>
#include <linux/ip.h>
#include <linux/udp.h>

/* Macro domain
 */
/*#define DMPLUG_INT */ //(INT39)
/*#define INT_CLKOUT */ //(INT39 ClkOut)
/*#define INT_TWO_STEP */ //(INT39 two_step)
/*#define DMPLUG_WD */ //(wd mode)

/* Macro for already known platforms
 */
#define PLUG_ENABLE_INT
#ifdef PLUG_ENABLE_INT
#define DMPLUG_INT //(INT39)

//#define PLUG_INT_CLKOUT
#ifdef PLUG_INT_CLKOUT
#define INT_CLKOUT //(INT39_CLKOUT)
#endif

//#define PLUG_INT_2STEP
#ifdef PLUG_INT_2STEP
#define INT_TWO_STEP //(INT39_TWO_STEP)
#endif
#endif

//#define PLUG_ENABLE_WD
#ifdef PLUG_ENABLE_WD
#define DMPLUG_WD //(wd mode)
#endif

/*#include extern/dm9051_ptp1.h */ //(extern/)
/*#define DMPLUG_PTP */ //(ptp1588)
/*#define DMPLUG_PPS_CLKOUT */ //(ptp1588 pps)
/*#define DMPLUG_PTP_TWO_STEP */ //(ptp1588 two step)

/* Capabilities:
 *        hardware-transmit
 *        hardware-receive
 *        hardware-raw-clock
 */
#define PLUG_PTP_1588
#ifdef PLUG_PTP_1588
#define DMPLUG_PTP //(ptp 1588)

#define PLUG_PTP_PPS
#ifdef PLUG_PTP_PPS
#define DMPLUG_PPS_CLKOUT //(REG0x3C_pps)
#endif

/* "dm9051 PTP HW TWO STEP", Always essential (Mandartory recommanded) */
#define PLUG_PTP_TWO_STEP //(always essential)(if not support, master NO follow up send)
#ifdef PLUG_PTP_TWO_STEP
#define DMPLUG_PTP_TWO_STEP //(HW Two step)
#endif
#endif //(ptp 1588)

/*Capabilities:
 *        software-transmit
 *        software-receive
 *        software-system-clock
 *PTP Hardware Clock: none
 *Hardware Transmit Timestamp Modes: none
 *Hardware Receive Filter Modes: none
 */
#define PLUG_PTP_1588_SW
#ifdef PLUG_PTP_1588_SW
#define DMPLUG_PTP_SW //(ptp 1588 S/W)
#endif //(ptp 1588 S/W)

/* pragma
 */
#if defined(DMPLUG_INT) && defined(MAIN_DATA)
#pragma message("dm9051: INT")
#endif
#if !defined(DMPLUG_INT) && defined(MAIN_DATA)
#pragma message("dm9051: POL")
#endif
#if defined(INT_CLKOUT) && defined(MAIN_DATA)
#warning "INT: INT_CLKOUT"
#endif
#if defined(INT_TWO_STEP) && defined(MAIN_DATA)
#warning "INT: TWO_STEP"
#endif

#if defined(DMPLUG_WD) && defined(MAIN_DATA)
#pragma message("dm9051: WD")
#endif
#if !defined(DMPLUG_WD) && defined(MAIN_DATA)
#pragma message("dm9051: BD")
#endif

/* pragma, Extended support header files
 */

#if defined(DMPLUG_PTP) && defined(MAIN_DATA)
//#warning "dm9051 PTP"
#pragma message("dm9051: H/W PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT) && defined(MAIN_DATA)
//#warning "dm9051 PPS"
#pragma message("dm9051: H/W PPS")
#endif
#if defined(DMPLUG_PTP_TWO_STEP) && defined(MAIN_DATA)
//#warning "dm9051 PTP TWO STEP"
#pragma message("dm9051: H/W PTP TWO STEP")
#endif

#if defined(DMPLUG_PTP_SW) && defined(MAIN_DATA)
#pragma message("dm9051: S/W PTP (TWO STEP)")
#endif

/* Extended support header files
 */
/*#include plug/plug.h */ //(plug/)
/*#include extern/extern.h */ //(extern/)

/* Device identification
 */
#define DM9051_ID              0x9051
#define DRVNAME_9051           "dm9051"

/* Register addresses */
#define DM9051_NCR             0x00
#define DM9051_NSR             0x01
#define DM9051_TCR             0x02
#define DM9051_RCR             0x05
#define DM9051_BPTR            0x08
#define DM9051_FCR             0x0A
#define DM9051_EPCR            0x0B
#define DM9051_EPAR            0x0C
#define DM9051_EPDRL           0x0D
#define DM9051_EPDRH           0x0E
#define DM9051_PAR             0x10
#define DM9051_MAR             0x16
#define DM9051_GPCR            0x1E
#define DM9051_GPR             0x1F

/* Additional registers */
#define DM9051_VIDL            0x28
#define DM9051_VIDH            0x29
#define DM9051_PIDL            0x2A
#define DM9051_PIDH            0x2B
#define DM9051_SMCR            0x2F
#define DM9051_ATCR            0x30
#define DM9051_SPIBCR          0x38
#define DM9051_INTCR           0x39
#define DM9051_TXFSSR          0x3B
#define DM9051_PPCR            0x3D

/* Control registers */
#define DM9051_IPCOCR          0x54
#define DM9051_MPCR            0x55
#define DM9051_LMCR            0x57
#define DM9051_MBNDRY          0x5E

/* Memory access registers */
#define DM9051_MRRL            0x74
#define DM9051_MRRH            0x75
#define DM9051_MWRL            0x7A
#define DM9051_MWRH            0x7B
#define DM9051_TXPLL           0x7C
#define DM9051_TXPLH           0x7D
#define DM9051_ISR             0x7E
#define DM9051_IMR             0x7F

/* SPI commands */
#define DM_SPI_MRCMDX          0x70
#define DM_SPI_MRCMD           0x72
#define DM_SPI_MWCMD           0x78
#define DM_SPI_WR              0x80

/* Register bits definitions */
/* NCR (0x00) */
#define NCR_WAKEEN             BIT(6)
#define NCR_FDX                BIT(3)
#define NCR_RST                BIT(0)
/* NSR (0x01) */
#define NSR_SPEED              BIT(7)
#define NSR_LINKST             BIT(6)
#define NSR_WAKEST             BIT(5)
#define NSR_TX2END             BIT(3)
#define NSR_TX1END             BIT(2)
//0x02
#define TCR_TS_EN               BIT(7)  //_15888_
#define TCR_DIS_JABBER_TIMER	BIT(6)  //for Jabber Packet support 
#define TCR_TXREQ				BIT(0)
#define TCR_TS_EMIT		TCR_DIS_JABBER_TIMER
//0x05
#define RCR_DIS_WATCHDOG_TIMER	BIT(6)   //for Jabber Packet support 
#define RCR_DIS_LONG		BIT(5)
#define RCR_DIS_CRC		BIT(4)
#define RCR_ALL			BIT(3)
#define RCR_PRMSC		BIT(1)
#define RCR_RXEN		BIT(0)
#define RCR_RX_DISABLE		(RCR_DIS_LONG | RCR_DIS_CRC)
//0x06
#define RSR_RF			BIT(7)
#define RSR_MF			BIT(6)
#define RSR_LCS			BIT(5)
#define RSR_RWTO		BIT(4)
#define RSR_PLE			BIT(3)
#define RSR_AE			BIT(2)
#define RSR_CE			BIT(1)
#define RSR_FOE			BIT(0)
#define	RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | \
				 RSR_AE | RSR_CE | RSR_FOE)
//#define RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO |
//				 RSR_AE | RSR_CE | RSR_FOE) /* | RSR_PLE */
//0x0A
#define FCR_TXPEN		BIT(5)
#define FCR_BKPA		BIT(4)
#define FCR_BKPM		BIT(3)
#define FCR_FLCE		BIT(0)
#define FCR_RXTX_BITS		(FCR_TXPEN | FCR_BKPA | FCR_BKPM | FCR_FLCE)
//0x0B
#define EPCR_WEP		BIT(4)
#define EPCR_EPOS		BIT(3)
#define EPCR_ERPRR		BIT(2)
#define EPCR_ERPRW		BIT(1)
#define EPCR_ERRE		BIT(0)
//0x1E
#define GPCR_GEP_CNTL		BIT(0)
//0x1F
#define GPR_PHY_OFF		BIT(0)
//0x30
#define	ATCR_AUTO_TX		BIT(7)
#define	ATCR_TX_MODE2		BIT(4)
//0x39
#define INTCR_POL_LOW		(1 << 0)
#define INTCR_POL_HIGH		(0 << 0)
//0x3D
//Pause Packet Control Register - default = 1
#define PPCR_PAUSE_COUNT	0x08
#define PPCR_PAUSE_ADVCOUNT	0x0F
#define PPCR_PAUSE_UNLIMIT	0x00
//0x54
#define IPCOCR_CLKOUT		BIT(7)
#define IPCOCR_DUTY_LEN		1
//0x55
#define MPCR_RSTTX		BIT(1)
#define MPCR_RSTRX		BIT(0)
//0x57
//LEDMode Control Register - LEDMode1
//Value 0x81  bit[7] = 1, bit[2] = 0, bit[10] = 01b
#define LMCR_NEWMOD		BIT(7)
#define LMCR_TYPED1		BIT(1)
#define LMCR_TYPED0		BIT(0)
#define LMCR_MODE1		(LMCR_NEWMOD | LMCR_TYPED0)
/* 0x5E */
#define MBNDRY_BYTE		BIT(7)
#define MBNDRY_WORD		0
//0xFE
#define ISR_MBS			BIT(7)
#define ISR_LNKCHG		BIT(5)
#define ISR_ROOS		BIT(3)
#define ISR_ROS			BIT(2)
#define ISR_PTS			BIT(1)
#define ISR_PRS			BIT(0)
#define ISR_CLR_INT		(ISR_LNKCHG | ISR_ROOS | ISR_ROS | \
				 ISR_PTS | ISR_PRS)
#define ISR_STOP_MRCMD		(ISR_MBS)
//0xFF
#define IMR_PAR			BIT(7)
#define IMR_LNKCHGI		BIT(5)
#define IMR_PTM			BIT(1)
#define IMR_PRM			BIT(0)

/* Constants */
#define DM9051_PHY_ADDR		1       /* PHY id */
#define DM9051_PHY		0x40    /* PHY address 0x01 */
#define DM9051_PKT_RDY		0x01    /* Packet ready to receive */
#define DM9051_PKT_MAX		1536    /* Received packet max size */
#define DM9051_TX_QUE_HI_WATER	50
#define DM9051_TX_QUE_LO_WATER	25
#define DM_EEPROM_MAGIC		0x9051

/* Helper macros */
#define SCAN_BL(dw)		(dw & GENMASK(7, 0))
#define SCAN_BH(dw)		((dw & GENMASK(15, 8)) >> 8)
#define	DM_RXHDR_SIZE		sizeof(struct dm9051_rxhdr)
#define TIMES_TO_RST		10

/* Feature control flags */
#define FORCE_SILENCE_RXB               0
#define FORCE_MONITOR_RXB               1

#define	FORCE_SILENCE_RX_COUNT		0
#define	FORCE_MONITOR_RX_COUNT		1

#define	FORCE_SILENCE_TX_TIMEOUT	0
#define	FORCE_MONITOR_TX_TIMEOUT	1

/* driver config */
#define MI_FIX                          1

#define AMDIX_LOG_BUFSIZE		72

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
#define HEAD_LOG_BUFSIZE	62

#define TX_DELAY		1 // by .ndo_start_xmit
#define TX_THREAD0		2 // in rx loop0
#define TX_THREAD		3 // in rx loop

struct rx_ctl_mach {
	u32 status_err_counter;
	u32 large_err_counter;
	u32 rx_err_counter;
	u32 tx_err_counter;
	u32 fifo_rst_counter;

	u16 evaluate_rxb_counter;
	int nRxcF;
	u16 ndelayF; /* only for poll.o */

	char head[HEAD_LOG_BUFSIZE];

	u16 mode;
};

/**
 * struct dm9051_rxctrl - dm9051 driver rx control
 * @hash_table: Multicast hash-table data
 * @rcr_all: KS_RXCR1 register setting
 * @bus_word: Encryption key from fixed code or efuse
 *
 * The settings needs to control the receive filtering
 * such as the multicast hash-filter and the receive register settings
 */
struct dm9051_rxctrl {
	u16 hash_table[4];
	u8 rcr_all;
	u8 bus_word;
};

/**
 * struct dm9051_rxhdr - rx packet data header
 * @headbyte: lead byte equal to 0x01 notifies a valid packet
 * @status: status bits for the received packet
 * @rxlen: packet length
 *
 * The Rx packet pack, entered into the FIFO memory, start with these
 * four bytes which is the Rx header, followed by the ethernet
 * packet data and ends with an appended 4-byte CRC data.
 * Both Rx header and CRC data are for check purpose and finally
 * are dropped by this driver
 */
struct dm9051_rxhdr {
	u8 headbyte;
	u8 status;
	__le16 rxlen;
};

typedef struct ptp_board_info {
	//#ifdef DMPLUG_PTP

//	int						ptp_master_last_tx_flags; //BIT(0): SKBTX_HW_TSTAMP, BIT(1): SKBTX_SW_TSTAMP
//	.ptp_master_last_tx_flags = (skb_shinfo(skb)->tx_flags & SKBTX_SW_TSTAMP) ? SKBTX_SW_TSTAMP : SKBTX_HW_TSTAMP;

	int						ptp_skp_hw_tstamp; //0: skb software tstamp 1: skb hardware tstamp
	int						ptp_chip_push_tstamp; //0: no push tstamp 1: push tstamp
	int						ptp_enable;

	struct board_info		*db;
	struct ptp_clock_info 	ptp_caps;
	int						ptp_on; //_15888_
	struct ptp_clock        *ptp_clock;
	u8						ptp_step; //dividual
	u8						_ptp_rsrv; //ptp_packet; //dividual

	struct hwtstamp_config	tstamp_config;
	s64						pre_rate;
	u8              		rxTSbyte[8]; //_15888_ // Store 1588 Time Stamp
	//#endif
} ptp_board_info_t;

/**
 * struct board_info - maintain the saved data
 * @msg_enable: message level value
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @regmap_dm: regmap for register read/write
 * @regmap_dmbulk: extra regmap for bulk read/write
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @irq_workp: Work queue for polling mode
 * @pause: ethtool pause parameter structure
 * @spi_lockm: between threads lock structure
 * @reg_mutex: regmap access lock structure
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @rctl: rx control setting structure
 * @imr_all: to store operating imr value for register DM9051_IMR
 * @lcr_all: to store operating rcr value for register DM9051_LMCR
 */
struct board_info {
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct regmap *regmap_dm;
	struct regmap *regmap_dmbulk;
	struct work_struct rxctrl_work;
	struct work_struct tx_work;

#if defined(DMPLUG_INT)
#ifdef INT_TWO_STEP
	struct delayed_work irq_servicep;
#endif
#endif

#ifndef DMPLUG_INT
	struct delayed_work irq_workp;
#endif

	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex reg_mutex;
	struct rx_ctl_mach bc;
	struct dm9051_rxhdr rxhdr;
	struct dm9051_rxctrl rctl;
	u8 imr_all;
	u8 lcr_all;

	unsigned int csum_gen_val;
	unsigned int csum_rcv_val;

	unsigned int n_automdix;
	unsigned int stop_automdix_flag;
	char automdix_log[3][AMDIX_LOG_BUFSIZE]; //u16 automdix_flag[3];

	unsigned int tcr_wr;
	unsigned int data_len;
	unsigned int pad;

	unsigned int xmit_in; //
	unsigned int xmit_tc; //
	unsigned int xmit_zc; //zero count

	unsigned int xmit_thrd0;
	unsigned int xmit_ttc0; //zero count
	unsigned int xmit_thrd;
	unsigned int xmit_ttc; //zero count

	unsigned int bmsr;
	unsigned int lpa;
	unsigned int mdi; //= 0x0830;

	/* 1 ptpc */
	struct ptp_board_info pbi; //=struct ptp_board_info pbi;
};

#if (defined(__x86_64__) || defined(__aarch64__)) && defined(MAIN_DATA)
#ifdef CONFIG_64BIT // 64-bit specific code
#pragma message("dm9051 @ __aarch64__")
#else
#warning "dm9051 @ __aarch64__"
#warning "dm9051 but is @ CONFIG_32BIT"
#endif
#elif (!defined(__x86_64__) && !defined(__aarch64__)) && defined(MAIN_DATA)
#ifdef CONFIG_64BIT // 64-bit specific code
#warning "dm9051 @ __aarch32__"
#warning "dm9051 but is @ CONFIG_64BIT"
#else
#pragma message("dm9051 @ __aarch32__")
#endif
#endif //__x86_64__ || __aarch64__

#if (defined(__x86_64__) || defined(__aarch64__))
#define INFO_CPU_BITS(dev, db)				USER_CONFIG(dev, db, "dm9051: __aarch64__")
#ifdef CONFIG_64BIT
#define INFO_CPU_MIS_CONF(dev, db)			// silence conditionally
#else // config !64-bit specific code
#define INFO_CPU_MIS_CONF(dev, db)			USER_CONFIG(dev, db, "dm9051: CONFIG_32BIT (kconfig) ?!")
#endif
#elif (!defined(__x86_64__) && !defined(__aarch64__))
#define INFO_CPU_BITS(dev, db)				USER_CONFIG(dev, db, "dm9051: __aarch32__")
#ifdef CONFIG_64BIT // config 64-bit specific code
#define INFO_CPU_MIS_CONF(dev, db)			USER_CONFIG(dev, db, "dm9051: CONFIG_64BIT(kconfig) ?!")
#else
#define INFO_CPU_MIS_CONF(dev, db)			// silence conditionally
#endif
#endif //__x86_64__ || __aarch64__

/* macro fakes
 */
//info INFO_FAK0
#define INFO_INT(dev, db)					USER_CONFIG(dev, db, "dm9051: POL")
#define INFO_INT_CLKOUT(dev, db)
#define INFO_INT_TWOSTEP(dev, db)
#define INFO_WD(dev, db)					USER_CONFIG(dev, db, "dm9051: BD")
#define INFO_PTP(dev, db)
#define INFO_PPS(dev, db)
#define INFO_PTP2S(dev, db)
#define INFO_PTP_SW_2S(dev, db)
#define INFO_LOG(dev, db)
#define INFO_BMCR_WR(dev, db)
#define INFO_MRR_WR(dev, db)
#define INFO_BUSWORK(dev, db)
#define INFO_CONTI(dev, db)
#define INFO_LPBK_TST(dev, db)

/* main */
#if defined(DMPLUG_INT)
#undef INFO_INT
#define INFO_INT(dev, db)					USER_CONFIG(dev, db, "dm9051: INT")
#endif

#if defined(INT_CLKOUT)
#undef INFO_INT_CLKOUT
#define INFO_INT_CLKOUT(dev, db)			USER_CONFIG(dev, db, "INT: INT_CLKOUT")
#endif

#if defined(INT_TWO_STEP)
#undef INFO_INT_TWOSTEP
#define INFO_INT_TWOSTEP(dev, db)			USER_CONFIG(dev, db, "INT: TWO_STEP")
#endif

#if defined(DMPLUG_WD)
#undef INFO_WD
#define INFO_WD(dev, db)					USER_CONFIG(dev, db, "dm9051: WD")
#endif

/* ptp, clkout, 2step */
#if defined(DMPLUG_PTP)
#undef INFO_PTP
#define INFO_PTP(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PTP")
#endif
#if defined(DMPLUG_PPS_CLKOUT)
#undef INFO_PPS
#define INFO_PPS(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PPS")
#endif
#if defined(DMPLUG_PTP_TWO_STEP)
#undef INFO_PTP2S
#define INFO_PTP2S(dev, db)					USER_CONFIG(dev, db, "dm9051: H/W PTP TWO STEP")
#endif

#if defined(DMPLUG_PTP_SW)
#undef INFO_PTP_SW_2S
#define INFO_PTP_SW_2S(dev, db)				USER_CONFIG(dev, db, "dm9051: S/W PTP (TWO STEP)")
#endif

/* Helper functions */
static inline struct board_info *to_dm9051_board(struct net_device *ndev)
{
	return netdev_priv(ndev);
}

static inline void USER_CONFIG(struct device *dev, struct board_info *db, char *str)
{
	if (dev)
		dev_warn(dev, "%s\n", str);
	else if (db)
		netif_info(db, drv, db->ndev, "%s\n", str);
}

int get_dts_irqf(struct board_info *db);
//static void USER_CONFIG(struct device *dev, struct board_info *db, char *str); //implement in dm9051.c

unsigned int SHOW_BMSR(struct board_info *db);

int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb);
int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val); //to used in the plug section
int dm9051_phyread(void *context, unsigned int reg, unsigned int *val);
int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff,
		    size_t len);
int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff,
		     size_t len);
int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen);

int dm9051_ncr_poll(struct board_info *db);
int dm9051_nsr_poll(struct board_info *db);
int dm9051_all_upfcr(struct board_info *db);

/* init functions */
//int dm9051_all_reinit(struct board_info *db);
int dm9051_all_start(struct board_info *db);
int dm9051_all_start_intr(struct board_info *db);
int dm9051_subconcl_and_rerxctrl(struct board_info *db);

int dm9051_read_mem_rxb(struct board_info *db, unsigned int reg, void *buff,
			size_t len);
int dm9051_read_mem_cache(struct board_info *db, unsigned int reg, u8 *buff,
			  size_t crlen);

/* operation functions */
void dm9051_tx_len1(struct board_info *db);
int dm9051_req_tx(struct board_info *db);
int rx_break(struct board_info *db, unsigned int rxbyte, netdev_features_t features);
int rx_head_break(struct board_info *db);
int trap_clr(struct board_info *db);
int trap_rxb(struct board_info *db, unsigned int *prxbyte);
int dm9051_mem_tx(struct board_info *db, u8 *p);
int dm9051_loop_rx(struct board_info *db);
int dm9051_loop_tx(struct board_info *db);
void dm9051_thread_irq(void *pw); //(int voidirq, void *pw)
irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw);

/* Param structures
 */
struct param_config {
	int force_monitor_rxb;
	int force_monitor_rxc;
	int force_monitor_tx_timeout;
	u64 tx_timeout_us;
};

/* Driver configuration structure
 */
enum {
	BURST_MODE_ALIGN = 0,
	BURST_MODE_FULL = 1,
};

struct plat_cnf_info {
	char *test_info;
	//int skb_wb_mode;
	int checksuming;
	struct align_config {
		char *burst_mode_info;
		int burst_mode;
		u32 tx_blk;
		u32 rx_blk;
	} align;
};

#ifdef MAIN_DATA
const struct param_config param_conf = {
	.force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,
	.tx_timeout_us = 210000,
};

const struct param_config *param = &param_conf;
#endif //MAIN_DATA

#ifdef MAIN_DATA
enum {
	SKB_WB_OFF = 0,
	SKB_WB_ON = 1, //'wb'
};

enum {
	DEFAULT_CHECKSUM_OFF = 0,
	DEFAULT_CHECKSUM_ON = 1,
};

const struct plat_cnf_info plat_align_mode = {
	.test_info = "Test in rpi5 bcm2712",
	//.skb_wb_mode = SKB_WB_ON, //SKB_WB_OFF, //SKB_WB_ON,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Alignment",
		.burst_mode = BURST_MODE_ALIGN,
		.tx_blk = 32, .rx_blk = 64
	},
};

const struct plat_cnf_info plat_burst_mode = {
	.test_info = "Test in rpi4 bcm2711",
	//.skb_wb_mode = SKB_WB_ON,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Burst",
		.burst_mode = BURST_MODE_FULL,
		.tx_blk = 0, .rx_blk = 0
	},
};

const struct plat_cnf_info plat_misc_mode = {
	.test_info = "Test in processor Cortex-A",
	//.skb_wb_mode = SKB_WB_OFF,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Burst",
		.burst_mode = BURST_MODE_FULL,
		.tx_blk = 0, .rx_blk = 0
	},
};
#endif //MAIN_DATA

//#define NOT_REQUEST_SUPPORTTED	0x0
//#define VOID_REQUEST_FUNCTION		-9
//#define REQUEST_SUPPORTTED		1 //REQUEST_SUPPORTTED (1)

/* Optional functions declaration const */
enum dm_req_not_support {
	VOID_REQUEST_FUNCTION =		-9,
	NOT_REQUEST_SUPPORTTED =	0,
};
enum dm_req_support {
	REQUEST_SUPPORTTED =		1,
};

/* MCO, re-direct, Verification */
#define MCO //(MainCoerce)

/* fake int */
#define FREE_IRQ(b)					//empty
#define CANCEL_DLY_IRQ2(b)			//empty
#define DM9051_PROBE_DLYSETUP(b)	//empty

/* fake int */
#define dm9051_int2_supp()		NOT_REQUEST_SUPPORTTED
#define dm9051_int2_irq(d,h)	VOID_REQUEST_FUNCTION

/* poll fake */
#define dm9051_poll_supp()		NOT_REQUEST_SUPPORTTED
#define dm9051_poll_sch(d)		VOID_REQUEST_FUNCTION

/* fake clkout */
#define INT_SET_CLKOUT(db)	0		//empty(NoError)

/* fake raw rx mode */
#define BOUND_CONF_BIT			MBNDRY_BYTE
#define SET_RCR(b)				dm9051_set_rcr(b)
#define PAD_LEN(len)			len

/* fake raw tx mode */
#define single_tx_len(b)		dm9051_tx_len1(b)
#define single_tx_skb(b,s)		dm9051_single_tx1(b,s) //~wd, i.e. bd (byte mode)
//#define TX_PAD(b,s)				dm9051_tx_data_len(b,s) //~wd, i.e. bd (byte mode)
//#define TX_SEND(b,s)			dm9051_tx_send(b,s)

#if defined(MCO) && defined(DMPLUG_INT)
#undef FREE_IRQ
#define FREE_IRQ(db) dm9051_thread_irq_free(db->ndev) //dm9051_free_irqworks(db);
#if defined(INT_TWO_STEP)
#undef CANCEL_DLY_IRQ2
#define CANCEL_DLY_IRQ2(db) cancel_delayed_work_sync(&db->irq_servicep) //of dm9051_thread_irq_free(ndev)
#undef DM9051_PROBE_DLYSETUP
#define DM9051_PROBE_DLYSETUP(b) PROBE_INT2_DLY_SETUP(b)
#endif
#endif

#if defined(MCO) && !defined(DMPLUG_INT)
#undef FREE_IRQ
#define FREE_IRQ(db) cancel_delayed_work_sync(&db->irq_workp) //dm9051_free_irqworks(db)
#undef DM9051_PROBE_DLYSETUP
#define DM9051_PROBE_DLYSETUP(b) PROBE_POLL_SETUP(b)
#endif

#if defined(MCO) && defined(INT_TWO_STEP) /* && defined(MAIN_DATA)*/
#undef dm9051_int2_supp
#undef dm9051_int2_irq
#define dm9051_int2_supp() REQUEST_SUPPORTTED
#define dm9051_int2_irq(d,h) DM9051_INT2_REQUEST(d,h)

void PROBE_INT2_DLY_SETUP(struct board_info *db);
void dm9051_rx_irq_servicep(struct work_struct *work);
irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw);
int DM9051_INT2_REQUEST(struct board_info *db, irq_handler_t handler);
#endif

#if defined(MCO) && !defined(DMPLUG_INT) && defined(MAIN_DATA)
#undef dm9051_poll_supp
#undef dm9051_poll_sch
#define dm9051_poll_supp() REQUEST_SUPPORTTED
#define dm9051_poll_sch(d) DM9051_POLL_SCHED(d)

void dm9051_threaded_poll(struct work_struct *work); //dm9051_poll_servicep()
void PROBE_POLL_SETUP(struct board_info *db);
void OPEN_POLL_SCHED(struct board_info *db);
int DM9051_POLL_SCHED(struct board_info *db);
#endif

#if defined(MCO) && defined(INT_CLKOUT) && defined(MAIN_DATA)
#undef INT_SET_CLKOUT
#define INT_SET_CLKOUT(db) dm9051_int_clkout(struct board_info *db)
int dm9051_int_clkout(struct board_info *db);
#endif

#if defined(MCO) && defined(DMPLUG_WD)
#undef BOUND_CONF_BIT
#define BOUND_CONF_BIT						MBNDRY_WORD
#undef PAD_LEN
#define PAD_LEN(len)						(len & 1) ? len + 1 : len

#undef single_tx_skb
#define single_tx_skb(b,s)					dm9051_single_tx_wd(b,s) //wd
int dm9051_single_tx_wd(struct board_info *db, struct sk_buff *skb);

//#undef TX_PAD
//#define TX_PAD(b,s)							dm9051_expand_skb_txreq(b,s) //wd
//struct sk_buff *dm9051_expand_skb_txreq(struct board_info *db, struct sk_buff *skb);
#endif

/* ptp/ macro fakes
 * extern/ macro fakes
 */
//struct board_info;

#define PTP_VER(b)
#define PTP_VER_SOFTWARE(b)

#define PTP_NEW(d)				0
#define PTP_INIT_RCR(d)
#define PTP_INIT(d)
#define PTP_END(d)
#define PTP_ETHTOOL_INFO(s)
#define PTP_STATUS_BITS(b)			RSR_ERR_BITS
#define PTP_NETDEV_IOCTL(s)
#define PTP_AT_RATE(b)

/* ptp2 */
#define DMPLUG_RX_TS_MEM(b)		0
#define DMPLUG_RX_HW_TS_SKB(b,s)
#define SHOW_ptp_rx_packet_monitor(b,s)
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b)

#define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	//0
#define DMPLUG_PTP_TX_PRE(b,s)
#define DMPLUG_TX_EMIT_TS(b,s)

/* ptp sw */
#define DMPLUG_PTP_TX_TIMESTAMPING_SW(s)

/* plug/ macro fakes
 */

/* raw fake encrypt */
#define BUS_SETUP(db)	0		//empty(NoError)
#define BUS_OPS(db, buff, crlen)	//empty
/* raw fake loop_test */
#define dmplug_loop_test(b)	0

//[fake]
#define SHOW_DEVLOG_REFER_BEGIN(d, b)
#define SHOW_LOG_REFER_BEGIN(b)
#define SHOW_DEVLOG_MODE(d)
#define SHOW_DEVLOG_XMIT_THRD0(b)
#define SHOW_DEVLOG_XMIT_THRD(b)
#define SHOW_DEVLOG_XMIT_IN(b)
#define SHOW_DEVLOG_TCR_WR(b)

#define SHOW_PLAT_MODE(d)
#define SHOW_MAC(b, a)
#define SHOW_MONITOR_RXC(b, n)

#define DMPLUG_LOG_RXPTR(h,b) //#define dm9051_headlog_regs(h, b, r1, r2)
#define DMPLUG_LOG_PHY(b) //#define dm9051_phyread_headlog(h, b, r)	(void)0

#define dm9051_dump_data1(b, p, l)
#define monitor_rxb0(b, rb)

/* raw(fake) bmsr_wr */
#define PHY_READ(d, n, av) dm9051_phyread(d, n, av)
#define LINKCHG_UPSTART(b) dm9051_all_upfcr(b)


/* #define CO1 //(functions re-construct)
 */

/* ptp sw */
#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
#undef PTP_ETHTOOL_INFO
#define PTP_ETHTOOL_INFO(s)		s = dm9051_ts_info,
#undef PTP_NETDEV_IOCTL
#define PTP_NETDEV_IOCTL(s)		s = dm9051_ptp_netdev_ioctl,
#endif

/* ethtool_ops
 * netdev_ops
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
static inline int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info)
#else
static inline int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
#endif
{
	struct board_info *db = netdev_priv(net_dev);
	ptp_board_info_t *pbi = &db->pbi;

//Spenser - get phc_index
	//info->phc_index = -1;
	info->phc_index = pbi->ptp_clock ? ptp_clock_index(pbi->ptp_clock) : -1;

	info->so_timestamping = 0;
#if 1
#if defined(DMPLUG_PTP_SW)
	/* .software ts */
	info->so_timestamping |=
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE;
#endif
#endif
#if defined(DMPLUG_PTP)
	info->so_timestamping |=
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
#endif

#if defined(DMPLUG_PTP)
	info->tx_types =
		BIT(HWTSTAMP_TX_ONESTEP_SYNC) |
		BIT(HWTSTAMP_TX_OFF) |
		BIT(HWTSTAMP_TX_ON);
#endif

#if defined(DMPLUG_PTP)
	info->rx_filters =
		BIT(HWTSTAMP_FILTER_NONE) |
		BIT(HWTSTAMP_FILTER_ALL);
#endif
	return 0;
}

int dm9051_ptp_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);

/* ptp sw */
#if defined(DMPLUG_PTP_SW)
/* re-direct ptp sw */
#undef PTP_VER_SOFTWARE
#define PTP_VER_SOFTWARE(b)	ptp_ver_software(b)

#undef DMPLUG_PTP_TX_TIMESTAMPING_SW
#define DMPLUG_PTP_TX_TIMESTAMPING_SW(s)	dm9051_ptp_tx_swtstamp(s)
#endif

/* ptp */
#if defined(DMPLUG_PTP) /*&& defined(MAIN_DATA) && defined(CO1) (re-direct ptpc) */
#undef PTP_VER
#undef PTP_NEW
#undef PTP_INIT_RCR
#undef PTP_INIT
#undef PTP_END
#undef PTP_STATUS_BITS
#undef PTP_AT_RATE

#define PTP_VER(b)				ptp_ver(b)
#define PTP_NEW(d) 				ptp_new(d)
#define PTP_INIT_RCR(d) 		ptp_init_rcr(d)
#define PTP_INIT(d) 			ptp_init(d)
#define PTP_END(d) 				ptp_end(d)
#define PTP_STATUS_BITS(b)		ptp_status_bits(db)
#define PTP_AT_RATE(b)			on_core_init_ptp_rate(b)

#undef DMPLUG_RX_TS_MEM
#undef DMPLUG_RX_HW_TS_SKB
#undef SHOW_ptp_rx_packet_monitor
#undef DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER

#define DMPLUG_RX_TS_MEM(b)				dm9051_read_ptp_tstamp_mem(b)
#define DMPLUG_RX_HW_TS_SKB(b,s) 		dm9051_ptp_rx_hwtstamp(b,s)
#define SHOW_ptp_rx_packet_monitor(b,s) dm9051_ptp_rx_packet_monitor(b,s)
#define DMPLUG_NOT_CLIENT_DISPLAY_RXC_FROM_MASTER(b) \
		dm9051_ptp_rxc_from_master(b)

#undef DMPLUG_PTP_TX_IN_PROGRESS
#undef DMPLUG_PTP_TX_PRE
#undef DMPLUG_TX_EMIT_TS

#define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	dm9051_ptp_tx_in_progress(b,s)
#define DMPLUG_PTP_TX_PRE(b,s)			dm9051_ptp_txreq(b,s)
#define DMPLUG_TX_EMIT_TS(b,s)			dm9051_ptp_txreq_hwtstamp(b,s)
#endif

void ptp_ver_software(struct board_info *db);
void dm9051_ptp_tx_swtstamp(struct sk_buff *skb);

void ptp_ver(struct board_info *db);
int ptp_new(struct board_info *db);
void ptp_init_rcr(struct board_info *db);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
u8 ptp_status_bits(struct board_info *db);
void on_core_init_ptp_rate(struct board_info *db);


int dm9051_read_ptp_tstamp_mem(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rxc_from_master(struct board_info *db);

void dm9051_ptp_tx_in_progress(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_txreq(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb);

#endif /* _DM9051_H_ */
