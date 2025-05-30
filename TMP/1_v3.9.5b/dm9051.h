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

/* Macro domain
 */
/*#define DMCONF_DIV_HLPR_32 */	//(32-bit division helper, __aeabi_ldivmod())
/*#define DMCONF_AARCH_64 */ //(64-bit OS)

/* Macro for already known platforms
 */ 
//#define PLUG_CFG_HLPR
#ifdef PLUG_CFG_HLPR
#define DMCONF_DIV_HLPR_32 //(32-bit division helper, __aeabi_ldivmod())
#endif

#define PLUG_CFG64
#ifdef PLUG_CFG64
#define DMCONF_AARCH_64 //(64-bit OS)
#endif

//define DMCONF_OPEN_EXTRA //(Negative, for already by dm9051_all_upstart())

/* Device identification */
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
#define TCR_TXREQ		BIT(0)
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
//_15888_,
// PTP use the same bit, timestamp is available
#define RSR_RXTS_EN       BIT(5)
// PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_LEN    	BIT(2)
//_15888_,
#define	RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | \
				 RSR_AE | RSR_CE | RSR_FOE)
//#define	RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO | 
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

#define	DM_RXHDR_SIZE		sizeof(struct dm9051_rxhdr)

/* Kernel version definitions */
//#define DM9051_KERNEL_5_10	5
//#define DM9051_KERNEL_5_15	6
//#define DM9051_KERNEL_6_1	7
//#define DM9051_KERNEL_6_6	8

//#define LXR_REF_CONF		DM9051_KERNEL_6_6

/* Helper functions */
static inline struct board_info *to_dm9051_board(struct net_device *ndev)
{
        return netdev_priv(ndev);
}

/* Feature control flags */
#define FORCE_SILENCE_RXB               0
#define FORCE_MONITOR_RXB               1

#define	FORCE_SILENCE_RX_COUNT		0
#define	FORCE_MONITOR_RX_COUNT		1

#define	FORCE_SILENCE_TX_TIMEOUT	0
#define	FORCE_MONITOR_TX_TIMEOUT	1

#define POLL_TABLE_NUM			5
#define POLL_OPERATE_INIT		0
#define POLL_OPERATE_NUM		1

/* Configuration structures */
struct eng_config {
        int force_monitor_rxb;
        int force_monitor_rxc;
        int force_monitor_tx_timeout;
        struct eng_sched {
                unsigned long delayF[POLL_TABLE_NUM];
                u16 nTargetMaxNum;
        } sched;
        u64 tx_timeout_us;
};

#if 0
//const struct eng_config engdata = {
//	.force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
//	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
//	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,
//	.sched = {
//		.delayF = {0, 1, 0, 0, 1}, 
//		.nTargetMaxNum = POLL_OPERATE_NUM},
//	.tx_timeout_us = 2100,
//};
#endif

/* driver config */
//#define	FORCE_TX_CONTI_OFF	0
//#define	FORCE_TX_CONTI_ON	1

#define	DEFAULT_CHECKSUM_OFF		0
#define	DEFAULT_CHECKSUM_ON		1

#define MI_FIX                          1

#if 1 //sticked fixed here is better!
#include "dm9051_plug.h" //for definition of 'INT_TWO_STEP'

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
struct rx_ctl_mach
{
	u16 status_err_counter;
	u16 large_err_counter;
	u16 rx_err_counter;
	u16 tx_err_counter;
	u16 fifo_rst_counter;

	u16 evaluate_rxb_counter;
	int nRxcF;
	u16 ndelayF;
	
	char head[62];
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
struct dm9051_rxctrl
{
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
struct dm9051_rxhdr
{
	u8 headbyte;
	u8 status;
	__le16 rxlen;
};

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
struct board_info
{
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

	#ifdef DMPLUG_INT
	#ifdef INT_TWO_STEP
	struct delayed_work irq_servicep;
	#endif //INT_TWO_STEP
	#else
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
	
	unsigned int n_automdix; //= 0;
	unsigned int stop_automdix_flag; //= 0
	unsigned int mdi; //= 0x0830;
	//u16 automdix_flag[3];
	char automdix_log[3][66];

//_15888_
	int                             ptp_on; 	//_15888_
	struct ptp_clock                *ptp_clock;
	struct ptp_clock_info 			ptp_caps;

	unsigned int                    ptp_tx_flags;
	u8								ptp_mode;	//_15888_ //0: Not PTP, 1: one-step, 2: two-step

	unsigned int					tcr_wr;
	s64								pre_rate;
	struct hwtstamp_config          tstamp_config;
};
#endif

#define TOGG_INTVL		1
#define TOGG_TOT_SHOW	5

int get_dts_irqf(struct board_info *db);

void dm9051_dump_reg2(struct board_info *db, unsigned int reg1, unsigned int reg2);
void dm9051_dump_reg2s(struct board_info *db, unsigned int reg1, unsigned int reg2);
void dm9051_dump_reg3(struct board_info *db, unsigned int reg1, unsigned int reg2, unsigned int reg3);
void dm9051_dump_registers(struct board_info *db);
int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff,
			size_t len);
int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen);

irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw);

int dm9051_nsr_poll(struct board_info *db);
int dm9051_set_interrupt_regs(struct board_info *db);
int dm9051_subconcl_and_rerxctrl(struct board_info *db);

/* amdix functions */
void amdix_link_change_up(struct board_info *db, unsigned int bmsr, unsigned int val);

#ifdef MAIN_DATA
/* Driver configuration structure */
struct mod_config
{
	char *test_info;
	int skb_wb_mode;
	int checksuming;
	struct align_config
	{
		char *burst_mode_info;
		int burst_mode;
		size_t tx_blk;
		size_t rx_blk;
	} align;
};

/* Default driver configuration */
//SPI_SYNC_ALIGN_MODE = 0,
//SPI_SYNC_BURST_MODE = 1,
//SPI_SYNC_MISC_MODE = 2,
//MODE_NUM = 3
const struct mod_config driver_align_mode = {
	.test_info = "Test in rpi5 bcm2712",
	.skb_wb_mode = SKB_WB_ON,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Alignment",
		.burst_mode = BURST_MODE_ALIGN,
		.tx_blk = 32, .rx_blk = 64},
};

const struct mod_config driver_burst_mode = {
	.test_info = "Test in rpi4 bcm2711",
	.skb_wb_mode = SKB_WB_ON,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Burst",
		.burst_mode = BURST_MODE_FULL,
		.tx_blk = 0, .rx_blk = 0},
};

const struct mod_config driver_misc_mode = {
	.test_info = "Test in processor Cortex-A",
	.skb_wb_mode = SKB_WB_OFF,
	.checksuming = DEFAULT_CHECKSUM_OFF,
	.align = {
		.burst_mode_info = "Burst",
		.burst_mode = BURST_MODE_FULL,
		.tx_blk = 0, .rx_blk = 0},
};
#else
//.
#endif

#endif /* _DM9051_H_ */
