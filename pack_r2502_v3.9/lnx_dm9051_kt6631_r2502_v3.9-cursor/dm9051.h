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
#define	RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | \
				 RSR_AE | RSR_CE | RSR_FOE)
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
#define DM9051_KERNEL_5_10	5
#define DM9051_KERNEL_5_15	6
#define DM9051_KERNEL_6_1	7
#define DM9051_KERNEL_6_6	8

#define	AARCH_OS_32             0
#define	AARCH_OS_64             1

#define LXR_REF_CONF		DM9051_KERNEL_6_6

/* Helper functions */
static inline struct board_info *to_dm9051_board(struct net_device *ndev)
{
        return netdev_priv(ndev);
}

const static char *linux_name[] = {
        "rsrv",
        "rsrv",
        "rsrv",
        "rsrv",
        "rsrv",
        "DM9051_KERNEL_5_10",
        "DM9051_KERNEL_5_15",
        "DM9051_KERNEL_6_1",
        "DM9051_KERNEL_6_6",
        "UNKNOW",
};

const static char dm9051_stats_strings[][ETH_GSTRING_LEN] = {
        "rx_packets",
        "tx_packets",
        "rx_errors",
        "tx_errors",
        "rx_bytes",
        "tx_bytes",
        "fifo_rst",
};

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
        int force_monitor_rxc;           /* int nRxcF; */
        int force_monitor_tx_timeout;
        struct {
                unsigned long delayF[POLL_TABLE_NUM];
                u16 nTargetMaxNum;       /* u16 ndelayF; */
        } sched;
        u64 tx_timeout_us;              /* escape, 'software_build_kernel_conf' */
};

const struct eng_config engdata = {
	.force_monitor_rxb = FORCE_SILENCE_RXB,              /* FORCE_MONITOR_RXB */
	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,       /* 0 */
	.sched = {
		.delayF = {0, 1, 0, 0, 1}, 
		.nTargetMaxNum = POLL_OPERATE_NUM}, /* POLL_OPERATE_INIT */
	.tx_timeout_us = 2100,
};

#define econf   (&engdata)
#define csched  (engdata.sched)

/* Architecture definitions */
#define AARCH_OS_32                     0
#define AARCH_OS_64                     1

/* driver config */
#define	FORCE_BUS_ENCPT_OFF		0
#define	FORCE_BUS_ENCPT_FIX_ON		1
#define	FORCE_BUS_ENCPT_FAB_ON		2
#define FORCE_BUS_ENCPT_NUL_KEY		0x00 //for NOTE!
#define FORCE_BUS_ENCPT_FIX_KEY		0x95 //for selected, for test in the lab.

#define	FORCE_TX_CONTI_OFF		0
#define	FORCE_TX_CONTI_ON		1

#define	DEFAULT_CHECKSUM_OFF		0
#define	DEFAULT_CHECKSUM_ON		1

#endif /* _DM9051_H_ */
