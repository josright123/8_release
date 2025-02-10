/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

#ifndef _DM9051_H_
#define _DM9051_H_

/* Standard Linux kernel headers */
#ifdef __KERNEL__
#include <linux/bits.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
/* Define basic types if not in kernel space */
typedef uint8_t         u8;
typedef uint16_t        u16;
typedef uint32_t        u32;
typedef uint64_t        u64;
#endif

/* Device identification */
#define DM9051_ID               0x9051

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

/* Continue with rest of register bits... */

/* Constants */
#define DM9051_PHY_ADDR        1       /* PHY id */
#define DM9051_PHY             0x40    /* PHY address 0x01 */
#define DM9051_PKT_RDY         0x01    /* Packet ready to receive */
#define DM9051_PKT_MAX         1536    /* Received packet max size */
#define DM9051_TX_QUE_HI_WATER 50
#define DM9051_TX_QUE_LO_WATER 25
#define DM_EEPROM_MAGIC        0x9051

/* Kernel version definitions */
#define DM9051_KERNEL_5_10     5
#define DM9051_KERNEL_5_15     6
#define DM9051_KERNEL_6_1      7
#define DM9051_KERNEL_6_6      8

/* Architecture definitions */
#define AARCH_OS_32            0
#define AARCH_OS_64            1

/* Feature control flags */
#define FORCE_SILENCE_RXB      0
#define FORCE_MONITOR_RXB      1

/* Continue with rest of definitions... */

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
	.force_monitor_rxb = FORCE_MONITOR_RXB,              /* FORCE_SILENCE_RXB */
	.force_monitor_rxc = FORCE_SILENCE_RX_COUNT,
	.force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,       /* 0 */
	.sched = {
		.delayF = {0, 1, 0, 0, 1}, 
		.nTargetMaxNum = POLL_OPERATE_NUM}, /* POLL_OPERATE_INIT */
	.tx_timeout_us = 2100,
};

#define econf   (&engdata)
#define csched  (engdata.sched)

#endif /* _DM9051_H_ */
