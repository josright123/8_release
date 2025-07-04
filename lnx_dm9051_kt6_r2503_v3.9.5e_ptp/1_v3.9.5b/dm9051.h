/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */
#ifndef _DM9051_H_
#define _DM9051_H_
#include <linux/bits.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_classify.h>
#include <linux/ip.h>
#include <linux/udp.h>

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
#define DM9051_CSCR            0x31           /* check sum control register */
#define DM9051_RCSSR           0x32           /* receive check sum status register */
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
// 0x02
#define TCR_RSV_BIT7           BIT(7) //_15888_
#define TCR_DIS_JABBER_TIMER   BIT(6) // for Jabber Packet support
#define TCR_TXREQ              BIT(0)
// 0x05
#define RCR_DIS_WATCHDOG_TIMER BIT(6) // for Jabber Packet support
#define RCR_DIS_LONG           BIT(5)
#define RCR_DIS_CRC            BIT(4)
#define RCR_ALL                BIT(3)
#define RCR_PRMSC              BIT(1)
#define RCR_RXEN               BIT(0)
#define RCR_RX_DISABLE         (RCR_DIS_LONG | RCR_DIS_CRC)
// 0x06
#define RSR_RF                 BIT(7)
#define RSR_MF                 BIT(6)
#define RSR_LCS                BIT(5)
#define RSR_RWTO               BIT(4)
#define RSR_PLE                BIT(3)
#define RSR_AE                 BIT(2)
#define RSR_CE                 BIT(1)
#define RSR_FOE                BIT(0)
#define RSR_ERR_BITS           (RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | RSR_AE | RSR_CE | RSR_FOE)
// #define RSR_ERR_BITS		(RSR_RF | RSR_LCS | RSR_RWTO |
//				 RSR_AE | RSR_CE | RSR_FOE) /* | RSR_PLE */
// 0x0A
#define FCR_TXPEN              BIT(5)
#define FCR_BKPA               BIT(4)
#define FCR_BKPM               BIT(3)
#define FCR_FLCE               BIT(0)
#define FCR_RXTX_BITS          (FCR_TXPEN | FCR_BKPA | FCR_BKPM | FCR_FLCE)
// 0x0B
#define EPCR_WEP               BIT(4)
#define EPCR_EPOS              BIT(3)
#define EPCR_ERPRR             BIT(2)
#define EPCR_ERPRW             BIT(1)
#define EPCR_ERRE              BIT(0)
// 0x1E
#define GPCR_GEP_CNTL          BIT(0)
// 0x1F
#define GPR_PHY_OFF            BIT(0)
// 0x30
#define ATCR_AUTO_TX           BIT(7)
#define ATCR_TX_MODE2          BIT(4)
// 0x31
#define TCSCR_UDPCS_GEN        BIT(2)
#define TCSCR_TCPCS_GEN        BIT(1)
#define TCSCR_IPCS_GEN         BIT(0)
// 0x32
#define RCSSR_UDPS             BIT(7)
#define RCSSR_TCPS             BIT(6)
#define RCSSR_IPS              BIT(5)
#define RCSSR_UDPP             BIT(4)
#define RCSSR_TCPP             BIT(3)
#define RCSSR_IPP              BIT(2)
#define RCSSR_RCS_EN           BIT(1)          // Receive Checksum Checking Enable
#define RCSSR_DCSE             BIT(0)          // Discard Checksum Error Packet
// 0x39
#define INTCR_POL_LOW          (1 << 0)
#define INTCR_POL_HIGH         (0 << 0)
// 0x3D
// Pause Packet Control Register - default = 1
#define PPCR_PAUSE_COUNT       0x08
#define PPCR_PAUSE_ADVCOUNT    0x0F
#define PPCR_PAUSE_UNLIMIT     0x00
// 0x54
#define IPCOCR_CLKOUT          BIT(7)
#define IPCOCR_DUTY_LEN        1
// 0x55
#define MPCR_RSTTX             BIT(1)
#define MPCR_RSTRX             BIT(0)
// 0x57
// LEDMode Control Register - LEDMode1
// Value 0x81  bit[7] = 1, bit[2] = 0, bit[10] = 01b
#define LMCR_NEWMOD            BIT(7)
#define LMCR_TYPED1            BIT(1)
#define LMCR_TYPED0            BIT(0)
#define LMCR_MODE1             (LMCR_NEWMOD | LMCR_TYPED0)
/* 0x5E */
#define MBNDRY_BYTE            BIT(7)
#define MBNDRY_WORD            0
// 0xFE
#define ISR_MBS                BIT(7)
#define ISR_LNKCHG             BIT(5)
#define ISR_ROOS               BIT(3)
#define ISR_ROS                BIT(2)
#define ISR_PTS                BIT(1)
#define ISR_PRS                BIT(0)
#define ISR_CLR_INT            (ISR_LNKCHG | ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)
#define ISR_STOP_MRCMD         (ISR_MBS)
// 0xFF
#define IMR_PAR                BIT(7)
#define IMR_LNKCHGI            BIT(5)
#define IMR_PTM                BIT(1)
#define IMR_PRM                BIT(0)

/* Constants */
#define DM9051_PHY_ADDR        1    /* PHY id */
#define DM9051_PHY             0x40 /* PHY address 0x01 */
#define DM9051_PKT_RDY         0x01 /* Packet ready to receive */
#define DM9051_PKT_MAX         1536 /* Received packet max size */
#define DM9051_TX_QUE_HI_WATER 50
#define DM9051_TX_QUE_LO_WATER 25
#define DM_EEPROM_MAGIC        0x9051

/* Helper macros */
#define SCAN_BL(dw)            (dw & GENMASK(7, 0))
#define SCAN_BH(dw)            ((dw & GENMASK(15, 8)) >> 8)
#define DM_RXHDR_SIZE          sizeof(struct dm9051_rxhdr)
#define TIMES_TO_RST           10
#define DMPLUG_LOG_RXC         3

#define MAX_USR_CONFIG         20 // check grow develop of SHOW_ALL_USER_CONFIG()
#define AMDIX_LOG_BUFSIZE      72
#define HEAD_LOG_BUFSIZE       62

/**
 * struct rx_ctl_mach - rx activities record
 * @status_err_counter: rx status error counter
 * @large_err_counter: rx get large packet length error counter
 * @rx_err_counter: receive packet error counter
 * @tx_err_counter: transmit packet error counter
 * @fifo_rst_counter: restart reset operation counter
 * @up_rst_counter: up reset operation counter
 *
 * To keep track for the driver operation statistics
 */
#define TX_DELAY               1 // by .ndo_start_xmit
#define TX_THREAD0             2 // in rx loop0
#define TX_THREAD              3 // in rx loop

struct rx_ctl_mach
{
    u32 status_err_counter;
    u32 large_err_counter;
    u32 rx_err_counter;
    u32 tx_err_counter;
    u32 fifo_rst_counter;
    u32 up_rst_counter;

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
struct dm9051_rxctrl
{
    u16 hash_table[4];
    u8  rcr_all;
    u8  bus_word;
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

typedef struct ptp_board_info
{
    //	int						ptp_master_last_tx_flags; //BIT(0): SKBTX_HW_TSTAMP, BIT(1):
    // SKBTX_SW_TSTAMP 	.ptp_master_last_tx_flags = (skb_shinfo(skb)->tx_flags & SKBTX_SW_TSTAMP) ? SKBTX_SW_TSTAMP :
    // SKBTX_HW_TSTAMP;

    struct board_info    *db;
    struct ptp_clock     *ptp_clock;
    struct ptp_clock_info ptp_caps;

    int ptp_skp_hw_tstamp;    // 0: skb software tstamp 1: skb hardware tstamp
    int ptp_chip_push_tstamp; // 0: no push tstamp 1: push tstamp
    int ptp_enable;
    int ptp_on; //_15888_
    int ptp_ts_bytes;
    u8  ptp_step; // dividual
    u8  _ptp_step;
    u8  ptp_rx_msgtype; // ptp_packet; //dividual
    u8  ptp_tx_msgtype;
    int total_ptp_frames;

    struct ptp_header     *ptp_hdr_rx; // save as a flag
    struct hwtstamp_config tstamp_config;

    s64 pre_rate;
    u8  clkTSbyte[8];
    u8  rxTSbyte[8]; //_15888_ // Store 1588 Time Stamp
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
struct board_info
{
    u32 msg_enable;

    struct spi_device *spidev;
    struct net_device *ndev;
    struct mii_bus    *mdiobus;
    struct phy_device *phydev;

    struct regmap *regmap_dm;
    struct regmap *regmap_dmbulk;

    struct sk_buff_head txq;
    struct work_struct  rxctrl_work;
    struct work_struct  tx_work;

#if defined(DMPLUG_INT)
    #ifdef INT_TWO_STEP
    struct delayed_work irq_servicep;
    #endif
#endif

#ifndef DMPLUG_INT
    struct delayed_work irq_workp;
#endif

    struct ethtool_pauseparam pause;

    struct rx_ctl_mach   bc;
    struct dm9051_rxctrl rctl;
    struct dm9051_rxhdr  rxhdr;

    struct mutex spi_lockm;
    struct mutex reg_mutex;

    u8 imr_all;
    u8 lcr_all;

    unsigned int csum_gen_val;
    unsigned int csum_rcv_val;

    unsigned int data_len;
    unsigned int pad;
    unsigned int tcr_wr;

    unsigned int xmit_in; //
    unsigned int xmit_tc; //
    unsigned int xmit_zc; // zero count

    unsigned int xmit_thrd0;
    unsigned int xmit_ttc0; // zero count
    unsigned int xmit_thrd;
    unsigned int xmit_ttc; // zero count

    /* user config strings */
    int  ucfg_count;
    char user_config_strings[MAX_USR_CONFIG][ETH_GSTRING_LEN];

    /* state bmsr */
    unsigned int st_bmsr1;
    unsigned int st_bmsr2;
    /* bmsr_wr */
    unsigned int bmsr;
    unsigned int lpa;
    unsigned int mdi; //= 0x0830;
    unsigned int n_automdix;
    unsigned int stop_automdix_flag;

    char automdix_log[3][AMDIX_LOG_BUFSIZE];

    /* 1 ptpc */
    struct ptp_board_info pbi; //=struct ptp_board_info pbi;
};

/* Helper functions */
static inline struct board_info *to_dm9051_board(struct net_device *ndev) { return netdev_priv(ndev); }

static inline void USER_CONFIG(struct device *dev, struct board_info *db, char *str)
{
    if (db->ucfg_count < MAX_USR_CONFIG)
        sprintf(db->user_config_strings[db->ucfg_count++], "%s", str); // Collection

    if (dev)
        dev_warn(dev, "%s", str);
    else if (db)
        netif_info(db, drv, db->ndev, "%s", str);
}

static inline void MACRO_MSG_CONFIG(struct device *dev, struct board_info *db)
{
    char buff[32];

    sprintf(buff, "msg_enable: 0x%08x", db->msg_enable);
    USER_CONFIG(dev, db, buff);
}

static inline void dm9051_tx_pad_wd(struct board_info *db, struct sk_buff *skb)
{
    if (skb->len & 1)
        db->pad = 1;
}

static inline struct sk_buff *EXPAND_SKB_WD(struct sk_buff *skb)
{
    struct sk_buff *skb2 = skb_copy_expand(skb, 0, 1, GFP_ATOMIC);
    if (skb2)
    {
        dev_kfree_skb(skb);
        return skb2;
    }
    return skb;
}

static inline struct sk_buff *dm9051_chg_skb_wd(struct board_info *db, struct sk_buff *skb)
{
    if (db->pad)
        skb = EXPAND_SKB_WD(skb);
    return skb;
}

static inline void disp_dump_rx_cnt(struct device *dev, struct board_info *db)
{
    char buff[32];

    sprintf(buff, "dm9051-DBGRXC: %d", DMPLUG_LOG_RXC);
    USER_CONFIG(dev, db, buff);
}

static inline void dump_data(struct board_info *db, u8 *packet_data, int packet_len) //._dm9051_dump_data1
{
	int i, j, rowsize = 32;
	int splen; //index of start row
	int rlen; //remain/row length
	char line[120];

	netif_info(db, pktdata, db->ndev, "%s\n", db->bc.head);
	for (i = 0; i < packet_len; i += rlen) {
		//rlen = print_line(packet_data+i, min(rowsize, skb->len - i)); ...
		rlen =  packet_len - i;
		if (rlen >= rowsize) rlen = rowsize;

		splen = 0;
		splen += sprintf(line + splen, " %3d", i);
		for (j = 0; j < rlen; j++) {
			if (!(j % 8)) splen += sprintf(line + splen, " ");
			if (!(j % 16)) splen += sprintf(line + splen, " ");
			splen += sprintf(line + splen, " %02x", packet_data[i + j]);
		}
		netif_info(db, pktdata, db->ndev, "%s\n", line);
	}
}

static inline void dm9051_rx_packet_dump(struct board_info *db, struct sk_buff *skb)
{
	if (db->ndev->stats.rx_packets < DMPLUG_LOG_RXC) { //test
		sprintf(db->bc.head, "rx_packet %ld, len %d", db->ndev->stats.rx_packets, skb->len);
		dump_data(db, skb->data, skb->len);
		//dm9051_dump_data1(db, skb->data, skb->len);
	}
}

//#if defined(_DMPLUG_LOG) || 1
/* Consider: Put into dm9051.c */
/* From dm9051_log.c to dm9051.c: directly use: allow */
//static inline void _dump_data(struct board_info *db, u8 *packet_data, int packet_len);
//#endif

int get_dts_irqf(struct board_info *db);
irqreturn_t dm9051_rx_threaded_plat(int voidirq, void *pw);
int dm9051_get_reg(struct board_info *db, unsigned int reg, unsigned int *prb);
int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val); // to used in the plug section
int dm9051_phyread(void *context, unsigned int reg, unsigned int *val);
int dm9051_ncr_poll(struct board_info *db);
int dm9051_all_start_intr(struct board_info *db);
int dm9051_subconcl_and_rerxctrl(struct board_info *db);
int dm9051_loop_rx(struct board_info *db);
int dm9051_loop_tx(struct board_info *db);

int dm9051_nsr_poll(struct board_info *db);
int dm9051_all_upfcr(struct board_info *db);

int dm9051_read_mem(struct board_info *db, unsigned int reg, void *buff, size_t len);
int dm9051_write_mem(struct board_info *db, unsigned int reg, const void *buff, size_t len);
int dm9051_write_mem_cache(struct board_info *db, u8 *buff, unsigned int crlen);

void dm9051_tx_len(struct board_info *db, struct sk_buff *skb);  /* fake(default) */
int  dm9051_mode_tx(struct board_info *db, struct sk_buff *skb); /* fake(default) */
int  dm9051_single_tx(struct board_info *db, struct sk_buff *skb);

#if 0
.static inline unsigned int SHOW_BMSR(struct board_info *db);
.static inline void dm9051_log_regs(char *head, struct board_info *db, unsigned int reg1, unsigned int reg2);
// int dm9051_all_reinit(struct board_info *db);
int dm9051_all_start(struct board_info *db);
int dm9051_read_mem_rxb(struct board_info *db, unsigned int reg, void *buff, size_t len);
int dm9051_read_mem_cache(struct board_info *db, unsigned int reg, u8 *buff, size_t crlen);
int  dm9051_req_tx(struct board_info *db);
int  rx_break(struct board_info *db, unsigned int rxbyte, netdev_features_t features);
int  rx_head_break(struct board_info *db);
int  trap_clr(struct board_info *db);
int  trap_rxb(struct board_info *db, unsigned int *prxbyte);
int  dm9051_mem_tx(struct board_info *db, u8 *p);
void dm9051_thread_irq(void *pw); //(int voidirq, void *pw)
#endif

//.
//#include "template.h"
//.
//#include "dm9051_main_data.h"

/* raw (fake) */
/* #include "template_0.h"= (For All C source code)(NOT coerced to supperted, almost can give up...)
 */
#define BUS_SETUP1(f, b, r)        //plug  BUS_SETUP(b) 0, or bus_setup(b)
#define BUS_OPS1(f, b, bf, l)
#define LOOPBACK_TEST1(f, b, r)    //plug, loopback_test(b) 0, or test_loop_test(b)
#define SHOW_BEGIN_LOG(d, b)       //extern
#define SHOW_LOG_REFER_BEGIN(b)
#define SHOW_DEVLOG_MODE(d)
#define SHOW_DEVLOG_XMIT_THRD0(b)
#define SHOW_DEVLOG_XMIT_THRD(b)
#define SHOW_DEVLOG_XMIT_IN(b)
#define SHOW_PLAT_MODE(d)
#define SHOW_MAC(b, a)
#define SHOW_MONITOR_RXC(b, n)
#define DMPLUG_LOG_RXPTR(h, b)     //extern
#define DMPLUG_LOG_PHY(b)
#define monitor_rxb0(b, rb)        //extern
#define BMSR_OPERATION_CLEAR(b)    //extern
/* raw (fake) */
#define TX_CONTI_CONSTRAIN(f)
#define SET_RCR(b)                 dm9051_set_rcr(b)         //plug.conti
#define INTERN_PHY_READ(d, n, av)  dm9051_phyread(d, n, av)  //self
#define MDIO_PHY_READ(d, n, av)    dm9051_phyread(d, n, av)  //extern.bmcr_wr
#define LINKCHG_UPSTART(b)         dm9051_all_upfcr(b)       //extern.mrr_wr
/* ptp/ macro fakes
 * extern/ macro fakes
 */
// #define DMPLUG_PTP_TX_IN_PROGRESS(b,s)	//0
// #define DMPLUG_PTP_TX_PRE(b,s)
// #define DMPLUG_TX_EMIT_TS(b,s)

/* ptp implementation
 */
int  dm9051_get_clk_ts(struct board_info *db);
void on_core_init_ptp_rate(struct board_info *db);

void ptp_ver(struct board_info *db);
void ptp_operation_extern(struct board_info *db);
void ptp_checksum_limit(struct board_info *db, struct net_device *ndev);
void ptp_init(struct board_info *db);
void ptp_end(struct board_info *db);
u8   ptp_status_bits(struct board_info *db);
void dm9051_ptp_rxc_from_master(struct board_info *db);
int  dm9051_read_ptp_tstamp_mem(struct board_info *db);
void dm9051_ptp_rx_hwtstamp(struct board_info *db, struct sk_buff *skb);
void dm9051_ptp_rx_packet_monitor(struct board_info *db, struct sk_buff *skb);
int  dm9051_ptp_tx_packet_monitor(struct board_info *db, struct sk_buff *skb);
int  dm9051_ptp_single_tx(struct board_info *db, struct sk_buff *skb);
netdev_features_t dm9051_ptp_constrain_features(struct net_device *ndev, netdev_features_t features);

int is_ptp_announce_packet(u8 msgtype);
int is_ptp_sync_packet(u8 msgtype);
int is_ptp_delayreq_packet(u8 msgtype);
int is_ptp_delayresp_packet(u8 msgtype);
int is_peer_delayreq_packet(u8 msgtype);
int is_peer_delayresp_packet(u8 msgtype);
int is_peer_delayresp_followup_packet(u8 msgtype);
int is_ptp_rxts_en(struct board_info *db);

struct ptp_header *get_ptp_header(struct sk_buff *skb);
u8  get_ptp_message_type005(struct ptp_header *ptp_hdr);
struct ptp_header *dm9051_rx_ptp_hdr_monitor(struct board_info *db);
 
#endif /* _DM9051_H_ */
