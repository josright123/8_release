// "dm9051_main_data.h"

/* pragma
 */
#if defined(DMPLUG_INT) && defined(MAIN_DATA)
    #pragma message("dm9051: INT")

    #if defined(INT_CLKOUT) && defined(MAIN_DATA)
        #pragma message("INT: INT_CLKOUT")
    #endif
    #if defined(INT_TWO_STEP) && defined(MAIN_DATA)
        #pragma message("INT: TWO_STEP")
    #endif
#endif
#if !defined(DMPLUG_INT) && defined(MAIN_DATA)
    #pragma message("dm9051: POL")
#endif

#if defined(DMPLUG_WD) && defined(MAIN_DATA)
    #pragma message("dm9051: WD")

    #if defined(DMPLUG_SKB_PROTECT) && defined(MAIN_DATA)
        #pragma message("WD: SKB_PROT")
    #endif
    #if !defined(DMPLUG_SKB_PROTECT) && defined(MAIN_DATA)
        #pragma message("WD: NO SKB_PROT")
    #endif
#endif
#if !defined(DMPLUG_WD) && defined(MAIN_DATA)
    #pragma message("dm9051: BD")
#endif

#if defined(DMPLUG_MI_FIX) && defined(MAIN_DATA)
    #pragma message("dm9051: MI_FIX")
#endif

#if defined(DMPLUG_PTP_SW) && defined(MAIN_DATA)
    #pragma message("dm9051: PTP (S/W TWO STEP)")
#endif

#if (defined(__x86_64__) || defined(__aarch64__)) && defined(MAIN_DATA)
    #ifdef CONFIG_64BIT // 64-bit specific code
        #pragma message("platform: __aarch64__")
    #else
        #warning "platform @ __aarch64__"
        #warning "platform but is @ CONFIG_32BIT"
    #endif
#elif (!defined(__x86_64__) && !defined(__aarch64__)) && defined(MAIN_DATA)
    #ifdef CONFIG_64BIT // 64-bit specific code
        #warning "platform @ __aarch32__"
        #warning "platform but is @ CONFIG_64BIT"
    #else
        #pragma message("platform: __aarch32__")
    #endif
#endif //__x86_64__ || __aarch64__

#if defined(MAIN_DATA)
    #define LINUX_STRING "Linux: " UTS_RELEASE
    #pragma message(LINUX_STRING)
#endif // MAIN_DATA

enum
{
    DEFAULT_CHECKSUM_OFF = 0,
    DEFAULT_CHECKSUM_ON  = 1,
};
enum
{
    BURST_MODE_ALIGN = 0,
    BURST_MODE_FULL  = 1,
};
struct plat_cnf_info
{
    char *test_device;
    int checksuming;
    struct align_config
    {
        char *mode;
        int   burst_mode;
        u32   tx_blk;
        u32   rx_blk;
    } align;
};

#ifdef MAIN_DATA
const struct plat_cnf_info plat_align_mode = {
    .test_device = "rpi5 bcm2712",
    //.skb_wb_mode = SKB_WB_ON, //SKB_WB_OFF, //SKB_WB_ON,
    .checksuming = DEFAULT_CHECKSUM_OFF,
    .align       = {.mode = "Alignment", .burst_mode = BURST_MODE_ALIGN, .tx_blk = 32, .rx_blk = 64},
};
const struct plat_cnf_info plat_misc_mode = {
    .test_device = "Dev Cortex-A",
    //.skb_wb_mode = SKB_WB_OFF,
    .checksuming = DEFAULT_CHECKSUM_OFF,
    .align       = {.mode = "Burst", .burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
};
#endif

/* Param structures
 */
/* Feature control flags */
#define FORCE_SILENCE_RXB        0
#define FORCE_MONITOR_RXB        1

#define FORCE_SILENCE_RX_COUNT   0
#define FORCE_MONITOR_RX_COUNT   1

#define FORCE_SILENCE_TX_TIMEOUT 0
#define FORCE_MONITOR_TX_TIMEOUT 1

struct param_config
{
    int force_monitor_rxb;
    int force_monitor_rxc;
    int force_monitor_tx_timeout;
    u64 tx_timeout_us;
};
/* Driver configuration structure
 */
#ifdef MAIN_DATA
const struct param_config param_conf = {
    .force_monitor_rxb = FORCE_SILENCE_RXB, /* FORCE_MONITOR_RXB */
    .force_monitor_rxc = FORCE_SILENCE_RX_COUNT,

    .force_monitor_tx_timeout = FORCE_SILENCE_TX_TIMEOUT,

    .tx_timeout_us = 210000,
};

const struct param_config *param = &param_conf;
#endif // MAIN_DATA

#if 0
    #ifdef MAIN_DATA
	const struct plat_cnf_info plat_burst_mode = {
		.test_device = "rpi4 bcm2711",
		//.skb_wb_mode = SKB_WB_ON,
		.checksuming = DEFAULT_CHECKSUM_OFF,
		.align       = {.mode = "Burst", .burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
	};

	const struct plat_cnf_info plat_misc_mode = {
		.test_device = "processor Cortex-A",
		//.skb_wb_mode = SKB_WB_OFF,
		.checksuming = DEFAULT_CHECKSUM_OFF,
		.align       = {.mode = "Burst", .burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
	};
    #endif // MAIN_DATA

	// #define NOT_REQUEST_SUPPORTTED	0x0
	// #define VOID_REQUEST_FUNCTION		-9
	// #define REQUEST_SUPPORTTED		1 //REQUEST_SUPPORTTED (1)

	// #define TX_PAD(b,s)				dm9051_tx_data_len(b,s) //~wd, i.e. bd (byte mode)
	// #undef TX_PAD
	// #define TX_PAD(b,s)							dm9051_expand_skb_txreq(b,s) //wd
	// #undef MODE_TX
	// #define MODE_TX(b,s)					dm9051_mode_tx2(b,s) //wd
	// #define TX_SEND(b,s)			dm9051_mode_tx1(b,s)

	// struct sk_buff *dm9051_expand_skb_txreq(struct board_info *db, struct sk_buff *skb);
	// int dm9051_mode_tx2(struct board_info *db, struct sk_buff *skb);

	void ptp_ver_software(struct board_info *db);
	void dm9051_ptp_tx_swtstamp(struct sk_buff *skb);

	// int ptp_new(struct board_info *db);
	//void ptp_init_rcr(struct board_info *db);

	// void dm9051_ptp_tx_in_progress(struct board_info *db, struct sk_buff *skb);
	// void dm9051_ptp_tcr_2wr(struct board_info *db, struct sk_buff *skb);
	// void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb);

    #if defined(DMPLUG_LOG) || 1
	/* Consider: Put into dm9051.c */
	/* of dm9051_log.c: directly use : allow */
	void dump_data(struct board_info *db, u8 *packet_data, int packet_len);
    #endif
#endif

/* system */
#if (defined(__x86_64__) || defined(__aarch64__))
    // #define INFO_CPU_BITS(dev, db) USER_CONFIG(dev, db, "platform: __aarch64__")
    #ifdef CONFIG_64BIT
    // #define INFO_CPU_MIS_CONF(dev, db) // silence conditionally
    #else // config !64-bit specific code
        #undef INFO_CPU_MIS_CONF
        #define INFO_CPU_MIS_CONF(dev, db) USER_CONFIG(dev, db, "platform: CONFIG_32BIT (kconfig) ?!")
    #endif
#elif (!defined(__x86_64__) && !defined(__aarch64__))
    #undef INFO_CPU_BITS
    #define INFO_CPU_BITS(dev, db) USER_CONFIG(dev, db, "platform: __aarch32__")
    #ifdef CONFIG_64BIT // config 64-bit specific code
        #undef INFO_CPU_MIS_CONF
        #define INFO_CPU_MIS_CONF(dev, db) USER_CONFIG(dev, db, "platform: CONFIG_64BIT(kconfig) ?!")
    #else
    // #define INFO_CPU_MIS_CONF(dev, db) // silence conditionally
    #endif
#endif //__x86_64__ || __aarch64__

#if defined(DMPLUG_INT)
    #undef INFO_INT
    #define INFO_INT(dev, db) USER_CONFIG(dev, db, "dm9051: INT")

    #if defined(INT_CLKOUT)
        #undef INFO_INT_CLKOUT
        #define INFO_INT_CLKOUT(dev, db) USER_CONFIG(dev, db, "INT: INT_CLKOUT")
    #endif

    #if defined(INT_TWO_STEP)
        #undef INFO_INT_TWOSTEP
        #define INFO_INT_TWOSTEP(dev, db) USER_CONFIG(dev, db, "INT: TWO_STEP")
    #endif
#endif

#if defined(DMPLUG_WD)
    #undef INFO_WD
    #define INFO_WD(dev, db) USER_CONFIG(dev, db, "dm9051: WD")

    #if defined(DMPLUG_SKB_PROTECT)
        #undef INFO_SKB_PROT
        #define INFO_SKB_PROT(dev, db) USER_CONFIG(dev, db, "WD: SKB PROT")
    #else
        #undef INFO_SKB_PROT
        #define INFO_SKB_PROT(dev, db) USER_CONFIG(dev, db, "WD: no SKB_PROT")
    #endif
#endif

#if defined(DMPLUG_MI_FIX)
    #undef INFO_MI_FIX
    #define INFO_MI_FIX(dev, db) USER_CONFIG(dev, db, "dm9051: MI_FIX")
#endif

#if defined(DMPLUG_PTP_SW)
    #undef INFO_PTP_SW_2S
    #define INFO_PTP_SW_2S(dev, db) USER_CONFIG(dev, db, "dm9051: PTP (S/W TWO STEP)")
#endif

/* MCO, re-direct, Verification */
#define MCO //(MainCoerce)

#if defined(MCO) && defined(DMPLUG_INT)
    #if defined(INT_TWO_STEP)
        #undef DM9051_PROBE_DLYSETUP
        #define DM9051_PROBE_DLYSETUP(b) PROBE_INT2_DLY_SETUP(b)
        #undef DM9051_STOP_CANCELDLY2
        #define DM9051_STOP_CANCELDLY2(db)                                                                             \
            cancel_delayed_work_sync(&db->irq_servicep) // of dm9051_thread_irq_free(ndev)
    #endif
    #undef DM9051_STOP_FREEIRQ
    #define DM9051_STOP_FREEIRQ(db) dm9051_thread_irq_free(db->ndev) // dm9051_free_irqworks(db);
#endif

#if defined(MCO) && !defined(DMPLUG_INT)
    #undef DM9051_PROBE_DLYSETUP
    #define DM9051_PROBE_DLYSETUP(b) PROBE_POLL_SETUP(b)
    #undef DM9051_STOP_FREEIRQ
    #define DM9051_STOP_FREEIRQ(db) cancel_delayed_work_sync(&db->irq_workp) // dm9051_free_irqworks(db)
#endif

#if defined(MCO) && defined(INT_CLKOUT) && defined(MAIN_DATA)
    #undef INT_SET_CLKOUT
    #define INT_SET_CLKOUT(db) dm9051_int_clkout(db)
int dm9051_int_clkout(struct board_info *db); // in "dm9051.c"
#endif

#if defined(MCO) && defined(INT_TWO_STEP) /* && defined(MAIN_DATA)*/
    #undef dm9051_int2_supp
    #undef dm9051_int2_irq
    #define dm9051_int2_supp()    REQUEST_SUPPORTTED
    #define dm9051_int2_irq(d, h) DM9051_INT2_REQUEST(d, h)

void PROBE_INT2_DLY_SETUP(struct board_info *db);
void dm9051_rx_irq_servicep(struct work_struct *work);

irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw); // of "dm9051_int2.c"

int DM9051_INT2_REQUEST(struct board_info *db, irq_handler_t handler);
#endif

#if defined(MCO) && !defined(DMPLUG_INT) && defined(MAIN_DATA)
    #undef dm9051_poll_supp
    #undef dm9051_poll_sch
    #define dm9051_poll_supp() REQUEST_SUPPORTTED
    #define dm9051_poll_sch(d) DM9051_POLL_SCHED(d)

void dm9051_threaded_poll(struct work_struct *work); // dm9051_poll_servicep()
void PROBE_POLL_SETUP(struct board_info *db);
void OPEN_POLL_SCHED(struct board_info *db);
int  DM9051_POLL_SCHED(struct board_info *db);
#endif

#if defined(MCO) && defined(DMPLUG_WD)
    #undef BOUND_CONF_BIT
    #define BOUND_CONF_BIT MBNDRY_WORD

    #undef PAD_LEN
    #define PAD_LEN(len) (len & 1) ? len + 1 : len

    #undef PAD_TX
    #define PAD_TX(b, s) dm9051_tx_pad_wd(b, s)
// void dm9051_tx_pad_xx(struct board_info *db, struct sk_buff *skb); //of "dm9051_wd,c"

    #if defined(DMPLUG_SKB_PROTECT)
        #undef CHG_SKB_TX
        #define CHG_SKB_TX(b, s) s = dm9051_chg_skb_wd(b, s)
    // struct sk_buff *dm9051_chg_skb_xx(struct board_info *db, struct sk_buff *skb); //of "dm9051_wd,c"
    #endif
#endif

/* mi fixed */
#if defined(DMPLUG_MI_FIX)
    #undef MI_MUTEX_LOCK
    #define MI_MUTEX_LOCK(b) mutex_lock(&b->spi_lockm)
    #undef MI_MUTEX_UNLOCK
    #define MI_MUTEX_UNLOCK(b) mutex_unlock(&b->spi_lockm)
#endif

/* ptp sw */
#if defined(DMPLUG_PTP_SW)
    /* re-direct ptp sw */
    #undef PTP_VER_SOFTWARE
    #define PTP_VER_SOFTWARE(b) ptp_ver_software(b) /* impl in dm9051_log.c */
    #undef DMPLUG_PTP_TX_TIMESTAMPING_SW
    #define DMPLUG_PTP_TX_TIMESTAMPING_SW(s) dm9051_ptp_tx_swtstamp(s)
#endif

/* ptp and ptp sw */
#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
    #undef PTP_ETHTOOL_INFO
    #define PTP_ETHTOOL_INFO(s) s = dm9051_ts_info,
    #undef PTP_NETDEV_IOCTL
    #define PTP_NETDEV_IOCTL(s) s = dm9051_eth_ioctl,
#endif
