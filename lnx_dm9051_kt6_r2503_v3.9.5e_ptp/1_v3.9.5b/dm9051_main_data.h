/* "dm9051_main_data.h"
 * main data
 */

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
#endif // _MAIN_DATA

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
const struct plat_cnf_info *plat_cnf = &plat_misc_mode; //'&plat_align_mode'; /* Driver configuration */
#endif // _MAIN_DATA

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

static inline void SHOW_PLAT_CONF(struct board_info *db)
{
    netif_crit(db, hw, db->ndev, "plat_cnf->test_device: %s", plat_cnf->test_device);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.mode: %s", plat_cnf->align.mode);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.txsize: %d", plat_cnf->align.tx_blk);
    netif_crit(db, hw, db->ndev, "plat_cnf->align.rxsize: %d", plat_cnf->align.rx_blk);
    netif_crit(db, hw, db->ndev, "plat_cnf->checksuming: %d", plat_cnf->checksuming);
}
#endif // _MAIN_DATA

#if 0
    #ifdef _MAIN_DATA
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
    #endif // _MAIN_DATA

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

	// int ptp_new(struct board_info *db);
	//void ptp_init_rcr(struct board_info *db);

	// void dm9051_ptp_tx_in_progress(struct board_info *db, struct sk_buff *skb);
	// void dm9051_ptp_tcr_2wr(struct board_info *db, struct sk_buff *skb);
	// void dm9051_ptp_txreq_hwtstamp(struct board_info *db, struct sk_buff *skb);

	void ptp_ver_software(struct board_info *db);
	void dm9051_ptp_tx_swtstamp(struct sk_buff *skb);
#endif //0

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

#if defined(MCO) && defined(INT_TWO_STEP)
    #undef dm9051_int2_supp
    #undef dm9051_int2_irq
    #define dm9051_int2_supp()    REQUEST_SUPPORTTED
    #define dm9051_int2_irq(d, h) DM9051_INT2_REQUEST(d, h)

void PROBE_INT2_DLY_SETUP(struct board_info *db);
void dm9051_rx_irq_servicep(struct work_struct *work);

irqreturn_t dm9051_rx_int2_delay(int voidirq, void *pw); // of "dm9051_int2.c"

int DM9051_INT2_REQUEST(struct board_info *db, irq_handler_t handler);
#endif

#if defined(MCO) && !defined(DMPLUG_INT) /* && defined(_MAIN_DATA) */
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

/* ptp and ptp sw */
#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
    #undef PTP_ETHTOOL_INFO
    #define PTP_ETHTOOL_INFO(s) s = dm9051_ts_info,
    #undef PTP_NETDEV_IOCTL
    #define PTP_NETDEV_IOCTL(s) s = dm9051_eth_ioctl,
#endif
 
/* ethtool_ops
 * netdev_ops
 */
#if (defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)) && defined(MAIN_DATA)
/* ----------------------
 * Inline function Block.
 * ----------------------
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
static inline int dm9051_ts_info(struct net_device *net_dev, struct kernel_ethtool_ts_info *info)
#else
static inline int dm9051_ts_info(struct net_device *net_dev, struct ethtool_ts_info *info)
#endif
{
    info->so_timestamping = 0;

#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
    info->tx_types   = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
    info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_ALL);
#endif

#if defined(DMPLUG_PTP_SW)
    info->so_timestamping |=
        SOF_TIMESTAMPING_TX_SOFTWARE |
        SOF_TIMESTAMPING_RX_SOFTWARE |
        SOF_TIMESTAMPING_SOFTWARE; /* .software ts */
#endif

#if defined(DMPLUG_PTP)
    info->so_timestamping |=
        SOF_TIMESTAMPING_TX_HARDWARE |
        SOF_TIMESTAMPING_RX_HARDWARE |
        SOF_TIMESTAMPING_RAW_HARDWARE;
#endif

#if defined(DMPLUG_PTP)
    info->tx_types |=
        BIT(HWTSTAMP_TX_ONESTEP_SYNC);
#endif

#if defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)
    do
    {
        struct board_info *db  = netdev_priv(net_dev);
        ptp_board_info_t  *pbi = &db->pbi;
        info->phc_index        = pbi->ptp_clock ? ptp_clock_index(pbi->ptp_clock) : -1;
        // info->phc_index = -1; // Spenser - get phc_index
    } while (0);
#endif

    return 0;
}
#endif

#if (defined(DMPLUG_PTP) || defined(DMPLUG_PTP_SW)) && defined(MAIN_DATA)
int all_know_allow_show = 5;

static int lan_ptp_get_ts_ioctl(struct net_device *netdev, struct ifreq *ifr)
{
    struct board_info      *db     = netdev_priv(netdev);
    ptp_board_info_t       *pbi    = &db->pbi;
    struct hwtstamp_config *config = &pbi->tstamp_config;

    /* copy from db _tstamp_config, to user */
    return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ? -EFAULT : 0;
}

static int lan743x_ptp_set_ts_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
    struct board_info     *db  = netdev_priv(netdev);
    ptp_board_info_t      *pbi = &db->pbi;
    struct hwtstamp_config config;
    //	int ret = 0;

    if (!ifr)
    {
        netif_err(db, hw, db->ndev, "SIOCSHWTSTAMP, ifr == NULL\n");
        return -EINVAL;
    }

    if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
        return -EFAULT;

    if (config.flags)
    {
        netif_warn(db, hw, db->ndev, "ignoring _hwtstamp_config.flags == 0x%08X, expected 0\n", config.flags);
    }

    switch (config.tx_type)
    {
    case HWTSTAMP_TX_OFF:
        // dev_info(&adb->spidev->dev, "IOCtl - Now db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, false)\n",
        // adb->ptp_on);
        netif_info(db, hw, db->ndev, "tx_type= HWTSTAMP_TX_OFF(0): Now db->ptp_on %d, NOTE: Stop tx sync !\n",
                   pbi->ptp_on);
        // lan743x_ptp_set_sync_ts_insert(adapter, false);
        break;
    case HWTSTAMP_TX_ONESTEP_SYNC:
        //.		db->ptp_onestep = true;
        pbi->ptp_on = 1;
        // dev_info(&adb->spidev->dev, "IOCtl - Set db->ptp_on %d, _ptp_set_sync_ts_insert(adapter, true)\n",
        // adb->ptp_on);
        if (all_know_allow_show)
            netif_info(db, hw, db->ndev,
                       "tx_type= _TX_ONESTEP_SYNC(2): _ptp_set_sync_ts_insert(adapter, true)\n"); //"Set db.ptp_on %d",
                                                                                                  //pbi->ptp_on
        // gem_ptp_set_one_step_sync(bp, 1);
        // lan743x_ptp_set_sync_ts_insert(adapter, true);
        break;
    case HWTSTAMP_TX_ON:
        //.		db->ptp_onestep = false;
        pbi->ptp_on = 1;
        netif_info(db, hw, db->ndev,
                   "tx_type= _TX_ON(1): _ptp_set_sync_ts_insert(adapter, false)\n"); //"Set db.ptp_on %d", pbi->ptp_on
        // gem_ptp_set_one_step_sync(bp, 0);
        // lan743x_ptp_set_sync_ts_insert(adapter, false);
        break;
    case HWTSTAMP_TX_ONESTEP_P2P:
        // ret = -ERANGE;
        netif_warn(db, hw, db->ndev, "tx_type= _TX_ONESTEP_P2P(3): Now db->ptp_on %d, Error Range !?! \n", pbi->ptp_on);
        return -ERANGE;
    // break;
    default:
        netif_warn(db, hw, db->ndev, "  tx_type = %d, UNKNOWN\n", config.tx_type);
        return -EINVAL;
        // ret = -EINVAL;
        // break;
    }

    switch (config.rx_filter)
    {
    case HWTSTAMP_FILTER_NONE:
        break;
    case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
        break;
    case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
        break;
    case HWTSTAMP_FILTER_PTP_V2_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
    case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
    case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
        // dev_info(&adb->spidev->dev, "config->rx_filter - to be, HWTSTAMP_FILTER_PTP_V2_EVENT\n"); //~ db->ptp_on = 1;
        if (all_know_allow_show)
            netif_info(db, hw, db->ndev, "rx_filter= _PTP_V2_EVENT(12): To be HWTSTAMP_FILTER_PTP_V2_EVENT\n");
        config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
        break;
    case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
    case HWTSTAMP_FILTER_ALL:
        // db->ptp_on = 1;
        netif_info(db, hw, db->ndev, "config->rx_filter - to be, HWTSTAMP_FILTER_ALL\n");
        config.rx_filter = HWTSTAMP_FILTER_ALL;
        break;
    default:
        netif_warn(db, hw, db->ndev, "  rx_filter = %d, UNKNOWN\n", config.rx_filter);
        config.rx_filter = HWTSTAMP_FILTER_NONE;
        return -ERANGE;
    }

    //	switch (config.tx_type) {
    //	case HWTSTAMP_TX_OFF:
    //		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
    //			index++)
    //			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
    //							 false, false);
    //		lan743x_ptp_set_sync_ts_insert(adapter, false);
    //		break;
    //	case HWTSTAMP_TX_ON:
    //		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
    //			index++)
    //			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
    //							 true, false);
    //		lan743x_ptp_set_sync_ts_insert(adapter, false);
    //		break;
    //	case HWTSTAMP_TX_ONESTEP_SYNC:
    //		for (index = 0; index < LAN743X_MAX_TX_CHANNELS;
    //			index++)
    //			lan743x_tx_set_timestamping_mode(&adapter->tx[index],
    //							 true, true);

    //		lan743x_ptp_set_sync_ts_insert(adapter, true);
    //		break;
    //	case HWTSTAMP_TX_ONESTEP_P2P:
    //		ret = -ERANGE;
    //		break;
    //	default:
    //		netif_warn(adapter, drv, adapter->netdev,
    //			   "  tx_type = %d, UNKNOWN\n", config.tx_type);
    //		ret = -EINVAL;
    //		break;
    //	}

    //	netif_info(db, hw, db->ndev, "_lan743x_ptp_ioctl = flag %d, tx_typ %d, rx_fltr %d\n",
    //		   config.flags,
    //		   config.tx_type,
    //		   config.rx_filter);

    /* copy to db _tstamp_config */
    memcpy(&pbi->tstamp_config, &config, sizeof(pbi->tstamp_config));

    /* copy to user */
    return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

/* netdev_ops
 * tell support ptp */
int dm9051_eth_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
    struct board_info *db  = to_dm9051_board(ndev);
    ptp_board_info_t  *pbi = &db->pbi;
    int ret;

    if (!netif_running(ndev))
        return -EINVAL;

    switch (cmd)
    {
    case SIOCGHWTSTAMP:
        // struct hwtstamp_config config;
        // return dm9051_ptp_get_ts_config(ndev, rq);
        ret = lan_ptp_get_ts_ioctl(ndev, rq);
        if (all_know_allow_show)
            netif_warn(db, hw, db->ndev, "_ptp_get_ts_ioctl/SIOCGHWTSTAMP = flag %d, tx_typ %d, rx_fltr %d\n",
                       pbi->tstamp_config.flags, pbi->tstamp_config.tx_type, pbi->tstamp_config.rx_filter);
        return ret;
    case SIOCSHWTSTAMP:
        // return dm9051_ptp_set_ts_config(ndev, rq);
        ret = lan743x_ptp_set_ts_ioctl(ndev, rq, cmd);
        if (all_know_allow_show)
            printk("_ptp_set_ts_ioctl/SIOCSHWTSTAMP = flag %d, tx_typ %d, rx_fltr %d [allow %d]\n",
                   pbi->tstamp_config.flags, pbi->tstamp_config.tx_type, pbi->tstamp_config.rx_filter,
                   all_know_allow_show);
        if (all_know_allow_show)
            all_know_allow_show--;
        return ret;
    case SIOCBONDINFOQUERY:
        printk("dm9051_netdev_ioctl SIOCBONDINFOQUERY = cmd 0x%X. NOT support\n", cmd);
        return -EOPNOTSUPP;
    default:
        printk("dm9051_netdev_ioctl phy_mii_ioctl, cmd = 0x%X\n", cmd);
        return phy_mii_ioctl(ndev->phydev, rq, cmd); //'rq' is ifr
    }
}
#endif // (defined(_DMPLUG_PTP) || defined(_DMPLUG_PTP_SW)) && defined(_MAIN_DATA)
