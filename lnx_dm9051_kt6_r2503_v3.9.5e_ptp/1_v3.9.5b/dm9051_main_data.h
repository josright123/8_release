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
#endif //MAIN_DATA

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
		char *test_info;
		// int skb_wb_mode;
		int checksuming;
		struct align_config
		{
			char *burst_mode_info;
			int   burst_mode;
			u32   tx_blk;
			u32   rx_blk;
		} align;
	};

	#ifdef MAIN_DATA
	const struct plat_cnf_info plat_align_mode = {
		.test_info = "Test in rpi5 bcm2712",
		//.skb_wb_mode = SKB_WB_ON, //SKB_WB_OFF, //SKB_WB_ON,
		.checksuming = DEFAULT_CHECKSUM_OFF,
		.align       = {.burst_mode_info = "Alignment", .burst_mode = BURST_MODE_ALIGN, .tx_blk = 32, .rx_blk = 64},
	};
	const struct plat_cnf_info plat_misc_mode = {
		.test_info = "Test in processor Cortex-A",
		//.skb_wb_mode = SKB_WB_OFF,
		.checksuming = DEFAULT_CHECKSUM_OFF,
		.align       = {.burst_mode_info = "Burst", .burst_mode = BURST_MODE_FULL, .tx_blk = 0, .rx_blk = 0},
	};
	#endif

	/* Param structures
	 */
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
