//[2.driver options configurations]

#define THREADED_MODE               threaded_str.threaded_mode[threadedcfg.interrupt_supp]
//#define THREADED_MODE             threaded_str.threaded_mode[CODE_SETUP_INT_FIXED]

#define DRV_SPI_BUS_NUMBER			dtscfg.cfg.boardinfo.bus_num
#define DRV_SPI_CHIP_SELECT			dtscfg.cfg.boardinfo.chip_select
#define DM_CONF_MAX_SPEED_HZ		dtscfg.cfg.boardinfo.speed_hz
#define DM_CONF_INTERRUPT_IRQ		dtscfg.cfg.boardinfo.irq

/* Driver code compiler oriented configuration-
 */
 
#define CODE_SPIREGISTER_NDTS_SRC_CODE			1	// default 1, with ndts support code [Temp verify] 0, disable spi setup When verify such as older KT4.1 h/w

/*
 * User is basically use Linux DTS for his DM9051 SPI interface configurations,
 * So that the file ./plugin/dts_conf.h is to include "dm9051_empty_using_dts.c"
 * Or if not exist with Linux DTS for DM9051,
 * Pls edit the file ./plugin/dts_conf.h to include "dm9051_plugin_ndts_v0.0.3.c",
 * 
 * structure data 'dtscfg' is for user to define register form and the settings if
 * it is 'ADOPTE_NDTS'.
 * (Old: And then below structure data 'drv_spiconf' is for the user to define
 * the settings for DM9051 SPI interface configuration.)
 * 
 */

/* Selection
 */

static const struct dm9051_spi_conf dtscfg = {
//	(1) [REG_BOARD_DTS](option 1, dts)

	//.cfg = {
	//	.adopte_dts = ADOPTE_DTS,
	//},

//	(2) [REG_BOARD_NDTS](option 2, ndts)

	//.cfg = {
	//	.adopte_dts = ADOPTE_NDTS,
	//	.boardinfo = {
	//	  .bus_num = 0,
	//	  .chip_select = 1,
	//	  .speed_hz = 31200000,
	//	  .irq = 26,
	//	},
	//},

	 .cfg = {
		.adopte_dts = ADOPTE_NDTS,
		.boardinfo = {
		  .bus_num = 0,
		  .chip_select = 1,
		  .speed_hz = 31200000,
		  .irq = 26,
		},
	},

};

/* Driver code compiler oriented configuration-
 */

#define CODE_RX_REQUEST_SRC_CODE			1	// fit. CODE, either for interrupt or polling (this macro is large range than code_setup_int_fixed.)
	
/*
 * User who want to use polling, pls define 'CODE_SETUP_INT_FIXED' to 0,
 * In fact, polling is not recommanded since it is worse in performance and also occupied 
 * heavy CPU's resource!
 *
 * User who use interrupt, and had defined 'CODE_SETUP_INT_FIXED' to 1 above,
 * For the interrupt, User can select the interrupt threaded operation type as 1 or 2 s stages.
 * Basically 1 stage is ok, so define 'CODE_SETUP_INT_2STAGE' to 0,
 * Or if need 2 stages, pls define 'CODE_SETUP_INT_2STAGE' to 1.
 * 
 * .	#define CODE_SETUP_INT_FIXED				1 //1 //[0 is for polling], [1 is interrupt]	// fix. to 1, FIXED (ONLY WHEN ASKED, THEN CAN SET 0 TO USE POLLING, NOT BE RECOMMANDED)	 
 * .	#define CODE_SETUP_INT_2STAGE				1  // ORG: by = (DAVICOM_CONF < DM9051_KERNEL_4_19) 
 */

/* Selection
 */
 	
#define THREADED_POLL	0  //POLL
#define THREADED_INT	1  //1 STAGE
#define THREADED_INT2	2  //2 STAGE

static const struct dm9051_threaded threadedcfg = {
  //(1) .interrupt_supp = THREADED_POLL,
  //(2) .interrupt_supp = THREADED_INT,
  //(3) .interrupt_supp = THREADED_INT2,
 .interrupt_supp = THREADED_INT,
};

#define ETHn_NDEVOPS_FIXED					// essential, Used internal for if commented
