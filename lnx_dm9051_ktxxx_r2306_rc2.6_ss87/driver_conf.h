/*
 * File : kernel_roadmap.h
 * 
 * If not handle this roadmap.h,
 * Get below unknown symbol, while insmod dm9051.ko in KT 5.15.76,
 *
 * [ 1141.140242] dm9051: Unknown symbol __devm_regmap_init_spi (err -2)
 * 
 */
#ifndef	KERNEL_ROADMAP_CONF_H_
#define	KERNEL_ROADMAP_CONF_H_

//#include "linux/linux_kernel_conf.h"
//#include "linux/test_kernel_conf.h"
//#include "linux/example_kernel_conf.h"
//#include "linux/kernel_conf(formal).h"
//#include "linux/teinpoa_kernel_conf.h"
#include "kernel_conf.h"

#include "kernel_config_conf.h"            //[Mandatory: for diff kernels' selection]

//Test. for local driver only compiler sanity check
//
// #define CONFIG_PM_SLEEP
// #define LOCAL_PM_SUPPORT
// 'wb'

#define	MUST_PM_SUPPORT				1
		/* define 0, normal networking support
		 * define 1, add suspend/resume support 
		 */

	//.#if MOD_REGMAP == 1
	#if CODE_SUPPORT_REGMAP
	/*
	 * when _MOD_REGMAP 1:
	 * (1) include below for the regmap revision for suit-compiler
	 */
	/*
	 * KT_4_1 not has regmap can use for spi. 
	 * KT 4.9 not know during this announce time. (DM9051_KERNEL_4_9)
	 */
	#include "linux/spi/regmap_spi/regmap_conf.h"          // handle for compiler compatible
        
    #if DAVICOM_CONF >= DM9051_KERNEL_4_19
        
        /* We discard below 3 includes, because we adopt the simplest 
        * model, as KT_4_1 for we found it is suitable for all later 
        * kernels (we hope it is true.)
         *
        #include "linux/spi/regmap/drv_kernel/hc._regmap_init.c"      // handle for compiler compatible
        #include "linux/spi/regmap/drv_kernel/hc.regmap_init.c"       // handle for compiler compatible

        #include "linux/spi/regmap/drv_heri/regmap_change.h"          // Joseph CHANG, __regmap_noinc_write(),__regmap_noinc_read()... 202306
        * = To be as belwo:
        */
        
		/* We are elder than Kernel 5.18 before dm9051.c had submit to LXR.
		 * When insmod dm9051.ko,
		 *   sm9051: Unknow symbol __devm_regmap_init_spi (err 01)
		 * pls add below: 
		 */
			
		/* Unknown symbol __devm_regmap_init_spi (err -2), while insmod dm9051.ko in KT 5.15.76,,
		 * Called by 'devm_regmap_init_spi()',
		 * We coding a '__devm_regmap_init_spi_dm' instead.
		 */
		#undef __devm_regmap_init_spi //compiler not undefined, since used in _wrapper
		struct regmap *__devm_regmap_init_spi_dm(struct spi_device *spi,
							  const struct regmap_config *config,
							  struct lock_class_key *lock_key,
							  const char *lock_name)
		{
			const struct regmap_bus *bus = &regmap_bus_dm;
			/*const struct regmap_bus *bus = regmap_get_spi_bus(spi, config);

			if (IS_ERR(bus))
				return ERR_CAST(bus);*/

			return __devm_regmap_init(&spi->dev, bus, &spi->dev, config, lock_key, lock_name);
		}

		#undef devm_regmap_init_spi
		#define devm_regmap_init_spi(dev, config)				\
			__regmap_lockdep_wrapper(__devm_regmap_init_spi_dm, #config,	\
						dev, config)
    #endif
	#endif //CODE_SUPPORT_REGMAP
	//.#endif

/* Driver construction (define driver version) -
 * - include to add the desc
 * - include to add the skb dump (for tx/rx)
 * - include to add the dts usage (or ndts manner)
 * - include skb protection
 */

#include "plugin/desc_conf.h"               // "dm9051_desc_v0.0.2.c"   //(Joseph CHANG, or as header file)
#include "plugin/skbdbg_conf.h"             // "dm9051_cache_skb_empty.c"   //(Joseph CHANG)
#include "plugin/dts_conf.h"				// "dm9051_cover_dts_v0.0.2.c"  //(Joseph CHANG)
#include "plugin/skbprot_conf.h"            // Joseph CHANG, 'wb', tx buffer read, further protection issue! 202306
#include "plugin/empty_endall.c"			//end-all

#ifndef	DM9051_DESC_H_
#define	dm9051_cache_desc_init(s, titleflg)		//[desc init]
#endif

#ifndef	CACHE_SKB_H_
#define	function_monitor_rx(ndev,skb)		//[monitor skb rx/tx]
#define	function_monitor_tx(ndev,skb)		//[monitor skb rx/tx]
#define	function_monitor_rxe(ndev)
#define	function_monitor_txe(ndev)
#endif

#ifndef	DM9051_COVER_DTS_H_
#define	dm9051_node_info(nc)		\
	if (nc) printk("*dm9 ADOPTE_DTS, system DTS \"davicom,dm9051\" found\n");	\
	else printk(" *dm9 ADOPTE_DTS, No \"davicom,dm9051\" DTS in the system !! (Note: do ndts is recommeded)\n")
						//[empty]
#define	dm9051_conf_spi_board(nc)
						//[empty][to customized if non-dts]
#define	dm9051_unconf_spi_board(nc)		\
	if (!nc) printk(" *dm9 exit WARN, No DTS davicom dm9051 compatible node found !\n")
						//[empty][to customized if non-dts]

#define	dm9051_dts_attr(headstr, tailstr, dev)	\
	dev_info(dev, "%s - %s%s\n", headstr, dts_str.dts_mode[ADOPTE_DTS], tailstr)
	
//#define	dm9051_spi_bus_attr(dev, num, cs)	
//	dev_info(dev, "from dts, spi%u cs%u\n", (unsigned int) num, (unsigned int) cs)
						//[to print busnum attributes from.]
#define dm9051_spi_speed_attr(dev, val)  	\
	dev_info(dev, "from dts, spi speed %u\n", (unsigned int) val)
						//[to print spi speed attributes from.]
//#define	dm9051_irq_num_attr(dev, irqno)	
//	dev_info(dev, "from dts, irqno %u\n", (unsigned int) irqno)
						//[to print irq no attributes from.]

#define dm9051_irq_speed_irqno_attr(dev, val, irqno)	\
	dev_info(dev, "from dts, spi speed %u irqno %u\n", (unsigned int) val, (unsigned int) irqno)
						//[to print spi speed & irqno attributes from.]
#endif

/* (, u32 max_speed_hz) */
/*	dev_info(dev, "%s - spi clock %u\n", headstr, max_speed_hz); // when probe, max_speed_hz is '00000000' (wait if we can get the reday value) */

//static void dm9051_threaded_attr(char *headstr, struct device *dev);= //(in xxsupport.c)
/* in ./linux/interrupt/support.c and/or ./linux/polling/support.c) */
static void dm9051_interrupt_attr(char *headstr, char *tailstr, struct device *dev); //(in ./linux/interrupt/support.c)
static void dm9051_poll_attr(char *headstr, char *tailstr, struct device *dev); //(in ./linux/polling/support.c)

static inline void dm9051_driver_info(char *headstr, char *tailstr, struct device *dev) {
	
	//if (!strcmp(headstr, "dm9051_probe"))
	//	dev_info(dev, "%s - software %s (%s)\n", headstr, DM9051_KT, DM9051_DRC);
    
	dm9051_dts_attr(headstr, tailstr, dev); //to be simpl
	if (threadedcfg.interrupt_supp == THREADED_POLL)
		dm9051_poll_attr(headstr, tailstr, dev); //to be simpl
	else
		dm9051_interrupt_attr(headstr, tailstr, dev); //to be simpl

#if CODE_SUPPORT_REGMAP
	if (xfer & RAW_REGMAP)
		dev_info(dev, "%s - %s%s\n", headstr, raw_str.raw_mode[0], tailstr); //be simpl
	else 
#endif
	if (xfer & RAW_SPISYN)
		dev_info(dev, "%s - %s%s\n", headstr, raw_str.raw_mode[1], tailstr); //be simpl
	else if (xfer & RAW_SPISYNB)
		dev_info(dev, "%s - %s%s\n", headstr, raw_str.raw_mode[2], tailstr); //be simpl
	else
		dev_info(dev, "%s - %s\n", headstr, "regmap-x unknow ................");
		
//.#if MOD_REGMAP == 0
    //dev_info(dev, "%s - regmap-0\n", headstr);
    //#if MOD_REGMAP0ExT == 1
    //dev_info(dev, "%s - regmap-0 ExT(tradition)\n", headstr);
    //#endif
    //#if MOD_REGMAP0ExB == 1
    //dev_info(dev, "%s - regmap-0 ExB(bust)\n", headstr);
    //#endif
//.#elif MOD_REGMAP == 1
    //dev_info(dev, "%s - regmap-1\n", headstr);
//.#endif

	if (phycfg.setup_phy & PHYCFG_LIB)
		dev_info(dev, "%s - %s%s\n", headstr, phy_str.phy_mode[0], tailstr); //be simpl
	else if (phycfg.setup_phy & PHYCFG_MII)
		dev_info(dev, "%s - %s%s\n", headstr, phy_str.phy_mode[1], tailstr); //"objmii exMII(mii read/write)"
	else if (phycfg.setup_phy & PHYCFG_MDIO)
		dev_info(dev, "%s - %s%s\n", headstr, phy_str.phy_mode[2], tailstr); //"objmii exMDIO(internal phy mdio read/write)"
	else
		dev_info(dev, "%s - %s\n", headstr, "phy-unknow check!! .............");

//.#if MOD_OBJ_MII == 1
	//if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dev_info(dev, "%s - objmii\n", headstr);
		//.#if CODE_SUPPORT_PHYMII == 1
		//.#if CODE_SUPP_MII == 1
		//if (phycfg.setup_phy & PHYCFG_MII)
		//	dev_info(dev, "%s - objmii exMII(mii read/write)\n", headstr);
		//else
		//.#endif
		//	dev_info(dev, "%s - objmii exMDIO(internal phy mdio read/write)\n", headstr);
	//}
//.#endif
//.#if MOD_PHYLIB == 1
	//if (phycfg.setup_phy & PHYCFG_LIB)
		//dev_info(dev, "%s - phylib\n", headstr);
//.#endif
}

static void dm9051_open_info(struct device *dev, struct spi_device *spi)
{
	dev_info(dev, "software %s (%s)\n", DM9051_KT, DM9051_DRC);

	//dm9051_spi_bus_attr(dev, spi->master->bus_num, spi->chip_select);
	if (threadedcfg.interrupt_supp == THREADED_POLL)
		dm9051_spi_speed_attr(dev, spi->max_speed_hz); //[display spi speed]
		//dm9051_irq_num_attr(dev, spi->irq);
	else
		dm9051_irq_speed_irqno_attr(dev, spi->max_speed_hz, spi->irq);

	//dev_info(&spi->dev, "from dts, controlr spi%u\n", (unsigned int) spi->controller->bus_num);
	//dev_info(&spi->dev, "from dts, controlr spicontroller num_chipselect %u\n", (unsigned int) spi->controller->num_chipselect);
	//dev_info(&spi->dev, "from dts, controlr spicontroller max_speed_hz %u\n", (unsigned int) spi->controller->max_speed_hz);
	//dev_info(&spi->dev, "from dts, master spicontroller num_chipselect %u\n", (unsigned int) spi->master->num_chipselect);
	//dev_info(&spi->dev, "from dts, spi cs_gpio %d\n", spi->cs_gpio);
	//dev_info(&spi->dev, "from dts, spi cs_gpio 0x%x\n", (unsigned int) spi->cs_gpio);
}

#endif //KERNEL_ROADMAP_CONF_H_
