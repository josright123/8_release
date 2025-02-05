#ifndef	DM9051_COVER_DTS_H_
#define	DM9051_COVER_DTS_H_

//#undef	dm9051_node_info
//#undef	dm9051_conf_spi_board
//#undef	dm9051_unconf_spi_board

#define	dm9051_node_info(nc)		\
	if (nc) printk("*dm9 init NDTS-NoCODE, system DTS \"davicom,dm9051\" found\n");	\
	else printk(" *dm9 init NDTS-NoCODE, No \"davicom,dm9051\" DTS in the system !!\n")
						//[empty]
#define	dm9051_conf_spi_board(nc)		\
	printk(" *dm9 init NDTS NoCODE, for __spi_register_board_info of the driver !!\n")
						//[empty][to customized if non-dts]
#define	dm9051_unconf_spi_board(nc)		\
	printk(" *dm9 exit NDTS NoCODE, for _spi_unregister_board_info of the driver !!\n")
		

#undef  dm9051_dts_attr
#define	dm9051_dts_attr(headstr, tailstr, dev)	\
	if (dtscfg.cfg.adopte_dts == ADOPTE_DTS) \
		dev_info(dev, "%s - %s%s\n", headstr, dts_str.dts_mode[ADOPTE_DTS], tailstr); \
	else \
		dev_info(dev, "%s - %s%s\n", headstr, dts_str.dts_mode[ADOPTE_NDTS], tailstr)
	
//#undef  dm9051_spi_bus_attr
//#define dm9051_spi_bus_attr(dev, num, cs)
//		dev_info(dev, "const-define, spi%u cs%u\n", (unsigned int) num, (unsigned int) cs)

#undef  dm9051_spi_speed_attr
#define dm9051_spi_speed_attr(dev, val)  \
		if (dtscfg.cfg.adopte_dts == ADOPTE_DTS) \
			dev_info(dev, "from dts, spi speed %u\n", (unsigned int) val); \
		else \
			dev_info(dev, "const-define,  spi clock %u\n", (unsigned int) val)
		
//#undef  dm9051_irq_num_attr
//#define dm9051_irq_num_attr(dev, irqno)
//		dev_info(dev, "const-define, irqno %u\n", (unsigned int) irqno)

#undef	dm9051_irq_speed_irqno_attr
#define dm9051_irq_speed_irqno_attr(dev, val, irqno)	\
		if (dtscfg.cfg.adopte_dts == ADOPTE_DTS) \
			dev_info(dev, "from dts, spi speed %u irqno %u\n", (unsigned int) val, (unsigned int) irqno); \
		else \
			dev_info(dev, "const-define,  spi clock %u irqno %u\n", (unsigned int) val, (unsigned int) irqno)
	
		//printk("dm9* [ request threaded irq (irq %d from dts script) ]\n", irqno)
        //printk("dm9* request threaded irq (irq %d processed by davicom-cover-dts-v0.0.2.c src prgoram)\n", irqno)
/*static void dm9051_show_irq_num2(struct spi_device *spi) {
	printk("dm9* request threaded irq (irq %d processed by davicom-cover-dts-v0.0.2.c src prgoram)\n", spi->irq);
}*/

#if 1
	#if DAVICOM_CONF >= DM9051_KERNEL_5_15 // provide,  struct gpio_desc { int x; ... }
		//[verified range 5.15.30 ~ 5.15.61]
		struct gpio_device {
			int			id;
			struct device		dev;
			struct cdev		chrdev;
			struct device		*mockdev;
			struct module		*owner;
			struct gpio_chip	*chip;
			struct gpio_desc	*descs;
			int			base;
			u16			ngpio;
			const char		*label;
			void			*data;
			struct list_head        list;
			struct blocking_notifier_head notifier;

		#ifdef CONFIG_PINCTRL
			/*
			 * If CONFIG_PINCTRL is enabled, then gpio controllers can optionally
			 * describe the actual pin range which they serve in an SoC. This
			 * information would be used by pinctrl subsystem to configure
			 * corresponding pins for gpio usage.
			 */
			struct list_head pin_ranges;
		#endif
		};

		struct gpio_desc {
			struct gpio_device	*gdev;
			unsigned long		flags;
		/* flag symbols are bit numbers */
		#define FLAG_REQUESTED	0
		#define FLAG_IS_OUT	1
		#define FLAG_EXPORT	2	/* protected by sysfs_lock */
		#define FLAG_SYSFS	3	/* exported via /sys/class/gpio/control */
		#define FLAG_ACTIVE_LOW	6	/* value has active low */
		#define FLAG_OPEN_DRAIN	7	/* Gpio is open drain type */
		#define FLAG_OPEN_SOURCE 8	/* Gpio is open source type */
		#define FLAG_USED_AS_IRQ 9	/* GPIO is connected to an IRQ */
		#define FLAG_IRQ_IS_ENABLED 10	/* GPIO is connected to an enabled IRQ */
		#define FLAG_IS_HOGGED	11	/* GPIO is hogged */
		#define FLAG_TRANSITORY 12	/* GPIO may lose value in sleep or reset */
		#define FLAG_PULL_UP    13	/* GPIO has pull up enabled */
		#define FLAG_PULL_DOWN  14	/* GPIO has pull down enabled */
		#define FLAG_BIAS_DISABLE    15	/* GPIO has pull disabled */
		#define FLAG_EDGE_RISING     16	/* GPIO CDEV detects rising edge events */
		#define FLAG_EDGE_FALLING    17	/* GPIO CDEV detects falling edge events */
		#define FLAG_EVENT_CLOCK_REALTIME	18 /* GPIO CDEV reports REALTIME timestamps in events */

			/* Connection label */
			const char		*label;
			/* Name of the GPIO */
			const char		*name;
		#ifdef CONFIG_OF_DYNAMIC
			struct device_node	*hog;
		#endif
		#ifdef CONFIG_GPIO_CDEV
			/* debounce period in microseconds */
			unsigned int		debounce_period_us;
		#endif
		};
	#else //[e.g. DM9051_KERNEL_5_4]
		//[verified range 5.4]
		struct gpio_device {
			int			id;
			struct device		dev;
			struct cdev		chrdev;
			struct device		*mockdev;
			struct module		*owner;
			struct gpio_chip	*chip;
			struct gpio_desc	*descs;
			int			base;
			u16			ngpio;
			const char		*label;
			void			*data;
			struct list_head        list;

		#ifdef CONFIG_PINCTRL
			/*
			 * If CONFIG_PINCTRL is enabled, then gpio controllers can optionally
			 * describe the actual pin range which they serve in an SoC. This
			 * information would be used by pinctrl subsystem to configure
			 * corresponding pins for gpio usage.
			 */
			struct list_head pin_ranges;
		#endif
		};

		struct gpio_desc {
			struct gpio_device	*gdev;
			unsigned long		flags;
		/* flag symbols are bit numbers */
		#define FLAG_REQUESTED	0
		#define FLAG_IS_OUT	1
		#define FLAG_EXPORT	2	/* protected by sysfs_lock */
		#define FLAG_SYSFS	3	/* exported via /sys/class/gpio/control */
		#define FLAG_ACTIVE_LOW	6	/* value has active low */
		#define FLAG_OPEN_DRAIN	7	/* Gpio is open drain type */
		#define FLAG_OPEN_SOURCE 8	/* Gpio is open source type */
		#define FLAG_USED_AS_IRQ 9	/* GPIO is connected to an IRQ */
		#define FLAG_IRQ_IS_ENABLED 10	/* GPIO is connected to an enabled IRQ */
		#define FLAG_IS_HOGGED	11	/* GPIO is hogged */
		#define FLAG_TRANSITORY 12	/* GPIO may lose value in sleep or reset */
		#define FLAG_PULL_UP    13	/* GPIO has pull up enabled */
		#define FLAG_PULL_DOWN  14	/* GPIO has pull down enabled */

			/* Connection label */
			const char		*label;
			/* Name of the GPIO */
			const char		*name;
		};
	#endif
#endif //1

#if CODE_SPIREGISTER_NDTS_SRC_CODE //Should be always stick to be 1, only testing temp to be 0.

	#undef	dm9051_node_info
	#undef	dm9051_conf_spi_board
	#undef	dm9051_unconf_spi_board

	#define	dm9051_node_info(nc) \
			do { \
				if (nc) { \
					if (dtscfg.cfg.adopte_dts == ADOPTE_DTS) \
						printk("*dm9 ADOPTE_DTS, system has \"davicom,dm9051\" DTS found\n"); \
					else \
						printk("*dm9 ADOPTE_NDTS, system has \"davicom,dm9051\" DTS found\n"); \
				} else { \
					if (dtscfg.cfg.adopte_dts == ADOPTE_DTS) \
						printk("*dm9 ADOPTE_DTS, No DTS davicom dm9051 compatible node found !!\n"); \
					else \
						printk("*dm9 ADOPTE_NDTS, No DTS davicom dm9051 compatible node found !!\n"); \
				} \
			} while(0)
	#define	dm9051_conf_spi_board(nc)		dm_spi_register_board_info(nc)
	#define	dm9051_unconf_spi_board(no_used_nc)		dm_spi_unregister_board_info()

	static struct spi_board_info dm9051_spi_board_devs[] __initdata = {
	  [0] = {
		.modalias = "dm9051",
		.mode = SPI_MODE_0,
		//.bus_num = DRV_SPI_BUS_NUMBER,
		//.chip_select = DRV_SPI_CHIP_SELECT,
		//.max_speed_hz = DM_CONF_MAX_SPEED_HZ,
		//.irq = DM_CONF_INTERRUPT_IRQ,
	  },
	};

	struct gpio_st {
	  int gpio;
	};

	#define GPIO_ANY_GPIO_DESC	"int_pin_dm9051"
	static struct spi_device *spi_device;
	static struct gpio_st gd = {
	  .gpio = 0,  //empty value
	};

	static int gpio_config_gpio_to_irq(int enter_spidev_irq, int gpio_num) {
		int irq;

	  //printk("---------------------------------------------------------\n");
	  //printk("dm9 No dm9051 DTS in the system !!\n");
	  //printk("dm9 gpio_request, pin %d, desc '%s'\n", gpio_num, GPIO_ANY_GPIO_DESC);
	  //printk("---------------------------------------------------------\n");

	  //#include <../drivers/gpio/gpiolib.h> //'struct gpio_desc'		
	  
		if (gpio_to_desc(gpio_num)) {
			struct gpio_desc *gdesc = gpio_to_desc(gpio_num);
			if (gdesc->label) {

			  //printk("dm9 _gpio_to_desc(%d) ok, has gpio_desc label '%s'\n", gpio_num, gdesc->label);
			  //printk("dm9 _gpio_free(%d) and then call gpio_request(%d, '%s')\n", gpio_num, gpio_num, GPIO_ANY_GPIO_DESC);
			  
			  gpio_free(gpio_num); //called in next-round, (this is not good if change to use on dm9051.c-DTS-mode.)
			} 
			//else
			  //printk("*dm9 gpio_to_desc(%d) ok, but gpio_desc label (null)\n", gpio_num);

			//printk("*dm9 desc way, gpio_to_desc(%d) is true, next call gpio_request(%d, '%s')\n", gpio_num, gpio_num, GPIO_ANY_GPIO_DESC);
		}
		//else
			//printk("*dm9 special way, gpio_to_desc(%d) is NULL, next call gpio_request(%d, '%s')\n", gpio_num, gpio_num, GPIO_ANY_GPIO_DESC);
		
		//.#if 1
		//.printk("*dm9 gpio_request(%d, '%s')\n", gpio_num, GPIO_ANY_GPIO_DESC);
		//.#endif
	 
		if (gpio_request(gpio_num, GPIO_ANY_GPIO_DESC)) {	
			printk("---------------------------------------------------------\n");
			printk("dm9051 gpio_request Error! pin %d, desc '%s'\n", gpio_num, GPIO_ANY_GPIO_DESC);
			printk("---------------------------------------------------------\n");
			//gpio_free(gpio_num); //called later
			return -1;
		}
		gpio_direction_input(gpio_num);
		irq = gpio_to_irq(gpio_num);  //exp: enum gpio_to_irq( 17) = 187
		if (irq <= 0) { // jj:enum
			printk("Error! dm9051 failed to get irq_no, %d\n", irq);
			return irq;
		}
		//printk("*dm9 gpio_request(%d) & gpio_to_irq(%d) make a new spi device org(%d)to(%d)\n", gpio_num, gpio_num, enter_spidev_irq, irq);
		return irq;
	}
	
	/* Joseph: find/delete/new */

	static void dm9051_device_spi_delete(struct spi_master * master, unsigned cs) {
	  struct device * dev;
	  char str[32];

	  snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);
	  dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	  if (dev) {
		device_del(dev);
		//.#if 1
		//.printk("*dm9 Delete the %s device, first\n", str);
		//.#endif
	  }
	}

	static void dm_spi_unregister_board_info(void)
	{
	  //printk("---------------------------------------------------------\n");
	  //printk("dm9051 _gpio_free, pin %d, desc '%s'\n", gd.gpio, GPIO_ANY_GPIO_DESC);
	  //printk("---------------------------------------------------------\n");
	  //gpio_free(gpio_num); //called later
	  if (spi_device) {
		device_del(&spi_device->dev);
		kfree(spi_device);
	  }
	}

	static int dm_spi_register_board_info(struct device_node *nc) {
		
	// int spi_register_board_info(strucr spi_board_info *, unsigned n) {}
	//   struct spi_board_info *spi = dm9051_spi_board_devs,
	//   unsigned n = ARRAY_SIZE(dm9051_spi_board_devs),
	  /* Joseph_20151030: 'n' is always 1, ARRAY_SIZE(table) is 1 table-item in this design  */
	  struct spi_board_info *spi = dm9051_spi_board_devs;
	  struct spi_master * master;
	  
	  spi->bus_num = DRV_SPI_BUS_NUMBER;
	  spi->chip_select = DRV_SPI_CHIP_SELECT;
	  spi->max_speed_hz = DM_CONF_MAX_SPEED_HZ;
	  spi->irq = DM_CONF_INTERRUPT_IRQ;
	  
	  spi_device = NULL; // most init.
	  
	  master = spi_busnum_to_master(spi->bus_num);
	  if (!master) {
		printk("*dm9 spi_busnum_to_master(%d) returned NULL\n", spi->bus_num);
		pr_err("*dm9 spi_busnum_to_master(%d) returned NULL\n",
		  spi->bus_num);
		return -EINVAL;
	  }

	  /* make sure it's available */
	  dm9051_device_spi_delete(master, spi->chip_select);
	  spi_device = spi_new_device(master, spi);
	  put_device(&master->dev);
	  //#if 1
	  //char str[32];
	  //snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), spi->chip_select);
	  //printk("*dm9 New a %s device in master bus\n", str);
	  //#endif
	  if (!spi_device) {
		pr_info(DRVNAME_9051 ":    spi_new_device() returned NULL\n");
		pr_err(DRVNAME_9051 ":    spi_new_device() returned NULL\n");
		return -EPERM;
	  }

	  /* created by 'spi_new_device' is here! */
	  //printk("dm9 new config spi_board_info [spi->max_speed_hz %u]\n", (unsigned int) spi->max_speed_hz);
	  //printk("dm9 new create spi_device [spi_device->max_speed_hz %u]\n", (unsigned int) spi_device->max_speed_hz);

	  gd.gpio = spi->irq;
	  
	  printk("*dm9 gpio_request(%d, '%s')\n", gd.gpio, GPIO_ANY_GPIO_DESC);
	  spi_device->irq = gpio_config_gpio_to_irq(spi_device->irq, gd.gpio); //New.JJ.20220907
	  if (spi_device->irq <= 0) {
		dm_spi_unregister_board_info();
		return -EPERM;
	  }
	  //.printk("*dm9 gpio %d (to) irqno %d\n", spi->irq, spi_device->irq);
	  return 0;
	}
#endif //CODE_SPIREGISTER_NDTS_SRC_CODE

#endif //DM9051_COVER_DTS_H_
