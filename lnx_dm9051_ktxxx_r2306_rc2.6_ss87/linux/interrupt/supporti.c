#if CODE_RX_REQUEST_SRC_CODE
//#if CODE_SETUP_INT_FIXED == 1
static int dm9051_request_irq_flag(struct board_info *db)
{
	//#define CODE_FOR_FALLING_EDGE_SEL		0	// Old kernel set 1, New kernel versions set 0 
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);
	
	//.printk("DM9 IRQGETTRIGGER_TYPE %d\n", irq_type);

	if (irq_type) {
		printk("DM9 IRQGETTRIGGER_TYPE get %d %s\n", irq_type, (irq_type == 8) ? "low" :
			(irq_type == 4) ? "high" : (irq_type == 2) ? "falling" : 
			(irq_type == 1) ? "rising" : "unknow");
		return irq_type;
	}

#if DAVICOM_CONF < DM9051_KERNEL_4_19
	//printk("DM9 level triggers won't currently work.\n");
	printk("DM9 Board setup must set the relevant edge trigger type;\n");
	printk("DM9 TO USINGTRIGGER_TYPE set %d\n", IRQF_TRIGGER_FALLING);
	return IRQF_TRIGGER_FALLING;
#else
	if (threadedcfg.interrupt_supp == THREADED_INT2) {
		//printk("DM9 level triggers won't currently work.\n");
		//printk("DM9 Board setup must set the relevant edge trigger type;\n");
		printk("DM9 TO USINGTRIGGER_TYPE set %d\n", IRQF_TRIGGER_FALLING);
		return IRQF_TRIGGER_FALLING;
	}

	//.printk("DM9 TO USINGTRIGGER_TYPE %d\n", IRQF_TRIGGER_LOW);
	return IRQF_TRIGGER_LOW;
#endif
}
//#endif
#endif //CODE_RX_REQUEST_SRC_CODE

//#if MOD_THREADED_INT
static void dm9051_interrupt_attr(char *headstr, char *tailstr, struct device *dev)
{
	dev_info(dev, "%s - %s%s\n", headstr, THREADED_MODE, tailstr);
	//if (threadedcfg.interrupt_supp == THREADED_INT)
	//	dev_info(dev, "%s - %s-mode\n", headstr, THREADED_MODE);
	//else if (threadedcfg.interrupt_supp == THREADED_INT2)
	//	dev_info(dev, "%s - %s-mode\n", headstr, THREADED_MODE);
}
//#endif
