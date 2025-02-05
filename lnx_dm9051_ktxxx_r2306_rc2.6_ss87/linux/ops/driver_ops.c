/* (driver_conf.h)
 */
 
/* ethtool-ops
 */
static void dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	#if DAVICOM_CONF >= DM9051_KERNEL_4_19
		strscpy(info->driver, DRVNAME_9051, sizeof(info->driver));
	#else
		strlcpy(info->driver, DRVNAME_9051, sizeof(info->driver));
	#endif
}

static void dm9051_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->msg_enable = value;
}

static u32 dm9051_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	return db->msg_enable;
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
	return 128;
}

static int dm9051_get_eeprom(struct net_device *ndev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2) {
		ret = dm9051_eeprom_read(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static int dm9051_set_eeprom(struct net_device *ndev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2) {
		ret = dm9051_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static void dm9051_get_pauseparam(struct net_device *ndev,
				  struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	*pause = db->pause;
}

static int dm9051_set_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return dm9051_update_fcr(db);

//.#if MOD_PHYLIB == 1
	if (phycfg.setup_phy & PHYCFG_LIB) {
	#if CODE_SUPPORT_PHYLIB
		#if DAVICOM_CONF > DM9051_KERNEL_4_1
		phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
				  pause->autoneg);
		phy_start_aneg(db->phydev);
		#else
		//
		//#error "must CODE_SUPPORT_PHYLIB then can select (phycfg.setup_phy & PHYCFG_LIB).JJ"
		//CODE keep (phycfg.setup_phy & PHYCFG_LIB), CODE got empty block section.JJ
		/* KT_4_1 has no phy_set_sym_pause()
		 */
		if (pause->rx_pause || pause->autoneg) {
			db->phydev->supported |= ADVERTISED_Pause;
			db->phydev->advertising |= ADVERTISED_Pause;
		} else {
			db->phydev->supported &= ~ADVERTISED_Pause;
			db->phydev->advertising &= ~ADVERTISED_Pause;
		}
		
		if (pause->autoneg)
			phy_start_aneg(db->phydev);
		#endif
	#else
		//
		//#error "must CODE_SUPPORT_PHYLIB then can select (phycfg.setup_phy & PHYCFG_LIB).JJ"
		//CODE keep (phycfg.setup_phy & PHYCFG_LIB), CODE got empty block section.JJ
	#endif
	}
//.#endif
	
	return 0;
}

//#if BOARD_CONF >= DM9051_KERNEL_5_15 || DAVICOM_CONF >= DM9051_KERNEL_4_19
		//static int dm9051_get_link_ksettings(struct net_device *dev, //LNX_KERNEL_v58
		//					 struct ethtool_link_ksettings *cmd)
		//{
		//}
		//static int dm9051_set_link_ksettings(struct net_device *dev, //LNX_KERNEL_v58
		//					 const struct ethtool_link_ksettings *cmd)
		//{
		//}
//#else
		//static int dm9051_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
		//{
		//}
		//static int dm9051_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
		//{
		//}
//#endif

/*static int dm9051_nway_reset(struct net_device *dev) {
	#if CODE_SUPPORT_PHYMII == 1
	//.#if CODE_SUPP_MII == 1
	if (phycfg.setup_phy & PHYCFG_MII) {
		struct board_info *dm = to_dm9051_board(dev);
		return mii_nway_restart(&dm->mii);
	} else
	#endif
		return mdio_nway_restart();
}

static u32 dm9051_get_link(struct net_device *dev) {
	#if CODE_SUPPORT_PHYMII == 1
	//.#if CODE_SUPP_MII == 1
	if (phycfg.setup_phy & PHYCFG_MII) {
		struct board_info *db = to_dm9051_board(dev);
		return mii_link_ok(&db->mii);
	} else
	#endif
		return mdio_link_ok(dev);
}*/
/*
#if MOD_OBJ_MII == 1
	//#if BOARD_CONF >= DM9051_KERNEL_5_15 || DAVICOM_CONF >= DM9051_KERNEL_4_19
	.get_link_ksettings = dm9051_get_link_ksettings,
	.set_link_ksettings = dm9051_set_link_ksettings,
	#else
	.get_settings = dm9051_get_settings,
	.set_settings = dm9051_set_settings,
	#endif
	.nway_reset = dm9051_nway_reset,
	.get_link = dm9051_get_link,
#elif MOD_PHYLIB == 1
	//#if BOARD_CONF >= DM9051_KERNEL_5_15 || DAVICOM_CONF >= DM9051_KERNEL_4_19
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.nway_reset = phy_ethtool_nway_reset,
	#else
	.get_settings = dm9051_get_settings,
	.set_settings = dm9051_set_settings,
	.nway_reset = dm9051_nway_reset,
	#endif
	.get_link = ethtool_op_get_link,
#endi
*/

#if BOARD_CONF >= DM9051_KERNEL_5_15 || DAVICOM_CONF >= DM9051_KERNEL_4_19
static int syn_get_link_ksettings(struct net_device *dev, //LNX_KERNEL_v58
							 struct ethtool_link_ksettings *cmd)
{
	struct board_info *dm = to_dm9051_board(dev);

	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dm9051_get_link_ksettings(dev, cmd);=
		mii_ethtool_get_link_ksettings(&dm->mii, cmd);
		return 0;
	}
	return phy_ethtool_get_link_ksettings(dev, cmd);
}
static int syn_set_link_ksettings(struct net_device *dev, //LNX_KERNEL_v58
					 const struct ethtool_link_ksettings *cmd)
{
	struct board_info *dm = to_dm9051_board(dev);

	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO))
		//dm9051_set_link_ksettings(dev, cmd);=
		return mii_ethtool_set_link_ksettings(&dm->mii, cmd);
	return phy_ethtool_set_link_ksettings(dev, cmd);
}
#else
static int syn_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
#if CODE_SUPPORT_PHYMII | CODE_SUPPORT_PHYLIB
	struct board_info *dm = to_dm9051_board(dev);
#endif
	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dm9051_get_settings(dev, cmd);=
		
		#if CODE_SUPPORT_PHYMII == 1
		if (phycfg.setup_phy & PHYCFG_MII) {
			mii_ethtool_gset(&dm->mii, cmd);
			return 0;
		}
		#endif
		mdio_ethtool_gset();
		return 0;
	}
#if CODE_SUPPORT_PHYLIB
	return phy_ethtool_gset(dm->phydev, cmd);
#else
	return 0;
#endif
}
static int syn_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
#if CODE_SUPPORT_PHYMII | CODE_SUPPORT_PHYLIB
	struct board_info *dm = to_dm9051_board(dev);
#endif
	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dm9051_set_settings(dev, cmd);=
		
		#if CODE_SUPPORT_PHYMII == 1
		if (phycfg.setup_phy & PHYCFG_MII)
			return mii_ethtool_sset(&dm->mii, cmd);
		#endif
		return mdio_ethtool_sset();
	}
#if CODE_SUPPORT_PHYLIB
	return phy_ethtool_sset(dm->phydev, cmd);
#else
	return 0;
#endif
}
#endif

static int syn_nway_reset(struct net_device *dev)
{
	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dm9051_nway_reset(dev);=
		#if CODE_SUPPORT_PHYMII == 1
		struct board_info *dm = to_dm9051_board(dev);
		if (phycfg.setup_phy & PHYCFG_MII)
			return mii_nway_restart(&dm->mii);
		#endif
		return mdio_nway_restart();
	}
	#if CODE_SUPPORT_PHYLIB
	if (phycfg.setup_phy == PHYCFG_LIB) {
		struct board_info *dm = to_dm9051_board(dev);
		return genphy_restart_aneg(dm->phydev);
	}
	#endif
	return 0;
}
static u32 syn_get_link(struct net_device *dev)
{
	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//dm9051_get_link(dev);=
		#if CODE_SUPPORT_PHYMII == 1
		struct board_info *db = to_dm9051_board(dev);
		if (phycfg.setup_phy & PHYCFG_MII)
			return mii_link_ok(&db->mii);
		#endif
		return mdio_link_ok(dev);
	}
	return ethtool_op_get_link(dev);
}

static const struct ethtool_ops dm9051_ethtool_ops = {
	.get_drvinfo = dm9051_get_drvinfo,
	#if BOARD_CONF >= DM9051_KERNEL_5_15 || DAVICOM_CONF >= DM9051_KERNEL_4_19
	.get_link_ksettings = syn_get_link_ksettings,
	.set_link_ksettings = syn_set_link_ksettings,
	#else
	.get_settings = syn_get_settings,
	.set_settings = syn_set_settings,
	#endif
	.nway_reset = syn_nway_reset,
	.get_link = syn_get_link,

	.get_msglevel = dm9051_get_msglevel,
	.set_msglevel = dm9051_set_msglevel,
	.get_eeprom_len = dm9051_get_eeprom_len,
	.get_eeprom = dm9051_get_eeprom,
	.set_eeprom = dm9051_set_eeprom,
	.get_pauseparam = dm9051_get_pauseparam,
	.set_pauseparam = dm9051_set_pauseparam,
};

#if CODE_RX_REQUEST_SRC_CODE

/* read packets from the fifo memory
 * return value,
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data error, caller escape from rx operation
 */
static int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
#if 0
	unsigned int rxbyte;
#else
	u8 rxbyte[2];
#endif
	int ret, rxlen;
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;

	do {
		ret = dm9051_read_mem(db, DM_SPI_MRCMDX, &rxbyte[0], 2);
		if (ret)
			return ret;
			
		if ((rxbyte[1] & GENMASK(7, 0)) != DM9051_PKT_RDY)
			break; /* exhaust-empty */

#if 0
		printk("dm9 rxbyte %02x %02x, rxbyte[1] & GENMASK(7, 0)= %02lx ; (GENMASK %02lx)\n", 
			rxbyte[0], rxbyte[1],
			rxbyte[1] & GENMASK(7, 0), GENMASK(7, 0));
#endif

		ret = dm9051_read_mem(db, DM_SPI_MRCMD, &db->rxhdr, DM_RXHDR_SIZE);
		if (ret)
			return ret;

		ret = dm9051_stop_mrcmd(db);
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		if (db->rxhdr.status & RSR_ERR_BITS || rxlen > DM9051_PKT_MAX) {
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   db->rxhdr.headbyte);

			if (db->rxhdr.status & RSR_ERR_BITS) {
				db->bc.status_err_counter++;
				netdev_dbg(ndev, "check rxstatus-eror (%02x)\n",
					   db->rxhdr.status);
			} else {
				db->bc.large_err_counter++;
				netdev_dbg(ndev, "check rxlen large-eror (%d > %d)\n",
					   rxlen, DM9051_PKT_MAX);
			}
			return dm9051_all_restart(db);
		}

		skb = dev_alloc_skb((rxlen + 1) & (~1)); //'wb'
		if (!skb) {
			ret = dm9051_dumpblk(db, DM_SPI_MRCMD, (rxlen + 1) & (~1)); //dm9051_dumpblk(); //'wb'
			if (ret < 0)
				return ret;
			return scanrr;
		}

		#if 0
		if (rxlen & 1)
			printk("[dm9051 MUST_WB_SUPPORT] rxlen %d, rx-mem16 %d\n", rxlen, (rxlen + 1) & (~1));
		#endif

		rdptr = skb_put(skb, rxlen - 4);
		ret = dm9051_read_mem(db, DM_SPI_MRCMD, rdptr, (rxlen + 1) & (~1)); //'wb'
		if (ret) {
			db->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			return ret;
		}

		ret = dm9051_stop_mrcmd(db);
		if (ret)
			return ret;

		function_monitor_rx(ndev, skb);
		skb->protocol = eth_type_trans(skb, db->ndev);
		if (db->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx_ni(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		function_monitor_rxe(db->ndev);
		scanrr++;
	} while (!ret);

	return scanrr;
}

static irqreturn_t dm9051_rx_threaded_irq(int irq, void *pw)
{
	/* consider this interrupt service routine,
	 * This is by INT event from DM9051 INT pin-24, so is hardware driven!
	 */
	struct board_info *db = pw;
	int result, result_tx;

	mutex_lock(&db->spi_lockm);

	result = dm9051_disable_interrupt(db);
	if (result)
		goto out_unlock;
		
//.printk("dm9051_rx_threaded_irq: irq= %d\n", irq);

	result = dm9051_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do {
		result = dm9051_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;
		result_tx = dm9051_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
	} while (result > 0);

	dm9051_enable_interrupt(db);

	/* To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}
#endif

//#if CODE_SETUP_INT_FIXED == 0 //[ON POLLING]= 'dm9051_continue_poll'
//#endif
#define DM_TIMER_EXPIRE1	1
#define DM_TIMER_EXPIRE2	0
#define DM_TIMER_EXPIRE3	0

static void dm9051_irq_delayp(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct board_info *db = container_of(dwork, struct board_info, irq_workp);
	
	dm9051_rx_threaded_irq(0, db); // 0 is no-used.
	/* consider not be 0, to alower and not occupy almost all CPU resource.
	 * This is by CPU scheduling-poll, so is software driven!
	 */
	schedule_delayed_work(&db->irq_workp, DM_TIMER_EXPIRE1);
}

static void dm9051_irq_delayi(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, irq_worki);
	dm9051_rx_threaded_irq(0, db); // 0 is no-used.
}

static irqreturn_t dm9051_irq_2stg(int irq, void *pw)
{
	/* Also consider this interrupt service routine,
	 * This is by INT event from DM9051 INT pin-24, so is hardware driven!
	 * Here we have one more step by apply a schedule working delay, because
	 * when less KT4.19, the system need it to avoid crash during the interrupt
	 * service procedures! 
	 */
	struct board_info *db = pw;
	
	schedule_work(&db->irq_worki);
	return IRQ_HANDLED;
}

static int dm9051_irq_setup(int irqno, char *ndev_name, struct board_info *db)
{	
	db->intcr_all = INTCR_POL_LOW;
	
#if CODE_RX_REQUEST_SRC_CODE
//#if CODE_SETUP_INT_FIXED == 1
	if (threadedcfg.interrupt_supp == THREADED_INT ||
			threadedcfg.interrupt_supp == THREADED_INT2)
	/*do*/ 
	{
			int ret;
			int irq_flag = dm9051_request_irq_flag(db);

			db->intcr_all = (irq_flag == IRQF_TRIGGER_FALLING || irq_flag == IRQF_TRIGGER_LOW) ?
							  INTCR_POL_LOW : INTCR_POL_HIGH;		
			//.printk("GET DM9051_INTCR = 0x%02x\n", db->intcr_all);
			
			//#if CODE_SETUP_INT_2STAGE //[DAVICOM_CONF < DM9051_KERNEL_4_19]
			if (threadedcfg.interrupt_supp == THREADED_INT2) {
				/* KT-4.1.11 using,
				 * request_irq();
				 * About IRQF_ONESHOT,
				 * Used by threaded interrupts which need to keep the irq
				 * line disabled until the threaded handler has been run.
				 */
				#if DAVICOM_CONF >= DM9051_KERNEL_4_19 //[request_threaded_irq() test in rpi4 KT4.15.76 ok]
					/* IRQF_SHARED */
					//printk("has = 0x%02x, IRQF_SHARED is 0x%02x, request | IRQF_SHARED =  0x%02x\n", irq_flag, IRQF_SHARED, irq_flag | IRQF_SHARED);
					//ret = request_irq(spi->irq, dm9051_irq, irq_flag
					//					| IRQF_SHARED
					//					, ndev->name, db);
										
					//.printk("USINGTRIGGER = 0x%02x, IRQF_ONESHOT is 0x%02x, request | IRQF_ONESHOT =  0x%02x\n", irq_flag, IRQF_ONESHOT, irq_flag | IRQF_ONESHOT);
					ret = request_threaded_irq(irqno, NULL, dm9051_irq_2stg, irq_flag 
												   | IRQF_ONESHOT,
												   ndev_name, db);
				#else //[request_irq() only that ever ok tested in rpi2 KT4.1.11]
					/* IRQF_ONESHOT */
					//printk("but = 0x%02x, IRQF_ONESHOT is 0x%02x, request | IRQF_ONESHOT =  0x%02x\n", irq_flag, IRQF_ONESHOT, irq_flag | IRQF_ONESHOT);
					//ret = request_threaded_irq(spi->irq, NULL, dm9051_irq, irq_flag  //dm9051_rx_threaded_irq
					//							   | IRQF_ONESHOT
					//								, ndev->name, db);
					
					//.printk("USINGTRIGGER = 0x%02x, no IRQF add, request =  0x%02x\n", irq_flag, irq_flag);
					ret = request_irq(irqno, dm9051_irq_2stg, irq_flag, ndev_name, db);
				#endif
			} else {
				/* IRQF_ONESHOT, Morden style */
				//.printk("USINGTRIGGER = 0x%02x, IRQF_ONESHOT is 0x%02x, request | IRQF_ONESHOT =  0x%02x\n", irq_flag, IRQF_ONESHOT, irq_flag | IRQF_ONESHOT);
				ret = request_threaded_irq(irqno, NULL, dm9051_rx_threaded_irq, irq_flag 
											   | IRQF_ONESHOT,
											   ndev_name, db);
			}
			//#else
			//#endif
			if (ret < 0)
				return ret;
	} /*while (0);*/
//#endif
#endif
	
	return 0;
}

/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;

	dm9051_cache_desc_init("dm9051_open", 0);
	db->imr_all = IMR_PAR | IMR_PRM;
	db->lcr_all = LMCR_MODE1;
	db->rctl.rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

  dm9051_open_info(&spi->dev, spi);

	ndev->irq = spi->irq; /* either by dts or by source gpiolib process */
		
	ret = dm9051_irq_setup(spi->irq, ndev->name, db);
	if (ret < 0) {
		netdev_info(ndev, "err %d, request irq fail\n", ret); //netdev_err()
		return ret;
	}

	//printk("dm9 : request threaded irq.e (irq %d) operate on ndev, ndev->irq= %d\n", spi->irq, ndev->irq);
	
	#if CODE_SUPPORT_PHYLIB
	if (phycfg.setup_phy == PHYCFG_LIB) {
		phy_support_sym_pause(db->phydev);
		phy_start(db->phydev);
	}
	#endif

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	#if CODE_SUPPORT_PHYLIB
	//.#if MOD_PHYLIB == 1
	if ((phycfg.setup_phy & PHYCFG_LIB) &&
		db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;
	//.#endif
	#endif

	ret = dm9051_all_start(db);
	if (ret) {
		#if CODE_SUPPORT_PHYLIB
		if (phycfg.setup_phy == PHYCFG_LIB)
			phy_stop(db->phydev);
		#endif
		free_irq(spi->irq, db);
		return ret;
	}

	netif_wake_queue(ndev);

	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO)) {
		//#if MOD_OBJ_MII == 1
		#if CODE_SUPPORT_PHYMII == 1
		//.#if CODE_SUPP_MII == 1
		if (phycfg.setup_phy & PHYCFG_MII)
			dm_carrier_init(db);
		else
		#endif
			mdio_check_link(db);
		init_sched_phy(db);
		//#endif
	}
	printk("\n");
	
//#if CODE_SETUP_INT_FIXED == 0 //[START]
//#endif
	if (threadedcfg.interrupt_supp == THREADED_POLL)
		schedule_delayed_work(&db->irq_workp, HZ * 1); // 1 second when start
	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = dm9051_all_stop(db);
	if (ret)
		return ret;

	//.#if MOD_OBJ_MII == 1
	if (phycfg.setup_phy & (PHYCFG_MII | PHYCFG_MDIO))
		cancel_delayed_work_sync(&db->phy_poll); //~flush_work();
	//.#endif

	//.printk("\n");
	//.dm9051_cache_desc_init("dm9051_stop", 0);
	
//#if CODE_SETUP_INT_FIXED == 0 //[FINI]
//	cancel_delayed_work_sync(&db->irq_workp);
//#elif CODE_SETUP_INT_2STAGE //[DAVICOM_CONF < DM9051_KERNEL_4_19]
//	flush_work(&db->irq_worki);
//#endif
	if (threadedcfg.interrupt_supp == THREADED_POLL)
		cancel_delayed_work_sync(&db->irq_workp);
	else if (threadedcfg.interrupt_supp == THREADED_INT2)
		flush_work(&db->irq_worki);

	//.#if has_ndev_ops
	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);
	//.#endif

	#if CODE_SUPPORT_PHYLIB
	if (phycfg.setup_phy == PHYCFG_LIB)
		phy_stop(db->phydev);
	#endif

//#if CODE_SETUP_INT_FIXED == 1
#if CODE_RX_REQUEST_SRC_CODE
	if (threadedcfg.interrupt_supp == THREADED_INT ||
		threadedcfg.interrupt_supp == THREADED_INT2)
		free_irq(db->spidev->irq, db);
#endif
//#endif

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

#if 1
	//struct device *dev = &db->spidev->dev;
	//dm9051_driver_info("dm9051_stop", ".", dev); //( db->spidev->max_speed_hz);
#endif

	#if 0
	printk("*dm9 free irq (irq %d) basic is ndev->irq= %d\n", db->spidev->irq, ndev->irq);
	#endif

	return 0;
}
