//.#if MOD_OBJ_MII == 1
/*
 * carrier
 */
#define init_sched_phy(db)          schedule_delayed_work(&db->phy_poll, HZ * 1) //sched_start
#define re_sched_phy(db)            schedule_delayed_work(&db->phy_poll, HZ * 1)

#if CODE_SUPPORT_PHYMII == 1
#define dm_carrier_on(dev)          netif_carrier_on(dev) //open
#define dm_carrier_init(db)         mii_check_link(&db->mii) //init
#define dm_carrier_poll(db)         mii_check_link(&db->mii) //poll
#define dm_carrier_off(dev)         netif_carrier_off(dev) //no-used
#endif
//.#endif

#if prob_block
//.#if MOD_OBJ_MII == 1
//#if prob_on
static int dm9051_phy_read_lock(struct net_device *dev, int phy_reg_unused, int reg)
{
    unsigned int val = 0xffff;
    struct board_info *db = netdev_priv(dev);
    mutex_lock(&db->spi_lockm); //mutex_lock(&db->spi_lockm);
//  ADDR_LOCK_HEAD_ESSENTIAL(db); //mutex_lock
//  int val = dm_phy_read_func(db, reg);
#if spi_block
    dm9051_phyread(db, reg, &val);
#endif
//  ADDR_LOCK_TAIL_ESSENTIAL(db); //mutex_unlock
    mutex_unlock(&db->spi_lockm); //mutex_unlock(&db->spi_lockm);
    return (int) val;
}

static void dm9051_phy_write_lock(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
    struct board_info *db = netdev_priv(dev);
    mutex_lock(&db->spi_lockm); //mutex_lock(&db->spi_lockm);
//  ADDR_LOCK_HEAD_ESSENTIAL(db); //mutex_lock
//  dm_phy_write_func(db, reg, value);
#if spi_block
    dm9051_phywrite(db, reg, (unsigned int) value);
#endif
//  ADDR_LOCK_TAIL_ESSENTIAL(db); //mutex_unlock
    mutex_unlock(&db->spi_lockm); //mutex_unlock(&db->spi_lockm);
}
//#endif //prob_on
//.#endif
#endif //prob_block

//#if prob_on || 1
//.#if MOD_PHYLIB == 1
#if CODE_SUPPORT_PHYLIB
static int dm9051_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
    struct board_info *db = bus->priv;
    unsigned int val = 0xffff;
    int ret;

    if (addr == DM9051_PHY_ADDR) {
    ret = dm9051_phyread(db, regnum, &val);
    if (ret)
        return ret;
    }

    return val;
}

static int dm9051_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
    struct board_info *db = bus->priv;

    if (addr == DM9051_PHY_ADDR)
    return dm9051_phywrite(db, regnum, val);

    return -ENODEV;
}
#endif
//.#endif

static int dm9051_mdio_register(struct board_info *db)
{
    int ret = 0;
    
    if (phycfg.setup_phy & PHYCFG_LIB) {
	#if CODE_SUPPORT_PHYLIB
	//.#if MOD_PHYLIB == 1
		struct spi_device *spi = db->spidev;

		#if DAVICOM_CONF >= DM9051_KERNEL_5_15 // devm_xx_alloc
		db->mdiobus = devm_mdiobus_alloc(&spi->dev);
		#else //[e.g. DM9051_KERNEL_5_4]
		db->mdiobus = mdiobus_alloc_size(0);
		#endif
		if (!db->mdiobus)
		return -ENOMEM;

		db->mdiobus->priv = db;
		db->mdiobus->read = dm9051_mdio_read;
		db->mdiobus->write = dm9051_mdio_write;
		db->mdiobus->name = "dm9051-mdiobus";
		db->mdiobus->phy_mask = (u32)~BIT(DM9051_PHY_ADDR); //~BIT(1);
		db->mdiobus->parent = &spi->dev;
		snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
		 "dm9051-%s.%u", dev_name(&spi->dev), spi->chip_select);

		#if DAVICOM_CONF >= DM9051_KERNEL_5_15 // devm_xx_register
		ret = devm_mdiobus_register(&spi->dev, db->mdiobus);
		if (ret)
			dev_err(&spi->dev, "Could not register MDIO bus\n");
		#else //[e.g. Liunx 5.4]
		ret = mdiobus_register(db->mdiobus);
		if (ret) {
			dev_err(&spi->dev, "Could not register MDIO bus\n");
			mdiobus_free(db->mdiobus); //tmp devm_mdiobus_free(&spi->dev, db->mdiobus);
		}
		#endif
	//.#endif
	#endif
	}

	if (phycfg.setup_phy & PHYCFG_MII) {
		//.#if MOD_OBJ_MII == 1
		db->mii.dev = db->ndev; //ndev;
		db->mii.phy_id = DM9051_PHY_ADDR;
		db->mii.phy_id_mask = 0x01;
		db->mii.reg_num_mask = 0x1f;
		db->mii.mdio_read = dm9051_phy_read_lock;
		db->mii.mdio_write = dm9051_phy_write_lock;
		//.#endif
	}
    return ret;
}
//#endif //prob_on

/* Now below one is no more effect! kernel_conf.h has constrain it in advance~
 */
#if DAVICOM_CONF <= DM9051_KERNEL_4_19
#if CODE_SUPPORT_PHYLIB
	#ifndef phy_support_sym_pause
	/* This function start from kernel 4.20
	 */
	#define phy_support_sym_pause(phydev)
	//void phy_support_sym_pause(struct phy_device *phydev) {
	//	phydev->supported &= ~ADVERTISED_Asym_Pause;
	//	phydev->supported |= ADVERTISED_Pause;
	//	
	//	phydev->advertising = phydev->supported;
	//}
	#endif
#endif
#endif

#if CODE_SUPPORT_PHYLIB
//.#if MOD_PHYLIB == 1
//.#endif

/*
 * WHen KT_4_1, we define our selves
 */
#ifndef phy_speed_to_str
const char *phy_speed_to_str(int speed) {
	switch (speed) {
		case SPEED_10:
			return "10Mbps";
		case SPEED_100:
			return "100Mbps";
		case SPEED_1000:
			return "1Gbps";
		case SPEED_UNKNOWN:
			return "Unknown";
		default:
			return "Unspported (update phy-core.c)";
	}
}
#endif
#ifndef phy_duplex_to_str
const char *phy_duplex_to_str(unsigned int duplex) {
	if (duplex == DUPLEX_HALF)
		return "Half";
	if (duplex == DUPLEX_FULL)
		return "Full";
	if (duplex == DUPLEX_UNKNOWN)
		return "Unknown";
	return "Unspported (update phy-core.c)";
}
#endif

static void dev_print_status(struct board_info *db)
{
    struct phy_device *phydev = db->phydev;

    if (phydev->link) {
		int lpa = //_mdiobus_read(db->mdiobus, DM9051_PHY_ADDR, MII_LPA);
			  dm9051_mdio_read(db->mdiobus, DM9051_PHY_ADDR, MII_LPA);
			  //_dm9051_mdio_read(db->mdiobus, phydev->mdio.addr, MII_LPA); (while KT4.1, NO 'mdio' member)
		netdev_info(phydev->attached_dev, "Link Up, %s, %s, lpa 0x%04X\n",
			phy_speed_to_str(phydev->speed),
			phy_duplex_to_str(phydev->duplex),
			lpa);
    } else
		netdev_info(phydev->attached_dev, "Link Down\n");
}

static void dev_ethtool_gset(struct board_info *db, /*struct mii_if_info *mii,*/
         struct ethtool_cmd *ecmd)
{
    //struct phy_device *phydev = db->phydev;
    u16 bmcr;

    bmcr = dm9051_mdio_read(db->mdiobus, DM9051_PHY_ADDR, MII_BMCR);
    //bmcr = dm9051_mdio_read(db->mdiobus, phydev->mdio.addr, MII_BMCR);
    //bmcr = dm9620_mdio_read(dev->net, dev->mii.phy_id, MII_BMCR);
    ethtool_cmd_speed_set(ecmd, bmcr & BMCR_SPEED100 ?
    SPEED_100 : SPEED_10);
    ecmd->duplex = (bmcr & BMCR_FULLDPLX) ?
    DUPLEX_FULL : DUPLEX_HALF;
}

static void dm9051_handle_link_change(struct net_device *ndev)
{
    struct board_info *db = to_dm9051_board(ndev);

    dev_print_status(db); //phy_print_status(db->phydev);

    /* only write pause settings to mac. since mac and phy are integrated
     * together, such as link state, speed and duplex are sync already
     */
    if (db->phydev->link) {
    struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
    //mii_check_media(&dev->mii, 1, 1);
    dev_ethtool_gset(db, &ecmd);
    netdev_info(ndev, "link_change() speed: %u duplex: %d\n",
           ethtool_cmd_speed(&ecmd), ecmd.duplex);

    if (db->phydev->pause) {
        db->pause.rx_pause = true;
        db->pause.tx_pause = true;
    }
    dm9051_update_fcr(db);
    }
}
#endif

/* phy connect as poll mode
 */
static int dm9051_hand_connect(struct board_info *db)
{
	#if CODE_SUPPORT_PHYLIB
	if (phycfg.setup_phy == PHYCFG_LIB) {
		//#if MOD_PHYLIB == 1
		//#if CODE_SUPP_PHYLIB
			char phy_id[MII_BUS_ID_SIZE + 3];

			snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
					  db->mdiobus->id, DM9051_PHY_ADDR);

			db->phydev = phy_connect(db->ndev, phy_id, dm9051_handle_link_change,
										PHY_INTERFACE_MODE_MII);
			if (IS_ERR(db->phydev))
				return PTR_ERR_OR_ZERO(db->phydev);
		//#endif
		//#endif
	}
	#endif
    return 0;
}

static int dm9051_phy_connect(struct board_info *db)
{
	int ret;

	ret = dm9051_hand_connect(db);
    if (ret) {
		#if DAVICOM_CONF < DM9051_KERNEL_5_15 //[e.g. DM9051_KERNEL_5_4]
		#if CODE_SUPPORT_PHYLIB
		//.#if MOD_PHYLIB == 1
		if (phycfg.setup_phy & PHYCFG_LIB)
			mdiobus_unregister(db->mdiobus); //20230302 for Linux 5.4
		//.#endif
		#endif
		#endif
		return ret;
	}
	return 0;
}
