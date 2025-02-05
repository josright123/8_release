//.#if CODE_SUPP_MII == 0
#if BOARD_CONF < DM9051_KERNEL_5_15 && DAVICOM_CONF < DM9051_KERNEL_4_19
static void mdio_ethtool_gset(void)
{
	printk("No mii.ko, todo use 'phy_mdio-ethtool_gset()'\n");
}
static int mdio_ethtool_sset(void)
{
	printk("No mii.ko, todo use 'phy_mdio-ethtool_sset()'\n");
	return 0;
}
#endif
static int mdio_nway_restart(void)
{
	printk("No mii.ko, todo use 'phy_mdio-nway_restart()'\n");
	return 0;
}

static int mdio_link_ok(struct net_device *ndev)
{
	int val = dm9051_phy_read_lock(ndev, /*(int) NULL*/ 0, MII_BMSR);
	return val & BIT(2) ? 1 : 0;
}
static void mdio_check_link(struct board_info *db)
{
	int val = dm9051_phy_read_lock(db->ndev, /*(int) NULL*/ 0, MII_BMSR);
	int cur_link = val & 0x4 ? 1 : 0;
	int prev_link = netif_carrier_ok(db->ndev);
	
	if (cur_link && !prev_link)
		netif_carrier_on(db->ndev);
	else if (prev_link && !cur_link)
		netif_carrier_off(db->ndev);
}
//#endif
