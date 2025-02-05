/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > DM9051_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	//.#if has_ndev_ops
	schedule_work(&db->tx_work);
	//.#endif

#if 0
	//ptp
	skb_tx_timestamp(skb);
#endif
	return NETDEV_TX_OK;
}

/* event: play with a schedule starter
 */
static void dm9051_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct dm9051_rxctrl rxctrl;
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;

	memset(&rxctrl, 0, sizeof(rxctrl));

	/* rx control */
	if (ndev->flags & IFF_PROMISC) {
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	rxctrl.rcr_all = rcr;

	/* broadcast address */
	rxctrl.hash_table[0] = 0;
	rxctrl.hash_table[1] = 0;
	rxctrl.hash_table[2] = 0;
	rxctrl.hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = ether_crc_le(ETH_ALEN, ha->addr) & GENMASK(5, 0);
		rxctrl.hash_table[hash_val / 16] |= BIT(0) << (hash_val % 16);
	}

	/* schedule work to do the actual set of the data if needed */

	if (memcmp(&db->rctl, &rxctrl, sizeof(rxctrl))) {
		memcpy(&db->rctl, &rxctrl, sizeof(rxctrl));
		//.#if has_ndev_ops
		schedule_work(&db->rxctrl_work);
		//.#endif
	}
}

//[phytool], [mii-tool] support
#if DAVICOM_CONF >= DM9051_KERNEL_5_4 // net_xx_ops, _eth_ioctl ( >= kt54), func
static int dm9051_netdev_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	//struct usbnet *dev = netdev_priv(net);
	//return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL); // struct mii_if_info *mii;
	struct board_info *db = to_dm9051_board(ndev);
	/*struct phy_device *phydev = db->phydev;*/
	struct mii_ioctl_data *data = if_mii(rq);

//.#if MOD_PHYLIB == 1 //[JJ: 20230526]
#if 1 //[JJ: programming on 20230221]
	//[int result = 0;]
	switch(cmd) {
		case SIOCGMIIPHY:
			data->phy_id = DM9051_PHY_ADDR;
			fallthrough;
			
		case SIOCGMIIREG:
			//read();
			//.#if MOD_PHYLIB == 1	
			if (phycfg.setup_phy & PHYCFG_LIB)
				data->val_out = dm9051_mdio_read(db->mdiobus, /*phydev->mdio.addr*/ DM9051_PHY_ADDR, data->reg_num & 0x1f);
			//.#else
			else 
				do {
					unsigned int val;
					dm9051_phyread(db, data->reg_num & 0x1f, &val);
					data->val_out = val;
				} while(0);
			//.#endif
			break;
		case SIOCSMIIREG:
			//write();
			//#if MOD_PHYLIB == 1	
			if (phycfg.setup_phy & PHYCFG_LIB)
				dm9051_mdio_write(db->mdiobus, /*phydev->mdio.addr*/ DM9051_PHY_ADDR, data->reg_num & 0x1f, data->val_in);
			else
			//#else
				dm9051_phywrite(db, data->reg_num & 0x1f, data->val_in);
			//#endif
			break;
		default:
			return -EOPNOTSUPP;
	}
#endif
//.#endif
	return 0;
	//[return result;]
}
#endif

/* event: write into the mac registers and eeprom directly
 */
static int dm9051_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_dm9051_board(ndev);
	int ret;

	ret = eth_prepare_mac_addr_change(ndev, p);
	if (ret < 0)
		return ret;

	eth_commit_mac_addr_change(ndev, p);
	return dm9051_set_regs(db, DM9051_PAR, ndev->dev_addr, sizeof(ndev->dev_addr));
}

#ifdef ETHn_NDEVOPS_FIXED
static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_rx_mode,
#if DAVICOM_CONF >= DM9051_KERNEL_5_4 // net_xx_ops, _eth_ioctl ( >= kt54)
	.ndo_do_ioctl = dm9051_netdev_ioctl,  //support [phytool], [mii-tool] when >= KT54
	#if DAVICOM_CONF >= DM9051_KERNEL_5_15 // net_xx_ops, _eth_ioctl ( >= kt515)
		.ndo_eth_ioctl = dm9051_netdev_ioctl, //support [phytool], [mii-tool], when >= KT515
	#endif
#endif
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
};
#endif //ETHn_NDEVOPS_FIXED

static int dm9051_ndev_register(struct net_device *ndev, struct board_info *db)
{
//.#if prob_on == 1
		int ret;

	#ifdef ETHn_NDEVOPS_FIXED
		ndev->netdev_ops = &dm9051_netdev_ops;
		ndev->ethtool_ops = &dm9051_ethtool_ops;

		#if DAVICOM_CONF < DM9051_KERNEL_5_15 //[e.g. DM9051_KERNEL_5_4]
		ret = register_netdev(ndev);
		#else
		ret = devm_register_netdev(&db->spidev->dev, ndev);
		#endif
	
		if (ret) {
			#if CODE_SUPPORT_PHYLIB
			if (phycfg.setup_phy & PHYCFG_LIB)
				phy_disconnect(db->phydev);
			#endif
			#if DAVICOM_CONF >= DM9051_KERNEL_5_15 // devm no unreg
				ret = dev_err_probe(&db->spidev->dev, ret, "device register failed");
				return ret; //goto out; //return ret;
			#else //[e.g. DM9051_KERNEL_5_4]
				//[verified range 5.4]
				#if CODE_SUPPORT_PHYLIB
				//.#if MOD_PHYLIB == 1
				if (phycfg.setup_phy & PHYCFG_LIB)
					mdiobus_unregister(db->mdiobus); //20230302 for Linux 5.4
				//.#endif
				#endif
				dev_err(&db->spidev->dev, "error %d: device register failed\n", ret);
				return ret; //goto out; //return ret;
			#endif
		}
	#endif //ETHn_NDEVOPS_FIXED
	
		return 0;
//.#endif //prob_on
//.#if prob_on == 0
//  printk("dm9 probe, return 0\n");
//	return 0;
//.#endif //prob_on
}

#if DAVICOM_CONF < DM9051_KERNEL_5_15
static void dm9051_ndev_unregister(struct net_device *ndev)
{
	#ifdef ETHn_NDEVOPS_FIXED
	#if DAVICOM_CONF < DM9051_KERNEL_5_15 // devm no unregi (obsect)
		unregister_netdev(ndev);
	#endif
	#endif //ETHn_NDEVOPS_FIXED
}
#endif
