static int dm9051_enable_interrupt(struct board_info *db)
{
    return dm9051_set_reg(db, DM9051_IMR, db->imr_all); /* enable int */
}

static int dm9051_set_recv(struct board_info *db)
{
    int ret;

    ret = dm9051_set_regs(db, DM9051_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table));
    if (ret)
    return ret;

    return dm9051_set_reg(db, DM9051_RCR, db->rctl.rcr_all); /* enable rx */
}

static int dm9051_all_start(struct board_info *db)
{
	int ret;

	/* GPR power on of the internal phy
	 */
	ret = dm9051_set_reg(db, DM9051_GPR, 0);
	if (ret)
		return ret;

	/* dm9051 chip registers could not be accessed within 1 ms
	 * after GPR power on, delay 1 ms is essential
	 */
	msleep(1);

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;

	return dm9051_enable_interrupt(db);
}

static int dm9051_all_stop(struct board_info *db)
{
	int ret;

	/* GPR power off of the internal phy,
	 * The internal phy still could be accessed after this GPR power off control
	 */
	ret = dm9051_set_reg(db, DM9051_GPR, GPR_PHY_OFF);
	if (ret)
		return ret;

	return dm9051_set_reg(db, DM9051_RCR, RCR_RX_DISABLE);
}

#if CODE_RX_REQUEST_SRC_CODE

static int dm9051_set_fcr(struct board_info *db)
{
    u8 fcr = 0;

    if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
    if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;
		
    //[cast to flow control  enable]
	//.#if MOD_REGMAP == 0
    //if  ((fcr & FCR_RXTX_BITS) != FCR_RXTX_BITS) {
	//	printk("dm9.set_fcr. 0x%02x cast to 0x%02x\n", (u8) fcr, (u8) (fcr | FCR_RXTX_BITS));
	//	fcr |= FCR_RXTX_BITS;
    //}
	//.#endif

    return dm9051_set_reg(db, DM9051_FCR, fcr);
}

static int dm9051_disable_interrupt(struct board_info *db)
{
    return dm9051_set_reg(db, DM9051_IMR, IMR_PAR); /* disable int */
}

static int dm9051_stop_mrcmd(struct board_info *db)
{
    return dm9051_set_reg(db, DM9051_ISR, ISR_STOP_MRCMD); /* to stop mrcmd */
}

static int dm9051_clear_interrupt(struct board_info *db)
{
    return dm9051_update_bits(db, DM9051_ISR, ISR_CLR_INT, ISR_CLR_INT);
}

/* fifo reset while rx error found
 */
static int dm9051_all_restart(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret;

	ret = dm9051_core_reset(db);
	if (ret)
		return ret;

	ret = dm9051_enable_interrupt(db);
	if (ret)
		return ret;

	netdev_dbg(ndev, " rxstatus_Er & rxlen_Er %d, RST_c %d\n",
		   db->bc.status_err_counter + db->bc.large_err_counter,
		   db->bc.fifo_rst_counter);

	ret = dm9051_set_recv(db);
	if (ret)
		return ret;

	return dm9051_set_fcr(db);
}
#endif
