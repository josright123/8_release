//.
//.static int _dm9051_get_reg1(struct board_info *db, unsigned int reg, void *val);
//.
#if CODE_SUPPORT_REGMAP

#if CODE_RX_REQUEST_SRC_CODE
static int dm9051_dumpblk1(struct board_info *db, u8 reg, size_t count)
{
	/* no skb buffer,
	 * both reg and &rb must be noinc,
	 * read once one byte via regmap_read
	 */
	 
	unsigned int rb;
	int ret;
	do {
		ret = _dm9051_get_reg1(db, reg, &rb);
		if (ret < 0)
			break;
	} while (--count);
	return ret;
}
#endif

static int dm9051_write_mem1(struct board_info *db, unsigned int reg, const void *buff,
			    size_t len)
{
	int ret;
	ret = regmap_raw_write(db->regmap_dm, reg, buff, len); //__regmap_noinc_write
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc writing regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

#if CODE_RX_REQUEST_SRC_CODE
static int dm9051_read_mem1(struct board_info *db, unsigned int reg, void *buff,
			   size_t len)
{
	int ret;

	ret = regmap_raw_read(db->regmap_dm, reg, buff, len); //__regmap_noinc_read
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d noinc reading regs %02x\n",
			  __func__, ret, reg);
	return ret;
}
#endif

static int dm9051_map_init1(struct spi_device *spi, struct board_info *db)
{
	#if DAVICOM_CONF >= DM9051_KERNEL_4_19
	//
	// Use devm_regmap_init() as below < DM9051_KERNEL_4_19 else case,
	// It could also work OK too. Only need take your decision yourselves.
	//
	
	/* create two regmap instances,
	 * split read/write and bulk_read/bulk_write to individual regmap
	 * to resolve regmap execution confliction problem
	 */
	regconfigdm.lock_arg = db;
	//db->regmap_dm = devm_regmap_init_spi(spi, &regconfigdm);
	db->regmap_dm = devm_regmap_init_spi(db->spidev, &regconfigdm);
	if (IS_ERR(db->regmap_dm))
		return PTR_ERR(db->regmap_dm);

	regconfigdmbulk.lock_arg = db;
	//db->regmap_dmbulk = devm_regmap_init_spi(spi, &regconfigdmbulk);
	db->regmap_dmbulk = devm_regmap_init_spi(db->spidev, &regconfigdmbulk);
	if (IS_ERR(db->regmap_dmbulk))
		return PTR_ERR(db->regmap_dmbulk);
	#else
        /* [test]
         * KT_4_1 not has regmap can use for spi. 
         * KT 4.9 not know during this announce time. (DM9051_KERNEL_4_9)
         */
		regconfigdm.lock_arg = db;
		//db->regmap_dm = devm_regmap_init_spi(spi, &regconfigdm);
		db->regmap_dm = devm_regmap_init(&spi->dev, &regmap_bus_dm, &spi->dev, &regconfigdm);
		if (IS_ERR(db->regmap_dm))
			return PTR_ERR(db->regmap_dm);

		regconfigdmbulk.lock_arg = db;
		//db->regmap_dmbulk = devm_regmap_init_spi(spi, &regconfigdmbulk);
		db->regmap_dmbulk = devm_regmap_init(&spi->dev, &regmap_bus_dm, &spi->dev, &regconfigdmbulk);
		if (IS_ERR(db->regmap_dmbulk))
			return PTR_ERR(db->regmap_dmbulk);
	#endif

	return 0;
}

static int dm9051_set_reg1(struct board_info *db, unsigned int reg, unsigned int val)
{
	int ret;

	ret = regmap_write(db->regmap_dm, reg, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d set reg %02x\n",
			  __func__, ret, reg);
	return ret;
}

static int dm9051_update_bits1(struct board_info *db, unsigned int reg, unsigned int mask,
			      unsigned int val)
{
	int ret;

	ret = regmap_update_bits(db->regmap_dm, reg, mask, val);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d update bits reg %02x\n",
			  __func__, ret, reg);
	return ret;
}

static int dm9051_set_regs1(struct board_info *db, unsigned int reg, const void *val,
			   size_t val_count)
{
	int ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk writing regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

static int dm9051_get_regs1(struct board_info *db, unsigned int reg, void *val,
			   size_t val_count)
{
	int ret;

	ret = regmap_bulk_read(db->regmap_dmbulk, reg, val, val_count);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d bulk reading regs %02x\n",
			  __func__, ret, reg);
	return ret;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
static int dm9051_nsr_poll1(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_TCR, mval,
					!(mval & TCR_TXREQ), 1, 1);
					
	#if 0
	/*
	#if 1
		ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_TCR, mval,
						!(mval & TCR_TXREQ), 1, 20);
	#else
		ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_NSR, mval,
						   mval & (NSR_TX2END | NSR_TX1END), 1, 20);
	#endif

		if (ret == -ETIMEDOUT) {
			printk("timeout in checking for tx end\n");
		}*/
	#endif

	return ret;
}

static int dm9051_epcr_poll1(struct board_info *db)
{
	unsigned int mval;
	int ret;

	ret = regmap_read_poll_timeout(db->regmap_dm, DM9051_EPCR, mval,
				       !(mval & EPCR_ERRE), 100, 10000);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");
	return ret;
}

/* skb buffer exhausted, just discard the received data
 */
static int _dm9051_get_reg1(struct board_info *db, unsigned int reg, void *val) //CALLED in  dm9051_map_chipid
{
	int ret;

	ret = regmap_read(db->regmap_dm, reg, val); //&rb
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d regmap read reg %02x\n",
			  __func__, ret, reg);
	return ret;
}

static int dm9051_core_reset1(struct board_info *db)
{
	int ret;
	db->bc.fifo_rst_counter++;
	ret = regmap_write(db->regmap_dm, DM9051_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_MBNDRY, MBNDRY_BYTE & 0); /* MemBound, 'wb' */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_PPCR, PPCR_PAUSE_COUNT); /* Pause Count */
	if (ret)
		return ret;
	ret = regmap_write(db->regmap_dm, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	return dm9051_set_reg1(db, DM9051_INTCR, /*dm9051_intcr_value(db)*/ db->intcr_all);
}

static int dm9051_eeprom_read1(struct board_info *db, int offset, u8 *to)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll1(db);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, to, 2);
}

static int dm9051_eeprom_write1(struct board_info *db, int offset, u8 *data)
{
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, offset);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, data, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll1(db);
	if (ret)
		return ret;

	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

static int dm9051_phyread1(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll1(db);
	if (ret)
		return ret;
		
	ret = regmap_write(db->regmap_dm, DM9051_EPCR, 0);
	if (ret)
		return ret;

	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
	 * only fill lower 2 bytes
	 */
	*val = 0;
	return regmap_bulk_read(db->regmap_dmbulk, DM9051_EPDRL, val, 2);
}

static int dm9051_phywrite1(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPAR, DM9051_PHY | reg);
	if (ret)
		return ret;

	ret = regmap_bulk_write(db->regmap_dmbulk, DM9051_EPDRL, &val, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(db->regmap_dm, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = dm9051_epcr_poll1(db);
	if (ret)
		return ret;
		
	return regmap_write(db->regmap_dm, DM9051_EPCR, 0);
}

#endif //CODE_SUPPORT_REGMAP
