#if CODE_RX_REQUEST_SRC_CODE
static int dm9051_dumpblk0(struct board_info *db, u8 reg, size_t count)
{
	/* no skb buffer,
	 * both reg and &rb must be noinc,
	 * read once one byte via regmap_read
	 */
	u8 lagcy_bf[2];
	u8 rb[2];
	int ret;
	
	dm9051_reg_lock_mutex(db);
	
	lagcy_bf[0] = reg;
		
	do {
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = lagcy_bf, .len = 2, 
					.rx_buf = &rb[0], .cs_change = 0, };
		
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		ret = spi_sync(db->spidev, &m);
		/*
		 * _spi_sync, rb[1] has the reading data byte */
		
		if (ret < 0) {
			dbg_log("spi-mem1 lagecy cmd %02x\n", reg);
			dbg_log("spi-mem1 read LEN %d (db->fer->len %d)\n", 1, 1 + 1);
			dbg_log("spi-sync communication fail! ret= %d\n", ret);
		}
		
		/*rb = std_spi_read_reg(db, reg);
		 */
	} while (--count);
	
	dm9051_reg_unlock_mutex(db);
	return 0;
}
#endif

//u8 _lagcy_buf[1536+ 4];
static int dm9051_write_mem0(struct board_info *db, unsigned int reg, const void *buff,
			    size_t len)
{
	int ret = 0;
	dm9051_reg_lock_mutex(db);
	
	//#if MOD_REGMAP0ExT == 1
	//=[std_write_tx_buf(), already as below:]
	if (xfer == RAW_SPISYN) {
	#if 0 // (not succeed coding), 1
		u8 *p = buff;
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = buff, .len = len, 
								  .rx_buf = NULL, .cs_change = 0, };
		p[0] = DM_SPI_WR | (u8) reg;
	#else	
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = db->lagcy_buf, .len = len + 1, 
								  .rx_buf = NULL, .cs_change = 0, };
		db->lagcy_buf[0] = DM_SPI_WR | (u8) reg;
		memcpy(&db->lagcy_buf[1], buff, len);
	#endif
			
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		ret = spi_sync(db->spidev, &m);
	} else if (xfer == RAW_SPISYNB) {
	//#endif
	//#if MOD_REGMAP0ExB == 1
		u8 lagcy_bf[2];
		struct spi_message m;
		struct spi_transfer t[2] = { { .tx_buf = lagcy_bf, .len = 1, },
						 { .tx_buf = buff, .len = len, }, };
		
		lagcy_bf[0] = DM_SPI_WR | (u8) reg;
		
		spi_message_init(&m);
		spi_message_add_tail(&t[0], &m);
		spi_message_add_tail(&t[1], &m);
		ret = spi_sync(db->spidev, &m);
	}
	//#endif
	
	if (ret < 0) {
		dbg_log("spi-mem0_wr lagecy cmd %02x\n", (u8) (reg & GENMASK(7, 0)));
		dbg_log("spi-sync communication fail! ret= %d\n", ret);
	}
	
	dm9051_reg_unlock_mutex(db);
	return ret;
}

#if CODE_RX_REQUEST_SRC_CODE
static int _tregmap_raw_read(struct board_info *db, unsigned int reg, void *buff, unsigned int chnklen, bool noinc) 
{
	//(struct spi_device *spi)
	//return spi_write_then_read(spi, &reg, 1, buff, chnklen);
	//(struct board_info *db)
	return spi_write_then_read(db->spidev, &reg, 1, buff, chnklen); //as spi_sync() in spi.c
	//[Basically,OK]
	//else OR use std_read_rx_buf_ncpy(db, &reg, buff, chnklen);
}

static int dm9051_read_mem0(struct board_info *db, unsigned int reg, void *buff,
			   size_t len)
{
	//KT5.15.76
	//=[std_read_rx_buf_ncpy()]	
	
	/*
	 * How to for every support KTs ?
	 */
	int ret = 0;

	dm9051_reg_lock_mutex(db);

	//#if MOD_REGMAP0ExT == 1
	if (xfer == RAW_SPISYN)
		 ret = _tregmap_raw_read(db, reg, buff, len, false); // JJ-adopt
	//#endif
	//#if MOD_REGMAP0ExB == 1
	else  if (xfer == RAW_SPISYNB)
		ret = spi_write_then_read(db->spidev, &reg, 1, buff, len); //as spi_sync() in spi.c
	//#endif

	 if (ret)
		 dev_err(&db->spidev->dev, "%s: error %d read mem %02x\n",
				  __func__, ret, reg);

	dm9051_reg_unlock_mutex(db);
	return ret;
}
#endif

static int dm9051_map_init0(struct spi_device *spi, struct board_info *db)
{
	/* create two regmap instances,
	 * split read/write and bulk_read/bulk_write to individual regmap
	 * to resolve regmap execution confliction problem
	 */
	//[ * DM_VER_BURSTMODE    -- To get faster in _spi_sync, only kernel less than 5.15 or more earlier can be suitable ]
	//[ *  (DM_VER_BURSTMODE) Could get _spi_sync error, such as KT5.15 (while test more and more times, sometimes.) ]

	//.#if MOD_REGMAP0ExT == 1
	//memset(&db->Tfer, 0, sizeof(struct spi_transfer));  //[Add.]
	//spi_message_init(&db->Tmsg);
	//spi_message_add_tail(&db->Tfer, &db->Tmsg);
	//.#endif
	return 0;
}

static int dm9051_set_reg0(struct board_info *db, unsigned int reg, unsigned int val)
{
	std_spi_write_reg(db, reg, val);
	return 0;
}

static int dm9051_update_bits0(struct board_info *db, unsigned int reg, unsigned int mask,
			      unsigned int val)
{
	u8 rb, tmp;
	
	rb = std_spi_read_reg(db, reg);
	
	tmp = rb & ~mask;
	tmp |= val & mask;
	
	if (tmp != rb)
		std_spi_write_reg(db, reg, tmp);
	return 0;
}

static int dm9051_set_regs0(struct board_info *db, unsigned int reg, const void *val,
			   size_t val_count)
{
	int i;
	u8 *p = (u8 *) val;
	
	for (i = 0; i < val_count; i++)
		std_spi_write_reg(db, reg+i, *p++);
	return 0;
}

static int dm9051_get_regs0(struct board_info *db, unsigned int reg, void *val,
			   size_t val_count)
{
	int i;
	u8 *p = val;
	
	for (i = 0; i < val_count; i++)
		*p++ = std_spi_read_reg(db, reg+i);
	return 0;
}

/* waiting tx-end rather than tx-req
 * got faster
 */
static int dm9051_nsr_poll0(struct board_info *db)
{
	int i;
	unsigned int rb;
	
	for (i = 0; i < 1; i++) {
		rb = std_spi_read_reg(db, DM9051_TCR);
		if (!(rb & TCR_TXREQ))
			break;
		udelay(1);
	}
	return 0;
}

static int dm9051_epcr_poll0(struct board_info *db)
{
	int i;
	unsigned int rb;
	
	for (i = 0; i < 100; i++) {
		rb = std_spi_read_reg(db, DM9051_EPCR);
		if (!(rb & EPCR_ERRE))
			break;
		udelay(100);
	}
	return 0;
}

/* skb buffer exhausted, just discard the received data
 */
static int _dm9051_get_reg0(struct board_info *db, unsigned int reg, void *val) //CALLED in  _dm9051_map_chipid
{
	u8 *p = val;
	*p = std_spi_read_reg(db, reg);
	return 0;
}

static int dm9051_core_reset0(struct board_info *db)
{
	db->bc.fifo_rst_counter++;
	std_spi_write_reg(db, DM9051_NCR, NCR_RST);
	std_spi_write_reg(db, DM9051_MBNDRY, MBNDRY_BYTE & 0); /* MemBound, 'wb' */
	std_spi_write_reg(db, DM9051_PPCR, PPCR_PAUSE_COUNT);
	std_spi_write_reg(db, DM9051_LMCR, db->lcr_all);

	return dm9051_set_reg0(db, DM9051_INTCR, /*dm9051_intcr_value(db)*/ db->intcr_all);
}

static int dm9051_eeprom_read0(struct board_info *db, int offset, u8 *to)
{
	int ret;
	
	std_spi_write_reg(db, DM9051_EPAR, offset);
	std_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR);

	ret = dm9051_epcr_poll0(db);
	if (ret)
		return ret;

	std_spi_write_reg(db, DM9051_EPCR, 0);
	dm9051_get_regs0(db, DM9051_EPDRL, to, 2);
	return 0;
}

static int dm9051_eeprom_write0(struct board_info *db, int offset, u8 *data)
{
	int ret;

	std_spi_write_reg(db, DM9051_EPAR, offset);
	dm9051_set_regs0(db, DM9051_EPDRL, data, 2);	
	std_spi_write_reg(db, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);

	ret = dm9051_epcr_poll0(db);
	if (ret)
		return ret;

	std_spi_write_reg(db, DM9051_EPCR, 0);
	return 0;
}

static int dm9051_phyread0(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;
	
	std_spi_write_reg(db, DM9051_EPAR, DM9051_PHY | reg);
	std_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);

	ret = dm9051_epcr_poll0(db);
	if (ret)
		return ret;

	std_spi_write_reg(db, DM9051_EPCR, 0);
	
	/* this is a 4 bytes data, clear to zero since following regmap_bulk_read
	 * only fill lower 2 bytes
	 */
	*val = 0;
	dm9051_get_regs0(db, DM9051_EPDRL, val, 2);
	return 0;
}

static int dm9051_phywrite0(void *context, unsigned int reg, unsigned int val)
{
	struct board_info *db = context;
	int ret;

	std_spi_write_reg(db, DM9051_EPAR, DM9051_PHY | reg);
	dm9051_set_regs0(db, DM9051_EPDRL, &val, 2);
	std_spi_write_reg(db, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);

	ret = dm9051_epcr_poll0(db);
	if (ret)
		return ret;

	std_spi_write_reg(db, DM9051_EPCR, 0);
	return 0;
}
