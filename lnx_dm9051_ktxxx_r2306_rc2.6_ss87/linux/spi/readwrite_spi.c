// KT515,		when 'on_regmap': regmap OK, _spi_write_then_read OK
// kt515		when 'on_spisyn/on_spisynb': regmap NG, _spi_sync OK
#if 0
//To include "(regmap_1)kt515"
//To include "(regmap_0)kt515_low"
#endif

static void dm9051_reg_lock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_lock(&db->reg_mutex);
}

static void dm9051_reg_unlock_mutex(void *dbcontext)
{
	struct board_info *db = dbcontext;

	mutex_unlock(&db->reg_mutex);
}

	#define dbg_log(format, args...)					\
		do{												\
			printk("dm9051 :" " %s _%d_: " format,	\
					__FILE__, __LINE__, ## args);	\
		}while(0)

static void std_spi_write_reg(struct board_info *db, unsigned int reg, unsigned int val)
{
	u8 lagcy_bf[2];
	int ret;

	//#if MOD_REGMAP0ExT == 1
	if (xfer == RAW_SPISYN) {
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = lagcy_bf, .len = 2, 
					.rx_buf = NULL, .cs_change = 0, }; // { ... }

		lagcy_bf[0] = DM_SPI_WR | reg;
		lagcy_bf[1] = val;
		
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		dm9051_reg_lock_mutex(db);
		ret = spi_sync(db->spidev, &m); //spi_sync(db->spidev, &db->Tmsg); //ret = spi_sync(db -> spidev, db -> msg);
		dm9051_reg_unlock_mutex(db);
		if (ret < 0) {
			dbg_log("spi-spi-1 ExT cmd %02x\n", reg);
			dbg_log("spi-spi-1 write LEN %d (db->fer->len %d)\n", 1, 1 + 1);
			dbg_log("spi-sync communication fail! ret= %d\n", ret);
		}
	//#endif
	//#if MOD_REGMAP0ExB == 1
	} else  if (xfer == RAW_SPISYNB) {
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = lagcy_bf, .len = 2, 
					.rx_buf = NULL, .cs_change = 0, };

		lagcy_bf[0] = (u8) (DM_SPI_WR | reg);
		lagcy_bf[1] = (u8) val;
			
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		dm9051_reg_lock_mutex(db);
		ret = spi_sync(db->spidev, &m);
		dm9051_reg_unlock_mutex(db);
		/*
		 * _spi_sync, rb[1] has the reading data byte */
		if (ret < 0) {
			dbg_log("spi-mem1 ExB cmd %02x\n", reg);
			dbg_log("spi-mem1 write LEN %d (db->fer->len %d)\n", 1, 1 + 1);
			dbg_log("spi-sync communication fail! ret= %d\n", ret);
		}
	//#endif
	}
}
				
static u8 std_spi_read_reg(struct board_info *db, u8 reg)
{
	//#if MOD_REGMAP0ExT == 1
	if (xfer == RAW_SPISYN) {
		u8 txb[2];
		u8 lagcy_bf[2];
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = txb, .len = 2, 
					.rx_buf = lagcy_bf, .cs_change = 0, }; // { ... }
		int ret;
		
		#define DM_SPI_RD		(0x00)
		
		txb[0] = DM_SPI_RD | reg;
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		dm9051_reg_lock_mutex(db); //ADDR_LOCK_HEAD_SPI(db);
		ret = spi_sync(db->spidev, &m); //ret = spi_sync(db->spidev, &db->Tmsg); //ret = spi_sync(db -> spidev, db -> msg);
		dm9051_reg_unlock_mutex(db); //ADDR_LOCK_TAIL_SPI(db);
		if (ret < 0) {
			dbg_log("spi-1 ExT cmd %02x\n", txb[0]);
			dbg_log("spi-1 rxbuf %02x %02x\n", lagcy_bf[0], lagcy_bf[1]);
			dbg_log("spi-1 LEN %d (db->fer->len %d)\n", 1, 1 + 1);
			dbg_log("spi-1 communication fail! ret=%d\n", ret);
		}
		return lagcy_bf[1];
	//#endif
	//#if MOD_REGMAP0ExB == 1
	} else if (xfer == RAW_SPISYNB) {
		u8 lagcy_bf[2];
		u8 rb[2];
		struct spi_message m;
		struct spi_transfer t = { .tx_buf = lagcy_bf, .len = 2, 
					.rx_buf = rb, .cs_change = 0, };
		int ret;

		lagcy_bf[0] = reg;		
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		dm9051_reg_lock_mutex(db);
		ret = spi_sync(db->spidev, &m);
		dm9051_reg_unlock_mutex(db);
		/*
		 * _spi_sync, rb[1] has the reading data byte */	
		if (ret < 0) {
				dbg_log("spi-mem1 ExB cmd %02x\n", reg);
				dbg_log("spi-mem1 read LEN %d (db->fer->len %d)\n", 1, 2);
				dbg_log("spi-sync communication fail! ret= %d\n", ret);
		}

		return rb[1];
	//#endif
	}
	return 0xff;
}

//.#if MOD_REGMAP == 1
//.#endif
#if CODE_SUPPORT_REGMAP
static struct regmap_config regconfigdm = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
};

static struct regmap_config regconfigdmbulk = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_stride = 1,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0,
	.write_flag_mask = DM_SPI_WR,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.lock = dm9051_reg_lock_mutex,
	.unlock = dm9051_reg_unlock_mutex,
	#if DAVICOM_CONF >= DM9051_KERNEL_4_19
	.use_single_read = true,
	.use_single_write = true,
	#endif
};

/* when if BOARD_CONF < DM9051_KERNEL_5_15 && DAVICOM_CONF < DM9051_KERNEL_4_19,
 * We write thus macro internal ourselves.
 */
#ifndef regmap_read_poll_timeout
	#define regmap_read_poll_timeout(map, addr, val, cond, sleep_us, time_us) \
	({ \
		int i; \
		int __ret; \
		for (i = 0; i < time_us; i += sleep_us) { \
			__ret = regmap_read((map), (addr), &(val)); \
			if (__ret) \
				break; \
			if (cond) \
				break; \
			udelay(sleep_us); \
		} \
		__ret ?: ((cond) ? 0 : -ETIMEDOUT); \
	})
#endif
#endif

//.#if MOD_REGMAP == 0
//.#elif MOD_REGMAP == 1
//.#endif
#include "regmap_0.c"	
#include "regmap_1.c"

//static int dm9051_set_reg(struct board_info *db, unsigned int reg, unsigned int val)
//{
//	if (xfer == RAW_REGMAP)
//		return dm9051_set_reg1(db, reg, val);
//	return dm9051_set_reg0(db, reg, val);
//}
static int dm9051_update_bits(struct board_info *db, unsigned int reg, unsigned int mask,
			      unsigned int val)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_update_bits1(db, reg, mask, val);
#endif
	return dm9051_update_bits0(db, reg, mask, val);
}
static int dm9051_set_regs(struct board_info *db, unsigned int reg, const void *val,
			   size_t val_count)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_set_regs1(db, reg, val, val_count);
#endif
	return dm9051_set_regs0(db, reg, val, val_count);
}
static int dm9051_get_regs(struct board_info *db, unsigned int reg, void *val,
			   size_t val_count)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_get_regs1(db, reg, val, val_count);
#endif
	return dm9051_get_regs0(db, reg, val, val_count);
}

static int dm9051_nsr_poll(struct board_info *db)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_nsr_poll1(db);
#endif
	return dm9051_nsr_poll0(db);	
}
//static int dm9051_epcr_poll(struct board_info *db)
//{
//	return dm9051_epcr_poll0(db);
//	return dm9051_epcr_poll1(db);
//}

//static int _dm9051_get_reg(struct board_info *db, unsigned int reg, void *val) //CALLED in  dm9051_map_chipid
//{
//	if (xfer == RAW_REGMAP)
//		return _dm9051_get_reg1(db, reg, val);
//	return _dm9051_get_reg0(db, reg, val);
//}
static int dm9051_core_reset(struct board_info *db)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_core_reset1(db);
#endif
	return dm9051_core_reset0(db);
}

static int dm9051_eeprom_read(struct board_info *db, int offset, u8 *to)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_eeprom_read1(db, offset, to);
#endif
	return dm9051_eeprom_read0(db, offset, to);
}
static int dm9051_eeprom_write(struct board_info *db, int offset, u8 *data)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_eeprom_write1(db, offset, data);
#endif
	return dm9051_eeprom_write0(db, offset, data);
}
static int dm9051_phyread(void *context, unsigned int reg, unsigned int *val)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_phyread1(context, reg, val);
#endif
	return dm9051_phyread0(context, reg, val);
}
static int dm9051_phywrite(void *context, unsigned int reg, unsigned int val)
{
#if CODE_SUPPORT_REGMAP
	if (xfer == RAW_REGMAP)
		return dm9051_phywrite1(context, reg, val);
#endif
	return dm9051_phywrite0(context, reg, val);
}
