//[hc.]
#define V510_COMPLEX	1
#define V510_COMPLEX2	1

#define COMPLEX2_ARM32_64	0

static int regmap_spi_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write(spi, data, count);
}
static int regmap_spi_gather_write(void *context,
				   const void *reg, size_t reg_len,
				   const void *val, size_t val_len)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[2] = { { .tx_buf = reg, .len = reg_len, },
				     { .tx_buf = val, .len = val_len, }, };

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	return spi_sync(spi, &m);
}
static int regmap_spi_read(void *context,
			   const void *reg, size_t reg_size,
			   void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write_then_read(spi, reg, reg_size, val, val_size);
}

static int regmap_spi_async_write(void *context, //.V510_COMPLEX
				  const void *reg, size_t reg_len,
				  const void *val, size_t val_len,
				  struct regmap_async *a)
{
	printk("NOT SUPPORT: regmap_spi_async_write(context,...)\n");
	return -1;
}
static struct regmap_async *regmap_spi_async_alloc(void) //.V510_COMPLEX
{
	printk("NOT SUPPORT: regmap_spi_async_alloc(void)\n");
	return NULL;
}

static const struct regmap_bus regmap_bus_dm = {
	.write = regmap_spi_write,
	.gather_write = regmap_spi_gather_write,
	.async_write = regmap_spi_async_write, //.V510_COMPLEX
	.async_alloc = regmap_spi_async_alloc, //.V510_COMPLEX
	.read = regmap_spi_read,
	.read_flag_mask = 0x80,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

//can in [dm9051.c]!
#if 0
static const struct regmap_bus *regmap_get_spi_bus(struct spi_device *spi,
						   const struct regmap_config *config)
{
#if V510_COMPLEX2
	size_t max_size = spi_max_transfer_size(spi);
	struct regmap_bus *bus;

	#if COMPLEX2_ARM32_64
	printk("s.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %d, SIZE_MAX %d (equ)\n", max_size, SIZE_MAX); //32-bit
	//printk("s.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %ld, SIZE_MAX %ld (equ)\n", max_size, SIZE_MAX); //64-bit
	#endif
	
	if (max_size != SIZE_MAX) { //.V510_COMPLEX2
			
		#if COMPLEX2_ARM32_64
		printk("m.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %d, SIZE_MAX %d (neq)\n", max_size, SIZE_MAX); //32-bit
		//printk("m.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %ld, SIZE_MAX %ld (neq)\n", max_size, SIZE_MAX); //64-bit
		#endif
		
		bus = kmemdup(&regmap_bus_dm, sizeof(*bus), GFP_KERNEL);
		if (!bus)
			return ERR_PTR(-ENOMEM);

		//#if 0 //[VER DIFF]
		//bus->free_on_exit = true;
		//#endif
		bus->max_raw_read = max_size;
		bus->max_raw_write = max_size;
		return bus;
	}
	
	#if COMPLEX2_ARM32_64
	printk("e.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %d, SIZE_MAX %d\n", max_size, SIZE_MAX); //32-bit
	//printk("e.dm9051 devm_regmap_init_spi use as '(struct regmap_bus *) bus->max_raw_write' %ld, SIZE_MAX %ld\n", max_size, SIZE_MAX); //64-bit
	#endif
#endif

	return &regmap_bus_dm;
}
#endif
