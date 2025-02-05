/*
 * Data structure definition
 */
#ifndef	DM9051_DESC_H_
#define	DM9051_DESC_H_

/*
 * Parameter : fname, titleflg
 */
#undef dm9051_cache_desc_init
#define	dm9051_cache_desc_init(funcname, titleflg) \
	printk("\n"); \
	if (titleflg) { \
		printk("[%s]( %s hardware config: %s )\n", funcname, DM9051_DRV_NAME, BOARD_KT); \
		printk("[%s]( %s software config: %s )\n", funcname, DM9051_DRV_NAME, DM9051_KT); \
		printk("[%s]( %s history: %s )\n", funcname, DM9051_DRV_NAME, DM9051_VINF); \
		printk("[%s]( %s version: %s )\n", funcname, DM9051_DRV_NAME, DM9051_DRC); \
	 } else \
		printk("[module] - %s\n", funcname)

#endif //DM9051_DESC_H_
