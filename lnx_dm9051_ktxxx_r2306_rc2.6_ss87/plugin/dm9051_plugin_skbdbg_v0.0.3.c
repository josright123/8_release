#ifndef	CACHE_SKB_H_
#define	CACHE_SKB_H_

//Determine [conf_ver A] [A. compiler options mode]
//       or [conf_ver B] [B. _mdef in program code]
#define	DEF_MONITOR_NRX			0 //0 //packet number for rx debug display	//[optional define can be eventually permenently removed~]
#define	DEF_MONITOR_NTX			0 //0 //packet number for tx debug display
#define	DEF_RX_GATE_ALL_ALLOW		1
#define	DEF_TX_GATE_ALL_ALLOW		1
#define	DEF_MONITOR_NRX_MISC		12
#define DEF_DHCP_NRX			2
#define DEF_DHCP_NTX			2
#define DEF_COMPLETE_NRX		90
#define DEF_COMPLETE_NTX		30

#define	dmc_rx(ndev, skb, enable_nrx, enable_ntx, dhcp_nrx, nrx_misc)		\
	if (enable_nrx | enable_ntx)			\
		do {					\
			const u8 *buf = skb->data; \
			if (((cb.rx_total.val < enable_nrx) && (dm_rx_gate(skb) || dm_rx_gate_all_allow())) || \
				(IS_ICMP && !IS_ICMP_REQUEST && !IS_ICMP_REPLY && IS_ICMP_DST_UNREACHABLE) || \
				(cb.dhcp_total_rx.val < dhcp_nrx && IS_DHCPRX) ) { /* only (IS_ICMP && IS_ICMP_DST_UNREACHABLE) is enough */ \
				/*printk(".dm9 [monitor] <<rx [%d/%d] ~len~ %d\n", (cb.rx_total.val+1), enable_nrx, skb->len);*/ \
				/* if (cb.dhcp_total_rx.val < _modef.def_monitor_nrx_dhcp && IS_DHCPRX) { */ \
				/* } */ \
				cb.rx_total.val++; \
				/* if (cb.rx_total.val <= nrx_misc) { */ \
					sprint_hex_dump0(KERN_DEBUG, NULL, "monitor <<rx  ", 32, skb->data, 0, (skb->len < 66) ? skb->len : 66, true, DGROUP_NONE, true); \
					dm_check_rx(ndev, skb);		\
				/* } */ \
			} \
		} while(0)

#define	dmc_tx(ndev, skb, enable_nrx, enable_ntx, dhcp_ntx)		\
	if (enable_nrx | enable_ntx)			\
		do {					\
			const u8 *buf = skb->data; \
			if (((cb.tx_total.val < enable_ntx) && (dm_tx_gate(skb) || dm_tx_gate_all_allow())) || \
				(IS_ICMP && !IS_ICMP_REQUEST && !IS_ICMP_REPLY && IS_ICMP_DST_UNREACHABLE) || \
				(cb.dhcp_total_tx.val < dhcp_ntx && IS_DHCPTX) ) { /* only (IS_ICMP && IS_ICMP_DST_UNREACHABLE) is enough */ \
				/* if (cb.dhcp_total_tx.val < def_monitor_ntx_dhcp && IS_DHCPTX) { */ \
				/* } */ \
				printk(".dm9 [monitor] tx>> [%d/%d] _len_ %d\n", (cb.tx_total.val+1), enable_ntx, skb->len); \
				cb.tx_total.val++;		\
				sprint_hex_dump0(KERN_DEBUG, NULL, "monitor   tx>>", 32, skb->data, 0, (skb->len < 66) ? skb->len : 66, true, DGROUP_NONE, true); \
				dm_check_tx(ndev, skb);		\
			} \
		} while(0)

#if DEF_MONITOR_NRX | DEF_MONITOR_NTX

  //[conf_ver A].//[A. compiler options mode]
  #undef	function_monitor_rx
  #undef	function_monitor_tx
  #define	function_monitor_rx(ndev,skb)		if (ndev->stats.rx_packets < DEF_COMPLETE_NRX) \
													dmc_rx(ndev, skb, DEF_MONITOR_NRX, DEF_MONITOR_NTX, DEF_DHCP_NRX, /*DEF_MONITOR_NRX_MISC*/ 0)
  #define	function_monitor_tx(ndev,skb)		if (ndev->stats.tx_packets < DEF_COMPLETE_NTX) \
													dmc_tx(ndev, skb, DEF_MONITOR_NRX, DEF_MONITOR_NTX, DEF_DHCP_NTX)
  #undef	function_monitor_rxe
  #undef	function_monitor_txe
  #define	function_monitor_rxe(ndev)			if (ndev->stats.rx_packets == DEF_COMPLETE_NRX) \
													printk("complete r%d monitor\n", DEF_COMPLETE_NRX)
  #define	function_monitor_txe(ndev)			if (ndev->stats.tx_packets == DEF_COMPLETE_NTX) \
													printk("complete t%d monitor\n", DEF_COMPLETE_NTX)
													
  //#define	function_monitor_rx(ndev,skb)		dm9051_cache_rx(ndev, skb)	//dm_skb_rx(ndev,skb)
  //#define	function_monitor_tx(ndev,skb)		dm9051_cache_tx(ndev, skb)	//dm_skb_tx(ndev,skb)
  //#include	"skb_main0.c"
 
 #if CODE_RX_REQUEST_SRC_CODE
  static int dm_rx_gate_all_allow(void) {
    return DEF_RX_GATE_ALL_ALLOW;
  }
 #endif
  
  static int dm_tx_gate_all_allow(void) {
    return DEF_TX_GATE_ALL_ALLOW;
  }

#else 

  //[conf_ver B].//[B. _mdef in program code]
  struct mod_ct {
	int	def_monitor_nrx;
	int	def_monitor_ntx;
	int	def_rx_gate_all_allow;
	int	def_tx_gate_all_allow;
  /*int def_monitor_nrx_misc; */
	int def_monitor_nrx_dhcp;
	int def_monitor_ntx_dhcp;
	int def_complete_nrx;
	int def_complete_ntx;
  };
  static const struct mod_ct _modef = { //[mdef, static const data mode]
	.def_monitor_nrx	= 1, //1,//0, //packet number for rx debug display
	.def_monitor_ntx	= 2, //5,//No debug tx (0 for no debug tx), packet number for tx debug display (1000 for debug tx)
	.def_rx_gate_all_allow	= 1, // When starting your debug, you may need display all rx packets
	.def_tx_gate_all_allow	= 1, // When starting your debug, you may need display all tx packets
  /*.def_monitor_nrx_misc	= 3, */
	.def_monitor_nrx_dhcp	= 2,
	.def_monitor_ntx_dhcp	= 2,
	.def_complete_nrx	= 90,
	.def_complete_ntx	= 30,
  };
  #undef	function_monitor_rx
  #undef	function_monitor_tx
  #define	function_monitor_rx(ndev,skb)		if (ndev->stats.rx_packets < _modef.def_complete_nrx) \
													dmc_rx(ndev, skb, _modef.def_monitor_nrx, _modef.def_monitor_ntx, _modef.def_monitor_nrx_dhcp, /*_modef.def_monitor_nrx_misc*/ 0)
  #define	function_monitor_tx(ndev,skb)		if (ndev->stats.tx_packets < _modef.def_complete_ntx) \
													dmc_tx(ndev, skb, _modef.def_monitor_nrx, _modef.def_monitor_ntx, _modef.def_monitor_ntx_dhcp)
  #undef	function_monitor_rxe
  #undef	function_monitor_txe
  #define	function_monitor_rxe(ndev)			if (ndev->stats.rx_packets == _modef.def_complete_nrx) \
													printk("complete r%d monitor\n", _modef.def_complete_nrx)
  #define	function_monitor_txe(ndev)			if (ndev->stats.tx_packets == _modef.def_complete_ntx) \
													printk("complete t%d monitor\n", _modef.def_complete_ntx)
  //#define	function_monitor_rx(ndev,skb)		dm9051_mod_rx(ndev, skb)
  //#define	function_monitor_tx(ndev,skb)		dm9051_mod_tx(ndev, skb)
  //#include	"mdef_main.c"
  
 #if CODE_RX_REQUEST_SRC_CODE
  static int dm_rx_gate_all_allow(void) {
    return _modef.def_rx_gate_all_allow;
  }
 #endif
 
  static int dm_tx_gate_all_allow(void) {
    return _modef.def_tx_gate_all_allow;
  }

#endif

#include "dm_uip_hexdump.c"
#include "dm_subdump.c"
#include "dm_tcpipdump.c"

#endif //CACHE_SKB_H_
