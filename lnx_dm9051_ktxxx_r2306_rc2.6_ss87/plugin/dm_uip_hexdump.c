#if 1
// NOte: only for edit software's declaration key-words, to got bold colored for easy visual.
struct dm_togg {
  int o_togg_expect_trips;
  int o_togg_enough_to_link;
};
struct eeprom {
};
struct dm_status {
};
struct dm_lnk_reset {
};
struct dm_boot_msg {
};
struct cons_t {
};
struct tog {
};
struct wr {
  int o_val;
};
struct ram {
};
struct cnt {
};
struct tmp {
};

  struct arp_total_tx { //to got bold and colored
  };
  struct arp_uni_out_tx {
  };
  struct dhcp_total_tx { //to got bold and colored
  };
  struct misc_total_tx { //to got bold and colored
  };
  struct tx_total { //to got bold and colored
  };
  struct arp_total_rx { //to got bold and colored
  };
  struct arp_uni_me_rx { //to got bold and colored
  };
  struct arp_brdcst_rx {
  };
  struct dhcp_total_rx { //to got bold and colored
  };
  struct misc_total_rx {
  };
  struct rx_total { //to got bold and colored
  };
#endif
 
//[dm_conf_ver.c].s
/**
 * struct board_ct - the all for the configuration
 * @src_version_desc318: driver's version string, indivdual from each kernel version
 * @src_version_desc515: driver's version string, indivdual from each kernel version
 * @eeprom_restore1: restore to Korsgaard's dongle default
 * @eeprom_display1: optional for display the eeprom content
 * @flg_disable_mdix: disable MDIX
 * @flg_usb_pwr: set USB power *2 mA
 * @flg_ep3_ms: set EP3 interval *1 ms
 * @evtflg_defer: put the EVENT_LINK_RESET event
 * @trip_printed1: set EXPECT TRIPS
 * @max_intrip1: set ENOUGH TO LINK
 * @poll1: set STATUS IN COUNT TO TOGGLE
 * @flg_writereg20: while EVENT_LINK_RESET event, optional software write MDI/MDIX
 * @cons_t: the struct for all the const definitions
 *
 * To configure the driver, since there are sevaral options. 
 */
 
 #define	MSG_TX_MODSRC0	(1 << 0)  //Src IP addr 0.0.0.0
 #define	MSG_TX_MODSELF	(1 << 1)  //Src to Dst self
 #define	MSG_TX_MODELSE	(1 << 2)  //others
 
struct protc { /* Enable flag and Counter */
  int enable, hex_show, val;
};
 
struct board_ct {
  struct {
    u16 usb_pwr;
    u16 ep3_ms;

    int eeprom_restore1;
    int eeprom_display1;

    struct {
	    int flg_disable_mdix;
	    int flg_usb_pwr;
	    int flg_ep3_ms;
    } eeprom;
    
    struct {
	    int evtflg_defer;
	    struct {
	      int trip_printed1; //[_n_togg_expect_trips1] //_display_count-controlling
	      int max_intrip1; //[_n_togg_enough_to_link1] //[_minus_count_to_enough] //= int n_togg_expect; //[_expect_enough]
	      int poll1; //'machine_status_in1' //'machine_status_toggle'
	    } dm_togg;
    } dm_status;

    struct {
	    int flg_writereg20; //const flag
    } dm_lnk_reset;
    
    struct {
	int flag_ttxrrx_boot;
	int flag_tx_brdcst_arp_hex_bootx;
	int flag_tx_unicast_hex_booty;
	int flag_tx_mod_bootz;
	int flag_tx_self_ip_summ_boote;
    
	int msg_tx_uni_arp_hex_msg;
	int msg_tx_uni_tcp_hex;
	int msg_tx_uni_udp_hex;
	int msg_tx_uni_xxx_hex;
	int msg_rx_multicast_arp_hex;
	int msg_rx_unicastme_arp_hex;

	int msg_print_arp_parse_my_ip;
	int msg_print_arp_brdcst_case1_tx;
	int msg_print_arp_hex; //We debug arp
    } dm_boot_msg;
    
    int msg_print_arp_tx_for_ip;
    int msg_print_arp_rx_for_ip;
    int msg_print_arp_tx_to_gateway;
    int msg_print_arp_rx_from_gateway;
    
    int msg_print_tx_multi_udp;
    int msg_print_tx_uni_udp;
    
    int local_dip_ttx;
    int local_dip_ttx_reach;
    int local_dip_rrrx;
    int local_dip_rrrx_reach;
    int local_gap_reach_ttx;
    int local_gap_reach_rrrx;
    
  } cons_t;
};

struct board_st {
  struct {
    struct {
      int trip; //[_n_togg_expect_trips]
      int intrip; //[_n_togg_enough_to_link] //[_minus_count_to_enough]
      int poll; //[_machine_status_in]
    } tog;
    struct {
      u16 mdio_val;
    } wr;
    //int force_ok_to_print;
    int force_mode_magic;
    
    u8 dhcp_tx_srvaddr[4];
    u8 dhcp_rx_srvaddr[4];
    u8 dhcp_req_ip[4];
    u16 detect_srv_ip[2];

    int cas1, cas2, cas3;
    
    int tx_total_next;
    int tx_reachfor_disp;
    
    int rx_total_next;
    int rx_reachfor_disp;
  } ram;
  
  struct protc	arp_total_tx;
  struct protc	arp_uni_out_tx;
  struct protc	dhcp_total_tx;
  struct protc	misc_total_tx;
  struct protc	tx_total;
  struct protc	arp_total_rx;
  struct protc	arp_uni_me_rx, arp_brdcst_rx;
  struct protc	dhcp_total_rx;
  struct protc	misc_total_rx;
  struct protc	rx_total;
  
  //struct {
  //  int arp_total_tx;
  //  int dhcp_total_tx;
  //  int misc_total_tx;
  //  int tx_total;

  //  int arp_total_rx;
  //  int arp_uni_me_rx;
  //  int dhcp_total_rx;
  //  int rx_total;
  //} cnt;

  struct {
    int	last_print_icmp;
    int msg_print_icmp; //We debug ping command, icmp
    //int flg_has_arp_unicast_enter; //=arp_enter_unicast_fnd;

    int flg_wait_uni_arp_finish; //=..
    
    int arp_we_trans_count;
    int arpbrdcst_do_count_RX;
    int arp_wait_cas1_tx_brdcst;
    int arp_never_cas2_tx_uni;
    int arp_safty_cas1_tx_brdcst;
    int arp_safty_cas3_tx_uni;
  } tmp;

  int n;
  char bff[100];
};

//#if DEF_MONITOR_RX | DEF_MONITOR_RX
// NOte: board_ct, manipulated const data in execution code
static const struct board_ct cdef = { //[cdef][conf_ver]
  .cons_t = {
    .usb_pwr			= 0x005a,
    .ep3_ms			= 0x00fa,
    
    .eeprom_restore1		= 0,
    .eeprom_display1		= 0, //1,
    .eeprom = {
      .flg_disable_mdix		= 1, //word3.bit[15:14]= 01b,word7
      .flg_usb_pwr		= 0, //word3.bit[15:14]= 01b,word11 ??bit[15:14]
      .flg_ep3_ms		= 1, //word12
    },

    .dm_status = {
      .evtflg_defer = 1,
      .dm_togg = {
        .trip_printed1		= 3, //_display_count-controlling
        .max_intrip1		= 6, //(.n_togg_expect = 6,)
        .poll1			= 6,
      },
    },
    .dm_lnk_reset = {
      .flg_writereg20		= 0, // <-- 1
    },
    
    .dm_boot_msg = {
      .flag_ttxrrx_boot		= 0,
      .flag_tx_brdcst_arp_hex_bootx	= 0,
      .flag_tx_unicast_hex_booty = 0,
      .flag_tx_mod_bootz	= 0 | MSG_TX_MODELSE,
      .flag_tx_self_ip_summ_boote = 0,
      
      .msg_tx_uni_arp_hex_msg	= 0,
      .msg_tx_uni_tcp_hex	= 1,
      .msg_tx_uni_udp_hex	= 0,
      .msg_tx_uni_xxx_hex	= 1,
      .msg_rx_multicast_arp_hex	= 0,
      .msg_rx_unicastme_arp_hex	= 0,
      
      .msg_print_arp_parse_my_ip = 0, //arp declare my ip print
      .msg_print_arp_brdcst_case1_tx = 0, //every send arp 
      .msg_print_arp_hex	= 0, //arp print
    },
    
    .msg_print_arp_tx_for_ip	= 1, //arp's
    .msg_print_arp_rx_for_ip	= 1, //arp's
    .msg_print_arp_tx_to_gateway = 1, //arp's
    .msg_print_arp_rx_from_gateway = 1, //arp's
    
    .msg_print_tx_multi_udp	= 0, //tx-multi-udp
    .msg_print_tx_uni_udp	= 0, //tx-uni-udp
    
    .local_dip_ttx		= 0,
    .local_dip_ttx_reach	= 1,
    .local_dip_rrrx		= 0,
    .local_dip_rrrx_reach	= 1,
    .local_gap_reach_ttx	= 1, //1, further gap show for reduce display
    .local_gap_reach_rrrx	= 1, //1, further gap show for reduce display
  },
};
//#endif

// Note: ram
static struct board_st cb = {
  .ram = {
    .tog = {
      .trip			= 0,
      .intrip			= 0, //(.n_togg_expect minus counting)
      .poll			= 0,
    },
    .wr = {
      .mdio_val			= 0x810,
    },
    //.force_ok_to_print	= 0,
    .force_mode_magic		= 0,
    .dhcp_tx_srvaddr[0]	= 0,
    .dhcp_tx_srvaddr[1]	= 0,
    .dhcp_tx_srvaddr[2]	= 0,
    .dhcp_tx_srvaddr[3]	= 0,
    .dhcp_rx_srvaddr[0]	= 0,
    .dhcp_rx_srvaddr[1]	= 0,
    .dhcp_rx_srvaddr[2]	= 0,
    .dhcp_rx_srvaddr[3]	= 0,
    .dhcp_req_ip[0]		= 0,
    .dhcp_req_ip[1]		= 0,
    .dhcp_req_ip[2]		= 0,
    .dhcp_req_ip[3]		= 0,
    .detect_srv_ip[0]	= 0,
    .detect_srv_ip[1]	= 0,
    .cas1			= 0,
    .cas2			= 0,
    .cas3			= 0,

    .tx_total_next		= 0,
    .tx_reachfor_disp		= 0,
    
    .rx_total_next		= 0,
    .rx_reachfor_disp		= 0,
  },
  
  .arp_total_tx = { .enable= 1, .val= 0},
  .arp_uni_out_tx = { .enable= 1, .val= 0}, //.arp_uni_out_tx = { .enable= 0, .val= 0},

  .dhcp_total_tx = {
	.enable	= 1,
	.hex_show = 0, //dhcp_tx
	.val	= 0,
  },
  .misc_total_tx = {
	.enable	= 1,
	.hex_show = 0, //misc_tx
	.val	= 0,
  },
  .tx_total = {
	.enable	= 1,
	.val	= 0,
  },

  .arp_total_rx = { .enable= 1, .val= 0},
  .arp_uni_me_rx = { .enable= 1, .val= 0}, .arp_brdcst_rx = { .enable= 1, .val= 0},
  
  .dhcp_total_rx = {
	.enable	= 1,
	.val	= 0,
  },
  .misc_total_rx = {
	.enable	= 1,
	.hex_show = 0, //misc_rx
	.val	= 0,
  },
  .rx_total = {
	.enable	= 1,
	.val	= 0,
  },
    
  .tmp = {
    .msg_print_icmp		= 50, //100, //0, 1, We debug ping command, icmp
    //.flg_has_arp_unicast_enter = 0,
    //.flg_wait_no_unicast_arp_enter = 0,
      .flg_wait_uni_arp_finish	= 0, //1 for immediately effect, or 0 for wait a period. ethier one is OK ///..=(.flg_has_arp_unicast_enter)
    
    .arp_we_trans_count		= 0, //(instead to for 'trans_may_unicast_count')
    .arpbrdcst_do_count_RX	= 0,

    .arp_wait_cas1_tx_brdcst	= 0,
    .arp_never_cas2_tx_uni	= 0,
    .arp_safty_cas1_tx_brdcst	= 0,
    .arp_safty_cas3_tx_uni	= 0,
    
    
    //.arp_total_tx		= 0,
    //.arp_uni_me_rx		= 0,
    
    //.dhcp_total_tx		= 0,
    //.misc_total_tx		= 0,
    //.tx_total			= 0,
    
    //.dhcp_total_rx		= 0,
    //.rx_total			= 0,
  },
  .n				= 0,
};

//#define GAP_REACH_TX_ENABLE	1
//#define GAP_REACH_RX_ENABLE	1
#define GAP_START_FOR		6
#define GAP_REACH_FOR_TX	100000 //control TX ARP unicast safty reach for extra total quantity and display
#define GAP_REACH_FOR_RX	200000
#define GAP_REACH_FOR_TX_II	1000000 
#define GAP_REACH_FOR_RX_II	1000000
//[dm_conf_ver.c].e

#define DGROUP_TX		0
#define DGROUP_TXII		1
#define DGROUP_RX		2
#define DGROUP_RXII		3
#define DGROUP_DUMP		4
#define DGROUP_NONE		5
#define SEG_QUAN		16

#define	DEF_BRDCST_IT_RX	0
#define	DEF_BRDCST_IT_TX	0

struct uip_eth_addr {
  u8 addr[6];
};
struct uip_eth_hdr {
  struct uip_eth_addr dest;
  struct uip_eth_addr src;
  u16 type;
};

struct arp_hdr {
  struct uip_eth_hdr ethhdr;
  u16 hwtype;
  u16 protocol;
  u8 hwlen;
  u8 protolen;
  u16 opcode;
  struct uip_eth_addr shwaddr;
  u16 sipaddr[2];
  struct uip_eth_addr dhwaddr;
  u16 dipaddr[2];
};

struct ethip_hdr {
  struct uip_eth_hdr ethhdr;
  /* IP header. */
  u8 vhl,
    tos,
    len[2],
    ipid[2],
    ipoffset[2],
    ttl,
    proto;
  u16 ipchksum;
  u16 srcipaddr[2],
    destipaddr[2];
};

struct uip_icmpip_hdr { /* The ICMP and IP headers. */
#if 1
  /* IPv4 header. */
    u8 vhl, tos,
      len[2], ipid[2], ipoffset[2],
      ttl, proto;
    u16 ipchksum;
    u16 srcipaddr[2],
      destipaddr[2];
#endif
  
  /* ICMP (echo) header. */
  u8 type, icode;
  u16 icmpchksum;
  u16 id, seqno;
};

struct uip_udpip_hdr { /* The UDP and IP headers. */
#if 1
  /* IP header. */
  u8 vhl, tos,
    len[2], ipid[2], ipoffset[2],
    ttl, proto;
  u16 ipchksum;
  u16 srcipaddr[2],
    destipaddr[2];
#endif /* UIP_CONF_IPV6 */
  
  /* UDP header. */
  u16 srcport,
    destport;
  u16 udplen;
  u16 udpchksum;
};

#if 0
/* The TCP and IP headers. */
struct uip_tcpip_hdr {
  /* IPv4 header. */
  #if 1
  u8_t vhl,
    tos,
    len[2],
    ipid[2],
    ipoffset[2],
    ttl,
    proto;
  u16_t ipchksum;
  u16_t srcipaddr[2],
    destipaddr[2];
  #endif
  
  /* TCP header. */
  u16_t srcport,
    destport;
  u8_t seqno[4],
    ackno[4],
    tcpoffset,
    flags,
    wnd[2];
  u16_t tcpchksum;
  u8_t urgp[2];
  u8_t optdata[4];
};
#endif

struct dhcp_msg {
  u8 op, htype, hlen, hops;
  u8 xid[4];
  u16 secs, flags;
  u8 ciaddr[4]; /* current ip */
  u8 yiaddr[4]; /* your ip */
  u8 siaddr[4]; /* srv ip */
  u8 giaddr[4]; /* gateway ip */
  u8 chaddr[16];
#ifndef UIP_CONF_DHCP_LIGHT
  u8 sname[64];
  u8 file[128];
#endif
  u8 options[312];
};

#define dm_ipaddr1(addr)		((((u16 *)(addr))[0]) & 0xff)		//To display and compare
#define dm_ipaddr2(addr)		((((u16 *)(addr))[0]) >> 8)
#define dm_ipaddr3(addr) 		((((u16 *)(addr))[1]) & 0xff)
#define dm_ipaddr4(addr) 		((((u16 *)(addr))[1]) >> 8)

#define	HTONS(n)				(u16)((((u16) (n)) << 8) | (((u16) (n)) >> 8))

#define	dm_ip16(htonl, htonh)	HTONS(((htonl) << 8) | (htonh))		//To collect two oct. to be u16

#define UIP_LLH_LEN				14
#define UIP_UDPH_LEN			8
#define UIP_IPH_LEN				20
#define UIP_IPUDPH_LEN			(UIP_UDPH_LEN + UIP_IPH_LEN)

#define BUF   ((struct arp_hdr *)buf) //&ptr[0]
#define IPBUF ((struct ethip_hdr *)buf) //&ptr[0]
#define ICMPBUF ((struct uip_icmpip_hdr *)&(((u8 *)buf)[ UIP_LLH_LEN]))
#define UDPBUF ((struct uip_udpip_hdr *)&(((u8 *)buf)[ UIP_LLH_LEN]))
#define DHCPBUF ((struct dhcp_msg *)&(((u8 *)buf)[ UIP_LLH_LEN + UIP_IPUDPH_LEN]))

#define UIP_PROTO_ICMP  			1
#define UIP_PROTO_IGMP_JJ  			2
#define UIP_PROTO_TCP   			6
#define UIP_PROTO_UDP   			17

//#define IsArp(buf)			(HTONS(((struct arp_hdr *)(buf))->ethhdr.type) == (0x0806))
#define	IsBroadcast(add)		(memcmp(add, broadcast_ethaddr.addr, sizeof(struct uip_eth_addr))==0)
#define	IsMulticast(buf)		(((u8 *)(buf))[0] & 0x01)
#define	IsUnicastMe(add,uniadd)		(memcmp(add, uniadd, ETH_ALEN)==0)

#define	IS_ARP				(HTONS(BUF->ethhdr.type) == (0x0806))
#define	IS_ICMP				(IPBUF->proto == UIP_PROTO_ICMP)

#define ICMP_ECHO_REPLY 			0
#define ICMP_DEST_UNREACH       		3
#define ICMP_ECHO       			8

#define	ICMP_NET_UNREACH			0
#define	ICMP_PORT_UNREACH			3
#define	ICMP_FRAG_NEEDED			4

#define	IS_ICMP_REPLY			(ICMPBUF->type == 0 && ICMPBUF->icode == 0)
#define	IS_ICMP_DST_UNREACHABLE		(ICMPBUF->type == 3)
#define	IS_ICMP_REQUEST			(ICMPBUF->type == 8 && ICMPBUF->icode == 0)
#define	DST_UNREACHABLE_ICODE		(ICMPBUF->icode)

#define	IS_PING_REQ_HEAD		(IS_ICMP && IS_ICMP_REQUEST)
//#define IS_PING_REQ_BODY		(IS_ICMP && "unknow")
#define	IS_PING_REP_HEAD		(IS_ICMP && IS_ICMP_REPLY)
//#define IS_PING_REP_BODY		(IS_ICMP && "unknow")

#define DHCPC_SERVER_PORT			67
#define DHCPC_CLIENT_PORT			68
#define DHCP_REQUEST				1
/*#define DHCP_HTYPE_ETHERNET		1 */

#define DHCP_OPTION_REQ_IPADDR		0x32 //=50
#define DHCP_OPTION_SERVER_ID		0x36 //=54

#define DHCP_OPTION_LEASE_TIME		0x33 //=51
#define DHCP_OPTION_END				0xff //=255

#define DHCP_OPTION_MSG_TYPE		0x35 //=53

#define DHCPDISCOVER				1
#define DHCPREQUEST					3
#define DHCPOFFER					2
#define DHCPACK						5

//#define	IsDHCPRx(buf)		(HTONS(UDPBUF->srcport) == DHCPC_SERVER_PORT &&
//								HTONS(UDPBUF->destport) == DHCPC_CLIENT_PORT)
//#define	IsDHCPRx_OFFER(buf)	(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&
//								DHCPBUF->options[5] == 1 &&
//								DHCPBUF->options[6] == DHCPOFFER)
//#define	IsDHCPRx_ACK(buf)		(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&
//								DHCPBUF->options[5] == 1 &&
//								DHCPBUF->options[6] == DHCPACK)
#define	IS_DHCPRX				(HTONS(UDPBUF->srcport) == DHCPC_SERVER_PORT &&	\
								HTONS(UDPBUF->destport) == DHCPC_CLIENT_PORT)
#define	IS_DHCPRX_OFFER			(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&	\
								DHCPBUF->options[5] == 1 &&	\
								DHCPBUF->options[6] == DHCPOFFER)
#define	IS_DHCPRX_ACK			(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&	\
								DHCPBUF->options[5] == 1 &&	\
								DHCPBUF->options[6] == DHCPACK)
//#define	IsDHCPTx			(HTONS(UDPBUF->srcport) == DHCPC_CLIENT_PORT &&
//								HTONS(UDPBUF->destport) == DHCPC_SERVER_PORT)
//#define	IsDHCPTx_DISCOVER 	(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&
//								DHCPBUF->options[5] == 1 &&
//								DHCPBUF->options[6] == DHCPDISCOVER)
//#define	IsDHCPTx_REQUEST 	(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&
//								DHCPBUF->options[5] == 1 &&
//								DHCPBUF->options[6] == DHCPREQUEST)
//#define	DM_arp_self_IP		(BUF->sipaddr[0]==BUF->dipaddr[0] && BUF->sipaddr[1]==BUF->dipaddr[1])
#define	IS_DHCPTX				(HTONS(UDPBUF->srcport) == DHCPC_CLIENT_PORT &&	\
								HTONS(UDPBUF->destport) == DHCPC_SERVER_PORT)
#define	IS_DHCPTX_DISCOVER 		(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&	\
								DHCPBUF->options[5] == 1 &&						\
								DHCPBUF->options[6] == DHCPDISCOVER)
#define	IS_DHCPTX_REQUEST 		(DHCPBUF->options[4] == DHCP_OPTION_MSG_TYPE &&	\
								DHCPBUF->options[5] == 1 &&						\
								DHCPBUF->options[6] == DHCPREQUEST)
#define	DM_ARP_SELF_IP			(BUF->sipaddr[0]==BUF->dipaddr[0] && BUF->sipaddr[1]==BUF->dipaddr[1])

static const struct uip_eth_addr broadcast_ethaddr = {{0xff,0xff,0xff,0xff,0xff,0xff}};

#define	DM_save_first(to, toc, val)	\
	do {				\
		to = val;	\
	} while(0)
	

#define BL_SIZE 10 // sizeof(blacklist_DAVICOM_COM_TW)/sizeof(ip_t)
typedef struct { //typedef char[4] ipu;
  u8 addr[4];
} ip_t;

static const ip_t blacklist_DAVICOM_COM_TW[BL_SIZE] = {
	{{  0,  0,  0, 0}}, {{192,168,  2,11}}, {{192,168,  2,13}}, {{192,168,  2,30}}, {{192,168,  2,100}},
	{{192,168,  2,228}}, {{192,168,254,12}}, {{192,168,254,83}}, {{192,168,  0,120}}, {{192,168,  3,100}},
};
static int IsBlackList(struct sk_buff *skb)
{
	const u8 *buf = skb->data;
	int i;
	
	for (i=0; i< BL_SIZE; i++) {
		//if (!memcmp(&blacklist_DAVICOM_COM_TW[i], &BUF->sipaddr[0], sizeof(ip_t))) //BUF->sipaddr[0] BUF->sipaddr[1]
		//	break;
		if (blacklist_DAVICOM_COM_TW[i].addr[0] == (BUF->sipaddr[0] & 0xff) &&
			blacklist_DAVICOM_COM_TW[i].addr[1] == (BUF->sipaddr[0] >> 8) &&
			blacklist_DAVICOM_COM_TW[i].addr[2] == (BUF->sipaddr[1] & 0xff) &&
			blacklist_DAVICOM_COM_TW[i].addr[3] == (BUF->sipaddr[1] >> 8))
			break;
	}
	return (i < BL_SIZE);
}

	
	


#if DEF_BRDCST_IT_TX
static void Write_ARP_broadcast(void *buf)
{
  printk(" %d.Write_ICMP_broadcast()\n", ++cb.tmp.arpbrdcst_do_count_RX);
  memcpy(IPBUF->ethhdr.dest.addr, broadcast_ethaddr.addr, 6);
}
#endif

//#if DEF_MONITOR_TX //vs. DEF_MONITOR_RX | DEF_MONITOR_TX
static void roll_next_tx(int reachfor_now, int total_now)
{
	if (cdef.cons_t.local_dip_ttx_reach) {
	if (cdef.cons_t.local_gap_reach_ttx) {
	//#if GAP_REACH_TX_ENABLE
		if (cb.tx_total.val > cb.ram.tx_total_next) {
		  cb.ram.tx_reachfor_disp ++;
		  if (cb.ram.tx_reachfor_disp < GAP_START_FOR) //=5
		    cb.ram.tx_total_next = cb.tx_total.val + GAP_START_FOR; //=5
		  else if (cb.ram.tx_reachfor_disp < (GAP_START_FOR+3)) { //=5+3 (8)
		    cb.ram.tx_total_next /= GAP_REACH_FOR_TX; //100000;
		    cb.ram.tx_total_next *= GAP_REACH_FOR_TX; //100000;
		    cb.ram.tx_total_next += GAP_REACH_FOR_TX; //100000;
		  } else {
		    cb.ram.tx_total_next /= GAP_REACH_FOR_TX_II;
		    cb.ram.tx_total_next *= GAP_REACH_FOR_TX_II;
		    cb.ram.tx_total_next += GAP_REACH_FOR_TX_II;
		  }

		  printk(" (tx reach %2d).s (%7d next >  ttx %7d)(tx next reach %2d).e\n", reachfor_now, total_now, cb.ram.tx_total_next, cb.ram.tx_reachfor_disp);
		}
	//#endif
	}
	}
}
//#endif

static void roll_next_rx(int reachfor_now, int total_now)
{
  cb.ram.rx_reachfor_disp++;
  if (cb.ram.rx_reachfor_disp < GAP_START_FOR) //=5
    cb.ram.rx_total_next = cb.rx_total.val + GAP_START_FOR; //=5
  else if (cb.ram.rx_reachfor_disp < (GAP_START_FOR+1)) {
    cb.ram.rx_total_next /= GAP_REACH_FOR_RX; //200000;
    cb.ram.rx_total_next *= GAP_REACH_FOR_RX; //200000;
    cb.ram.rx_total_next += GAP_REACH_FOR_RX; //200000;
  } 
  else {
    cb.ram.rx_total_next /= GAP_REACH_FOR_RX_II;
    cb.ram.rx_total_next *= GAP_REACH_FOR_RX_II;
    cb.ram.rx_total_next += GAP_REACH_FOR_RX_II;
  }
  printk(" -rx reach %2d-.s (%7d next > rrrx %7d)(rx next reach %2d).e\n", reachfor_now, total_now, cb.ram.rx_total_next, cb.ram.rx_reachfor_disp);
}

#if DEF_BRDCST_IT_RX || DEF_BRDCST_IT_TX
static void sprint_DA_field(void *buf)
{
  u8 *ptr = buf;

  printk(" da: %02x %02x %02x %02x %02x %02x\n", BUF->ethhdr.dest.addr[0],
   BUF->ethhdr.dest.addr[1],BUF->ethhdr.dest.addr[2],BUF->ethhdr.dest.addr[3],BUF->ethhdr.dest.addr[4],BUF->ethhdr.dest.addr[5]);
}
#endif

//#if DEF_MONITOR_RX | DEF_MONITOR_TX
typedef enum {
	SIP_TYPE_ARP16 = 0,
	SIP_TYPE_ARP32,
	SIP_TYPE_DHCPTX,
	SIP_TYPE_DHCPRX_OPT_SRV_ID,
	SIP_TYPE_DHCPTX_IP_DEST,
	SIP_TYPE_END,
} spi_type;

static void server_ip_handler(spi_type stype, u16 sipadd0, u16 sipadd1)
{
	#if 0
	const char *spiheadstr[SIP_TYPE_END] = {
		"sTTX dm_arp_st16",
		"sTTX dm_arp_st32",
		"sTTX dm_dhcp_tx",
		"sRRRX dm_dhcp_rx_opt_srv_id",
		"sTTX dm_dhcp_tx_ip_dest",
	};
	#endif
	static spi_type mld = SIP_TYPE_END;
	
	if (mld != stype) {
		mld = stype;

		cb.ram.detect_srv_ip[0] = sipadd0;
		cb.ram.detect_srv_ip[1] = sipadd1;
	#if 0
		printk("!      %s detect_srv_ip= %d.%d.%d.%d      !\n",
			spiheadstr[stype],
			cb.ram.detect_srv_ip[0] & 0xff, cb.ram.detect_srv_ip[0] >> 8,
			cb.ram.detect_srv_ip[1] & 0xff, cb.ram.detect_srv_ip[1] >> 8);
		printk("!      [.dm9 save dhcp-srv ip] ~\n");
	#endif
	}
}
       
static void sprint_arp_field_parse(void *buf, int segpos, int segsize, int dgroup)
{
  int dm_st16_flg = 0;

  switch(segpos) {
    case 0:
      segsize -= SEG_QUAN;
      if (segsize)
        goto dm_st16;
      break;
    case 16:
dm_st16:

      segsize -= SEG_QUAN;
      if (segsize) {
		dm_st16_flg = 1;
        goto dm_st32;
      }
      if (dgroup == DGROUP_RX)
       printk("(from %02x:%02x:%02x:%02x:%02x:%02x %d.%d.%d.%d)\n",BUF->shwaddr.addr[0],
         BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
         BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8);
      else { //DGROUP_TX
       if (BUF->sipaddr[0] == 0 && BUF->sipaddr[1] == 0)
		;
       else 
        server_ip_handler(SIP_TYPE_ARP16, BUF->sipaddr[0], BUF->sipaddr[1]);
       printk("(dm9 %02x:%02x:%02x:%02x:%02x:%02x %d.%d.%d.%d)\n",BUF->shwaddr.addr[0],
         BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
         BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8);
      }
      break;
    case 32:
dm_st32:

      if (dm_st16_flg) {
	      if (dgroup == DGROUP_RX || dgroup == DGROUP_RXII) {
			if (dgroup == DGROUP_RXII)
			  printk("-from %02x:%02x:%02x:%02x:%02x:%02x src %d.%d.%d.%d (dm9 %d.%d.%d.%d)- Check Whether const black list macsrc\n", BUF->shwaddr.addr[0],
			   BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
				BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8, 
				BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
			else if (cb.rx_total.val > cb.ram.rx_total_next)
			  printk("-from %02x:%02x:%02x:%02x:%02x:%02x src %d.%d.%d.%d (dm9 %d.%d.%d.%d)\n", BUF->shwaddr.addr[0],
			   BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
				BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8, 
				BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
	      } else { //DGROUP_TX
			int mod = 0;
			if (BUF->sipaddr[0] == 0 && BUF->sipaddr[1] == 0)
				mod |= MSG_TX_MODSRC0;
			else {
			  if (BUF->sipaddr[0] == BUF->dipaddr[0] && BUF->sipaddr[1] == BUF->dipaddr[1])
				  mod |= MSG_TX_MODSELF;
			  else
				  mod |= MSG_TX_MODELSE;
			  server_ip_handler(SIP_TYPE_ARP32, BUF->sipaddr[0], BUF->sipaddr[1]);
			}

			if (cdef.cons_t.dm_boot_msg.flag_tx_mod_bootz & mod) {
				if (cb.tx_total.val > cb.ram.tx_total_next) {
				u16 la = BUF->dipaddr[1] >> 8;
				if (la < 10)
					  printk("-dm9 %02x:%02x:%02x:%02x:%02x:%02x dm9 %03d.%03d.%d.%d (to %03d.%03d.%d.%d)\n", BUF->shwaddr.addr[0],
					   BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
					BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8, 
					BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
				else if (la < 100)
					  printk("-dm9 %02x:%02x:%02x:%02x:%02x:%02x dm9 %03d.%03d.%d.%02d (to %03d.%03d.%d.%2d)\n", BUF->shwaddr.addr[0],
					   BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
					BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8, 
					BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
				else
					  printk("-dm9 %02x:%02x:%02x:%02x:%02x:%02x dm9 %03d.%03d.%d.%03d (to %03d.%03d.%d.%3d)\n", BUF->shwaddr.addr[0],
					   BUF->shwaddr.addr[1],BUF->shwaddr.addr[2],BUF->shwaddr.addr[3],BUF->shwaddr.addr[4],BUF->shwaddr.addr[5], 
					BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8, 
					BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
			   }
			}
	    }
      } else {
	      if (dgroup == DGROUP_RX)
	        printk("(dm9 %02x%02x%02x%02x%02x%02x %d.%d.%d.%d)\n", BUF->dhwaddr.addr[0],
	         BUF->dhwaddr.addr[1],BUF->dhwaddr.addr[2],BUF->dhwaddr.addr[3],BUF->dhwaddr.addr[4],BUF->dhwaddr.addr[5], 
		  BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
	      else
	        printk("(dst %02x:%02x:%02x:%02x:%02x:%02x %d.%d.%d.%d)\n", BUF->dhwaddr.addr[0],
	         BUF->dhwaddr.addr[1],BUF->dhwaddr.addr[2],BUF->dhwaddr.addr[3],BUF->dhwaddr.addr[4],BUF->dhwaddr.addr[5], 
		  BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
      }
      segsize -= (sizeof(BUF->dhwaddr) + sizeof(BUF->dipaddr));
      if (!segsize)
        break;

      segsize -= 6;
      if (!segsize)
        break;

      if (segsize)
        goto dm_st48;
      break;
    case 48:
dm_st48:
      segsize -= 12;
      break;
  }
  if (segsize)
    printk("dm9 MUST segsize 0, BUT is %d (Error! Check!)\n", segsize);
}

#if 0
/* Specific for DHCP Requuest Message - Lease Time (Option 51) */
static void sprint_hex_dump_specific_2(const char *level, struct net_device *ndev, const char *prefix_str,
		    int rowsize, const void *buf, int seg_start, size_t len, bool must, int dgroup, int useflg)
{
	const u8 *ptr = buf;
	int si, se;
	int i, linelen, remaining = len;
	unsigned char linebuf[(12 * 3) + (3 * 16) + 1 + 32];
	
	si = seg_start;
	se = seg_start + len;
	for (i = si; i < se; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;
		
		do {
			int j, nb = 0;
			for (j=0; j< linelen && nb < sizeof(linebuf); j++) {
				if (j && !(j%8))
					nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
				if (((rowsize>>1)!=8) && !(j%(rowsize>>1)))
					nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
				nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, "%02x ", ptr[i+j]); //ptr[i+j] = *(ptr + i + j)
			}
		} while(0);
		
		printk("[%02x][%02x][%02x][%02x][%02x][%02x]   <---   %d, %d, %d seconds\n",
				ptr[i], ptr[i+1], ptr[i+2], ptr[i+3], ptr[i+4], ptr[i+5],
				ptr[i], ptr[i+1], (ptr[i+2] << 24) + (ptr[i+3] << 16) +  (ptr[i+4] << 8) + ptr[i+5]);
	}
}
/* Specific for DHCP Requuest Message - for Requested IP Address (Option 50) */
static void sprint_hex_dump_specific_1(const char *level, struct net_device *ndev, const char *prefix_str,
		    int rowsize, const void *buf, int seg_start, size_t len, bool must, int dgroup, int useflg)
{
	const u8 *ptr = buf;
	int si, se;
	int i, linelen, remaining = len;
	unsigned char linebuf[(12 * 3) + (3 * 16) + 1 + 32];
	
	si = seg_start;
	se = seg_start + len;
	for (i = si; i < se; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;
		
		do {
			int j, nb = 0;
			for (j=0; j< linelen && nb < sizeof(linebuf); j++) {
				if (j && !(j%8))
					nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
				if (((rowsize>>1)!=8) && !(j%(rowsize>>1)))
					nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
				nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, "%02x ", ptr[i+j]); //ptr[i+j] = *(ptr + i + j)
			}
		} while(0);
		
		printk("[%02x][%02x][%02x][%02x][%02x][%02x]   <---   %d, %d, %d.%d.%d.%d\n",
				ptr[i], ptr[i+1], ptr[i+2], ptr[i+3], ptr[i+4], ptr[i+5],
				ptr[i], ptr[i+1], ptr[i+2], ptr[i+3], ptr[i+4], ptr[i+5]);
	}
}
//#endif

static void print_parse_options(struct sk_buff *skb, u8 option_item_name, int dbg) {
  if (dbg) {
	  void *buf = skb->data;
	  
	  u8 *ptr = &DHCPBUF->options[4]; //&DHCPBUF->options[0];
	  u8 *end = skb->data + skb->len;
	  
//.	  printk("[%02x][%02x][%02x][%02x] start\n", DHCPBUF->options[0],DHCPBUF->options[1],DHCPBUF->options[2],DHCPBUF->options[3]);
	  while (ptr < end) {
		if (*ptr == DHCP_OPTION_END) {
//.			printk("[%02x] done.\n", DHCP_OPTION_END);
			return;
		}
		if (DHCP_OPTION_REQ_IPADDR == option_item_name && *ptr == option_item_name)
			sprint_hex_dump_specific_1(KERN_DEBUG, NULL, "GOT_opt", 32, ptr, 0, ptr[1] + 2, true, DGROUP_NONE, true);
		else {
			if (dbg > 1 && DHCP_OPTION_LEASE_TIME == option_item_name && *ptr == option_item_name)
				sprint_hex_dump_specific_2(KERN_DEBUG, NULL, "ELS_opt", 32, ptr, 0, ptr[1] + 2, true, DGROUP_NONE, true);
		}
		ptr += ptr[1] + 2;
	  }
	  printk("[over-track].\n");
  }
}
#endif


//#if DEF_MONITOR_RX | DEF_MONITOR_TX
typedef struct eth_class {
  int index;
  char *str;
} eth_class_t;

static const eth_class_t eclass[]= {
	{0, "tx-discover"},
	{1, "rx-offer"},
	{2, "tx-request"},
	{3, "rx-ACK"},
	{4, "ARP"},
	{5, "icmp"}, //insteaded
	{6, "rx-uni-tcp"},
	{7, "rx-uni-udp"},
	{8, "rx-uni-xxx"},
	{9, "rx-multi-unknow"},
	{10, "check-data"}, //default
	{11, "check-data"}, //rsrv
	{12, "check-data"}, //rsrv
	{13, "check-data"}, //rsrv
	{14, "check-data"}, //rsrv
	{15, "check-data"}, //rsrv
	{16, "tx-uni-tcp"},
	{17, "tx-uni-udp"},
	{18, "tx-uni-xxx"},
	{19, "tx-multi-unknow"},
	
	{20, "rx-icmp"},
	{21, "tx-icmp"},
};

static void sprint_hex_dump0(const char *level, struct net_device *ndev, const char *prefix_str,
		    int rowsize, const void *buf, int seg_start, size_t len, bool must, int dgroup, int useflg)
{
	if (useflg) {
		int si, se;
		int i, linelen, remaining = len;
		//const eth_class_t *ec = &eclass[10];

		si = seg_start;
		se = seg_start + len;
		for (i = si; i < se; i += rowsize) {
			unsigned char linebuf[(12 * 3) + (3 * 16) + 1 + 32]; //here!

			linelen = min(remaining, rowsize);
			remaining -= rowsize;
			do {
				const u8 *ptr = buf;
				int j, nb = 0;
				for (j=0; j< linelen && nb < sizeof(linebuf); j++) {
					if (j && !(j%8))
						nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
					if (((rowsize>>1)!=8) && !(j%(rowsize>>1)))
						nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");

					nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, "%02x ", *(ptr + i + j));
				}
			} while(0);
			
			if (prefix_str)
				printk("(dm9 %s) %.8x: %s\n", prefix_str, i, linebuf); //"%s", ec->str
			else
				printk("(dm9) %.8x: %s\n", i, linebuf); //"%s", ec->str
		}
	}
}

static void re_first_line(unsigned char *linebuf, int _bufsize, const u8 *ptr, int linelen) {
	int j, nb = 0;
	int i = 0;
	for (j=0; j< linelen && nb < _bufsize; j++) {
		if (j && !(j%8))
			nb += snprintf(linebuf+nb, _bufsize - nb, " ");
		if (j==16)
			nb += snprintf(linebuf+nb, _bufsize-nb, " ");
		
		if ((j+1)==6)
			nb += snprintf(linebuf+nb, _bufsize-nb, "%02x) ", *(ptr + i + j));
		else
			nb += snprintf(linebuf+nb, _bufsize-nb, "%02x ", *(ptr + i + j));
	}
}

static void sprint_hex_dump(struct sk_buff *skb, struct net_device *ndev, const char *prefix_str,
		    int rowsize, const void *buf, int seg_start, size_t len, bool must, int dgroup, int useflg)
{
	if (useflg) {
		const u8 *ptr = buf;
		int i, linelen, remaining = len;
		unsigned char linebuf[(12 * 3) + (3 * 16) + 1 + 32];
		int si, se, c;
		const eth_class_t *ec = &eclass[10]; //NULL; //&eclass[10];
		const char *ts = "CHK";

		if (IS_ARP)
			ec = &eclass[4];
		else if (IS_ICMP) {
			ec = (dgroup==DGROUP_RX)? &eclass[20]: (dgroup==DGROUP_TX)? &eclass[21]: &eclass[5];
			ts = (dgroup==DGROUP_RX)? "Rx": (dgroup==DGROUP_TX)? "TX": "CHK";
		} else {
			if (dgroup == DGROUP_RX || dgroup == DGROUP_RXII) {
				if (!memcmp(ptr, ndev->dev_addr, 6)) {
					if (IPBUF->proto == UIP_PROTO_TCP)
						ec = &eclass[6];
					else if (IPBUF->proto == UIP_PROTO_UDP)
						ec = &eclass[7];
					else
						ec = &eclass[8];
				} else
					ec = &eclass[9];
			}
			else if (dgroup == DGROUP_TX || dgroup == DGROUP_TXII) { /* TX, TXII */
				
				if (!(ptr[0] & 0x01)) {
					if (IPBUF->proto == UIP_PROTO_TCP)
						ec = &eclass[16];
					else if (IPBUF->proto == UIP_PROTO_UDP)
						ec = &eclass[17];
					else
						ec = &eclass[18];
				} else
					ec = &eclass[19];
			}
		}

		if (!IS_ICMP && !IS_ARP) {
		  sprint_hex_dump0(KERN_DEBUG, NULL, prefix_str, rowsize, buf, seg_start, len, must, DGROUP_NONE, useflg);
		  return;
		}

		c = 0;
		si = seg_start;
		se = seg_start + len;
		for (i = si; i < se; i += rowsize) {
			linelen = min(remaining, rowsize);
			remaining -= rowsize;
			do {
				const u8 *ptr = buf;
				int j, nb = 0;
				for (j=0; j< linelen && nb < sizeof(linebuf); j++) {
					if (j && !(j%8))
						nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
					if (((rowsize>>1)!=8) && !(j%(rowsize>>1)))
						nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, " ");
					
					if (IS_ARP && (i+j)>= 42 && *(ptr + i + j)==0) {
						c++;
						if (c <= 2)
							nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, ".. ");
						else
							nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, "   ");
					} else
						nb += snprintf(linebuf+nb, sizeof(linebuf)-nb, "%02x ", *(ptr + i + j));
				}
			} while(0);

			if (IS_ICMP) {
				if (IS_ICMP_REQUEST || IS_ICMP_REPLY) {
				  /*
				   * skb->len is unsigned int (32-bit OS, %u) and long unsigned int (%lu, 64-bit OS)
				   */
				   if (i==0) {
					re_first_line(linebuf, sizeof(linebuf), buf, linelen);
					printk("-dm9 %s\n", ec->str);
					//printk("-dm9 seqNO %s-%3d, id %d (%s: %s  len %u\n", ts, HTONS(ICMPBUF->seqno), HTONS(ICMPBUF->id), ec->str, linebuf,
					//	(unsigned int) (skb->len - (UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr))));
				   } else
				    printk(" dm9 %s\n", ec->str);
					//printk(" dm9 seqNO %s-%3d, id %d (%s: %s) len %u\n", ts, HTONS(ICMPBUF->seqno), HTONS(ICMPBUF->id), ec->str, linebuf,
					//	(unsigned int) (skb->len - (UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr))));
				} else if (!IS_ICMP_DST_UNREACHABLE) { //.misc
					printk(" dm9 %s (misc)\n", ec->str);
					//printk(" dm9 seqNO %s-%3d, id %d (%s: ) len %u\n", ts, HTONS(ICMPBUF->seqno), HTONS(ICMPBUF->id), ec->str,
					//	(unsigned int) (skb->len - (UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr)))); //.misc
				} else { //is (Destination Unreachable~)
					if (i==0) {
						switch (DST_UNREACHABLE_ICODE) {
						   //case 0:  //Network Unreachable
						   case ICMP_NET_UNREACH:
								do {
								  //void *pbuf = skb->data;
								  //struct uip_udpip_hdr *udpbuf = pbuf + UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr);
								  #if DEF_MONITOR_NRX | DEF_MONITOR_NTX
								   printk(" dm9 %s (Net Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
								  #else
								   printk(" dm9 %s (Net Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
								  #endif
								   //printk("-dm9 seqNO %s-%s [%s] %d.%d.%d.%d use icmp(tell) %d.%d.%d.%d, udp/tcp Net Unreachable, icode %d, your pkt_len %d\n",
									//ts, "   ", ec->str,
									//dm_ipaddr1(ICMPBUF->srcipaddr),dm_ipaddr2(ICMPBUF->srcipaddr),dm_ipaddr3(ICMPBUF->srcipaddr),dm_ipaddr4(ICMPBUF->srcipaddr),
									//dm_ipaddr1(ICMPBUF->destipaddr),dm_ipaddr2(ICMPBUF->destipaddr),dm_ipaddr3(ICMPBUF->destipaddr),dm_ipaddr4(ICMPBUF->destipaddr),
									//DST_UNREACHABLE_ICODE, (udpbuf->len[0] << 8) + udpbuf->len[1] + UIP_LLH_LEN);
								} while(0);
							   break;
						   case 1:  //Host Unreachable
								do {
								  //void *pbuf = skb->data;
								  //struct uip_udpip_hdr *udpbuf = pbuf + UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr);
								  #if DEF_MONITOR_NRX | DEF_MONITOR_NTX
								   printk(" dm9 %s (Host Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
								  #else
								   printk(" dm9 %s (Host Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
								  #endif
								   //printk("-dm9 seqNO %s-%s %s, %d.%d.%d.%d use tell %d.%d.%d.%d, udp/tcp Host Unreachable, icode %d, your pktLen %d\n",
									//ts, "x", ec->str,
									//dm_ipaddr1(ICMPBUF->srcipaddr),dm_ipaddr2(ICMPBUF->srcipaddr),dm_ipaddr3(ICMPBUF->srcipaddr),dm_ipaddr4(ICMPBUF->srcipaddr),
									//dm_ipaddr1(ICMPBUF->destipaddr),dm_ipaddr2(ICMPBUF->destipaddr),dm_ipaddr3(ICMPBUF->destipaddr),dm_ipaddr4(ICMPBUF->destipaddr),
									//DST_UNREACHABLE_ICODE, (udpbuf->len[0] << 8) + udpbuf->len[1] + UIP_LLH_LEN);
								} while(0);
							   break;
						   case 2:  //Protocol Unreachable
								do {
								  //void *pbuf = skb->data;
								  //struct uip_udpip_hdr *udpbuf = pbuf + UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr);
								  #if DEF_MONITOR_NRX | DEF_MONITOR_NTX
								   printk(" dm9 %s (Proto Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
								  #else
								   printk(" dm9 %s (Proto Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
								  #endif
								   //printk("-dm9 seqNO %s-%s [%s] %d.%d.%d.%d use icmp(tell) %d.%d.%d.%d, udp/tcp Proto Unreachable, icode %d, your pkt_len %d\n",
									//ts, "   ", ec->str,
									//dm_ipaddr1(ICMPBUF->srcipaddr),dm_ipaddr2(ICMPBUF->srcipaddr),dm_ipaddr3(ICMPBUF->srcipaddr),dm_ipaddr4(ICMPBUF->srcipaddr),
									//dm_ipaddr1(ICMPBUF->destipaddr),dm_ipaddr2(ICMPBUF->destipaddr),dm_ipaddr3(ICMPBUF->destipaddr),dm_ipaddr4(ICMPBUF->destipaddr),
									//DST_UNREACHABLE_ICODE, (udpbuf->len[0] << 8) + udpbuf->len[1] + UIP_LLH_LEN);
								} while(0);
							   break;
						   //case 3:  //Port Unreachable
						   case ICMP_PORT_UNREACH:
							  //if (i==0) {
							  if (cb.misc_total_tx.hex_show && cb.misc_total_rx.hex_show)
								sprint_hex_dump0(KERN_DEBUG, NULL, "PEEK_icmp_data", 32, skb->data, 0, skb->len, true, DGROUP_NONE, true); //TEMP

							  do {
								  //void *pbuf = skb->data;
								  //struct uip_udpip_hdr *udpbuf = pbuf + UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr);
								  
								  #if DEF_MONITOR_NRX | DEF_MONITOR_NTX
								  printk(" dm9 %s (your udp/tcp Port Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
								  #else
								  printk(" dm9 %s (your udp/tcp Port Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
								  #endif
								  //printk("-dm9 seqNO %s-%s(%s) %d.%d.%d.%d icmp(tell) %d.%d.%d.%d, (your udp/tcp Port Unreachable) your pkt_len %d\n",
									//ts, "   ", ec->str,
									//dm_ipaddr1(ICMPBUF->srcipaddr),dm_ipaddr2(ICMPBUF->srcipaddr),dm_ipaddr3(ICMPBUF->srcipaddr),dm_ipaddr4(ICMPBUF->srcipaddr),
									//dm_ipaddr1(ICMPBUF->destipaddr),dm_ipaddr2(ICMPBUF->destipaddr),dm_ipaddr3(ICMPBUF->destipaddr),dm_ipaddr4(ICMPBUF->destipaddr),
									//(udpbuf->len[0] << 8) + udpbuf->len[1] + UIP_LLH_LEN);
							  } while(0);
							  //}
							  break;
						   //case 4:  //Fragmentation Needed and DF set
						   case ICMP_FRAG_NEEDED:
						   case 5:  //Source Route Failed
						   case 6:  //Destination Network Unknow
						   case 7:  //Destination Host Unknow
						   case 8:  //Source Host Isolated
						   case 9:  //Communication with Destination Network Administratively Prohibited
						   case 10: //Communication with Destination Host Administratively Prohibited
						   case 11: //Network Unreachable for Type of Service
						   case 12: //Host Unreachable for Type of Service
						   default:
								do {
								  //void *pbuf = skb->data;
								  //struct uip_udpip_hdr *udpbuf = pbuf + UIP_LLH_LEN + sizeof(struct uip_icmpip_hdr);
								  #if DEF_MONITOR_NRX | DEF_MONITOR_NTX
								   printk(" dm9 %s (tcp/udp YYY Reason Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
								  #else
								   printk(" dm9 %s (tcp/udp YYY Reason Unreachable).rx [%d/%d] ~len~ %d\n", ec->str,
										  cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
								  #endif
								   //printk("-dm9 seqNO %s-%s [%s] %d.%d.%d.%d use icmp(tell) %d.%d.%d.%d, tcp/udp YYY Reason-Unreachable, icode %d, your pkt_len %d\n",
									//ts, "   ", ec->str,
									//dm_ipaddr1(ICMPBUF->srcipaddr),dm_ipaddr2(ICMPBUF->srcipaddr),dm_ipaddr3(ICMPBUF->srcipaddr),dm_ipaddr4(ICMPBUF->srcipaddr),
									//dm_ipaddr1(ICMPBUF->destipaddr),dm_ipaddr2(ICMPBUF->destipaddr),dm_ipaddr3(ICMPBUF->destipaddr),dm_ipaddr4(ICMPBUF->destipaddr),
									//DST_UNREACHABLE_ICODE, (udpbuf->len[0] << 8) + udpbuf->len[1] + UIP_LLH_LEN);
								} while(0);
							   break;
						}
					}
				}
			} else if (IS_ARP) {
				if (cdef.cons_t.dm_boot_msg.msg_print_arp_hex || must) { //'cdef'
					if (prefix_str)
						printk("(dm9 %s)%s%.8x: %s\n", prefix_str, ec->str, i, linebuf);
					else
						printk("(dm9)%s.%.8x: %s\n", ec->str, i, linebuf);
				}
			} 
			else
				printk("(dm9)%s%.8x: %s\n", ec->str, i, "please check!");
		}
	}
}
//#endif

//#if DEF_MONITOR_RX | DEF_MONITOR_TX
static u8 *get_parse_options(struct sk_buff *skb, u8 option_item_name, int dbg)
{
  void *buf = skb->data;
  u8 *ptr = &DHCPBUF->options[4];
  u8 *end = skb->data + skb->len;
  
  if (dbg)
	sprint_hex_dump0(KERN_DEBUG, NULL, "DBG_PARSE_OPTION", 32, ptr, 0, end-ptr, true, DGROUP_NONE, true);
  
  while (ptr < end) {
    if (*ptr == DHCP_OPTION_END) {
	  if (dbg) printk("DBG_PARSE_OPTION not found [%02x] at end\n", option_item_name);
      return NULL;
    }

    if (*ptr == option_item_name) {
      if (dbg) {
	    printk("DBG_PARSE_OPTION found [%02x]\n", option_item_name);
	    sprint_hex_dump0(KERN_DEBUG, NULL, "DBG_found", 32, ptr, 0, end-ptr, true, DGROUP_NONE, true);
	  }
      return ptr;
    }
    
    ptr += ptr[1] + 2;
    
    if (dbg) {
      printk("DBG_PARSE_OPTION next [%02x]\n", option_item_name);
      sprint_hex_dump0(KERN_DEBUG, NULL, "DBG_next", 32, ptr, 0, end-ptr, true, DGROUP_NONE, true);
    }
  }
  if (dbg) printk("DBG_PARSE_OPTION not found [%02x] at fail-track\n", option_item_name);
  return NULL;
}
//#endif
