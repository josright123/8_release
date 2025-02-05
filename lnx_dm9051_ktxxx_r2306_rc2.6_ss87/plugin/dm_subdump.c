 
static void arp_unicast_me(struct sk_buff *skb, struct net_device *ndev, int fullcheck)
{
	if (fullcheck)
	{
		const u8 *buf = skb->data;
		static u16 mela = 0xffff; //static u16 melac = 0xffff;
		u16 la;
		
		la = BUF->sipaddr[1] >> 8; //since, buf= skb->data
		if (cdef.cons_t.local_dip_rrrx) {
			if (cb.rx_total.val > cb.ram.rx_total_next) {
				if (la != mela)
				  printk("RRRX %d RX .%d Brd %d/%d --- dm9 no-reach now %d rst to 0 [ARP Rx uni from %d.%d.%d.%d] sklen %d\n",
					cb.rx_total.val,
					cb.arp_uni_me_rx.val, cb.arp_brdcst_rx.val, cb.arp_total_rx.val,
					cb.tmp.arp_we_trans_count, 
					BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8,
					skb->len);
				else
				  printk("RRRX %d RX .%d(ttx %d)\n", cb.rx_total.val, cb.arp_uni_me_rx.val, cb.tx_total.val);
			}
			sprint_hex_dump(skb, ndev, "RXUNI", 32, skb->data, 0, skb->len - 0, true, DGROUP_RX,
					cdef.cons_t.dm_boot_msg.msg_rx_unicastme_arp_hex);
			sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_RX);
		}
		DM_save_first(mela, melacxx, la); //[.e]

		if (cdef.cons_t.local_dip_rrrx_reach) {
			//#if GAP_REACH_RX_ENABLE
			if (cdef.cons_t.local_gap_reach_rrrx) {
				if (cb.rx_total.val > cb.ram.rx_total_next) 
				  roll_next_rx(cb.ram.rx_reachfor_disp, cb.rx_total.val);
			}
			//#endif
		}

		if (cb.tmp.flg_wait_uni_arp_finish)
		  printk("RX %d Brd %d/%d (here receive uni-ARP)---> dm9 (This clear 'flg_wait_uni_arp_finish' ) next-wait for could_touch, sklen %d\n",
				cb.arp_uni_me_rx.val, cb.arp_brdcst_rx.val, cb.arp_total_rx.val,
				skb->len);
	}
}

static void arp_broadcast_rx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;
	
	//if (IsBlackList(skb)) //too many so make it non-display
	//  return;
	
	/*
	 * [Check whether is from our LAN computer][This can use my IP. gotten from hdcpc]
	 */
	if ((BUF->sipaddr[0] & 0xff) == (cb.ram.detect_srv_ip[0] & 0xff) && /* 192 */
	    (BUF->sipaddr[0] >> 8) == (cb.ram.detect_srv_ip[0] >> 8) && /* 168 */
	    (BUF->sipaddr[1] & 0xff) == (cb.ram.detect_srv_ip[1] & 0xff)) /* 1 */
	  ;
	else {
	  printk("RRRX %d RX %d Brd %d/%d ---> dm9 [broadcast] sklen %d\n",
		cb.rx_total.val, cb.arp_uni_me_rx.val, cb.arp_brdcst_rx.val, cb.arp_total_rx.val,
		skb->len);
	  sprint_hex_dump(skb, ndev, "RXBrd", 32, skb->data, 0, 6, true, DGROUP_RX,
		cdef.cons_t.dm_boot_msg.msg_rx_multicast_arp_hex);
	  //_sprint_arp_field_parse(skb->data, 0, ...); (discard previous 16 bytes)
	  //_sprint_arp_field_parse(skb->data, SEG_QUAN, ...);
	  sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_RXII);
	}
}

#if 0
static void arp_unicast_couldtouch_tx(struct net_device *ndev, struct sk_buff *skb)
{
	cb.ram.cas2++;
	cb.tmp.arp_never_cas2_tx_uni++;

	printk("\n"); //case2
	printk("TX %d/%d ---> dm9 [no-reach then %d, %d ARP.%d %d [unicast here, could touch, depenp-on view following..] sklen %d\n",
		cb.tmp.arp_never_cas2_tx_uni, cb.arp_total_tx.val,
		cb.tmp.arp_we_trans_count, cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, skb->len);
	sprint_hex_dump(skb, ndev, "CAS2 ", 32, skb->data, 0, skb->len - 0, true, DGROUP_TX, 1); // print HEX

#if DEF_BRDCST_IT_TX
	sprint_DA_field(skb->data);
	Write_ARP_broadcast(skb->data);
	sprint_DA_field(skb->data);
#else
	//printk("dm9 [TX Write_ARP_unicast here should be, But disabled to brdcst.]\n");
#endif
}
#endif

static void arp_unicast_safty_tx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;
	static u16 unila = 0xffff;
	u16 la = BUF->dipaddr[1] >> 8; //since, buf= skb->data
	//int local_ttx = 0;
	//int local_ttx_reach = 1; 

	cb.ram.cas3++;
	cb.tmp.arp_safty_cas3_tx_uni++;

	if (cdef.cons_t.local_dip_ttx) {
		//static u16 unilac = 0xffff;
		//printk("\n"); //case3
		if (la != unila)
		{
			DM_save_first(unila, unilac, la);
			if (cb.tx_total.val > cb.ram.tx_total_next)
			  printk("TTX %2d TX .%d Brd %d/%d --- dm9 %d %d ARP.%d, no-reach %d (safty Tx uni to %d.%d.%d.%d) sklen %d\n",
				cb.tx_total.val,
				cb.tmp.arp_safty_cas3_tx_uni, cb.tmp.arp_wait_cas1_tx_brdcst + cb.tmp.arp_safty_cas1_tx_brdcst,
				cb.arp_total_tx.val, cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, cb.tmp.arp_we_trans_count,
				BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8,
				skb->len);
		}
		else
		{
			if (cb.tx_total.val > cb.ram.tx_total_next)
			  printk("TTX %2d TX .%d\n", cb.tx_total.val, cb.tmp.arp_safty_cas3_tx_uni);
		}
		sprint_hex_dump(skb, ndev, "CAS3 ", 32, skb->data, 0, skb->len - 0, true, DGROUP_TX,
			cdef.cons_t.dm_boot_msg.msg_tx_uni_arp_hex_msg); // print HEX
		//sprint_arp_field_parse(skb->data, 0, SEG_QUAN, DGROUP_TX);
		sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_TX);
	}
}

#if 0
static void arp_broadcast_couldtouch_tx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;
	cb.ram.cas1++;
	cb.tmp.arp_wait_cas1_tx_brdcst++;
	if (cdef.cons_t.dm_boot_msg.msg_print_arp_brdcst_case1_tx) {
		printk("\n"); //case1
		printk("dm9 .%d %d %d/%d --- ARP.%d %d %d [tx brd-cast, on 'flg_wait_uni_arp_finish', could touch here] sklen %d\n",
			cb.tmp.arp_wait_cas1_tx_brdcst, cb.tmp.arp_safty_cas1_tx_brdcst, cb.tmp.arp_safty_cas3_tx_uni, cb.arp_total_tx.val,
			cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, skb->len); //, cb.tmp.arp_we_trans_count
	}
	else {
		printk("dm9 .%d %d %d/%d --- ARP.%d %d %d (tx brd-cast, on 'flg_wait_uni_arp_finish', could touch here) sklen %d\n",
			cb.tmp.arp_wait_cas1_tx_brdcst, cb.tmp.arp_safty_cas1_tx_brdcst, cb.tmp.arp_safty_cas3_tx_uni, cb.arp_total_tx.val,
			cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, skb->len);
		sprint_hex_dump(skb, ndev, "CASW", 32, skb->data, 0, 6, true, DGROUP_TX, 1); // print 6HEX

		printk("-dm9 %d.%d.%d.%d (dst %d.%d.%d.%d)(total TTX/RRRX = %d/%d)\n",
			BUF->sipaddr[0] & 0xff, BUF->sipaddr[0] >> 8, BUF->sipaddr[1] & 0xff, BUF->sipaddr[1] >> 8,
			BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8,
			cb.tx_total.val, cb.rx_total.val);
	}
}

static void arp_segment_dump_tx(struct net_device *ndev, struct sk_buff *skb)
{
	if (IsBroadcast(skb->data))
		sprint_hex_dump(skb, ndev, "ARP", 32,
				skb->data, 0, skb->len - 0, false, DGROUP_TX, 1);
	else {
		sprint_hex_dump(skb, ndev, "ARP", 16,
					skb->data, 0, SEG_QUAN, false, DGROUP_TX, 1);
		sprint_hex_dump(skb, ndev, "ARP", 16,
					skb->data, SEG_QUAN, skb->len - SEG_QUAN, false, DGROUP_TX, 1);
	}
	if (cdef.cons_t.dm_boot_msg.msg_print_arp_parse_my_ip) { //to know me ip declare stages
		//sprint_arp_field_parse(skb->data, 0, SEG_QUAN, DGROUP_TX); (parse discard previous 16 bytes of a pkt)
		sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_TX);
	}
}

static void arp_broadcast_safty_tx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;
	static u16 sla = 0xffff;
	/* static u16 slac = 0xffff; */
	u16 la = BUF->dipaddr[1] >> 8; //since, buf= skb->data
	char *ad_str;

	cb.ram.cas1++;
	cb.tmp.arp_safty_cas1_tx_brdcst++;

	ad_str = "to";
	if ((BUF->dipaddr[0] | BUF->dipaddr[1]) &&
	    ((BUF->dipaddr[0] & 0xff) == cb.ram.dhcp_req_ip[0]) &&
	    ((BUF->dipaddr[0] >> 8) == cb.ram.dhcp_req_ip[1]) && 
	    ((BUF->dipaddr[1] & 0xff) == cb.ram.dhcp_req_ip[2]) &&
	    ((BUF->dipaddr[1] >> 8) == cb.ram.dhcp_req_ip[3]))
	  ad_str = "self";

	if (la != sla) {
	  DM_save_first(sla, slac, la);
	  printk("TTX %2d TX %d .Brd %d/%d --- dm9 safty brdcst ARP.%d %d %d (%s %d.%d.%d.%d) no-reach %d sklen %d\n", cb.tx_total.val,
		cb.tmp.arp_safty_cas3_tx_uni, cb.tmp.arp_wait_cas1_tx_brdcst + cb.tmp.arp_safty_cas1_tx_brdcst, cb.arp_total_tx.val,
		cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, ad_str,
		BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8,
		cb.tmp.arp_we_trans_count, skb->len);
	} else {
	  printk("TTX %2d TX %d .Brd %d/%d --- dm9 safty brdcst ARP.%d %d %d (%s %d.%d.%d.%d)\n", cb.tx_total.val,
		cb.tmp.arp_safty_cas3_tx_uni, cb.tmp.arp_wait_cas1_tx_brdcst + cb.tmp.arp_safty_cas1_tx_brdcst, cb.arp_total_tx.val,
		cb.ram.cas1, cb.ram.cas2, cb.ram.cas3, ad_str,
		BUF->dipaddr[0] & 0xff, BUF->dipaddr[0] >> 8, BUF->dipaddr[1] & 0xff, BUF->dipaddr[1] >> 8);
	}
	sprint_hex_dump(skb, ndev, "CASS", 32, skb->data, 0, 6, true, DGROUP_TX,
		cdef.cons_t.dm_boot_msg.flag_tx_brdcst_arp_hex_bootx); // print 6HEX
	//sprint_arp_field_parse(skb->data, 0, SEG_QUAN, DGROUP_TX); (parse discard previous 16 bytes of a pkt)
	sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_TX);

	if (DM_ARP_SELF_IP && cdef.cons_t.dm_boot_msg.flag_tx_self_ip_summ_boote) { //=DM_arp_self_IP(skb->data)
	 //( arp_self_ip = 1;) ..not need this
	  printk(" tx misc %d [dhcptx %d, arptx %d, TTX %d]\n", cb.misc_total_tx.val, cb.tmp.dhcp_total_tx,
		cb.arp_total_tx.val, cb.tx_total.val);
	}

}
#endif

static void arp_counting_reach_disp_tx(struct net_device *ndev, struct sk_buff *skb)
{
	if (cb.tmp.arp_we_trans_count > 2) {
		size_t x;
		printk("TX %d Brd %d/%d (more safty uni-ARP)---> dm9 (This reach 'flg_wait_uni_arp_finish' ) next could-touch, sklen %d\n",
			cb.tmp.arp_safty_cas3_tx_uni, cb.tmp.arp_wait_cas1_tx_brdcst + cb.tmp.arp_safty_cas1_tx_brdcst, cb.arp_total_tx.val,
			skb->len);
		//sprint_hex_dump(skb, ndev, "CAS0", 32, skb->data, 0, skb->len - 0, true, DGROUP_TX, 1); // "CAS-no_unicast_arp_enter"
		x = (size_t) min(32, (int) skb->len);
		sprint_hex_dump(skb, ndev, "CAS0", 32, skb->data, 0, x, true, DGROUP_TX, 1); // a "CAS-no_unicast_arp_enter"
		sprint_arp_field_parse(skb->data, SEG_QUAN, skb->len - SEG_QUAN, DGROUP_TX);
	}
}
