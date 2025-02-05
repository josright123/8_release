
static void arp_counting_to_safty_tx(int to_must_safty)
{
	//rst 0
	if (cb.tmp.arp_we_trans_count > 2 || to_must_safty) {
		cb.tmp.arp_we_trans_count = 0;
		cb.tmp.flg_wait_uni_arp_finish = 1; //=(cb.tmp.flg_has_arp_unicast_enter = 0;) //back-to-search-a-one.
	}
}

static int pprint_icmp(struct sk_buff *skb, struct net_device *ndev, int dgroup, int msg_print_n, bool must)
{
	const void *buf = skb->data;

	if (msg_print_n || must)
	{
		sprint_hex_dump(skb, ndev, NULL, 16, skb->data, 0, SEG_QUAN -2, false, dgroup, 1);
	}
	if ( (!IS_ICMP_REQUEST &&
		!IS_ICMP_REPLY) || must)
	{
		return msg_print_n;
	}
	if (msg_print_n)
	{
		return msg_print_n - 1;
	}
	return 0;
}

static void arp_rx(struct sk_buff *skb, struct net_device *ndev, int fullcheck)
{
	if (IsUnicastMe(skb->data, ndev->dev_addr)) {
		//[start]
		cb.arp_uni_me_rx.val++;

		arp_unicast_me(skb, ndev, fullcheck);

		//[end]
		arp_counting_to_safty_tx(1); //must rst 0
	} else {
		cb.arp_brdcst_rx.val++;
		if (fullcheck){
			if (IsBlackList(skb)) 
				; //too many so make it non-display
			else
				arp_broadcast_rx(ndev, skb);
		}
		/* else {
		 *		is no need blacklist detect processing...
		 * }
		 */
	}
}

static void arp_tx(struct sk_buff *skb, struct net_device *ndev, int fullcheck)
{
	if (cb.tmp.flg_wait_uni_arp_finish == 0) {
		if (!IsBroadcast(skb->data)) {
			cb.arp_uni_out_tx.val++;
			if (cb.arp_uni_out_tx.enable) {
				cb.tmp.arp_we_trans_count++; //inc 1
				arp_unicast_safty_tx(ndev, skb);
				if (fullcheck) roll_next_tx(cb.ram.tx_reachfor_disp, cb.tx_total.val);
			}
		}
		
		if (fullcheck) {
	#if 0
			/* seg-dump*/
			arp_segment_dump_tx(ndev, skb);
	#endif
			//[Transmiting...2/2]
			if (cb.tmp.arp_we_trans_count > 2) { //counting reach expire
				arp_counting_reach_disp_tx(ndev, skb); //counting reach expire
				arp_counting_to_safty_tx(0); //counting reach expire, and rst 0
			}
		} else {
			//part of= [Transmiting...2/2]
			if (cb.tmp.arp_we_trans_count > 2) //counting reach expire
				arp_counting_to_safty_tx(0); //counting reach expire, and rst 0
		}
	}

	if (fullcheck) {
#if 0
		if (cb.tmp.flg_wait_uni_arp_finish) { //=(cb.tmp.flg_has_arp_unicast_enter==0)
			if (IsBroadcast(skb->data)) 
				arp_broadcast_couldtouch_tx(ndev, skb);
			else {
				arp_unicast_couldtouch_tx(ndev, skb);
				// roll_next_tx(cb.ram.tx_reachfor_disp, cb.tx_total.val); //= Need here or not, is both OK.
			}
			/* seg-dump*/
			arp_segment_dump_tx(ndev, skb);
		}
		else {
			//[Transmiting...1/2]
			if (IsBroadcast(skb->data)) 
				arp_broadcast_safty_tx(ndev, skb);
			else {
				cb.tmp.arp_we_trans_count++; //inc 1
				arp_unicast_safty_tx(ndev, skb);
				roll_next_tx(cb.ram.tx_reachfor_disp, cb.tx_total.val);
			}
			/* seg-dump*/
			arp_segment_dump_tx(ndev, skb);
			//[Transmiting...2/2]
			arp_counting_reach_disp_tx(ndev, skb);
			arp_counting_to_safty_tx(0); //counting reach expire, and rst 0
		}
#endif
	}
	else {
#if 0
		if (cb.tmp.flg_wait_uni_arp_finish) {
			if (!IsBroadcast(skb->data)) {
				arp_unicast_couldtouch_tx(ndev, skb);
				//roll_next_tx(cb.ram.tx_reachfor_disp, cb.tx_total.val); //= Need here or not, is both OK.
			}
		} else {
			//[Transmiting...1/2]
			if (!IsBroadcast(skb->data)) {
				cb.tmp.arp_we_trans_count++; //inc 1
				arp_unicast_safty_tx(ndev, skb);
				//roll_next_tx(cb.ram.tx_reachfor_disp, cb.tx_total.val);
			}
			//part of= [Transmiting...2/2]
			arp_counting_to_safty_tx(0); //counting reach expire, and rst 0
		}
#endif
	}
}

static void dhcp_rx(struct sk_buff *skb, int fullcheck)
{
	if (fullcheck) {
		const void *buf = skb->data;
		static u16 mrx = 0xffff;

		if (IS_DHCPRX_OFFER) { //IsDHCPRx_OFFER(skb->data)
		#if 0
			printk("RRRX %2d RX. %d -- dhcp offer\n", cb.rx_total.val, cb.dhcp_total_rx.val);
		#endif
			printk("dhcp offer");
		}
		if (IS_DHCPRX_ACK) { //IsDHCPRx_ACK(skb->data)
		    u8 *ps;
	//static u8 srvaddr[4];
		    //memset(srvaddr, 0, 4);

		    ps = get_parse_options(skb, DHCP_OPTION_SERVER_ID, 0);
		    
		    if (ps) {
		      if (ps[1] == 4) {
				memcpy(cb.ram.dhcp_rx_srvaddr, &ps[2], 4); //memcpy(srvaddr, &ps[2], 4);
				server_ip_handler(SIP_TYPE_DHCPRX_OPT_SRV_ID, ps[2] | (ps[3] << 8), ps[4] | (ps[5] << 8));
		      } else
				printk("Dm9 RRRX Not Know DHCP src IP address ...\n");
			
		      /*ps++;
		      if (*ps++ == 4) {
				//sprint_hex_dump0(KERN_DEBUG, NULL, "DEBUG", 32, ps, 0, 16, true, DGROUP_NONE, 1);
				memcpy(srvaddr, ps, 4);
			  } else
				printk("Dm9 RRRX Not Know DHCP src IP address ...\n");*/
		    }
		    
		#if 0
		    print_parse_options(skb, DHCP_OPTION_LEASE_TIME, 2);
		#endif
		
		    if (!ps || cb.ram.dhcp_rx_srvaddr[0] == mrx) {
		      mrx = 0xffff; // then, can use normally
		      printk("RRRX %2d RX. %d -- dhcp ack\n",  cb.rx_total.val, cb.dhcp_total_rx.val);
		    } else {
		      const u8 *buf = skb->data;
			  
		      DM_save_first(mrx, inoused, cb.ram.dhcp_rx_srvaddr[0]);
		      printk("RRRX %2d RX. %d -- dhcp ack (from srv %d.%d.%d.%d) - (iph srcip %d.%d.%d.%d dhcppayload yourip %d.%d.%d.%d)\n",
				cb.rx_total.val, cb.dhcp_total_rx.val,
				cb.ram.dhcp_rx_srvaddr[0], cb.ram.dhcp_rx_srvaddr[1], cb.ram.dhcp_rx_srvaddr[2], cb.ram.dhcp_rx_srvaddr[3],
				IPBUF->srcipaddr[0] & 0xff, IPBUF->srcipaddr[0] >> 8, IPBUF->srcipaddr[1] & 0xff, IPBUF->srcipaddr[1] >> 8,
				DHCPBUF->yiaddr[0], DHCPBUF->yiaddr[1], DHCPBUF->yiaddr[2], DHCPBUF->yiaddr[3]);
		    }
		    printk("dhcp ack");
		}
	}
}

static void dhcp_tx(struct sk_buff *skb)
{
	const void *buf = skb->data;
	static u16 sf_destipaddr[2] = { 0, 0};
	//static u16 mtx = 0xffff;

	if (IS_DHCPTX_DISCOVER)
	#if 0
	  printk("TTX %2d TX. %d -- dhcp discover\n", cb.tx_total.val, cb.tx_total.val);
	#endif
	  printk("dhcp discover\n");
	
	/* 
	 * if already know ip in prev. stage, direct send 'request' for my client ip
	 */
	if (IS_DHCPTX_REQUEST) {
	  u8 *ps, *p;
	  //.char zReqip[26];
	  
	  //u8 srvaddr[4], 
	  //memset(srvaddr, 0, 4);
	  //u8 ipaddr[4];
	  //memset(ipaddr, 0, 4);

	  ps = get_parse_options(skb, DHCP_OPTION_SERVER_ID, 0); //=54
	  p = get_parse_options(skb, DHCP_OPTION_REQ_IPADDR, 0); //=50
	  if (ps) {
	    if (ps[1] == 4) {
	      memcpy(cb.ram.dhcp_tx_srvaddr, &ps[2], 4); //=srvaddr
	      server_ip_handler(SIP_TYPE_DHCPTX, ps[2] | (ps[3] << 8), ps[4] | (ps[5] << 8));
	    } else
	      printk("Dm9 TTX Not Know DHCP src IP address ...\n");
	  }

	  //.snprintf(zReqip, 26, "req ip (%d.%d.%d.%d)", cb.ram.dhcp_req_ip[0],cb.ram.dhcp_req_ip[1],cb.ram.dhcp_req_ip[2],cb.ram.dhcp_req_ip[3]);
	  if (p) {
	    if (p[1] == 4) {
		  //memcpy(ipaddr, &p[2], 4);
	      memcpy(cb.ram.dhcp_req_ip, &p[2], 4);
	      //.snprintf(zReqip, 26, "req ip %d.%d.%d.%d", cb.ram.dhcp_req_ip[0],cb.ram.dhcp_req_ip[1],cb.ram.dhcp_req_ip[2],cb.ram.dhcp_req_ip[3]);
	    } else
	      printk("Dm9 TTX Not Know DHCP request IP address ...\n");
	  }

	  //#if DEF_MONITOR_TX | DEF_MONITOR_RX
	  //[IPBUF->srcipaddr, or IPBUF->destipaddr]
	  if (IPBUF->destipaddr[0] != sf_destipaddr[0]
			|| IPBUF->destipaddr[1] != sf_destipaddr[1]) {
	    
		    //DM_save_first(mtx, inoused, cb.ram.dhcp_tx_srvaddr[0]);
		    sf_destipaddr[0] = IPBUF->destipaddr[0];
		    sf_destipaddr[1] = IPBUF->destipaddr[1];
		    if (sf_destipaddr[0] != 0xffff && sf_destipaddr[1] != 0xffff) {
				cb.ram.dhcp_tx_srvaddr[0] = sf_destipaddr[0] & 0xff;
				cb.ram.dhcp_tx_srvaddr[1] = sf_destipaddr[0] >> 8;
				cb.ram.dhcp_tx_srvaddr[2] = sf_destipaddr[1] & 0xff;
				cb.ram.dhcp_tx_srvaddr[3] = sf_destipaddr[1] >> 8;
				server_ip_handler(SIP_TYPE_DHCPTX_IP_DEST, sf_destipaddr[0], sf_destipaddr[1]);
		    }
		    
		    //.if (ps)
		    //.  printk("TTX %2d TX. %2d -- dhcp request (to srv %d.%d.%d.%d) - %s\n", cb.tx_total.val, cb.tx_total.val,
		    //.   cb.ram.dhcp_tx_srvaddr[0], cb.ram.dhcp_tx_srvaddr[1], cb.ram.dhcp_tx_srvaddr[2], cb.ram.dhcp_tx_srvaddr[3],
		    //.   zReqip);

		    if (!ps)
		    {
		      if (cb.dhcp_total_tx.hex_show)
				sprint_hex_dump0(KERN_DEBUG, NULL, "DBG", 32, buf, 0, skb->len, true, DGROUP_NONE, true);
			  #if 0
		      print_parse_options(skb, DHCP_OPTION_REQ_IPADDR, 1);
		      printk("TTX %2d TX. %2d -- dhcp request (to mac %02x:%02x:%02x:%02x:%02x:%02x ip %d.%d.%d.%d) - %s\n", cb.tx_total.val, cb.tx_total.val,
		       BUF->ethhdr.dest.addr[0],BUF->ethhdr.dest.addr[1],BUF->ethhdr.dest.addr[2],
		       BUF->ethhdr.dest.addr[3],BUF->ethhdr.dest.addr[4],BUF->ethhdr.dest.addr[5],
		       dm_ipaddr1(IPBUF->destipaddr),dm_ipaddr2(IPBUF->destipaddr),dm_ipaddr3(IPBUF->destipaddr),dm_ipaddr4(IPBUF->destipaddr),
		       zReqip);
			  #endif
			printk("dhcp request\n");
		    }
	   }
	   else
	   {
	   
		    if (ps)
				printk("TTX %2d TX. %2d -- dhcp request (to srv %d.%d.%d.%d)\n", cb.tx_total.val, cb.tx_total.val,
				   cb.ram.dhcp_tx_srvaddr[0], cb.ram.dhcp_tx_srvaddr[1], cb.ram.dhcp_tx_srvaddr[2], cb.ram.dhcp_tx_srvaddr[3]);
		    else {
				if (cb.dhcp_total_tx.hex_show)
				  sprint_hex_dump0(KERN_DEBUG, NULL, "DBGs", 32, buf, 0, skb->len, true, DGROUP_NONE, true);
			#if 0
				print_parse_options(skb, DHCP_OPTION_REQ_IPADDR, 1);
				printk("TTX %2d TX. %2d -- dhcp request (to mac %02x:%02x:%02x:%02x:%02x:%02x ip %d.%d.%d.%d)\n", cb.tx_total.val, cb.tx_total.val,
				   BUF->ethhdr.dest.addr[0],BUF->ethhdr.dest.addr[1],BUF->ethhdr.dest.addr[2],
				   BUF->ethhdr.dest.addr[3],BUF->ethhdr.dest.addr[4],BUF->ethhdr.dest.addr[5],
				   dm_ipaddr1(IPBUF->destipaddr),dm_ipaddr2(IPBUF->destipaddr),dm_ipaddr3(IPBUF->destipaddr),dm_ipaddr4(IPBUF->destipaddr));
			#endif
				printk("dhcp request\n");
		    }
	   }
	   //#endif
	}
}

#if CODE_RX_REQUEST_SRC_CODE
static int dm_rx_gate(struct sk_buff *skb) {
	const u8 *buf = skb->data;

	if (IS_ARP)
		return 0;
	else if (IS_DHCPRX)
		return 0;
	else {
		if (IS_ICMP)
			return 1;
		return 0;
	}
}
#endif

static void dm_check_rx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;

	if (IS_ARP) {
		cb.arp_total_rx.val++;
		arp_rx(skb, ndev, DEF_MONITOR_NRX);
	}
	else if (IS_DHCPRX) {
		cb.dhcp_total_rx.val++;
		dhcp_rx(skb, DEF_MONITOR_NRX);
		
		#if 0
		//#if DEF_MONITOR_NRX | DEF_MONITOR_NTX
		//printk("[temp] DHCPRX.rx [%d/%d] ~len~ %d\n", cb.rx_total.val, DEF_MONITOR_NRX, skb->len);
		//#else
		//printk("[temp] DHCPRX.rx [%d/%d] ~len~ %d\n", cb.rx_total.val, _modef.def_monitor_nrx, skb->len);
		//#endif
		#endif

	#if 0 //[debug]
		//printk("rx-debug srcport == DHCPC_SERVER_PORT, destport == DHCPC_CLIENT_PORT : %x %x, %x %x\n", 
		//	HTONS(UDPBUF->srcport), DHCPC_SERVER_PORT,
		//	HTONS(UDPBUF->destport), DHCPC_CLIENT_PORT);
	#endif
	#if 0
		printk("dhcp src SRVPORT, dst CLNTPORT: %x, %x\n", 
			DHCPC_SERVER_PORT, DHCPC_CLIENT_PORT);
	#endif
		if (IS_DHCPRX_OFFER)
			printk("dhcp offer\n");
		else if (IS_DHCPRX_ACK)
			printk("dhcp ack\n");
		else
			printk("dhcp rx-packet\n");
	} else {
		cb.misc_total_rx.val++;
		if (IS_ICMP) {
			if (cb.tmp.msg_print_icmp) {
				cb.tmp.last_print_icmp = cb.tmp.msg_print_icmp;
				cb.tmp.msg_print_icmp = pprint_icmp(skb, ndev, DGROUP_RX,
							    cb.tmp.last_print_icmp,
							    false);
				#if 0
				if (cb.tmp.last_print_icmp)
					printk("-dm9 [rrx-icmp seqNO-%3d, id %d (%d %d) msg check now %3d from last %3d]\n",
						HTONS(ICMPBUF->seqno), HTONS(ICMPBUF->id), ICMPBUF->type, ICMPBUF->icode,
						cb.tmp.msg_print_icmp, cb.tmp.last_print_icmp);
				#endif
			}
		}
	}
}

static void protocol_tx(struct sk_buff *skb, struct net_device *ndev, int fullcheck)
{
	const u8 *buf = skb->data; 

	if (IS_ICMP) {
		if (cb.tmp.msg_print_icmp) {
			cb.tmp.last_print_icmp = cb.tmp.msg_print_icmp;
			cb.tmp.msg_print_icmp = pprint_icmp(skb, ndev, DGROUP_TX,
							    cb.tmp.last_print_icmp, false);
			if (cb.tmp.last_print_icmp)
				printk("[ttx-icmp seqNO-%3d, id %d (%d %d) msg check now %3d from last %3d]\n",
					HTONS(ICMPBUF->seqno), HTONS(ICMPBUF->id), ICMPBUF->type, ICMPBUF->icode,
					cb.tmp.msg_print_icmp, cb.tmp.last_print_icmp);
		}
		return;
	}

	if (fullcheck) {
		if (IsMulticast(skb->data)) {
			if (IPBUF->proto == UIP_PROTO_UDP) {
				if (cdef.cons_t.msg_print_tx_multi_udp)
					printk("TTX %2d dm9 tx misc %3d multicast udp\n", cb.tx_total.val, cb.misc_total_tx.val);
			} else if (IPBUF->proto == UIP_PROTO_IGMP_JJ)
				printk("TTX %2d dm9 tx misc %3d multicast igmp\n", cb.tx_total.val, cb.misc_total_tx.val);
			else {
				printk("TTX %2d dm9 tx misc %3d multicast-xx-else\n", cb.tx_total.val, cb.misc_total_tx.val);
				sprint_hex_dump(skb, ndev, "MISC", 32, skb->data, 0, 14+20+20, true, DGROUP_TX, 1);
			}
		} else {
			if (IPBUF->proto == UIP_PROTO_TCP) {
				//ec = &eclass[16];
				printk("TTX %2d dm9 tx misc %3d uni-tcp\n", cb.tx_total.val, cb.misc_total_tx.val);
				sprint_hex_dump(skb, ndev, "MISC", 32, skb->data, 0, 14+20+20, true, DGROUP_TX, cdef.cons_t.dm_boot_msg.msg_tx_uni_tcp_hex);
			} else if (IPBUF->proto == UIP_PROTO_UDP) {
				if (cdef.cons_t.msg_print_tx_uni_udp) {
					//ec = &eclass[17];
					printk("TTX %2d dm9 tx misc %3d uni-udp\n", cb.tx_total.val, cb.misc_total_tx.val);
					sprint_hex_dump(skb, ndev, "MISC", 32, skb->data, 0, 14+20+20, true, DGROUP_TX, cdef.cons_t.dm_boot_msg.msg_tx_uni_udp_hex);
				}
			} else {
				//ec = &eclass[18];
				printk("TTX %2d dm9 tx misc %3d uni-xx-else\n", cb.tx_total.val, cb.misc_total_tx.val);
				sprint_hex_dump(skb, ndev, "MISC", 32, skb->data, 0, 14+20+20, true, DGROUP_TX, cdef.cons_t.dm_boot_msg.msg_tx_uni_xxx_hex);
			}
		}
	}
}

static int dm_tx_gate(struct sk_buff *skb) {
	const u8 *buf = skb->data;
	if (IS_ARP)
		return 0;
	else if (IS_DHCPTX)
		return 1;
	else
		return 0;
}
static void dm_check_tx(struct net_device *ndev, struct sk_buff *skb)
{
	const u8 *buf = skb->data;

	if (IS_ARP) {
		cb.arp_total_tx.val++;
		arp_tx(skb, ndev, DEF_MONITOR_NTX); //1 or 0
	}
	else if (IS_DHCPTX) {
		cb.dhcp_total_tx.val++; //'~tx_total'
		dhcp_tx(skb);
	} else {
		cb.misc_total_tx.val++;
		protocol_tx(skb, ndev, DEF_MONITOR_NTX);
	}
}
