### Linux DM9051 Driver r2502_v3.9 driver information
<br>

**Updation List:**

 - A: For variated individual kernal version
    - Tested in DM9051_KERNEL_5_10/ DM9051_KERNEL_5_15/ DM9051_KERNEL_6_1/ DM9051_KERNEL_6_6
 
	An example:
	
		#define KERNEL_BUILD_CONF DM9051_KERNEL_6_6
 
 - B: Compatiable with lower speed connection
    - Longer .tx_timeout_us make it adapt to 10Mbps lower speed networking
	
 - C: Enhance rxb process
 
	- Make sure to check rxb exactly 
	- Introduce the usage of SCAN_BH(dw)


 * D: Usage of struct driver_config
 
	An example:
	
	- Device= rpi5, 
	- Run with 
		- Interrupt mode (INT Clock out) 
		- Encryption enable, enable
		- 16-bit mode
		- TX continue mode off
		- Checksum offload disable
		- Alignment mode enable
		
	```c
	const struct driver_config confdata = {
		.release_version = "lnx_dm9051_kt6631_r2502_v3.9",
		.interrupt = MODE_INTERRUPT_CLKOUT,
		.mid = MODE_A,
		.mod = {
			{
				.test_info = "Test in rpi5 bcm2712",
				.encpt_mode = FORCE_BUS_ENCPT_FAB_ON,
				.encpt_pad = 0x00,
				.skb_wb_mode = SKB_WB_ON,
				.tx_mode = FORCE_TX_CONTI_OFF,
				.checksuming = DEFAULT_CHECKSUM_ON,
				.align = {
					.burst_mode = BURST_MODE_ALIGN,
					.tx_blk = 32,
					.rx_blk = 64
				}
			},
			// Additional configurations for other modes...
		}
	};
	```
<br>

**Brief Functions List:**

(1) Interrupt support
   - Interrupt 
   - Interrupt Clk out
   - Polling

(2) Alignment mode

   - TX burst alignment
   - RX burst alignment
   - TX/RX burst full mode

     **Note:** work around to dm9051_write_mem/dm9051_read_mem<br> for 
	  solve below issue: 
	  
			"dw_axi_dmac_platform 1f0018000.dma: invalid buffer alignment.... "
			
     **Example:** align_config 
	  
			.burst_mode = BURST_MODE_ALIGN, .tx_blk =  32, .rx_blk =64


(3) Enhance Random MAC address
- Provide 00:60:6e:xx:xx;xx RANDOM

(4) RX Schedule delay configurable
- For polling mode
- Program control, **POLL_OPERATE_NUM** determine the usage of the iterated different delay times.

(5) misc

- Monitor rxb, enable by **FORCE_MONITOR_RXB**
- Monitor rx packets, enable by **FORCE_MONITOR_RX_COUNT**
- Monitor tx timeout condition, enable by **FORCE_MONITOR_TX_TIMEOUT**

