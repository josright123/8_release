# DM9051 Linux Driver Development Guide

## Project Overview

This repository contains a Linux kernel driver for the **Davicom DM9051A** SPI-based Fast Ethernet controller with IEEE 1588 PTP (Precision Time Protocol) hardware timestamping support. The driver is split across multiple source files and supports Linux kernels 6.1 and 6.6+.

**Primary source**: `dm9051_linux_driver/` directory contains the active driver implementation.

## Architecture Components

### Core Driver Files
- **`dm9051_main.c`**: Main driver logic - SPI communication, network device operations, interrupt/polling modes, packet TX/RX
- **`dm9051_ptp.c`**: PTP/IEEE 1588 timestamping implementation - hardware clock sync, external timestamps, periodic outputs
- **`dm9051.h`**: Register definitions, board_info structure, hardware constants
- **`dm9051_ptp.h`**: PTP-specific constants, GPIO pin definitions, timestamp configurations

### Key Architectural Decisions

1. **Dual Regmap Design**: The driver creates two regmap instances (`regmap_dm` and `regmap_dmbulk`) to handle different access patterns and avoid regmap execution conflicts. Both use custom regmap bus `regmap_bus_dm` due to dynamic loading requirements.
   - `regmap_dm`: Single register access, supports `regmap_noinc_read/write` for FIFO operations
   - `regmap_dmbulk`: Bulk operations with `use_single_read/write = true` to ensure SPI transaction atomicity
   - **Critical block sizes**: `CBLKRX=64` (RX), `CBLKTX=32` (TX) - tuned for SPI performance without timeout
   - Both share same mutex (`reg_mutex`) via `lock_arg = db` to prevent concurrent access

2. **Interrupt vs Polling Modes**: Runtime switchable via `use_interrupt` module parameter (default: `false`).
   - **Polling mode** (default): Uses `delayed_work` (`irq_workp`) scheduled with `DM_TIMER_EXPIRE2` (0 jiffies for fast polling)
   - **Interrupt mode**: Requires `modprobe dm9051 use_interrupt=1`, uses `request_threaded_irq()` with `IRQF_ONESHOT`
   - **Interrupt polarity**: Auto-detected via `irq_get_trigger_type()`, configured in DM9051_INTCR (0x39)
     - `INTCR_POL_LOW` (bit 0 = 1) for falling edge/low level
     - `INTCR_POL_HIGH` (bit 0 = 0) for rising edge/high level
   - Fallback: If IRQ request fails, driver automatically falls back to polling mode with warning

3. **Kernel Version Compatibility**: Configure `BOARD_CONF` in `dm9051.h`:
   ```c
   #define DM9051_KERNEL_6_1    7
   #define DM9051_KERNEL_6_6    8
   #define BOARD_CONF           DM9051_KERNEL_6_6  // Select your kernel
   ```
   This affects regmap API usage (e.g., `regmap_noinc_write` availability).

4. **PTP Hardware Timestamp Flow**:
   - **RX Path**: Timestamps stored in packet footer (8 bytes when PTP enabled). Check `RSR_RXTS_EN` bit in RX status to determine if timestamp present.
     - Modified error bits when PTP enabled: `RSR_ERR_BITS_PTP` excludes `RSR_LCS` and `RSR_PLE` (repurposed as `RSR_RXTS_EN` and `RSR_PARITY`)
   - **TX Path**: One-step (hardware inserts timestamp) vs two-step (software reads timestamp after TX).
     - One-step: Set `TCR_TS_EN` (bit 7) in TCR register during `dm9051_single_tx()`, hardware auto-fills timestamp
     - Two-step: Read timestamp from register 0x68 after TX completion, requires `GPTXRX_RD_TS` bit set
   - **Register 0x61 (DM9051_1588_CLK_CTRL)**: PTP clock control
     - `DM9051_CCR_PTP_EN` (bit 0): Enable PTP timestamping globally
     - `DM9051_CCR_IDX_RST` (bit 7): Reset index pointer for multi-byte timestamp reads
   - **Register 0x64 (DM9051_1588_RX_CONF1)**: RX filter configuration
     - `DM9051A_RC_RXTS_EN` (bit 4): Enable RX timestamping
     - `DM9051A_RC_FLTR_MASK` (bits 1-0): Filter modes (0=all, 1=mcast, 2=DA match, 3=DA specified)
   - **Registers 0x65-0x66**: One-step timestamp/checksum insertion offsets (see "Timestamp Offset Calculation" section)

## Critical Build & Development Workflows

### Building the Driver
```bash
cd dm9051_linux_driver
make                    # Builds dm9051.ko and compresses to dm9051.ko.xz
make install            # Install driver + load in polling mode
make interrupt          # Install driver + load with interrupt mode
make remove             # Unload driver
```

**Makefile targets**:
- `default`: Compiles driver and creates compressed `.ko.xz`
- `clean`: Removes all build artifacts
- `first`/`second`: Quick test workflows (down interface, load driver, up interface)

### Testing PTP Functionality

#### Required Tools
- **ptp4l**: PTP daemon (master/slave synchronization)
- **phc2sys**: Sync PHC (PTP Hardware Clock) to system clock
- **testptp**: Low-level PTP clock testing (in `test/` directory)

#### PTP Configuration Files
Located in `ptp4l_configs/`:
- `one_step_master.cfg`: Hardware one-step master configuration
- `l2_one_step_master.cfg`: Layer 2 (Ethernet) one-step master
- `phc2sys_gentle_converge.conf`: Gentle system clock convergence settings

#### Running as PTP Master (One-Step)
```bash
sudo ptp4l -i eth1 -m -f dm9051_linux_driver/ptp4l_configs/one_step_master.cfg
```

#### Running as PTP Slave
```bash
sudo ptp4l -i eth1 -m -H -s  # -H: hardware timestamping, -s: slave-only
sudo phc2sys -s eth1 -c CLOCK_REALTIME -m  # Sync system clock to PHC
```

#### Testing GPIO External Timestamps
```bash
cd dm9051_linux_driver/test
make
sudo ./extts /dev/ptp0 0  # Monitor GPIO1 external timestamp events
sudo ./perout /dev/ptp0 0 # Generate periodic output on GPIO1
```

### Debugging Patterns

1. **Enable Debug Printing**: Modify `dm_printk` level in `dm9051.h`:
   ```c
   #define DM_DEBUG_LEVEL KERN_DEBUG  // More verbose (default: KERN_INFO)
   #define DP_REG_WRITE BIT(0)        // Enable register write tracing
   ```
   Set `db->debug_print |= DP_REG_WRITE` at runtime to trace all register writes.

2. **Common Debug Points**:
   - **RX error recovery**: Check `dm9051_all_restart()` calls and `bc.fifo_rst_counter`
     - Look for: `"dm9.Monitor headbyte/status/rxlen"` followed by `"check rxstatus-error"` or `"check rxlen large-error"`
     - Restart message: `"dm9.Show rxstatus_Er & rxlen_Er %d, RST_c %d"`
   - **PTP timestamp issues**: Monitor `ptp_tx_skb`, `tstamp_config`, GPIO status registers (0x60, 0x6A)
     - Check: `"[1588 Time Stamp] RX Packet Timestamp"` messages (commented out by default)
     - Verify one-step config: `"[in dm9051_netdev_ioctl()] *SIOCSHWTSTAMP*"` and `"[in dm9051_netdev_ioctl()] *SIOCGHWTSTAMP*"`
   - **SPI communication**: Look for `regmap_read`/`regmap_write` errors in dmesg
     - Error pattern: `"error %d get/set reg %02x"`
   - **Mode switching**: Confirm polling/interrupt mode
     - Polling: `"using polling mode (interrupt unavailable)"`
     - Interrupt: `"interrupt mode enabled (use_interrupt=1)"`
     - Fallback: `"irq request failed (%d), fallback to polling mode"`

3. **Register Inspection**:
   ```bash
   cat /sys/kernel/debug/regmap/spi0.1/registers  # View cached register state (requires CONFIG_REGMAP_DEBUG)
   cat /proc/interrupts | grep dm9051              # Check interrupt count (interrupt mode only)
   sudo modinfo dm9051                             # Verify loaded driver version
   ```

4. **Common Issues**:
   - **PHY power down warnings**: Look for `"BMCR 0x3900: power down (warn)"` - indicates improper shutdown
   - **FIFO desync**: Repeated `"eval_rxb"` messages with non-0x01 byte patterns
   - **PTP clock not found**: Check `/dev/ptp0` exists and `ptp_clock_index(db->ptp_clock)` != -1

## Project-Specific Conventions

### Register Access Patterns
Always use exported accessor functions (never direct regmap calls from outside dm9051_main.c):
```c
dm9051_get_reg(db, DM9051_NSR, &val);           // Read single register
dm9051_set_reg(db, DM9051_RCR, RCR_RXEN);       // Write single register
dm9051_get_regs(db, DM9051_PAR, addr, 6);       // Bulk read (e.g., MAC address)
dm9051_write_mem(db, DM_SPI_MWCMD, buff, len);  // FIFO write (TX packet data)
dm9051_read_mem(db, DM_SPI_MRCMD, rdptr, rxlen); // FIFO read (RX packet data)
```

**Common patterns**:
- **FIFO operations**: Use `DM_SPI_MRCMDX` (0x70) for dummy read, `DM_SPI_MRCMD` (0x72) for actual data
- **Multi-byte timestamp reads**: Always wrap in `mutex_lock(&db->tsreg_lock)` and reset index with `DM9051_CCR_IDX_RST`
- **Update bit patterns**: Use `dm9051_update_bits(db, reg, mask, val)` for atomic read-modify-write
- **Error checking**: All register functions return `int` - check for `< 0` errors from regmap layer

**SPI write flag**: All write operations automatically OR with `DM_SPI_WR` (0x80) via `regconfigdm.write_flag_mask`

### Locking Hierarchy
1. **`spi_lockm`**: Big lock for all SPI transactions (held in threaded IRQ, work queues)
2. **`reg_mutex`**: Regmap lock (internal to regmap operations)
3. **`tsreg_lock`**: PTP timestamp register access (to prevent race during multi-byte reads)

### Error Handling Philosophy
- **RX errors trigger FIFO reset**: If `RSR_ERR_BITS` detected or invalid rxlen, call `dm9051_all_restart()` to reset chip state
  - **Invalid rxlen check**: `rxlen > DM9051_PKT_MAX` (1536 bytes) triggers `large_err_counter++` and restart
  - **RX byte validation**: `eval_rxb()` checks if byte pattern indicates packet ready (`0x0100` in bits [15:8])
    - Accumulates up to `TIMES_TO_RST` (10) invalid patterns before forcing restart
    - Helps detect and recover from FIFO pointer desynchronization
  - **Restart sequence**: `dm9051_core_reset()` → NCR reset → PHY reset → restore IMR/INTCR → re-enable RX
- **TX errors are logged but non-fatal**: Increment `bc.tx_err_counter`, continue operation
  - TX timeout handled by `dm9051_nsr_poll()` with 20μs max wait for `NSR_TX2END | NSR_TX1END`
- **Monitor error counters**: `bc.status_err_counter`, `bc.large_err_counter`, `bc.fifo_rst_counter` tracked in `board_info`
  - Printed during restart: `"rxstatus_Er & rxlen_Er %d, RST_c %d"`

### PTP Packet Type Detection
Use Linux's `ptp_classify.h` for packet inspection:
```c
db->ptp_class = ptp_classify_raw(skb);
if (PTP_CLASS_V2_IPV4 == (db->ptp_class & PTP_CLASS_VMASK)) {
    // Handle IPv4 PTP packet
}
db->ptp_msgtype = ptp_get_msgtype(db->ptp_hdr, db->ptp_class);
```

## Integration Points

### SPI Configuration
Driver expects SPI device defined in device tree with:
- Compatible string: `"davicom,dm9051"`
- IRQ line for interrupt mode
- Chip select configuration via `spi_get_chipselect(spi, 0)`

### PHY/MDIO Integration
Internal PHY at address `DM9051_PHY_ADDR` (1). MDIO bus registered at probe:
```c
db->mdiobus->phy_mask = (u32)~BIT(1);  // Only PHY 1 exists
```

### PTP Clock Registration
Creates `/dev/ptp0` device supporting:
- `PTP_CLOCK_GETCAPS`: Query capabilities (max_adj, n_ext_ts, n_per_out, n_pins)
- `PTP_EXTTS_REQUEST`: Configure GPIO external timestamp capture (rising/falling edge)
- `PTP_PEROUT_REQUEST`: Configure GPIO periodic output (TOD-based or continuous)
- `SIOCGHWTSTAMP`/`SIOCSHWTSTAMP`: Get/set hardware timestamp configuration

## Key File Locations

- **Main driver source**: `dm9051_linux_driver/{dm9051_main.c, dm9051_ptp.c, dm9051.h, dm9051_ptp.h}`
- **Build system**: `dm9051_linux_driver/Makefile`
- **PTP test utilities**: `dm9051_linux_driver/test/{extts.c, perout.c, testptp.c, phase.c}`
- **Config examples**: `dm9051_linux_driver/ptp4l_configs/`
- **Version history**: Directory names like `lnx_dm9051_kt6_r2503_v3.9.5e_ptp/` indicate release versions

## Timestamp Offset Calculation (One-Step Mode)

For hardware to insert timestamps at correct offset:
```
IPv4: Eth(14) + IP(20) + UDP(8) + PTP_originTimestamp(34) = 76 → Set 0x65 = 0x4E (76+2)
IPv6: Eth(14) + IP(40) + UDP(8) + PTP_originTimestamp(34) = 96 → Set 0x65 = 0x62 (96+2)
L2:   Eth(14) + PTP_originTimestamp(34) = 48 → Set 0x65 = 0x32 (48+2)
```
+2 bytes because DM9051A uses 8 of 10 timestamp bytes.

## Branch Context

Current branch: `pi3_interrupt_release_v001` (indicates interrupt mode development/testing focus)
Repository owner: josright123
