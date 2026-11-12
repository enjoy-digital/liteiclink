# Build options {#build-options}

This appendix lists the constructor parameters of the PLLs, the SerDes wrappers, the DRP access
block and the SerWB PHYs and cores, with their defaults. The chapters explain when to use them;
this appendix gives their names. The options of the bench targets and host scripts are in
[Benches and examples](#examples), those of the simulation script in [Simulation and
tests](#simulation).

## PLLs {#opt-pll}

Every PLL class takes the reference clock signal, its frequency in Hz and the line rate in bit/s (`5e9` for 5 Gbps),
and solves its dividers at construction. It raises a `ValueError` when no divider set fits.

| Class | Family | Extra parameters |
|---------------------------------|------------------------------------|-----------------------------------------------------------------------|
| `GTPQuadPLL` | Artix-7 GTP | `channel`, `shared`, `refclk_from_fabric` |
| `GTXChannelPLL` | Kintex-7, Virtex-7 GTX | None (CPLL) |
| `GTXQuadPLL` | Kintex-7, Virtex-7 GTX | None (QPLL) |
| `GTHChannelPLL` | GTH (each GTH module) | None (CPLL) |
| `GTHQuadPLL` | Virtex-7 GTH | None (QPLL) |
| `GTH3QuadPLL` | UltraScale GTH | None (QPLL0 or QPLL1) |
| `GTH4QuadPLL` | UltraScale+ GTH | `refclk_from_fabric` |
| `GTYChannelPLL` | UltraScale+ GTY | None (CPLL) |
| `GTYQuadPLL` | UltraScale+ GTY | `refclk_from_fabric`, `qpll`, `qpll_params` |
| `SerDesECP5PLL` | ECP5 DCU | None |

| Parameter | Default | Meaning |
|---------------------------------------------|------------------|-----------------------------------------------------------------------------|
| `refclk` | required | Reference clock signal |
| `refclk_freq` | required | Reference clock frequency, Hz |
| `linerate` | required | Line rate, bit/s |
| `channel` | `0` | GTP: PLL0 (0) or PLL1 (1) of the quad |
| `shared` | `False` | GTP: do not instantiate the quad common block; it is built elsewhere |
| `refclk_from_fabric` | `False` | Take the reference clock from the clock network (GTP: `GTGREFCLK`) instead of the dedicated pins |
| `qpll` | `None` | GTY: force `"qpll0"` or `"qpll1"`; `None` lets the solver choose |
| `qpll_params` | `None` | GTY: dictionary of QPLL attributes to override; the derived dividers cannot be overridden |

## Xilinx wrappers {#opt-xilinx}

`GTP`, `GTX`, `GTH` (Virtex-7), `GTH3`, `GTH4` and `GTY` share their parameters, with the
differences listed after the table.

| Parameter | Default | Meaning |
|-----------------------------------------------|-------------------------------|--------------------------------------------------------------|
| `pll` (`qpll` on GTP) | required | PLL object of the lane |
| `tx_pads`, `rx_pads` | required | Differential pads, with `p` and `n` |
| `sys_clk_freq` | required | System clock frequency, Hz, for the init timers |
| `tx_clk`, `rx_clk` | `None` | External user clocks; require the buffers |
| `data_width` | `20` | Word width in bits: 20 or 40 (GTY: also 80) |
| `tx_buffer_enable` | `False` | Use the TX buffer instead of the buffer bypass |
| `rx_buffer_enable` | `False` | Use the RX buffer instead of the buffer bypass |
| `clock_aligner` | `True` | Restart the RX until the comma is at bits 9:0 |
| `clock_aligner_comma` | `0b0101111100` | Comma that the clock aligner looks for |
| `tx_polarity` | `0` | Static TX P/N swap |
| `rx_polarity` | `0` | Static RX P/N swap |
| `pll_master` | `True` | The TX init drives the PLL reset (not on GTP) |
| `qpll_reset` | `True` | GTP: the TX init drives the quad PLL reset |
| `tx_usrclk`, `rx_usrclk` | `None` | GTP, 40-bit words with `tx_clk`/`rx_clk`: line rate / 20 clock for `TXUSRCLK`/`RXUSRCLK` |

The words carry `data_width/10` bytes on the stream endpoints. Set `pll_master` or `qpll_reset` to
`False` when another lane drives the reset of a shared PLL.

| Method | Adds |
|--------------------------------------------------------|------------------------------------------------------------------------------------|
| `add_stream_endpoints()` | `sink` and `source`, with `data` and `ctrl` (K flags) |
| `add_controls()` | The five groups below |
| `add_base_control()` | Enables, ready flags, `tx_inhibit`, square wave, clock aligner |
| `add_prbs_control()` | PRBS configuration and error registers |
| `add_loopback_control()` | `loopback` |
| `add_polarity_control()` | `tx_polarity`, `rx_polarity` |
| `add_electrical_control()` | Swing and cursor registers |
| `add_clock_cycles()` | `clock_latch`, `clock_tx_cycles`, `clock_rx_cycles` |

`add_controls()` takes `auto_enable` (default `True`) and `rx_prbs_errors_width` (default 32), and
passes them to the base and PRBS groups. With `auto_enable=False`, `tx_enable` and `rx_enable` reset to 0 and software starts the lane.

## ECP5 wrapper {#opt-ecp5}

| Parameter | Default | Meaning |
|--------------------------------------|------------------|------------------------------------------------------------------------------------|
| `pll` | required | `SerDesECP5PLL` object |
| `tx_pads`, `rx_pads` | required | Differential pads, with `p` and `n` |
| `dual` | `0` | DCU: 0 or 1 |
| `channel` | `0` | Channel of the DCU: 0 or 1 |
| `data_width` | `20` | 20 or 40; 40 requires `with_oob=True` |
| `tx_polarity` | `0` | TX P/N swap, applied through SCI |
| `rx_polarity` | `0` | RX P/N swap, applied through SCI |
| `with_oob` | `False` | Init sequence and OOB signalling for SATA |

`SerDesECP5` has the same `add_stream_endpoints()`, `add_controls()` and `add_clock_cycles()`
methods; `add_controls()` adds the base, PRBS and loopback registers only.

## GTR12 wrapper {#opt-gtr12}

| Parameter | Default | Meaning |
|-----------------------------|-----------------------------|----------------------------------------------------------------------------------|
| `platform` | required | Platform: device name and toolchain hook for the configuration |
| `linerate` | required | Line rate, bit/s; `line_rate` is the older name, also accepted |
| `csr_config` | required | Quad configuration, as generated by the Gowin tools |
| `data_width` | `40` | 8b/10b word width: 20, 40 or 80 |
| `raw_width` | `None` | Width of the raw words, at least `data_width` (20, 40 or 80) |
| `position` | `"Q1"` | Quad: `"Q0"` or `"Q1"` (`"Q0"` only on GW5AT-15) |
| `lane` | `0` | Lane of the quad |
| `pll` | `"QPLL1"` | PLL of the configuration, for the lock status |
| `tx_polarity` | `0` | TX P/N swap, applied in the fabric |
| `rx_polarity` | `0` | RX P/N swap, applied in the fabric |
| `csr_name` | `"gtr12.csr"` | File of the quad configuration in the gateware directory |
| `with_upar` | `True` | Instantiate the UPAR access block |
| `upar_clk` | `None` | UPAR clock; `None` takes `sys` |

The wrapper has `add_stream_endpoints()` and `add_status_csr()`; the UPAR registers come from
`upar.add_csr()`.

## DRP access {#opt-drp}

`DRPControl(drp)` (`liteiclink/serdes/common.py`) takes the `drp` interface of a Xilinx channel or
quad PLL and adds the registers of the [DRP access](#reg-serdes-drp) bank. `DRPInterface` has a
9-bit address and 16-bit data by default (`address_width`, `data_width`); the GTH4 and GTY
channels use a 10-bit address, and the `addr` register takes the width of the interface.

## SerWB PHYs {#opt-serwb-phy}

The high-speed PHY, `SERWBPHY` of `liteiclink/serwb/phy.py`, selects its I/O blocks from the
device name. The generic PHY, `SERWBPHY` of `liteiclink/serwb/genphy.py`, takes the subset marked
in the last column.

| Parameter | Default | Meaning | Generic |
|--------------------------------------------|-------------------------|-----------------------------------------------------|------------------|
| `device` | required | Device name, `platform.device` | Yes |
| `pads` | required | `clk`, `tx`, `rx`, differential (`_p`, `_n`) on the high-speed PHY | Yes |
| `mode` | `"master"` | `"master"` drives the link clock and the init; `"slave"` follows | Yes |
| `init_timeout` | `2**16` | Cycles of each init wait | Yes |
| `clk`, `clk4x` | `"sys"`, `"sys4x"` | Clock domains of the Efinix PHY | No |
| `clk_ratio` | `"1:1"` | Efinix: `"1:1"`, `"1:2"` or `"1:4"`; other PHYs `"1:1"` only | No |
| `clk_delay_taps` | `0` | Trion: static delay of the clock input | No |
| `rx_delay_taps` | `0` | Trion: static delay of the data input | No |
| `usp` | `False` | UltraScale+ variant of the UltraScale I/O blocks | No |
| `with_flow_control` | `False` | XON/XOFF flow control; on both sides of the link | Yes |
| `with_prbs` | `False` | PRBS31 link test registers | Yes |
| `with_auto_retrain` | `False` | Master: restart the init on bursts of invalid words | Yes |

| Device prefix | I/O blocks | Delay taps |
|---------------------------------------------------------------------------------------------|--------------------------|---------------------|
| `xcku`, `xcvu`, `xczu`, `xcau` | UltraScale(+) | 512 |
| `xc7a`, `xc7k`, `xc7v`, `xc7z`, `xc7s` | 7-Series | 32 |
| `Ti` | Efinix Titanium | 64 |
| `T` and a digit | Efinix Trion | static |

The match is on the first four characters for Xilinx and on the first two for Efinix: `Ti` for
Titanium, `T` and a digit for Trion. Artix UltraScale+ (`xcau`) always uses the UltraScale+
settings. Other names raise a `NotImplementedError`.

## SerWB core {#opt-serwb-core}

| Parameter | Default | Meaning |
|-----------------------------------------------------|----------------------|-----------------------------------------------------------------|
| `phy` | required | SerWB PHY |
| `clk_freq` | required | System clock frequency, Hz, for the timeouts |
| `mode` | required | `"slave"`: exposes `bus` to the SoC; `"master"`: drives `bus` on the remote side |
| `with_rst_on_link_down` | `True` | Hold the core in reset while the link is down |
| `port` | `0` | Port of the bus protocol in the packet layer |
| `protocol` | `"native"` | `"native"` or `"etherbone"` (deprecated); same on both sides |
| `burst_depth` | `4` | Native: most writes merged in one request; remote read buffer, words |
| `coalescing_timeout` | `4` | Native, SoC side: idle cycles before a pending write is sent; 0 sends at once |
| `response_timeout` | `1e-3` | Native, SoC side: seconds before a read without response ends with a bus error; `None` waits forever |
| `etherbone_buffer_depth` | `4` | Etherbone: record buffer, at least 2 |
| `tx_buffer_depth` | `None` | TX buffer, words; `None` sizes it from the largest packet |
| `rx_buffer_depth` | `None` | RX buffer, words; `None` adds 16 words with flow control |
| `depacketizer_timeout` | `1e-3` | Seconds before an incomplete packet is dropped |
| `with_crc` | `False` | CRC16 on every packet; on both sides of the link |
| `crc_max_words` | `None` | Largest payload accepted with the CRC, in words |

With `None`, each buffer holds at least 8 words and one complete packet of the chosen protocol.
`add_downstream_endpoint(port, endpoint)` and `add_upstream_endpoint(port, endpoint)` connect a
user stream to another port of the packet layer.

`SERIOCore(serwb_core, port=1)` tunnels 32 inputs (`i`) and 32 outputs (`o`) on a port of an
existing core; several SerIOs can share a core on different ports.
