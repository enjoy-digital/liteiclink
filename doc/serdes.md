# SerDes

LiteICLink wraps the FPGA multi-gigabit transceivers in a common interface: PLL configuration
computed from the reference clock and linerate, reset/init sequencing, 8b10b encoding/decoding,
user clock generation, stream endpoints and CSRs (PRBS, loopback, polarity, TX driver settings).

## Supported transceivers

| Family                                  | Module                       | Classes                               | Data width  |
|-----------------------------------------|------------------------------|---------------------------------------|-------------|
| Xilinx 7-Series GTP (Artix7)            | `serdes.gtp_7series`         | `GTPQuadPLL`, `GTP`                   | 20, 40      |
| Xilinx 7-Series GTX (Kintex7/Virtex7)   | `serdes.gtx_7series`         | `GTXChannelPLL`, `GTXQuadPLL`, `GTX`  | 20, 40      |
| Xilinx 7-Series GTH (Virtex7)           | `serdes.gth_7series`         | `GTHChannelPLL`, `GTHQuadPLL`, `GTH`  | 20, 40      |
| Xilinx UltraScale GTH (GTHE3)           | `serdes.gth3_ultrascale`     | `GTHChannelPLL`, `GTH3QuadPLL`, `GTH3`| 20, 40      |
| Xilinx UltraScale+ GTH (GTHE4)          | `serdes.gth4_ultrascale`     | `GTHChannelPLL`, `GTH4QuadPLL`, `GTH4`| 20, 40      |
| Xilinx UltraScale+ GTY (GTYE4)          | `serdes.gty_ultrascale`      | `GTYChannelPLL`, `GTYQuadPLL`, `GTY`  | 20, 40, 80  |
| Lattice ECP5 DCU                        | `serdes.serdes_ecp5`         | `SerDesECP5PLL`, `SerDesECP5`         | 20          |

`serdes.gth_ultrascale` re-exports the GTH3/GTH4 classes. The old `liteiclink.transceiver` module
name is a deprecated alias of `liteiclink.serdes`.

The data width is the transceiver internal width in bits (8b10b encoded): 20 bits carries 2 bytes
per user clock cycle, 40 bits 4 bytes and 80 bits 8 bytes.

## PLLs

PLL classes take `(refclk, refclk_freq, linerate)` and select the dividers in `compute_config`.
A `ValueError` is raised when the linerate can't be reached from the reference clock. `print(pll)`
shows the selected configuration.

- Channel PLLs (CPLL): one per channel, lower VCO range.
- Quad PLLs (QPLL): shared by the channels of a quad, higher VCO range (higher linerates). GTP only
  has quad PLLs.
  - `GTPQuadPLL(..., channel=0, shared=False)`: GTP PLL0/PLL1 selection; with `shared=True`, the
    `GTPE2_COMMON` is not instantiated (shared with another core).
  - `GTH4QuadPLL`/`GTYQuadPLL(..., refclk_from_fabric=False)`: reference clock from the fabric.
  - `GTYQuadPLL(..., qpll=None, qpll_params=None)`: QPLL0/QPLL1 selection, fractional feedback
    and primitive parameters tuning.
- `SerDesECP5PLL`: ECP5 DCU PLL (x8/x10/x16/x20 multipliers, x25 with 100MHz refclk).

Examples (8b10b, 20-bit): 125MHz refclk with CPLL for 1.25/2.5/5 Gbps, 156.25MHz refclk with QPLL
for 3.125/6.25/10.3125 Gbps.

## Transceiver

Xilinx transceivers share the same constructor:

```python
serdes = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
    tx_clk              = None,         # Optional external/shared TX user clock (requires TX buffer).
    rx_clk              = None,         # Optional external/shared RX user clock (requires RX buffer).
    data_width          = 20,           # 20, 40 (80 on GTY).
    tx_buffer_enable    = False,        # TX buffer (otherwise buffer bypass with delay alignment).
    rx_buffer_enable    = False,        # RX buffer (otherwise buffer bypass with delay alignment).
    clock_aligner       = True,         # Comma alignment by transceiver restarts (fixed latency).
    clock_aligner_comma = 0b0101111100, # K28.5.
    tx_polarity         = 0,            # Static TX P/N swap.
    rx_polarity         = 0,            # Static RX P/N swap.
    pll_master          = True,         # Init FSM drives the PLL reset (False for a shared PLL).
)
serdes.add_stream_endpoints() # sink/source: data (8 x data_width//10 bits), ctrl (K characters).
serdes.add_controls()         # CSRs (see below).
serdes.add_clock_cycles()     # TX/RX clock measurement CSRs.
```

GTP takes a `qpll` (and a `qpll_reset=True` option) instead of `pll`. `SerDesECP5(pll, tx_pads,
rx_pads, dual=0, channel=0, data_width=20, tx_polarity=0, rx_polarity=0, with_oob=False)`
selects the DCU/channel and optionally enables OOB signaling (SATA).

The transceiver provides `tx` and `rx` clock domains (`cd_tx`/`cd_rx`, `tx_clk_freq =
rx_clk_freq = linerate/data_width`) and `tx_ready`/`rx_ready` once the init FSMs are done. The
user design has to constrain them, for example:

```python
platform.add_period_constraint(serdes.cd_tx.clk, 1e9/serdes.tx_clk_freq)
platform.add_period_constraint(serdes.cd_rx.clk, 1e9/serdes.rx_clk_freq)
platform.add_false_path_constraints(soc.crg.cd_sys.clk, serdes.cd_tx.clk, serdes.cd_rx.clk)
```

### Clocking

- With the buffers bypassed (default, fixed latency), the TX user clock is derived from the PLL
  reference clock: `refclk_freq/tx_clk_freq` must be 1 or an integer (7-Series: BUFG, BUFR,
  or a PLL for non-integer ratios; UltraScale(+): BUFG_GT divider, integer only).
- `tx_buffer_enable`/`rx_buffer_enable` decouple the user clocks from the transceiver internal
  clocks (simpler clocking, variable latency), and are required to use external `tx_clk`/`rx_clk`
  (for example to share one user clock between several transceivers).

### Init

TX/RX init FSMs (`*_init.py`) follow the vendor reset sequences: PLL reset/lock, transceiver reset
and reset done, then (buffer bypass only) delay/phase alignment. A watchdog restarts the sequence
when it doesn't complete (1ms on 7-Series, 10ms on UltraScale, 4ms on GTP RX).
`tx_enable`/`rx_enable` restart the sequence when deasserted.

With `clock_aligner=True`, the RX is restarted until the comma is received aligned on the LSBs of
RXDATA, which gives a fixed RX latency (`BruteforceClockAligner`).

### CSRs

| CSR                                                | Description                                      |
|----------------------------------------------------|--------------------------------------------------|
| `tx_enable`, `rx_enable`                           | Enable TX/RX (deassert to restart init).         |
| `tx_ready`, `rx_ready`                             | TX/RX init done.                                 |
| `clock_aligner_disable`                            | Disable the clock aligner (with `clock_aligner`).|
| `tx_inhibit`                                       | Disable the TX driver.                           |
| `tx_produce_square_wave`                           | Generate a square wave (scope/clock checks).     |
| `tx_prbs_config`, `rx_prbs_config`                 | PRBS7/15/31 generator/checker (bit 2 on RX: pause error counter). |
| `rx_prbs_errors`                                   | PRBS errors counter.                             |
| `loopback`                                         | Near-end PCS/PMA, far-end PMA loopback.          |
| `tx_polarity`, `rx_polarity`                       | P/N swap.                                        |
| `tx_diffctrl`, `tx_postcursor`, `tx_precursor`     | TX swing and pre-emphasis (Xilinx, see vendor user guides). |
| `clock_latch`, `clock_tx_cycles`, `clock_rx_cycles`| TX/RX clock frequency measurement.               |

ECP5 provides the base, PRBS and loopback controls (no polarity/electrical CSRs).

## Benches

`bench/serdes/` contains test designs (K28.5 + counter over the link, LEDs) for KC705 (GTX), KCU105
(GTH3), XCU1525 (GTY), SQRL Acorn (GTP), Versa ECP5 and ECPIX-5, with a UARTBone for CSR access:

```sh
./kc705.py --connector sfp --linerate 2.5e9 --build --load
litex_server --uart --uart-port /dev/ttyUSBX
./test_prbs.py --mode prbs31 --linerate 2.5e9 --duration 60     # Add --loopback for near-end PMA loopback.
```

`test_prbs.py` measures the TX/RX clocks, enables PRBS on TX and RX and reports errors and BER.

## Tests

`python3 -m pytest -v test/test_serdes_*.py test/test_gty_qpll_config.py`:

- `test_serdes_xilinx.py`: Verilog generation of PLL + transceiver + endpoints + CSRs for each
  Xilinx family and the main options.
- `test_serdes_pll.py`: PLL configurations (dividers, VCO ranges) for common linerates.
- `test_serdes_init.py`: init FSMs against a behavioral transceiver model.
- `test_serdes_clock_aligner.py`: clock aligner.
- `test_serdes_qpll.py`, `test_gty_qpll_config.py`: GTH4/GTY QPLL reference clock selection and
  configuration.
- `test_serdes_ecp5.py`: ECP5 init (OOB) and word aligner.
