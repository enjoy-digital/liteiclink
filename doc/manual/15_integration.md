# Integration {#integration}

This chapter explains how to use LiteICLink in your own design. It starts with a SerDes wrapper in
a LiteX SoC, then covers the clocking and data width choices, PLL sharing, raw symbols and
register access to the transceiver.

It continues with a SerWB link between two FPGAs, shows how
LiteX and other cores use the wrappers, and ends with what a wrapper for a new FPGA family must
provide.

## At a glance {#integ-glance}

| Item | Value |
|-----------------------------|---------------------------------------------------------------|
| Language | Python (Migen, LiteX); no standalone Verilog generator |
| SerDes classes | A PLL class and a wrapper class per family, see [SerDes wrapper architecture](#serdes-architecture) |
| SerDes user interface | `sink`/`source` streams in the `tx`/`rx` clock domains, no backpressure |
| SerDes registers | `add_controls()`, `add_clock_cycles()`, `DRPControl` |
| SerWB classes | `SERWBPHY` (one per side), `SERWBCore`, `SERIOCore` |
| SerWB user interface | Wishbone, 32-bit data: a slave on the SoC side, a master on the remote side |
| Reference designs | The targets of `bench/serdes/` and `bench/serwb/` |

## A SerDes wrapper in a LiteX SoC {#integ-serdes-soc}

A design that uses a transceiver lane has four parts, shown in the figure below. The reference
clock enters the PLL class, which computes the dividers for the line rate. The wrapper drives the
transceiver channel, runs its initialization sequences and provides the user clocks. User logic
writes and reads the streams of the wrapper in the `tx` and `rx` clock domains.

![A SerDes wrapper in a LiteX SoC. The cyan arrows are the stream endpoints, in the user clock
domains of the wrapper; the registers sit on the CSR bus.](integ_serdes.svg)

The example below adds a GTX lane at 2.5 Gbps on the SFP cage of a KC705, with the 125 MHz
reference clock on the dedicated transceiver clock pins. It follows `bench/serdes/kc705.py` and
was elaborated as written in a `SoCMini`; `self.crg` is the clock generator of the SoC.

```python
from migen import *

from litex.soc.cores.code_8b10b import K

from liteiclink.serdes.gtx_7series import GTXChannelPLL, GTX

# Reference clock (dedicated transceiver clock pins).
refclk      = Signal()
refclk_pads = platform.request("sgmii_clock")
self.specials += Instance("IBUFDS_GTE2",
    i_CEB = 0,
    i_I   = refclk_pads.p,
    i_IB  = refclk_pads.n,
    o_O   = refclk,
)

# PLL and transceiver.
self.pll = pll = GTXChannelPLL(refclk, 125e6, 2.5e9)
self.serdes0 = serdes0 = GTX(pll,
    tx_pads          = platform.request("sfp_tx"),
    rx_pads          = platform.request("sfp_rx"),
    sys_clk_freq     = sys_clk_freq,
    tx_buffer_enable = True,
    rx_buffer_enable = True,
    clock_aligner    = False,
)
serdes0.add_stream_endpoints()
serdes0.add_controls()
serdes0.add_clock_cycles()

# Timing constraints.
platform.add_period_constraint(serdes0.cd_tx.clk, 1e9/serdes0.tx_clk_freq)
platform.add_period_constraint(serdes0.cd_rx.clk, 1e9/serdes0.rx_clk_freq)
platform.add_false_path_constraints(self.crg.cd_sys.clk,
    serdes0.cd_tx.clk, serdes0.cd_rx.clk)

# User logic: a K28.5 comma, then one data byte per tx cycle.
tx_byte = Signal(8)
rx_byte = Signal(8)
self.sync.tx += tx_byte.eq(tx_byte + 1)
self.comb += [
    serdes0.sink.valid.eq(1),
    serdes0.sink.ctrl.eq(0b01),
    serdes0.sink.data.eq(Cat(K(28, 5), tx_byte)),
]
self.sync.rx += If(serdes0.source.ctrl == 0b01,
    rx_byte.eq(serdes0.source.data[8:16])
)
```

At 20 bits, each stream word carries two bytes and two `ctrl` bits, one per byte: a `ctrl` bit
at 1 sends that byte as a K character.

The streams have no flow control: `sink.ready` and
`source.valid` are always 1, so user logic provides a word on every `tx` cycle and takes one on
every `rx` cycle. The other families follow the same pattern with their own classes, listed in
[SerDes wrapper architecture](#serdes-architecture).

### Clock domains and reset {#integ-serdes-clocks}

The wrapper creates the `tx` and `rx` clock domains (`cd_tx`, `cd_rx`), both at the line rate
divided by the data width (`tx_clk_freq`, `rx_clk_freq`). Each domain stays in reset until its
initialization sequence completes. With several wrappers in one SoC, Migen gives the domains
unique names prefixed by the submodule (`serdes0_tx`, `serdes1_tx`), as the Obsidian target of
LiteX-Boards notes.

The wrapper restarts its sequences when `tx_enable` or `rx_enable` goes low, and reports
completion on `tx_ready` and `rx_ready`. `add_controls()` maps these signals to registers,
enabled at reset; `add_controls(auto_enable=False)` leaves the lane off until software enables
it. To move data to the `sys` domain, use a clock domain crossing such as the LiteX
`stream.ClockDomainCrossing`.

### Timing constraints {#integ-constraints}

The wrapper does not constrain its user clocks: the design adds a period constraint on each
domain and false paths to `sys`, as in the example. The wrapper resynchronizes its control and
status signals between `sys` and the user clock domains (`MultiReg`). The ECP5 targets constrain
the transceiver clock outputs (`txoutclk`, `rxoutclk`) instead of the domain clocks.

## Choosing the data width and the buffers {#integ-choices}

The data width sets how many 10-bit symbols the transceiver exchanges per user clock cycle, so it
sets the user clock frequency. The table below gives the user clock for two line rates,
computed as line rate divided by data width; the 10 Gbps column applies only to the families
whose PLLs reach that rate.

| Data width | Bytes per word | Families | User clock at 5 Gbps (computed) | At 10 Gbps (computed) |
|------------|----------------|------------------------------|--------------------|-------------|
| 20 bits | 2 | All | 250 MHz | 500 MHz |
| 40 bits | 4 | All | 125 MHz | 250 MHz |
| 80 bits | 8 | GTY, Gowin GTR12 | 62.5 MHz | 125 MHz |

Pick the narrowest width whose user clock meets timing in your fabric. A wider word halves the
clock but doubles the logic per cycle, and the ECP5 needs `with_oob=True` for 40 bits, as
[Lattice ECP5 DCU](#lattice-ecp5) explains.

The Xilinx wrappers bypass the transceiver buffers by default, for a fixed latency;
`tx_buffer_enable` and `rx_buffer_enable` enable them, and external user clocks (`tx_clk`,
`rx_clk`) need them. The table below compares the two modes; the clock rules are detailed in
[SerDes wrapper architecture](#serdes-architecture).

| Mode | Latency | User clocks | Constraint |
|-------------------------|------------------|---------------------------------------|-------------------------------------------|
| Buffers bypassed | Fixed | Derived from the reference clock | Reference to user clock ratio: 1 or integer; non-integer on 7-Series only |
| Buffers enabled | Varies at start | Transceiver outputs, or external | None |

The clock aligner (`clock_aligner=True`, default) completes the fixed-latency mode: it restarts
the receiver until the comma lands on the first symbol of the word. With the buffers enabled,
the bench targets turn it off and let the transceiver align on commas.

## Sharing a PLL and user clocks {#integ-sharing}

The channels of a quad can share its quad PLL: create one quad PLL object and pass it to each
wrapper. Only one wrapper may reset the PLL. On GTX, GTH and GTY, set `pll_master=False` on the
others; on GTP, set `qpll_reset=False` on the others. When another core already instantiates the
`GTPE2_COMMON` of the quad, `GTPQuadPLL(shared=True)` reuses it.

Several channels can also share one user clock, which saves clock buffers. Pass the clock of the
first wrapper as `tx_clk` of the others, with `tx_buffer_enable=True`: every Xilinx wrapper
asserts it when `tx_clk` is given, so a missing flag fails at elaboration. The DVI PHY on UltraScale+
GTH of LiteX (`VideoUSPGTHHDMIPHY`) does both: four channels on one quad PLL, one PLL master and
one shared TX clock.

## Raw symbols and other line codes {#integ-raw}

The wrappers encode 8b/10b in the fabric. A protocol with another line code, such as 64b/66b or
128b/132b, needs the transceiver data without this encoding. Three wrappers have a raw path that
replaces the encoder output on every user clock cycle.

| Family | Raw TX | Raw RX | Enable |
|-------------------|------------------|------------------|-------------------------------|
| GTY | `tx_raw` | `rx_raw` | `tx_raw_enable` |
| Gowin GTR12 | `tx_raw` | `rx_raw` | `tx_raw_enable`, `raw_width` |
| ECP5 DCU | `tx_raw` | `rx_raw` | `tx_raw_enable` (only with `with_oob=True`) |

On the ECP5, `tx_raw_data` and `rx_raw_data` are the older names of the same signals.

The other families have `tx_pattern` with `tx_produce_pattern`, which replaces the encoder output
too, but through a synchronizer (`MultiReg`) meant for control signals.

The HDMI PHYs of LiteX
show both ways to stream raw words on them: the GTP one feeds `tx_pattern` from logic in the `tx`
domain, the GTH4 one overrides the `TXDATA` inputs through the `gth_params` dictionary of the
wrapper. The GTY also
changes rate on the fly through `tx_rate` and `rx_rate`, described in
[Xilinx UltraScale and UltraScale+](#xilinx-ultrascale).

## Transceiver registers from software {#integ-drp}

The Xilinx transceivers have a dynamic reconfiguration port (DRP) that reads and writes their
attributes at run time. Each wrapper exposes it as `drp`, behind a multiplexer (`drp_mux`).
`DRPControl` turns a DRP interface into registers:

```python
from liteiclink.serdes.common import DRPControl

self.drp = DRPControl(serdes0.drp)
```

Software writes `addr`, `we` and, for a write, `di`, then writes `start`. It polls `done`, and
reads the result in `do`.

On GTX, GTH and GTY, the user interface is the only input of the
multiplexer. On GTP, input 0, the default select, is shared by the receive initialization, which
uses the DRP for a reset workaround during its sequence, and the user interface, the rest of the
time: `DRPControl` works without logic of the design on `drp_mux.sel`, from `sys`. Input 1 is the
user interface alone; a design that selects it (as USB3-PIPE does with a register) blocks the
receive initialization and must select it only around the user accesses.

The ECP5 DCU has an equivalent serial interface, SCI, and the Gowin GTR12 a runtime register
access, UPAR. Their registers are in [Lattice ECP5 DCU](#lattice-ecp5) and
[Gowin GW5 GTR12](#gowin-gw5).

## SerWB between two FPGAs {#integ-serwb}

A SerWB link needs one PHY and one core on each side. The figure below shows the usual setup. The
SoC side maps an address region of its bus to the core in `mode="slave"`, which behaves as a
Wishbone slave.

The remote side drives its own bus with the core in `mode="master"`. The PHY in
`mode="master"`, usually on the SoC side, forwards the link clock and runs the initialization.

![SerWB between two LiteX SoCs. An access to the SerWB region of FPGA A travels over the three
cyan links and is replayed on the bus of FPGA B.](integ_serwb.svg)

The code below builds both sides with the high-speed PHY, which selects the 7-Series, UltraScale
or Efinix primitives from the device name. Both sides elaborate as written in a LiteX `SoCMini`.

```python
from litex.soc.integration.soc import SoCRegion

from liteiclink.serwb.phy  import SERWBPHY
from liteiclink.serwb.core import SERWBCore

# FPGA A (SoC side): the link appears as a bus region.
self.serwb_phy  = SERWBPHY(platform.device, platform.request("serwb"), mode="master")
self.serwb_core = SERWBCore(self.serwb_phy, sys_clk_freq, mode="slave")
self.bus.add_slave("serwb", self.serwb_core.bus,
    SoCRegion(origin=0x30000000, size=0x10000000, cached=False))

# FPGA B (remote side): the link masters the bus.
self.serwb_phy  = SERWBPHY(platform.device, platform.request("serwb"), mode="slave")
self.serwb_core = SERWBCore(self.serwb_phy, sys_clk_freq, mode="master")
self.bus.add_master("serwb", self.serwb_core.bus)
```

`SoCRegion` is cached by default: `cached=False` keeps a CPU from caching remote registers. With
a CPU, LiteX only accepts an uncached region inside the IO region of the CPU, from `0x80000000` on
VexRiscv, so the origin moves there.

The generic PHY (`liteiclink.serwb.genphy.SERWBPHY`) takes the same arguments and runs on any FPGA.
Three settings change the wire format and must match on both sides: `protocol`,
`with_flow_control` and `with_crc`. Their effects are in [SerWB architecture](#serwb-architecture).

### Clocks of the two sides {#integ-serwb-clocks}

The 7-Series and UltraScale PHYs serialize 8 bits per `sys` cycle with a `sys4x` clock and scan
their input delays. The 7-Series PHY uses `IDELAYE2`, which needs an `IDELAYCTRL` with a 200 MHz
reference clock. The UltraScale PHY uses `IDELAYE3` in tap-count mode (`COUNT`); its
`REFCLK_FREQUENCY` attribute is 200 MHz, or 300 MHz with `usp=True`, and neither the PHY nor the
bench targets instantiate an `IDELAYCTRL` for it.

The Nexys Video target generates `sys`, `sys4x` and the 200 MHz delay clock from one
PLL; copy its clock generator. The Efinix PHY takes the names of its two clocks and their ratio as
parameters (`clk`, `clk4x`, `clk_ratio`).

The link slave must run from the clock that the link master forwards. On the 7-Series and
UltraScale PHYs, the forwarded clock toggles every 5 bits, so its frequency is 8/10 of the master
`sys` clock.

A PLL on the slave regenerates `sys` and `sys4x` from it (`serdes.clocking.refclk`).
With the generic PHY, the forwarded clock is the master `sys` clock itself; the iCEBreaker of the
demonstration uses it directly as its `sys` clock.

### Addresses and register maps {#integ-serwb-map}

The remote side receives the address as the SoC side bus presents it, origin of the region
included. A memory that decodes only its low address bits, like the SRAM of the bench targets,
works as is.

A remote SoC with its own address map translates the addresses, in two steps in the
demonstration. The remote SoC adds the link as a bus master with a region
(`add_master(..., SoCRegion(origin=0, size=0x20000000))`): LiteX inserts a remapper that keeps the
offset of each access inside this 512 MiB region. A `wishbone.Remapper` then maps the offsets from
`0x10000000` to the RAM at `0x40000000`. The explicit remapper alone does not see these offsets.

The demonstration also merges the register maps of the two SoCs. The remote build exports its map
as JSON, and the SoC side build imports it at the origin of the SerWB region:

```python
# Remote side (iCEBreaker).
builder = Builder(soc, csr_csv="csr.csv", csr_json="icebreaker_soc.json")

# SoC side (ECPIX-5): remote registers, prefixed, at the SerWB region origin.
builder = Builder(soc, csr_csv="csr.csv")
builder.add_json("icebreaker_soc.json", 0x40000000, "icebreaker_soc")
```

The generated `csr.csv`, headers and JSON of the SoC side then name the remote registers too, for
example `icebreaker_soc_serwb_slave_phy_control_ready`. Build the remote side first.

### Link down and AXI {#integ-serwb-axi}

While the link is down, the core holds its logic in reset, and the SoC side core answers bus
accesses with a Wishbone error instead of stalling the bus. With the native protocol, a write
also sees an acknowledge in the same cycle, so a bus master must check the error first.

In a LiteX SoC with an AXI or AXI-Lite main bus, `add_slave` and `add_master` insert the LiteX
bridges. `AXILite2Wishbone` returns a Wishbone error as `SLVERR`, and the full AXI bridge goes
through it, so an AXI access with the link down ends at once with an error response.

## LiteX and LiteX-Boards {#integ-litex-users}

LiteX itself uses two wrappers for HDMI output on transceivers, in `litex/soc/cores/video.py`.
`VideoS7GTPHDMIPHY` drives three GTP channels with the TMDS symbols through `tx_pattern`, and
`VideoUSPGTHHDMIPHY` drives four GTH4 channels, the TMDS clock included, through `gth_params`.
LiteICLink is part of the `litex_setup.py` repository list, and the tests of these PHYs are skipped
when it is not installed.

Three LiteX-Boards targets import LiteICLink, directly or through these PHYs; the Obsidian
target adds its lanes with an `add_gtp_sfp` method. The table below gives what each one uses.

| Target | Uses | Wrapper |
|--------------------------------|----------------------------|------------------------------|
| `berkeleylab_obsidian` | GTP lanes on SFP cages | `GTPQuadPLL`, `GTP` |
| `decklink_mini_4k` | HDMI output | `VideoS7GTPHDMIPHY` |
| `modretro_m64` | HDMI output | `VideoUSPGTHHDMIPHY` |

No LiteX-Boards target uses SerWB; the targets of `bench/serwb/` are the references. Other cores
build their protocol layers on the wrappers: LiteEth, LiteSATA, the LitePCIe soft PHY and
USB3-PIPE, as [Introduction](#introduction) lists. Some of them, LiteEth and LiteSATA, reuse only
the PLL classes and the initialization sequences, around their own transceiver instances.

## Porting to a new FPGA family {#integ-porting}

A wrapper for a new family plugs into the benches, the scripts and the other cores when it
provides the interface of the existing ones. The checklist below lists it; the GTX wrapper is the
reference implementation.

- **PLL class**: takes the reference clock, its frequency and the line rate, computes the
  dividers in `compute_config`, raises `ValueError` when none fits, and prints its configuration.
- **Clock domains**: `cd_tx` and `cd_rx`, with `tx_clk_freq` and `rx_clk_freq`, each held in
  reset until its sequence completes.
- **Control signals**: `tx_enable`, `rx_enable`, `tx_ready`, `rx_ready`, `tx_polarity`,
  `rx_polarity`, `loopback` and the PRBS configuration signals.
- **Datapath**: an `encoder` and `decoders` from `litex.soc.cores.code_8b10b`, and
  `add_stream_endpoints()` creating `sink` and `source` with `data` and `ctrl`.
- **Registers**: `add_controls()` and `add_clock_cycles()` with the register names of the other
  families, so that `test_prbs.py` works unchanged.
- **Clock outputs**: `txoutclk` and `rxoutclk`, for timing constraints.
- **Tests**: an elaboration test in `test/`, as `test_serdes_xilinx.py` does for each Xilinx
  family, and a model of the initialization sequence when the family has one.

Where to go next: [Benches and examples](#examples) for complete designs, and
[Debugging](#debugging) to bring a new design up on hardware.
