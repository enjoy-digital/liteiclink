# FPGA transceivers {#transceiver-primer}

This chapter shows how FPGA vendors package the functions of [Serial links in a
nutshell](#serial-primer) into hard blocks. It gives the anatomy that all the transceivers share,
the names each vendor uses, the reset sequences, the register access ports and the reference
clock paths. Read it before the family chapters of Part II.

## Anatomy of a transceiver {#xcvr-anatomy}

An FPGA transceiver is a group of *channels*, each with one TX and one RX lane, that share some
PLLs. Xilinx and Gowin group four channels in a *quad*; the Lattice ECP5 groups two in a *dual*.
The figure below shows a Xilinx UltraScale quad.

![An UltraScale transceiver quad. The common block holds the two quad PLLs, fed by the reference
clocks, and each of them can clock every channel; each of the four channels has its own channel
PLL, and splits into a PCS, the digital part, and a PMA, the analog part.](xcvr_anatomy.svg)

Each channel splits into two layers. The *PMA* is the analog part: the serializer and the line
driver on TX, the equalizers, the CDR and the deserializer on RX. The *PCS* is the digital part:
8b/10b coders, comma aligner, buffers and gearboxes, PRBS, between the PMA and the fabric.

PLLs exist at two levels. A *channel PLL* serves one channel and covers the lower line rates; a
*quad PLL* sits in the common block, serves all the channels of the quad and reaches the higher
rates with a better jitter. A channel picks its TX and RX clocks from either one.

Most PCS functions can be bypassed. LiteICLink bypasses the 8b/10b coders and the PRBS of the
transceiver and does them in the fabric, so that the same logic runs on every family. It keeps
the PMA, the buffers or their bypass logic, and, where it uses them, the comma aligners.

## Vendor names {#xcvr-names}

The vendors name the same parts differently. The table below maps them for the families that
LiteICLink supports.

| Family | Transceiver | Group | Channel PLL | Quad or dual PLLs |
|--------------------------|------------------|---------|-------------|--------------------------|
| Artix-7 | GTP (`GTPE2`) | Quad | None | PLL0, PLL1 |
| Kintex-7, Virtex-7 | GTX (`GTXE2`) | Quad | CPLL | QPLL |
| Virtex-7 | GTH (`GTHE2`) | Quad | CPLL | QPLL |
| Kintex UltraScale | GTH (`GTHE3`) | Quad | CPLL | QPLL0, QPLL1 |
| UltraScale+ | GTH (`GTHE4`) | Quad | CPLL | QPLL0, QPLL1 |
| UltraScale+ | GTY (`GTYE4`) | Quad | CPLL | QPLL0, QPLL1 |
| Lattice ECP5-5G | DCU (`DCUA`) | Dual | None | TX PLL of the dual |
| Gowin GW5AST, GW5AT | GTR12 | Quad | CPLL | QPLL0, QPLL1 |

The Xilinx primitives come in pairs: a `_COMMON` primitive for the quad PLLs and a `_CHANNEL`
primitive per channel, such as `GTXE2_COMMON` and `GTXE2_CHANNEL`. The ECP5 has one `DCUA`
primitive per dual; the GW5 has one `GTR12_QUAD` primitive per quad, or `GTR12_QUADB` on the
GW5AT-15.

The vendor user guides are the reference for every Xilinx and Lattice attribute: UG482 for the
GTP, UG476 for the GTX and the 7-Series GTH, UG576 for the UltraScale and UltraScale+ GTH, UG578
for the GTY, and TN1261 for the ECP5 DCU. The GTR12 runs on hardware-validated settings given by
the caller ([Gowin GW5 GTR12](#gowin-gw5)).

## Clocks of a channel {#xcvr-clocking}

A channel has three kinds of clocks: the reference clock of its PLL, the serial clock that the PLL
derives from it, and the user clocks on which the fabric exchanges words. The figure below shows
how they connect on a Xilinx channel.

![Clocking of a Xilinx channel. The reference clock enters on dedicated pads and feeds the PLL;
the channel outputs TXOUTCLK and RXOUTCLK, which the fabric buffers and returns as the user clocks
of the channel.](xcvr_clocking.svg)

The channel outputs a TX clock and a recovered RX clock, *TXOUTCLK* and *RXOUTCLK* on Xilinx. The
design buffers them in the clock network and feeds them back to the channel as its user clocks,
*TXUSRCLK* and *RXUSRCLK*. The fabric logic that exchanges words with the channel runs on the
same buffered clocks.

When the reference clock and the user clock have the same frequency, a plain clock buffer is
enough. Otherwise the user clock is divided from TXOUTCLK: by a BUFR or a fabric PLL on 7-Series,
by the integer divider of a BUFG_GT on UltraScale.

In LiteICLink, each wrapper builds this clocking and exposes the user clocks as its `tx` and `rx`
clock domains, with `txoutclk` and `rxoutclk` for the timing constraints. [SerDes wrapper
architecture](#serdes-architecture) lists the cases.

## Reset and initialization {#xcvr-reset}

A transceiver does not work out of reset by itself. The user guides give a sequence: reset the
PLL and wait for its lock, release the channel reset and wait for its *reset done*, then, with
the buffers bypassed, run the phase alignment. The figure below shows the sequence of the Xilinx
wrappers.

![Init FSM of a Xilinx 7-Series GTX channel. Each state waits for a status of the transceiver; a
watchdog restarts the sequence from the start when a state waits too long.](xcvr_reset.svg)

The order matters: the channel must not leave reset before its PLL locks, and the user clocks
must run before the reset done. Some steps also need delays, such as the 500 ns that the GTX and
GTP FSMs leave between the end of the FPGA configuration and the first release of the channel
reset.

A step can wait forever: a reference clock missing at power-up, a PLL that loses lock, a phase
alignment that never completes. A *watchdog* timer therefore restarts the whole sequence when it
has not reached the ready state in time. The sequence also restarts on request, for example when
the user deasserts `rx_enable`.

In LiteICLink, the init FSMs live in the `*_init.py` files, with one TX and one RX FSM per
channel. The watchdog is 1 ms on GTX and 7-Series GTH, 1 ms (TX) and 4 ms (RX) on GTP.

The UltraScale and GTY FSMs use a 10 ms ready timer. The ECP5 sequence (`SerdesInit`) watches the
loss-of-lock outputs of the DCU instead.

## Register access at run time {#xcvr-drp}

Every transceiver has more attributes than ports. The attributes are set at build time, but a
port gives access to them at run time, to read a value, try a setting or apply a workaround. Each
vendor has its own:

- **DRP** (dynamic reconfiguration port), on Xilinx: an address, a 16-bit data bus, enable,
  write enable and ready, on each channel and on each common block;
- **SCI** (serial client interface), on the Lattice ECP5: an address and an 8-bit data bus, with a
  dual and a channel selection;
- **UPAR**, on Gowin GW5: the register port of a GTR12 quad, the same registers as the build-time
  configuration.

In LiteICLink, every Xilinx channel has a `drp` interface behind a DRP multiplexer, and
`DRPControl` turns a DRP into registers. The GTP RX init FSM itself uses the DRP for a reset
workaround ([Xilinx 7-Series](#xilinx-7series)). The ECP5 wrapper has an SCI controller
(`SerDesECP5SCI`), and the GTR12 wrapper a UPAR controller (`GTR12UPAR`).

## Reference clock paths {#xcvr-refclk}

A transceiver has dedicated reference clock pads near each quad, with their own input buffer
(`IBUFDS_GTE2` on 7-Series) and a dedicated route to the PLLs. This is the path the vendors
specify and the one to use whenever the board provides a clock on those pads.

Some boards have no oscillator on the transceiver clock pads. The reference clock then comes from
the fabric: a board oscillator, through a fabric PLL that derives the frequency the transceiver
needs. Xilinx transceivers have a dedicated input for it, *GTGREFCLK*, fed from the clock network.
The UG482 reserves it for test purposes, and the Vivado design rule REQP-49 flags its use.

The SQRL Acorn CLE-215+ is such a board: its GTP takes a 125 MHz clock from a fabric PLL. Through
the general interconnect, the link quality depended on the place and route; through GTGREFCLK,
every tested build was stable ([Results recorded on hardware](#perf-recorded) quotes the record).

The lesson is general: prefer the dedicated pads. When the clock must come from the fabric, use
the GTGREFCLK input rather than the general routing, and expect some build-to-build variation.

In LiteICLink, the `refclk_from_fabric` option of the GTP, UltraScale+ GTH and GTY quad PLLs
selects the GTGREFCLK input. The Acorn bench uses it and lowers REQP-49 to a warning ([Reference
clock from the fabric](#s7-gtp-refclk)).

## Where to go next {#xcvr-next}

[SerDes wrapper architecture](#serdes-architecture) describes the interface common to all the
wrappers; [Supported FPGAs and boards](#boards) lists the boards of the benches.
