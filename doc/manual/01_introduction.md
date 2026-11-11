```{=latex}
\mainmatter
\part{Understand}
```

```{=html}
<h1 class="part">Part I. Understand</h1>
```

# Introduction {#introduction}

This chapter says what LiteICLink is, why it exists, what it does and what it does not do. It
gives the hardware status of every transceiver family and SerWB PHY, the users of the library
across LiteX, the layout of the repository and a map of the manual. Every reader should start
here.

## What the project is {#intro-what}

LiteICLink is a library of inter-chip links for LiteX, written in Python with Migen and LiteX. It
has two independent halves: the SerDes wrappers and SerWB.

The *SerDes wrappers* (`liteiclink/serdes/`) drive the high-speed transceivers of the FPGA. Each
wrapper configures the PLL from a reference clock and a line rate, and runs the vendor reset
sequence. It hands the user a parallel stream of 8b/10b characters in its own `tx` and `rx` clock
domains. Protocol cores such as LiteEth, LiteSATA, LitePCIe and USB3-PIPE build on them, as the
figure below shows.

![A SerDes wrapper between a user core and the transceiver. The user core exchanges parallel
words in the `tx` and `rx` clock domains; the wrapper encodes them, drives the transceiver and the
line to the other end, and exposes its controls as registers.](intro_overview.svg)

*SerWB* (`liteiclink/serwb/`) extends the Wishbone bus of a SoC to a second FPGA over three
links: a clock, a TX link and an RX link, each a differential pair or a single wire. It does not
need a transceiver: it runs on regular IOs, with the IO serializers of the FPGA or with plain
flip-flops.

The registers and the memory of the second FPGA then appear in the address map of the first one,
as the figure below shows.

![SerWB between two FPGAs. The SoC side core turns bus accesses into packets; the link master PHY
drives the clock and the TX link; the link slave PHY on the other FPGA feeds the remote side core,
which replays the accesses on the bus of its peripherals.](intro_serwb.svg)

LiteICLink is a library: designs instantiate its classes in a LiteX SoC or in a Migen module.
There is no standalone Verilog generator.

## Why LiteICLink {#intro-why}

Every FPGA vendor provides a wizard that generates a transceiver wrapper for one configuration.
The generated code is vendor-specific, hard to read and has to be regenerated for each change of
rate, width or reference clock. LiteICLink replaces it with Python classes that cover five
families with the same interface.

- **One interface**: every wrapper exposes the same clock domains, ready signals, stream endpoints
  and control registers, so a protocol core moves to another FPGA family by changing the wrapper
  and its PLL, not its own logic.
- **A PLL solver**: the PLL classes search the dividers for a reference clock and a line rate,
  and fail at build time with a clear message when no setting exists.
- **Vendor sequences**: the reset and initialization sequences of the user guides are written
  once, with watchdogs that restart them when a step never completes.
- **Test before the protocol**: PRBS generators and checkers, loopbacks, polarity and TX driver
  controls are registers on the Xilinx families (PRBS and loopback on the ECP5), so a link can be
  measured on a board before any protocol runs on it.

SerWB answers another need: control a second FPGA, such as an IO expander, a front-end board or a
smaller FPGA beside the main one, from the CPU of the first. It needs three links on regular IOs
and no transceiver, and the remote registers are accessed as local memory.

## Features at a glance {#intro-features}

| Area | Features |
|-----------------------|---------------------------------------------------------------------|
| Transceivers | Xilinx 7-Series GTP, GTX, GTH; UltraScale GTH; UltraScale+ GTH and GTY; Lattice ECP5-5G DCU; Gowin GW5AST and GW5AT GTR12 |
| PLLs | Channel and quad PLLs, configuration computed from the reference clock and the line rate |
| Datapath | 8b/10b in the fabric; 20 and 40-bit words; 80-bit words on GTY and GTR12; raw symbols on GTY, ECP5 and GTR12 |
| Clocking | TX and RX buffer bypass (fixed latency) or buffers; shared user clocks; fixed-latency comma alignment |
| Test and tuning | PRBS7, PRBS15, PRBS31; square wave and pattern; loopbacks; polarity; TX swing and cursors; dynamic reconfiguration port access |
| SerWB link | Wishbone over a clock and two data links; LVDS or single-ended IOs; 8b/10b; automatic sampling window and word alignment |
| SerWB PHYs | Xilinx 7-Series and UltraScale(+) IO serializers, Efinix Titanium and Trion LVDS, vendor-agnostic generic PHY |
| SerWB protocol | Native protocol with posted writes, write coalescing and remote error reporting; multi-port packets; optional CRC16 and XON/XOFF flow control |
| SerWB health | PRBS31 link test, invalid word and link drop counters, automatic retraining |
| Verification | Elaboration and state machine tests of the wrappers; word-level, 8b/10b-level and Verilator bit-level tests of SerWB |

The per-family details, such as which family has which control, are in [SerDes wrapper
architecture](#serdes-architecture).

## Hardware status {#intro-status}

This is the one status table of the manual: other chapters link here. The status words are
defined below the table. The third column names the boards and rates that a public commit or
document records, and the last one the record. [Results recorded on hardware](#perf-recorded)
quotes every record of the table.

| Block | Status | Recorded on hardware | Record |
|---------------------------|-----------------------|------------------------------------------------|------------------------|
| Artix-7 GTP | Validated on hardware | SQRL Acorn CLE-215+, 5 Gbps (USB3-PIPE) | `095d2a0`, `f29cd43`, `578074a` |
| Kintex-7 and Virtex-7 GTX | Validated on hardware | Kintex-7 board, 2.5 and 5 Gbps, up to 8 lanes (LitePCIe soft PHY) | LitePCIe doc |
| Virtex-7 GTH | Validated on hardware | Not recorded publicly | None |
| UltraScale GTH | Validated on hardware | Not recorded publicly | None |
| UltraScale+ GTH | Validated on hardware | Not recorded publicly | None |
| UltraScale+ GTY | Validated on hardware | Alibaba XCKU3P, 10 and 5 Gbps, PRBS31; Kintex UltraScale+ board, 2.5 and 5 Gbps on 1 lane, 8 Gbps up to 8 lanes (LitePCIe soft PHY) | `107782b`, LitePCIe doc |
| Lattice ECP5-5G DCU | Validated on hardware | 2.5 Gbps (board not stated); ECPIX-5, 5 Gbps, 40-bit words; Versa, 2.5 Gbps, PCI Express mode (LitePCIe soft PHY) | `37d45f3`, `939f4ad`, LitePCIe doc |
| Gowin GTR12, up to 5 Gbps | Validated on hardware | Tang Mega 138K Pro and GW5AT-15, 5 Gbps (USB3-PIPE) | `16f1e31`, `5d098d2` |
| Gowin GTR12, above 5 Gbps | In development | None | None |
| SerWB 7-Series PHY | Validated on hardware | UltraScale link master to 7-Series link slave, rate not stated | `816996a` |
| SerWB UltraScale(+) PHY | Validated on hardware | UltraScale link master to 7-Series link slave, rate not stated | `816996a` |
| SerWB Efinix PHY | Validated on hardware | Titanium, clock ratios 1:1, 1:2 and 1:4, rate not stated | `568ae16` |
| SerWB generic PHY | Validated on hardware | Two-board demo (ECPIX-5 and iCEBreaker), 50 MHz clock, 50 Mbps | `f5abc46` |

The status words mean:

- **Validated on hardware**: the maintainer states that the block works on hardware. Numbers are
  given only where a public commit or document records them.
- **In development**: the code is present, but not yet validated at that rate or for that
  feature.

"Not recorded publicly" means that no public commit or document gives a board, a rate and a
result for that block; it does not lower its status.

## Users across LiteX {#intro-users}

The wrappers are shared: a change to a wrapper reaches every core in the table below.

| User | What it uses |
|---------------------|---------------------------------------------------------------------|
| LiteEth | PLLs, init FSMs and DRP of GTX, GTH and GTY in its 1000BASE-X and 10GBASE-R PHYs |
| LiteSATA | GTP PLL and init FSMs of GTP, GTX, GTH and GTY; ECP5 wrapper with OOB signalling |
| LitePCIe | GTP, GTX and GTY wrappers and PLLs, DRP access, in its soft PCIe PHY |
| USB3-PIPE | GTP, GTX, GTY (raw symbols and rate change), ECP5 (40-bit words) and GTR12 wrappers |
| LiteX | GTP and UltraScale+ GTH HDMI PHYs of `litex.soc.cores.video` |

Each core has its own manual for the protocol layer it builds on the wrappers. LiteEth, and
LiteSATA on Xilinx, instantiate their own transceiver primitives and reuse the PLL solver and the
initialization sequences. LitePCIe, USB3-PIPE and LiteSATA on the ECP5 use the complete wrappers.

## What it is not {#intro-limits}

LiteICLink provides the physical link and stops there. It does not implement:

- a protocol layer: Ethernet, SATA, PCIe and USB 3 live in their own cores;
- a line code other than 8b/10b: 64b/66b, 128b/130b and 128b/132b users send raw symbols and do
  the coding
  in their own core (see [Line coding](#primer-coding));
- Altera transceivers or the Lattice CrossLink-NX SerDes;
- automatic tuning of the equalizers, or an eye scan: TX swing and cursors are set by registers,
  RX equalization by fixed settings of the wrapper;
- a standalone generator: there is no command that writes a Verilog core.

The Gowin GTR12 wrapper does not compute its own configuration: the caller provides
hardware-validated settings of the quad, and the PRBS, loopback and electrical controls of the
other families are absent there (see [Gowin GW5 GTR12](#gowin-gw5)).

## Repository layout {#intro-layout}

| Path | Content |
|--------------------------------------|-------------------------------------------------------|
| `liteiclink/serdes/` | SerDes wrappers per family, their PLLs and init FSMs, clock aligner, DRP helpers |
| `liteiclink/serwb/` | SerWB PHYs, datapath, packet layer, native and Etherbone protocols, core, SerIO |
| `liteiclink/transceiver.py` | Alias of `liteiclink.serdes`, kept for old designs (warns) |
| `bench/serdes/` | Board targets of the PRBS benches and the `test_prbs.py` host script |
| `bench/serwb/` | Board targets, Verilator simulation, resource estimates, `test_serwb.py`, demo |
| `test/` | Unit and system tests (pytest) |
| `doc/` | This manual (`doc/manual/`), its figures (`doc/diagrams/`), photos, developer notes |

## License and credits {#intro-license}

LiteICLink is released under the BSD 2-Clause license by Enjoy-Digital. It can be used in
closed-source designs.

LiteICLink is used by the ARTIQ project. The file headers of the clock aligner and of the GTP,
7-Series GTH and UltraScale GTH wrappers carry a 2017 copyright line with an M-Labs address. The
parameters and signals
of the ECP5 DCU primitive come from the documentation of the Yumewatari project. The attributes of
the PCI Express mode and its receiver detection sequence come from the same project, and katsuo-pcie,
a PCI Express stack in Amaranth by Vegard Storheil Eriksen, confirmed them. The other
contributors are listed in `CONTRIBUTORS` and in the history of the repository.

## How to read this manual {#intro-reading}

If transceivers are new to you, read [Serial links in a nutshell](#serial-primer) and [FPGA
transceivers](#transceiver-primer) next; if you came for SerWB, read [Wishbone over
serial](#serwb-primer). Otherwise, the table below sends each question to its chapter.

| Question | Chapter |
|-----------------------------------------------|----------------------------------------|
| How does a serial link work? | [Serial links in a nutshell](#serial-primer) |
| What is inside an FPGA transceiver? | [FPGA transceivers](#transceiver-primer) |
| How does a bus cross a serial link? | [Wishbone over serial](#serwb-primer) |
| What do all the wrappers have in common? | [SerDes wrapper architecture](#serdes-architecture) |
| How is my Xilinx transceiver configured? | [Xilinx 7-Series](#xilinx-7series), [Xilinx UltraScale](#xilinx-ultrascale) |
| How is the ECP5 or GW5 transceiver driven? | [Lattice ECP5 DCU](#lattice-ecp5), [Gowin GW5 GTR12](#gowin-gw5) |
| How is SerWB organised? | [SerWB architecture](#serwb-architecture) |
| How does a SerWB link come up? | [SerWB PHYs and link initialization](#serwb-phys) |
| What is in a SerWB packet? | [SerWB packets and protocol](#serwb-protocol) |
| Which board can I use? | [Supported FPGAs and boards](#boards) |
| How do I run a first bench? | [Getting started](#getting-started) |
| How do I add a wrapper or SerWB to my design? | [Integration](#integration) |
| What do the benches do? | [Benches and examples](#examples) |
| How do I simulate and test? | [Simulation and tests](#simulation) |
| The link does not come up: what now? | [Debugging](#debugging) |
| What does this acronym mean? | [Glossary](#glossary) |
