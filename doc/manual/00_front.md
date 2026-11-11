```{=latex}
\frontmatter
```

# About this document {.unnumbered}

This manual is the reference of LiteICLink, the LiteX library of serial links between chips. It
covers its two halves: the SerDes wrappers, which drive the high-speed transceivers of Xilinx,
Lattice and Gowin FPGAs with one Python interface, and SerWB, which extends a Wishbone bus to a
second FPGA over three links on regular IOs.

No prior knowledge of transceivers is assumed. Part I explains serial links, FPGA transceivers
and Wishbone over serial in plain words, and points at the module that implements each mechanism.
Part II describes the SerDes wrappers, family by family. Part III describes SerWB: its
architecture, its PHYs and its protocol.

Part IV is the user guide: the supported FPGAs and boards, the first bench, the integration in a
design, the benches, the simulation and the debugging. The appendices hold the register reference,
the build options, the glossary, the performance figures and the maintenance notes of the manual.

| If you want to | Read |
|------------------------------------------|----------------------------------------------|
| Know what the project does and does not | [Introduction](#introduction) |
| Check the hardware status of a family | [Hardware status](#intro-status) |
| Learn how a serial link works | [Serial links in a nutshell](#serial-primer) |
| Learn how an FPGA transceiver is organised | [FPGA transceivers](#transceiver-primer) |
| Check that your FPGA or board is supported | [Supported FPGAs and boards](#boards) |
| Run a first PRBS or SerWB bench | [Getting started](#getting-started) |
| Add a transceiver or SerWB to your design | [Integration](#integration) |
| Understand a link that does not come up | [Debugging](#debugging) |

The manual follows the repository: every name and default is taken from the source code, and
every hardware result from a public commit or document. The register appendix and the figures are
generated from the repository; the chapters are written by hand. When the code and the manual
disagree, the code is right and the manual has a bug.

## Conventions {.unnumbered}

- Numbers: `0x` prefix for hexadecimal, `0b` for binary. Bit fields are written `[msb:lsb]`.
- Rates: line rates in Gbps and Mbps; clock frequencies in MHz; sizes in KiB. A bit error ratio
  is given as an error count over a number of bits; a bound on it is given only when computed, and
  labelled as computed.
- *Transceiver* is the hard serial block of the FPGA (GTP, GTX, GTH, GTY, DCU, GTR12); *SerDes
  wrapper* is the LiteICLink class that drives it. *Reference clock* is the input clock of a
  transceiver PLL; *user clock* is the parallel-side clock of a channel, the `tx` and `rx` clock
  domains of a wrapper.
- In SerWB, *link master* and *link slave* are the roles of the PHYs (the link master drives the
  clock and runs the initialization); *SoC side* and *remote side* are the roles of the cores (the
  SoC side exposes a bus slave, the remote side drives a bus master).
- *Register* is a control and status register of the LiteX CSR bus, named as in `csr.csv`, such as
  `serdes0_rx_prbs_errors`.
- `<target>` stands for the name of a board target script of `bench/serdes/` or `bench/serwb/`
  without its extension, such as `kc705` or `nexys_video`.
- Paths are relative to the repository root. Shell commands are given for a Linux host.
- When the text names a source file, it is the file to open to check or extend the fact.
