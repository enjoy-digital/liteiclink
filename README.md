```
                               __   _ __      ___________   _      __
                              / /  (_) /____ /  _/ ___/ /  (_)__  / /__
                             / /__/ / __/ -_)/ // /__/ /__/ / _ \/  '_/
                            /____/_/\__/\__/___/\___/____/_/_//_/_/\_\

                                Copyright 2017-2024 / EnjoyDigital

                            Small footprint and configurable Inter-Chip
                             communication cores powered by Migen & LiteX
```

[![](https://github.com/enjoy-digital/liteiclink/workflows/ci/badge.svg)](https://github.com/enjoy-digital/liteiclink/actions) ![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg) [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/enjoy-digital/liteiclink)


[> Intro
--------
LiteICLink provides small footprint and configurable inter chip communication
cores.

LiteICLink is part of LiteX libraries whose aims are to lower entry level of
complex FPGA cores by providing simple, elegant and efficient implementations
of components used in today's SoC such as Ethernet, SATA, PCIe, SDRAM Controller...

Using Migen to describe the HDL allows the core to be highly and easily configurable.

LiteICLink can be used as LiteX library or can be integrated with your standard
design flow by generating the verilog rtl that you will use as a standard core.

[> Features
-----------
SerWB (see [doc/serwb.md](doc/serwb.md)):
  - Wishbone over 3 LVDS I/Os (high speed) or 3 Single-Ended I/Os (low speed).
  - Xilinx 7-Series (Artix7/Kintex7/Virtex7/Zynq7/Spartan7), Ultrascale(+) and Efinix Titanium/Trion high-speed PHYs.
  - Vendor agnostic low-speed PHY.
  - 8B/10B, integrated gearbox.
  - Up to 1.25Gbps linerate / 32 bits @ 31.25Mhz user interface.
  - Automatic sampling window adjustement at startup, PRBS link test.
  - Lightweight MMAP protocol with posted writes, write coalescing and remote error reporting (Etherbone protocol deprecated).
  - Multi-port packet layer (SerIO for slow I/Os, user ports).
  - Optional link level flow control (XON/XOFF) and per-packet CRC16.
  - AXI-Lite/AXI SoCs supported through LiteX bridges.

SerDes (see [doc/serdes.md](doc/serdes.md)):
  - Xilinx 7-Series GTP/GTX/GTH, UltraScale GTH, UltraScale+ GTH/GTY, Lattice ECP5 and Gowin GW5AST/GW5AT GTR12 support.
  - Automatic PLL configuration (CPLL/QPLL) from reference clock and linerate.
  - Vendor reset/init sequences, TX/RX buffer bypass (fixed latency) or buffers.
  - 8b10b encoding/decoding, 20/40-bit (80-bit on GTY) data widths.
  - PRBS7/15/31 generator/checker, loopbacks, polarity and TX driver controls through CSRs.

[> Documentation
----------------
The **Transceivers & Inter-Chip Links manual** is the reference of LiteICLink: a nightly
[PDF](https://github.com/enjoy-digital/liteiclink/releases/download/manual-nightly/liteiclink_manual.pdf)
and [HTML page](https://github.com/enjoy-digital/liteiclink/releases/download/manual-nightly/liteiclink_manual.html)
are published on the [manual-nightly](https://github.com/enjoy-digital/liteiclink/releases/tag/manual-nightly)
release. Sources in [doc/manual](doc/manual/README.md) (build: `python3 doc/manual/build.py`, outputs in
`build/doc/`; pandoc, XeLaTeX and the fonts listed in the README of `doc/manual`):

- **Understand**: [introduction](doc/manual/01_introduction.md), [serial links in a nutshell](doc/manual/02_serial_primer.md),
  [FPGA transceivers](doc/manual/03_transceiver_primer.md), [Wishbone over serial](doc/manual/04_serwb_primer.md).
- **The SerDes wrappers**: [architecture](doc/manual/05_serdes_architecture.md), [Xilinx 7-Series](doc/manual/06_xilinx_7series.md),
  [Xilinx UltraScale and UltraScale+](doc/manual/07_xilinx_ultrascale.md), [Lattice ECP5 DCU](doc/manual/08_lattice_ecp5.md),
  [Gowin GW5 GTR12](doc/manual/09_gowin_gw5.md).
- **SerWB**: [architecture](doc/manual/10_serwb_architecture.md), [PHYs and link initialization](doc/manual/11_serwb_phys.md),
  [packets and protocol](doc/manual/12_serwb_protocol.md).
- **Use it**: [supported FPGAs and boards](doc/manual/13_boards.md), [getting started](doc/manual/14_getting_started.md),
  [integration](doc/manual/15_integration.md), [benches and examples](doc/manual/16_examples.md),
  [simulation and tests](doc/manual/17_simulation.md), [debugging](doc/manual/18_debugging.md).
- **Appendices**: [registers](doc/manual/A_registers.md), [build options](doc/manual/B_build_options.md),
  [glossary](doc/manual/C_glossary.md), [performance and resources](doc/manual/D_performance.md),
  [maintaining this manual](doc/manual/E_maintenance.md) (for contributors).

[> FPGA Proven
---------------
LiteICLink is already used in commercial and open-source designs:
- Software Defined Radio boards for CPRI/JESD204B: http://enjoy-digital.fr/
- USB3 Pipe: https://github.com/enjoy-digital/usb3_pipe
- ARTIQ: https://m-labs.hk/artiq/artiq/
- and others commercial designs...

[> Possible improvements
------------------------
- add support for Altera SerDes.
- add support for Lattice Crosslink NX SerDes.
- add more documentation
- ... See below Support and consulting :)

If you want to support these features, please contact us at florent [AT]
enjoy-digital.fr.

[> Getting started
------------------
1. Install Python 3.7+ and FPGA vendor's development tools.
2. Install LiteX and the cores by following the LiteX's wiki [installation guide](https://github.com/enjoy-digital/litex/wiki/Installation).
3. You can find examples of integration of the core with LiteX in LiteX-Boards and in the bench directory
   (SerDes PRBS benches in bench/serdes, SerWB benches and demo in bench/serwb).

[> Tests
--------
Unit tests are available in ./test/.
To run all the unit tests:
```sh
$ python3 -m pytest -v test/
```

Tests can also be run individually:
```sh
$ python3 -m unittest test.test_name
```

[> License
----------
LiteICLink is released under the very permissive two-clause BSD license. Under the
terms of this license, you are authorized to use LiteICLink for closed-source
proprietary designs.
Even though we do not require you to do so, those things are awesome, so please
do them if possible:
 - tell us that you are using LiteICLink
 - cite LiteICLink in publications related to research it has helped
 - send us feedback and suggestions for improvements
 - send us bug reports when something goes wrong
 - send us the modifications and improvements you have done to LiteICLink.

[> Support and consulting
-------------------------
We love open-source hardware and like sharing our designs with others.

LiteICLink is developed and maintained by EnjoyDigital.

If you would like to know more about LiteICLink or if you are already a happy user
and would like to extend it for your needs, EnjoyDigital can provide standard
commercial support as well as consulting services.

So feel free to contact us, we'd love to work with you! (and eventually shorten
the list of the possible improvements :)

[> Contact
----------
E-mail: florent [AT] enjoy-digital.fr
