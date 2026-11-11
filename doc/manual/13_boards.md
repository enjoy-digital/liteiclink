```{=latex}
\part{Use it}
```

```{=html}
<h1 class="part">Part IV. Use it</h1>
```

# Supported FPGAs and boards {#boards}

This chapter lists the boards that LiteICLink has a target for, with the connector each target
uses and where its reference clock comes from.

It also shows the connectors found on transceiver
boards, the ways to close a link for a first test, and what a custom board needs. The hardware
status of each FPGA family is in [Status](#intro-status); this chapter does not repeat it.

## At a glance {#boards-glance}

LiteICLink has two kinds of targets in `bench/`. A SerDes target drives one transceiver lane with
a test pattern and exposes the transceiver registers to the host. A SerWB target puts a link
master and a link slave on the same board and tests a memory through the link. Every target is a
LiteX `SoCMini`, without CPU, built on a LiteX-Boards platform; the main SoC of the two-board
demonstration is the exception, with a CPU.

| Board | FPGA | Block | Target |
|-----------------|---------------------------|--------------|----------------------------------------|
| KC705 | Kintex-7 XC7K325T | GTX | `kc705.py` |
| KCU105 | Kintex UltraScale XCKU040 | GTH (GTHE3) | `kcu105.py` |
| XCU1525 | Virtex UltraScale+ XCVU9P | GTY | `xcu1525.py` |
| Acorn CLE-215+ | Artix-7 XC7A200T | GTP | `sqrl_acorn.py` |
| Versa ECP5 | ECP5-5G LFE5UM5G-45F | DCU | `versa_ecp5.py` |
| ECPIX-5 | ECP5-5G LFE5UM5G-85F | DCU | `ecpix5.py` |
| Nexys Video | Artix-7 XC7A200T | SerWB, 7-Series | `nexys_video.py` |
| Trion T120 kit | Efinix T120F576 | SerWB, Efinix | `t120_bga576_dev_kit.py` |
| Titanium Ti60 kit | Efinix Ti60F225 | SerWB, Efinix | `ti60_f225_dev_kit.py` |
| iCEBreaker | iCE40UP5K | SerWB, generic | `icebreaker.py` |
| TrellisBoard | ECP5-5G LFE5UM5G-85F | SerWB, generic | `trellisboard.py` |
| ULX3S | ECP5 LFE5U-45F | SerWB, generic | `ulx3s.py` |

The SerDes targets are in `bench/serdes/`, the SerWB targets in `bench/serwb/`; the last column
names the SerWB PHY. The ECPIX-5 and the iCEBreaker also form the two-board SerWB demonstration
of `bench/serwb/demo/`. [Benches and examples](#examples) describes what each target does, and
[Getting started](#getting-started) runs the first ones.

## SerDes bench boards {#boards-serdes}

Each SerDes target except the ECPIX-5 selects the lane with `--connector`. All take the line rate
with `--linerate`, in bit/s (`2.5e9` for 2.5 Gbps, the default). The table below gives the lanes each target knows and its
reference clock. Pad names are those of the target or of its LiteX-Boards platform.

| Board | `--connector` | Reference clock | PLL |
|----------------|-------------------------------------|------------------------------------------|-----------------------|
| KC705 | `pcie` (default), `sfp`, `sma` | 125 MHz SGMII oscillator | CPLL, or `--pll qpll` |
| KCU105 | `pcie` (default), `sfp0`, `sfp1`, `sma` | 125 MHz from the FPGA, looped on SMA | CPLL |
| XCU1525 | `pcie` (default), `qsfp0`, `qsfp1`, `qsfp0+1` | 156.25 MHz `qsfp0_refclk` | CPLL, or `--pll qpll` |
| Acorn CLE-215+ | `pcie` only | 125 MHz from a fabric PLL | Quad PLL |
| Versa ECP5 | `pcie` (default), `sma` | `refclk` pads or a fabric PLL, by rate | DCU PLL |
| ECPIX-5 | SATA connector only | `sata_clk` pads or a fabric PLL, by rate | DCU PLL |

The notes below give what each board needs beyond the table.

- **KC705.** The reference clock is the `sgmii_clock` oscillator. The SFP lane comes from the
  platform, and the target drives `sfp_tx_disable_n` high to enable the module transmitter.
- **KCU105.** The target outputs its 125 MHz system clock on the `user_sma_clock` connectors. Two
  SMA cables must bring it back to the `user_sma_mgt_refclk` inputs, the GTH reference clock.
- **XCU1525.** The target drives the `fs` select pins of the QSFP reference clocks to `0b01`.
  `qsfp0+1` instantiates two GTY channels, `serdes0` and `serdes1`, one per cage.
- **Acorn CLE-215+.** The Acorn has no dedicated transceiver reference clock without a PCIe host.
  The target makes 125 MHz from the 200 MHz oscillator and feeds it to the GTP through the fabric
  clock network (`refclk_from_fabric=True`), as [Xilinx 7-Series](#xilinx-7series) explains.
- **ECP5 boards.** The DCU reference clock comes from the board pads at 1.25 and 2.5 Gbps, and
  from a fabric PLL at 1.5, 3.0 and 5.0 Gbps (150 MHz or 250 MHz). These five line rates are the
  only values the ECP5 targets accept.

The `pcie` lane of the Acorn target is lane 0 of the M.2 edge of the module (pins B6/A6 and
B10/A10). The LiteX Acorn Baseboard Mini routes this lane to its SATA connector: the `sata`
resource of the `sqrl_acorn` platform uses the same pins, and marks both pairs as inverted on the
Acorn.

In a loopback the two inversions cancel; towards another device, set the polarity
parameters of the wrapper.

## SerWB bench boards {#boards-serwb}

The SerWB targets put the link master and the link slave in the same FPGA. The three link signals
of each side (clock, TX, RX) leave the FPGA on a connector, and a cable or three wires bring them
back to the other side. The table below lists the connection each target expects.

| Board | Link master pins | Link slave pins | Connection | `sys` |
|------------------|-------------------------|-------------------------|----------------------------|---------|
| Nexys Video | HDMI out, RX on HDMI in | HDMI in, TX on HDMI out | HDMI cable, out to in | 125 MHz |
| Trion T120 | P2, RX on P3 | P3, TX on P2 | Cable P2 to P3, pin to pin | 100 MHz |
| Titanium Ti60 | P2 | P3 | Cable P2 to P3, pin to pin | 300 MHz |
| iCEBreaker | `PMOD1A` pins 0 to 2 | `PMOD1B` pins 0 to 2 | Three wires | 24 MHz |
| TrellisBoard | `pmoda` pins 0 to 2 | `pmodb` pins 0 to 2 | Three wires, or `--loopback` | 50 MHz |
| ULX3S | GPIO 0 to 2, P side | GPIO 0 to 2, N side | Three wires, or `--loopback` | 50 MHz |

The pin order of the generic PHY targets is clock, TX, RX. Wire clock to clock, master TX to slave
RX, and slave TX to master RX. `--loopback` connects the two sides inside the FPGA, without IOs
and without the clock wire. The iCEBreaker target parses `--loopback` but does not pass it to the
SoC, so it always uses the PMOD pins.

The Nexys Video target uses the HDMI connectors as LVDS pairs (`TMDS_33`) and drives
`serwb_enable`, the HDMI transmitter enable, high. With `--low-speed` it uses the generic PHY on
the same pins. The two Efinix targets use LVDS pairs and the `1:4` clock ratio described in
[SerWB PHYs and link initialization](#serwb-phys).

In the demonstration, the ECPIX-5 is the SoC side, with a CPU and Etherbone, and the iCEBreaker
the remote side. Three wires link the `pmod0` connector of the ECPIX-5 to `PMOD1B` of the
iCEBreaker; [Benches and examples](#examples) gives the build steps.

## Connectors for transceiver lanes {#boards-connectors}

A transceiver lane is two differential pairs, one per direction, so any connector with two
high-speed pairs can carry it. FPGA boards bring their lanes to a few connector types, and each
type has its own way to close the link for a test.

| Connector | Lanes | Loopback for a test |
|-------------------|----------------|------------------------------------------|
| SFP, SFP+, SFP28 | 1 per cage | SFP loopback module |
| QSFP | 4 per cage | QSFP loopback module |
| SMA | 4 SMA per lane | Two coaxial cables, TX to RX |
| PCIe edge, M.2 | 1 to 16 | PCIe loopback board |
| SATA | 1 | SATA cable to a second port or board |

The photos below show two boards with SFP ports. The LiteX Acorn Baseboard Mini carries the Acorn
module, two SFP ports and a SATA connector. The SPEC-A7, an open hardware Artix-7 board, has two
SFP ports and several SMA connectors.

![The LiteX Acorn Baseboard Mini with an Acorn CLE-215+ module: two SFP ports on the bracket
side and a SATA connector.](acorn_baseboard_mini.jpg){width=70%}

![The SPEC-A7 board: two SFP ports on the left and SMA connectors on the edges.](spec_a7.jpg){width=70%}

The larger LiteX Acorn Baseboard, below, has an SFP cage, an RJ45 connector and two SATA
connectors. A SATA connector on a transceiver lane is a cheap loopback point: a SATA cable
between two ports closes the link.

![The LiteX Acorn Baseboard with an Acorn CLE-215+ module: SFP cage, RJ45 and two SATA
connectors.](acorn_baseboard.jpg){width=70%}

## Closing the link for a test {#boards-loopback}

A PRBS test needs the transmitted bits to come back to a receiver. The figure below shows the two
ways to close the path on one board. The near-end loopback returns the bits inside the
transceiver and needs no hardware. The loopback module or cable returns them through the
connector, the board traces and the coupling capacitors.

![Two ways to close a SerDes link on one board. The near-end loopback stays inside the
transceiver; the cyan path goes through the connector and an external loopback.](boards_test_setups.svg)

Start with the near-end loopback: it checks the reference clock, the PLL, the initialization
sequence and the PRBS logic, independently of the board.

Then use the external loopback, which
adds the analog path. Two boards, or a board and a test instrument, come last: they add a second
clock and the frequency offset between the two ends. [Getting started](#gs-serdes-loopback) runs
this sequence; the loopback modes of each family are in
[SerDes wrapper architecture](#serdes-architecture).

## Boards used by other cores {#boards-other}

Other cores of the LiteX ecosystem drive LiteICLink wrappers on more boards. Their targets live in
those projects; the table below points to them as starting points for a new design.

| Board | FPGA and block | Used by |
|------------------------------|-----------------------------------|----------------------------------|
| Alibaba Cloud XCKU3P card | Kintex UltraScale+, GTY | USB3-PIPE, Gen 1 and Gen 2 |
| Tang Mega 138K Pro | Gowin GW5AST-138, GTR12 | USB3-PIPE |
| GW5AT-15 board | Gowin GW5AT-15, GTR12 | USB3-PIPE |
| LiteX Acorn Baseboard Mini | Artix-7, GTP on an SFP cage | USB3-PIPE |
| ECPIX-5 | ECP5-5G, DCU at 40 bits | USB3-PIPE; LiteSATA with OOB |
| Obsidian | Artix-7 XC7A35T, GTP | LiteX-Boards `berkeleylab_obsidian` |
| DeckLink Mini 4K | Artix-7 XC7A100T, GTP for HDMI | LiteX-Boards `decklink_mini_4k` |
| ModRetro M64 | Artix UltraScale+, GTH4 for HDMI | LiteX-Boards `modretro_m64` |

The Obsidian target adds GTP lanes on its SFP cages with its `add_gtp_sfp` method. The two video
targets use HDMI PHYs of LiteX built on the GTP and GTH4 wrappers, as
[LiteX and LiteX-Boards](#integ-litex-users) shows. The GW5 targets of USB3-PIPE carry the
hardware-validated GTR12 settings that [Gowin GW5 GTR12](#gowin-gw5) refers to.

## Requirements for a custom board {#boards-custom}

Any board with a transceiver lane on a connector can use the SerDes wrappers, and any board with
three free links can use SerWB. The checklists below give what the hardware must provide;
[Integration](#integration) covers the gateware.

For a SerDes lane:

- **Reference clock**: an oscillator on the dedicated transceiver clock pins of the quad, at a
  frequency the PLL can multiply to the line rate. The PLL classes raise `ValueError` when no
  setting exists, and `print(pll)` shows the chosen dividers.
- **Fabric reference clock**: a clock from the FPGA fabric works on some families, at a cost
  explained in [FPGA transceivers](#transceiver-primer). The Acorn and the ECP5 targets use one.
- **AC coupling**: the coupling capacitors that the protocol places on the line, on the board or
  inside the SFP module.
- **Polarity**: an inverted pair is fine. Set the `tx_polarity` or `rx_polarity` parameter of the
  wrapper, or the polarity registers of the families that have them.
- **Connector control pins**: the SFP `tx_disable`, the QSFP select pins or the reference clock
  enables of the board, driven by the target.
- **Register access**: a UART or JTAG link to a LiteX bridge, for the PRBS and debug scripts.

For SerWB:

- **Three links**: clock, TX and RX, as LVDS pairs for the high-speed PHYs or single-ended pins
  for the generic PHY. The link master drives the clock.
- **Clock input on the link slave**: the slave builds its clocks from the forwarded clock, as
  [SerWB between two FPGAs](#integ-serwb) explains; route it to a clock-capable pin.
- **Delay elements** on the receive pins for the 7-Series and UltraScale PHYs, which scan the
  sampling window at start-up. The 7-Series `IDELAYE2` needs an `IDELAYCTRL` and its 200 MHz
  reference clock; the UltraScale `IDELAYE3` runs in tap-count mode (`COUNT`), and neither the PHY nor the bench
  targets instantiate an `IDELAYCTRL` for it.

Where to go next: [Getting started](#getting-started) to run a first bench, and
[Integration](#integration) to add LiteICLink to your own design.
