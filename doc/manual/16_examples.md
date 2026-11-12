# Benches and examples {#examples}

This chapter describes the ready-made designs of `bench/`: the SerDes benches with their PRBS
script, the SerWB benches with their test script, the two-board SerWB demo and the resource
estimation script. Use them to check a board, a cable and a line rate before you integrate a
wrapper or a link in your own design.

## At a glance {#ex-glance}

Every bench is a LiteX target script. It builds a small SoC on a LiteX-Boards platform, with a
UARTBone or JTAGBone bridge (Etherbone for the main SoC of the demo) so that a host script reads
and writes the registers. Only the main
SoC of the SerWB demo has a CPU.

The SerDes benches are in `bench/serdes/` and use `test_prbs.py` on the host:

| Target | Board | Transceiver |
|---------------------------------|----------------------------------------------------------------------|-------------------------------------|
| `kc705.py` | Xilinx KC705 | GTX |
| `kcu105.py` | Xilinx KCU105 | UltraScale GTH |
| `xcu1525.py` | SQRL XCU1525 | UltraScale+ GTY |
| `sqrl_acorn.py` | SQRL Acorn CLE-215+ | GTP |
| `versa_ecp5.py` | Lattice Versa ECP5 | ECP5 DCU |
| `ecpix5.py` | LambdaConcept ECPIX-5 | ECP5 DCU |

The SerWB benches are in `bench/serwb/` and use `test_serwb.py`, except the two-board demo in
`bench/serwb/demo/`, which uses `litex_cli`:

| Target | Board | SerWB PHY |
|-----------------------------------------------------|--------------------------------------------------|-------------------------------------|
| `nexys_video.py` | Digilent Nexys Video | 7-Series |
| `t120_bga576_dev_kit.py` | Efinix Trion T120 kit | Efinix |
| `ti60_f225_dev_kit.py` | Efinix Titanium Ti60 kit | Efinix, with SerIO |
| `icebreaker.py` | iCEBreaker | Generic |
| `trellisboard.py` | TrellisBoard | Generic |
| `ulx3s.py` | ULX3S | Generic |
| `demo/ecpix5.py` | ECPIX-5, main SoC | Generic, with SerIO |
| `demo/icebreaker.py` | iCEBreaker, peripheral SoC | Generic, with SerIO |

The hardware status of each family is in the [status table](#intro-status), and the boards are
described in [Supported FPGAs and boards](#boards). The simulation script `bench/serwb/sim.py` is
described in [Simulation and tests](#simulation).

## SerDes benches {#ex-serdes}

The six SerDes benches share one structure. Each builds a PLL from a reference clock and a line
rate, prints its configuration, and instantiates one wrapper. The four Xilinx benches enable the
TX and RX buffers and turn the clock aligner off; the ECP5 benches use the DCU with 20-bit words.
Each bench then adds the stream endpoints, every control register of the wrapper
(`add_controls()`) and the clock counters (`add_clock_cycles()`).

| Target | Wrapper and PLL | Connectors | Reference clock | sys clock |
|------------------------------------|----------------------------|----------------------------|---------------------------------|---------------|
| `kc705.py` | `GTX`, CPLL or QPLL | `pcie`, `sfp`, `sma` | 125 MHz, `sgmii_clock` pins | 156 MHz |
| `kcu105.py` | `GTH3`, CPLL | `pcie`, `sfp0`, `sfp1`, `sma` | 125 MHz, looped on SMA cables | 125 MHz |
| `xcu1525.py` | `GTY`, CPLL or QPLL | `pcie`, `qsfp0`, `qsfp1`, `qsfp0+1` | 156.25 MHz, QSFP0 pins | 125 MHz |
| `sqrl_acorn.py` | `GTP`, quad PLL | `pcie` | 125 MHz from a fabric PLL | 100 MHz |
| `versa_ecp5.py` | `SerDesECP5` | `pcie`, `sma` | Pins or fabric PLL, by rate | 100 MHz |
| `ecpix5.py` | `SerDesECP5` | SATA connector | Pins or fabric PLL, by rate | 100 MHz |

Three boards need a word of explanation:

- **KCU105.** The design drives a 125 MHz clock, derived from the system clock, out of the
  `user_sma_clock` connectors. Two SMA cables bring it back to the `user_sma_mgt_refclk` inputs
  of the transceiver quad.
- **SQRL Acorn.** The board has no dedicated transceiver reference clock. A fabric PLL derives
  125 MHz from the 200 MHz oscillator, and the quad PLL takes it through the clock network with
  `refclk_from_fabric=True`. The target lowers the Vivado design rule REQP-49 to a warning for this
  path ([Xilinx 7-Series](#xilinx-7series)). Its identifier string reads "LiteSATA bench".
- **XCU1525.** With `qsfp0+1`, the target builds two lanes, `serdes0` and `serdes1`, from the
  same reference clock; the test pattern goes to `serdes0` only.

The ECP5 benches choose the reference clock from the line rate. They use the board pins, through
an `EXTREFB` input, when the board has a suitable oscillator, and a fabric PLL from the 100 MHz
oscillator otherwise. A line rate outside the table below stops the script.

| Line rate | Versa ECP5 reference clock | ECPIX-5 reference clock |
|--------------------|----------------------------------------------------------------|--------------------------------------------------------|
| 1.25 Gbps | 156.25 MHz, pins (`EXTREF0`) | 100 MHz, pins (`EXTREF1`) |
| 1.5 Gbps | 150 MHz, fabric PLL | 150 MHz, fabric PLL |
| 2.5 Gbps | 156.25 MHz, pins (`EXTREF0`) | 100 MHz, pins (`EXTREF1`) |
| 3.0 Gbps | 150 MHz, fabric PLL | 150 MHz, fabric PLL |
| 5.0 Gbps | 250 MHz, fabric PLL | 250 MHz, fabric PLL |

The ECP5 benches use 20-bit words; no bench exercises the 40-bit mode of the DCU. The Versa ECP5
bench uses channel 0 of the DCU for `pcie` and channel 1 for `sma`. The ECPIX-5 bench uses
channel 0 of DCU 1, wired to the SATA connector.

### Options {#ex-serdes-options}

| Option | Targets | Default | Meaning |
|------------------------------|-------------------------------------|---------------------|----------------------------------------------------|
| `--build` | All | off | Run the FPGA toolchain |
| `--load` | All | off | Load the bitstream into the FPGA (volatile) |
| `--connector` | All but `ecpix5.py` | `pcie` | Lane under test, from the table above |
| `--linerate` | All | `2.5e9` | Line rate in bit/s |
| `--pll` | `kc705.py`, `xcu1525.py` | `cpll` | `cpll` or `qpll` |
| `--toolchain` | ECP5 targets | `trellis` | `trellis` or `diamond` |
| `--device` | `ecpix5.py` | `85F` | FPGA model of the board: `85F` or `45F` |

Without `--build`, a target only elaborates the design. It prints the PLL configuration and writes
the gateware sources and the register map, without any vendor tool: the quickest check that a
reference clock and a line rate are possible. [Elaborate a SerDes target](#gs-serdes-elaborate)
shows the PLL configuration that the KC705 target prints this way.

The six SerDes targets elaborate this way at their default options. A line rate that the PLL
cannot reach from the reference clock stops the script with a `ValueError` before any file is
written ([SerDes wrapper architecture](#serdes-architecture)). The Efinix SerWB targets run the
Efinity interface script even without `--build`, so they need Efinity installed.

The Xilinx targets write their register map to a file named after the board (`kc705.csv`,
`kcu105.csv`, `xcu1525.csv`, `sqrl_acorn.csv`); the ECP5 targets write `csr.csv`. Give the file
to the host scripts with `--csr-csv`.

### Test pattern and LEDs {#ex-serdes-pattern}

Each bench sends a fixed pattern on its TX: a K28.5 comma in the first byte of every word and the
top bits of a free-running counter in the second byte. The comma lets the receiver align its
words; the counter changes slowly enough to be seen on LEDs.

| Target | Heartbeat LEDs: sys, tx, rx | Received data LEDs |
|---------------------------------------|------------------------------------------|-----------------------------------------------------------|
| `kc705.py`, `kcu105.py` | 0, 1, 2 | 4 to 7: bits 3:0 of the first byte |
| `sqrl_acorn.py` | 0, 1, 2 | 3: bit 0 of the first byte |
| `versa_ecp5.py` | 0, 2, 1 | 4 to 7: received counter, inverted |
| `ecpix5.py` | RGB 0, 2, 1 (green) | RGB 3 (green): counter bit 0, inverted |
| `xcu1525.py` | None | None |

A heartbeat LED toggles with bit 26 of a counter in its clock domain: a still LED points to a
missing clock. The ECP5 benches look for the K28.5 in the received word and show the byte after it,
so their data LEDs count when the link works.

The Xilinx benches show the first received byte, which holds the K28.5 once the transceiver has
aligned to the comma. On a working link these LEDs show a fixed pattern, not the counter that the
comment of the target announces.

The Versa ECP5, ECPIX-5 and XCU1525 benches also include a LiteScope analyzer on the
initialization state machines (`analyzer.csv`); [Debugging](#debugging) shows how to use it.
`litescope_cli` reads `csr.csv` by default: on the XCU1525, pass `--csr-csv xcu1525.csv` to it.

## PRBS test: test_prbs.py {#ex-test-prbs}

`bench/serdes/test_prbs.py` runs a PRBS test on a lane through the registers of the wrapper. It
works with any design whose wrapper is named `serdes<N>` and has the PRBS and clock-counter
registers, the benches included. Start the bridge to the board first, then the script:

```sh
# Build and load the KC705 bench on its SFP cage, at 2.5 Gbps.
python3 bench/serdes/kc705.py --connector sfp --linerate 2.5e9 --build --load
# Bridge to the UARTBone of the bench (adapt the serial port).
litex_server --uart --uart-port=/dev/ttyUSBX
# PRBS31 for 60 s in near-end PMA loopback, then through the SFP module and cable.
python3 bench/serdes/test_prbs.py --csr-csv kc705.csv --mode prbs31 --linerate 2.5e9 --loopback
python3 bench/serdes/test_prbs.py --csr-csv kc705.csv --mode prbs31 --linerate 2.5e9
```

The script runs four steps:

1. It measures the TX and RX user clocks over 2 s with `clock_latch`, `clock_tx_cycles` and
   `clock_rx_cycles`, and prints them in MHz. Both must read the line rate divided by the word
   width: 125 MHz at 2.5 Gbps with 20-bit words.
2. It enables the PRBS generator and checker in the chosen mode. It also enables the square wave
   on TX with `--square-wave`, and the near-end PMA loopback (`loopback` = `0b010`) with
   `--loopback`.
3. Every 0.5 s, it pauses the error counter, reads `rx_prbs_errors`, releases the counter, and
   prints the errors of the interval, the elapsed time and a BER column.
4. At the end of `--duration`, or on Ctrl-C, it disables the PRBS, the square wave and the
   loopback.

| Option | Default | Meaning |
|---------------------------------|--------------------|---------------------------------------------------------------------------------------|
| `--csr-csv` | `csr.csv` | Register map of the design |
| `--port` | `1234` | Port of `litex_server` |
| `--serdes` | `0` | Wrapper under test: `serdes<N>` |
| `--mode` | `prbs7` | `prbs7`, `prbs15` or `prbs31` |
| `--square-wave` | off | Send the square wave instead of data, for a scope |
| `--duration` | `60` | Test duration in seconds |
| `--linerate` | `5e9` | Line rate used in the BER column only |
| `--loopback` | off | Enable the near-end PMA loopback during the test |

The BER column divides the total errors, without the first interval, by the elapsed time, first
interval included, and by `--linerate`. It is indicative only. Give the line rate of the design,
since the default is 5 Gbps.

The error register counts the words that hold at least one wrong bit, not the bits ([PRBS and bit
error ratio](#primer-prbs)). A run without error prints 0: what it proves is the bound computed in
the primer, below 3 divided by the number of bits tested.

The script has three limitations:

- It prints "Disabling Clock Aligner" but writes 0, which keeps the clock aligner enabled. The
  benches build without the clock aligner, so this matters only for a design that has one.
- On ECP5, the `0b010` that `--loopback` writes requests the DCU loopback, which is not known to
  give a working loopback ([Control registers](#ecp5-controls)).
- A user clock that does not run is printed as `2147.484MHz`: two equal counter reads are taken
  as a full wrap of the 32-bit counter.
- The GTR12 wrapper has no PRBS registers, so the script does not apply to it.

## SerWB benches {#ex-serwb}

The SerWB board benches put both ends of a link in one FPGA. A link master PHY with a core exposes a
bus slave to the SoC; a link slave PHY with a core masters a small SRAM.

The three wires of the link leave the FPGA and come back through a connector, a cable or the board.
The host reaches the SRAM at `0x30000000` of the SoC bus, across the link.

| Target | PHY and clocks | Link wiring | Bridge |
|-------------------------------------------------------|---------------------------------------|--------------------------|--------------------|
| `nexys_video.py` | 7-Series, sys 125 MHz, sys4x 500 MHz | HDMI output to HDMI input | UARTBone |
| `t120_bga576_dev_kit.py` | Efinix Trion, sys 100 MHz, ratio 1:4 | LVDS pins of the kit | UARTBone |
| `ti60_f225_dev_kit.py` | Efinix Titanium, sys 300 MHz, ratio 1:4 | LVDS pins of the kit | JTAGBone |
| `icebreaker.py` | Generic, sys 24 MHz | PMOD 1A to PMOD 1B | UARTBone |
| `trellisboard.py` | Generic, sys 50 MHz | PMOD A to PMOD B | UARTBone |
| `ulx3s.py` | Generic, sys 50 MHz | GPIO pins of the board | UARTBone |

LEDs 0 and 1 show the `ready` and `error` of the master PHY, LEDs 2 and 3 those of the slave PHY,
on the Nexys Video, iCEBreaker, TrellisBoard and ULX3S. The Titanium bench adds a SerIO on port 1
of both cores; its slave side increments its 32-bit SerIO input word as a counter, about every
10 ms.

| Option | Targets | Meaning |
|----------------------------------------|-----------------------------------------------------|-----------------------------------------------|
| `--build`, `--load` | All | Run the toolchain, load the bitstream |
| `--low-speed` | `nexys_video.py` | Generic PHY instead of the 7-Series PHY |
| `--with-analyzer` | All but `icebreaker.py` | LiteScope analyzer on both PHYs |
| `--loopback` | `icebreaker.py`, `trellisboard.py`, `ulx3s.py` | Link inside the FPGA, no pins |
| `--flash` | Efinix targets | Write the bitstream to the flash |
| `--lvds-rx-delay` | `t120_bga576_dev_kit.py` | Static delay of the data inputs, in taps, on both PHYs |
| `--lvds-clk-delay` | `t120_bga576_dev_kit.py` | Static delay of the clock inputs, in taps, on both PHYs |

### SerWB test: test_serwb.py {#ex-test-serwb}

`bench/serwb/test_serwb.py` drives a SerWB bench through `litex_server`. It expects the register
names of the benches (`serwb_master_phy_control_*` and `serwb_slave_phy_control_*`) and the
`serwb` memory region. It imports LiteScope, which must be installed.

```sh
python3 bench/serwb/icebreaker.py --build --load
litex_server --uart --uart-port=/dev/ttyUSBX
python3 bench/serwb/test_serwb.py --ident --init --sram --access
```

| Option | Action |
|----------------------|----------------------------------------------------------------------------------------------------------------------|
| `--ident` | Read and print the identifier string of the SoC |
| `--init` | Reset the link master, wait up to 4 s for `ready` or `error`, print the state of both PHYs |
| `--sram` | Write a counter to the first 128 words of the SRAM and dump them, then write four known words and dump the same 128 words |
| `--access` | Write `0x12345678` at offset `0x100` of the SRAM over the link and read it back |
| `--port` | Port of `litex_server` (default `1234`) |

A working link prints "Success." after `--init`, then `ready: 1` and `error: 0` for both sides.
"Success." only means that the master left its wait: a master in error also prints it, and a
timeout prints "Failed." without the state of the PHYs. The
dumps of `--sram` and the read of `--access` match what was written.

The generic PHY has no delay registers, so `--init` prints only its shift (`bitslip`).
[Debugging](#debugging) explains how to read the delays, the shift and the error counters when the
link does not come up.

## ECPIX-5 and iCEBreaker demo {#ex-serwb-demo}

The demo in `bench/serwb/demo/` links two boards with the generic PHY, on three PMOD wires, at a
system clock of 25 MHz. The ECPIX-5 runs the main SoC: a VexRiscv CPU, Etherbone on its Ethernet
port and the link master.

The iCEBreaker runs a peripheral SoC without CPU, clocked by the link clock it receives, with 1 KiB
of RAM and the link slave as its bus master.

The ECPIX-5 maps the link at `0x40000000`, with a size of 512 MiB. The iCEBreaker keeps the
offset of each access in its 512 MiB master region, then remaps the offsets from `0x10000000` to
its RAM at `0x40000000`, so its RAM appears at `0x50000000` on the ECPIX-5
([Addresses and register maps](#integ-serwb-map)). Two SerIOs, on ports 1 and 2, carry counters of the iCEBreaker to two
red LEDs of the ECPIX-5.

Build the iCEBreaker first: the ECPIX-5 target reads its register map (`icebreaker_soc.json`) and
merges it into its own exports, so the host sees the registers of both FPGAs by name.

```sh
cd bench/serwb/demo
python3 icebreaker.py --build --load
python3 ecpix5.py --build --load
# Etherbone to the ECPIX-5 at its default IP address, 192.168.1.50.
litex_server --udp
# Registers of both boards, prefixed with icebreaker_soc_ for the iCEBreaker.
litex_cli --regs
# RAM of the iCEBreaker, over Ethernet then SerWB.
litex_cli --write 0x50000000 0x12345678
litex_cli --read 0x50000000
```

The link is up when `serwb_master_phy_control_ready` and
`icebreaker_soc_serwb_slave_phy_control_ready` read 1; the LEDs driven by `ready` on both boards
light up. The read returns the value written. The README of the demo shows photos of the setup and
a register dump that predates the link health registers.

## Resource estimation: resources.py {#ex-resources}

`bench/serwb/resources.py` synthesizes a SerWB PHY and core with Yosys (`synth_xilinx`, 7-Series
cells) and counts LUTs, flip-flops, LUT RAMs, shift registers and carry chains. It compares
configurations and tracks the cost of a change; the counts are estimates, not vendor results.

| Option | Default | Meaning |
|-------------------------|------------------|-------------------------------------------------------------------------------------------------|
| `--phy` | `all` | `gen`, `s7`, `ku`, or a comma-separated list |
| `--options` | `all` | `default`, `flow_control`, `crc`, `etherbone`, `etherbone_fc`, `prbs` |
| `--mode` | `master` | PHY side: `master` (the core exposes a bus slave) or `slave` |
| `--json` | none | Save the results to a file |
| `--compare` | none | Print the difference with a saved file |

The script reads the cell counts of the Yosys statistics, printed after the cell name by older
Yosys versions and before it by recent ones. The relative figures of the native protocol are in
[Performance and resources](#performance).

## In the code {#ex-code}

| Path | Content |
|-----------------------------------------|---------------------------------------------------------------------------------------------------|
| `bench/serdes/*.py` | SerDes bench targets and `test_prbs.py` |
| `bench/serwb/*.py` | SerWB bench targets, `test_serwb.py`, `sim.py`, `resources.py` |
| `bench/serwb/demo/` | ECPIX-5 and iCEBreaker demo and its README |

## Where to go next {#ex-next}

[Getting started](#getting-started) walks through a first bench, [Debugging](#debugging) reads
the registers when a bench fails, and [Integration](#integration) moves a wrapper or a link into
your own SoC.
