# Getting started {#getting-started}

This chapter takes you from a fresh machine to a first SerDes link and a first SerWB link. It
checks the installation with the unit tests and a SerWB simulation, elaborates a SerDes target
without vendor tools, then builds a bench, runs a PRBS test in loopback and initializes a SerWB
link on a board.

Each step gives the commands, what you should see, and what to check when you do
not.

## Prerequisites {#gs-install}

LiteICLink is a Python package built on Migen and LiteX. Install LiteX by following the
installation page of its wiki: the `standard` configuration of `litex_setup.py` installs Migen,
LiteX, LiteX-Boards and the cores, LiteICLink included. LiteICLink imports LiteEth for the SerWB
packet CRC and the Etherbone protocol, and LiteScope for the bench analyzers, both part of the
same installation.

```sh
./litex_setup.py --init --install --config=standard
```

To work on a clone of the repository, install it in development mode:

```sh
git clone https://github.com/enjoy-digital/liteiclink
cd liteiclink
pip3 install -e .
```

The other tools depend on what you run. The table below lists them.

| Task | Tools |
|-----------------------------------|-------------------------------------------------------|
| Unit tests | Python 3, pytest |
| SerWB simulation (`sim.py`) | Verilator, a C++ compiler, libevent, json-c |
| AMD 7-Series, UltraScale(+) | Vivado |
| Lattice ECP5 | Yosys, nextpnr-ecp5 and Trellis, or Diamond |
| Lattice iCE40 | Yosys, nextpnr-ice40 and IceStorm |
| Efinix Trion, Titanium | Efinity |
| Gowin GW5 | Gowin IDE |
| Register access on a board | `litex_server` and `litex_cli`, installed with LiteX |

## Step 1: check the installation {#gs-check}

The unit tests elaborate every SerDes wrapper, run the initialization sequences against
behavioural models, and run SerWB links end to end in Migen simulation. No board and no FPGA tool
is needed. The Verilator tests of SerWB are skipped when Verilator is not installed.

```sh
python3 -m pytest -v test/
# Quicker local run: skip the slowest SerWB link tests.
LITEICLINK_QUICK_TESTS=1 python3 -m pytest -v test/
```

**What you should see.** All tests pass, or are reported as skipped. The continuous integration
runs the same suite with Python 3.9.

**If not.** An import error names the missing package: `litex`, `liteeth` or `litescope`. Rerun
the LiteX installation, or `pip3 install -e .` in the clone. [Simulation and tests](#simulation)
describes the test files.

## Step 2: a SerWB link in simulation {#gs-serwb-sim}

`bench/serwb/sim.py --test` builds a small SoC with a link master and a link slave on the generic
PHY, connected bit by bit, and runs it in Verilator. A hardware Wishbone master writes a pattern
through the link to a remote SRAM, reads it back and compares.

```sh
cd bench/serwb
./sim.py --test
```

**What you should see.** Verilator compiles the SoC, then the simulation prints its verdict and
stops. The run below was made on the code described by this manual, with the default options;
the build log and the simulator banner are cut.

```console
[...]
SERWB TEST: PASS (     24599 cycles)
[...]
```

The cycle count is the number of `sys` cycles from reset to the end of the test: the link
initialization, then the 32 writes and the 32 reads.
`--protocol`, `--with-flow-control`, `--with-crc` and `--sram-latency` run the same test with
other link options, as [Simulation and tests](#simulation) lists.

**If not.** `SERWB TEST: FAIL` points to a change in the SerWB code: run the unit tests of
`test/test_serwb_*.py` first. A compiler error points to the Verilator installation.

## Step 3: elaborate a SerDes target {#gs-serdes-elaborate}

A target script without `--build` elaborates the SoC and writes its register map, with no vendor
tool. It already runs the PLL solver: an unreachable line rate stops here with a `ValueError`.
The KC705 target below drives the GTX lane of the SFP cage at the default 2.5 Gbps.

```sh
cd bench/serdes
./kc705.py --connector sfp
```

**What you should see.** The LiteX log, then the PLL configuration that the target prints with
`print(pll)`. The excerpt below comes from this command; the diagram that precedes the
configuration is cut.

```console
GTXChannelPLL
==============
[...]
  config:
  -------
    CLKIN    = 125.0MHz
    CLKOUT   = CLKIN x (N1 x N2) / M = 125.0MHz x (4 x 5) / 1
             = 2.5GHz
    LINERATE = CLKOUT x 2 / D = 2.5GHz x 2 / 2
             = 2.5GHz
```

The script also writes `kc705.csv` in the current directory: the register map for the host
scripts. Its `serdes0_` registers are the controls of the wrapper (`serdes0_tx_ready`,
`serdes0_rx_prbs_errors`, `serdes0_loopback` and the others of [Register reference](#registers)).

## Step 4: build and load the bench {#gs-serdes-build}

The figure below shows the bench. The host reaches the registers of the bench SoC through
`litex_server` and the UART bridge of the board; `test_prbs.py` drives the PRBS generator and
checker of the wrapper through these registers.

![The first SerDes bench. Host scripts reach the wrapper registers through the UART bridge; the
cyan path is the line, closed by a loopback.](gs_serdes_bench.svg)

Build with Vivado and load the bitstream through JTAG, then start the server on the serial port of
the board. `/dev/ttyUSBX` stands for that port.

```sh
./kc705.py --connector sfp --build --load
litex_server --uart --uart-port /dev/ttyUSBX
```

In another terminal, check the identifier and the ready registers of the wrapper:

```sh
litex_cli --csr-csv kc705.csv --ident
litex_cli --csr-csv kc705.csv --regs --filter serdes0_
```

**What you should see.** The identifier is `LiteICLink bench on KC705`. `serdes0_tx_ready` and
`serdes0_rx_ready` read 1: the PLL locked and the initialization sequences completed. LEDs 0 to 2
of the board blink with the `sys`, `tx` and `rx` clocks.

**If not.** No identifier means no register access: check the serial port and that the bitstream
is loaded. A ready register at 0 points to the reference clock or the PLL, the first checks of
[Debugging](#debugging).

## Step 5: PRBS in near-end loopback {#gs-serdes-loopback}

The near-end loopback returns the transmitted bits inside the transceiver, so the test runs with
nothing plugged into the cage. `test_prbs.py` measures the user clocks, enables the generator and
the checker, and prints the error count every half second.

```sh
./test_prbs.py --csr-csv kc705.csv --linerate 2.5e9 \
    --mode prbs31 --loopback --duration 60
```

`--csr-csv` must name the register map of the target, since the Xilinx targets do not write
`csr.csv`. `--linerate` only feeds the BER computation and defaults to 5 Gbps: give
the line rate of the bitstream. `--loopback` selects the near-end PMA loopback (`0b010`).

**What you should see.** The script prints `TX freq` and `RX freq`, then one line per interval:

```text
Errors: <count> / Duration: <seconds>s / BER: <ber>
```

Both clocks run at the line rate divided by the data width: 2.5 Gbps / 20 = 125 MHz here. Every
error count stays at 0, and the printed BER at `0.000e+00`. At the end, the script disables the
PRBS and the loopback.

**If not.** A clock far from the expected value points to the PLL configuration or the reference
clock. A clock that does not run is printed as `2147.484MHz`, not 0: the script reads the same
counter value twice and takes it as a full wrap of the 32-bit counter. Errors in near-end loopback point to the clocking inside the FPGA, not to the board:
[Debugging](#debugging) covers both cases.

The ECP5 wrapper has a one-bit loopback register, and `--loopback` writes `0b010` to it: on the
ECP5 targets the option leaves the loopback off. Write `serdes0_loopback` to 1 with `litex_cli`
instead, and run `test_prbs.py` without `--loopback`. This ECP5 loopback is not validated: the
code marks it `FIXME` (`serdes_ecp5.py`). If the test fails with it, use an external loopback.

## Step 6: PRBS through the connector {#gs-serdes-cable}

Plug an SFP loopback module into the cage, or the cable of the connector you built for, and run
the same test without `--loopback`. This adds the transmitter driver, the board traces, the cage
and the receiver equalization to the path.

```sh
./test_prbs.py --csr-csv kc705.csv --linerate 2.5e9 --mode prbs31 --duration 60
```

**What you should see.** The same clock lines, and error counts at 0 for the whole run.

**Reading the result.** The test checks about `duration x linerate` bits. The count is the number
of errored words (user clock cycles with at least one wrong bit), not of bit errors. The errors of
the first interval are printed but not added to the total, while its time stays in the BER
column, so that column is indicative only.

A run without error does not give a BER of zero: it
bounds it. With 0 errors in N bits, the BER is below 3/N with 95 % confidence. For 60 s at
2.5 Gbps, N is 1.5e11 bits and the bound is 2e-11 (computed from these two numbers).

**If not.** Errors here but not in near-end loopback point to the analog path: module, cable,
polarity, TX swing and cursors. `--square-wave` puts a square wave at the line rate divided by
the data width on the TX pair, for a check with an oscilloscope. [Debugging](#debugging) lists
the registers to try.

## Step 7: a SerWB link on a board {#gs-serwb-board}

The ULX3S and TrellisBoard targets connect the link master and the link slave inside the FPGA with
`--loopback`, so no wiring is needed. Build with the open source ECP5 toolchain, load, and start
the server on the serial port of the board.

```sh
cd bench/serwb
./ulx3s.py --loopback --build --load
litex_server --uart --uart-port /dev/ttyUSBX
```

`test_serwb.py` then reads the identifier, restarts the link initialization, prints the result of
each side and tests the remote SRAM:

```sh
./test_serwb.py --ident --init --sram
```

**What you should see.** An identifier that starts with `LiteICLink SerWB bench on ULX3S`,
followed by the build date and time, then `Success.` when the initialization ends, and `ready: 1`,
`error: 0` for both sides.

The generic PHY has no delay
element, so only the `bitslip` line comes before them. The SRAM test writes a 128-word counter through the link and dumps its first 32
words, then writes four patterns and dumps the same 32 words: each dump must match what was
written.

**If not.** `Failed.` comes after a 4 s timeout; the script then stops without printing the
state of each side. `Success.` only means that the sequence ended: `error: 1` on a side means it
found no valid alignment.
Without `--loopback`, check the three wires: clock to clock, master TX to slave RX, slave TX to
master RX. [Debugging](#debugging) covers the SerWB registers.

## Other boards {#gs-other-boards}

The other targets follow the same steps. The commands below build and load each one;
[Supported FPGAs and boards](#boards) gives the connectors, and [Benches and examples](#examples)
the options.

```sh
# SerDes targets, in bench/serdes/.
./kcu105.py --connector sfp0 --build --load
./xcu1525.py --connector qsfp0 --pll qpll --build --load
./sqrl_acorn.py --build --load
./versa_ecp5.py --connector sma --build --load
./ecpix5.py --linerate 5e9 --build --load
# SerWB target, in bench/serwb/.
./trellisboard.py --loopback --build --load
```

The two ECP5 SerDes targets and all the SerWB targets write their register map to `csr.csv`, the
default of the host scripts; the Xilinx SerDes targets write a file named after the board.

The Efinix targets use the LiteX argument parser, with `--build` and `--load` like the others; the
Titanium target has no UART and exposes its registers through a JTAG bridge (JTAGBone).

Where to go next: [Integration](#integration) to use the wrappers or SerWB in your own SoC, and
[Debugging](#debugging) when a step fails.
