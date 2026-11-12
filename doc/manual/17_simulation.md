# Simulation and tests {#simulation}

This chapter describes how LiteICLink is checked before a bitstream is built: the unit tests of
the SerDes wrappers and of SerWB, the Verilator simulation of a SerWB link, and the continuous
integration. It is for contributors who change the gateware and for integrators who want to
reproduce a SerWB problem without a board.

## Verification strategy {#sim-strategy}

The transceivers are hard blocks: the vendors model them for their own simulators, and the project
has no model of a GT channel, an ECP5 DCU or a GTR12 quad.

The SerDes tests therefore check what the wrappers compute and generate, and the logic around the
primitive. SerWB is plain logic, so its tests run the whole link, from bus accesses down to the bits
on the wires.

| Level | What runs | Model of the other side | Simulator |
|------------------------------|--------------------------------------------------|--------------------------------------|----------------------|
| SerDes PLL solver | Dividers for common reference clocks and rates | None | None |
| SerDes elaboration | PLL, wrapper, endpoints and registers to Verilog | None | None |
| SerDes init and aligner | Init state machines, clock aligner | Behavioural transceiver signals | Migen |
| ECP5 and GTR12 logic | Word aligner, gearboxes, decoder, wiring | Python stimulus | Migen |
| SerWB init and health | Calibration, PRBS test, counters, retrain | SerDes models | Migen |
| SerWB packets and link | Packet layer, both cores, protocols, faults | Word-level and 8b/10b PHY models | Migen |
| SerWB bit level | Two generic PHYs, cores and SRAM | Real PHYs back to back | Verilator |
| Hardware | Benches of `bench/` | The board, cable or module | |

A SerDes change therefore needs a run on hardware before it is trusted: the tests prove that the
wrapper elaborates and sequences its resets, not that the link carries bits
([Benches and examples](#examples)).

## Test files {#sim-tests}

The tests are in `test/`, one file per area, and run with pytest.

| File | What it checks |
|-----------------------------------------------------------------|---------------------------------------------------------------------------|
| `test_serdes_pll.py` | Xilinx PLL dividers give the line rate with the VCO in range; impossible rates are rejected; `print(pll)` works |
| `test_serdes_qpll.py` | Reference input of the GTH4 and GTY quad PLLs, dedicated or from the fabric |
| `test_gty_qpll_config.py` | GTY QPLL0 or QPLL1 choice, fractional feedback, `qpll_params` |
| `test_serdes_xilinx.py` | Six Xilinx families with CPLL and QPLL to Verilog: widths, registers, external clocks, clock dividers, GTP fabric reference clock, GTY raw symbols and rate |
| `test_serdes_init.py` | Init state machines of every Xilinx family: done, restart, watchdog, GTP DRP sequence |
| `test_serdes_gtp_drp.py` | GTP user DRP: `DRPControl` accesses with the default mux select, shared with the RX init DRP sequence |
| `test_serdes_clock_aligner.py` | Clock aligner: aligned, no comma, errors after ready, disable |
| `test_serdes_ecp5.py` | ECP5: OOB opt-in, word aligner, 5 Gbps PLL, 40-bit mode and gearboxes |
| `test_serdes_gtr12.py` | GTR12: primitives, tie-offs, lane wiring, quad configuration, decoder, UPAR nets, Verilog |
| `test_serwb_init.py` | SerWB init: delay and shift scans, Efinix phase search, generic PHY, control registers |
| `test_serwb_health.py` | SerWB link health: PRBS link test, link drops, auto retrain |
| `test_serwb_packet.py` | Packetizer and depacketizer: CRC, oversized and zero-length packets, timeout |
| `test_serwb_core.py` | Scrambler, and Wishbone accesses through a core and a fake PHY |
| `test_serwb_link.py` | Both cores end to end: native and Etherbone protocols, flow control, CRC, SerIO, AXI-Lite, recovery, bit errors |
| `test_serwb_sim.py` | `bench/serwb/sim.py --test` in six configurations, with Verilator |

The init tests run each state machine against a behavioural model of the transceiver signals: the
PLL locks some cycles after its reset is released, reset done follows the channel reset, and the
buffer bypass alignment completes a few cycles after its requests. A model that never raises reset
done checks the watchdog.

The SerWB link tests use two PHY models. The word-level model carries one 32-bit word every few
cycles, without backpressure, as the real PHYs do. The 8b/10b model runs the real TX and RX
datapaths over an 8-bit parallel wire, so K characters, flow control and bit errors are exercised.

## Running the tests {#sim-run}

```sh
# Every test, as the continuous integration runs them.
python3 -m pytest -v
# One area.
python3 -m pytest -v test/test_serwb_link.py
# Skip the slowest SerWB tests during quick local iterations.
LITEICLINK_QUICK_TESTS=1 python3 -m pytest -v
```

`LITEICLINK_QUICK_TESTS=1` skips the tests marked slow in `test_serwb_link.py`. `test_serwb_sim.py`
needs Verilator and is skipped when it is not installed; the other tests need only LiteX and its
dependencies, LiteEth included.

## SerWB Verilator simulation {#sim-serwb}

`bench/serwb/sim.py` builds a simulated LiteX SoC with both ends of a link: two generic PHYs wired
back to back, the two cores and an SRAM behind the remote side. Verilator runs it bit by bit, with
the same gateware as on an FPGA. The figure below shows the SoC.

![SerWB simulation. The tx and rx wires of the two generic PHYs are connected inside the simulated
SoC; the host, through Etherbone in the interactive mode, or a test state machine masters the SoC
side, and the SRAM sits behind the remote side.](sim_serwb.svg)

The script has three modes:

| Mode | Option | Design | Bus master |
|---------------------------|------------------|--------------------------------------------------------|---------------------------------------|
| Self-checking | `--test` | Link, cores and SRAM, no CPU | A test state machine |
| Interactive | none | Link, cores, SRAM, Etherbone, optional SerIO | The host, over a TAP interface |
| Minimal | `--min` | The two PHYs only, counters both ways | Counter generators |

**Self-checking mode.** A state machine waits for both PHYs to be ready, writes 32 words of a
pattern to the remote SRAM, reads them back and compares. It prints `SERWB TEST: PASS` with the
number of cycles, or `SERWB TEST: FAIL` with the error count or a timeout, and ends the simulation.
This is what `test_serwb_sim.py` runs. [A SerWB link in simulation](#gs-serwb-sim) shows the
output of a run and what its cycle count includes.

| Option | Mode | Meaning |
|-----------------------------------------------|-----------------------|----------------------------------------------------------------------|
| `--protocol` | `--test` | `native` (default) or `etherbone` (deprecated) |
| `--with-flow-control` | `--test` | Enable the flow control of both PHYs |
| `--with-crc` | `--test` | Enable the CRC of both cores |
| `--sram-latency` | `--test` | Delay the SRAM acknowledge by this number of cycles |
| `--with-serio` | Interactive | Add a SerIO on both cores |
| `--with-debug` | Interactive | Print the latencies of each access, in cycles |
| `--write-stress` | Interactive | Add a bus master that writes without pause, with a slow SRAM |
| `--read-stress` | Interactive | Same with reads |
| `--alt-stress` | Interactive | Alternate four writes and four reads |
| `--trace` | All | Write a waveform; `--trace-start`, `--trace-end` in cycles |
| `--trace-fst` | All | FST waveform instead of VCD |
| `--output-dir` | All | Build directory |

**Interactive mode.** The SoC adds Etherbone on a simulated Ethernet port, bound to the TAP
interface `tap0` with the host at 192.168.1.100, and the SoC at the LiteX default address. The host
then reaches the SRAM with `litex_server --udp` and `litex_cli` or `test_serwb.py`. Creating the
TAP interface needs the rights of the network administrator.

**Minimal mode.** Only the two PHYs, each sending a counter to the other. The simulation prints the
state changes of both initialization state machines, so it shows the calibration of the link step by
step.

After both PHYs reach `READY`, each counter checker prints "check started" at the first counter
word, then a "check error" line for each word out of sequence; a working link prints none.
`test/test_serwb_sim_min.py` runs this mode. The self-checking mode is the reference for the bus
traffic.

The simulated system clock is 1 MHz and the PHYs use a short initialization timeout
(`init_timeout=128`), so the link comes up in a few thousand cycles. Rates and latencies in cycles
are those of the generic PHY, one bit per system clock cycle, not those of a high-speed PHY.

## Continuous integration {#sim-ci}

Two workflows run on every push and pull request:

- `.github/workflows/ci.yml` installs LiteX with `litex_setup.py`, builds a pinned Verilator
  (cached), checks that the register appendix and its snapshots are up to date, then runs
  `python3 -m pytest -v` with Python 3.9.
- `.github/workflows/doc.yml` checks the figures and the sources of this manual, and builds it
  every night ([Maintaining this manual](#maintenance)).

The register appendix check runs the two commands below; it fails when a register changes in the
gateware and the appendix was not regenerated ([Register reference](#registers)).

```sh
python3 doc/manual/tools/csr_doc.py dump --check
python3 doc/manual/tools/csr_doc.py generate --check
```

## Checking a change for the other cores {#sim-downstream}

The SerDes wrappers are shared: LiteEth, LiteSATA, LitePCIe, USB3-PIPE and the HDMI PHYs of LiteX
build on them ([users of LiteICLink](#intro-users)). A change to a PLL, an init state machine or a
default parameter reaches all of them. Before merging such a change:

- run the test suites of the cores that use the changed family, with this version of LiteICLink
  installed;
- state in the pull request which families and line rates were rebuilt, and which ran on
  hardware;
- keep parameter names and defaults: the other cores pass them by name.

## In the code {#sim-code}

| Path | Content |
|-----------------------------------------------------------------|---------------------------------------------------------------------------|
| `test/test_serdes_*.py` | SerDes tests |
| `test/test_gty_qpll_config.py` | GTY QPLL configuration tests |
| `test/test_serwb_*.py` | SerWB tests |
| `bench/serwb/sim.py` | Verilator simulation of a SerWB link |
| `.github/workflows/ci.yml` | Continuous integration |

## Where to go next {#sim-next}

[Debugging](#debugging) takes over when the board does not behave like the simulation, and
[Benches and examples](#examples) describes the hardware benches.
