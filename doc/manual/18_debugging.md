# Debugging {#debugging}

This chapter is a method for a link that does not come up or loses data: which register to read
first, what each value means, and when to take a LiteScope capture. The first half covers the
SerDes wrappers, the second the SerWB links. The registers are listed in full in the [Register
reference](#registers).

## Method {#dbg-method}

Debug from the bottom up and from registers to traces. Each step below relies on the one before,
and each is checked by a register read, which takes seconds; a LiteScope capture comes last, on the
one signal that the registers point to.

1. **Reach the board.** Start `litex_server` and read the identifier (`litex_cli --ident`, or
   `test_serwb.py --ident`). Without it, no register read means anything.
2. **Clocks.** Check that the user clocks run at the expected frequency.
3. **Initialization.** Check `tx_ready` and `rx_ready`, or the `ready` and `error` of a SerWB PHY.
4. **Loopback inside the FPGA.** Run a PRBS test in near-end loopback: it proves the wrapper and
   the transceiver without the board.
5. **The line.** Run the same test through the connector, the cable or the module.
6. **Traffic.** Only then run the protocol or the bus accesses.

| Symptom | First registers | Section |
|-----------------------------------------|--------------------------------------------------------------|-------------------------------------|
| `ValueError` at elaboration | None: the PLL solver | PLL configuration |
| `tx_ready` or `rx_ready` stays 0 | `clock_*_cycles`, `tx_ready`, `rx_ready` | Clocks and initialization |
| PRBS errors in loopback | `rx_prbs_errors`, `loopback` | PRBS in loopback |
| PRBS errors on the line only | `rx_polarity`, `tx_diffctrl`, cursors | The line |
| SerWB `error` is 1 | `control_delay_*`, `control_shift` | SerWB initialization |
| SerWB accesses fail after `ready` | `control_invalid_errors`, `rx_overflows` | SerWB link health |

## SerDes wrappers {#dbg-serdes}

### PLL configuration {#dbg-pll}

The PLL classes solve their dividers when the design is elaborated. When no divider set gives the
line rate from the reference clock within the VCO ranges, the constructor raises a `ValueError` and
the build stops; the ECP5 PLL lists the reference clocks that would work. The targets print the
PLL (`print(pll)`) with its dividers, as in [Benches and examples](#ex-serdes-options).

A design that elaborates can still fail to lock if the board does not deliver the reference clock
given to the PLL.

Check the frequency of the oscillator, its enable or select pins (the XCU1525 bench drives
`qsfp0_fs`, the Versa ECP5 bench `refclk_en`), and that the clock reaches the quad of the lane. On
the Acorn, the fabric reference clock path is the one to use ([Xilinx 7-Series](#xilinx-7series)).

### Clocks and initialization {#dbg-init}

The wrappers count the cycles of their TX and RX user clocks. Writing `clock_latch` copies both
counters into `clock_tx_cycles` and `clock_rx_cycles`; two latches some seconds apart give the
frequencies, which `test_prbs.py` prints first ([PRBS test](#ex-test-prbs)).

The two frequencies must equal the line rate divided by the word width. Read the two counters only: they are
written by the hardware and a software write is overwritten at the next latch.

```sh
litex_server --uart --uart-port=/dev/ttyUSBX
# Wrapper state, from the register map of the build.
litex_cli --csr-csv kc705.csv --regs --filter serdes0
```

| Observation | Meaning | Next step |
|-----------------------------------------|-----------------------------------------------|----------------------------------------------------|
| TX clock stopped (two equal `clock_tx_cycles` reads; `test_prbs.py` prints 2147.484 MHz) | No PLL lock, or TX held in reset | Reference clock, `tx_enable` |
| TX right, `tx_ready` 0 | TX init loops on its watchdog | Reference clock quality, PLL choice |
| `tx_ready` 1, `rx_ready` 0 | RX init or clock aligner not done | Signal at the RX, clock aligner |
| Both ready, RX clock off by ppm | Normal without a common reference | Nothing |

The init state machines of the Xilinx wrappers follow the vendor reset sequence and restart it when
a step does not complete within its watchdog (1 ms on GTP TX, GTX and GTH 7-Series, 4 ms on GTP RX,
10 ms on the UltraScale GTH and the GTY).

A ready flag that never rises therefore hides a sequence that runs again and again. Writing 0 then 1
to `tx_enable` or `rx_enable` restarts a sequence by hand.

With the clock aligner (`clock_aligner=True`, the default of the wrappers), `rx_ready` also waits
for the comma: the aligner restarts the RX until the K28.5 arrives at bits 9:0 of the word. A link
whose far end sends no comma never becomes ready. Writing 1 to `clock_aligner_disable` stops these
restarts; the benches build without the aligner.

The ECP5 benches and the XCU1525 bench include a LiteScope analyzer on the initialization state
machines. `litescope_cli` reads `csr.csv` by default: on the XCU1525, pass `--csr-csv
xcu1525.csv`. The ECP5 sequence is short: reset everything, wait for the TX PLL lock, wait for the RX
CDR lock, then ready; a loss of lock returns to the start, a loss of signal to the TX PLL wait.

```sh
# Signals of the analyzer of bench/serdes/ecpix5.py, then a capture on a loss of RX lock.
litescope_cli --csv analyzer.csv --list
litescope_cli --csv analyzer.csv -r serdesecp5_init_rx_lol
```

The names come from the `analyzer.csv` of the build: list them first, then trigger on the one the
registers point to.

### PRBS in loopback {#dbg-prbs}

The PRBS test of the wrapper sends a PRBS7, PRBS15 or PRBS31 sequence in place of the data and
checks it at the RX ([PRBS and bit error ratio](#primer-prbs)). Start with the near-end loopback,
which stays inside the transceiver, then move outwards.

| `loopback` | Path | Proves |
|----------------------|-----------------------|-----------------------------------------------------------------------------------------------|
| `0b001` | Near-end PCS | Wrapper logic, user clocks, PRBS (on the GTY, see [Known issues](#dbg-known)) |
| `0b010` | Near-end PMA | Serializer, CDR and deserializer of the lane |
| `0b100` | Far-end PMA | The far end sends back what it receives |
| `0b110` | Far-end PCS | Same, through the far-end PCS |

```sh
# Near-end PMA loopback, PRBS31, then the same on the line.
python3 bench/serdes/test_prbs.py --csr-csv kc705.csv --mode prbs31 --linerate 2.5e9 --loopback
python3 bench/serdes/test_prbs.py --csr-csv kc705.csv --mode prbs31 --linerate 2.5e9
```

The error register counts the RX words with at least one wrong bit, not the bit errors: one line
error can mark one or two words. It is cleared when the checker is disabled (`rx_prbs_config` at
0) and wraps at 2^32. Set the `pause` bit (bit 2 of `rx_prbs_config`) before the read, as
`test_prbs.py` does: the count stops a few cycles later, well before a read through a bridge.

Errors in near-end loopback point to the wrapper, its clocks or its constraints, not to the board.
On ECP5, `test_prbs.py --loopback` requests the DCU loopback, a reconfiguration marked FIXME in
the code ([Control registers](#ecp5-controls)).

### The line: polarity, swing and cursors {#dbg-line}

When the loopback test passes and the line test fails, the problem is between the two pins.

- **Errors on every word.** The checker never locks: a P/N swap, a PRBS mode that differs between
  the two ends, or no signal. Flip `rx_polarity` (or `tx_polarity` at the far end) while the test
  runs. A swap known from the schematic belongs in the `tx_polarity` and `rx_polarity` parameters
  of the wrapper.
- **Bursts of errors.** The eye is marginal: losses of the channel, a poor connector, or a cable or
  module out of specification. Adjust the TX of the far end: `tx_diffctrl` sets the swing,
  `tx_postcursor` and `tx_precursor` the de-emphasis ([The eye, losses and
  equalization](#primer-eye)). These registers drive the transceiver ports directly, so a change
  applies while the test runs; change one at a time and compare the error counts.
- **Rare errors.** Run longer, with PRBS31, and report the count of errored words over the bits
  tested: their ratio is a lower bound of the bit error ratio. 0 errors gives only an upper bound,
  computed as in [PRBS and bit error ratio](#primer-prbs).

`tx_produce_square_wave` replaces the data with a square wave of half a word of ones and half a word
of zeros, at the line rate divided by the word width: on a scope or a frequency counter, it checks
the TX clock and the presence of the output without a receiver.

`tx_inhibit` turns the Xilinx TX driver off, to check that the far end sees the loss of signal.

The ECP5 and GTR12 wrappers have no swing or cursor registers; their polarity is a parameter of
the wrapper ([Lattice ECP5 DCU](#lattice-ecp5), [Gowin GW5 GTR12](#gowin-gw5)).

### Transceiver registers: DRP, SCI and UPAR {#dbg-drp}

Every family gives software access to the configuration registers of the transceiver, to read an
attribute the bitstream set, or to try a value before changing the code.

**Xilinx DRP.** `DRPControl(serdes.drp)` adds the registers of the [DRP access](#reg-serdes-drp)
bank. Software writes `addr`, `di` and `we`, then any value to `start`, polls `done` and reads `do`.

The channel's DRP port goes through a multiplexer, `drp_mux`, whose select input the design drives.
On the GTP, input 0 belongs to the RX init sequence, which uses the DRP for a reset workaround, and
the user port is input 1. On the other Xilinx wrappers the user port is input 0.

The GTP wrapper never drives `drp_mux.sel`, so it stays at 0. A `DRPControl` access on a GTP whose
design does not select input 1 never completes: `done` stays at 0, which looks like a dead DRP.

**ECP5 SCI.** The `sci_reconfig_` registers of the ECP5 wrapper read and write one byte of the
channel (`sel` = 0) or dual (`sel` = 1) configuration.

Write 1 to `pause` to stop the hardware reconfiguration loop, set `adr` (and `dat_w`), write `re` or
`we`, wait for `done`, then read `dat_r`. Write 0 to `pause` afterwards: the loop applies the
polarity and loopback settings.

**GTR12 UPAR.** The `upar_` registers read and write the quad registers at run time. Write the
address to `upar_addr`, then 2 to `upar_ctrl` for a read (or the data to `upar_wrdata` and 1 for a
write), poll bit 0 of `upar_status` (`ready`), and read `upar_rddata`. Requests of the user logic
have priority; `upar_cfg` aborts an access that does not complete.

### GTR12 status {#dbg-gtr12}

The GTR12 wrapper has no PRBS, loopback or clock-counter registers. Its `status` register gives
the lane state, to read in the order of the link bring-up:

| Field | Bits | Expected on a working link |
|----------------------------------|-------------|---------------------------------------------------------------------------------------------|
| `refclk_det` | 25:24 | The reference clock of the quad is detected |
| `plls_lock` | 28:26 | The PLL of the configuration is locked |
| `pll_lock` | 0 | 1: the PLL selected by the `pll` parameter is locked |
| `rx_elecidle` | 4 | 0: a signal is present at the RX |
| `cdr_lock` | 1 | 1: the CDR has locked on the received signal |
| `aligned`, `k_lock` | 2, 3 | 1: the comma alignment of the PCS is done |

`tx_ready` and `rx_ready` of the GTR12 follow the enable and the PLL lock only, so they rise before
the RX has data. The traffic of the protocol, or the decoders, is the end-to-end check.

## SerWB links {#dbg-serwb}

### Initialization {#dbg-serwb-init}

A SerWB link calibrates itself at reset: the slave scans its input delay and its word shift on the
commas of the master, then the master does the same on the commas of the slave
([SerWB PHYs and link initialization](#serwb-phys)). The `control_` registers of both PHYs give
the result; `test_serwb.py --init` restarts the link and prints them.

| Master `ready`, `error` | Meaning | Next step |
|-------------------------------|-------------------------------------------------|------------------------------------------------------------|
| 1, 0 | Link up | Check `control_shift` and delays of both sides |
| 0, 1 | No sampling window found on the master | Master RX wire, IO standard, delays |
| 0, 0 after the timeout | The slave did not finish its calibration | Slave registers, clock wire, slave clock |

The master waits for the slave to answer with commas before it calibrates. A master that is
neither ready nor in error therefore points to the slave: read its registers if they are reachable
(same FPGA, as in the benches), check the clock wire, and on a slave that runs from the link clock,
its PLL.

On the high-speed PHYs, `control_delay_min` and `control_delay_max` bound the window where the comma
is received, and `control_delay` is its centre. A window must be at least 1/16 of the delay range
wide, otherwise the scan goes on.

A narrow window or a centre near the end of the range means a marginal timing: lower the system
clock, check the IO standard, or balance the lengths of the clock and data wires. The generic PHY
has no delays: it only scans its shift, so its rate must leave a wide margin.

### Link health {#dbg-serwb-health}

Once the link is up, four counters tell what happens to the words.

| Register | Counts | Typical cause and fix |
|-----------------------------------------------------|--------------------------------------------|-------------------------------------------|
| `control_invalid_errors` | Words with a symbol of an invalid ones count, while ready | Marginal sampling: see above |
| `control_link_drops` | Transitions from ready to not ready | Resets, retrains, unstable clock |
| `rx_overflows` (core) | Words lost on a full RX buffer, idle words included | Enable flow control on both sides |
| `crc_errors` (core) | Packets dropped on a bad CRC | Bit errors on the line |

`control_invalid_errors` restarts from 0 at each initialization. With `with_auto_retrain=True` on
the master, 16 invalid words within one window of 2^16 cycles (fixed windows, not sliding)
restart the initialization, which `control_link_drops` then counts.

Without flow control, a link has no backpressure: a remote side slower than the requests loses
words, which `rx_overflows` reveals. The idle words fill the RX buffer too, so `rx_overflows` can
count while no data is lost; a non-zero count says that the buffer filled up, not that an access
was lost. When the link is down, the SoC-side core answers every access
with a bus error instead of stalling the bus.

### PRBS link test {#dbg-serwb-prbs}

The PHYs test the link with their `control_prbs_` registers. With `with_prbs=True`, one side sends
PRBS31 words and the other checks them:

1. On the sending PHY, write 1 to `control_prbs_tx`: its traffic stops.
2. On the checking PHY, write 1 to `control_prbs_rx`, then the test duration in cycles to
   `control_prbs_cycles`, then any value to `control_prbs_start`.
3. After the duration, read `control_prbs_errors` on the checking PHY, then write 0 to both
   enables.

`control_prbs_cycles` counts `sys` cycles, not words. A word arrives every 5 `sys` cycles on the
Xilinx PHYs, every 5 x N on the Efinix PHY at a 1:N ratio and every 40 on the generic PHY: divide
by this to get the words checked, 32 bits each.

`control_prbs_errors` counts the words with one or
more wrong bits, not the bit errors. With 0 errors, the bound on the bit error ratio is computed
from the bits checked as in [PRBS and bit error ratio](#primer-prbs); with errors, the count over
the bits checked is a lower bound.

During the test the link carries no bus access. A remote side reached only through the link, as in
the two-board demo, cannot be configured during the test; run it from a design where both sides are
local, or from each FPGA's own bridge.

Without `with_prbs`, a test counts the idle words received as non-zero while no traffic is sent.
With flow control, the fill words are removed before the counter, so this test counts almost
nothing and does not measure the link: use `with_prbs`.

### LiteScope on the benches {#dbg-serwb-scope}

The SerWB benches add a LiteScope analyzer with `--with-analyzer`: the init state machines of both
PHYs, the raw received and sent words, and the comma and idle flags. Trigger on the init state
machine of the side that fails, and look at the received words during the calibration: they show
whether commas arrive, and from which delay or shift.

```sh
python3 bench/serwb/trellisboard.py --with-analyzer --build --load
litex_server --uart --uart-port=/dev/ttyUSBX
litescope_cli --csv analyzer.csv --list
```

## Known issues {#dbg-known}

The points below are limitations of the code as it stands; they are worth knowing before reading a
register.

- `clock_tx_cycles` and `clock_rx_cycles` are written by the hardware but declared as writable
  registers; never write them.
- `test_prbs.py` writes 0 when it announces that it disables the clock aligner, and its
  `--loopback` has no effect on ECP5 ([PRBS test](#ex-test-prbs)).
- On ECP5, the `tx_inhibit` and `loopback` registers drive signals marked FIXME in the code.
- `control_phase` exists on the 7-Series and UltraScale SerWB PHYs, where it has no meaning.
- `rx_prbs_errors` wraps at 2^32; read it periodically on long runs, as `test_prbs.py` does.
- `test_prbs.py` prints a stopped user clock as 2147.484 MHz, and its BER column keeps the time of
  the first interval while it drops its errors ([PRBS test](#ex-test-prbs)).
- On the GTY, the near-end PCS loopback (`0b001`) is meant to hold the CDR (`RXCDRHOLD`), but the
  condition is evaluated once at elaboration and is always false: the CDR is never held.
- On the GTP, the wrapper never drives `drp_mux.sel`, so the user DRP port is unreachable until the
  design drives it ([DRP](#dbg-drp)).
- The `clock_tx_cycles` and `clock_rx_cycles` latches copy counters of the `tx` and `rx` domains
  in `sys` without synchronization: repeat a measurement that gives an implausible value.
- With the native SerWB protocol and the link down, a write gets the acknowledge and the error in
  the same cycle.
- Several bench targets have option or pin issues, listed in [Benches and examples](#ex-serwb).

## Where to go next {#dbg-next}

[Register reference](#registers) gives every register and field, [Benches and
examples](#examples) the targets and host scripts, and [Simulation and tests](#simulation) the
way to reproduce a SerWB problem without a board.
