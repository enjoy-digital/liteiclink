```{=latex}
\part{The SerDes}
```

```{=html}
<h1 class="part">Part II. The SerDes</h1>
```

# SerDes wrapper architecture {#serdes-architecture}

A SerDes wrapper is the LiteICLink class that drives one transceiver channel and gives the user
core a simple interface: two clock domains, a ready flag and a stream of bytes with K flags.

This chapter describes what the wrappers share: the interface, the PLL solver, the clocks, the init
sequences, the datapath, the test features and the registers. The families follow in
[Xilinx 7-Series](#xilinx-7series), [Xilinx UltraScale](#xilinx-ultrascale),
[Lattice ECP5](#lattice-ecp5) and [Gowin GW5](#gowin-gw5).

The concepts (PLL, CDR, 8b/10b, comma, PRBS, loopback) are explained in
[Serial links in a nutshell](#serial-primer) and [FPGA transceivers](#transceiver-primer). The
hardware status of each family is in the [status table](#intro-status) of the introduction.

## At a glance {#sd-glance}

| Family | Wrapper | PLL classes | Widths |
|---------------------------|----------------|----------------------------------|------------|
| Artix-7 GTP | `GTP` | `GTPQuadPLL` | 20, 40 (1) |
| Kintex-7, Virtex-7 GTX | `GTX` | `GTXChannelPLL`, `GTXQuadPLL` | 20, 40 |
| Virtex-7 GTH | `GTH` | `GTHChannelPLL`, `GTHQuadPLL` | 20, 40 |
| UltraScale GTH | `GTH3` | `GTHChannelPLL`, `GTH3QuadPLL` | 20, 40 |
| UltraScale+ GTH | `GTH4` | `GTHChannelPLL`, `GTH4QuadPLL` | 20, 40 |
| UltraScale+ GTY | `GTY` | `GTYChannelPLL`, `GTYQuadPLL` | 20, 40, 80 |
| ECP5-5G DCU | `SerDesECP5` | `SerDesECP5PLL` | 20, 40 |
| GW5 GTR12 | `GTR12` | None: quad configuration | 20, 40, 80 |

(1) The GTP 40-bit width elaborates but has no recorded run ([Limitations](#s7-limitations)).

The data width counts the bits of one user-clock word on the transceiver side, after 8b/10b
encoding. A 20-bit word carries two symbols, so two bytes; a 40-bit word four bytes; an 80-bit
word eight bytes. The user clock runs at the line rate divided by the data width: 125 MHz for
5 Gbps at 40 bits.

The Xilinx and ECP5 wrappers follow the contract of this chapter. The GTR12 wrapper keeps the
interface but takes hardware-validated settings of the quad from the caller and has fewer test
features; its chapter lists the differences.

## Blocks of a wrapper {#sd-blocks}

The figure below shows the blocks of a wrapper. The 8b/10b encoder and decoders are LiteX fabric
modules, not the transceiver ones. The transceiver moves 10-bit symbols as they are, which lets
the same wrapper also send PRBS, a square wave or the raw symbols of another line code.

![Blocks of a SerDes wrapper: the user core writes bytes and K flags into the fabric encoder, the
TX source mux picks what goes to TXDATA, the decoders turn RXDATA back into bytes. The init FSMs
reset the channel and its PLL. The cyan path is the transmit data.](sd_blocks.svg)

The wrapper owns the init state machines and the clock buffers of the channel. The PLL is a
separate object, created first and passed to the wrapper, so that several channels can share one
quad PLL ([Sharing a PLL](#sd-sharing)).

## The interface contract {#sd-interface}

A wrapper is built in three steps: the PLL, the wrapper, then the optional endpoints and
registers. The constructor is the same on every Xilinx family; the GTP takes `qpll` and
`qpll_reset` instead of `pll` and `pll_master`.

```python
pll    = GTXQuadPLL(refclk, refclk_freq=156.25e6, linerate=6.25e9)
serdes = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
    data_width       = 40,     # 20 or 40 (80 on GTY).
    tx_buffer_enable = True,   # TX elastic buffer (False: buffer bypass).
    rx_buffer_enable = True,   # RX elastic buffer (False: buffer bypass).
    clock_aligner    = False,  # Comma alignment by RX restarts (fixed latency).
    tx_polarity      = 0,      # Static P/N swap of the TX pair.
    rx_polarity      = 0,      # Static P/N swap of the RX pair.
)
serdes.add_stream_endpoints()  # sink/source.
serdes.add_controls()          # Registers: base, PRBS, loopback, polarity, electrical.
serdes.add_clock_cycles()      # Registers: TX/RX clock measurement.
```

The wrapper then exposes the following attributes.

| Attribute | Domain | Role |
|------------------------------|--------|--------------------------------------------------|
| `cd_tx`, `cd_rx` | | `tx` and `rx` clock domains, created by the wrapper |
| `tx_clk_freq`, `rx_clk_freq` | | Line rate divided by the data width, in Hz |
| `tx_enable`, `rx_enable` | `sys` | Deasserting one restarts its init sequence |
| `tx_ready`, `rx_ready` | `sys` | Init done; RX: aligned, with the clock aligner (then in `tx`) |
| `sink` | `tx` | `data` (8 bits per symbol), `ctrl` (one K flag per symbol) |
| `source` | `rx` | Same layout: decoded bytes and K flags |
| `encoder`, `decoders` | `tx`, `rx` | The LiteX 8b/10b modules, for direct access |
| `txoutclk`, `rxoutclk` | | Clock outputs of the channel, for constraints |
| `drp`, `drp_mux` | `sys` | Register access to the channel ([DRP](#sd-drp)) |

The endpoints have no flow control. The sink is always ready and the source always valid: one
word goes out and one word comes in on every cycle of the user clock. A user core that has
nothing to send must send the idle symbols of its protocol.

Symbol 0 of a word is the first on the line. Byte `data[7:0]` with `ctrl[0]` is symbol 0,
`data[15:8]` with `ctrl[1]` symbol 1, and so on. A K flag set on a byte that is not a valid K
character sends an undefined code group, which the receiver may decode without flagging it.

The `tx` and `rx` domains are held in reset through a reset synchronizer until their init
sequence is done. Logic in those domains therefore starts on a running, stable clock.

With the clock aligner, `rx_ready` is the ready output of the aligner, which runs in `tx`; a `sys`
consumer must synchronize it.

On the Xilinx wrappers, the reset of the `rx` domain passes through
a register clocked by `tx`. While the `tx` domain is in reset, or its clock stops, that register
holds 0 and the `rx` domain leaves reset whatever the RX init says.

## PLL solver {#sd-pll}

Each PLL class computes its dividers from the reference clock and the line rate at construction
(`compute_config`). It walks the dividers of the hardware in a fixed order and keeps the first
combination whose VCO lies in the allowed range and whose output is exactly the requested line
rate. Without a match it raises a `ValueError`:

```console
ValueError: No config found for 100.00 MHz refclk / 10.31 Gbps linerate.
```

The result is a dictionary (`pll.config`) with the dividers, the VCO frequency and the line rate.
Printing the PLL draws its structure and the computed values. The output below was produced
without hardware, with this command in the repository:

```sh
python3 -c "from migen import Signal; from liteiclink.serdes.gtx_7series import GTXQuadPLL; \
print(GTXQuadPLL(Signal(), 156.25e6, 10.3125e9))"
```

```console
GTXQuadPLL
===========
  [...]
  config:
  -------
    CLKIN    = 156.25MHz
    CLKOUT   = CLKIN x N / (2 x M) = 156.25MHz x 66 / (2 x 1)
             = 5.15625GHz
    VCO      = 10.3125GHz (upper)
    LINERATE = CLKOUT x 2 / D = 5.15625GHz x 2 / 1
             = 10.3125GHz
```

The match is exact: give the reference clock to the solver at its nominal value. The divider
ranges per family are in their chapters. The GTY quad PLL is the only one with a fractional
feedback divider, so it reaches line rates that the integer solvers reject
([GTY quad PLL](#us-gty-qpll)).

## Clocking {#sd-clocking}

The wrapper creates the `tx` and `rx` domains from the clock outputs of the channel, as the figure
below shows. The RX clock always comes from RXOUTCLK through a global buffer. The TX clock depends
on the TX buffer.

![User clocks of a Xilinx wrapper: TXOUTCLK goes through a clock buffer, and a divider or a PLL
when the TX buffer is bypassed, to the tx domain; RXOUTCLK goes through a clock buffer to the rx
domain.](sd_clocking.svg)

**TX buffer enabled.** TXOUTCLK already runs at the user clock frequency and drives the `tx`
domain through a buffer. The latency through the elastic buffer can differ from one reset to the
next.

**TX buffer bypassed** (the default). On the GTP, GTX and GTH, TXOUTCLK is the reference clock,
and the wrapper derives the user clock from it. The ratio of the reference clock to the user clock
decides how:

| Ratio | 7-Series | UltraScale, UltraScale+ |
|-----------------|-----------------------------------|----------------------------|
| 1 | BUFG | BUFG_GT |
| Integer | BUFR with `BUFR_DIVIDE`, then BUFG | BUFG_GT with its divider (1 to 8) |
| Not an integer | `S7PLL` from LiteX | Rejected (assertion) |

For example, a 125 MHz reference clock at 2.5 Gbps and 20 bits gives a 125 MHz user clock,
ratio 1. The same reference at 5 Gbps and 40 bits gives the same ratio. The test
`test_ultrascale_unaligned_tx_clock_rejected` checks the UltraScale rejection.

The UltraScale wrappers check only that the ratio is an integer. The BUFG_GT divider input has 3
bits, so a ratio above 8 is truncated without error and gives a wrong user clock: keep it at 8 or
less.

The GTY selects its program divider output as TXOUTCLK in bypass mode instead of the
reference clock, while its divider is computed for the reference clock. That path has no recorded
run: use the GTY with `tx_buffer_enable=True`.

**External user clocks.** The constructor takes `tx_clk` and `rx_clk` to clock the domains from
outside, for example one clock for several channels. Each requires its buffer
(`tx_buffer_enable`, `rx_buffer_enable`); the wrapper asserts it.

**Constraints.** The wrapper adds no timing constraint. The design declares the periods and the
false paths to `sys`:

```python
platform.add_period_constraint(serdes.cd_tx.clk, 1e9/serdes.tx_clk_freq)
platform.add_period_constraint(serdes.cd_rx.clk, 1e9/serdes.rx_clk_freq)
platform.add_false_path_constraints(soc.crg.cd_sys.clk, serdes.cd_tx.clk, serdes.cd_rx.clk)
```

## Init sequences {#sd-init}

Each wrapper has one init state machine for TX and one for RX, in `sys`. They follow the reset
sequences of the vendor user guides; the figure below shows their common shape. Every output to
the transceiver is registered and every input is synchronized into `sys`.

![Init FSM of the Xilinx wrappers: power down and PLL reset, optional DRP access, wait for the PLL
lock, release the channel reset, wait for reset done, then the buffer bypass alignment or directly
Ready. A restart goes back to power down; the watchdog, described below, does the same before
Ready.](sd_init.svg)

1. **Power down.** The channel is reset and powered down; the TX FSM also resets the PLL. On
   UltraScale this state lasts at least 2 µs.
2. **DRP access.** A hook for a wrapper that must write channel registers before the reset; it
   passes through when unused. The GTP RX FSM has its own DRP sequence
   ([GTP RX init](#s7-gtp-rx-init)).
3. **Wait PLL lock.** The FSM waits for the lock output of the PLL.
4. **Release reset.** The GTP and GTX do not release the channel reset before 500 ns after the
   `sys` reset, after a Xilinx answer record cited in the code; later restarts do not wait again.
5. **Wait reset done.** The FSM sets the user-ready input and waits for the reset done output of
   the channel. On GTX and Virtex-7 GTH (TX and RX) and on UltraScale RX, it also waits 1024
   cycles for the clock recovery.
6. **Delay and phase align.** With the buffer bypassed, the FSM runs the delay and phase
   alignment of the channel and waits for its done outputs. With the buffer enabled, it goes to
   Ready.

A watchdog restarts the FSM when it has not reached Ready in time:

| Family | TX watchdog | RX watchdog |
|----------------------|-------------|-------------|
| GTP | 1 ms | 4 ms |
| GTX, Virtex-7 GTH | 1 ms | 1 ms |
| GTH3, GTH4, GTY | 10 ms | 10 ms |

The 10 ms of the UltraScale FSMs leave time for the alignment at low line rates. A deasserted
`tx_enable` or `rx_enable` restarts the matching FSM at any time.

The TX FSM drives the PLL reset when the wrapper owns the PLL (`pll_master=True`, or
`qpll_reset=True` on GTP). Both FSMs wait for the lock. Every restart of that TX FSM resets the
PLL, so on a shared PLL it also restarts the other channels ([Sharing a PLL](#sd-sharing)).

## Datapath {#sd-datapath}

### Transmit {#sd-tx}

The encoder turns the bytes and K flags of the sink into 10-bit symbols, carrying the running
disparity from one symbol to the next. A TX source mux then chooses what reaches the transceiver,
in the fixed order of priority shown in the figure below.

![TX source mux of the Xilinx wrappers: the square wave has priority, then the pattern, then the
raw symbols of the GTY, then the output of the PRBS generator, which is the encoder output when
PRBS is off.](sd_tx_mux.svg)

| Priority | Source | Control |
|----------|------------------|----------------------------------------------------|
| 1 | Square wave | `tx_produce_square_wave` register |
| 2 | Fixed pattern | `tx_produce_pattern` and `tx_pattern` signals |
| 3 | Raw symbols | `tx_raw_enable` and `tx_raw` signals, GTY only |
| 4 | PRBS or encoder | `tx_prbs_config` register; off: encoder output |

The square wave is a word of ones in its lower half and zeros in its upper half. It toggles at the
user clock frequency, the line rate divided by the data width, so a scope or a frequency counter
on the pair confirms the line rate.

The pattern signals cross into `tx` through a two-stage synchronizer: they suit a constant
pattern, not data that changes every cycle. The ECP5 and GTR12 wrappers order their sources
differently; see their chapters.

### Receive {#sd-rx}

Each symbol of the received word goes to one decoder. The decoders give the bytes, the K flags and
an invalid flag per symbol. The GTP registers the received word once before the decoders, for
timing ([GTP RX timing](#s7-gtp-rx-timing)).

Word alignment depends on the buffer and on the clock aligner.

With the RX buffer enabled and the clock aligner off, the comma aligner of the transceiver aligns
on K28.5, with the comma on symbol 0. With the 80-bit GTY words, it aligns on 4-byte boundaries,
so the comma lands on symbol 0 or 4.

It is enabled while the `rx_align` signal is set and PRBS is
off. With the clock aligner on, the transceiver aligner is off and the clock aligner does the
job.

## Fixed-latency clock aligner {#sd-aligner}

With the RX buffer bypassed, the received words come out in a phase set by the clock recovery
after each reset. The clock aligner restarts the RX until that phase puts the comma on symbol 0,
which gives the same latency after every reset (`BruteforceClockAligner`). It is on by default
(`clock_aligner=True`).

![Clock aligner: at each check, the aligner moves from Wait comma to Wait no error to Ready when
it has seen the comma on symbol 0, then no symbol error; a missing comma or an error restarts the
RX init FSM.](sd_aligner.svg)

The aligner checks the received symbols every 6 ms, every 10 ms on GTP:

- **Wait comma.** It looks for the comma on bits 9 to 0 of the word, in either disparity. Without
  it, it restarts the RX init.
- **Wait no error.** It counts the ones of symbol 0 and flags an error when there are not 4, 5 or
  6. A comma without errors moves it to Ready.
- **Ready.** `rx_ready` now follows the aligner, not the RX init FSM. A symbol error restarts the
  RX.

The default comma is `0b0101111100`: K28.5 with its bits reversed, because the Xilinx
transceivers send the least significant bit first. The transmitter must send this comma
regularly on symbol 0. The `clock_aligner_disable` register stops the checks, and the `rx_align`
signal gates the restarts.

## Test features {#sd-test}

### PRBS {#sd-prbs}

The PRBS generator and checker are LiteX fabric modules (`PRBSTX`, `PRBSRX`). They replace the
whole word, so the line carries a pure PRBS sequence. The bits of each word are reversed so that
the serial order is the standard one despite the LSB-first transceiver. The PRBS blocks of the
transceiver itself are not used.

| `tx_prbs_config`, `rx_prbs_config` | Sequence |
|-------------------------------------|----------|
| `0b00` | Off |
| `0b01` | PRBS7 |
| `0b10` | PRBS15 |
| `0b11` | PRBS31 |

The checker counts one error per word that holds at least one wrong bit, so the 32-bit
`rx_prbs_errors` register counts errored words, not bit errors. A word with several wrong bits
counts once.

The checker rebuilds the sequence from the received bits, so one bit error on the
line can mark two consecutive words. A received word that stays constant for 1024 cycles marks
every word, which makes a dead link visible.

Turning the checker off clears the counter; bit 2 of `rx_prbs_config` pauses it, so that software
reads a stable value. The counter wraps instead of saturating, and the host script `test_prbs.py`
handles the wrap ([Results recorded on hardware](#perf-recorded) gives the reason). Its use is in
[Getting started](#getting-started).

### Loopback {#sd-loopback}

The `loopback` register selects the loopback mode of the Xilinx channels.

| Value | Mode |
|---------|-----------------------|
| `0b000` | Normal operation |
| `0b001` | Near-end PCS loopback |
| `0b010` | Near-end PMA loopback |
| `0b100` | Far-end PMA loopback |
| `0b110` | Far-end PCS loopback |

A near-end PMA loopback with PRBS checks the PLL, the serializer, the clock recovery and the fabric
datapath without a cable. `test_prbs.py --loopback` selects it. On the GTY, the wrapper holds the
clock recovery in the near-end PCS loopback (`RXCDRHOLD`).

### Polarity {#sd-polarity}

The constructor takes `tx_polarity` and `rx_polarity` for a board with swapped P and N lines. When
`add_controls` is called, the `tx_polarity` and `rx_polarity` registers replace these static
values.

### Electrical settings {#sd-electrical}

The electrical registers set the TX driver of the Xilinx channels: the swing (`tx_diffctrl`) and
the pre-emphasis before and after the main cursor (`tx_precursor`, `tx_postcursor`). Their widths
are those of the channel inputs; their reset values are the defaults of the wrappers. Each family
chapter gives them. The wrappers do not tune the receiver equalizers.

### Clock measurement {#sd-clock-cycles}

`add_clock_cycles` adds a free-running cycle counter in each of `tx` and `rx`. A write to
`clock_latch` copies both into `clock_tx_cycles` and `clock_rx_cycles`. Two latches a known time
apart give the frequencies of the user clocks, which `test_prbs.py` prints first.

The counters
are copied into `sys` without synchronization, so a latch can catch one while it changes: repeat
the measurement when a value is implausible.

## Registers {#sd-registers}

`add_controls` adds five groups. Each also exists as its own method: `add_base_control`,
`add_prbs_control`, `add_loopback_control`, `add_polarity_control` and `add_electrical_control`.

| Group | Registers |
|--------------|-------------------------------------------------------------------------|
| Base | `tx_enable`, `tx_ready`, `tx_inhibit`, `tx_produce_square_wave`, `rx_enable`, `rx_ready`, `clock_aligner_disable` |
| PRBS | `tx_prbs_config`, `rx_prbs_config`, `rx_prbs_errors` |
| Loopback | `loopback` |
| Polarity | `tx_polarity`, `rx_polarity` |
| Electrical | `tx_diffctrl`, `tx_postcursor`, `tx_precursor`, `_inv` bits where present |
| Clock cycles | `clock_latch`, `clock_tx_cycles`, `clock_rx_cycles` |

`clock_aligner_disable` exists only with the clock aligner. `tx_inhibit` turns the TX driver off.
`tx_enable` and `rx_enable` reset to 1; `auto_enable=False` keeps the channel idle until software
enables it.

`clock_tx_cycles` and `clock_rx_cycles` are storage registers written by the hardware: read them,
never write them. The fields and reset values of every register are in the
[Register reference](#registers).

## Register access to the channel: DRP {#sd-drp}

Every Xilinx channel and quad PLL has a dynamic reconfiguration port (DRP), a small bus to read
and write the configuration of the primitive at run time. The wrappers expose it as a
`DRPInterface`: 9-bit address, 16-bit data, enable, write enable and ready.

The channel port sits behind a mux (`drp_mux`, a `DRPMux` of up to 16 interfaces selected by its
4-bit `sel`).

On GTX, GTH and GTY the mux has one input, the `drp` attribute of the wrapper. On GTP it has two:
input 0 is the RX init FSM, input 1 the user `drp`.

The GTP wrapper never drives the select, so
the user `drp` is reachable only when the design drives it ([GTP RX init](#s7-gtp-rx-init)). The
quad PLL classes have their own `drp`.

`DRPControl(drp)` turns a DRP interface into registers for software, clocked by `sys`:

| Register | Role |
|----------|-----------------------------|
| `addr` | Address |
| `di` | Data to write |
| `we` | 1 for a write, 0 for a read |
| `start` | Write to start the access |
| `done` | Access complete |
| `do` | Data read |

```python
self.drp = DRPControl(serdes.drp)  # drp_addr, drp_di, drp_do, drp_we, drp_start, drp_done.
```

On the GTP, this example also needs `serdes.drp_mux.sel` set to 1 during the accesses.

A write to `start` restarts its state machine, which performs one access and then holds `done`.
The state machine also performs one read at address 0 after reset.

## Sharing a PLL {#sd-sharing}

A quad PLL can feed several channels of its quad. Pass the same PLL object to each wrapper and let
only one of them drive its reset: `pll_master=True` on one wrapper and `False` on the others,
`qpll_reset` on GTP. All wrappers wait for the lock.

A restart of the owner's TX init, from a
cleared `tx_enable` or its watchdog, resets the shared PLL, and with it every channel on the PLL.

On GTP, `GTPQuadPLL(shared=True)` computes the configuration without instantiating the common
block. The design then connects the `clk`, `refclk`, `lock` and `reset` signals of the PLL object
to a common block instantiated elsewhere. Sharing user clocks between channels uses `tx_clk` and
`rx_clk` with the buffers enabled ([Clocking](#sd-clocking)).

## Capabilities per family {#sd-capabilities}

| Feature | GTP | GTX, GTH | GTH3, GTH4 | GTY | ECP5 DCU | GTR12 |
|---------------|-----------|-----------|------------|-----------|-------------|-------------|
| PRBS | Yes | Yes | Yes | Yes | Yes | No |
| Loopback | 4 modes | 4 modes | 4 modes | 4 modes | 1 bit | No |
| Polarity | Register | Register | Register | Register | Static | Static |
| Electrical | Yes | Yes | Yes | Yes | No | No |
| Raw symbols | No | No | No | Yes | Yes | Yes |
| Rate change | No | No | No | Yes | No | No |
| Register access | DRP | DRP | DRP | DRP | SCI | UPAR |
| Clock aligner | Yes | Yes | Yes | Yes | No | No |

*Static* means a constructor parameter only. The ECP5 has its own alignment logic and reaches its
configuration through the SCI bus. The GTR12 takes hardware-validated settings from the caller
and is reached through UPAR. Their chapters describe them.

## In the code {#sd-code}

The wrapper files are in `liteiclink/serdes/`; the 8b/10b and PRBS modules come from LiteX.

| File | Content |
|-----------------------------|-----------------------------------------------------|
| `common.py` | `DRPInterface`, `DRPMux`, `DRPControl` |
| `clock_aligner.py` | `BruteforceClockAligner` |
| `*_init.py` | TX and RX init FSMs of each Xilinx family |
| LiteX `code_8b10b.py` | `Encoder`, `Decoder` |
| LiteX `prbs.py` | `PRBSTX`, `PRBSRX` |
| `test/test_serdes_*.py` | Elaboration, PLL, init FSM and aligner tests |

Where to go next: [Xilinx 7-Series](#xilinx-7series) and [Xilinx UltraScale](#xilinx-ultrascale)
for the Xilinx families, [Integration](#integration) to add a wrapper to a SoC, and
[Debugging](#debugging) when a link does not come up.
