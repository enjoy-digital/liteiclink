# SerWB PHYs and link initialization {#serwb-phys}

A SerWB PHY turns 32-bit words into bits on the TX lane and back, forwards or receives the link
clock, and brings the link up. This chapter describes the four PHYs, the initialization, the word
datapath, the PHY registers, the link test and health counters, and the flow control.

The source synchronous concepts (forwarded clock, delay taps, bitslip) are in
[Wishbone over serial](#serwb-primer).

## At a glance {#swb-phy-glance}

| PHY | Device names | Delay taps | Bits per cycle |
|------------------|-----------------------------------------|--------------|---------------------|
| 7-Series | `xc7a`, `xc7k`, `xc7v`, `xc7z`, `xc7s` | 32, dynamic | 8 per `sys` cycle |
| UltraScale(+) | `xcku`, `xczu` | 512, dynamic | 8 per `sys` cycle |
| Efinix Titanium | `Ti` | 64, dynamic | 8 per `clk` cycle |
| Efinix Trion | `T1`, `T2` (T13, T20, T120) | Static | 8 per `clk` cycle |
| Generic | Any | None | 1 per `sys` cycle |

The four high-speed PHYs share one class, `SERWBPHY` of `liteiclink/serwb/phy.py`. It picks the
I/O module from the start of the device name: four letters for Xilinx, two for Efinix. Other names
raise a `NotImplementedError`.

These include Virtex UltraScale(+) devices: the Xilinx list holds
`xvu`, three letters, which a four-letter prefix never matches. Artix UltraScale+ (`xcau`) and the
Trion devices whose name does not start with `T1` or `T2` (T4, T8, T35, T55, T85) are refused too.

The generic PHY is another class with the same name, in `liteiclink/serwb/genphy.py`. The hardware
status of each PHY is in the [status table](#intro-status) of the introduction.

## Pads and the forwarded clock {#swb-phy-clock}

The pads record has three lanes. The high-speed PHYs use differential pairs: `clk_p`/`clk_n`,
`tx_p`/`tx_n` and `rx_p`/`rx_n`. The generic PHY takes either pairs or single-ended `clk`, `tx`
and `rx` signals, and adds the clock lane only when the record has one.

The link master forwards a clock that the link slave uses to sample the data:

| PHY | Clock sent by the link master | Link slave |
|------------------|---------------------------------------------|--------------------------------------|
| 7-Series | `0b1111100000` through an `OSERDESE2`: line rate / 10 | Received clock as `clocking.refclk` |
| UltraScale(+) | Same pattern, `OSERDESE3` | Same, when the record has `clk_p` |
| Efinix | `0b11110000` through an LVDS block: line rate / 8 | PLL in the PHY |
| Generic | `sys` inverted, through a DDR output | Received clock is the slave `sys` |

On the 7-Series and UltraScale PHYs, the link slave design multiplies the received clock with a
PLL or MMCM into its own `sys` and `sys4x`; the PHY does not instantiate that PLL. With
`sys` at 125 MHz, the forwarded clock is 100 MHz (computed from the pattern).

The Efinix PHY of the link slave instantiates a Titanium or Trion PLL on the received clock and
creates three domains: `rx_sys` at the `sys` frequency, `rx_clk` and `rx_clk4x`. The bench targets
run the link slave PHY and core in `rx_sys` with a `ClockDomainsRenamer`.

The generic link master sends its `sys` clock inverted, so that the link slave, clocked by it,
samples each bit half a period after the master launched it.

## High-speed PHYs {#swb-phy-hs}

### 7-Series {#swb-phy-s7}

The figure below shows the 7-Series PHY. On the transmit side, the word datapath gives 8 bits per
`sys` cycle to an `OSERDESE2` in 8:1 DDR mode clocked by `sys4x`. On the receive side, an
`IDELAYE2` with 32 taps delays the input, and an `ISERDESE2` in 1:8 DDR mode deserializes it.

![7-Series SerWB PHY: in S7Serdes, the TX datapath feeds an OSERDESE2 and the received lane goes
through an IDELAYE2 and an ISERDESE2 into the RX datapath; the init state machine of SERWBPHY moves
the delay taps and the bitslip. The clock lane is not drawn.](swb_phy_s7.svg)

The init state machine belongs to `SERWBPHY`, around the `S7Serdes` I/O module. It moves the
`IDELAYE2` taps one by one and uses the `BITSLIP` input of the `ISERDESE2` to move the bit
boundary. The delay is in `VARIABLE` mode with a 200 MHz reference, which needs an `IDELAYCTRL`
in the design; the Nexys Video bench target adds one.

On the link master, a second `OSERDESE2` sends the clock pattern; the figure does not draw this
clock lane.

### UltraScale and UltraScale+ {#swb-phy-ku}

The UltraScale PHY has the same structure with `OSERDESE3`, `IDELAYE3` (512 taps, `COUNT` format)
and `ISERDESE3`. It moves the bit boundary with a fabric bitslip module on the deserialized 8-bit
words instead of a primitive input.

`usp=True` sets the UltraScale+ simulation device and the `REFCLK_FREQUENCY` attribute of the
delay to 300 MHz instead of 200 MHz. The delays run in `COUNT` format, without voltage and
temperature compensation (`EN_VTC` at 0); the PHY instantiates no `IDELAYCTRL`. Only `clk_ratio="1:1"` is accepted on the Xilinx PHYs.

### Efinix Titanium and Trion {#swb-phy-efinix}

The Efinix PHY does not instantiate primitives: it describes LVDS transmit and receive blocks
(8 bits, half-rate fast clock) that the Efinix toolchain generates in its interface design.

The LVDS blocks take two clocks named by `clk` and `clk4x` (`"sys"` and `"sys4x"` by default): the
parallel 8-bit clock and the serial clock at four times its rate. The datapath runs in `sys`.

`clk_ratio` (`"1:1"`, `"1:2"` or `"1:4"`) lets the LVDS clock run slower than `sys`: with 1:N,
`clk` runs at `sys` / N and the datapath moves one 8-bit word every N `sys` cycles. The receiver
does not know which of the N cycles carries the word, so the initialization tries each phase in
turn; the `phase` register gives the result.

The Titanium LVDS receiver has a dynamic delay of 64 taps. The Trion receiver has none: its delays
are fixed at build time by `clk_delay_taps` and `rx_delay_taps`, and the initialization only finds
the word boundary.

### Generic PHY {#swb-phy-gen}

The generic PHY needs no I/O primitive: it sends one bit per `sys` cycle from a register, and
samples the input with a register.

The generic PHY suits low-speed links and FPGAs without serializers, and it
is the PHY of the Verilator simulation. It has no delay taps: its initialization only searches the
word boundary, one bit at a time, among the 40 positions.

## Link initialization {#swb-phy-init}

The link master runs the initialization, and the link slave follows the state of the line. The
figure below shows the sequence: an idle line resets the link slave, each side then calibrates on
the commas of the other, and the commas stop when both are done.

![SerWB link initialization: the master holds the line at zero to reset the slave, sends K28.5
commas while the slave calibrates, receives the slave commas while it calibrates itself and keeps
sending its own until it is ready, then both sides stop sending commas.](swb_init.svg)

1. The master holds its TX lane at zero. A slave that receives only zeros, or only ones, for 32
   `sys` cycles resets its initialization and its I/O module.
2. The master sends K28.5 commas. The slave calibrates on them while it keeps its own lane at zero.
3. The slave sends commas. When the master sees the slave lane active, it calibrates on them,
   and sends its own commas until it is done.
4. The master stops sending commas and is ready.
5. The slave sees the commas stop and is ready.

Each wait of the sequence lasts `init_timeout` `sys` cycles, 2^16 by default. The master restarts
the sequence when software writes its `reset` register, or by itself with automatic retraining.

### Calibration {#swb-phy-calibration}

A side is calibrated when it decodes K28.5 at the start of its received words. The state machine
searches a delay setting and a word boundary that give stable commas:

1. For the word boundary under test (`shift`), step the delay from tap 0 upwards, waiting
   `init_timeout` cycles at each tap.
2. The first tap where commas are seen for `init_timeout` cycles is the start of the window
   (`delay_min`); the first later tap without commas, or the last tap, is its end (`delay_max`).
3. If the window is narrower than 1/16 of the taps, keep scanning. Otherwise set the delay to the
   middle of the window and stop.
4. If the last tap is reached without a valid window, move the word boundary by one bit and start
   again from tap 0, for up to 40 positions.
5. On Efinix with `clk_ratio` 1:2 or 1:4, try the next phase after the 40 positions. When nothing
   works, the PHY reports `error` and stays there until it is reset.

A word boundary step moves the bit position in the I/O module. Every 8 steps, the datapath also
drops one 8-bit word (`RXAligner`), so the 40 steps cover the 40 bits of an encoded word. The
generic PHY has no taps and runs only step 4.

The `error` state is reached only after a full scan. A full scan without a window lasts taps x 40
x `init_timeout` cycles per phase, for example about 10.7 s on UltraScale (512 taps) and 0.67 s on
7-Series with `sys` at 125 MHz (computed).

A link master whose RX lane stays idle, with the link slave missing or not configured, waits in
step 3 without a timeout: `ready` and `error` both stay at 0.

## Word datapath {#swb-phy-datapath}

The word datapath is the same for every PHY; only its width on the I/O side differs, 8 bits for
the high-speed PHYs and 1 bit for the generic PHY.

| Direction | Steps |
|-----------|-----------------------------------------------------------------------------|
| TX | 32-bit word, four 8b/10b symbols (40 bits), converter to the I/O width |
| RX | Word boundary shift, converter to 40 bits, four 8b/10b decoders, 32-bit word |

The control words put a K character in the first symbol of a word:

| Word | Use |
|-----------------|---------------------------------------------------------------------|
| K28.5 | Comma, during initialization |
| K28.1 | Fill word when there is nothing to send, with flow control |
| K28.3 | XOFF, with flow control |
| K28.4 | XON, with flow control |
| Raw zeros | Idle line, which resets the link slave (not an 8b/10b symbol) |

Without flow control, a side with nothing to send transmits idle words: the data word zero,
encoded. The receiver forwards them to the core, whose depacketizer skips them while it looks for
the preamble of the next packet.

The receiver also flags each received word with an invalid symbol, a symbol without 4, 5 or 6 ones,
for the health counters. The datapath has an experimental scrambler (`with_scrambling`) that no
PHY enables.

## Registers {#swb-phy-registers}

`SerdesControl` adds the PHY registers under `<phy>_control_`, for example
`serwb_phy_control_ready`. The delay and `phase` registers exist only on the high-speed PHYs.

| Register | Access | Width | Role |
|---------------------------|--------|-------|-----------------------------------------------------|
| `reset` | Write | 1 | Restart the initialization (link master only) |
| `ready` | Read | 1 | Link ready |
| `error` | Read | 1 | Initialization failed, no window found |
| `delay` | Read | 9 | Delay tap in use |
| `delay_min_found` | Read | 1 | Start of the window found |
| `delay_min` | Read | 9 | First tap of the window |
| `delay_max_found` | Read | 1 | End of the window found |
| `delay_max` | Read | 9 | Last tap of the window |
| `shift` | Read | 6 | Word boundary, 0 to 39 |
| `phase` | Read | 2 | Efinix phase in use; the last phase tried after an error |
| `prbs_start` | Write | 1 | Start a link test |
| `prbs_cycles` | Write | 32 | Length of the link test, in `sys` cycles |
| `prbs_errors` | Read | 32 | Errored words counted by the last link test |
| `invalid_errors` | Read | 32 | Words with invalid symbols since the link is ready |
| `link_drops` | Read | 32 | Transitions from ready to not ready |
| `prbs_tx` | Write | 1 | Send PRBS31 instead of traffic (`with_prbs`) |
| `prbs_rx` | Write | 1 | Check received words as PRBS31 (`with_prbs`) |

The health counters (`invalid_errors`, `link_drops`) saturate at their maximum value. The core adds `rx_overflows` and, with CRC,
`crc_errors` ([SerWB packets and protocol](#swb-crc)).

## Link test {#swb-phy-prbs}

A write to `prbs_start` clears `prbs_errors` and counts errors for `prbs_cycles` `sys` cycles; the
result stays in `prbs_errors` until the next test. What counts as an error depends on the build:

- **Without `with_prbs`**: every received word that is not zero. This works only on an idle link
  without flow control, where the other side sends encoded zeros; any traffic counts as errors.
- **With `with_prbs`**: one side sets `prbs_tx` and sends a PRBS31 sequence of 32-bit words
  instead of its traffic; the other side sets `prbs_rx` and checks them. While `prbs_rx` is set,
  the received words are not forwarded to the core, so random data never reaches the
  depacketizer. The option costs about 160 LUTs (from `doc/serwb.md`).

A test gives a count of errored words, each with one or more wrong bits: the checker counts one
error per word with any mismatch. It does not count bits, and one line error can mark one or two
words, so the count is not a bit error count.

`prbs_cycles` counts `sys` cycles, not words. A word arrives every 5 `sys` cycles on the Xilinx
PHYs, every 5 x N on Efinix at 1:N, and every 40 on the generic PHY, so the words checked are
`prbs_cycles` divided by that number.

With no errors, compute the bit error ratio bound from the
bits checked, 32 per word, as in [PRBS and bit error ratio](#primer-prbs), and label it computed.

## Link health {#swb-phy-health}

Two counters watch a link that is up. `invalid_errors` counts the received words with an invalid
symbol; it is cleared while the link is not ready. `link_drops` counts the losses of the ready
state.

A link master built with `with_auto_retrain=True` restarts the initialization by itself when 16
invalid words arrive within one window of 2^16 `sys` cycles while the link is ready (fixed
windows: 16 invalid words spread over two windows do not retrain).

Without it, a degraded link
stays ready until software writes `reset`. The link slave follows the restart through the idle
line.

## Flow control {#swb-phy-flow}

The link has no backpressure: without flow control, words that arrive when the RX buffer of the
core is full are lost. With `with_flow_control=True` on both PHYs, the receiving side pauses the
sending side with XOFF and XON words. The figure below shows an exchange.

![SerWB flow control: side B sends XOFF when its RX buffer is almost full, side A sends K28.1 fill
words while paused, and resumes after XON; XON or XOFF is sent again periodically.](swb_flow.svg)

- The core sets `pause` when 16 or fewer words of its RX buffer are free, and clears it when more
  are free.
- On each change of `pause`, the PHY inserts one XOFF or XON word in its TX stream; this takes the
  place of one data word and never blocks local traffic.
- The PHY sends the pause state again every 1024 `sys` cycles, which recovers a lost XON or
  XOFF.
- While paused by the other side, or with nothing to send, the PHY sends K28.1 fill words. The
  receiver drops fill, XON and XOFF words before the core.

Flow control changes the line format, so both sides must enable it. With flow control, the fill
words are removed before the link test counter, so the link test without `with_prbs` counts almost
nothing and does not measure the link; use `with_prbs`.

## Line rates {#swb-phy-rates}

The line rate follows from the clocks: 8 bits per `sys` cycle for the Xilinx PHYs, 8 bits per
`clk` cycle for the Efinix PHY, 1 bit per `sys` cycle for the generic PHY. One 32-bit word takes
40 bits. The values below are computed from the clocks of the bench targets, not measured:

| PHY, bench target | Clocks | Line rate | Words per second |
|---------------------------------|------------------|-----------|------------------|
| 7-Series, `nexys_video.py` | `sys` 125 MHz | 1 Gbps | 25 M |
| 7-Series, `doc/serwb.md` example | `sys` 156.25 MHz | 1.25 Gbps | 31.25 M |
| Titanium, `ti60_f225_dev_kit.py` | `sys` 300 MHz, 1:4 | 600 Mbps | 15 M |
| Trion, `t120_bga576_dev_kit.py` | `sys` 100 MHz, 1:4 | 200 Mbps | 5 M |
| Generic, ECPIX-5 demo | `sys` 25 MHz | 25 Mbps | 0.625 M |

Packets, requests and responses take several words each; [SerWB packets and protocol](#serwb-protocol)
gives the word counts and [Performance and resources](#performance) the resulting throughput.

## PHY parameters {#swb-phy-params}

| Parameter | Default | PHYs | Role |
|-----------------------------|--------------|--------------|-------------------------------------|
| `device` | | All | Device name; selects the high-speed I/O module |
| `pads` | | All | Clock, TX and RX lanes |
| `mode` | `"master"` | All | Link master or link slave |
| `init_timeout` | `2**16` | All | `sys` cycles of each initialization wait |
| `with_flow_control` | `False` | All | XON/XOFF flow control, both sides |
| `with_prbs` | `False` | All | PRBS31 link test registers |
| `with_auto_retrain` | `False` | All | Automatic retraining, link master only |
| `clk`, `clk4x` | `"sys"`, `"sys4x"` | Efinix | LVDS clocks, link master (the link slave uses `rx_clk`, `rx_clk4x`) |
| `clk_ratio` | `"1:1"` | Efinix | `sys` to LVDS clock ratio |
| `clk_delay_taps` | 0 | Trion | Static delay of the clock input |
| `rx_delay_taps` | 0 | Trion | Static delay of the RX input |
| `usp` | `False` | UltraScale | UltraScale+ primitives settings |

The generic PHY takes `device`, `pads`, `mode`, `init_timeout` and the three options.

## Recorded results {#swb-phy-results}

Commit messages record three hardware runs: the Efinix PHY at clock ratios 1:1, 1:2 and 1:4 on
Titanium, a link between an UltraScale link master and a 7-Series link slave, and the generic PHY
of the ECPIX-5 and iCEBreaker demo at 50 MHz (the demo runs at 25 MHz on master).
[Results recorded on hardware](#perf-recorded) quotes them.

## In the code {#swb-phy-code}

| File | Content |
|-----------------------|-----------------------------------------------------------------------|
| `phy.py` | High-speed `SERWBPHY`, device selection, `_SerdesInit` |
| `s7serdes.py` | `S7Serdes`: 7-Series clocking, TX and RX |
| `kuserdes.py` | `KUSerdes`: UltraScale(+) clocking, TX and RX |
| `efinixserdes.py` | `EfinixSerdes`, LVDS blocks, `BitSlip` |
| `genphy.py` | Generic `SERWBPHY` and its init state machine |
| `common.py` | `SerdesControl` registers, link test, `SERWBPHYBase.add_datapath` |
| `datapath.py` | `TXDatapath`, `RXAligner`, `RXDatapath`, `FlowControl` |
| `scrambler.py` | Experimental scrambler, not used by the PHYs |

The files are in `liteiclink/serwb/`; the tests of the initialization and of the health counters
are `test/test_serwb_init.py` and `test/test_serwb_health.py`.

Where to go next: [SerWB packets and protocol](#serwb-protocol) for the words above the PHY,
[Benches and examples](#examples) for the bench targets, [Debugging](#debugging) for a link that
does not come up.
