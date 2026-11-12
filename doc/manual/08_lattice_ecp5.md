# Lattice ECP5 DCU {#lattice-ecp5}

The ECP5-5G FPGAs have one or two DCUs (dual channel units) of two transceiver channels each.
This chapter describes `SerDesECP5`, the SerDes wrapper of one DCU channel, with its PLL solver,
its datapath modes, its OOB signalling and its register access through the SCI.

The common interface of all wrappers is in [SerDes wrapper architecture](#serdes-architecture).

## At a glance {#ecp5-glance}

| Item | ECP5 DCU wrapper |
|------------------------|----------------------------------------------------------------|
| Classes | `SerDesECP5PLL`, `SerDesECP5` |
| Placement | `dual` 0 or 1 (DCU0, DCU1), `channel` 0 or 1; one wrapper per DCU |
| Line rate | Up to 5 Gbps (DCU maximum rate setting) |
| Data width | 20 bits, or 40 bits with `with_oob=True` |
| User clocks | Line rate / 20, or line rate / 40 in the 40-bit mode |
| Line coding | 8b/10b in the fabric, DCU in raw 10-bit mode |
| Comma alignment | Fabric (OOB and 40-bit modes); DCU state machine enabled in the default mode, not validated |
| Test patterns | Square wave, fixed pattern, PRBS7/15/31 |
| Runtime controls | Enables, PRBS, loopback, raw register access (SCI) |
| Not provided | Swing and de-emphasis registers, runtime polarity registers |

The hardware status of the family is in the [status table](#intro-status) of the introduction.

## The PLL {#ecp5-pll}

Each DCU has one transmit PLL shared by its two channels. It multiplies the reference clock by
M into the VCO, then divides the VCO by D to give the line rate. `SerDesECP5PLL(refclk,
refclk_freq, linerate)` searches M and D for an exact match and stores them in `config`.

| Setting | Values in the solver |
|------------------|--------------------------------------------------------|
| Multiplier M | 8, 10, 16, 20; 25 only with a 100 MHz reference clock |
| Divider D | 1, 2, 4, 8, 16, 32 |
| Search order | Largest D first, which gives the highest VCO frequency |

The solver checks only that VCO / D equals the line rate. The DCU itself is set for 5 Gbps at most
(`D_TX_MAX_RATE` and `CHX_CDR_MAX_RATE` of `"5.0"`). When no setting matches, the solver raises a
`ValueError` that lists the reference clock frequencies, up to 250 MHz, that would work.

The PLL prints its configuration with `print(pll)`, as the bench targets do. The two runs below
use the solver without hardware: a 250 MHz reference clock for 5 Gbps, then a 125 MHz one, which
has no matching multiplier.

```sh
python3 -c "from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL; \
print(SerDesECP5PLL(None, refclk_freq=250e6, linerate=5e9))"
python3 -c "from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL; \
SerDesECP5PLL(None, refclk_freq=125e6, linerate=5e9)"
```

```console
SerDesECP5PLL
==============
[...]
  config:
  -------
    CLKIN    = 250.0MHz
    VCO      = CLKIN x M = 250.0MHz x 20 = 5.0GHz
    LINERATE = VCO / D   = 5.0GHz / 1
             = 5.0GHz
[...]
ValueError: No config found for 125.0000 MHz refclk / 5.0000 Gbps linerate.
Possible refclk frequencies:
 - 250.0000MHz
```

The reference clock enters the DCU either from its dedicated pads, through an `EXTREFB`
primitive, or from the fabric, for example from an `ECP5PLL` output. The two bench targets choose
the source per line rate:

| Line rate | ECPIX-5 reference clock | Versa ECP5 reference clock |
|-----------|----------------------------------|----------------------------------|
| 1.25 Gbps | 100 MHz, `EXTREFB` | 156.25 MHz, `EXTREFB` |
| 1.5 Gbps | 150 MHz, fabric PLL | 150 MHz, fabric PLL |
| 2.5 Gbps | 100 MHz, `EXTREFB` | 156.25 MHz, `EXTREFB` |
| 3.0 Gbps | 150 MHz, fabric PLL | 150 MHz, fabric PLL |
| 5.0 Gbps | 250 MHz, fabric PLL | 250 MHz, fabric PLL |

The figure below shows where the user clocks come from. The DCU gives one transmit word clock from
its PLL and one receive word clock from the clock and data recovery (CDR), both at the line rate
divided by 20.

The CDR takes the reference clock directly, not the PLL output. In the 40-bit mode,
a `CLKDIVF` divides each word clock by two.

![ECP5 DCU clocking: the PLL sets the transmit line rate and the CDR recovers the receive clock,
both from the reference clock; the DCU word clocks run at line rate / 20. In the 20-bit mode they
drive tx and rx directly; in the 40-bit mode they become the tx_full and rx_full domains while
CLKDIVF /2 outputs drive the tx and rx domains.](ecp5_clocks.svg)

## Datapath {#ecp5-datapath}

The DCU channel runs in its raw 10-bit mode (`CHX_PROTOCOL = "10BSER"`): its 8b/10b encoder and
decoder, and its clock compensation FIFO, are bypassed. Its internal 1:2 gearbox gives two 10-bit
symbols per word clock cycle.

The wrapper places them on bits 0 to 9 and 12 to 21 of the 24-bit
DCU bus and presents them as a 20-bit word.

The 8b/10b encoder and decoders are the LiteX ones, in the fabric, as on the other families. In
front of the DCU, the transmit word comes from one source, in this priority order:

1. A square wave at the word rate (`tx_produce_square_wave`), for a scope.
2. Raw words (`tx_raw_enable`, `tx_raw`), only when `with_oob=True`.
3. A fixed pattern (`tx_produce_pattern`, `tx_pattern`).
4. The PRBS generator, which passes the encoder output when PRBS is off.

On the receive side, the PRBS checker and the decoders both take the received word. The aligned
raw word is also available on `rx_raw`. `tx_raw_data` and `rx_raw_data` are older names of the
two raw signals, kept as aliases. `add_stream_endpoints()` gives the `sink` and
`source` endpoints of the common interface: 8 data bits and one K flag per symbol, no flow control.

### Three datapath modes {#ecp5-modes}

Two constructor parameters select one of three modes. They differ in the word width and in where
comma alignment happens.

| Mode | Parameters | Word clock | Comma alignment |
|----------|----------------------------------|----------------|------------------------|
| Default | `data_width=20` | Line rate / 20 | DCU state machine (not validated) |
| OOB | `with_oob=True` | Line rate / 20 | Fabric, word aligner |
| 40-bit | `data_width=40`, `with_oob=True` | Line rate / 40 | Fabric, symbol aligner |

The default mode uses the `SerdesInit` state machine and the two OOB modes `SerdesInitOOB`, both
described in [Initialization](#ecp5-init).

In the default mode, the wrapper enables the DCU link state machine (`CHX_ENABLE_CG_ALIGN`) with
K28.5 in both running disparities, compared on all 10 bits. The `rx_align` signal gates it, and
the wrapper turns it off while the RX PRBS checker runs, because PRBS data contains comma
patterns.

No run records this alignment, and the source states that the DCU comma aligner is inactive on
the raw 10-bit path. The default-mode bench of the ECPIX-5 therefore looks for K28.5 on either
symbol of the word. A user core that needs the comma on symbol 0 uses an OOB mode.

In the OOB modes, the DCU link state machine is disabled (`CHX_LSM_DISABLE`) and the fabric aligns
the raw words.

### Fabric comma alignment {#ecp5-alignment}

The word aligner of the 20-bit OOB mode (`SerDesECP5WordAligner`) looks for the 7-bit comma,
shared by K28.1, K28.5 and K28.7, at each of the 20 bit offsets of a two-word window. It moves the
word boundary only when two consecutive detections agree on a new offset.

The wrapper arms this aligner only after a long absence of K28.5, because scrambled payload can
contain comma-like bit patterns that must not move a healthy link. Alignment is allowed when all
these conditions hold:

- `rx_align` is set;
- no K28.5 was decoded for 2047 receive clock cycles;
- 8 invalid symbols were decoded since the last K28.5, or no K character at all for 4096 cycles.

The symbol aligner of the 40-bit mode (`SerDesECP5SymbolAligner`) searches the commas of K28.1,
K28.5 and K28.7 at the 10 bit offsets of an 80-bit window. It moves to a new offset after two
consecutive words with commas at that offset.

The symbol aligner does not move the symbols within the 40-bit word,
like the 4-byte comma alignment of the Xilinx transceivers, and it pauses while the RX PRBS checker
runs. Its `offset` and `aligned` outputs show the state.

## The 40-bit mode {#ecp5-40bit}

At 5 Gbps the DCU word clock runs at 250 MHz, which the ECP5 fabric logic does not reach. The
40-bit mode halves the user clock frequency: the encoder, decoders, PRBS and patterns work on
40-bit words at line rate / 40, 125 MHz at 5 Gbps. The figure below shows the datapath of this
mode.

![ECP5 40-bit datapath: 40-bit words in the fabric, gearboxes between the 40-bit tx and rx domains
and the 20-bit DCU bus, and the symbol aligner on the receive side.](ecp5_datapath.svg)

Two gearboxes cross between the clocks. The TX gearbox (`SerDesECP5TXGearbox`) sends bits 0 to 19
of each 40-bit word first; the RX gearbox (`SerDesECP5RXGearbox`) builds 40-bit words with the
first received word in bits 0 to 19.

Both run in the `tx_full` and `rx_full` domains, the DCU word
clocks, which have no reset. They find the phase of the half-rate clock from a register that
toggles in the half-rate domain.

The `tx_clk_freq` and `rx_clk_freq` attributes give the frequency of the `tx` and `rx` domains,
line rate / 40. The `txoutclk` and `rxoutclk` signals run at line rate / 20, so their period
constraints use twice that frequency, as USB3-PIPE does:

```python
platform.add_period_constraint(serdes.txoutclk, 1e9/(2*serdes.tx_clk_freq))
platform.add_period_constraint(serdes.rxoutclk, 1e9/(2*serdes.rx_clk_freq))
platform.add_false_path_constraints(soc.crg.cd_sys.clk, serdes.txoutclk, serdes.rxoutclk)
```

The `tx` and `rx` domains, behind the `CLKDIVF` dividers, get no constraint of their own: the
toolchain derives them from these periods.

The 40-bit mode requires `with_oob=True`, which provides the fabric alignment; the constructor
asserts it.

## Initialization {#ecp5-init}

The wrapper resets the DCU in a fixed order with one of two state machines in the `sys` domain.
Both watch the TX PLL loss of lock, the RX CDR loss of lock and the RX loss of signal of the DCU,
resynchronized to `sys`. The default one (`SerdesInit`) brings TX and RX up together:

| State | Held in reset | Leaves when |
|---------------------------------------------|----------------|--------------------------|
| `RESET-ALL` | TX PLL, RX CDR, PCS | 1024 cycles elapsed |
| `RESET-RX-PCS-WAIT-TX-PLL-LOCK` | RX CDR, PCS | 1024 cycles and TX PLL locked |
| `RESET-PCS-WAIT-RX-CDR-LOCK` | PCS | 1024 cycles and RX CDR locked |
| `READY` | None | Loss of lock or of signal |

From `READY`, a loss of lock returns to `RESET-ALL` and a loss of signal to the second state.
`tx_ready` and `rx_ready` are both the `READY` state in this mode.

The OOB state machine (`SerdesInitOOB`) brings TX up first, so that an OOB initiator can transmit
before it receives any signal. It drives separate resets for the TX PLL, TX PCS, RX CDR and RX PCS:

| State | Leaves when |
|--------------------|-------------------------------------------------|
| `RESET-ALL` | 1024 cycles elapsed |
| `WAIT-TX-PLL-LOCK` | TX PLL locked for 1024 cycles |
| `RESET-TX-PCS` | 8 cycles; TX ready from the next state |
| `WAIT-RX-SIGNAL` | Signal present for 64 cycles |
| `RESET-RX-CDR` | 8 cycles |
| `WAIT-RX-CDR-LOCK` | Signal present and CDR locked for 64 cycles |
| `RESET-RX-PCS` | 8 cycles; RX ready in `READY` |

In the OOB machine, a TX PLL loss of lock returns to `RESET-ALL` from any state. A receive problem
returns only the RX side to `WAIT-RX-SIGNAL`, and TX stays ready.

Clearing `rx_enable` holds the RX resets of the channel. Clearing `tx_enable` holds the TX lane
reset and the TX PLL reset of the DCU, which the other channel would also use. The DCU macro and
dual resets follow the `sys` reset.

The loss of signal is also available in `sys` as `rx_idle`, and the
state machine and lock signals (`init.fsm`, `init.tx_lol`, `init.rx_lol`) can be captured with
LiteScope, as the ECPIX-5 bench target does.

## OOB signalling {#ecp5-oob}

SATA and USB 3 send out-of-band (OOB) signals: bursts separated by electrical idle, where the
transmitter drives no data. `with_oob=True` adds what these protocols need on top of the fabric
alignment and the TX-first initialization:

- The TX electrical idle input of the channel (`CHX_FFC_EI_EN`) is driven directly by `tx_idle`,
  resynchronized to `tx`, instead of through the SCI loop described below.
- The raw transmit words (`tx_raw_enable`, `tx_raw`) bypass the encoder; electrical idle is
  released while they are sent, so that the bursts stay on the line.
- The transmit word is registered before the DCU, for timing.

LiteSATA (`litesata/phy/ecp5sataphy.py`) uses the OOB mode at 20 bits for SATA. USB3-PIPE uses the
40-bit mode, with OOB, for LFPS at 5 Gbps. The protocol logic that generates and detects the
bursts belongs to those cores.

## Register access through the SCI {#ecp5-sci}

The DCU registers are reachable at runtime through its serial control interface (SCI): 6-bit
addresses and 8-bit data, on the channel registers or on the dual registers. `SerDesECP5SCI` drives
the SCI ports of the `DCUA` instance. `SerDesECP5SCIReconfig` uses it in two ways.

In normal operation, a state machine repeats read-modify-write cycles on four channel registers,
three in the OOB modes. It applies the signals that the DCU primitive has no port for:

| Channel register | Bits written | Signal |
|------------------|--------------|----------------------------------------------|
| `0x01` | 0, 1 | `rx_polarity`, `tx_polarity` |
| `0x02` | 6 | `tx_idle` (default mode only) |
| `0x15` | 3:0 | `0b0001` when `loopback` is set |
| `0x18` | 6 | `rx_cdr_hold` |

The loop is held in reset until TX is ready. The polarity parameters of the constructor accept a
constant or a signal; a change of the signal is applied by the next pass of the loop.

The SCI port belongs to the DCU of the wrapper, placed at `LOC DCU<dual>`, so the `dual` index
does not select it. The dual select only reaches the dual registers, for software accesses with
`sel` set to 1; the loop accesses select the channel registers only.

When software sets the `pause` register, the loop stops and the registers below give direct access
to the SCI. The constructor always adds them, so they appear under `<serdes>_sci_reconfig_` in the
register map even without `add_controls()`.

| Register | Access | Role |
|----------|--------|--------------------------------------------------------|
| `pause` | Write | 1: stop the loop and allow software accesses |
| `sel` | Write | 0: channel registers, 1: dual registers |
| `adr` | Write | Register address, 6 bits |
| `dat_w` | Write | Data to write, 8 bits |
| `we` | Write | Any write starts a write access |
| `re` | Write | Any write starts a read access |
| `done` | Read | 1 when no access is running |
| `dat_r` | Read | Data of the last read, 8 bits |

A read is: set `pause`, write `sel` and `adr`, write `re`, wait for `done`, read `dat_r`. Clearing
`pause` restarts the loop, which then rewrites its registers.

The DCU parameters themselves are fixed at build time in the `serdes_params` dictionary of the
instance: 50 ohm terminations, TX amplitude, RX equalizer off, loss of signal level and several
undocumented values; the source comments give the origin of each value.

A user core can update the
dictionary before the design is finalized, with `CHX` in the names for the channel, as USB3-PIPE
does for its receive settings.

## Control registers {#ecp5-controls}

`add_controls()` adds the base, PRBS and loopback registers of the common interface, and
`add_clock_cycles()` the clock measurement registers. The full list is in the
[Register reference](#registers); the ECP5 specifics are:

- `loopback` takes the 3-bit encoding of the Xilinx wrappers, so that the same software applies:
  any non-zero value sets the loopback field of register `0x15` through the SCI loop. The source
  notes that this "does not seem enough" for a working loopback.
- `tx_inhibit` exists for compatibility with the other wrappers but drives nothing.
- There are no polarity or electrical registers. Polarity comes from the constructor; swing and
  de-emphasis are DCU parameters, reachable at runtime only through the raw SCI access.

## Limitations {#ecp5-limits}

- 5 Gbps at most; the PLL solver does not check the VCO range of the DCU.
- The loopback register is not known to give a working loopback, and `tx_inhibit` has no effect.
- The raw transmit words are used only with `with_oob=True`.
- The 40-bit mode needs `with_oob=True`; its symbol aligner pauses while PRBS is checked.
- The SCI registers are always present, one bank per wrapper.
- The default-mode comma alignment by the DCU has no recorded run, against a source comment that
  says the DCU aligner is inactive on the raw path ([Three datapath modes](#ecp5-modes)).
- One wrapper per DCU: each wrapper instantiates the whole `DCUA` at `LOC DCU<dual>`, so the two
  channels of a DCU cannot be used by two wrappers. The TX PLL and its reset are per DCU.

## Recorded results {#ecp5-results}

Commit `939f4ad`, which added the 40-bit mode, records a 40-bit loopback and a USB 3 device run at
5 Gbps on the LambdaConcept ECPIX-5 ([Results recorded on hardware](#perf-recorded)).

## In the code {#ecp5-code}

| Class | Role |
|--------------------------------|---------------------------------------------------|
| `SerDesECP5PLL` | PLL solver and printout |
| `SerDesECP5` | DCU channel wrapper, datapath, controls |
| `SerdesInit` | Default initialization state machine |
| `SerdesInitOOB` | OOB initialization state machine (TX first) |
| `SerDesECP5WordAligner` | Fabric K28.5 alignment, 20-bit OOB mode |
| `SerDesECP5SymbolAligner` | Fabric comma alignment, 40-bit mode |
| `SerDesECP5TXGearbox` | 40 to 20 bits, 40-bit mode |
| `SerDesECP5RXGearbox` | 20 to 40 bits, 40-bit mode |
| `SerDesECP5SCI` | SCI port driver |
| `SerDesECP5SCIReconfig` | SCI loop and software access registers |

All are in `liteiclink/serdes/serdes_ecp5.py`. The bench targets are `bench/serdes/ecpix5.py` and
`bench/serdes/versa_ecp5.py`; the unit tests are in `test/test_serdes_ecp5.py`.

Where to go next: [Gowin GW5 GTR12](#gowin-gw5) for the other non-Xilinx transceiver,
[Benches and examples](#examples) for the ECP5 bench targets, [Debugging](#debugging) for PRBS
runs and register checks.
