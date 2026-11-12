# Gowin GW5 GTR12 {#gowin-gw5}

The Gowin GW5AST and GW5AT FPGAs have GTR12 transceiver quads of four lanes. This chapter
describes `GTR12`, the SerDes wrapper of one lane: the quad configuration, the fabric datapath, the
clocks and resets, the lane status and the UPAR register access.

The common interface of all wrappers is in [SerDes wrapper architecture](#serdes-architecture).

> The GTR12 wrapper is validated on hardware up to 5 Gbps. Line rates above 5 Gbps, such as the
> 80-bit raw words of `raw_width` for 10 Gbps, are in development; see the
> [status table](#intro-status).

## At a glance {#gw5-glance}

| Item | GTR12 wrapper |
|------------------|------------------------------------------------------------------|
| Class | `GTR12`, with `GTR12UPAR` and `GTR12Decoder` |
| Devices | GW5AST-138, GW5AT-138 (`GTR12_QUAD`, quads Q0 and Q1) |
| | GW5AT-15 (`GTR12_QUADB`, quad Q0 only) |
| Lanes | 0 to 3 of the quad, one per wrapper |
| Configuration | Hardware-validated register writes given by the caller (`csr_config`) |
| Line rate | Set by the configuration; `linerate` only sets the clock values |
| Data width | 20, 40 or 80 bits; raw words up to 80 bits (`raw_width`) |
| User clocks | Line rate / data width, from the PCS fabric clocks |
| Line coding | 8b/10b in the fabric, decoders in logic lookups |
| Comma alignment | In the PCS, when the configuration enables it |
| Polarity | Inversion in the fabric, constant or signal of the constructor |
| Runtime access | Lane status register, quad registers through the UPAR |
| Not provided | PLL solver, PRBS, loopback, square wave, electrical controls |

The wrapper picks the primitives from the device name of the platform. A name ending in `-138`,
optionally followed by letters, selects `GTR12_QUAD` and `GTR12_UPAR`; one ending in `-15` selects
`GTR12_QUADB` and `GTR12_UPARA`. Other names raise a `ValueError`, and a platform without a device
name gets the GW5AST-138 primitives.

## Configuration model {#gw5-config}

The Xilinx and ECP5 wrappers set their transceivers with primitive parameters computed in Python.
The GTR12 quad is different: it is configured by a list of register writes that the Gowin
toolchain applies to the quad.

The line rate, the PLLs and the CDR, the PMA width, the TX and RX
buffer gears (the fabric data width) and the comma alignment all come from these writes.

LiteICLink holds no configuration: the user core stores hardware-validated settings, as register
writes, and passes them as `csr_config`. The figure below shows the flow.

![GTR12 configuration flow: the user core holds the hardware-validated register writes, GTR12
writes them into the toolchain script with set_csr, and the toolchain loads them into the quad;
the UPAR port gives runtime access to the same registers.](gw5_config.svg)

`GTR12` writes the text into a file of the gateware directory (`csr_name`, `gtr12.csr` by
default) and adds a `set_csr` command to the Gowin TCL script. USB3-PIPE keeps the
hardware-validated settings of its lanes and rates in its `usb3_pipe/gw5/` directory. This manual
reproduces none of them.

The wrapper cannot check the configuration. Its parameters must match what the register writes
set up:

| Parameter | Must match |
|--------------|------------------------------------------------------------------|
| `linerate` | The configured line rate (used for `tx_clk_freq`, `rx_clk_freq`) |
| `data_width` | The fabric data width of the TX and RX buffers |
| `raw_width` | The widest fabric word used on the lane |
| `position` | The quad of the configuration, `Q0` or `Q1` |
| `lane` | The configured lane, 0 to 3 |
| `pll` | The PLL the configuration uses, for the lock status |

A lane of a GW5AST-138 at 5 Gbps with 40-bit words, as used by USB3-PIPE on the Tang Mega 138K
Pro, is created like this:

```python
from liteiclink.serdes.gtr12_gw5 import GTR12

# csr_config: hardware-validated register writes of the lane, held by the user core.
self.serdes0 = serdes0 = GTR12(platform,
    linerate   = 5e9,
    csr_config = csr_config,
    data_width = 40,
    position   = "Q1",
    lane       = 0,
    pll        = "QPLL1",
)
serdes0.add_stream_endpoints()
serdes0.add_status_csr()
serdes0.upar.add_csr()
```

## Lane datapath {#gw5-datapath}

The quad runs in a raw PCS mode: the fabric sends and receives words of 10-bit symbols, the first
symbol on bits 0 to 9, and does the 8b/10b coding itself. The figure below shows the datapath
around the lane.

![GTR12 lane datapath: fabric 8b/10b encoder, TX source selection and polarity inversion before
the lane; polarity inversion and logic-lookup decoders after it.](gw5_lane.svg)

On the transmit side, a register in the `tx` domain selects the word, in this priority order:

1. Raw words (`tx_raw_enable`, `tx_raw`, `raw_width` bits), which bypass the encoder.
2. A fixed pattern (`tx_produce_pattern`, `tx_pattern`).
3. The output of the 8b/10b encoder.

The selected word is inverted bit by bit when `tx_polarity` is set, then padded with zeros to the
80-bit TX data input of the lane. The TX valid input of the lane is tied high.

On the receive side, the lane gives up to 88 bits per cycle and a valid flag. The wrapper reads its
RX FIFO whenever it is not empty, applies `rx_polarity`, and registers the first `raw_width` bits
as `rx_raw`, with `rx_valid` beside them. The decoders take their symbols from `rx_raw`.
`add_stream_endpoints()` gives the `sink` and `source` endpoints of the common interface.

The PCS aligns the received symbols on commas when the configuration enables it. The `rx_align`
signal drives the align trigger input of the lane; there is no aligner in the fabric.

### Decoders in logic {#gw5-decoders}

`GTR12Decoder` has the interface and the one-cycle latency of the LiteX 8b/10b decoder. Its 6b/5b
and 4b/3b lookups are written as case statements, with registered outputs marked
`syn_romstyle "logic"`.

Two problems led to this decoder. A lookup mapped to a block RAM had a slow clock-to-output delay
that limited the receive path.

With registered lookups, GowinSynthesis inferred block ROMs and
merged them with other lookups on the same address into wrong contents: the decoded symbols were
wrong on hardware while correct in simulation. The `invalid` output only checks that a symbol has
4, 5 or 6 ones; there is no running disparity check.

### Raw words wider than the 8b/10b words {#gw5-raw-width}

`raw_width` makes `tx_raw` and `rx_raw` wider than the 8b/10b words. The use case is a lane that
carries 40-bit 8b/10b words at 5 Gbps and 80-bit raw words at 10 Gbps, with the 8b/10b words on
the least significant bits.

The rate change itself, a sequence of register writes through the
UPAR, belongs to the user core. This use is in development (see the note at the top of the
chapter).

## Clocks and resets {#gw5-clocks}

The PCS fabric clock outputs of the lane drive the `tx` and `rx` clock domains directly, and the
same clocks feed back to the fabric clock inputs of the lane.

The clock signals carry a `syn_keep`
attribute so that their names survive synthesis for the timing constraints. Their frequency is
`linerate / data_width`, 125 MHz for 5 Gbps and 40-bit words.

The reset logic is simple:

- The lane is held in reset while both `tx_enable` and `rx_enable` are clear.
- `tx_ready` is `tx_enable` with the selected PLL locked; `rx_ready` is `rx_enable` with the same
  lock. The lock is resynchronized to `sys`.
- The PCS TX and RX resets, and the `tx` and `rx` domain resets, follow `tx_ready` and `rx_ready`.

`rx_ready` does not wait for the CDR: a user core checks `cdr_lock` and `aligned` in the lane
status. There is no init state machine and no watchdog, unlike the Xilinx wrappers.

## Lane status {#gw5-status}

The lane outputs status signals in the quad clock domains. `add_status_csr()` resynchronizes them
to `sys` in one read-only register, `<serdes>_status`:

| Bits | Signal | Meaning |
|-------|---------------|------------------------------------------------------|
| 0 | `pll_lock` | Lock of the PLL selected by `pll` |
| 1 | `cdr_lock` | RX CDR locked |
| 2 | `aligned` | PCS comma alignment done |
| 3 | `k_lock` | PCS K character lock |
| 4 | `rx_elecidle` | RX electrical idle |
| 10:5 | `astat` | Lane analog status |
| 23:11 | `stat` | Lane status |
| 25:24 | `refclk_det` | Quad reference clocks detected (CM0, CM1) |
| 28:26 | `plls_lock` | QPLL0, QPLL1 and lane CPLL locks |

## UPAR register access {#gw5-upar}

The UPAR port gives runtime read and write access to the quad registers, the same ones that the
configuration sets. `GTR12UPAR` drives it with one access at a time: a 24-bit address, 32-bit data,
a write or read request, and a done pulse.

It serves two requesters. The user requests (`addr`, `wrdata`, `we`, `re`, `done`, `rddata`), for
example a rate change sequencer, have priority. The register requests of `add_csr()` wait until no
user request is pending.

An `abort` input returns the state machine to idle when an access never
completes. It is a level: while it is set, every access ends at once with stale data.

The CSR mode input of the UPAR primitive is set to `0b10100`; with other values the accesses do not
complete.

The UPAR primitive also drives the AHB reset and the test decoder enable of the quad, and its
clock is the AHB clock of the quad. The toolchain requires these on the same nets, so
`with_upar=True` is the default. On the GW5AT-15, the toolchain connects `GTR12_UPARA` to the quad.

`upar_clk` selects the UPAR clock, `sys` by default. The registers of `add_csr()` assume that the
UPAR runs in `sys`:

| Register | Access | Role |
|----------|--------|-----------------------------------------------------------------|
| `addr` | Write | Register address, 24 bits |
| `wrdata` | Write | Data to write, 32 bits |
| `ctrl` | Write | Bit 0: start a write, bit 1: start a read |
| `status` | Read | Bit 0: no access pending; bits 1, 2: UPAR ready and read valid |
| `cfg` | Write | Bit 0: abort the access in progress; write 0 afterwards |
| `rddata` | Read | Data of the last read, 32 bits |

A read is: write `addr`, write `ctrl` with bit 1 set, wait for bit 0 of `status`, read `rddata`.

## Compared with the Xilinx wrappers {#gw5-compare}

The GTR12 wrapper gives the common stream interface, enables, ready signals, encoder and decoders.
The other parts of the Xilinx wrappers are missing or different:

| Feature | Xilinx wrappers | GTR12 |
|----------------------------|----------------------------------|------------------------------|
| PLL settings | Python PLL solver | Register writes of the user |
| PRBS, square wave | `add_controls()` | None |
| Loopback | Register | None |
| Polarity | Registers | Constructor parameter |
| TX swing, de-emphasis | Registers | Configuration, UPAR |
| Clock measurement | `add_clock_cycles()` | None |
| Init | State machines, watchdog | PLL lock only |
| Line rate parameter | `linerate` | `linerate` (`line_rate` also accepted) |
| Raw words | `tx_raw`, `rx_raw` (GTY) | `tx_raw`, `rx_raw` |

## Recorded results {#gw5-results}

The commits that added the wrapper (`16f1e31`, Sipeed Tang Mega 138K Pro) and the GW5AT-15 support
(`5d098d2`) record USB 3 runs at 5 Gbps with USB3-PIPE ([Results recorded on
hardware](#perf-recorded)).

## In the code {#gw5-code}

| Name | Role |
|----------------------------|----------------------------------------------------|
| `GTR12` | Lane wrapper, datapath, resets, status, endpoints |
| `GTR12Decoder` | 8b/10b decoder with logic lookups |
| `GTR12UPAR` | UPAR access state machine and its registers |
| `gtr12_primitives` | Quad and UPAR primitives of a device |
| `gtr12_quad_params` | Quad parameters with every unused input tied off |
| `gtr12_add_csr_config` | Configuration file and `set_csr` command |

All are in `liteiclink/serdes/gtr12_gw5.py`; the unit tests are in `test/test_serdes_gtr12.py`.
LiteICLink has no GTR12 bench target: the hardware runs above were made with USB3-PIPE.

Where to go next: [Lattice ECP5 DCU](#lattice-ecp5) for the other non-Xilinx transceiver,
[Integration](#integration) for the clock constraints of a SerDes wrapper,
[Debugging](#debugging) for the lane status and UPAR checks.
