# Xilinx UltraScale and UltraScale+: GTH and GTY {#xilinx-ultrascale}

This chapter covers the transceivers of the UltraScale and UltraScale+ families: the GTH of
UltraScale (GTHE3), the GTH of UltraScale+ (GTHE4) and the GTY of UltraScale+ (GTYE4).

They follow
the common wrapper contract of [SerDes wrapper architecture](#serdes-architecture). The GTY adds a
fractional quad PLL, an 80-bit width, raw symbols and an on-the-fly rate change, which make it the
wrapper for line codes other than 8b/10b. The hardware status of each one is in the
[status table](#intro-status).

## At a glance {#us-glance}

| Item | GTH3 | GTH4 | GTY |
|-----------------------|--------------------------------|--------------------------------|--------------------------------|
| Devices | UltraScale | UltraScale+ | UltraScale+ |
| Primitives | `GTHE3_COMMON`, `GTHE3_CHANNEL` | `GTHE4_COMMON`, `GTHE4_CHANNEL` | `GTYE4_COMMON`, `GTYE4_CHANNEL` |
| Wrapper | `GTH3` | `GTH4` | `GTY` |
| PLLs | `GTHChannelPLL` (GTH3 file), `GTH3QuadPLL` | `GTHChannelPLL` (GTH4 file), `GTH4QuadPLL` | `GTYChannelPLL`, `GTYQuadPLL` |
| Data widths | 20, 40 | 20, 40 | 20, 40, 80 |
| Quad PLL feedback | Integer | Integer | Fractional |
| Fabric reference clock | No | Yes | Yes |
| Raw symbols, rate change | No | No | Yes |
| User guide cited | UG576 | UG576 | UG578 |

`gth3_ultrascale.py` and `gth4_ultrascale.py` each define their own `GTHChannelPLL`, and each
wrapper accepts only the class of its own file. The module `gth_ultrascale` re-exports the GTH3 and
GTH4 wrappers and quad PLLs, but only the GTH3 channel PLL. A GTH4 on its channel PLL therefore
imports `GTHChannelPLL` from `gth4_ultrascale`; the re-exported class fails at elaboration.

The three wrappers share one init FSM design (`gth_ultrascale_init.py`,
`gty_ultrascale_init.py`) and the clocking of the UltraScale column of [Clocking](#sd-clocking).

## Channel PLL {#us-cpll}

The channel PLL (CPLL) has the same structure as on 7-Series: the reference clock divided by M,
multiplied by N1 x N2, then the line rate is twice the VCO divided by D.

| PLL | VCO range | N1 | N2 | M | D |
|--------------------|-----------------|------|--------|------|-------------|
| `GTHChannelPLL` | 2.0 to 6.25 GHz | 4, 5 | 1 to 5 | 1, 2 | 1, 2, 4, 8, 16 |
| `GTYChannelPLL` | 2.0 to 6.25 GHz | 4, 5 | 1 to 5 | 1, 2 | 1, 2, 4, 8 |

From these ranges the solvers accept up to 12.5 Gbps on the CPLL (computed). The GTY solver stops
D at 8; a comment in the code marks the larger dividers as to be checked.

## GTH quad PLLs {#us-gth-qpll}

The GTH common block has two quad PLLs with overlapping VCO ranges. The solver walks N, then M,
then D, and takes QPLL1 whenever the VCO falls in its range; QPLL0 serves the VCO frequencies above
13 GHz. The line rate is the VCO divided by D.

| PLL | VCO range | N | M | D |
|-------|---------------------|------------------------------------------|--------|----------------|
| QPLL1 | 8 to 13 GHz | 16, 20, 32, 40, 60, 64, 66, 75, 80, 84, 90, 96, 100, 112, 120, 125, 150, 160 | 1 to 4 | 1, 2, 4, 8, 16 |
| QPLL0 | 9.8 to 16.375 GHz | Same | 1 to 4 | 1, 2, 4, 8, 16 |

The solver therefore accepts up to 16.375 Gbps on the GTH quad PLLs (computed). The selected PLL is
in `pll.config["qpll"]`; the wrapper sets the clock selects of the channel from it.

`GTH4QuadPLL` takes `refclk_from_fabric`, which drives the reference clock to `GTGREFCLK0/1` and
selects it, as on the GTP ([Reference clock from the fabric](#s7-gtp-refclk)). `GTH3QuadPLL` has
no such option. In the GTH3 class, comments mark the `QPLL0_FBDIV_G3` and `QPLL1_FBDIV_G3` values
as to be checked.

## GTY quad PLL {#us-gty-qpll}

The GTY quad PLL has a fractional feedback divider: a sigma-delta modulator (SDM) adds a fraction
to the integer N. The solver therefore reaches line rates that need a non-integer ratio to the
reference clock. The figure below shows its structure.

![GTY quad PLL: the reference clock divided by M feeds QPLL0 or QPLL1; the feedback divider of
each is N plus a fraction from its SDM; the output runs at full or half rate, then the channel
divides by D to the line rate.](us_gty_qpll.svg)

The solver works backwards from the line rate:

1. For each D in 1, 2, 4, 8, 16, it computes the VCO: the line rate times D up to 16.375 Gbps
   (half-rate output), the line rate times D divided by 2 above (full-rate output).
2. It takes QPLL1 if the VCO is within 8 to 13 GHz, else QPLL0 if within 9.8 to 16.375 GHz.
3. For each M in 1 to 4, it computes N plus fraction and keeps the first one between 16 and 160.

The highest line rate the solver accepts is 32.75 Gbps; above it raises a `ValueError`.

The solver does not round the fraction to the 24 bits of the SDM. The 24-bit SDM value is the
rounded fraction, given as the reset value of the `sdm0_data` and `sdm1_data` signals. With a non-zero
fraction, the class clears the SDM bypass bit of the active PLL.

A 100 MHz reference clock at 10.3125 Gbps shows the difference. The integer GTX solver rejects it
([PLL solver](#sd-pll)); the GTY solver returns QPLL1 with N = 103.125. The output below comes from
the same command as in that section, with `GTYQuadPLL(Signal(), 100e6, 10.3125e9)`:

```console
  config:
  -------
    CLKIN    = 100.0MHz
    CLKOUT   = CLKIN x N.Fractional / (clkout_rate x M) = 100.0MHz x 103.125 / (2 x 1)
             = 5.15625GHz
    VCO      = 10.3125GHz (QPLL1)
    LINERATE = CLKOUT x 2 / D = 5.15625GHz x 2 / 1
             = 10.3125GHz
```

### Selection and tuning {#us-gty-qpll-tuning}

Two constructor parameters override the solver's choices:

| Parameter | Values | Effect |
|------------------|---------------------------|--------------------------------------------------|
| `qpll` | None, `"qpll0"`, `"qpll1"` | Forces one PLL; `ValueError` if its VCO range cannot match |
| `qpll_params` | Dictionary | Replaces named attributes of `GTYE4_COMMON`, without the `p_` prefix |

`qpll_params` refuses the attributes the solver derives (the feedback and reference dividers, the
output rate and `SDM_CFG0`) and names that are not attributes of the primitive. The tests in
`test_gty_qpll_config.py` cover both rules.

Several attribute values of the GTY quad PLL carry a comment with an alternative value for 10 or
12 Gbps. `qpll_params` is the way to try such values without editing the class.

`GTYQuadPLL` also takes `refclk_from_fabric`, with the same effect as on the GTH4.

## GTY raw symbols {#us-gty-raw}

The GTY wrapper exposes the received and transmitted words without the 8b/10b, for a line code
handled in the fabric, such as 128b/132b. The figure below shows the raw path beside the 8b/10b
path.

![GTY raw path: tx_raw enters the TX source mux beside the encoder output; rx_raw is the received
word before the decoders. The channel itself does no 8b/10b in either case.](us_gty_raw.svg)

| Signal | Domain | Role |
|-------------------|--------|----------------------------------------------------------|
| `tx_raw` | `tx` | Word to send, the full data width |
| `tx_raw_enable` | Any | Selects `tx_raw` in the TX source mux (synchronized into `tx`) |
| `rx_raw` | `rx` | Received word, always present, before the decoders |

`tx_raw` has priority 3 in the TX source mux, below the square wave and the pattern
([Transmit](#sd-tx)). The word is sent as the encoder output would be: bit 0 first, in groups of
10 bits on the channel's data and control inputs. With 80 bits, the wrapper moves eight symbols
or 80 raw bits per user clock cycle.

## GTY rate change {#us-gty-rate}

The GTY wrapper drives the `TXRATE` and `RXRATE` inputs of the channel from the 3-bit `tx_rate`
and `rx_rate` signals, synchronized into `tx` and `rx`. They change the line rate on the fly by
changing the output divider of the channel, with the same PLL.

| Value | Divider |
|-------|-----------------------------------------------|
| `000` | D of the PLL configuration |
| `001` | 1 |
| `010` | 2 |
| Above | Larger powers of two, as defined in UG578 |

With the PLL at 10 Gbps and D = 1, value `010` gives 5 Gbps. A run on the Alibaba XCKU3P switched
between 10 and 5 Gbps in near-end PMA loopback with PRBS31 and no errors, the user clocks following
the rate ([Results recorded on hardware](#perf-recorded)).

`tx_clk_freq` and `rx_clk_freq` keep the
value of the PLL line rate, so timing constraints written from them cover the lower rates. USB 3.2
Gen 2 is the reason for this feature: USB3-PIPE trains at 5 Gbps with 8b/10b, then runs at 10 Gbps
with 128b/132b on raw symbols. The test `test_gty_raw_and_rate` checks the wiring.

## Init and clocking {#us-init}

The UltraScale init FSMs follow the common shape ([Init sequences](#sd-init)) with three
differences. The PLL reset lasts at least 2 µs. The watchdog is 10 ms, for the alignment at low line
rates. In buffer bypass, the TX waits for two rising edges of the phase alignment done output, the
RX for the sync done output.

The TX user clock comes from a BUFG_GT.

In buffer bypass, its divider is the ratio of the reference clock to the user clock and must be an
integer; the wrapper asserts it. For example, a 156.25 MHz reference clock at 5 Gbps and 40 bits
gives a 125 MHz user clock: the ratio 1.25 is rejected in buffer bypass, and accepted with the TX
buffer enabled.

The ratio must also be 8 or less, the range of the 3-bit BUFG_GT divider, which
the wrapper does not check.

In buffer bypass, the GTY selects its program divider output as TXOUTCLK, not the reference clock
as the GTH does, while the BUFG_GT divider is computed for the reference clock. No run records
this path: use the GTY with `tx_buffer_enable=True`, as the XCU1525 bench does.

## Electrical defaults {#us-electrical}

| Register | GTH3 | GTH4 | GTY |
|-------------------------|----------------|----------------|-----------------|
| `tx_diffctrl` | 4 bits, `0b1100` | 4 bits, `0b1100` (1) | 5 bits, `0b11000` |
| `tx_postcursor` | 5 bits, 0 | 5 bits, 0 | 5 bits, 0 |
| `tx_precursor` | 5 bits, 0 | 5 bits, 0 | 5 bits, 0 |
| `_inv` bits | 1 bit each | Absent | Absent |

(1) The register is 4 bits wide while the wrapper's own `A_TXDIFFCTRL` attribute value is 5 bits
wide (`0b01100`): check the width of the channel input in UG576 before relying on the upper part
of the range.

The value-to-voltage tables are in UG576 for the GTH and UG578 for the GTY. Software that writes
these registers must take the width of `tx_diffctrl` and the missing `_inv` bits into account.

## DRP {#us-drp}

Each channel has one DRP interface behind its mux: the `drp` attribute of the wrapper, selected by
default ([DRP](#sd-drp)). Debug tools reach the channel registers through it, for example with
`DRPControl(serdes.drp)`. The quad PLLs have their own `drp`.

## Limitations {#us-limitations}

- On GTH3 and GTH4, the RX clock select of the channel (`RXSYSCLKSEL`) is fixed to the CPLL value,
  whatever the PLL; the TX side and the GTY follow the selected PLL. Receiving with a quad PLL on
  GTH has no recorded result. With a quad PLL, the CPLL also stays powered (`CPLLPD` at 0).
- `gth_ultrascale` re-exports the GTH3 channel PLL, which the GTH4 wrapper rejects
  ([At a glance](#us-glance)).
- The GTY quad PLL attributes carry several comments with alternative values, and its fraction is
  not rounded in the solver.
- The DRP interfaces have a 9-bit address, as on 7-Series. Check the DRP address width of the GTH4
  and GTY channels in UG576 and UG578: registers above that range are not reachable through `drp`.
- The BUFG_GT divider ratio is not checked against its 1 to 8 range, and the GTY TX clock in buffer
  bypass has no recorded run ([Init and clocking](#us-init)).
- On the GTY, `RXCDRHOLD` is meant to follow the near-end PCS loopback, but the condition is a
  Python test of a signal, evaluated once at elaboration: the input is always 0.
- With 80-bit GTY words, the comma aligner aligns on 4-byte boundaries, so the comma lands on
  symbol 0 or 4 ([Receive](#sd-rx)).
- The reset of the `rx` domain passes through a register clocked by `tx`, as on 7-Series
  ([Limitations](#s7-limitations)).

## Users {#us-users}

USB3-PIPE drives the GTY of an XCKU3P board at 5 and 10 Gbps, with the raw symbols and the rate
change. The LitePCIe soft PHY uses the GTY wrapper. The HDMI-over-GT video PHY of LiteX uses the
GTH4 wrapper, and LiteEth reuses the PLL classes and init FSMs with its own channel instances.

## In the code {#us-code}

The wrappers are in `liteiclink/serdes/`, the benches in `bench/serdes/`.

| File | Content |
|-------------------------------|--------------------------------------------------------|
| `gth3_ultrascale.py` | `GTHChannelPLL`, `GTHQuadPLLBase`, `GTH3QuadPLL`, `GTH3` |
| `gth4_ultrascale.py` | `GTHChannelPLL`, `GTHQuadPLLBase`, `GTH4QuadPLL`, `GTH4` |
| `gth_ultrascale_init.py` | `GTHTXInit`, `GTHRXInit` |
| `gty_ultrascale.py` | `GTYChannelPLL`, `GTYQuadPLL`, `GTY` |
| `gty_ultrascale_init.py` | `GTYTXInit`, `GTYRXInit` |
| `kcu105.py`, `xcu1525.py` | GTH3 and GTY benches |

Where to go next: [Lattice ECP5](#lattice-ecp5) and [Gowin GW5](#gowin-gw5) for the other
families, [Integration](#integration) to add a wrapper to a SoC, and [Debugging](#debugging) for a
link that does not come up.
