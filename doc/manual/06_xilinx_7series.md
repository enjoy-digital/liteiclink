# Xilinx 7-Series: GTP, GTX and GTH {#xilinx-7series}

This chapter covers the three transceivers of the Xilinx 7-Series: the GTP of the Artix-7, the
GTX of the Kintex-7 and Virtex-7, and the GTH of the Virtex-7.

All three follow the common wrapper
contract of [SerDes wrapper architecture](#serdes-architecture); this chapter gives what differs:
the PLLs and their ranges, the reference clock inputs, the GTP specifics and the electrical
defaults. The hardware status of each one is in the [status table](#intro-status).

## At a glance {#s7-glance}

| Item | GTP | GTX | GTH |
|------------------|--------------------------|---------------------------------|---------------------------------|
| Devices | Artix-7 | Kintex-7, Virtex-7 | Virtex-7 |
| Primitives | `GTPE2_COMMON`, `GTPE2_CHANNEL` | `GTXE2_COMMON`, `GTXE2_CHANNEL` | `GTHE2_COMMON`, `GTHE2_CHANNEL` |
| Wrapper | `GTP` | `GTX` | `GTH` |
| PLLs | `GTPQuadPLL` | `GTXChannelPLL`, `GTXQuadPLL` | `GTHChannelPLL`, `GTHQuadPLL` |
| Data widths | 20, 40 (1) | 20, 40 | 20, 40 |
| Fabric reference clock | Yes | No | No |
| User guide cited | UG482 | UG476 | UG476 |

(1) The GTP 40-bit width has no recorded run ([Limitations](#s7-limitations)).

The GTP has no channel PLL: its two PLLs, PLL0 and PLL1, sit in the common block of the quad. The
GTX and GTH have one channel PLL (CPLL) per channel and one quad PLL (QPLL) per quad.

## PLLs and line rates {#s7-plls}

The channel PLLs of the GTX and GTH and the two PLLs of the GTP share one structure: the reference
clock divided by M, multiplied by N1 x N2 in the VCO, then the line rate is twice the VCO divided
by D.

| PLL | VCO range | N1 | N2 | M | D |
|-------------------|-----------------|------|-----------|------|-----------------|
| GTP PLL0, PLL1 | 1.6 to 3.3 GHz | 4, 5 | 1 to 5 | 1, 2 | 1, 2, 4, 8, 16 |
| GTX CPLL | 1.6 to 3.3 GHz | 4, 5 | 1 to 5 | 1, 2 | 1, 2, 4, 8, 16 |
| GTH CPLL | 2.0 to 6.25 GHz | 4, 5 | 1 to 5 | 1, 2 | 1, 2, 4, 8, 16 |

The quad PLL of the GTX and GTH has two VCO bands. Its output runs at half the VCO and the line
rate is the VCO divided by D.

| PLL | Lower band | Upper band | N | M | D |
|-------------------|------------------|-------------------|-----------------------------------|--------|----------------|
| GTX, GTH QPLL | 5.93 to 8 GHz | 9.8 to 12.5 GHz | 16, 20, 32, 40, 64, 66, 80, 100 | 1 to 4 | 1, 2, 4, 8, 16 |

The QPLL configuration value of the primitive depends on the band, which the solver reports in
`pll.config["vco_band"]`. The ranges give the line rates the solvers accept, computed from the
tables above:

| PLL | Line rates allowed by the solver (computed) |
|-------------------|-----------------------------------------------------|
| GTP | 0.2 Gbps to just below 6.6 Gbps (the wrapper asserts below 6.6 Gbps) |
| GTX CPLL | 0.2 to 6.6 Gbps |
| GTH CPLL | 0.25 to 12.5 Gbps |
| GTX, GTH QPLL | Up to 8 Gbps (lower band), 9.8 to 12.5 Gbps (upper band) |

Between 8 and 9.8 Gbps no PLL of the GTX matches: the CPLL stops at 6.6 Gbps and the QPLL bands
leave that gap at D = 1. These are limits of the solvers, not measured rates.

The Virtex-7 GTH solvers use a CPLL VCO range of 2.0 to 6.25 GHz and the two QPLL bands of the
GTX. Check a GTH configuration against the GTHE2 PLL ranges of UG476 before relying on it.

The table below gives configurations the solvers return for the reference clocks of the benches,
computed with `compute_config`:

| PLL | Reference clock | Line rate | N1 x N2 or N | M | D |
|---------------------|-----------------|--------------|---------------|---|---|
| `GTPQuadPLL` | 125 MHz | 2.5 Gbps | 4 x 5 | 1 | 2 |
| `GTPQuadPLL` | 125 MHz | 5 Gbps | 4 x 5 | 1 | 1 |
| `GTXChannelPLL` | 125 MHz | 1.25 Gbps | 4 x 5 | 1 | 4 |
| `GTXQuadPLL` | 156.25 MHz | 3.125 Gbps | 40, lower | 1 | 2 |
| `GTXQuadPLL` | 156.25 MHz | 6.25 Gbps | 40, lower | 1 | 1 |
| `GTXQuadPLL` | 156.25 MHz | 10.3125 Gbps | 66, upper | 1 | 1 |

The wrapper picks the PLL type from the class of the object it receives: a channel PLL or a quad
PLL. It sets the clock selects of the channel and the CDR configuration for the chosen D.

## GTP {#s7-gtp}

### PLL0, PLL1 and sharing {#s7-gtp-pll}

`GTPQuadPLL` instantiates the common block with one PLL in use: PLL0 for `channel=0`, PLL1 for
`channel=1`. The unused PLL is powered down. The GTP channel selects the matching PLL clock.

`qpll_reset=True` (the default) lets the TX init FSM of the wrapper reset the PLL. Set it to False
on all but one wrapper when several channels share the PLL. `shared=True` skips the common block,
for a design that instantiates it elsewhere ([Sharing a PLL](#sd-sharing)).

### Reference clock from the fabric {#s7-gtp-refclk}

A GTP PLL takes its reference clock from the dedicated clock pads of the quad, through
`IBUFDS_GTE2`, on the `GTREFCLK0` input. Some boards have no dedicated reference clock wired to the
quad; the reference clock then comes from a fabric clock, for example a PLL output. The figure
below shows the two inputs.

![GTP reference clock inputs: the dedicated pads reach GTREFCLK0 through IBUFDS_GTE2; with
refclk_from_fabric, a fabric clock reaches GTGREFCLK0 or GTGREFCLK1 through the global clock
network.](s7_gtp_refclk.svg)

With `refclk_from_fabric=True`, `GTPQuadPLL` drives the clock to `GTGREFCLK0` (PLL0) or
`GTGREFCLK1` (PLL1) and selects it with `REFCLKSEL = 0b111`. The clock then reaches the PLL on the
global clock network, instead of the general interconnect that a fabric clock on `GTREFCLK0`
would use.

UG482 reserves `GTGREFCLK` for test purposes, and Vivado flags it with the REQP-49 design rule.
The design must lower that rule to a warning, as the Acorn bench does:

```python
platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
self.pll = GTPQuadPLL(self.cd_refclk.clk, 125e6, linerate, refclk_from_fabric=True)
```

The option exists because, on the SQRL Acorn, a fabric clock on `GTREFCLK0` gave a link quality
that depended on the place and route, while `GTGREFCLK0` gave stable links on every tested build
([Results recorded on hardware](#perf-recorded)). Use the dedicated pads when the board has them;
when it does not, use `refclk_from_fabric`. The test `test_gtp_refclk_from_fabric` checks the
inputs and selects.

### RX init and the DRP workaround {#s7-gtp-rx-init}

The GTP RX init FSM differs from the common shape ([Init sequences](#sd-init)). It applies a
workaround of UG482 around the RX PMA reset: it clears bit 11 of DRP address `0x011` while the PMA
reset runs, then writes the original value back after the falling edge of `RXPMARESETDONE`.

The FSM reads the original value once, on its first pass, and restores that value on every later
pass. A restart between the change and the restore would otherwise read the changed value and
"restore" it, leaving the receiver in the wrong width.

That failure was seen on the Acorn at
5 Gbps, with a wrong RX clock after some RX restarts ([Results recorded on
hardware](#perf-recorded)). The tests `test_gtp_rx_drp_sequence` and
`test_gtp_rx_drp_restart_before_restore` model it.

The channel DRP mux of the GTP (`GTPDRPMux`) shares its input 0, the default select, between the
RX init FSM and the user `drp`. The FSM requests the DRP before its sequence and gets it between two
accesses; the user `drp` has the DRP the rest of the time. A user access issued during the sequence
is held and issued after it, so a `DRPControl` access completes with the select left at 0.

The shared input is clocked by `sys`, the clock of the RX init FSM: use the user `drp` from `sys`,
as `DRPControl` does. Input 1 is the user `drp` alone, with its own DRP clock, for a design that
selects it (as USB3-PIPE does with a register). While input 1 is selected, the DRP access of the FSM
never completes and its 4 ms watchdog restarts it again and again: select it only around the user
accesses. The tests of `test_serdes_gtp_drp.py` model both cases.

### RX timing {#s7-gtp-rx-timing}

The GTP wrapper registers the received word once before the 8b/10b decoders. This costs one RX
clock cycle of latency. Without it, the path from the channel output through the decoders missed
timing at 250 MHz in an Acorn build ([Results recorded on hardware](#perf-recorded)).

The PRBS checker and the clock aligner take the word before this register.

## GTX and GTH {#s7-gtx-gth}

The GTX and the Virtex-7 GTH wrappers have the same structure and the same registers. They differ
in the CPLL range and in one init step: the GTX waits 500 ns before releasing the channel reset,
the GTH does not.

**CPLL or QPLL.** Pass the channel PLL of the family (`GTXChannelPLL` on GTX, `GTHChannelPLL` on
GTH) for rates up to 6.6 Gbps on GTX and 12.5 Gbps on GTH (solver limits), with one PLL per
channel and no sharing.

Pass the quad PLL (`GTXQuadPLL` or `GTHQuadPLL`) for the higher rates, or to feed several channels
from one PLL. Each wrapper accepts only the PLL classes of its own file. The quad PLL takes its reference clock on `GTREFCLK0` only; the
7-Series GTX and GTH classes have no fabric reference clock option.

**PLL ownership.** `pll_master=True` (the default) lets the TX init FSM reset the PLL. With a
shared QPLL, set it on one wrapper only ([Sharing a PLL](#sd-sharing)).

The KC705 bench uses a GTX with a 125 MHz reference clock, the CPLL by default and the QPLL with
`--pll qpll`; it is described in [Benches and examples](#examples).

## Electrical defaults {#s7-electrical}

The electrical registers ([Electrical settings](#sd-electrical)) have the same fields on the three
transceivers. Without `add_controls`, the channel uses the same values as constants.

| Register | Bits | Reset | Role |
|--------------------------|------|-----------|---------------------------------|
| `tx_diffctrl` | 4 | `0b1000` | Driver swing |
| `tx_postcursor` | 5 | `0b00000` | Post-cursor pre-emphasis |
| `tx_postcursor_inv` | 1 | `0b0` | Post-cursor polarity |
| `tx_precursor` | 5 | `0b00000` | Pre-cursor pre-emphasis |
| `tx_precursor_inv` | 1 | `0b0` | Pre-cursor polarity |

The value-to-voltage tables are in UG482 for the GTP and UG476 for the GTX and GTH. The register
descriptions cite them.

## Limitations {#s7-limitations}

- **GTP 40-bit words.** The GTP internal datapath is 2-byte only, so UG482 gives a 2:1 ratio
  between the user clock pins with the 4-byte interface. With `data_width=40`, a `S7PLL` makes
  `TXUSRCLK` (line rate / 20) and the `tx` domain on `TXUSRCLK2` (line rate / 40) from TXOUTCLK,
  and the same pair for the RX from RXOUTCLK. With `tx_clk` or `rx_clk`, pass the line rate / 20
  clock as `tx_usrclk` or `rx_usrclk`; without it, both pins take the external clock. No run
  records this mode, and the PLLs do not follow a rate change.
- **Init delay.** The 500 ns delay of the GTP and GTX runs once after the `sys` reset, not after
  each restart.
- **Virtex-7 GTH ranges.** The GTH solvers use ranges of other families ([PLLs and line
  rates](#s7-plls)).

## Users {#s7-users}

USB3-PIPE drives the GTP of the Acorn and the GTX of the KC705 through these wrappers. The LitePCIe
soft PHY uses the GTP and GTX wrappers, and the HDMI-over-GT video PHY of LiteX the GTP. LiteSATA
and LiteEth reuse the PLL classes and the init FSMs with their own channel instances.

## In the code {#s7-code}

The wrappers are in `liteiclink/serdes/`, the benches in `bench/serdes/`.

| File | Content |
|----------------------------|---------------------------------------------------|
| `gtp_7series.py` | `GTPQuadPLL`, `GTP` |
| `gtp_7series_init.py` | `GTPTXInit`, `GTPRXInit` (DRP workaround) |
| `gtx_7series.py` | `GTXChannelPLL`, `GTXQuadPLL`, `GTX` |
| `gtx_7series_init.py` | `GTXTXInit`, `GTXRXInit` |
| `gth_7series.py` | `GTHChannelPLL`, `GTHQuadPLL`, `GTH` |
| `gth_7series_init.py` | `GTHTXInit`, `GTHRXInit` |
| `sqrl_acorn.py`, `kc705.py` | GTP and GTX benches |

Where to go next: [Xilinx UltraScale](#xilinx-ultrascale) for the GTH and GTY of the newer
families, [Integration](#integration) to add a wrapper to a SoC, and [Debugging](#debugging) for a
link that does not come up.
