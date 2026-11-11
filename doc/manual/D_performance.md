# Performance and resources {#performance}

This appendix gathers the figures of LiteICLink: the payload and user clock of a SerDes lane, the
throughput of a SerWB link per access type, the resource comparison recorded for SerWB, and the
results recorded on hardware. Computed values are labelled as such and derived in the text; recorded
results are quoted verbatim from the commits that report them.

## SerDes lane: payload and user clock {#perf-serdes}

A wrapper encodes 8b/10b in the fabric: each byte takes a 10-bit symbol on the line, so the payload
is 8/10 of the line rate.

The user clock is the line rate divided by the word width, and a word carries one byte per 10 bits
of width. The table computes both for the line rates used in the benches and in the examples of this
manual.

| Line rate | Payload (computed) | 20-bit words | 40-bit words | 80-bit words (GTY, GTR12) |
|--------------------|-----------------------------------|------------------------|------------------------|-------------------------------------|
| 1.25 Gbps | 125 MB/s | 62.5 MHz | 31.25 MHz | 15.625 MHz |
| 2.5 Gbps | 250 MB/s | 125 MHz | 62.5 MHz | 31.25 MHz |
| 3.125 Gbps | 312.5 MB/s | 156.25 MHz | 78.125 MHz | 39.0625 MHz |
| 5 Gbps | 500 MB/s | 250 MHz | 125 MHz | 62.5 MHz |
| 6.25 Gbps | 625 MB/s | 312.5 MHz | 156.25 MHz | 78.125 MHz |
| 10 Gbps | 1000 MB/s | 500 MHz | 250 MHz | 125 MHz |

The payload is that of the 8b/10b layer only: a protocol on top of the wrapper spends part of it
on its own framing, and a design that sends raw symbols with another line code has another ratio.
A wider word lowers the user clock for the same line rate, at the cost of a wider datapath
([SerDes wrapper architecture](#serdes-architecture)).

No resource figure of the wrappers is recorded. The fabric part of a wrapper is the 8b/10b encoder
and decoders, the PRBS generator and checker, the init state machines and the registers; its cost
grows with the word width.

## SerWB link: line rate and words {#perf-serwb-rate}

SerWB sends each 32-bit word as four 8b/10b symbols, 40 bits on the line. The high-speed PHYs
send 8 bits per cycle of their clock, the generic PHY 1 bit per system clock cycle, so the word
rate is the line rate divided by 40.

| Configuration | Line rate (computed) | Words per second (computed) |
|-----------------------------------------------------------------|--------------------------------|-------------------------------------------|
| 7-Series or UltraScale PHY, sys 156.25 MHz | 1.25 Gbps | 31.25 M |
| 7-Series PHY, sys 125 MHz (Nexys Video bench) | 1 Gbps | 25 M |
| Generic PHY, sys 50 MHz (TrellisBoard, ULX3S) | 50 Mbps | 1.25 M |
| Generic PHY, sys 25 MHz (two-board demo) | 25 Mbps | 0.625 M |

The link runs both directions at the same rate: requests travel on one wire and read responses on
the other.

## SerWB link: throughput per access {#perf-serwb-access}

Every packet carries two words of framing, a preamble and a header, before its payload. The native
protocol adds a command word and an address word to a request, and a status word to a read
response ([SerWB packets and protocol](#serwb-protocol)). The CRC option adds one word per packet.

| Access | Words on the link | With CRC |
|-----------------------------------------------------------------|-------------------------------------------|--------------------------------|
| Single write | 5 | 6 |
| Coalesced write of N words | 4 + N | 5 + N |
| Read of one word | 4 out, 4 back | 5 out, 5 back |

The table below computes the throughput of the bus payload, 4 bytes per access, at 31.25 M words
per second (a high-speed PHY at a 156.25 MHz system clock). Divide by 50 for the generic PHY at
25 MHz.

| Access | Throughput (computed) | With CRC (computed) |
|-------------------------------------------------------------------|---------------------------------------|----------------------------------|
| Single writes | 25 MB/s | 20.8 MB/s |
| Writes coalesced by 4 (`burst_depth=4`) | 62.5 MB/s | 55.6 MB/s |
| Reads, link bound | 31.25 MB/s | 25 MB/s |

Writes are posted: the SoC side acknowledges them at once, so a stream of writes reaches the link
bound when it is coalesced. Reads are blocking: the bus waits for each response, so the round-trip
latency of the PHYs, the cores and the remote bus limits reads well below the link bound in the
table.

The deprecated Etherbone protocol takes 7 words for a single write and 14 for a read round trip.
Idle fill words and the XON/XOFF characters of the flow control are not counted.

No latency or throughput measurement of a SerWB link is recorded. The interactive mode of
`bench/serwb/sim.py` with `--with-debug` prints the latency of each access in cycles, measured on
the generic PHY ([SerWB Verilator simulation](#sim-serwb)).

## SerWB resources {#perf-serwb-resources}

`bench/serwb/resources.py` estimates the resources of a PHY and a core with Yosys. The relative
cost of the native protocol against the deprecated Etherbone protocol is recorded in
`doc/serwb.md` from this script:

> Smaller: about −16% LUTs / −47% LUTRAM on the SoC side and −24% LUTs / −42% FFs on the remote
> side vs Etherbone (`bench/serwb/resources.py`).

The same page gives about 160 LUTs for the `with_prbs` option of the PHY. Absolute counts are not
recorded. With recent Yosys versions, the script reads 0 for every cell
([Resource estimation](#ex-resources)).

## Results recorded on hardware {#perf-recorded}

The results below are quoted verbatim from the commits of the repository that report them, and
from the LitePCIe documentation for the soft PCIe PHY. Each title gives the board and the rate,
or says that the record does not state them. The [status table](#intro-status) summarizes them per
family.

**ECP5 DCU, 40-bit words, LambdaConcept ECPIX-5, 5 Gbps** (commit `939f4ad`):

```text
Tested on the LambdaConcept ECPIX-5 (LFE5UM5G-85F, 5Gbps): 40-bit loopback 1.5e10 symbols 0
errors, USB3 device (enjoy-digital/usb3_pipe ECP5USB3SerDes): SuperSpeed, Bulk IN/OUT 431/449MB/s,
full duplex 342 + 349MB/s, 4 min soak 0 errors.
```

**ECP5 DCU, 2.5 Gbps**, board not stated (commit `37d45f3`, which added the wrapper):

```text
transceiver: add serdes_ecp5 (validated at 2.5gbps)
```

**GTY rate change, Alibaba XCKU3P, 10 and 5 Gbps, PRBS31 in near-end PMA loopback** (commit
`107782b`):

```text
Hardware (Alibaba XCKU3P, GTY QPLL 10Gbps from a 156.25MHz reference, 40-bit, near-end PMA loopback,
PRBS31): rate 000 (10Gbps, 250MHz user clocks) -> 010 (5Gbps, 125MHz) -> 000, 0 errors at each rate.
```

**GTP reference clock from the fabric, SQRL Acorn, 5 Gbps** (commit `095d2a0`):

```text
On the SQRL Acorn (no dedicated GTP reference clock, 200MHz oscillator from the fabric), the
USB3-PIPE RX quality with GTREFCLK0 depended on the place and route (some builds never reached
a stable link, deterministic per bitstream), while GTGREFCLK0 gave stable links on all tested
builds (13/13 loads vs 0/9 for the unstable GTREFCLK0 builds).
```

**GTP RX initialization, SQRL Acorn, 5 Gbps** (commit `f29cd43`, before its fix):

```text
Observed on hardware (usb3_pipe, Acorn GTP at 5Gbps, RX re-initialized on each link training
failure): RXOUTCLK measured at 312.5MHz instead of 250MHz and DRP 0x011 = 0x1130 instead of 0x1930
after some RX re-initializations, link training failing.
```

**GTP RX timing, SQRL Acorn, 5 Gbps** (commit `578074a`):

```text
GTPE2 RXDATA clock-to-out and the decoders logic in a single RX clock cycle missed timing at 250MHz
(usb3_pipe Acorn build: rx_clk WNS -0.800ns on GTPE2_CHANNEL/RXUSRCLK2 -> decoder registers, other
builds with a few tens of ps of slack). With the register: rx_clk WNS +0.410ns. One RX clock cycle of
additional latency.
```

**GTR12, Sipeed Tang Mega 138K Pro, 5 Gbps** (commit `16f1e31`):

```text
Tested on the Sipeed Tang Mega 138K Pro (GW5AST-138B, Q1 lane 0, 100MHz REFCLK1) at 5Gbps with a
USB3 host (usb3_pipe): QPLL1 lock, CDR lock, comma alignment and received training sequences.
```

**GTR12, GW5AT-15, 5 Gbps** (commit `5d098d2`):

```text
Hardware: GW5AT-LV15MG132C1 (device version B, Gowin 1.9.12.04), USB3 at 5Gbps with USB3-PIPE (Q0 lane 0,
125MHz REFCLK0, QPLL0) OK.
```

**SerWB, UltraScale link master to 7-Series link slave**, rate not stated (commit `816996a`):

```text
Tested on HW with a link
between KU (master) and S7 (slave)
```

**SerWB Efinix PHY, Titanium**, rate not stated (commit `568ae16`):

```text
serwb/efinix: Finish clk_ratio support, tested at 1:1, 1:2 and 1:4 on Titanium.
```

**SerWB generic PHY, two-board demo, 50 MHz clock** (commit `f5abc46`; the generic PHY sends one
bit per clock cycle, so 50 Mbps, computed):

```text
bench/serwb/demo: Adjustements, link working at 50MHz.
```

**GTX and GTY, LitePCIe soft PHY** (LitePCIe repository, `doc/softphy.md`; PCIe Gen1, Gen2 and
Gen3 run at 2.5, 5 and 8 Gbps per lane):

```text
Gen1 and Gen2 run up to x8 on a Kintex-7 GTX board, Gen1 and Gen2 x1
and Gen3 up to x8 on a Kintex UltraScale+ GTY board
```

**PRBS error counter**, rate not stated (commit `1ef0fff`, which made the counter wrap instead of
dropping to 0):

```text
Previously if the PRBS counter reached its max value of 0xffffffff,
the reported number of errors would drop to 0. And this happens
pretty quickly given the fast data rates.
```

The ECP5 commit does not say whether a symbol is a 10-bit code group or a 40-bit word. Taking a
symbol as one 10-bit code group ([Glossary](#glossary)), the run of 1.5e10 symbols without error
tests 1.5 x 10^11 bits; with the bound of the primer, the bit error ratio is below 2 x 10^-11 with
95 % confidence, computed ([PRBS and bit error ratio](#primer-prbs)). The other runs do not state
their duration.
