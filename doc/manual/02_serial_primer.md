# Serial links in a nutshell {#serial-primer}

This chapter explains high-speed serial links to an FPGA engineer who has never used a
transceiver. It starts from the serializer and the clock, then gives the line codes, the
alignment, the buffers, the loopbacks and the bit error measurement that every serial protocol
relies on. It ends with the eye and the equalizers.

Each section starts with the idea, then gives the mechanism, and ends with the part of LiteICLink
that implements it. The next chapter, [FPGA transceivers](#transceiver-primer), shows how the
vendors package these functions in their hard blocks.

## Why serial {#primer-why}

A parallel bus sends each bit of a word on its own wire, with a clock beside them. At a few
hundred MHz the wires of a bus no longer arrive at the same time: the difference in their
lengths, the *skew*, eats the bit period. Wide buses also cost many pins and a careful layout.

A serial link sends the bits one after the other on a single differential pair, and the receiver
recovers the clock from the data itself. There is no skew between wires because there is only one,
so the bit rate can climb to several Gbps, over a cable or a backplane. A link uses two pairs, one
per direction.

The price is the logic around the pair: the transmitter must serialize, encode and drive the
line; the receiver must recover the clock, find the word boundaries and decode. An FPGA
transceiver does most of this in hard logic; the rest of this chapter explains each step.

## Serializer and deserializer {#primer-serdes}

The *serializer* takes one parallel word per cycle of the user clock and shifts it out bit by
bit at the line rate. The *deserializer* does the reverse: it samples the line, collects the bits
and outputs one parallel word per cycle. Together they form a *SerDes*. The figure below shows
the path from the transmitter to the receiver.

![A serial link. The serializer shifts out the parallel word at the line rate; at the other end,
clock and data recovery extracts a clock from the bit stream and the deserializer rebuilds the
parallel words.](primer_serdes.svg)

The word width and the line rate fix the user clock: a 20-bit word at 5 Gbps gives a 250 MHz
user clock, and a 40-bit word at the same rate gives 125 MHz. A wider word lowers the clock seen
by the fabric at the cost of more logic per cycle.

The time of one bit on the line is the *unit interval* (UI): 200 ps at 5 Gbps. Jitter, eye width
and alignment steps are measured in UI.

In LiteICLink, the `data_width` parameter of a wrapper sets the word width (20, 40 or 80 bits),
and the wrapper computes its user clock frequency as `tx_clk_freq`, the line rate divided by the
word width.

## Reference clock and PLL {#primer-pll}

The serializer needs a clock at the bit rate, or at half of it when it uses both clock edges.
No board distributes a 5 GHz clock: it provides a *reference clock* of some tens or hundreds of
MHz, and a PLL inside the transceiver multiplies it. The figure below shows the parts of such a
PLL.

![A transceiver PLL. The phase frequency detector compares the divided reference clock with the
divided VCO output; the VCO runs at a multiple of the reference, and an output divider gives the
line rate.](primer_pll.svg)

The *VCO* (voltage-controlled oscillator) runs at a frequency that the loop locks to the
reference clock divided by M and multiplied by the feedback divider N. An output divider D then
selects one of several line rates from the same VCO. Each VCO has a frequency range, so only some
combinations of reference clock and line rate exist.

For the channel PLL of a Xilinx GTX, the relations are:

- VCO frequency = reference clock x N1 x N2 / M, between 1.6 and 3.3 GHz;
- line rate = VCO frequency x 2 / D, the factor 2 coming from the use of both clock edges.

With a 125 MHz reference clock and a 5 Gbps line rate, the PLL solver of LiteICLink picks
N1 = 4, N2 = 5, M = 1 and D = 1: the VCO runs at 2.5 GHz. The same VCO gives 2.5 Gbps with D = 2
and 1.25 Gbps with D = 4. These values are computed by `GTXChannelPLL.compute_config`.

The reference clock quality matters. Its *jitter*, the random variation of its edges, passes
through the PLL to the line when it is slow enough for the loop to follow. Its frequency error, given in parts per million (ppm),
makes the two ends of a link run at slightly different rates when each has its own oscillator.

In LiteICLink, every family except the GTR12 has PLL classes, such as `GTXChannelPLL`. Their
configuration method searches the dividers and raises an error when no setting exists. Printing a
PLL object shows its diagram and the selected dividers. [SerDes wrapper
architecture](#serdes-architecture) describes the solver.

## Clock and data recovery {#primer-cdr}

The receiver gets no clock: it has to find the bit boundaries in the data. A *clock and data
recovery* circuit (CDR) runs a local clock near the line rate and moves its phase towards the
transitions it sees, so that it samples each bit in its middle. When the phase is stable, the CDR
is *locked*.

The CDR learns only from transitions. A long run of identical bits gives it nothing to track,
and its phase drifts. Every serial protocol therefore bounds the *run length*, the number of
identical bits in a row, by its line code or by a scrambler.

The CDR also absorbs the small frequency difference between the two ends: its recovered clock
follows the transmitter, not the local reference. The parallel data leaves the deserializer on a
clock derived from it, the *recovered clock*.

In LiteICLink, the GTX, GTH and GTY init FSMs give the CDR a fixed 1024 cycles to lock before
they continue; this is a wait, not a lock detection, and the GTP FSM has none. The GTR12 wrapper
reports its CDR lock in its status register.

## Line coding {#primer-coding}

A line code maps the data to the bits on the line. It has three jobs: bound the run length for
the CDR, keep as many ones as zeros on the line (*DC balance*) for AC-coupled links, and provide
special patterns that cannot appear in data, for alignment and control.

### 8b/10b {#primer-8b10b}

8b/10b maps each byte to a 10-bit *symbol*. The byte is split into 5 and 3 bits, coded on 6 and
4 bits, as the figure below shows. A symbol has 4, 5 or 6 ones, and no more than 5 identical bits
follow each other on the line.

![8b/10b coding. The low 5 bits of the byte map to a 6-bit sub-block and the high 3 bits to a
4-bit sub-block. Below, K28.5 with negative running disparity: the comma, two zeros followed by
five ones, marks the symbol boundary.](primer_8b10b.svg)

Symbols with 4 or 6 ones are not balanced. The encoder keeps a *running disparity* (RD), the sign
of the imbalance so far, and picks for each byte one of two symbols so that the imbalance never
grows beyond one symbol.

A symbol that breaks this rule, or that is not in the table, is a *disparity error* or an
*invalid symbol*: the receiver detects bit errors this way.

Besides the 256 data symbols (written Dx.y), the code has 12 control symbols, the *K characters*
(Kx.y). K28.5 is the most used: it contains a *comma*, a pattern that no sequence of data symbols
can produce, so the receiver can find the symbol boundaries by searching for it.

The cost is the overhead: 10 line bits for 8 data bits, so a 5 Gbps link carries 4 Gbps of data.
8b/10b is the code of 1000BASE-X Ethernet, SATA, PCIe Gen1 and Gen2, USB 3.2 Gen 1, and most
FPGA-to-FPGA links.

In LiteICLink, the 8b/10b encoder and decoders are the LiteX `Encoder` and `Decoder` of
`litex.soc.cores.code_8b10b`, in the fabric; the GTR12 has its own decoder with the same
interface. The wrappers bypass the coder of the transceiver.
The stream endpoints carry each byte with a K flag, `data` and `ctrl`.

### 64b/66b and 128b/132b {#primer-64b66b}

Faster protocols use codes with less overhead. 64b/66b, the code of 10GBASE-R Ethernet (IEEE
802.3 clause 49), sends 64 bits of payload after a 2-bit *sync header*: `01` for data, `10` for a
block that carries control characters. The payload is scrambled, which bounds the run length in
practice instead of by construction.

The sync header always has one transition, so the receiver finds the block boundary by sliding
until the header is valid in every block. The overhead is 2 bits in 66, about 3 %. USB 3.2 Gen 2
uses 128b/132b with a 4-bit header, and PCIe Gen3 uses 128b/130b with a 2-bit header, on the same
principle.

LiteICLink does not implement these codes. A core that needs them sends *raw symbols*: the
wrappers of the GTY, the ECP5 and the GTR12 can pass words untouched between the user and the
transceiver, and the core does the coding and the scrambling in its own logic.

## Comma and word alignment {#primer-alignment}

The deserializer outputs words of 20, 40 or 80 bits, but it does not know where a symbol starts. After
lock, the boundary of its words falls anywhere in the bit stream. The receiver searches the comma
and moves the word boundary until the comma starts a symbol, as the figure below shows.

![Word alignment. Before alignment the comma of K28.5 straddles two received words; shifting the
boundary by 15 bits puts each symbol at the start of a 10-bit slot, and K28.5 at bits `[9:0]` of
word 0.](primer_alignment.svg)

There are two ways to move the boundary. The receiver can *slip* bits: drop one bit, or shift a
barrel shifter, until the comma is in place. Or it can restart the deserializer until the phase it
picks happens to be the right one; this keeps the latency of the receiver fixed and known, which
some protocols need.

Bit order matters when reading a symbol. 8b/10b symbols are written `abcdei fghj` and sent bit
`a` first. Xilinx transceivers send bit 0 of a word first, so a symbol stored in a register has
`a` at bit 0, the reverse of the written order: K28.5 with negative RD is written `0011111010`
and stored `0b0101111100`.

In LiteICLink, the Xilinx wrappers use the clock aligner (`BruteforceClockAligner`) by default:
it restarts the receiver until the comma appears at bits `[9:0]`, the fixed-latency method. With
the RX buffer enabled and the clock aligner disabled, the comma aligner of the transceiver is used
instead.

The ECP5 wrapper enables the comma alignment of the DCU in its default mode, although its source
states that this aligner is inactive on the raw 10-bit path. Its OOB and 40-bit modes align in
the fabric ([Lattice ECP5 DCU](#lattice-ecp5)). The GTR12 uses the comma aligner of its PCS, as
set by its configuration.

## Polarity {#primer-polarity}

A differential pair has a positive and a negative wire. When a board swaps them, to ease the
routing, every bit arrives inverted. The fix costs nothing: the transceiver, or the fabric, inverts
the bits back.

In LiteICLink, the `tx_polarity` and `rx_polarity` parameters of a wrapper set the inversion at
build time, and the Xilinx wrappers also have polarity registers to change it at run time.

## Elastic buffers and fixed latency {#primer-buffers}

Data crosses clocks inside a transceiver. On the TX side, the user clock and the clock of the
serializer have the same frequency but an unknown phase. On the RX side, the recovered clock
follows the remote transmitter and differs by some ppm from the local clocks.

A *buffer* (an elastic FIFO) absorbs the phase difference. It is simple and robust, but its
filling level, and thus the latency, changes from one start to the next. A buffer that absorbs a
frequency difference also needs *clock correction*: the protocol inserts skip symbols that the
receiver may drop or repeat.

The other choice is to bypass the buffer and align the phases instead. The transceiver runs a
*phase alignment* procedure after reset, and the user clock is then derived from the transceiver
clock itself. The latency is fixed and the same after each restart, at the cost of a stricter
clocking scheme.

In LiteICLink, the Xilinx wrappers bypass both buffers by default (`tx_buffer_enable` and
`rx_buffer_enable` at `False`) and run the phase alignment in their init FSMs. The RX user clock is
the recovered clock, so the user logic sees the remote rate; clock correction, when a protocol
needs it, is done by the protocol core.

## Loopbacks {#primer-loopback}

A *loopback* sends the transmitted data back to a receiver, to test part of the link without the
other end. The figure below shows the four loopbacks of a Xilinx transceiver. *Near-end*
loopbacks stay inside the local transceiver; *far-end* loopbacks return what the remote side sent
back to it.

![The four loopbacks. Near-end PCS returns the TX words before the serializer, near-end PMA returns
the serial stream inside the transceiver; far-end PMA and far-end PCS return the received data to
the line.](primer_loopback.svg)

The *PCS* (physical coding sublayer) is the digital part of the transceiver, and the *PMA*
(physical medium attachment) its analog part, with the serializer, the CDR and the line driver.
Near-end PMA loopback is the usual first test: it exercises the PLL, the serializer, the CDR and
the deserializer, without the board, the connector or the cable.

In LiteICLink, the Xilinx wrappers expose the four loopbacks through their `loopback` register;
the `test_prbs.py` host script selects near-end PMA with `--loopback`.

## PRBS and bit error ratio {#primer-prbs}

A link is measured with a known pseudo-random stream. A *PRBS* (pseudo-random binary sequence) is
produced by a linear feedback shift register (LFSR): a shift register whose input is the XOR of
some of its bits. The receiver runs the same register and compares, as the figure below shows.

![PRBS test. The generator LFSR produces the sequence on the TX side; the checker on the RX side
predicts each bit from the bits it received before and counts the mismatches.](primer_prbs.svg)

The common sequences are named by the length of their register. A register of n bits repeats
after 2^n - 1 bits; longer sequences contain longer runs of identical bits and stress the CDR and
the equalizers more.

| Sequence | Polynomial | Period (bits) | Longest run |
|----------|-----------------------|---------------|-------------|
| PRBS7 | x^7 + x^6 + 1 | 127 | 7 |
| PRBS15 | x^15 + x^14 + 1 | 32767 | 15 |
| PRBS31 | x^31 + x^28 + 1 | 2^31 - 1 | 31 |

The checker is *self-synchronizing*: it fills its register with the received bits, so it locks
to the stream after n bits without any handshake. One wrong bit on the line then gives a mismatch
for itself and for each tap it later feeds, so one line error shows as up to three checker errors.

The *bit error ratio* (BER) is the number of wrong bits divided by the number of bits sent. A
test with no error gives no ratio, only a bound.

With 0 errors in N bits, the BER is below 3/N with 95 % confidence (computed from -ln(0.05),
about 3). A 60 s run at 5 Gbps tests 3 x 10^11 bits; 0 errors bound the BER below 10^-11,
computed.

In LiteICLink, the PRBS generator and checker are the LiteX `PRBSTX` and `PRBSRX`, placed on the
encoded words of the wrapper; the transceiver's own PRBS is unused. The error register counts the
user-clock cycles with at least one mismatch.

A non-zero count is therefore neither a count of line errors nor of bit errors: one line error
makes three mismatches, which fall in one or two cycles. The ratio of the count to the bits sent
is a lower bound of the checker error ratio; with a count of 0, the bound 3/N above holds.
[Debugging](#debugging) explains how to run and read a PRBS test.

## The eye, losses and equalization {#primer-eye}

Overlaying many bit periods of the received signal on a scope draws an *eye*: the open area
between the rising and falling edges, where the receiver can sample. Jitter closes the eye from
the sides, noise and losses from the top and the bottom.

A board trace or a cable loses more at high frequencies than at low ones. A single 1 after a run
of 0s does not reach its full level, and each bit leaks into the next ones: this is
*inter-symbol interference*. The figure below shows how the transmitter fights it.

![TX de-emphasis. At full swing, every bit has the same amplitude. With de-emphasis, the bit after
each transition keeps the full swing and the bits that repeat it are lowered, so the transitions,
which the channel attenuates most, stand out.](primer_eye.svg)

The transmitter has three settings: the *swing*, the peak-to-peak amplitude; the *post-cursor*,
which lowers the bits that follow a transition (de-emphasis); and the *pre-cursor*, which acts on
the bit before it. The receiver has its own equalizers: a *CTLE* (continuous-time linear
equalizer), a high-pass filter, and a *DFE* (decision feedback equalizer), which subtracts the
leakage of the previous bits.

Most links are *AC coupled*: a capacitor on each wire blocks the DC level, so that the two ends
can use different common-mode voltages. A capacitor passes no DC, so the line code must be DC
balanced; this is one of the jobs of 8b/10b.

In LiteICLink, the Xilinx wrappers expose the TX swing and the two cursors as registers, listed
in [SerDes wrapper architecture](#serdes-architecture); the RX equalizers keep the settings of the
wrapper. A square wave mode sends a clock-like pattern for a scope.

## Where to go next {#primer-next}

[FPGA transceivers](#transceiver-primer) shows how the vendors build these functions;
[SerDes wrapper architecture](#serdes-architecture) shows how LiteICLink drives them.
