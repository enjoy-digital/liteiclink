# Wishbone over serial {#serwb-primer}

This chapter explains the ideas behind SerWB: how a bus reaches a second FPGA over a few regular
IOs. It covers source-synchronous links, the sampling window, the role of 8b/10b on IOs, the
mapping of bus accesses to packets, and what happens when the link fails. Part III then describes
the implementation.

## The problem {#serwbp-problem}

A design sometimes spans two FPGAs: a main FPGA with the CPU, and a second one that drives
converters, IOs or a front-end board. The CPU must reach the registers and the memory of the
second FPGA, ideally as if they were on its own bus, and with as few pins as possible between
the chips.

A parallel bus between the FPGAs costs tens of pins. A transceiver link costs few pins, but not
every FPGA has a transceiver, and the transceivers are often taken by the main function of the
board. SerWB needs three links on regular IOs: a clock, a TX and an RX, each a differential pair
or a single wire.

## Source-synchronous links on regular IOs {#serwbp-ios}

Regular IOs have no CDR. The transmitter therefore sends its clock beside the data, on its own
link: the link is *source-synchronous*. The receiver samples the data with this *forwarded
clock*, so both ends share one frequency and only a phase difference remains to correct.

FPGA IOs also have serializers, much simpler than a transceiver: *OSERDES* and *ISERDES* on
Xilinx, LVDS serializer blocks on Efinix. They shift 8 bits per cycle of the system clock out at
a faster IO clock, using both of its edges (DDR). With an IO clock at four times the system
clock, 8 bits leave per system cycle.

The high-speed PHYs use these serializers and LVDS pairs. The generic PHY uses plain flip-flops:
1 bit per system cycle, on single-ended or differential IOs, on any FPGA. It is much slower but
needs nothing specific from the vendor.

In SerWB, the link master generates the clock. On 7-Series, it sends a pattern of five ones and
five zeros through an OSERDES, a clock at one tenth of the line rate; the link slave
multiplies it back with a PLL. A 7-Series PHY on a 156.25 MHz system clock runs at 1.25 Gbps.

## Sampling window and delay scan {#serwbp-window}

Even with a forwarded clock, the receiver does not know where in the bit period it samples. The
traces, the IO buffers and the serializers add delays that differ between the clock and the
data. The receiver must place its sampling point in the middle of the valid part of the bit, the
*sampling window*.

An *input delay* line in front of the deserializer shifts the data by small steps, the *taps*.
The receiver scans the taps while the other side sends a known pattern, and records the first
and the last tap where the pattern is received correctly. It then sets the delay to the centre,
as the figure below shows.

![Sampling window scan. Stepping the input delay tap by tap, the receiver finds the first tap
where the pattern is seen (min) and the last one (max); the sampling point is set halfway, in the
centre of the data eye.](serwbp_window.svg)

The pattern must also land on the word boundary, which the receiver does not know either. A
*bitslip* moves the word boundary by one bit. The scan therefore tries each bit position in turn,
and for each one scans the delay taps.

In SerWB, the delay lines are the `IDELAYE2` of 7-Series (32 taps), the `IDELAYE3` of UltraScale
(512 taps) and the Titanium delay blocks (64 taps); Trion PHYs use static delays.

The window must be at least 1/16 of the taps wide. The generic PHY has no delay line and scans the bit position
only. [SerWB PHYs and link initialization](#serwb-phys) gives the sequence and the registers.

## Why 8b/10b on IOs {#serwbp-8b10b}

A source-synchronous link needs no transitions for clock recovery, but SerWB uses 8b/10b all the
same. The code gives the receiver what the scan needs: a comma, K28.5, that appears only at a
symbol boundary, and a simple check of the received symbols.

During initialization, each side sends commas. The receiver knows it samples correctly, on the
right bit boundary, when it sees the comma. Once the link runs, the receiver checks that each
symbol has four, five or six ones; a symbol that fails reveals a bit error, although many single
bit errors keep a valid count and pass. The optional CRC catches the rest ([Flow control and
integrity](#serwbp-flow)). With flow control, K characters also carry the pause requests and the
idle fill.

In SerWB, each 32-bit word is sent as four symbols, 40 bits on the line. At 1.25 Gbps, the link
carries 31.25 million words per second.

## Bus accesses as packets {#serwbp-packets}

A bus access does not cross the link as signals: it becomes a short *packet*. The side that
receives the CPU accesses, the *SoC side*, exposes a bus slave. It turns each access into a
*request*. The *remote side* replays the request on its own bus as a bus master, and returns a
*response* for reads.

The two kinds of access behave differently, as the figure below shows. A write is *posted*: the
SoC side acknowledges it as soon as the request is queued, so the CPU does not wait for the
round trip. A read is *blocking*: the CPU waits until the response comes back with the data.

![Accesses over SerWB, time going down. A write request is acknowledged locally and executed later
on the remote bus; a read request waits on the SoC side until the read response returns with the
data.](serwbp_access.svg)

Posted writes raise two questions. Order: a read that follows a write must see it, so the SoC side
sends its requests in the order of the accesses. Errors: a write has no response, so a bus error on
the remote side cannot reach the CPU; only reads report errors.

Small requests also waste the link. Each request carries a command word and an address before its
data, so consecutive writes to incrementing addresses are best merged into one request, a
*write coalescing* that saves a header per word.

In SerWB, the *native protocol* (`SERWBMMAPSlave` and `SERWBMMAPMaster`) does all of this: posted
writes, coalescing of up to `burst_depth` words, blocking reads and error reporting on reads.
[SerWB packets and protocol](#serwb-protocol) gives the formats.

## Flow control and integrity {#serwbp-flow}

The remote bus can be slower than the link: a slow peripheral, or a burst of posted writes. The
receiver buffers the incoming words, and its buffer can overflow. *Flow control* lets the
receiver pause the transmitter: it sends XOFF when its buffer is almost full and XON when it has
room again.

Bit errors are rare on a calibrated link, but not impossible. Without a check, a wrong bit in an
address or in data reaches the remote bus silently. A *CRC* over each packet lets the receiver drop
a corrupted packet instead of executing it.

In SerWB, both are options, to be enabled on both sides: `with_flow_control` sends XOFF and XON as
K characters, and `with_crc` adds a CRC16 to each packet. The error counters of the PHY and the
core count invalid words, link drops, CRC errors and RX buffer overflows.

## When the link goes down {#serwbp-down}

A link can be lost: a cable unplugged, a board reset, a drift that moves the sampling window.
The SoC side must not hang the CPU bus while waiting for responses that never come. The remote side
must not execute half a packet.

In SerWB, the cores are held in reset while the PHY is not ready, and the SoC side flags every
access with a bus error until the link is back. The link master can restart the initialization by
a register write, or automatically with `with_auto_retrain` when it receives too many invalid
words. A read response lost while the link stays ready is not covered: [SerWB packets and
protocol](#serwb-protocol) gives the limits.

## Where to go next {#serwbp-next}

[SerWB architecture](#serwb-architecture) describes the cores and the PHYs;
[Getting started](#getting-started) runs a first SerWB link in simulation or on a board.
