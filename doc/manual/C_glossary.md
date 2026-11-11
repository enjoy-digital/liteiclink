# Glossary {#glossary}

The terms of serial links, FPGA transceivers and LiteICLink used in this manual, in alphabetical
order. Each entry points to the chapter that explains it.

| Term | Meaning |
|-----------------------|---------------------------------------------------------------------------------------------------------------------|
| 8b/10b | Line code that sends each byte as a 10-bit symbol, with a balanced number of ones and zeros and K characters for control. See [Line coding](#primer-8b10b) |
| BER | Bit error ratio: wrong bits divided by bits sent; a test without error gives only a bound. See [PRBS and bit error ratio](#primer-prbs) |
| Bitslip | Shift of the received bits by one position, used to find the word boundary. See [Wishbone over serial](#serwb-primer) |
| Buffer bypass | Transceiver mode without TX or RX elastic buffer, for a fixed latency, with a phase alignment step at init. See [FPGA transceivers](#transceiver-primer) |
| CDR | Clock and data recovery: the part of the receiver that recovers the clock from the transitions of the data. See [Serial links in a nutshell](#primer-cdr) |
| Channel | One TX and RX lane of a transceiver block, with its PCS and PMA. See [FPGA transceivers](#transceiver-primer) |
| Clock aligner | LiteICLink block that restarts the RX until the comma arrives at a fixed position in the word. See [SerDes wrapper architecture](#serdes-architecture) |
| Comma | Symbol containing a bit pattern (the comma) that cannot appear across the symbol boundaries of valid data, used to align words; K28.5 in LiteICLink. See [Comma and word alignment](#primer-alignment) |
| CPLL | Channel PLL of the Xilinx GTX, GTH and GTY: one per lane. See [FPGA transceivers](#transceiver-primer) |
| CTLE | Continuous-time linear equalizer: an RX filter that boosts the high frequencies lost in the channel. See [The eye, losses and equalization](#primer-eye) |
| DCU | Dual channel unit: the transceiver block of the Lattice ECP5, two channels. See [Lattice ECP5 DCU](#lattice-ecp5) |
| De-emphasis | TX equalization that lowers the level of repeated bits relative to transitions, set by the cursors. See [The eye, losses and equalization](#primer-eye) |
| DFE | Decision feedback equalizer: an RX equalizer that subtracts the echo of past bits. See [The eye, losses and equalization](#primer-eye) |
| DRP | Dynamic reconfiguration port: the register access port of a Xilinx transceiver. See [FPGA transceivers](#transceiver-primer) |
| Eye | Overlay of the received bit periods; its opening shows the margin of the sampler. See [The eye, losses and equalization](#primer-eye) |
| Generic PHY | SerWB PHY on plain I/Os, one bit per system clock cycle, for any FPGA. See [SerWB PHYs and link initialization](#serwb-phys) |
| GTR12 | Transceiver of the Gowin GW5AST and GW5AT FPGAs, four lanes per quad. See [Gowin GW5 GTR12](#gowin-gw5) |
| High-speed PHY | SerWB PHY on the serializer blocks of the I/Os of Xilinx and Efinix FPGAs. See [SerWB PHYs and link initialization](#serwb-phys) |
| IDELAY | Programmable input delay of Xilinx I/Os, used to centre the sampling point. See [Wishbone over serial](#serwb-primer) |
| ISERDES | Input deserializer block of Xilinx I/Os; OSERDES is the output serializer. See [Wishbone over serial](#serwb-primer) |
| K character | 8b/10b symbol reserved for control, such as K28.5. See [Line coding](#primer-8b10b) |
| Line rate | Bits per second on the wire, in Gbps, coding included. See [Serial links in a nutshell](#serial-primer) |
| Link master | SerWB PHY that drives the link clock and starts the initialization. See [SerWB architecture](#serwb-architecture) |
| Link slave | SerWB PHY that receives the link clock and follows the initialization. See [SerWB architecture](#serwb-architecture) |
| Loopback | Path that sends the TX back to the RX, near end (inside the transceiver) or far end (at the other side). See [Loopbacks](#primer-loopback) |
| LVDS | Low-voltage differential signalling: the I/O standard of the high-speed SerWB PHYs. See [Wishbone over serial](#serwb-primer) |
| MMAP | The native SerWB protocol: memory-mapped requests and responses. See [SerWB packets and protocol](#serwb-protocol) |
| PCS | Physical coding sublayer: the digital part of a transceiver (coding, alignment, buffers). See [FPGA transceivers](#transceiver-primer) |
| PLL | Phase-locked loop: multiplies the reference clock up to the serial clock. See [Serial links in a nutshell](#primer-pll) |
| PMA | Physical medium attachment: the analog part of a transceiver (serializer, CDR, drivers). See [FPGA transceivers](#transceiver-primer) |
| Polarity | Swap of the P and N wires of a differential pair, corrected by inverting the bits. See [Polarity](#primer-polarity) |
| Posted write | Write that the SoC side acknowledges at once, without waiting for the remote side. See [SerWB packets and protocol](#serwb-protocol) |
| PRBS | Pseudo-random binary sequence, PRBS7, PRBS15 or PRBS31, used to test a link. See [PRBS and bit error ratio](#primer-prbs) |
| QPLL | Quad PLL of the Xilinx transceivers, shared by the lanes of a quad. See [FPGA transceivers](#transceiver-primer) |
| Quad | Group of four transceiver lanes sharing PLLs and reference clocks. See [FPGA transceivers](#transceiver-primer) |
| Raw symbols | Words sent to the transceiver without the 8b/10b encoder, for another line code. See [SerDes wrapper architecture](#serdes-architecture) |
| RD | Running disparity: the sign of the ones-minus-zeros balance that 8b/10b keeps near zero. See [Line coding](#primer-8b10b) |
| Reference clock | Clock given to the transceiver PLL, from which the line rate is derived. See [Serial links in a nutshell](#primer-pll) |
| SCI | Serial configuration interface: the register access port of the ECP5 DCU. See [Lattice ECP5 DCU](#lattice-ecp5) |
| SerDes | Serializer and deserializer; in this manual, the transceiver and its wrapper. See [Serial links in a nutshell](#primer-serdes) |
| SerDes wrapper | LiteICLink class that drives a transceiver: PLL, init, 8b/10b, PRBS and registers. See [SerDes wrapper architecture](#serdes-architecture) |
| SerIO | 32 inputs and 32 outputs tunnelled on a port of a SerWB link. See [SerWB architecture](#serwb-architecture) |
| SerWB | Wishbone over serial: a bus bridge between two FPGAs on three wires. See [Wishbone over serial](#serwb-primer) |
| Symbol | One 10-bit 8b/10b code group. See [Line coding](#primer-8b10b) |
| UI | Unit interval: the duration of one bit on the wire. See [The eye, losses and equalization](#primer-eye) |
| UPAR | Register access port of the Gowin GTR12 quad. See [Gowin GW5 GTR12](#gowin-gw5) |
| User clock | Clock of the parallel words of a lane: the `tx` and `rx` clock domains. See [SerDes wrapper architecture](#serdes-architecture) |
| Word | The parallel data of a lane in one user clock cycle: 2, 4 or 8 bytes. See [SerDes wrapper architecture](#serdes-architecture) |
| XON/XOFF | Flow control of SerWB: K characters that resume or pause the sender. See [SerWB PHYs and link initialization](#serwb-phys) |
