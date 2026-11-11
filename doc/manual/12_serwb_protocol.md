# SerWB packets and protocol {#serwb-protocol}

Above the PHY, SerWB moves packets of 32-bit words, and the bus protocol turns bus accesses into
request and response packets. This chapter describes the packet layer and its CRC, the native and
Etherbone protocols, the SerIO and user ports, and the use of SerWB from an AXI SoC.

The general principles (posted writes, blocking reads, ordering) are in
[Wishbone over serial](#serwb-primer).

## At a glance {#swb-proto-glance}

| Item | Value |
|-----------------------|------------------------------------------------------------------|
| Packet | Preamble, port and length header, payload, optional CRC16 word |
| Preamble | `0x5aa55aa5` |
| Ports | 8-bit port number; port 0 is the bus protocol |
| Integrity | Optional CRC-16 (CCITT) per packet, invalid packets dropped |
| Bus protocol | Native MMAP (default) or Etherbone (deprecated), same on both sides |
| Writes | Posted, consecutive writes merged into one request |
| Reads | Blocking round trips of one word |
| Errors | Link down and remote read errors return a bus error |

## Packet layer {#swb-packet}

Every packet starts with a fixed preamble word, then a header word with the port and the length of
the payload in bytes, then the payload words. With `with_crc=True`, a CRC word ends the packet.
The figure below shows the layout.

![SerWB packet: preamble, header word with the port in bits 7:0 and the payload length in bytes in
bits 23:8, payload words, and the optional CRC16 word.](swb_packet.svg)

The receiver (`Depacketizer`) discards every word until it sees the preamble, which skips the idle
words between packets. It then reads the header and forwards `length / 4` payload words to the
port, marking the last one. Two cases end a packet early:

- A header with less than 4 bytes of payload: the packet is dropped.
- No word for `depacketizer_timeout` seconds, 1 ms by default, inside a packet: the receiver
  returns to the preamble search. The timer does not run while the port endpoint stalls the
  packet, only while no word arrives.

Without CRC, the payload words go to the port as they arrive. A packet cut by the timeout ends
with an error word, marked last and with the `error` flag: it is not packet data, and the bus
protocol discards the request or fails the read instead of taking the next packet as its rest.

Without flow control, idle words fill every word slot, so the timer does not run inside a packet,
and a lost word is replaced by the following word. Only `with_crc=True` protects against both.

### CRC {#swb-crc}

Without CRC, a bit error on the link corrupts the access silently: a wrong address, a wrong data
word or a wrong byte enable. `with_crc=True`, on both cores, adds a CRC-16 word to each packet:

| Item | Value |
|-----------------|-------------------------------------------------------------|
| Polynomial | `0x1021` (CRC-16 CCITT) |
| Initial value | `0xffff` |
| Covered words | Header word and payload, not the preamble |
| CRC word | 16-bit CRC in bits 15:0, bits 31:16 sent and checked as zero |

The receiver (`CRCDepacketizer`) stores the payload, checks the CRC word and only then forwards the
packet. It drops a packet with a wrong CRC, a payload larger than `crc_max_words`, or a timeout.

The `crc_errors` register of the core counts the CRC failures. Packets dropped for their size or a
timeout, and packets whose preamble is corrupted, are not counted. Corrupted data never reaches a
bus; a dropped write is lost, and a dropped read ends with a bus error after the response
timeout (see [Reads](#swb-reads)).

The CRC engine is the `LiteEthMACCRCEngine` of LiteEth, imported by `liteiclink/serwb/packet.py`.
SerWB therefore needs the LiteEth package installed, whatever the protocol and even without CRC.

## Native MMAP protocol {#swb-mmap}

The native protocol (`protocol="native"`, the default) is designed for the point-to-point link. The
SoC side core sends requests; the remote side core executes them on its bus and answers reads.
Every packet goes on the port of the bus protocol, port 0 by default. The figure below shows the
payload words.

![Native MMAP payloads: a request is a command word, a word address and the write data; a read
response is a status word and the read data. The command and status words carry the 0x5357
magic.](swb_mmap.svg)

| Word | Field | Meaning |
|---------|-------------------|---------------------------------------------------------------|
| Command | `[0]` op | 0: write, 1: read |
| | `[7:4]` sel | Wishbone byte enables, the same for every word of the request |
| | `[15:8]` count | Number of words minus one, up to 256 words |
| | `[31:16]` magic | `0x5357` |
| Address | All bits | Word address of the first access, incremented for the next ones |
| Status | `[0]` err | A Wishbone error happened on one of the reads |
| | `[31:16]` magic | `0x5357` |

### Writes and coalescing {#swb-coalescing}

Writes are posted: the SoC side acknowledges a write as soon as it stores it, before it crosses the
link. Consecutive writes to incrementing addresses with the same byte enables join the pending
request, up to `burst_depth` words. The figure below shows four writes merged into one request,
then a read.

![Write coalescing: four posted writes to consecutive addresses are acknowledged at once and sent
as one request; the following read waits for its response.](swb_coalesce.svg)

The SoC side sends the pending request when one of these happens:

- no access arrives for `coalescing_timeout` cycles, 4 by default;
- the request holds `burst_depth` words;
- an access that cannot join it arrives, a read or a non-consecutive write, which keeps the order
  of the accesses.

With `coalescing_timeout=0`, every write is sent at once in its own request.

### Reads {#swb-reads}

A read is a round trip: the SoC side sends a one-word read request and holds the bus access until
the response arrives.

The SoC side returns the data with an acknowledge, or a bus error when the status
reports an error, when the response has no data word, or when its magic is wrong. The SoC side
handles one access at a time, so reads and writes keep their order.

A read request or response can be lost while the link stays up, for example dropped by the CRC
check or by an RX overflow. The SoC side then ends the read with a bus error after
`response_timeout` seconds, 1 ms by default, shorter than the LiteX bus timeout.

A response that arrives after the timeout is dropped: the SoC side waits for it up to one more
timeout before it serves the next read. A response to a read that the bus master gave up, for
example after the LiteX bus timeout, is dropped too, and never acknowledges the next access.

### Remote side {#swb-mmap-remote}

The remote side core checks the magic of each request. A request with another magic, such as an
Etherbone packet, is dropped with the rest of its packet, so a protocol mismatch never reaches the
remote bus. A write request without data words is dropped; the words that follow the announced
count of a write request are discarded.

A write request becomes `count` Wishbone writes at incrementing addresses. A read request becomes
reads into a buffer of `burst_depth` words, then a response; a read request longer than the
buffer is dropped. Errors of the reads are reported in the status word; errors of the writes are
not reported.

### Word counts {#swb-words}

Each access costs a fixed number of words on the link, counted from the packet layouts (preamble
and header included, plus one CRC word per packet with `with_crc=True`):

| Access | Native | Etherbone |
|---------------------------------|------------------|-------------------|
| Single write | 5 | 7 |
| Four consecutive writes | 8, one request | 28, four requests |
| Read, request and response | 4 + 4 | 7 + 7 |

The request and the response of a read go on the two lanes of the link, so a read takes the time
of a round trip rather than 8 word slots of one lane. [Performance and resources](#performance)
turns these counts into throughputs.

### Native protocol limits {#swb-mmap-limits}

- Reads are single words: the remote side accepts multi-word read requests, but the SoC side does
  not send them.
- Errors of remote writes are not reported, since writes are posted.
- Responses carry no request tag. A response delayed by more than twice `response_timeout`, by a
  remote bus slower than that, can still be taken as the response of a later read.

## Etherbone protocol (deprecated) {#swb-etherbone}

The Etherbone protocol carries each access as an Etherbone record, with the Etherbone modules of
LiteEth. `protocol="etherbone"` emits a `FutureWarning`, and the protocol will be removed.

| Item | Etherbone over SerWB |
|------------------|-----------------------------------------------------------|
| Packet | One Etherbone record per packet |
| Fields | 32-bit addresses and data, little-endian on both sides |
| Not supported | Probing, address spaces |
| Single access | 7 words: 2 packet, 2 Etherbone, record header, address, data |
| Writes | Not merged |
| Remote errors | Not reported |

Both sides of a link must use the same protocol. A pair built with Etherbone moves to the native
protocol by rebuilding both sides together, or keeps `protocol="etherbone"` on both sides.

A mismatched pair does not execute any access: the magic checks on each side drop the requests. A
native SoC side then ends every read with a bus error after the response timeout
([Reads](#swb-reads)).

## SerIO and user ports {#swb-serio}

`SERIOCore(serwb_core, port=1)` forwards 32 static or slow signals in each direction. Its input
word `i` is resynchronized bit by bit, and each change sends a packet of one payload word, three
words on the link with the preamble and header.

On the other side, the output word `o` takes the
new value when the packet arrives. Bits are resynchronized independently, so a multi-bit value that
changes at once may cross in two steps.

Only changes are sent, not a periodic copy of the state: after the other FPGA restarts, its outputs
stay at their reset value until an input changes.

A link can carry several SerIOs on different ports: the ECPIX-5 and iCEBreaker demo uses ports 1
and 2. Both sides must declare the same ports, as packets for an unknown port are dropped.

A user module can use any free port with a pair of stream endpoints of `packet_description(32)`:

```python
from litex.soc.interconnect  import stream
from liteiclink.serwb.packet import packet_description

self.tx = tx = stream.Endpoint(packet_description(32)) # To the other side.
self.rx = rx = stream.Endpoint(packet_description(32)) # From the other side.
self.serwb_core.add_downstream_endpoint(port=3, endpoint=tx)
self.serwb_core.add_upstream_endpoint(  port=3, endpoint=rx)
```

On the downstream endpoint, the module sets `length` (bytes) for the packet and `last` on its final
word; the core sets the port. The upstream endpoint receives `data`, `length` and `last`, and
`error` on the last word of a packet cut by the timeout, which must then be discarded. The
endpoints are connected when the core is finalized, so they are added while the SoC is built.

- Set `length` to four times the number of words: the receiver frames the packet on `length`,
  not on `last`.
- With CRC, keep packets within `crc_max_words`, 16 words with the default parameters; longer
  packets are dropped.
- Keep the upstream endpoint always ready, or enable flow control: it shares the RX buffer with
  the bus protocol, and a stalled port fills it and blocks the bus traffic behind its packet.

## AXI and AXI-Lite SoCs {#swb-axi}

SerWB exposes Wishbone. In a LiteX SoC built with `bus_standard="axi-lite"` or `"axi"`,
`add_slave` and `add_master` insert the LiteX bridges of `litex.soc.interconnect.axi`. Outside a
SoC, a bridge is added by hand:

```python
from litex.soc.interconnect.axi import AXILiteInterface, AXILite2Wishbone

axi_lite = AXILiteInterface(data_width=32, address_width=32)
self.axi2wb = AXILite2Wishbone(axi_lite, self.serwb_core.bus)
```

Two behaviours come from the bridges:

- AXI bursts are split into single beats, and each beat is a separate SerWB access.
- `AXILite2Wishbone` returns a Wishbone error as `SLVERR`, so with the link down an AXI-Lite
  access ends at once with `SLVERR`.

## In the code {#swb-proto-code}

| Name | File | Role |
|-----------------------|------------------|-----------------------------------------|
| `Packetizer` | `packet.py` | Preamble, header, CRC word |
| `Depacketizer` | `packet.py` | Preamble search, header, timeout |
| `CRCDepacketizer` | `packet.py` | Store, check and forward, `crc_error` |
| `SERWBMMAPSlave` | `mmap.py` | SoC side native protocol, write coalescing |
| `SERWBMMAPMaster` | `mmap.py` | Remote side native protocol |
| `Etherbone` | `etherbone.py` | Deprecated protocol on LiteEth Etherbone |
| `SERIOCore` | `core.py` | 32 inputs and outputs on one port |

The files are in `liteiclink/serwb/`. The end-to-end tests, with both protocols, flow control,
CRC, SerIO, AXI-Lite and injected bit errors, are in `test/test_serwb_link.py`; the packet layer
tests are in `test/test_serwb_packet.py`.

Where to go next: [Integration](#integration) for a SerWB link between two SoCs,
[Simulation and tests](#simulation) for the Verilator simulation of a link,
[Performance and resources](#performance) for throughput and resource figures.
