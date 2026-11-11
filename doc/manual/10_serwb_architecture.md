```{=latex}
\part{SerWB}
```

```{=html}
<h1 class="part">Part III. SerWB</h1>
```

# SerWB architecture {#serwb-architecture}

SerWB extends the Wishbone bus of a SoC to a second FPGA over three lanes: a forwarded clock, a
transmit lane and a receive lane. This chapter describes how SerWB is built: the roles of the two
sides, the blocks of the core, its ports and buffers, link down behaviour and clock domains.

The concepts are in [Wishbone over serial](#serwb-primer); the PHYs are in
[SerWB PHYs and link initialization](#serwb-phys), and the packets and the bus protocol in
[SerWB packets and protocol](#serwb-protocol).

## At a glance {#swb-glance}

| Item | SerWB |
|------------------|------------------------------------------------------------------------|
| Wires | Clock, TX and RX: LVDS pairs or single-ended I/Os |
| Line coding | 8b/10b, one 32-bit word as four symbols (40 bits) |
| High-speed PHY | `SERWBPHY` of `serwb/phy.py`: 8 bits per `sys` cycle |
| Generic PHY | `SERWBPHY` of `serwb/genphy.py`: 1 bit per `sys` cycle |
| Core | `SERWBCore`: bus protocol, ports, packets, buffers |
| Bus protocol | Native MMAP (default) or Etherbone (deprecated), on port 0 |
| Other ports | `SERIOCore` (32 inputs, 32 outputs) and user stream endpoints |
| Options | Flow control, CRC, PRBS test, automatic retraining |
| Clock domain | `sys` for the core and the PHY logic |

## Two roles on each side {#swb-roles}

A SerWB side has two roles, set separately on the PHY and on the core. The PHY role decides who
drives the link clock and the initialization. The core role decides which side of the bus it
presents.

| Object | `mode="master"` | `mode="slave"` |
|----------------|----------------------------------------|-------------------------------------------|
| `SERWBPHY` | Drives the link clock, runs the init | Uses the received clock, follows the init |
| `SERWBCore` | Drives a bus master (remote side) | Exposes a bus slave (SoC side) |

The two sides of a link always have opposite modes, for the PHYs and for the cores.

In this manual,
the *SoC side* is the core in `mode="slave"`, whose bus slave is mapped in the address space of
the SoC, and the *remote side* is the core in `mode="master"`, which drives the bus of the other
FPGA. The *link master* and *link slave* are the PHYs.

The code does not tie the two roles together: a SoC side can have either PHY role. The bench
targets and the ECPIX-5 and iCEBreaker demo put the link master on the SoC side; in the demo, the
iCEBreaker runs its whole SoC from the clock it receives.

```python
from liteiclink.serwb.phy  import SERWBPHY
from liteiclink.serwb.core import SERWBCore

# FPGA A, SoC side: link master PHY, core exposing a bus slave.
self.serwb_phy  = SERWBPHY(platform.device, platform.request("serwb"), mode="master")
self.serwb_core = SERWBCore(self.serwb_phy, sys_clk_freq, mode="slave")
self.bus.add_slave("serwb", self.serwb_core.bus,
    SoCRegion(origin=0x30000000, size=0x10000000, cached=False))

# FPGA B, remote side: link slave PHY, core driving a bus master.
self.serwb_phy  = SERWBPHY(platform.device, platform.request("serwb"), mode="slave")
self.serwb_core = SERWBCore(self.serwb_phy, sys_clk_freq, mode="master")
self.bus.add_master("serwb", self.serwb_core.bus)
```

The region is not cached (`cached=False`): a `SoCRegion` is cached by default, and a CPU data
cache would hold stale copies of remote registers. The remote bus receives the address that the
SoC side bus presents, region origin included; [Addresses and register maps](#integ-serwb-map)
shows how to translate it.

## Blocks of the core {#swb-core}

The figure below shows the blocks of one core. Accesses leave through the top row and arrive
through the bottom row; the core of the other side is the same, with the other bus protocol
module.

![Blocks of a SerWB core: the bus protocol module on port 0, the arbiter and packetizer towards the
TX buffer and the PHY, the RX buffer, depacketizer and dispatcher back to the
ports.](swb_blocks.svg)

| Block | Role |
|---------------|---------------------------------------------------------------------------|
| Bus protocol | Turns bus accesses into request packets and back (MMAP slave or master) |
| Arbiter | Picks one port with a packet to send, one whole packet at a time |
| Packetizer | Adds the preamble and the port and length header, and the CRC if enabled |
| TX buffer | FIFO of words in front of the PHY |
| RX buffer | FIFO of received words; it overflows if the core cannot keep up |
| Depacketizer | Finds the preamble, checks the CRC if enabled, rebuilds the packet |
| Dispatcher | Sends each packet to the endpoint of its port |

The link has no backpressure. The PHY transmits a word on every word slot, idle words when the TX
buffer is empty, and the receiving PHY forwards every word it decodes. A full RX buffer loses words,
and the `rx_overflows` register of the core counts these events.

Link flow control, described in [Flow control](#swb-phy-flow), prevents them. Without flow control, the lost words can be idle
words, which carry no data: `rx_overflows` only counts the lost words that belong to a packet,
found from the preamble and the length of each header ([Buffer sizes](#swb-buffers)).

## Ports {#swb-ports}

The packet header carries an 8-bit port number, so several independent streams share one link.
The bus protocol uses port 0 (the `port` parameter of `SERWBCore`).

Other modules add a pair of
endpoints on another port: a downstream endpoint with the packets to send and an upstream endpoint
for the packets received. The figure below shows how received packets are dispatched.

![Port dispatching: the dispatcher sends each received packet to the endpoint of its port, and the
packets of ports without an endpoint to a drop endpoint.](swb_ports.svg)

`add_downstream_endpoint(port, endpoint)` and `add_upstream_endpoint(port, endpoint)` register the
endpoints; a port used twice raises a `ValueError`. The endpoints carry 32-bit words with the
`port` and `length` fields of `packet_description(32)`; `length` is in bytes. Packets received for
a port without an upstream endpoint are dropped, so both sides must declare the same ports.

`SERIOCore` is the ready-made user of a port: it forwards 32 inputs and 32 outputs, on port 1 by
default. It is described in [SerIO and user ports](#swb-serio).

## Buffer sizes {#swb-buffers}

The TX and RX buffers hold at least one full packet of the bus protocol, so that a request or a
response never waits for the link inside a packet. The core computes the defaults from the
protocol parameters:

| Value | Native protocol | Etherbone protocol |
|-------------------------------|-----------------------|-----------------------------------------|
| Largest packet, words | 4 + `burst_depth` | 6 + `etherbone_buffer_depth` |
| With the default parameters | 8 | 10 |
| `tx_buffer_depth` default | Largest packet, at least 8 | Largest packet, at least 8 |
| `rx_buffer_depth` default | Same, plus 16 with flow control | Same, plus 16 with flow control |

With flow control, the RX buffer asks the other side to pause when 16 free words remain, the words
that can be in flight on the link. With `with_crc=True`, each packet carries one more word, which
the defaults do not count.

Without flow control, the receiving PHY pushes one word, data or idle, into the RX buffer at every
word slot. When the remote bus holds a request for more word slots than the buffer has free
entries, the next words are lost.

The lost words may be idle words, which `rx_overflows` counts although no data is lost. They may
also be the words of the next request, which is then lost or corrupted while the SoC side has
already acknowledged it. Larger buffers or flow control avoid this with a slow remote bus.

## Link down {#swb-link-down}

The core only works while its PHY reports the link ready. With `with_rst_on_link_down=True`, the
default, the bus protocol module, the packetizer, the depacketizer and both buffers are held in
reset while the link is not ready. A link loss therefore discards the packets in flight.

While the link is not ready, the SoC side core flags every bus access with an error instead of
leaving the bus waiting. With the native protocol, a write also sees an acknowledge in the same
cycle (the protocol module, held in reset, accepts it), so the bus master must check the error
first.

A CPU or a host tool sees a bus error and can retry after the link comes back.

The link goes down only when the initialization restarts: a write to the `reset` register of the
link master, automatic retraining (`with_auto_retrain`), an idle line seen by the link slave, or a
reset ([Link health](#swb-phy-health)). A link that degrades while it stays ready loses or
corrupts accesses instead of returning errors.

## Clock domains {#swb-clocks}

The core and the PHY logic run in `sys`. The PHYs add the clocks of their I/O primitives:

| PHY | Clocks | Link slave side |
|---------------------|----------------------------------------|-------------------------------------------|
| 7-Series | `sys`, `sys4x`, 200 MHz `IDELAYCTRL` | `sys` and `sys4x` from the received clock |
| UltraScale(+) | `sys`, `sys4x` (delays in tap count) | Same |
| Efinix | `clk` and `clk4x` names, `clk_ratio` | PLL in the PHY, `rx_sys` domain |
| Generic | `sys` | `sys` is the received clock |

On a link slave, the `sys` clock must come from the clock that the link master forwards, because
the PHY samples the data with it and has no elastic buffer. The 7-Series and UltraScale PHYs give
the received clock as `serdes.clocking.refclk` and leave the PLL to the design; the Efinix PHY
instantiates the PLL itself. [SerWB PHYs](#serwb-phys) gives the frequencies.

## Core parameters {#swb-params}

`SERWBCore(phy, clk_freq, mode)` takes the PHY, the `sys` frequency (for the timeouts), the
role and these options:

| Parameter | Default | Role |
|------------------------------------|--------------|-------------------------------------------|
| `with_rst_on_link_down` | `True` | Hold the core in reset while the link is down |
| `port` | 0 | Port of the bus protocol |
| `protocol` | `"native"` | `"native"` or `"etherbone"` (deprecated), both sides |
| `burst_depth` | 4 | Native: largest request, in words |
| `coalescing_timeout` | 4 | Native: idle cycles before pending writes are sent |
| `etherbone_buffer_depth`| 4 | Etherbone: record buffer, at least 2 |
| `tx_buffer_depth` | `None` | TX buffer depth, computed when `None` |
| `rx_buffer_depth` | `None` | RX buffer depth, computed when `None` |
| `depacketizer_timeout` | `1e-3` | Silence inside a packet before the preamble search restarts, seconds |
| `with_crc` | `False` | CRC16 on every packet, both sides |
| `crc_max_words` | `None` | Largest payload accepted with CRC, computed when `None` |

The PHY parameters are in [SerWB PHYs and link initialization](#serwb-phys); every option of the
project is listed in [Build options](#build-options).

## In the code {#swb-code}

| File | Content |
|----------------|---------------------------------------------------------------|
| `core.py` | `SERWBCore`, `SERIOCore` and its packetizer and depacketizer |
| `packet.py` | Packet layout, `Packetizer`, `Depacketizer`, CRC |
| `mmap.py` | Native protocol, `SERWBMMAPSlave`, `SERWBMMAPMaster` |
| `etherbone.py` | Deprecated Etherbone protocol |
| `phy.py` | High-speed `SERWBPHY` and its init state machine |
| `genphy.py` | Generic `SERWBPHY` |

The files are in `liteiclink/serwb/`. The arbiter and the dispatcher are the LiteX `Arbiter` and `Dispatcher` of
`litex.soc.interconnect.packet`.

Where to go next: [SerWB PHYs and link initialization](#serwb-phys) for the line side,
[SerWB packets and protocol](#serwb-protocol) for the words on the link,
[Integration](#integration) for a SerWB link between two SoCs.
