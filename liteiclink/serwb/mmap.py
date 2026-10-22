#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
SerWB native MMAP protocol.

Default SerWB protocol (SERWBCore(protocol="native"), must be used on both sides of the link),
lightweight alternative to Etherbone designed for the point-to-point SerWB link:

Requests (Slave -> Master side):
- Word 0 (cmd)  : [0] op (0: write, 1: read), [7:4] byte enables (sel), [15:8] count - 1,
                  [31:16] magic.
- Word 1 (addr) : Word address of the first access (incrementing for next accesses).
- Words 2..     : Write data (count words, write requests only).

Read Responses (Master -> Slave side):
- Word 0 (status) : [0] err (Wishbone error on one of the accesses), [31:16] magic.
- Words 1..       : Read data (count words).

A single 32-bit write takes 5 words on the link (vs 7 with Etherbone) and a read round trip 8 words
(vs 14). Writes are posted, and consecutive incrementing writes (same byte enables) are coalesced in
a single request (up to `burst_depth` words). Requests/responses are fully buffered before being
sent (no bubbles inside packets). Remote Wishbone errors are reported on reads.

The magic identifies the protocol: requests with an invalid magic (for example Etherbone packets
from a remote side built with protocol="etherbone") are dropped instead of being executed, and read
responses with an invalid magic return an error on the bus.
"""

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone

from liteiclink.serwb.packet import packet_description

# Constants ----------------------------------------------------------------------------------------

MMAP_MAGIC    = 0x5357 # "SW" (Etherbone magic is 0x4e6f).
MMAP_OP_WRITE = 0
MMAP_OP_READ  = 1

# MMAP Slave ---------------------------------------------------------------------------------------

class SERWBMMAPSlave(LiteXModule):
    """Wishbone Slave (SoC side): converts Wishbone accesses to requests, receives read responses.

    ``coalescing_timeout``: idle cycles (no consecutive write) before sending a pending write
    request (0: send writes immediately, no coalescing).
    """
    def __init__(self, burst_depth=4, coalescing_timeout=4):
        assert 1 <= burst_depth <= 256
        self.bus    = bus    = wishbone.Interface(data_width=32, adr_width=30)
        self.source = source = stream.Endpoint(packet_description(32))
        self.sink   = sink   = stream.Endpoint(packet_description(32))

        # # #

        access = Signal()
        self.comb += access.eq(bus.cyc & bus.stb)

        # Pending Write Request.
        # ----------------------
        self.wr_fifo = wr_fifo = stream.SyncFIFO([("data", 32)], burst_depth)
        wr_adr     = Signal(30)
        wr_sel     = Signal(4)
        wr_count   = Signal(max=burst_depth + 1)
        wr_next    = Signal(30) # Address of the next consecutive write.
        wr_pending = Signal()
        wr_append  = Signal()
        wr_accept  = Signal()
        wr_send    = Signal()
        self.comb += [
            wr_pending.eq(wr_count != 0),
            # Consecutive incrementing write with same byte enables: append to pending request.
            wr_append.eq(wr_pending & (wr_count != burst_depth) & (bus.adr == wr_next) & (bus.sel == wr_sel)),
            wr_accept.eq(access & bus.we & (~wr_pending | wr_append)),
        ]
        if coalescing_timeout:
            self.wr_timer = wr_timer = WaitTimer(coalescing_timeout)
            self.comb += [
                wr_timer.wait.eq(wr_pending & ~access),
                # Send when idle, full or when another access (read, non-consecutive write) is
                # pending (ordering).
                wr_send.eq(wr_pending & (wr_timer.done | (wr_count == burst_depth) | (access & ~wr_accept))),
            ]
        else:
            self.comb += wr_send.eq(wr_pending)

        # Request.
        # --------
        op    = Signal()
        adr   = Signal(30)
        sel   = Signal(4)
        count = Signal(max=burst_depth + 1)
        index = Signal(max=burst_depth + 1)

        # Response.
        # ---------
        err   = Signal()
        dat_r = Signal(32)

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(wr_send,
                NextValue(op,    MMAP_OP_WRITE),
                NextValue(adr,   wr_adr),
                NextValue(sel,   wr_sel),
                NextValue(count, wr_count),
                NextState("SEND-CMD")
            ).Elif(wr_accept,
                # Posted write: buffer it and ack.
                bus.ack.eq(1),
                wr_fifo.sink.valid.eq(1),
                wr_fifo.sink.data.eq(bus.dat_w),
                If(~wr_pending,
                    NextValue(wr_adr, bus.adr),
                    NextValue(wr_sel, bus.sel),
                ),
                NextValue(wr_next,  bus.adr + 1),
                NextValue(wr_count, wr_count + 1),
            ).Elif(access & ~bus.we,
                NextValue(op,    MMAP_OP_READ),
                NextValue(adr,   bus.adr),
                NextValue(sel,   bus.sel),
                NextValue(count, 1),
                NextState("SEND-CMD")
            )
        )
        self.comb += [
            source.port.eq(0),
            source.length.eq(Mux(op == MMAP_OP_WRITE, 4*(2 + count), 4*2)),
        ]
        fsm.act("SEND-CMD",
            source.valid.eq(1),
            source.data[0].eq(op),
            source.data[4:8].eq(sel),
            source.data[8:16].eq(count - 1),
            source.data[16:32].eq(MMAP_MAGIC),
            If(source.ready,
                NextState("SEND-ADDR")
            )
        )
        fsm.act("SEND-ADDR",
            source.valid.eq(1),
            source.last.eq(op == MMAP_OP_READ),
            source.data.eq(adr),
            If(source.ready,
                NextValue(index, 0),
                If(op == MMAP_OP_READ,
                    NextState("RECEIVE-STATUS")
                ).Else(
                    NextState("SEND-DATA")
                )
            )
        )
        fsm.act("SEND-DATA",
            source.valid.eq(1),
            source.last.eq(index == (count - 1)),
            source.data.eq(wr_fifo.source.data),
            If(source.ready,
                wr_fifo.source.ready.eq(1),
                NextValue(index, index + 1),
                If(source.last,
                    NextValue(wr_count, 0),
                    NextState("IDLE")
                )
            )
        )
        fsm.act("RECEIVE-STATUS",
            sink.ready.eq(1),
            If(sink.valid,
                # Truncated response (no data) or invalid magic: error.
                NextValue(err, sink.data[0] | sink.last | (sink.data[16:32] != MMAP_MAGIC)),
                If(sink.last,
                    NextState("ACK")
                ).Else(
                    NextState("RECEIVE-DATA")
                )
            )
        )
        fsm.act("RECEIVE-DATA",
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(dat_r, sink.data),
                If(sink.last,
                    NextState("ACK")
                )
            )
        )
        fsm.act("ACK",
            bus.dat_r.eq(dat_r),
            bus.ack.eq(~err),
            bus.err.eq(err),
            NextState("IDLE")
        )

# MMAP Master --------------------------------------------------------------------------------------

class SERWBMMAPMaster(LiteXModule):
    """Wishbone Master (remote side): executes requests, sends read responses.

    Read requests larger than ``burst_depth`` (read response buffer) and requests with an invalid
    magic are dropped (``drop`` pulse).
    """
    def __init__(self, burst_depth=4):
        assert 1 <= burst_depth <= 256
        self.bus    = bus    = wishbone.Interface(data_width=32, adr_width=30)
        self.sink   = sink   = stream.Endpoint(packet_description(32))
        self.source = source = stream.Endpoint(packet_description(32))
        self.drop   = Signal()

        # # #

        # Request.
        # --------
        op    = Signal()
        adr   = Signal(30)
        sel   = Signal(4)
        count = Signal(9)
        index = Signal(9)
        err   = Signal()

        # Read Response Buffer.
        # ---------------------
        self.rd_fifo = rd_fifo = stream.SyncFIFO([("data", 32)], burst_depth)

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="RECEIVE-CMD")
        fsm.act("RECEIVE-CMD",
            sink.ready.eq(1),
            If(sink.valid,
                If(sink.data[16:32] != MMAP_MAGIC,
                    self.drop.eq(1),
                    If(~sink.last,
                        NextState("DRAIN") # Invalid magic (other protocol).
                    )
                ).Elif(~sink.last,
                    NextValue(op,    sink.data[0]),
                    NextValue(sel,   sink.data[4:8]),
                    NextValue(count, sink.data[8:16] + 1),
                    NextValue(index, 0),
                    NextValue(err,   0),
                    NextState("RECEIVE-ADDR")
                )
            )
        )
        fsm.act("RECEIVE-ADDR",
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(adr, sink.data),
                If(op == MMAP_OP_READ,
                    If(~sink.last,
                        NextState("DRAIN") # Malformed request.
                    ).Elif(count <= burst_depth,
                        NextState("READ")
                    ).Else(
                        NextState("RECEIVE-CMD") # Unsupported request.
                    )
                ).Else(
                    If(~sink.last,
                        NextState("WRITE")
                    ).Else(
                        NextState("RECEIVE-CMD") # Malformed request.
                    )
                )
            )
        )
        fsm.act("WRITE",
            bus.stb.eq(sink.valid),
            bus.cyc.eq(sink.valid),
            bus.we.eq(1),
            bus.adr.eq(adr + index),
            bus.sel.eq(sel),
            bus.dat_w.eq(sink.data),
            If(bus.stb & (bus.ack | bus.err),
                sink.ready.eq(1),
                NextValue(index, index + 1),
                If(sink.last,
                    NextState("RECEIVE-CMD")
                ).Elif(index == (count - 1),
                    NextState("DRAIN") # Malformed request (extra words).
                )
            )
        )
        fsm.act("DRAIN",
            sink.ready.eq(1),
            If(sink.valid & sink.last,
                NextState("RECEIVE-CMD")
            )
        )
        fsm.act("READ",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.adr.eq(adr + index),
            bus.sel.eq(sel),
            If(bus.ack | bus.err,
                rd_fifo.sink.valid.eq(1),
                rd_fifo.sink.data.eq(bus.dat_r),
                NextValue(err, err | bus.err),
                NextValue(index, index + 1),
                If(index == (count - 1),
                    NextValue(index, 0),
                    NextState("SEND-STATUS")
                )
            )
        )
        self.comb += [
            source.port.eq(0),
            source.length.eq(4*(1 + count)),
        ]
        fsm.act("SEND-STATUS",
            source.valid.eq(1),
            source.data[0].eq(err),
            source.data[16:32].eq(MMAP_MAGIC),
            If(source.ready,
                NextState("SEND-DATA")
            )
        )
        fsm.act("SEND-DATA",
            source.valid.eq(1),
            source.last.eq(index == (count - 1)),
            source.data.eq(rd_fifo.source.data),
            If(source.ready,
                rd_fifo.source.ready.eq(1),
                NextValue(index, index + 1),
                If(source.last,
                    NextState("RECEIVE-CMD")
                )
            )
        )
