#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import Arbiter, Dispatcher

from liteiclink.serwb.packet import packet_description
from liteiclink.serwb.packet import Packetizer, Depacketizer
from liteiclink.serwb.etherbone import Etherbone

# SERWB Core ---------------------------------------------------------------------------------------

class SERWBCore(LiteXModule):
    def __init__(self, phy, clk_freq, mode,
        etherbone_buffer_depth = 4,
        tx_buffer_depth        = 8,
        rx_buffer_depth        = 8,
    ):

        # Etherbone.
        # ----------
        self.etherbone = etherbone = Etherbone(mode, etherbone_buffer_depth)

        # Bus.
        # ----
        self.bus = etherbone.wishbone.bus

        # Packetizer / Depacketizer.
        # --------------------------
        self.packetizer   = packetizer   = Packetizer()
        self.depacketizer = depacketizer = Depacketizer(clk_freq)

        # Buffering.
        # ----------
        tx_fifo = stream.SyncFIFO([("data", 32)], tx_buffer_depth, buffered=True)
        rx_fifo = stream.SyncFIFO([("data", 32)], rx_buffer_depth, buffered=True)
        self.submodules += tx_fifo, rx_fifo

        # Data-Path.
        # ----------
        self.comb += [
            # Phy <--> Core
            packetizer.source.connect(tx_fifo.sink),
            tx_fifo.source.connect(phy.sink),

            phy.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(depacketizer.sink),
        ]

        # Downstream/Upstream Endpoints.
        # ------------------------------
        self.downstream_endpoints = {0: etherbone.source}
        self.upstream_endpoints   = {0: etherbone.sink  }

    def do_finalize(self):
        # Downstream Arbitration.
        # -----------------------
        downstream_endpoints = [stream.Endpoint(packet_description(32)) for _ in range(len(self.downstream_endpoints))]
        for i, (k, v) in enumerate(self.downstream_endpoints.items()):
            self.comb += [
                v.connect(downstream_endpoints[i], keep={"valid", "ready", "last", "data", "length"}),
                downstream_endpoints[i].port.eq(k),
            ]
        self.arbiter = Arbiter(
            masters = downstream_endpoints,
            slave   = self.packetizer.sink,
            keep    = {"valid", "ready", "last", "data", "port", "length"},
        )

        # Upstream Dispatching.
        # ---------------------
        self.dispatcher = Dispatcher(
            master  = self.depacketizer.source,
            slaves  = [ep for _, ep in self.upstream_endpoints.items()],
            one_hot = False,
            keep    = {"valid", "ready", "last", "data", "length"},
        )
        for i, (k, v) in enumerate(self.upstream_endpoints.items()):
            self.comb += If(self.depacketizer.source.port == k, self.dispatcher.sel.eq(i))

# SERIO Core ---------------------------------------------------------------------------------------

class SERIOPacketizer(LiteXModule):
    def __init__(self):
        self.inputs = Signal(32)
        self.source = source = stream.Endpoint(packet_description(32))

        # # #

        # Signals.
        # --------
        inputs        = Signal(32)
        inputs_d      = Signal(32)

        # Re-Synchronize Inputs.
        # ----------------------
        self.specials += MultiReg(self.inputs, inputs)

        # Register Inputs.
        # ----------------
        self.sync += If(source.ready, inputs_d.eq(inputs))

        # Generate Packet.
        # ----------------
        self.comb += [
            source.valid.eq(inputs != inputs_d),
            source.last.eq(1),
            source.data.eq(inputs),
            source.length.eq(4),
        ]

class SERIODepacketizer(LiteXModule):
    def __init__(self):
        self.sink    = sink = stream.Endpoint(packet_description(32))
        self.outputs = Signal(32)

        # # #

        # Generate Outputs.
        # -----------------
        self.comb += sink.ready.eq(1)
        self.sync += If(sink.valid & sink.last, self.outputs.eq(sink.data))

class SERIOCore(LiteXModule):
    def __init__(self, serwb_core):
        self.inputs  = Signal(32)
        self.outputs = Signal(32)

        # # #

        # Packetizer.
        # -----------
        self.packetizer = SERIOPacketizer()
        self.comb += self.packetizer.inputs.eq(self.inputs)

        # Depacketizer.
        # -------------
        self.depacketizer = SERIODepacketizer()
        self.comb += self.outputs.eq(self.depacketizer.outputs)

        # Add to SERWB Downstreams/Upstreams Endpoints.
        # ---------------------------------------------
        serwb_core.downstream_endpoints.update({1: self.packetizer.source})
        serwb_core.upstream_endpoints.update(  {1: self.depacketizer.sink})
