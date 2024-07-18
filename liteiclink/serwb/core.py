#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect        import stream
from litex.soc.interconnect.packet import Arbiter, Dispatcher

from liteiclink.serwb.packet    import packet_description
from liteiclink.serwb.packet    import Packetizer, Depacketizer
from liteiclink.serwb.etherbone import Etherbone

# SERWB Core ---------------------------------------------------------------------------------------

class SERWBCore(LiteXModule):
    def __init__(self, phy, clk_freq, mode, with_rst_on_link_down=True, port=0,
        etherbone_buffer_depth = 4,
        tx_buffer_depth        = 8,
        rx_buffer_depth        = 8,
    ):
        # Downstream/Upstream Endpoints.
        # ------------------------------
        self.downstream_endpoints = {}
        self.upstream_endpoints   = {}

        # Etherbone.
        # ----------
        self.etherbone = etherbone = ResetInserter()(Etherbone(mode, etherbone_buffer_depth))
        self.add_downstream_endpoint(port=port, endpoint=etherbone.source)
        self.add_upstream_endpoint(  port=port, endpoint=etherbone.sink)

        # Bus.
        # ----
        self.bus = etherbone.wishbone.bus

        # Packetizer / Depacketizer.
        # --------------------------
        self.packetizer   = packetizer   = ResetInserter()(Packetizer())
        self.depacketizer = depacketizer = ResetInserter()(Depacketizer(clk_freq))

        # Buffering.
        # ----------
        self.tx_fifo = tx_fifo = ResetInserter()(stream.SyncFIFO([("data", 32)], tx_buffer_depth, buffered=True))
        self.rx_fifo = rx_fifo = ResetInserter()(stream.SyncFIFO([("data", 32)], rx_buffer_depth, buffered=True))

        # Data-Path.
        # ----------
        self.comb += [
            # Core -> PHY.
            packetizer.source.connect(tx_fifo.sink),
            tx_fifo.source.connect(phy.sink),

            # PHY -> Core.
            phy.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(depacketizer.sink),
        ]

        # Reset internal module when link down.
        # -------------------------------------
        if with_rst_on_link_down:
            self.comb += [
                etherbone.reset.eq(    ~phy.init.ready),
                packetizer.reset.eq(   ~phy.init.ready),
                depacketizer.reset.eq( ~phy.init.ready),
                tx_fifo.reset.eq(      ~phy.init.ready),
                rx_fifo.reset.eq(      ~phy.init.ready),
            ]

    def add_downstream_endpoint(self, port, endpoint):
        if port in self.downstream_endpoints.keys():
            raise ValueError(f"Downstream endpoint for port {port} already exists.")
        self.downstream_endpoints[port] = endpoint


    def add_upstream_endpoint(self, port, endpoint):
        if port in self.upstream_endpoints.keys():
            raise ValueError(f"Upstream endpoint for port {port} already exists.")
        self.upstream_endpoints[port] = endpoint

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
        self.i      = Signal(32)
        self.source = source = stream.Endpoint(packet_description(32))

        # # #

        # Signals.
        # --------
        i   = Signal(32)
        i_d = Signal(32)

        # Re-Synchronize Inputs.
        # ----------------------
        self.specials += MultiReg(self.i, i)

        # Register Inputs.
        # ----------------
        self.sync += If(source.ready, i_d.eq(i))

        # Generate Packet.
        # ----------------
        self.comb += [
            source.valid.eq(i != i_d),
            source.last.eq(1),
            source.data.eq(i),
            source.length.eq(4),
        ]

class SERIODepacketizer(LiteXModule):
    def __init__(self):
        self.sink = sink = stream.Endpoint(packet_description(32))
        self.o    = Signal(32)

        # # #

        # Generate Outputs.
        # -----------------
        self.comb += sink.ready.eq(1)
        self.sync += If(sink.valid & sink.last, self.o.eq(sink.data))

class SERIOCore(LiteXModule):
    def __init__(self, serwb_core, port=1):
        self.i = Signal(32)
        self.o = Signal(32)

        # # #

        # Packetizer.
        # -----------
        self.packetizer = SERIOPacketizer()
        self.comb += self.packetizer.i.eq(self.i)

        # Depacketizer.
        # -------------
        self.depacketizer = SERIODepacketizer()
        self.comb += self.o.eq(self.depacketizer.o)

        # Add to SERWB Downstreams/Upstreams Endpoints.
        # ---------------------------------------------
        serwb_core.add_downstream_endpoint(port=port, endpoint=self.packetizer.source)
        serwb_core.add_upstream_endpoint  (port=port, endpoint=self.depacketizer.sink)
