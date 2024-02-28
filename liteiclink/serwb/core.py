#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

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
        downstream_endpoints = [stream.Endpoint(packet_description(32))] * len(self.downstream_endpoints)
        for i, (k, v) in enumerate(self.downstream_endpoints.items()):
            self.comb += [
                v.connect(downstream_endpoints[i], keep={"valid", "ready", "last", "data", "length"}),
                downstream_endpoints[i].port.eq(k),
            ]
        self.arbiter = Arbiter(
            masters = downstream_endpoints,
            slave   = self.packetizer.sink,
            keep    = {"valid", "ready", "last", "data", "length"},
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
