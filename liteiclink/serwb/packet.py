#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import HeaderField, Header

from liteeth.common import eth_udp_user_description

# Layouts ------------------------------------------------------------------------------------------

def phy_description(dw):
    layout = [("data", dw)]
    return stream.EndpointDescription(layout)

# Packetizer ---------------------------------------------------------------------------------------

class Packetizer(LiteXModule):
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(eth_udp_user_description(32))
        self.source = source = stream.Endpoint(phy_description(32))

        # # #

        # Packet description
        #   - Preamble : 4 bytes
        #   - Length   : 4 bytes
        #   - Payload

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="PREAMBLE")
        fsm.act("PREAMBLE",
            If(sink.valid,
                source.valid.eq(1),
                source.data.eq(0x5aa55aa5),
                If(source.ready,
                    NextState("LENGTH")
                )
            )
        )
        fsm.act("LENGTH",
            source.valid.eq(1),
            source.data.eq(sink.length),
            If(source.ready,
                NextState("DATA")
            )
        )
        fsm.act("DATA",
            source.valid.eq(sink.valid),
            source.data.eq(sink.data),
            sink.ready.eq(source.ready),
            If(source.ready & sink.last,
                NextState("PREAMBLE")
            )
        )

# Depacketizer -------------------------------------------------------------------------------------

class Depacketizer(LiteXModule):
    def __init__(self, clk_freq, timeout=10):
        self.sink   = sink   = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(eth_udp_user_description(32))

        # # #

        # Packet description
        #   - Preamble : 4 bytes
        #   - Length   : 4 bytes
        #   - Payload

        # Signals.
        # --------
        count  = Signal(len(source.length))
        length = Signal(len(source.length))

        # Timer.
        # ------
        self.timer = timer = WaitTimer(clk_freq*timeout)

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="PREAMBLE")
        fsm.act("PREAMBLE",
            sink.ready.eq(1),
            If(sink.valid &
              (sink.data == 0x5aa55aa5),
                NextState("LENGTH")
            )
        )
        fsm.act("LENGTH",
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(count, 0),
                NextValue(length, sink.data),
                NextState("DATA")
            ),
            timer.wait.eq(1)
        )
        fsm.act("DATA",
            source.valid.eq(sink.valid),
            source.last.eq(count == (length[2:] - 1)),
            source.length.eq(length),
            source.data.eq(sink.data),
            sink.ready.eq(source.ready),
            If(timer.done,
                NextState("PREAMBLE")
            ).Elif(source.valid & source.ready,
                NextValue(count, count + 1),
                If(source.last,
                    NextState("PREAMBLE")
                )
            ),
            timer.wait.eq(1)
        )
