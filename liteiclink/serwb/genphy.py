#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.build.io import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect     import stream

from liteiclink.serwb.datapath import TXDatapath, RXDatapath, FlowControl
from liteiclink.serwb.common   import SerdesControl, SERWBPHYBase

# SerDes Clocking ----------------------------------------------------------------------------------

class _SerdesClocking(LiteXModule):
    def __init__(self, pads, mode="master"):
        self.refclk = Signal()

        # # #

        # In Master mode, generate the clock with 180° phase shift so that Slave can use this clock
        # to sample data.
        if mode == "master":
            self.specials += DDROutput(0, 1, self.refclk)
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialOutput(self.refclk, pads.clk_p, pads.clk_n)
            else:
                self.comb += pads.clk.eq(self.refclk)

        # In Slave mode, use the clock provided by Master.
        elif mode == "slave":
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialInput(pads.clk_p, pads.clk_n, self.refclk)
            else:
                self.comb += self.refclk.eq(pads.clk)
        else:
            raise ValueError

# SerDes TX ----------------------------------------------------------------------------------------

class _SerdesTX(LiteXModule):
    def __init__(self, pads):
        # Control.
        self.idle  = idle  = Signal()
        self.comma = comma = Signal()

        # Datapath.
        self.sink = sink = stream.Endpoint([("data", 32)])

        # # #

        # Datapath.
        # ---------
        self.datapath = datapath = TXDatapath(1)
        self.comb += [
            sink.connect(datapath.sink),
            datapath.source.ready.eq(1),
            datapath.idle.eq(idle),
            datapath.comma.eq(comma),
        ]

        # Output data (on rising edge of sys_clk).
        # ----------------------------------------
        self.data = data = Signal()
        self.sync += data.eq(datapath.source.data)
        if hasattr(pads, "tx_p"):
            self.specials += DifferentialOutput(data, pads.tx_p, pads.tx_n)
        else:
            self.comb += pads.tx.eq(data)

# SerDes RX ----------------------------------------------------------------------------------------

class _SerdesRX(LiteXModule):
    def __init__(self, pads):
        # Control.
        self.shift = shift = Signal(6)

        # Status.
        self.idle  = idle  = Signal()
        self.comma = comma = Signal()

        # Datapath.
        self.source = source = stream.Endpoint([("data", 32)])

        # # #

        # Input data (on rising edge of sys_clk).
        # ---------------------------------------
        self.data = data = Signal()
        if hasattr(pads, "rx_p"):
            _data = Signal()
            self.specials += DifferentialInput(pads.rx_p, pads.rx_n, _data)
            self.sync += data.eq(_data)
        else:
            self.sync += data.eq(pads.rx)

        # Datapath.
        # ---------
        self.datapath = datapath = RXDatapath(1)
        self.comb += [
            datapath.sink.valid.eq(1),
            datapath.sink.data.eq(data),
            datapath.shift_inc.eq(self.shift),
            datapath.source.connect(source),
            idle.eq(datapath.idle),
            comma.eq(datapath.comma),
        ]

# SerDes -------------------------------------------------------------------------------------------

@ResetInserter()
class _Serdes(LiteXModule):
    def __init__(self, pads, mode="master"):
        if hasattr(pads, "clk") or hasattr(pads, "clk_p"):
            self.clocking = _SerdesClocking(pads, mode)
        self.tx = _SerdesTX(pads)
        self.rx = _SerdesRX(pads)


# SerDes Initialization/Synchronisation ------------------------------------------------------------
#
# - 1) Master sends IDLE patterns (zeroes) to Slave to reset it.
# - 2) Master sends K28.5 commas to allow Slave to calibrate, Slave sends IDLE patterns.
# - 3) Slave sends K28.5 commas to allow Master to calibrate, Master sends K28.5 commas.
# - 4) Master stops sending K28.5 commas.
# - 5) Slave stops sending K28.5 commas.
# - 6) Physical link is ready.
# --------------------------------------------------------------------------------------------------

# SerDes Init --------------------------------------------------------------------------------------

@ResetInserter()
class _SerdesInit(LiteXModule):
    """Link initialization: word alignment (shift scan), see sequence above."""
    def __init__(self, serdes, timeout, mode="master"):
        assert mode in ["master", "slave"]
        self.ready = Signal()
        self.error = Signal()

        # # #

        # Shift.
        # ------
        self.shift = shift = Signal(max=40)

        # Pattern sent while aligning: Master sends commas (aligned by Slave first), Slave sends idle
        # until its alignment is done.
        calib_tx = {"master": serdes.tx.comma, "slave": serdes.tx.idle}[mode]

        # Timer.
        # ------
        self.timer = timer = WaitTimer(timeout)

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="IDLE")
        # Master: Reset Slave (IDLE), then send commas and wait for Slave to be aligned.
        if mode == "master":
            fsm.act("IDLE",
                NextValue(shift, 0),
                NextState("RESET-SLAVE"),
                serdes.tx.idle.eq(1)
            )
            fsm.act("RESET-SLAVE",
                timer.wait.eq(1),
                If(timer.done,
                    timer.wait.eq(0),
                    NextState("SEND-PATTERN")
                ),
                serdes.tx.idle.eq(1)
            )
            fsm.act("SEND-PATTERN",
                If(~serdes.rx.idle,
                    timer.wait.eq(1),
                    If(timer.done,
                        NextState("CHECK-PATTERN")
                    )
                ),
                serdes.tx.comma.eq(1)
            )
        # Slave: Align on Master commas.
        if mode == "slave":
            fsm.act("IDLE",
                NextValue(shift, 0),
                timer.wait.eq(1),
                If(timer.done,
                    timer.wait.eq(0),
                    NextState("WAIT-STABLE"),
                ),
                serdes.tx.idle.eq(1)
            )
        # Alignment.
        fsm.act("WAIT-STABLE",
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK-PATTERN")
            ),
            calib_tx.eq(1)
        )
        fsm.act("CHECK-PATTERN",
            If(serdes.rx.comma,
                timer.wait.eq(1),
                If(timer.done,
                    NextState({"master": "READY", "slave": "SEND-PATTERN"}[mode])
                )
            ).Else(
                NextState("INC-SHIFT")
            ),
            calib_tx.eq(1)
        )
        fsm.act("INC-SHIFT",
            NextState("WAIT-STABLE"),
            If(shift == (40 - 1),
                NextState("ERROR")
            ).Else(
                serdes.rx.shift.eq(1),
                NextValue(shift, shift + 1)
            ),
            calib_tx.eq(1)
        )
        # Slave: Send commas until Master is aligned (Master stops sending commas).
        if mode == "slave":
            fsm.act("SEND-PATTERN",
                timer.wait.eq(1),
                If(timer.done,
                    If(~serdes.rx.comma,
                        NextState("READY")
                    )
                ),
                serdes.tx.comma.eq(1)
            )
        fsm.act("READY",
            self.ready.eq(1)
        )
        fsm.act("ERROR",
            self.error.eq(1)
        )

# SERWB PHY ----------------------------------------------------------------------------------------

class SERWBPHY(SERWBPHYBase):
    def __init__(self, device, pads, mode="master", init_timeout=2**16, with_flow_control=False):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32)])
        assert mode in ["master", "slave"]

        # # #

        # SerDes.
        # -------
        self.serdes = _Serdes(pads, mode)

        # SerDes Init.
        # ------------
        self.init = _SerdesInit(self.serdes, init_timeout, mode=mode)

        # SerDes Control.
        # ---------------
        self.control = SerdesControl(self.serdes, self.init, mode, with_delay=False)

        # Data-Path.
        # ----------
        if with_flow_control:
            self.flow_control = FlowControl(self.serdes, ready=self.init.ready)
        self.add_datapath()
