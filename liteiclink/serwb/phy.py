#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream

from liteiclink.serwb.kuserdes     import KUSerdes
from liteiclink.serwb.s7serdes     import S7Serdes
from liteiclink.serwb.efinixserdes import EfinixSerdes
from liteiclink.serwb.datapath     import FlowControl
from liteiclink.serwb.common       import SerdesControl, SERWBPHYBase

# SerDes Initialization/Synchronisation ------------------------------------------------------------
#
# - 1) Master sends IDLE patterns (zeroes) to Slave to reset it.
# - 2) Master sends K28.5 commas to allow Slave to calibrate, Slave sends IDLE patterns.
# - 3) Slave sends K28.5 commas to allow Master to calibrate, Master sends K28.5 commas.
# - 4) Master stops sending K28.5 commas.
# - 5) Slave stops sending K28.5 commas.
# - 6) Physical link is ready.
# --------------------------------------------------------------------------------------------------

# Serdes Init --------------------------------------------------------------------------------------

@ResetInserter()
class _SerdesInit(LiteXModule):
    """Link initialization: sampling window calibration (delay scan) and word alignment (shift scan)
    for each phase (Efinix clk_ratio), see sequence above."""
    def __init__(self, serdes, taps, timeout, clk_ratio="1:1", mode="master"):
        assert mode in ["master", "slave"]
        self.ready = Signal()
        self.error = Signal()

        # # #

        self.delay           = delay           = Signal(max=taps)
        self.delay_min       = delay_min       = Signal(max=taps)
        self.delay_min_found = delay_min_found = Signal()
        self.delay_max       = delay_max       = Signal(max=taps)
        self.delay_max_found = delay_max_found = Signal()
        self.shift           = shift           = Signal(max=40)
        self.phase_sel       = phase_sel       = Signal(2)

        # Pattern sent while calibrating: Master sends commas (calibrated by Slave first), Slave
        # sends idle until its calibration is done.
        calib_tx = {"master": serdes.tx.comma, "slave": serdes.tx.idle}[mode]

        # Timer.
        # ------
        self.timer = timer = WaitTimer(timeout)

        # FSM.
        # ----
        self.fsm = fsm = FSM(reset_state="RESET")
        fsm.act("RESET",
            NextValue(phase_sel, 0),
            NextState("IDLE")
        )
        reset_calibration = [
            NextValue(delay,           0),
            NextValue(delay_min,       0),
            NextValue(delay_min_found, 0),
            NextValue(delay_max,       0),
            NextValue(delay_max_found, 0),
            NextValue(shift,           0),
            serdes.rx.delay_rst.eq(1),
        ]
        # Master: Reset Slave (IDLE), then send commas and wait for Slave to be calibrated.
        if mode == "master":
            fsm.act("IDLE",
                *reset_calibration,
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
        # Slave: Calibrate on Master commas.
        if mode == "slave":
            fsm.act("IDLE",
                *reset_calibration,
                timer.wait.eq(1),
                If(timer.done,
                    timer.wait.eq(0),
                    NextState("WAIT-STABLE"),
                ),
                serdes.tx.idle.eq(1)
            )
        # Calibration.
        fsm.act("WAIT-STABLE",
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK-PATTERN")
            ),
            calib_tx.eq(1)
        )
        fsm.act("CHECK-PATTERN",
            If(~delay_min_found,
                If(serdes.rx.comma,
                    timer.wait.eq(1),
                    If(timer.done,
                        timer.wait.eq(0),
                        NextValue(delay_min, delay),
                        NextValue(delay_min_found, 1),
                    )
                ).Else(
                    NextState("INC-DELAY-SHIFT")
                ),
            ).Else(
                If(~serdes.rx.comma | (delay == (taps - 1)),
                    NextValue(delay_max, delay),
                    NextValue(delay_max_found, 1),
                    NextState("CHECK-SAMPLING-WINDOW")
                ).Else(
                    NextState("INC-DELAY-SHIFT")
                )
            ),
            calib_tx.eq(1)
        )
        fsm.act("INC-DELAY-SHIFT",
            NextState("WAIT-STABLE"),
            If(delay == (taps - 1),
                If(shift == (40 - 1),
                    NextState("INC-PHASE-SEL")
                ).Else(
                    NextValue(delay_min_found, 0),
                    NextValue(delay_min,       0),
                    NextValue(delay_max_found, 0),
                    NextValue(delay_max,       0),
                    NextValue(shift, shift + 1),
                    serdes.rx.shift_inc.eq(1)
                ),
                NextValue(delay, 0),
                serdes.rx.delay_rst.eq(1)
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx.delay_inc.eq(1)
            ),
            calib_tx.eq(1)
        )
        if hasattr(serdes.rx, "phase_sel"):
            self.comb += serdes.rx.phase_sel.eq(phase_sel)
        fsm.act("INC-PHASE-SEL",
            NextValue(phase_sel, phase_sel + 1),
            If(phase_sel == ({"1:1":1, "1:2":2, "1:4":4}[clk_ratio] - 1),
                NextState("ERROR")
            ).Else(
                NextState("IDLE")
            )
        )
        fsm.act("CHECK-SAMPLING-WINDOW",
            If((delay_max - delay_min) < taps//16,
               NextValue(delay_min_found, 0),
               NextValue(delay_max_found, 0),
               NextState("WAIT-STABLE")
            ).Else(
                NextValue(delay, 0),
                serdes.rx.delay_rst.eq(1),
                NextState("CONFIGURE-SAMPLING-WINDOW")
            ),
            calib_tx.eq(1)
        )
        fsm.act("CONFIGURE-SAMPLING-WINDOW",
            If(delay == (delay_min + (delay_max - delay_min)[1:]),
                NextState({"master": "READY", "slave": "SEND-PATTERN"}[mode])
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx.delay_inc.eq(1)
            ),
            calib_tx.eq(1)
        )
        # Slave: Send commas until Master is calibrated (Master stops sending commas).
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
    def __init__(self, device, pads, mode="master", init_timeout=2**16, clk="sys", clk4x="sys4x", clk_ratio="1:1", clk_delay_taps=0, rx_delay_taps=0, usp=False, with_flow_control=False):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32)])
        assert mode in ["master", "slave"]

        # # #

        # SerDes.
        # -------

        # Xilinx Ultrascale(+).
        if device[:4] in ["xcku", "xvu", "xczu"]:
            assert clk_ratio == "1:1"
            taps = 512
            self.serdes = KUSerdes(pads, mode, usp=usp)

        # Xilinx 7-Series.
        elif device[:4] in ["xc7a", "xc7k", "xc7v", "xc7z", "xc7s"]:
            assert clk_ratio == "1:1"
            taps = 32
            self.serdes = S7Serdes(pads, mode)

        # Efinix Titanium.
        elif device[:2] == "Ti":
            taps = 64
            self.serdes = EfinixSerdes(pads, mode,
                clk       = clk,
                clk4x     = clk4x,
                clk_ratio = clk_ratio,
            )

        # Efinix Trion.
        elif device[:2] in ["T1", "T2"]:
            taps = 4 # No dynamic delay
            self.serdes = EfinixSerdes(pads, mode,
                clk            = clk,
                clk4x          = clk4x,
                clk_ratio      = clk_ratio,
                clk_delay_taps = clk_delay_taps,
                rx_delay_taps  = rx_delay_taps
            )
        else:
            raise NotImplementedError

        # SerDes Init.
        # ------------
        self.init = _SerdesInit(self.serdes, taps, init_timeout, clk_ratio, mode=mode)

        # SerDes Control.
        # ---------------
        self.control = SerdesControl(self.serdes, self.init, mode)

        # Data-Path.
        # ----------
        if with_flow_control:
            self.flow_control = FlowControl(self.serdes, ready=self.init.ready)
        self.add_datapath()
