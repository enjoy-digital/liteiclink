#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect     import stream
from litex.soc.interconnect.csr import *

from litex.soc.cores.prbs import PRBS31Generator, PRBS31Checker

# SerDes Init Control ------------------------------------------------------------------------------

class SerdesControl(LiteXModule):
    """SerDes Init Control/Status CSRs (shared by all SerWB PHYs).

    ``with_delay`` exposes the sampling window calibration results (PHYs with dynamic delays).
    ``with_prbs`` adds real PRBS31 TX/RX test CSRs (prbs_tx/prbs_rx, see SERWBPHYBase).
    ``with_auto_retrain`` (Master): restarts link initialization when ``retrain_threshold`` invalid
    8b10b words are received within ``retrain_window`` cycles while the link is ready.
    """
    def __init__(self, serdes, init, mode="master", with_delay=True, with_prbs=False,
        with_auto_retrain = False,
        retrain_threshold = 16,
        retrain_window    = 2**16,
    ):
        # Control/Status.
        if mode == "master":
            self.reset = CSR()
        self.ready = CSRStatus()
        self.error = CSRStatus()

        # Delay/Shift/Phase.
        if with_delay:
            self.delay           = CSRStatus(9)
            self.delay_min_found = CSRStatus()
            self.delay_min       = CSRStatus(9)
            self.delay_max_found = CSRStatus()
            self.delay_max       = CSRStatus(9)
        self.shift = CSRStatus(6)
        if with_delay:
            self.phase = CSRStatus(2)

        # PRBS.
        self.prbs_error  = Signal()
        self.prbs_start  = CSR()
        self.prbs_cycles = CSRStorage(32)
        self.prbs_errors = CSRStatus(32)

        self.invalid_errors = CSRStatus(32, description="Number of invalid 8b10b words received (link ready).")

        # Link Health.
        self.link_drops = CSRStatus(32, description="Number of link losses (ready to not ready).")
        if with_prbs:
            self.prbs_tx = CSRStorage(description="Transmit PRBS31 instead of traffic (link ready).")
            self.prbs_rx = CSRStorage(description="Check received words against PRBS31 (traffic not forwarded).")

        # # #

        # Link Drops Counter.
        # -------------------
        ready_d    = Signal()
        link_drops = self.link_drops.status
        self.sync += [
            ready_d.eq(init.ready),
            If(ready_d & ~init.ready & (link_drops != (2**32 - 1)),
                link_drops.eq(link_drops + 1)
            )
        ]

        # Auto Retrain (Master).
        # ----------------------
        retrain = Signal()
        if with_auto_retrain:
            assert mode == "master"
            self.retrain_timer = retrain_timer = WaitTimer(retrain_window)
            retrain_count = Signal(max=retrain_threshold + 1)
            self.comb += [
                retrain_timer.wait.eq(init.ready & ~retrain_timer.done),
                retrain.eq(init.ready & (retrain_count == retrain_threshold)),
            ]
            self.sync += [
                If(~init.ready | retrain_timer.done,
                    retrain_count.eq(0)
                ).Elif(serdes.rx.datapath.invalid & (retrain_count != retrain_threshold),
                    retrain_count.eq(retrain_count + 1)
                )
            ]

        # Invalid 8b10b Words Counter (cleared on link reset).
        # ----------------------------------------------------
        invalid_errors = self.invalid_errors.status
        self.sync += [
            If(~init.ready,
                invalid_errors.eq(0)
            ).Elif(serdes.rx.datapath.invalid & (invalid_errors != (2**32 - 1)),
                invalid_errors.eq(invalid_errors + 1)
            )
        ]

        # Master Mode.
        # ------------
        if mode == "master":
            # In Master mode, reset is coming from CSR, it resets the Master that will also reset
            # the Slave by putting the link in IDLE state.
            self.sync += init.reset.eq(self.reset.wr_stb | retrain)

        # Slave Mode.
        # -----------
        if mode == "slave":
            # In Slave mode, reset is coming from link, Master reset the Slave by putting the link
            # in IDLE state.
            self.sync += [
                init.reset.eq(serdes.rx.idle),
                serdes.reset.eq(serdes.rx.idle),
            ]

        # Control/Status.
        # ---------------
        self.comb += [
            self.ready.status.eq(init.ready),
            self.error.status.eq(init.error),
        ]
        if with_delay:
            self.comb += [
                self.delay.status.eq(init.delay),
                self.delay_min_found.status.eq(init.delay_min_found),
                self.delay_min.status.eq(init.delay_min),
                self.delay_max_found.status.eq(init.delay_max_found),
                self.delay_max.status.eq(init.delay_max),
            ]
        self.comb += self.shift.status.eq(init.shift)
        if with_delay:
            self.comb += self.phase.status.eq(init.phase_sel)

        # PRBS.
        # -----
        prbs_cycles = Signal(32)
        prbs_errors = self.prbs_errors.status
        self.prbs_fsm = prbs_fsm = FSM(reset_state="IDLE")
        prbs_fsm.act("IDLE",
            NextValue(prbs_cycles, 0),
            If(self.prbs_start.wr_stb,
                NextValue(prbs_errors, 0),
                NextState("CHECK")
            )
        )
        prbs_fsm.act("CHECK",
            NextValue(prbs_cycles, prbs_cycles + 1),
            If(self.prbs_error,
                NextValue(prbs_errors, prbs_errors + 1),
            ),
            If(prbs_cycles == self.prbs_cycles.storage,
                NextState("IDLE")
            )
        )

# SERWB PHY Base -----------------------------------------------------------------------------------

class SERWBPHYBase(LiteXModule):
    """Common SERWBPHY logic (shared by all SerWB PHYs)."""
    def add_datapath(self, with_prbs=False):
        """Connect sink/source to the SerDes, through FlowControl when enabled (`self.flow_control`
        created by the PHY: XON/XOFF and K28.1 fill characters, must be enabled on both sides of the
        link, the core drives `pause` from its RX buffer level).

        With ``with_prbs``, PRBS31 words can be sent (`control.prbs_tx`) instead of traffic and
        received words checked (`control.prbs_rx`, not forwarded to the core) for link tests.
        """
        serdes, init, sink, source = self.serdes, self.init, self.sink, self.source

        # PRBS31 TX/RX (between the core and the SerDes/FlowControl).
        # -------------------------------------------------------------
        if with_prbs:
            control = self.control
            self.prbs_generator = prbs_generator = CEInserter()(PRBS31Generator(32))
            self.prbs_checker   = prbs_checker   = CEInserter()(PRBS31Checker(32))
            _sink   = stream.Endpoint([("data", 32)])
            _source = stream.Endpoint([("data", 32)])
            self.comb += [
                If(control.prbs_tx.storage,
                    _sink.valid.eq(1),
                    _sink.data.eq(prbs_generator.o),
                    prbs_generator.ce.eq(_sink.ready),
                ).Else(
                    sink.connect(_sink),
                ),
                If(control.prbs_rx.storage,
                    _source.ready.eq(1),
                    prbs_checker.i.eq(_source.data),
                    prbs_checker.ce.eq(_source.valid),
                ).Else(
                    _source.connect(source),
                ),
            ]
            # Checker errors are registered: count one error per wrong word.
            prbs_checked = Signal()
            self.sync += prbs_checked.eq(prbs_checker.ce)
            prbs_error = Signal()
            self.comb += prbs_error.eq(control.prbs_rx.storage & prbs_checked & (prbs_checker.errors != 0))
            sink, source = _sink, _source
        flow_control = getattr(self, "flow_control", None)
        if flow_control is not None:
            self.pause = flow_control.pause
            self.comb += [
                sink.connect(flow_control.sink),
                flow_control.source.connect(source),
            ]
        else:
            self.comb += [
                If(init.ready,
                    If(sink.valid,
                        sink.connect(serdes.tx.sink),
                    ),
                    serdes.rx.source.connect(source)
                ).Else(
                    serdes.rx.source.ready.eq(1)
                ),
                serdes.tx.sink.valid.eq(1) # Always transmitting.
            ]

        # PRBS.
        # -----
        # Without prbs_rx, the PRBS test checks that idle words (sent as zeroes without traffic and
        # flow control) are received as zeroes.
        zero_error = Signal()
        self.comb += zero_error.eq(self.source.valid & self.source.ready & (self.source.data != 0))
        if with_prbs:
            self.comb += self.control.prbs_error.eq(Mux(self.control.prbs_rx.storage, prbs_error, zero_error))
        else:
            self.comb += self.control.prbs_error.eq(zero_error)
