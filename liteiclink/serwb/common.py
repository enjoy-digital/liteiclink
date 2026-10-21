#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# SerDes Init Control ------------------------------------------------------------------------------

class SerdesControl(LiteXModule):
    """SerDes Init Control/Status CSRs (shared by all SerWB PHYs).

    ``with_delay`` exposes the sampling window calibration results (PHYs with dynamic delays).
    """
    def __init__(self, serdes, init, mode="master", with_delay=True):
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

        # # #

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
            self.sync += init.reset.eq(self.reset.wr_stb)

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
    def add_datapath(self):
        """Connect sink/source to the SerDes, through FlowControl when enabled (`self.flow_control`
        created by the PHY: XON/XOFF and K28.1 fill characters, must be enabled on both sides of the
        link, the core drives `pause` from its RX buffer level)."""
        serdes, init, sink, source = self.serdes, self.init, self.sink, self.source
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
        # The PRBS test checks that idle words (sent as zeroes without traffic) are received as
        # zeroes.
        self.comb += self.control.prbs_error.eq(source.valid & source.ready & (source.data != 0))
