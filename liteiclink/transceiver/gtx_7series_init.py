from math import ceil

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.misc import WaitTimer


class GTXInit(Module):
    def __init__(self, sys_clk_freq, rx):
        self.done = Signal()
        self.restart = Signal()

        # GTX signals
        self.plllock = Signal()
        self.pllreset = Signal()
        self.gtXxreset = Signal()
        self.gtXxpd = Signal()
        self.Xxresetdone = Signal()
        self.Xxdlysreset = Signal()
        self.Xxdlysresetdone = Signal()
        self.Xxphaligndone = Signal()
        self.Xxuserrdy = Signal()

        # # #

        # Double-latch transceiver asynch outputs
        plllock = Signal()
        Xxresetdone = Signal()
        Xxdlysresetdone = Signal()
        Xxphaligndone = Signal()
        self.specials += [
            MultiReg(self.plllock, plllock),
            MultiReg(self.Xxresetdone, Xxresetdone),
            MultiReg(self.Xxdlysresetdone, Xxdlysresetdone),
            MultiReg(self.Xxphaligndone, Xxphaligndone)
        ]

        # Deglitch FSM outputs driving transceiver asynch inputs
        gtXxreset = Signal()
        gtXxpd = Signal()
        Xxdlysreset = Signal()
        Xxuserrdy = Signal()
        self.sync += [
            self.gtXxreset.eq(gtXxreset),
            self.gtXxpd.eq(gtXxpd),
            self.Xxdlysreset.eq(Xxdlysreset),
            self.Xxuserrdy.eq(Xxuserrdy)
        ]

        # After configuration, transceiver resets have to stay low for
        # at least 500ns (see AR43482)
        startup_cycles = ceil(500*sys_clk_freq/1000000000)
        startup_timer = WaitTimer(startup_cycles)
        self.submodules += startup_timer

        startup_fsm = ResetInserter()(FSM(reset_state="GTP_PD"))
        self.submodules += startup_fsm

        ready_timer = WaitTimer(int(sys_clk_freq/1000))
        self.submodules += ready_timer
        self.comb += [
            ready_timer.wait.eq(~self.done & ~startup_fsm.reset),
            startup_fsm.reset.eq(self.restart | ready_timer.done)
        ]

        if rx:
            cdr_stable_timer = WaitTimer(1024)
            self.submodules += cdr_stable_timer

        Xxphaligndone_r = Signal(reset=1)
        Xxphaligndone_rising = Signal()
        self.sync += Xxphaligndone_r.eq(Xxphaligndone)
        self.comb += Xxphaligndone_rising.eq(Xxphaligndone & ~Xxphaligndone_r)

        startup_fsm.act("GTP_PD",
            gtXxreset.eq(1),
            gtXxpd.eq(1),
            self.pllreset.eq(1),
            startup_timer.wait.eq(1),
            NextState("RESET_ALL")
        )
        startup_fsm.act("RESET_ALL",
            gtXxreset.eq(1),
            self.pllreset.eq(1),
            startup_timer.wait.eq(1),
            NextState("RELEASE_PLL_RESET")
        )
        startup_fsm.act("RELEASE_PLL_RESET",
            gtXxreset.eq(1),
            startup_timer.wait.eq(1),
            If(plllock & startup_timer.done, NextState("RELEASE_GTX_RESET"))
        )
        # Release GTX reset and wait for GTX resetdone
        # (from UG476, GTX is reset on falling edge
        # of gtXxreset)
        if rx:
            startup_fsm.act("RELEASE_GTX_RESET",
                Xxuserrdy.eq(1),
                cdr_stable_timer.wait.eq(1),
                If(Xxresetdone & cdr_stable_timer.done, NextState("ALIGN"))
            )
        else:
            startup_fsm.act("RELEASE_GTX_RESET",
                Xxuserrdy.eq(1),
                If(Xxresetdone, NextState("ALIGN"))
            )
        # Start delay alignment (pulse)
        startup_fsm.act("ALIGN",
            Xxuserrdy.eq(1),
            Xxdlysreset.eq(1),
            NextState("WAIT_ALIGN")
        )
        # Wait for delay alignment
        startup_fsm.act("WAIT_ALIGN",
            Xxuserrdy.eq(1),
            If(Xxdlysresetdone,
                NextState("WAIT_FIRST_ALIGN_DONE")
            )
        )
        # Wait 2 rising edges of Xxphaligndone
        # (from UG476 in buffer bypass config)
        startup_fsm.act("WAIT_FIRST_ALIGN_DONE",
            Xxuserrdy.eq(1),
            If(Xxphaligndone_rising, NextState("WAIT_SECOND_ALIGN_DONE"))
        )
        startup_fsm.act("WAIT_SECOND_ALIGN_DONE",
            Xxuserrdy.eq(1),
            If(Xxphaligndone_rising, NextState("READY"))
        )
        startup_fsm.act("READY",
            Xxuserrdy.eq(1),
            self.done.eq(1),
            If(self.restart, NextState("GTP_PD"))
        )
