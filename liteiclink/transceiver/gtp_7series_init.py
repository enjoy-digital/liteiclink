from math import ceil

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.misc import WaitTimer

# TODO:
# code is working but it seems we need to follow the obscure rx init sequence
# described in https://www.xilinx.com/support/answers/53561.html to remove
# our work arounds...

class GTPInit(Module):
    def __init__(self, sys_clk_freq, rx):
        self.done = Signal()
        self.restart = Signal()

        self.debug = Signal(8)

        # GTP signals
        self.plllock = Signal()
        self.pllreset = Signal()
        self.gtXxreset = Signal()
        self.Xxresetdone = Signal()
        self.Xxdlysreset = Signal()
        self.Xxdlysresetdone = Signal()
        self.Xxphinit = Signal()
        self.Xxphinitdone = Signal()
        self.Xxphalign = Signal()
        self.Xxphaligndone = Signal()
        self.Xxdlyen = Signal()
        self.Xxuserrdy = Signal()
        self.Xxsyncdone = Signal()

        # # #

        # Double-latch transceiver asynch outputs
        plllock = Signal()
        Xxresetdone = Signal()
        Xxdlysresetdone = Signal()
        Xxphinitdone = Signal()
        Xxphaligndone = Signal()
        Xxsyncdone = Signal()
        self.specials += [
            MultiReg(self.plllock, plllock),
            MultiReg(self.Xxresetdone, Xxresetdone),
            MultiReg(self.Xxdlysresetdone, Xxdlysresetdone),
            MultiReg(self.Xxphinitdone, Xxphinitdone),
            MultiReg(self.Xxphaligndone, Xxphaligndone),
            MultiReg(self.Xxsyncdone, Xxsyncdone)
        ]

        # Deglitch FSM outputs driving transceiver asynch inputs
        gtXxreset = Signal()
        Xxdlysreset = Signal()
        Xxphinit = Signal()
        Xxphalign = Signal()
        Xxdlyen = Signal()
        Xxuserrdy = Signal()
        self.sync += [
            self.gtXxreset.eq(gtXxreset),
            self.Xxdlysreset.eq(Xxdlysreset),
            self.Xxphinit.eq(Xxphinit),
            self.Xxphalign.eq(Xxphalign),
            self.Xxdlyen.eq(Xxdlyen),
            self.Xxuserrdy.eq(Xxuserrdy)
        ]

        # After configuration, transceiver resets have to stay low for
        # at least 500ns (see AR43482)
        startup_cycles = ceil(500*sys_clk_freq/1000000000)
        startup_timer = WaitTimer(startup_cycles)
        self.submodules += startup_timer

        startup_fsm = ResetInserter()(FSM(reset_state="RESET_ALL"))
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

        startup_fsm.act("RESET_ALL",
            self.debug.eq(0),
            gtXxreset.eq(1),
            self.pllreset.eq(1),
            startup_timer.wait.eq(1),
            NextState("RELEASE_PLL_RESET")
        )
        startup_fsm.act("RELEASE_PLL_RESET",
            self.debug.eq(1),
            gtXxreset.eq(1),
            startup_timer.wait.eq(1),
            If(plllock & startup_timer.done, NextState("RELEASE_GTP_RESET"))
        )
        # Release GTP reset and wait for GTP resetdone
        # (from UG482, GTP is reset on falling edge
        # of gtXxreset)
        if rx:
            startup_fsm.act("RELEASE_GTP_RESET",
                self.debug.eq(2),
                Xxuserrdy.eq(1),
                cdr_stable_timer.wait.eq(1),
                #If(Xxresetdone & cdr_stable_timer.done, NextState("ALIGN"))
                If(cdr_stable_timer.done, NextState("ALIGN")) # FIXME!
            )
            # Delay alignment
            startup_fsm.act("ALIGN",
                self.debug.eq(3),
                Xxuserrdy.eq(1),
                Xxdlysreset.eq(1),
                If(Xxdlysresetdone,
                    NextState("WAIT_ALIGN_DONE")
                )
            )
            startup_fsm.act("WAIT_ALIGN_DONE",
                self.debug.eq(4),
                Xxuserrdy.eq(1),
                If(1,
                #If(Xxsyncdone, # FIXME!
                    NextState("READY")
                )
            )
        else:
            startup_fsm.act("RELEASE_GTP_RESET",
                self.debug.eq(2),
                Xxuserrdy.eq(1),
                If(Xxresetdone, NextState("ALIGN"))
            )
            # Delay alignment
            startup_fsm.act("ALIGN",
                self.debug.eq(3),
                Xxuserrdy.eq(1),
                Xxdlysreset.eq(1),
                If(Xxdlysresetdone,
                    NextState("PHINIT")
                )
            )
            # Phase init
            startup_fsm.act("PHINIT",
                self.debug.eq(4),
                Xxuserrdy.eq(1),
                Xxphinit.eq(1),
                If(Xxphinitdone,
                    NextState("PHALIGN")
                )
            )
            # Phase align
            startup_fsm.act("PHALIGN",
                self.debug.eq(5),
                Xxuserrdy.eq(1),
                Xxphalign.eq(1),
                If(Xxphaligndone_rising,
                    NextState("DLYEN")
                )
            )
            startup_fsm.act("DLYEN",
                self.debug.eq(6),
                Xxuserrdy.eq(1),
                Xxdlyen.eq(1),
                If(Xxphaligndone_rising,
                    NextState("READY")
                )
            )
        startup_fsm.act("READY",
            self.debug.eq(7),
            Xxuserrdy.eq(1),
            self.done.eq(1),
            If(self.restart, NextState("RESET_ALL"))
        )
