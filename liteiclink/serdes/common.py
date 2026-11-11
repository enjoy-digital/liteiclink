#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter

from litex.gen import *

from litex.soc.interconnect.csr import *

# Clock Cycles -------------------------------------------------------------------------------------

class ClockCycles(LiteXModule):
    """Free-running cycles counter of the clock domain `cd`, readable in sys (`value`).

    The counter is Gray coded in `cd` (one bit changes per cycle) and resynchronized to sys, so
    that `value` is always a coherent count (lagging by the synchronization latency).
    """
    def __init__(self, cd, width=32):
        self.value = Signal(width)

        # # #

        # Gray Counter (cd).
        self.counter = counter = ClockDomainsRenamer(cd)(GrayCounter(width))
        self.comb += counter.ce.eq(1)

        # Resynchronization / Decoding (sys): binary bit n is the XOR of the Gray bits n and above.
        gray = Signal(width)
        self.specials += MultiReg(counter.q, gray)
        self.sync += [self.value[n].eq(Reduce("XOR", gray[n:])) for n in range(width)]

# DRP Layout ---------------------------------------------------------------------------------------

_drp_layout = [
    ("clk",                1, DIR_M_TO_S),
    ("en",                 1, DIR_M_TO_S),
    ("we",                 1, DIR_M_TO_S),
    ("rdy",                1, DIR_S_TO_M),
    ("addr", "address_width", DIR_M_TO_S),
    ("di",      "data_width", DIR_M_TO_S),
    ("do",      "data_width", DIR_S_TO_M),
]

# DRP Interface ------------------------------------------------------------------------------------

class DRPInterface(Record):
    def __init__(self, address_width=9, data_width=16):
        Record.__init__(self, set_layout_parameters(_drp_layout,
            address_width = address_width,
            data_width    = data_width,
        ))

# DRP Mux ------------------------------------------------------------------------------------------

class DRPMux(LiteXModule, DRPInterface):
    def __init__(self, **kwargs):
        DRPInterface.__init__(self, **kwargs)
        self.sel = Signal(4)
        self.interfaces = []

    def add_interface(self, interface):
        self.interfaces.append(interface)

    def do_finalize(self):
        assert len(self.interfaces) <= 16
        cases = {}
        for i, interface in enumerate(self.interfaces):
            cases[i] = interface.connect(self)
        self.comb += Case(self.sel, cases)

# DRP Control --------------------------------------------------------------------------------------

class DRPControl(LiteXModule):
    def __init__(self, drp):
        aw = len(drp.addr)
        self._addr  = CSRStorage(aw, description=f"DRP Address ({aw} bits).")
        self._di    = CSRStorage(16, description="DRP Data In (16 bits) for write operation.")
        self._do    = CSRStatus(16,  description="DRP Data Out (16 bits) for read operation.")
        self._we    = CSRStorage(1,  description="DRP Write Enable.")
        self._start = CSRStorage(1,  description="Write to initiate DRP operation.")
        self._done  = CSRStatus(1,   description="DRP operation is complete.")

        # # #

        # Clk.
        self.comb += drp.clk.eq(ClockSignal("sys"))

        # FSM.
        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(self._start.wr_stb)

        fsm.act("IDLE",
            NextState("ACCESS")
        )
        fsm.act("ACCESS",
            drp.en.eq(1),
            drp.we.eq(self._we.storage),
            drp.addr.eq(self._addr.storage),
            drp.di.eq(self._di.storage),
            NextState("WAIT")
        )
        fsm.act("WAIT",
            If(drp.rdy,
                NextValue(self._do.status, drp.do),
                NextState("DONE")
            )
        )
        fsm.act("DONE",
            self._done.status.eq(1)
        )
