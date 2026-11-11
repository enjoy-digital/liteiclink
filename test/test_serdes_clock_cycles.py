#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from liteiclink.serdes.common      import ClockCycles
from liteiclink.serdes.serdes_ecp5 import SerDesECP5

from test.test_serdes_xilinx import FAMILIES, SerDesTop

# Clock Cycles -------------------------------------------------------------------------------------

# Unrelated sys/tx/rx clock periods (even: the simulator toggles clocks every period//2).
CLOCKS = {"sys": 20, "tx": 14, "rx": 26, "tx_full": 20, "rx_full": 20}

class TestClockCycles(unittest.TestCase):
    def test_clock_cycles(self):
        # The counter is Gray coded in its domain (one bit changes per cycle) and the sys value is
        # monotonic and follows the clock ratio.
        dut     = ClockCycles("tx")
        grays   = []
        values  = []
        def tx_generator():
            for _ in range(1000):
                grays.append((yield dut.counter.q))
                yield
        def sys_generator():
            for _ in range(700):
                values.append((yield dut.value))
                yield
        run_simulation(dut, {"tx": tx_generator(), "sys": sys_generator()}, clocks=CLOCKS)
        for a, b in zip(grays, grays[1:]):
            self.assertLessEqual(bin(a ^ b).count("1"), 1)
        self.assertEqual(values, sorted(values))
        delta = values[-1] - values[100]
        self.assertAlmostEqual(delta, (len(values) - 1 - 100)*20/14, delta=4)

    def test_xilinx_resynchronized(self):
        # The tx/rx counters reach sys through MultiRegs (no direct sampling of tx/rx registers).
        for name, (device, _, _, cpll, qpll, serdes_cls) in FAMILIES.items():
            with self.subTest(family=name):
                top = SerDesTop(cpll or qpll, serdes_cls, 125e6, 2.5e9)
                for counter in [top.serdes.clock_tx_counter, top.serdes.clock_rx_counter]:
                    self.assertIsInstance(counter, ClockCycles)
                    specials = counter.get_fragment().specials
                    self.assertTrue(any(isinstance(s, MultiReg) for s in specials))

    def test_ecp5_resynchronized(self):
        class PLL:
            refclk = Signal()
            config = {"m": 20, "d": 1, "linerate": 3e9}
        class Pads:
            def __init__(self):
                self.p = Signal()
                self.n = Signal()
        serdes = SerDesECP5(PLL(), Pads(), Pads())
        serdes.add_clock_cycles()
        for counter in [serdes.clock_tx_counter, serdes.clock_rx_counter]:
            self.assertIsInstance(counter, ClockCycles)
            specials = counter.get_fragment().specials
            self.assertTrue(any(isinstance(s, MultiReg) for s in specials))

if __name__ == "__main__":
    unittest.main()
