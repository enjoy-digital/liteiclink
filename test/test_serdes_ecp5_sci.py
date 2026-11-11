#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *

from liteiclink.serdes.serdes_ecp5 import SerDesECP5, SerDesECP5SCIReconfig

# SCI Reconfiguration ------------------------------------------------------------------------------

class SerDesStub:
    def __init__(self):
        self.serdes_params = {}

class TestSerDesECP5SCI(unittest.TestCase):
    def test_dual_location_not_select(self):
        # dual is the DCU location of the instance, not a register bank select: the DCU (aux)
        # SCI enable/select must not follow it (only paused user accesses with sel=1 drive it).
        class PLL:
            refclk = Signal()
            config = {"m": 20, "d": 1, "linerate": 3e9}
        class Pads:
            p = Signal()
            n = Signal()
        serdes = SerDesECP5(PLL(), Pads(), Pads(), dual=1)
        serdes.do_finalize = lambda: None # No DCUA instance in simulation.
        samples = []
        def generator():
            for _ in range(64):
                samples.append((yield serdes.sci_reconfig.sci.dual_sel))
                yield
        clocks = {cd: 10 for cd in ["sys", "tx", "rx", "tx_full", "rx_full"]}
        run_simulation(serdes, generator(), clocks=clocks)
        self.assertEqual(set(samples), {0})

    def test_reconfig_loop_channel_only(self):
        # The hardware reconfiguration loop only accesses the channel registers.
        dut     = SerDesECP5SCIReconfig(SerDesStub())
        samples = []
        def generator():
            for _ in range(256):
                samples.append(((yield dut.sci.chan_sel), (yield dut.sci.dual_sel)))
                yield
        run_simulation(dut, generator())
        self.assertIn((1, 0), samples)
        self.assertNotIn((1, 1), samples)
        self.assertNotIn((0, 1), samples)

    def test_dual_access(self):
        # A paused, user DCU access (sel=1) selects the DCU registers only.
        dut     = SerDesECP5SCIReconfig(SerDesStub())
        samples = []
        def generator():
            yield dut.pause.storage.eq(1)
            yield dut.sel.storage.eq(1)
            # Wait for the loop to complete (paused in IDLE).
            for _ in range(128):
                yield
            while (yield dut.done.status) == 0:
                yield
            yield dut.re.re.eq(1)
            yield
            yield dut.re.re.eq(0)
            for _ in range(8):
                samples.append(((yield dut.sci.chan_sel), (yield dut.sci.dual_sel)))
                yield
        run_simulation(dut, generator())
        self.assertIn((0, 1), samples)
        self.assertNotIn((1, 1), samples)

if __name__ == "__main__":
    unittest.main()
