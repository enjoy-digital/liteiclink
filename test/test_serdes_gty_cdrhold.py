#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from test.test_serdes_xilinx import FAMILIES, SerDesTop, _instances, _input

# Test GTY RXCDRHOLD -------------------------------------------------------------------------------

class TestSerDesGTYCDRHold(unittest.TestCase):
    def test_rxcdrhold_follows_loopback(self):
        # RXCDRHOLD is set in Near-End PCS loopback (0b001) only.
        device, channel, common, cpll_cls, qpll_cls, serdes_cls = FAMILIES["gty"]
        top      = SerDesTop(qpll_cls, serdes_cls, 125e6, 2.5e9)
        fragment = top.get_fragment()
        gty,     = _instances(fragment, channel)
        rxcdrhold = Signal()
        dut = Module()
        dut.comb += rxcdrhold.eq(_input(gty, "RXCDRHOLD"))
        results = {}
        def generator():
            for loopback in range(8):
                yield top.serdes.loopback.eq(loopback)
                yield
                results[loopback] = (yield rxcdrhold)
        run_simulation(dut, generator())
        self.assertEqual(results, {loopback: int(loopback == 0b001) for loopback in range(8)})

if __name__ == "__main__":
    unittest.main()
