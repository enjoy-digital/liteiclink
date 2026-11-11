#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
import warnings

from migen import *

from litex.gen.sim import *

from test.test_serwb_link import DUTLink

# Test SERWB Link Down -----------------------------------------------------------------------------

class TestSERWBLinkDown(unittest.TestCase):
    def link_down_test(self, **kwargs):
        # Without link, writes and reads must end with err only (never ack and err together).
        dut     = DUTLink(connected=False, **kwargs)
        results = []
        def generator():
            for _ in range(100):
                yield
            for we in [1, 1, 0]:
                yield dut.bus.adr.eq(0)
                yield dut.bus.we.eq(we)
                yield dut.bus.dat_w.eq(0x12345678)
                yield dut.bus.sel.eq(0b1111)
                yield dut.bus.cyc.eq(1)
                yield dut.bus.stb.eq(1)
                yield
                for _ in range(100):
                    ack, err = (yield dut.bus.ack), (yield dut.bus.err)
                    if ack or err:
                        results.append((we, ack, err))
                        break
                    yield
                yield dut.bus.cyc.eq(0)
                yield dut.bus.stb.eq(0)
                yield
        run_simulation(dut, generator())
        self.assertEqual(results, [(1, 0, 1), (1, 0, 1), (0, 0, 1)])

    def test_link_down_native(self):
        self.link_down_test()

    def test_link_down_etherbone(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", FutureWarning)
            self.link_down_test(protocol="etherbone")

if __name__ == "__main__":
    unittest.main()
