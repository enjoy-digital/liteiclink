#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen.sim import *

from test.test_serwb_link import DUTLink, wait_link_ready, bus_access

# Test SERWB RX Overflows --------------------------------------------------------------------------

class TestSERWBRXOverflows(unittest.TestCase):
    def overflows_test(self, n, **kwargs):
        # Writes to a slow remote bus (no flow control). Returns (remote SRAM ok, rx_overflows).
        dut     = DUTLink(sram_latency=200, **kwargs)
        results = {}
        def generator():
            yield from wait_link_ready(dut)
            for i in range(n):
                _, err = yield from bus_access(dut.bus, i, we=1, dat_w=0x1000 + i)
                self.assertEqual(err, 0)
            for _ in range(n*400):
                yield
            results["sram"] = []
            for i in range(n):
                results["sram"].append((yield dut.sram.mem[i]))
            results["overflows"] = (yield dut.slave_core.rx_overflows.status)
        run_simulation(dut, generator())
        return results["sram"] == [0x1000 + i for i in range(n)], results["overflows"]

    def test_idle_words_not_counted(self):
        # One write completing correctly: only idle words are lost while the remote bus is slow,
        # rx_overflows must stay at 0.
        for kwargs in [{}, {"with_crc": True}]:
            self.assertEqual(self.overflows_test(n=1, **kwargs), (True, 0))

    def test_packet_words_counted(self):
        # Back-to-back writes (non-consecutive, not coalesced) to the slow remote bus: requests are
        # lost and counted.
        dut     = DUTLink(sram_latency=200, coalescing_timeout=0)
        results = {}
        def generator():
            yield from wait_link_ready(dut)
            for i in range(8):
                _, err = yield from bus_access(dut.bus, 2*i, we=1, dat_w=0x1000 + i)
            for _ in range(8*400):
                yield
            results["sram"] = []
            for i in range(8):
                results["sram"].append((yield dut.sram.mem[2*i]))
            results["overflows"] = (yield dut.slave_core.rx_overflows.status)
        run_simulation(dut, generator())
        self.assertNotEqual(results["sram"], [0x1000 + i for i in range(8)])
        self.assertGreater(results["overflows"], 0)

if __name__ == "__main__":
    unittest.main()
