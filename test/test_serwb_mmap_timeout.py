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

# Helpers ------------------------------------------------------------------------------------------

def lossy_link(dut):
    """Connects the PHYs of an unconnected DUTLink with word loss injection on each direction
    (lost words are replaced by idle words)."""
    dut.drop_requests  = Signal() # SoC side to remote side.
    dut.drop_responses = Signal() # Remote side to SoC side.
    dut.comb += [
        dut.master_phy.tx.connect(dut.slave_phy.rx),
        dut.slave_phy.tx.connect(dut.master_phy.rx),
        If(dut.drop_requests,  dut.slave_phy.rx.data.eq(0)),
        If(dut.drop_responses, dut.master_phy.rx.data.eq(0)),
        dut.master_phy.init.ready.eq(1),
        dut.slave_phy.init.ready.eq(1),
    ]
    return dut

# Test SERWB MMAP Timeout --------------------------------------------------------------------------

class TestSERWBMMAPTimeout(unittest.TestCase):
    def read_with_loss(self, dut, drop):
        yield from wait_link_ready(dut)
        _, err = yield from bus_access(dut.bus, 2, we=1, dat_w=0x12345678)
        self.assertEqual(err, 0)
        for _ in range(200):
            yield
        # Read with the request (or its response) lost.
        yield getattr(dut, drop).eq(1)
        dat_r, err = yield from bus_access(dut.bus, 2, timeout=5000)
        self.assertEqual(err, 1)
        yield getattr(dut, drop).eq(0)
        # Next accesses work.
        _, err = yield from bus_access(dut.bus, 3, we=1, dat_w=0xcafe0003)
        self.assertEqual(err, 0)
        for addr, value in [(2, 0x12345678), (3, 0xcafe0003)]:
            dat_r, err = yield from bus_access(dut.bus, addr, timeout=5000)
            self.assertEqual((hex(dat_r), err), (hex(value), 0))

    def test_lost_read_request(self):
        dut = lossy_link(DUTLink(connected=False))
        run_simulation(dut, self.read_with_loss(dut, "drop_requests"))

    def test_lost_read_response(self):
        dut = lossy_link(DUTLink(connected=False))
        run_simulation(dut, self.read_with_loss(dut, "drop_responses"))

    def test_abandoned_read(self):
        # The SoC side master gives up a slow read (as the LiteX bus timeout does): the late
        # response must not be given to the next access.
        dut = DUTLink(sram_latency=600)
        def generator():
            yield from wait_link_ready(dut)
            for addr, value in [(3, 0x33333333), (4, 0x44444444)]:
                _, err = yield from bus_access(dut.bus, addr, we=1, dat_w=value, timeout=5000)
                self.assertEqual(err, 0)
            for _ in range(3000):
                yield
            # Read of word 3, abandoned before its response.
            yield dut.bus.adr.eq(3)
            yield dut.bus.we.eq(0)
            yield dut.bus.cyc.eq(1)
            yield dut.bus.stb.eq(1)
            for _ in range(200):
                self.assertEqual((yield dut.bus.ack), 0)
                yield
            yield dut.bus.cyc.eq(0)
            yield dut.bus.stb.eq(0)
            yield
            # Read of word 4: must return word 4.
            dat_r, err = yield from bus_access(dut.bus, 4, timeout=5000)
            self.assertEqual((hex(dat_r), err), (hex(0x44444444), 0))
            dat_r, err = yield from bus_access(dut.bus, 3, timeout=5000)
            self.assertEqual((hex(dat_r), err), (hex(0x33333333), 0))
        run_simulation(dut, generator())

    def test_no_response_timeout(self):
        # response_timeout=None: the SoC side waits for the response (previous behavior).
        dut = DUTLink(response_timeout=None)
        self.assertFalse(hasattr(dut.master_core.mmap, "timer"))

if __name__ == "__main__":
    unittest.main()
