#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen.sim import *

from liteiclink.serwb.mmap import MMAP_MAGIC

from test.test_serwb_link import DUTLink, wait_link_ready, bus_access

# Test SERWB Packet Timeout ------------------------------------------------------------------------

class TestSERWBPacketTimeout(unittest.TestCase):
    def test_truncated_write_request(self):
        # The data word of a write request is lost and the link is silent (no word received, as
        # with flow control idle words): after the depacketizer timeout (1000 cycles), the next
        # request must not be taken as the continuation of the truncated one.
        dut = DUTLink(connected=False)
        dut.drop = Signal()
        dut.comb += [
            dut.master_phy.tx.connect(dut.slave_phy.rx),
            dut.slave_phy.tx.connect(dut.master_phy.rx),
            If(dut.drop, dut.slave_phy.rx.valid.eq(0)),
            dut.master_phy.init.ready.eq(1),
            dut.slave_phy.init.ready.eq(1),
        ]
        @passive
        def injector():
            # Drop the word following the address word of the first write request.
            tx    = dut.master_phy.tx
            index = None
            while True:
                if (yield tx.valid):
                    data = (yield tx.data)
                    if index is None and (data >> 16) == MMAP_MAGIC:
                        index = 0
                    elif index is not None:
                        index += 1
                        if index == 1: # Address word.
                            yield dut.drop.eq(1)
                            for _ in range(1500):
                                yield
                            yield dut.drop.eq(0)
                            return
                yield
        def generator():
            yield from wait_link_ready(dut)
            _, err = yield from bus_access(dut.bus, 0, we=1, dat_w=0x11111111)
            self.assertEqual(err, 0)
            # Wait for the end of the silence.
            for _ in range(2000):
                yield
            _, err = yield from bus_access(dut.bus, 5, we=1, dat_w=0x55555555)
            self.assertEqual(err, 0)
            for _ in range(500):
                yield
            self.assertEqual(hex((yield dut.sram.mem[0])), hex(0))          # Lost.
            self.assertEqual(hex((yield dut.sram.mem[5])), hex(0x55555555)) # Received.
            dat_r, err = yield from bus_access(dut.bus, 5)
            self.assertEqual((hex(dat_r), err), (hex(0x55555555), 0))
        run_simulation(dut, [generator(), injector()])

if __name__ == "__main__":
    unittest.main()
