#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *
from litex.gen.sim import *

from liteiclink.serwb.genphy import SERWBPHY

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    """Two generic PHYs (bit-level link), counters sent on both sides."""
    def __init__(self, **kwargs):
        master_pads = Record([("tx", 1), ("rx", 1)])
        slave_pads  = Record([("tx", 1), ("rx", 1)])
        self.comb += [
            slave_pads.rx.eq(master_pads.tx),
            master_pads.rx.eq(slave_pads.tx),
        ]
        self.master_phy = SERWBPHY("sim", master_pads, mode="master", init_timeout=128, **kwargs)
        self.slave_phy  = SERWBPHY("sim", slave_pads,  mode="slave",  init_timeout=128, **kwargs)
        self.ready = Signal()
        self.comb += self.ready.eq(self.master_phy.init.ready & self.slave_phy.init.ready)
        for phy in [self.master_phy, self.slave_phy]:
            self.sync += [
                phy.sink.valid.eq(self.ready),
                If(phy.sink.valid & phy.sink.ready, phy.sink.data.eq(phy.sink.data + 1)),
            ]

# Test SerWB PHY RX Drop ---------------------------------------------------------------------------

class TestSERWBPHYRXDrop(unittest.TestCase):
    def rx_drop_test(self, **kwargs):
        # The master RX source is not ready for a while (words lost, as on an RX buffer overflow):
        # whole words must be dropped, the next words must be received unchanged (word alignment
        # kept).
        dut     = DUT(**kwargs)
        results = []
        def generator():
            source = dut.master_phy.source
            for _ in range(20000):
                if (yield dut.ready):
                    break
                yield
            else:
                raise TimeoutError("SerWB link not ready.")
            for i in range(2000):
                yield source.ready.eq(not (100 <= i < 213))
                if (yield source.valid) and (yield source.ready):
                    results.append((yield source.data))
                yield
        run_simulation(dut, generator())
        # After the gap, received words are consecutive counter values.
        tail = results[-16:]
        self.assertEqual(tail, list(range(tail[0], tail[0] + 16)), [hex(r) for r in tail])

    def test_rx_drop(self):
        self.rx_drop_test()

    def test_rx_drop_flow_control(self):
        self.rx_drop_test(with_flow_control=True)

if __name__ == "__main__":
    unittest.main()
