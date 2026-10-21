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

from litex.soc.interconnect import stream

from liteiclink.serwb.common import SerdesControl, SERWBPHYBase

# Fake SerDes/Init ---------------------------------------------------------------------------------

class FakeSerdes(LiteXModule):
    """Word-level SerDes model (one word every `ratio` cycles, no RX backpressure)."""
    def __init__(self, ratio=5):
        self.reset = Signal()
        self.tx, self.rx = Module(), Module()
        self.tx.sink   = stream.Endpoint([("data", 32)])
        self.rx.source = stream.Endpoint([("data", 32)])
        self.rx.idle   = Signal()
        self.rx.datapath = Module()
        self.rx.datapath.invalid = Signal()

        # # #

        self.ce = ce = Signal(ratio, reset=1)
        self.sync += ce.eq(Cat(ce[1:], ce[0]))
        self.comb += self.tx.sink.ready.eq(ce[0])

class FakeInit(LiteXModule):
    def __init__(self):
        self.ready = Signal(reset=1)
        self.error = Signal()
        self.reset = Signal()
        self.shift = Signal(6)

class PHYModel(SERWBPHYBase):
    def __init__(self, mode="master", with_prbs=True):
        self.sink   = stream.Endpoint([("data", 32)])
        self.source = stream.Endpoint([("data", 32)])
        self.serdes  = FakeSerdes()
        self.init    = FakeInit()
        self.control = SerdesControl(self.serdes, self.init, mode, with_delay=False, with_prbs=with_prbs)
        self.add_datapath(with_prbs=with_prbs)

class DUTPRBS(LiteXModule):
    def __init__(self):
        self.a = a = PHYModel("master")
        self.b = b = PHYModel("slave")
        self.flip = Signal(32) # Bit errors injection (A to B).
        self.comb += [
            b.serdes.rx.source.valid.eq(a.serdes.tx.sink.valid & a.serdes.tx.sink.ready),
            b.serdes.rx.source.data.eq(a.serdes.tx.sink.data ^ self.flip),
        ]

# Helpers ------------------------------------------------------------------------------------------

def prbs_run(control, cycles=400):
    yield control.prbs_cycles.storage.eq(cycles)
    yield control.prbs_start.wr_stb.eq(1)
    yield
    yield control.prbs_start.wr_stb.eq(0)
    for _ in range(cycles + 10):
        yield
    return (yield control.prbs_errors.status)

# Test Health --------------------------------------------------------------------------------------

class TestSERWBHealth(unittest.TestCase):
    def test_prbs(self):
        dut     = DUTPRBS()
        results = {}
        forwarded = []
        @passive
        def monitor():
            while True:
                if (yield dut.b.control.prbs_rx.storage) and (yield dut.b.source.valid):
                    forwarded.append(1)
                yield
        def generator():
            yield dut.a.control.prbs_tx.storage.eq(1)
            yield dut.b.control.prbs_rx.storage.eq(1)
            yield dut.b.source.ready.eq(1)
            # Let checker synchronize on received PRBS.
            for _ in range(200):
                yield
            results["clean"] = yield from prbs_run(dut.b.control)
            # Inject a bit error.
            yield dut.b.control.prbs_cycles.storage.eq(400)
            yield dut.b.control.prbs_start.wr_stb.eq(1)
            yield
            yield dut.b.control.prbs_start.wr_stb.eq(0)
            for i in range(410):
                # Bit error during one word period (one word every 5 cycles).
                yield dut.flip.eq(1 << 5 if 100 <= i < 105 else 0)
                yield
            results["errors"] = (yield dut.b.control.prbs_errors.status)
        run_simulation(dut, [generator(), monitor()])
        self.assertEqual(results["clean"], 0)
        self.assertGreater(results["errors"], 0)
        # PRBS words must not be forwarded to the core.
        self.assertEqual(len(forwarded), 0)

    def test_prbs_disabled_traffic(self):
        # Without prbs_tx/prbs_rx, traffic goes through.
        dut     = DUTPRBS()
        results = []
        @passive
        def monitor():
            yield dut.b.source.ready.eq(1)
            while True:
                if (yield dut.b.source.valid) and ((yield dut.b.source.data) != 0):
                    results.append((yield dut.b.source.data))
                yield
        def generator():
            for value in [0x11223344, 0x55667788]:
                yield dut.a.sink.valid.eq(1)
                yield dut.a.sink.data.eq(value)
                yield
                while not (yield dut.a.sink.ready):
                    yield
            yield dut.a.sink.valid.eq(0)
            for _ in range(32):
                yield
        run_simulation(dut, [generator(), monitor()])
        self.assertEqual(results, [0x11223344, 0x55667788])

    def test_link_drops(self):
        phy = PHYModel(with_prbs=False)
        def generator():
            for ready in [1, 0, 0, 1, 1, 0, 1]:
                yield phy.init.ready.eq(ready)
                yield
            yield
            self.assertEqual((yield phy.control.link_drops.status), 2)
        run_simulation(phy, generator())

    def auto_retrain_test(self, invalid_cycles):
        serdes  = FakeSerdes()
        init    = FakeInit()
        control = SerdesControl(serdes, init, "master", with_delay=False,
            with_auto_retrain = True,
            retrain_threshold = 4,
            retrain_window    = 100,
        )
        dut = Module()
        dut.submodules += serdes, init, control
        resets = []
        def generator():
            for i in range(300):
                yield serdes.rx.datapath.invalid.eq(i in invalid_cycles)
                if (yield init.reset):
                    resets.append(i)
                yield
        run_simulation(dut, generator())
        return resets

    def test_auto_retrain(self):
        # 4 invalid words within window: retrain (init reset).
        self.assertNotEqual(self.auto_retrain_test(invalid_cycles=[10, 20, 30, 40]), [])

    def test_auto_retrain_sparse_errors(self):
        # Sparse invalid words (below threshold within window): no retrain.
        self.assertEqual(self.auto_retrain_test(invalid_cycles=[10, 120, 230]), [])

if __name__ == "__main__":
    unittest.main()
