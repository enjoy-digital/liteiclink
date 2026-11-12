#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *

from liteiclink.serdes.serdes_ecp5 import SerDesECP5, SerDesECP5SCIReconfig

# Loopback Control ---------------------------------------------------------------------------------

class PLL:
    refclk = Signal()
    config = {"m": 20, "d": 1, "linerate": 3e9}

class Pads:
    def __init__(self):
        self.p = Signal()
        self.n = Signal()

class SerDesStub:
    def __init__(self):
        self.serdes_params = {}

CLOCKS = {cd: 10 for cd in ["sys", "tx", "rx", "tx_full", "rx_full"]}

def ecp5_serdes(**kwargs):
    serdes = SerDesECP5(PLL(), Pads(), Pads(), **kwargs)
    serdes.do_finalize = lambda: None # No DCUA instance in simulation.
    return serdes

class TestSerDesECP5Loopback(unittest.TestCase):
    def test_loopback_register(self):
        # The loopback register takes the 3-bit encoding of the other wrappers (test_prbs.py
        # --loopback writes the Near-End PMA Loopback mode 0b010).
        serdes = ecp5_serdes()
        serdes.add_loopback_control()
        results = {}
        def generator():
            for value in [0b000, 0b001, 0b010, 0b100, 0b110, 0b000]:
                yield from serdes._loopback.write(value)
                yield
                results[value] = (yield serdes.loopback)
        run_simulation(serdes, generator(), clocks=CLOCKS)
        self.assertEqual(results, {v: v for v in [0b000, 0b001, 0b010, 0b100, 0b110]})

    def sci_writes(self, loopback, pden_sel=1, cycles=512):
        # SCI writes of the reconfiguration loop: {address: last data written}. SCI reads return
        # 0 (no DCU in simulation).
        dut    = SerDesECP5SCIReconfig(SerDesStub())
        writes = {}
        def generator():
            yield dut.loopback.eq(loopback)
            yield dut.pden_sel.eq(pden_sel)
            for _ in range(cycles):
                if (yield dut.sci.sci_wrn) == 0:
                    writes[(yield dut.sci.sci_addr)] = (yield dut.sci.sci_wdata)
                yield
        run_simulation(dut, generator())
        return writes

    def test_sci_loopback_modes(self):
        # CH_15 lb_ctl (TN1261): bit 0 serial TX to RX (Near-End), bit 1 serial RX equalizer to TX
        # (Far-End), cleared otherwise (also out of loopback). CH_1A pden_sel cleared in Near-End
        # loopback, attribute value otherwise (written only when it differs from the read value).
        for loopback, lb_ctl, pden_sel in [
            (0b000, 0b0000, 1),
            (0b001, 0b0001, 0),
            (0b010, 0b0001, 0),
            (0b100, 0b0010, 1),
            (0b110, 0b0000, 1),
            ]:
            with self.subTest(loopback=loopback):
                writes = self.sci_writes(loopback)
                self.assertEqual(writes[0x15] & 0b1111, lb_ctl)
                if pden_sel:
                    self.assertEqual((writes[0x1a] >> 3) & 1, 1)
                else:
                    self.assertNotIn(0x1a, writes) # Read value (0) already right.
        # Attribute overridden to 0b0: never written.
        self.assertNotIn(0x1a, self.sci_writes(0b000, pden_sel=0))

    def test_far_end_parallel_loopback(self):
        # Far-End parallel loopback through the DCU FFC_SB_PFIFO_LP input (TN1261 sb_felb_c).
        serdes = ecp5_serdes()
        results = {}
        def generator():
            for loopback in [0b000, 0b010, 0b100, 0b110]:
                yield serdes.loopback.eq(loopback)
                yield
                results[loopback] = (yield serdes.serdes_params["i_CHX_FFC_SB_PFIFO_LP"])
        run_simulation(serdes, generator(), clocks=CLOCKS)
        self.assertEqual(results, {0b000: 0, 0b010: 0, 0b100: 0, 0b110: 1})

    def test_near_end_loopback_masks_rx_los(self):
        # Near-End serial loopback: the RX loss of signal (RX pins unconnected) does not hold the
        # init; the other modes keep it (the RX receives the link partner).
        for with_oob in [False, True]:
            serdes  = ecp5_serdes(with_oob=with_oob)
            rx_los  = serdes.serdes_params["o_CHX_FFS_RLOS"]
            results = {}
            def generator():
                yield rx_los.eq(1)
                for loopback in [0b000, 0b001, 0b010, 0b100, 0b110]:
                    yield serdes.loopback.eq(loopback)
                    for _ in range(4):
                        yield
                    results[loopback] = (yield serdes.init.rx_los)
            run_simulation(serdes, generator(), clocks=CLOCKS)
            with self.subTest(with_oob=with_oob):
                self.assertEqual(results, {0b000: 1, 0b001: 0, 0b010: 0, 0b100: 1, 0b110: 1})

class TestSerDesECP5TXInhibit(unittest.TestCase):
    def test_tx_inhibit_sci(self):
        # Default mode: tx_inhibit requests the TX electrical idle (SCI pcie_ei_en, as tx_idle).
        serdes  = ecp5_serdes()
        results = []
        def generator():
            for tx_inhibit in [0, 1, 0]:
                yield serdes.tx_inhibit.eq(tx_inhibit)
                yield
                results.append((yield serdes.sci_reconfig.tx_idle))
        run_simulation(serdes, generator(), clocks=CLOCKS)
        self.assertEqual(results, [0, 1, 0])

    def test_tx_inhibit_oob(self):
        # OOB mode: tx_inhibit drives FFC_EI_EN, also with tx_raw_enable (tx_idle does not).
        serdes  = ecp5_serdes(with_oob=True)
        ei_en   = serdes.serdes_params["i_CHX_FFC_EI_EN"]
        results = {}
        def generator():
            for tx_idle, tx_inhibit, tx_raw_enable in [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 0), (0, 1, 1)]:
                yield serdes.tx_idle.eq(tx_idle)
                yield serdes.tx_inhibit.eq(tx_inhibit)
                yield serdes.tx_raw_enable.eq(tx_raw_enable)
                for _ in range(4):
                    yield
                results[(tx_idle, tx_inhibit, tx_raw_enable)] = (yield ei_en)
        run_simulation(serdes, generator(), clocks=CLOCKS)
        self.assertEqual(results, {(0, 0, 0): 0, (1, 0, 0): 1, (1, 0, 1): 0, (0, 1, 0): 1, (0, 1, 1): 1})

if __name__ == "__main__":
    unittest.main()
