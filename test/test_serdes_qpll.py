#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Instance, Signal

from liteiclink.serdes.gth4_ultrascale import GTH4QuadPLL
from liteiclink.serdes.gty_ultrascale import GTYQuadPLL


class TestQPLLReferenceClock(unittest.TestCase):
    def test_reference_input_and_selector(self):
        for cls, primitive in ((GTH4QuadPLL, "GTHE4_COMMON"), (GTYQuadPLL, "GTYE4_COMMON")):
            for linerate in (5.15625e9, 10.3125e9, 15.625e9):
                for fabric in (False, True):
                    with self.subTest(pll=cls.__name__, linerate=linerate, fabric=fabric):
                        refclk = Signal()
                        pll = cls(refclk, 156.25e6, linerate, refclk_from_fabric=fabric)
                        fragment = pll.get_fragment()
                        common = next(s for s in fragment.specials
                            if isinstance(s, Instance) and s.of == primitive)
                        index = int(pll.config["qpll"][-1])
                        selected = f"GTGREFCLK{index}" if fabric else f"GTREFCLK0{index}"
                        self.assertIs(common.get_io(selected), refclk)
                        self.assertEqual(common.get_io(f"QPLL{index}REFCLKSEL").value,
                            0b111 if fabric else 0b001)
                        unused = f"GTREFCLK0{index}" if fabric else f"GTGREFCLK{index}"
                        self.assertEqual(common.get_io(unused).value, 0)

    def test_default_keeps_dedicated_reference(self):
        for cls in (GTH4QuadPLL, GTYQuadPLL):
            pll = cls(Signal(), 156.25e6, 10.3125e9)
            common = next(s for s in pll.get_fragment().specials if isinstance(s, Instance))
            self.assertEqual(common.get_io("QPLL1REFCLKSEL").value, 0b001)
