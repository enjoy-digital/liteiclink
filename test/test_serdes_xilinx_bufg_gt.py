#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from test.test_serdes_xilinx import FAMILIES, SerDesTop, _instances, _input

# Test UltraScale(+) BUFG_GT Divider ---------------------------------------------------------------

class TestSerDesUltraScaleBUFGGT(unittest.TestCase):
    def elaborate(self, family, refclk_freq, linerate, **kwargs):
        device, channel, common, cpll_cls, qpll_cls, serdes_cls = FAMILIES[family]
        return SerDesTop(cpll_cls, serdes_cls, refclk_freq, linerate, **kwargs).get_fragment()

    def test_bufg_gt_divider(self):
        # Without TX buffer, the BUFG_GT divider (3-bit DIV: 1 to 8) is the reference clock to TX
        # user clock ratio.
        for family in ["gth3", "gth4", "gty"]:
            with self.subTest(family=family):
                # 250 MHz reference, 1.25 Gbps, 20-bit: 62.5 MHz user clock, ratio 4.
                divs = []
                for bufg_gt in _instances(self.elaborate(family, 250e6, 1.25e9), "BUFG_GT"):
                    try:
                        divs.append(_input(bufg_gt, "DIV").value)
                    except KeyError: # RX BUFG_GT (no divider).
                        pass
                self.assertEqual(divs, [4 - 1])
                # 250 MHz reference, 625 Mbps, 40-bit: 15.625 MHz user clock, ratio 16 (does not fit
                # in DIV, rejected instead of truncated).
                with self.assertRaises(AssertionError):
                    self.elaborate(family, 250e6, 625e6, data_width=40)

if __name__ == "__main__":
    unittest.main()
