#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from liteiclink.serdes.gth_ultrascale import GTHChannelPLL, GTH3, GTH4

from test.test_serdes_xilinx import SerDesTop, _instances, _inputs

# Test GTH UltraScale(+) Channel PLL ---------------------------------------------------------------

class TestSerDesGTHUltraScale(unittest.TestCase):
    def test_channel_pll_reexport(self):
        # The GTHChannelPLL re-exported by gth_ultrascale is accepted by GTH3 and GTH4.
        for serdes_cls, channel in [(GTH3, "GTHE3_CHANNEL"), (GTH4, "GTHE4_CHANNEL")]:
            with self.subTest(serdes=serdes_cls.__name__):
                top      = SerDesTop(GTHChannelPLL, serdes_cls, 125e6, 2.5e9)
                fragment = top.get_fragment()
                gth,     = _instances(fragment, channel)
                self.assertTrue(any(expr is top.pll.refclk for expr in _inputs(gth)))

if __name__ == "__main__":
    unittest.main()
