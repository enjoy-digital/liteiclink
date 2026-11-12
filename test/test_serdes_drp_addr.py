#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from liteiclink.serdes.common import DRPControl

from test.test_serdes_xilinx import FAMILIES, SerDesTop, _instances, _input

# Test Xilinx Transceivers DRP Address Width -------------------------------------------------------

# Channel DRPADDR width (GTHE4_CHANNEL/GTYE4_CHANNEL: 10-bit, UG576/UG578).
DRP_ADDRESS_WIDTHS = {"gtp": 9, "gtx": 9, "gth2": 9, "gth3": 9, "gth4": 10, "gty": 10}

class TestSerDesDRPAddressWidth(unittest.TestCase):
    def test_channel_drp_address_width(self):
        for family, (device, channel, common, cpll_cls, qpll_cls, serdes_cls) in FAMILIES.items():
            with self.subTest(family=family):
                top      = SerDesTop(qpll_cls, serdes_cls, 125e6, 2.5e9)
                top.drp  = DRPControl(top.serdes.drp)
                fragment = top.get_fragment()
                gt,      = _instances(fragment, channel)
                aw       = DRP_ADDRESS_WIDTHS[family]
                self.assertEqual(len(top.serdes.drp.addr), aw)
                self.assertEqual(len(_input(gt, "DRPADDR")), aw)
                self.assertEqual(len(top.drp._addr.storage), aw)

if __name__ == "__main__":
    unittest.main()
