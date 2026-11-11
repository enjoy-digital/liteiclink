#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from liteiclink.serwb.phy       import SERWBPHY
from liteiclink.serwb.s7serdes  import S7Serdes
from liteiclink.serwb.kuserdes  import KUSerdes

# Test SerWB PHY -----------------------------------------------------------------------------------

class TestSERWBPHY(unittest.TestCase):
    def test_device_serdes(self):
        # SerDes selected from the device name (7-Series or UltraScale(+), Kintex/Virtex/Zynq).
        pads = Record([(name, 1) for name in ["clk_p", "clk_n", "tx_p", "tx_n", "rx_p", "rx_n"]])
        devices = {
            "xc7a35ticsg324-1L"   : S7Serdes,
            "xc7k325t-ffg900-2"   : S7Serdes,
            "xcku040-ffva1156-2-e": KUSerdes,
            "xcvu9p-fsgd2104-2L-e": KUSerdes,
            "xczu9eg-ffvb1156-2-e": KUSerdes,
        }
        for device, serdes_cls in devices.items():
            with self.subTest(device=device):
                phy = SERWBPHY(device, pads, mode="master")
                self.assertIsInstance(phy.serdes, serdes_cls)

if __name__ == "__main__":
    unittest.main()
