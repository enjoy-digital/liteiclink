#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.fhdl.tools import list_targets

from test.test_serdes_xilinx import FAMILIES, SerDesTop

# Test Xilinx Transceivers RX Reset ----------------------------------------------------------------

def _targets(fragment, domain, name):
    statements = fragment.sync.get(domain, [])
    return [s for s in list_targets(statements) if s.backtrace[-1][0] == name]

class TestSerDesXilinxRXReset(unittest.TestCase):
    def test_rx_reset_registered_in_sys(self):
        # The rx domain reset is registered in sys (like the tx one): a tx domain register would be
        # cleared by the tx domain reset (or frozen without TX clock) and release the rx domain
        # whatever the RX init says.
        for family, (device, channel, common, cpll_cls, qpll_cls, serdes_cls) in FAMILIES.items():
            with self.subTest(family=family):
                top      = SerDesTop(qpll_cls, serdes_cls, 125e6, 2.5e9)
                fragment = top.get_fragment()
                for name in ["tx_reset_deglitched", "rx_reset_deglitched"]:
                    self.assertEqual(len(_targets(fragment, "sys", name)), 1, name)
                    self.assertEqual(len(_targets(fragment, "tx",  name)), 0, name)

if __name__ == "__main__":
    unittest.main()
