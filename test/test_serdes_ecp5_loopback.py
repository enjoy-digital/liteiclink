#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *

from liteiclink.serdes.serdes_ecp5 import SerDesECP5

# Loopback Control ---------------------------------------------------------------------------------

class PLL:
    refclk = Signal()
    config = {"m": 20, "d": 1, "linerate": 3e9}

class Pads:
    def __init__(self):
        self.p = Signal()
        self.n = Signal()

class TestSerDesECP5Loopback(unittest.TestCase):
    def test_loopback_modes(self):
        # The loopback register takes the 3-bit encoding of the other wrappers: the Near-End PMA
        # Loopback mode (0b010) written by test_prbs.py --loopback must request the DCU loopback.
        serdes = SerDesECP5(PLL(), Pads(), Pads())
        serdes.add_loopback_control()
        serdes.do_finalize = lambda: None # No DCUA instance in simulation.
        results = {}
        def generator():
            for value in [0b000, 0b001, 0b010, 0b100, 0b110, 0b000]:
                yield from serdes._loopback.write(value)
                yield
                results[value] = (yield serdes.loopback)
        clocks = {cd: 10 for cd in ["sys", "tx", "rx", "tx_full", "rx_full"]}
        run_simulation(serdes, generator(), clocks=clocks)
        self.assertEqual(results, {0b000: 0, 0b001: 1, 0b010: 1, 0b100: 1, 0b110: 1})

if __name__ == "__main__":
    unittest.main()
