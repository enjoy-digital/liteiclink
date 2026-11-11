#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest
import importlib.util
from types import SimpleNamespace

# Helpers ------------------------------------------------------------------------------------------

def load_test_prbs():
    path = os.path.join(os.path.dirname(__file__), "..", "bench", "serdes", "test_prbs.py")
    spec = importlib.util.spec_from_file_location("test_prbs_bench", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class FakeRegister:
    def __init__(self):
        self.writes = []

    def write(self, value):
        self.writes.append(value)

    def read(self):
        return 0

# Test PRBS Bench ----------------------------------------------------------------------------------

class TestBenchPRBS(unittest.TestCase):
    def test_clock_aligner_disable(self):
        # The clock aligner restarts the RX when no comma is received: it must be disabled during
        # the PRBS test (no commas) and enabled again after.
        test_prbs = load_test_prbs()
        names = ["clock_aligner_disable", "tx_produce_square_wave", "tx_prbs_config",
            "rx_prbs_config", "loopback"]
        regs  = {f"serdes0_{name}": FakeRegister() for name in names}
        wb    = SimpleNamespace(regs=SimpleNamespace(d=regs))
        serdes = test_prbs.SerDes(wb, 0)
        serdes.configure("prbs31", square_wave=False, loopback=False)
        self.assertEqual(regs["serdes0_clock_aligner_disable"].writes, [1])
        serdes.unconfigure()
        self.assertEqual(regs["serdes0_clock_aligner_disable"].writes, [1, 0])

if __name__ == "__main__":
    unittest.main()
