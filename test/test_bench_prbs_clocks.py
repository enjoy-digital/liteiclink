#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import io
import os
import unittest
import contextlib
import importlib.util
from types import SimpleNamespace

# Helpers ------------------------------------------------------------------------------------------

def load_test_prbs():
    path = os.path.join(os.path.dirname(__file__), "..", "bench", "serdes", "test_prbs.py")
    spec = importlib.util.spec_from_file_location("test_prbs_bench_clocks", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class CounterRegister:
    def __init__(self, values=()):
        self.values = list(values)

    def write(self, value):
        pass

    def read(self):
        return self.values.pop(0)

# Test PRBS Bench Clocks ---------------------------------------------------------------------------

class TestBenchPRBSClocks(unittest.TestCase):
    def measure(self, tx, rx):
        test_prbs = load_test_prbs()
        test_prbs.time = SimpleNamespace(sleep=lambda duration: None) # No wait (module local).
        regs = {
            "serdes0_clock_latch"     : CounterRegister(),
            "serdes0_clock_tx_cycles" : CounterRegister(tx),
            "serdes0_clock_rx_cycles" : CounterRegister(rx),
        }
        serdes = test_prbs.SerDes(SimpleNamespace(regs=SimpleNamespace(d=regs)), 0)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            serdes.measure_clocks()
        return output.getvalue()

    def test_stopped_clock(self):
        # Two equal counter reads: stopped clock (0 MHz), not a full 2^32 wrap.
        output = self.measure(tx=[1234, 1234], rx=[0, 0])
        self.assertIn("TX freq:      0.000MHz", output)
        self.assertIn("RX freq:      0.000MHz", output)

    def test_counter_wrap(self):
        # 250e6 cycles in 2 s across the 32-bit wrap: 125 MHz.
        start = 2**32 - 100e6
        output = self.measure(tx=[int(start), int(start + 250e6) % 2**32], rx=[0, int(250e6)])
        self.assertIn("TX freq:    125.000MHz", output)
        self.assertIn("RX freq:    125.000MHz", output)

if __name__ == "__main__":
    unittest.main()
