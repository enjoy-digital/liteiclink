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
    spec = importlib.util.spec_from_file_location("test_prbs_bench", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class FakeRegister:
    def __init__(self, reads=None):
        self.writes = []
        self.reads  = list(reads or [])

    def write(self, value):
        self.writes.append(value)

    def read(self):
        return self.reads.pop(0) if self.reads else 0

# Test PRBS Bench ----------------------------------------------------------------------------------

class TestBenchPRBSBER(unittest.TestCase):
    def test_ber(self):
        # 4 intervals of 0.5s: first interval (1000 errors, before/at synchronization) not counted,
        # then 10 errors per interval: 30 errors over 1.5s at 2.5Gbps (default linerate).
        test_prbs = load_test_prbs()
        names = ["tx_produce_square_wave", "tx_prbs_config", "rx_prbs_config", "loopback"]
        regs  = {f"serdes0_{name}": FakeRegister() for name in names}
        regs["serdes0_rx_prbs_errors"] = FakeRegister(reads=[1000, 1010, 1020, 1030])
        class RemoteClient:
            def __init__(self, **kwargs):
                self.regs = SimpleNamespace(d=regs)
            def open(self):  pass
            def close(self): pass
        test_prbs.RemoteClient = RemoteClient
        test_prbs.time         = SimpleNamespace(sleep=lambda t: None)
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            test_prbs.prbs_test(mode="prbs31", duration=2)
        lines = [l for l in stdout.getvalue().splitlines() if l.startswith("Errors:")]
        self.assertEqual(len(lines), 4)
        self.assertIn("BER (lower bound): {:1.3e}".format(30/(1.5*2.5e9)), lines[-1])

if __name__ == "__main__":
    unittest.main()
