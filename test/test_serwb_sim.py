#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
SerWB end-to-end Verilator simulations (bit-level generic PHY: link init/alignment + traffic).

Uses the self-checking SerWBTestSoC of bench/serwb/sim.py. Skipped when Verilator is not installed.
"""

import os
import sys
import shutil
import unittest
import tempfile
import subprocess

SIM = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb", "sim.py")

CONFIGS = {
    "etherbone"             : [],
    "etherbone_fc_crc"      : ["--with-flow-control", "--with-crc"],
    "etherbone_fc_slow"     : ["--with-flow-control", "--sram-latency", "64"],
    "native"                : ["--protocol", "native"],
    "native_fc_crc"         : ["--protocol", "native", "--with-flow-control", "--with-crc"],
    "native_slow"           : ["--protocol", "native", "--sram-latency", "64"],
}

@unittest.skipUnless(shutil.which("verilator"), "Verilator not installed.")
class TestSERWBSim(unittest.TestCase):
    def run_sim(self, args):
        with tempfile.TemporaryDirectory() as tmp:
            p = subprocess.run([sys.executable, os.path.abspath(SIM), "--test", "--output-dir", tmp, *args],
                cwd            = tmp,
                stdout         = subprocess.PIPE,
                stderr         = subprocess.STDOUT,
                text           = True,
                timeout        = 600,
            )
        results = [l for l in p.stdout.splitlines() if "SERWB TEST" in l]
        return results[0] if results else p.stdout[-2000:]

    def test_sim(self):
        for name, args in CONFIGS.items():
            with self.subTest(config=name):
                result = self.run_sim(args)
                self.assertIn("SERWB TEST: PASS", result, f"{name}: {result}")

if __name__ == "__main__":
    unittest.main()
