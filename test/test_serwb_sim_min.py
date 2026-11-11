#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
SerWB minimal Verilator simulation (bench/serwb/sim.py --min: the two generic PHYs only, counters
sent both ways and checked). Skipped when Verilator is not installed.
"""

import os
import sys
import shutil
import unittest
import tempfile
import subprocess

SIM = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb", "sim.py")

@unittest.skipUnless(shutil.which("verilator"), "Verilator not installed.")
class TestSERWBSimMin(unittest.TestCase):
    def test_sim_min(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = subprocess.run([sys.executable, os.path.abspath(SIM), "--min", "--output-dir", tmp],
                cwd     = tmp,
                stdout  = subprocess.PIPE,
                stderr  = subprocess.STDOUT,
                text    = True,
                timeout = 600,
            )
        lines = p.stdout.splitlines()
        self.assertTrue(any("MASTER entering READY" in l for l in lines), p.stdout[-2000:])
        self.assertTrue(any("SLAVE  entering READY" in l for l in lines), p.stdout[-2000:])
        for side in ["Master", "Slave"]:
            self.assertTrue(any(f"{side} check started" in l for l in lines), p.stdout[-2000:])
        errors = [l for l in lines if "check error" in l]
        self.assertEqual(errors, [])

if __name__ == "__main__":
    unittest.main()
