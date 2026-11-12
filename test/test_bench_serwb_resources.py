#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import shutil
import unittest
import importlib.util

from migen import *

from litex.gen import *

# Helpers ------------------------------------------------------------------------------------------

def load_resources():
    path = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb", "resources.py")
    spec = importlib.util.spec_from_file_location("serwb_resources", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class Counter(LiteXModule):
    def __init__(self):
        self.a      = Signal(4)
        self.q      = Signal(4)
        self.cd_sys = ClockDomain("sys")
        self.ios    = {self.a, self.q, self.cd_sys.clk, self.cd_sys.rst}
        self.sync += self.q.eq(self.q + self.a)

# Test SerWB Resources -----------------------------------------------------------------------------

class TestBenchSerWBResources(unittest.TestCase):
    @unittest.skipIf(shutil.which("yosys") is None, "Yosys not installed.")
    def test_synthesize(self):
        # Cell counts must be parsed whatever the Yosys stat format of the installed Yosys.
        resources = load_resources()
        results   = resources.synthesize(Counter(), "xc7a35ticsg324-1L")
        self.assertEqual(results["FF"], 4)
        self.assertGreater(results["LUT"], 0)

    def test_parse_stat_formats(self):
        resources = load_resources()
        old = "\n".join([
            "   Number of cells:                 19",
            "     CARRY4                          1",
            "     FDRE                            4",
            "     LUT2                            4",
        ])
        new = "\n".join([
            "       19 cells",
            "        1   CARRY4",
            "        4   FDRE",
            "        4   LUT2",
        ])
        for stat in [old, new]:
            results = resources.parse_stat(stat)
            self.assertEqual((results["FF"], results["LUT"], results["CARRY"]), (4, 4, 1))

if __name__ == "__main__":
    unittest.main()
