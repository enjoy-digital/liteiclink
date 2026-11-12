#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import ast
import unittest
import importlib.util

# Helpers ------------------------------------------------------------------------------------------

BENCH_DIR = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb")

def load_bench(name):
    spec = importlib.util.spec_from_file_location(f"serwb_bench_{name}", os.path.join(BENCH_DIR, f"{name}.py"))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

# Test SerWB Bench Region --------------------------------------------------------------------------

class TestBenchSerWBRegion(unittest.TestCase):
    def test_sim_region_uncached(self):
        # The SerWB region maps remote registers: it must not be cached.
        sim = load_bench("sim")
        soc = sim.SerWBSoC()
        soc.finalize()
        self.assertFalse(soc.bus.regions["serwb"].cached)

    def test_targets_region_uncached(self):
        # Every SerWB bench target adds the SerWB slave with SoCRegion(..., cached=False).
        names = ["icebreaker", "nexys_video", "sim", "t120_bga576_dev_kit", "ti60_f225_dev_kit",
            "trellisboard", "ulx3s"]
        for name in names:
            with self.subTest(target=name):
                tree  = ast.parse(open(os.path.join(BENCH_DIR, f"{name}.py")).read())
                calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and
                    getattr(n.func, "attr", None) == "add_slave" and
                    isinstance(n.args[0], ast.Constant) and n.args[0].value == "serwb"]
                self.assertEqual(len(calls), 1)
                region = calls[0].args[2]
                cached = {k.arg: k.value.value for k in region.keywords}.get("cached", True)
                self.assertFalse(cached)

if __name__ == "__main__":
    unittest.main()
