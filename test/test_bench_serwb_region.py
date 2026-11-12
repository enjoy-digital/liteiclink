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

    def test_ecpix5_demo_region_uncached(self):
        # The ECPIX-5 demo (VexRiscv) maps the iCEBreaker peripherals uncached: inside the CPU IO
        # region, aligned on its size (the iCEBreaker keeps the address offsets in its 512MB master
        # region), and the iCEBreaker register map is imported at the same origin.
        spec = importlib.util.spec_from_file_location("serwb_demo_ecpix5", os.path.join(BENCH_DIR, "demo", "ecpix5.py"))
        demo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(demo)
        from litex_boards.platforms import lambdaconcept_ecpix5
        platform = lambdaconcept_ecpix5.Platform(toolchain="trellis")
        platform.add_extension(demo.serwb_io)
        soc    = demo.SerWBDemoSoC(platform)
        region = soc.bus.regions["serwb"]
        self.assertFalse(region.cached)
        self.assertEqual(region.origin % region.size, 0)
        self.assertTrue(any(region.origin >= o and region.origin + region.size <= o + s
            for o, s in soc.cpu.io_regions.items()))
        tree  = ast.parse(open(os.path.join(BENCH_DIR, "demo", "ecpix5.py")).read())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "attr", None) == "add_json"]
        self.assertEqual(len(calls), 1)
        self.assertEqual(ast.unparse(calls[0].args[1]), "serwb_origin")
        self.assertEqual(demo.serwb_origin, region.origin)

if __name__ == "__main__":
    unittest.main()
