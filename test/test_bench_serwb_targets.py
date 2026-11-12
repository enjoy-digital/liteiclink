#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import unittest
import importlib.util
from unittest import mock

# Helpers ------------------------------------------------------------------------------------------

BENCH_DIR = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb")

def load_bench(name):
    spec = importlib.util.spec_from_file_location(f"serwb_target_{name}", os.path.join(BENCH_DIR, f"{name}.py"))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def _efinity_available():
    # Efinix platforms need the Efinity toolchain to elaborate (litex/build/efinix/toolchain.py).
    try:
        from litex.build.efinix.toolchain import find_efinity_path
        find_efinity_path()
        return True
    except Exception:
        return False


class _Built(Exception):
    pass

# Test SerWB Bench Targets -------------------------------------------------------------------------

class TestBenchSerWBTargets(unittest.TestCase):
    @unittest.skipUnless(_efinity_available(), "Efinity toolchain not available")
    def test_t120_lvds_delays(self):
        # --lvds-rx-delay/--lvds-clk-delay must reach the LVDS RX blocks of the PHYs.
        t120     = load_bench("t120_bga576_dev_kit")
        platform = t120.efinix_t120_f576_dev_kit.Platform()
        platform.add_extension(t120.serwb_io)
        t120.SerWBTestSoC(platform, lvds_rx_delay=3, lvds_clk_delay=5)
        delays = {b["name"]: b["delay"] for b in platform.toolchain.ifacewriter.blocks if "delay" in b}
        self.assertEqual(delays, {
            "serwb_master0_rx_p0" : 3,
            "serwb_slave0_rx_p0"  : 3,
            "serwb_slave0_clk_p0" : 5,
        })

    @unittest.skipUnless(_efinity_available(), "Efinity toolchain not available")
    def test_ti60_serio_domain(self):
        # The slave SerIO must run in rx_sys, like the slave core it is connected to.
        ti60     = load_bench("ti60_f225_dev_kit")
        platform = ti60.efinix_ti60_f225_dev_kit.Platform()
        platform.add_extension(ti60.serwb_io)
        soc = ti60.SerWBTestSoC(platform)
        self.assertEqual(set(soc.serio_slave.get_fragment().sync.keys()), {"rx_sys"})

    def test_csr_data_width(self):
        for name in ["t120_bga576_dev_kit", "ti60_f225_dev_kit"]:
            with self.subTest(target=name):
                self.assertNotIn("csr_data_with ", open(os.path.join(BENCH_DIR, f"{name}.py")).read())

    def test_icebreaker_loopback(self):
        # --loopback must be passed to the design.
        icebreaker = load_bench("icebreaker")
        kwargs     = {}
        def soc(platform, **_kwargs):
            kwargs.update(_kwargs)
            raise _Built()
        with mock.patch.object(icebreaker, "SerWBTestSoC", soc):
            with mock.patch.object(sys, "argv", ["icebreaker.py", "--loopback"]):
                with self.assertRaises(_Built):
                    icebreaker.main()
        self.assertEqual(kwargs.get("loopback"), True)

if __name__ == "__main__":
    unittest.main()
