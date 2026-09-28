#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Instance, Signal

from liteiclink.serdes.gty_ultrascale import GTYQuadPLL


def parameters(pll):
    common = next(s for s in pll.get_fragment().specials
        if isinstance(s, Instance) and s.of == "GTYE4_COMMON")
    return {p.name: p.value.value if hasattr(p.value, "value") else p.value
        for p in common.items if isinstance(p, Instance.Parameter)}


class TestGTYQPLLConfiguration(unittest.TestCase):
    def test_automatic_selection_keeps_existing_preference(self):
        for linerate, divider in ((5.15625e9, 2), (10.3125e9, 1)):
            config = GTYQuadPLL.compute_config(156.25e6, linerate)
            self.assertEqual(config["qpll"], "qpll1")
            self.assertEqual(config["n"], 66)
            self.assertEqual(config["d"], divider)
            self.assertEqual(config["f"], 0)

    def test_explicit_qpll_selection_and_fractional_feedback(self):
        for refclk, fractional in ((156.25e6, 0.5), (161.1328125e6, 0)):
            for selected in ("qpll0", "qpll1"):
                with self.subTest(refclk=refclk, qpll=selected):
                    pll = GTYQuadPLL(Signal(), refclk, 25.78125e9, qpll=selected)
                    self.assertEqual(pll.config["qpll"], selected)
                    self.assertEqual(pll.config["f"], fractional)
                    self.assertEqual(pll.config["linerate"], 25.78125e9)
                    self.assertEqual(pll.config["clkout_rate"], 1)
                    params = parameters(pll)
                    sdm = params[selected.upper() + "_SDM_CFG0"]
                    if fractional:
                        self.assertEqual(sdm & (1 << 7), 0)
                        self.assertEqual(pll.sdm0_data.reset.value, 1 << 23)
                    elif selected == "qpll0":
                        self.assertEqual(sdm & (1 << 7), 1 << 7)

    def test_selection_rejects_unavailable_vco(self):
        # Full-rate VCO is 15 GHz: QPLL0 can provide it, QPLL1 cannot.
        self.assertEqual(GTYQuadPLL.compute_config(150e6, 30e9, qpll="qpll0")["qpll"], "qpll0")
        with self.assertRaises(ValueError):
            GTYQuadPLL.compute_config(150e6, 30e9, qpll="qpll1")
        with self.assertRaises(ValueError):
            GTYQuadPLL.compute_config(156.25e6, 10.3125e9, qpll="qpll2")

    def test_tuning_applies_before_instantiation_without_changing_input(self):
        tuning = {"PPF0_CFG": 0x200, "QPLL0_CFG4": 0x80}
        pll = GTYQuadPLL(Signal(), 156.25e6, 25.78125e9, qpll="qpll0", qpll_params=tuning)
        params = parameters(pll)
        self.assertEqual(params["PPF0_CFG"], 0x200)
        self.assertEqual(params["QPLL0_CFG4"], 0x80)
        self.assertEqual(tuning, {"PPF0_CFG": 0x200, "QPLL0_CFG4": 0x80})

    def test_tuning_cannot_override_solver_or_ports(self):
        for name in ("QPLL0_FBDIV", "QPLL1_REFCLK_DIV", "QPLL0CLKOUT_RATE",
            "QPLL0_SDM_CFG0", "GTREFCLK00", "UNKNOWN"):
            with self.subTest(parameter=name), self.assertRaises(ValueError):
                GTYQuadPLL(Signal(), 156.25e6, 10.3125e9, qpll_params={name: 0})

    def test_existing_compute_config_subclass(self):
        class QPLL0(GTYQuadPLL):
            @staticmethod
            def compute_config(refclk_freq, linerate):
                config = GTYQuadPLL.compute_config(refclk_freq, linerate)
                config["qpll"] = "qpll0"
                return config

        pll = QPLL0(Signal(), 156.25e6, 25.78125e9)
        self.assertEqual(pll.config["qpll"], "qpll0")
        self.assertEqual(parameters(pll)["QPLL0_SDM_CFG0"], 0)
