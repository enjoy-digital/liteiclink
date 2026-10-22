#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Xilinx transceivers PLL configuration tests: the dividers returned by compute_config must give the
requested linerate with the VCO in its valid range, for common reference clocks/linerates.
"""

import unittest

from liteiclink.serdes.gtp_7series     import GTPQuadPLL
from liteiclink.serdes.gtx_7series     import GTXChannelPLL, GTXQuadPLL
from liteiclink.serdes.gth_7series     import GTHChannelPLL as GTH2ChannelPLL, GTHQuadPLL as GTH2QuadPLL
from liteiclink.serdes.gth3_ultrascale import GTHChannelPLL as GTH3ChannelPLL, GTH3QuadPLL
from liteiclink.serdes.gth4_ultrascale import GTHChannelPLL as GTH4ChannelPLL, GTH4QuadPLL
from liteiclink.serdes.gty_ultrascale  import GTYChannelPLL, GTYQuadPLL

# Helpers ------------------------------------------------------------------------------------------

def cpll_linerate(config):
    # VCO = CLKIN x (N1 x N2) / M, LINERATE = VCO x 2 / D.
    vco = config["clkin"]*config["n1"]*config["n2"]/config["m"]
    return vco, vco*2/config["d"]

def qpll_linerate(config):
    # VCO = CLKIN x N / M, CLKOUT = VCO / 2, LINERATE = CLKOUT x 2 / D.
    vco = config["clkin"]*config["n"]/config["m"]
    return vco, (vco/2)*2/config["d"]

def gty_qpll_linerate(config):
    # VCO = CLKIN x (N + F) / M, CLKOUT = VCO / RATE, LINERATE = CLKOUT x 2 / D.
    vco = config["clkin"]*(config["n"] + config["f"])/config["m"]
    return vco, (vco/config["clkout_rate"])*2/config["d"]

# PLLs: (class, linerate function, VCO ranges, (refclk_freq, linerate) cases).
PLLS = {
    "GTP QPLL"  : (GTPQuadPLL,     cpll_linerate,     [(1.6e9, 3.3e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTX CPLL"  : (GTXChannelPLL,  cpll_linerate,     [(1.6e9, 3.3e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTX QPLL"  : (GTXQuadPLL,     qpll_linerate,     [(5.93e9, 8e9), (9.8e9, 12.5e9)],
        [(125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9), (156.25e6, 10.3125e9)]),
    "GTH2 CPLL" : (GTH2ChannelPLL, cpll_linerate,     [(2.0e9, 6.25e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTH2 QPLL" : (GTH2QuadPLL,    qpll_linerate,     [(5.93e9, 8e9), (9.8e9, 12.5e9)],
        [(125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9), (156.25e6, 10.3125e9)]),
    "GTH3 CPLL" : (GTH3ChannelPLL, cpll_linerate,     [(2.0e9, 6.25e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTH3 QPLL" : (GTH3QuadPLL,    qpll_linerate,     [(8e9, 13e9), (9.8e9, 16.375e9)],
        [(125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9), (156.25e6, 10.3125e9)]),
    "GTH4 CPLL" : (GTH4ChannelPLL, cpll_linerate,     [(2.0e9, 6.25e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTH4 QPLL" : (GTH4QuadPLL,    qpll_linerate,     [(8e9, 13e9), (9.8e9, 16.375e9)],
        [(125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9), (156.25e6, 10.3125e9)]),
    "GTY CPLL"  : (GTYChannelPLL,  cpll_linerate,     [(2.0e9, 6.25e9)],
        [(100e6, 2.5e9), (125e6, 1.25e9), (125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9)]),
    "GTY QPLL"  : (GTYQuadPLL,     gty_qpll_linerate, [(8e9, 13e9), (9.8e9, 16.375e9)],
        [(125e6, 2.5e9), (125e6, 5e9), (156.25e6, 3.125e9), (156.25e6, 6.25e9), (156.25e6, 10.3125e9),
         (156.25e6, 25.78125e9)]),
}

# Test PLLs ----------------------------------------------------------------------------------------

class TestSerDesPLL(unittest.TestCase):
    def test_configs(self):
        for name, (cls, linerate_fn, vco_ranges, cases) in PLLS.items():
            for refclk_freq, linerate in cases:
                with self.subTest(pll=name, refclk=refclk_freq, linerate=linerate):
                    config = cls.compute_config(refclk_freq, linerate)
                    vco, computed_linerate = linerate_fn(config)
                    self.assertEqual(config["clkin"], refclk_freq)
                    self.assertAlmostEqual(computed_linerate, linerate, delta=1)
                    self.assertAlmostEqual(config["linerate"], linerate, delta=1)
                    self.assertAlmostEqual(config["vco_freq"], vco, delta=1)
                    self.assertTrue(any(vmin <= vco <= vmax for vmin, vmax in vco_ranges),
                        f"VCO {vco/1e9:.3f}GHz out of range.")

    def test_impossible_linerate_rejected(self):
        for name, (cls, linerate_fn, vco_ranges, cases) in PLLS.items():
            with self.subTest(pll=name):
                with self.assertRaises(ValueError):
                    cls.compute_config(125e6, 40e9)

    def test_repr(self):
        # __repr__ (config summary printed by user designs) must work for every PLL.
        for name, (cls, linerate_fn, vco_ranges, cases) in PLLS.items():
            with self.subTest(pll=name):
                refclk_freq, linerate = cases[0]
                pll = cls(None, refclk_freq, linerate)
                self.assertIn(f"{linerate/1e9}", repr(pll))

if __name__ == "__main__":
    unittest.main()
