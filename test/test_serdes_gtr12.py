#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen.fhdl.verilog import convert

from litex.build.gowin.common import gowin_special_overrides, gw5a_special_overrides

from liteiclink.serdes.gtr12_gw5 import GTR12, gtr12_quad_params


class Toolchain:
    def __init__(self):
        self.additional_tcl_commands = []

class Platform:
    def __init__(self):
        self.toolchain = Toolchain()


class TestGTR12(unittest.TestCase):
    def gtr12(self, **kwargs):
        platform = Platform()
        kwargs.setdefault("line_rate",  5e9)
        kwargs.setdefault("csr_config", "upar_write_driver(0x000000,0x00000000)\n")
        return platform, GTR12(platform, **kwargs)

    def test_quad_params_tie_offs(self):
        params = gtr12_quad_params("Q1")
        self.assertEqual(params["p_POSITION"], "Q1")
        for n in range(4):
            self.assertEqual(len(params[f"i_FABRIC_LN{n}_TXDATA_I"]), 80)
            self.assertEqual(params[f"i_FABRIC_LN{n}_TXDATA_I"].value, 0)

    def test_lane_wiring(self):
        for lane in range(2):
            with self.subTest(lane=lane):
                platform, gtr12 = self.gtr12(lane=lane, data_width=40)
                params = gtr12.gtr12_params
                self.assertIs(params[f"o_LANE{lane}_PCS_TX_O_FABRIC_CLK"], gtr12.cd_tx.clk)
                self.assertIs(params[f"o_LANE{lane}_PCS_RX_O_FABRIC_CLK"], gtr12.cd_rx.clk)
                self.assertIs(params[f"i_LANE{lane}_FABRIC_RX_CLK"],       gtr12.cd_rx.clk)
                self.assertEqual(len(params[f"i_FABRIC_LN{lane}_TXDATA_I"]), 80)
                self.assertEqual(gtr12.tx_clk_freq, 125e6)
                self.assertEqual(len(gtr12.decoders), 4)

    def test_pll_lock_selection(self):
        for pll, index in [("QPLL0", 0), ("QPLL1", 1), ("CPLL", 2)]:
            with self.subTest(pll=pll):
                platform, gtr12 = self.gtr12(pll=pll)
                assigns = [s for s in gtr12._fragment.comb if s.l is gtr12.pll_lock]
                self.assertEqual(len(assigns), 1)
                self.assertIs(assigns[0].r.value, gtr12.plls_lock)
                self.assertEqual(assigns[0].r.start, index)

    def test_csr_config(self):
        platform, gtr12 = self.gtr12(csr_name="test.csr")
        tcl = platform.toolchain.additional_tcl_commands
        self.assertIn("set_csr test.csr", tcl)
        self.assertTrue(any("upar_write_driver(0x000000,0x00000000)" in c for c in tcl))

    def test_upar_nets(self):
        # The quad AHB clock/reset and test decoder enable are the UPAR nets.
        platform, gtr12 = self.gtr12(with_upar=True)
        params = gtr12.gtr12_params
        self.assertIs(params["i_AHB_RSTN"],    gtr12.upar.ahb_rstn)
        self.assertIs(params["i_TEST_DEC_EN"], gtr12.upar.test_dec_en)

    def test_verilog(self):
        platform, gtr12 = self.gtr12()
        gtr12.add_stream_endpoints()
        gtr12.add_status_csr()
        top = Module()
        top.clock_domains.cd_sys = ClockDomain()
        top.submodules.gtr12 = gtr12
        v = str(convert(top, ios={top.cd_sys.clk, top.cd_sys.rst, gtr12.sink.valid, gtr12.source.valid},
            special_overrides={**gowin_special_overrides, **gw5a_special_overrides}))
        self.assertIn("GTR12_QUAD", v)
        self.assertIn("GTR12_UPAR", v)


if __name__ == "__main__":
    unittest.main()
