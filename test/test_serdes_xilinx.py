#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Xilinx transceivers elaboration tests: PLL + transceiver + stream endpoints + CSRs are built and
converted to Verilog for each family, PLL and main options (catches elaboration regressions).
"""

import unittest

from migen import *
from migen.fhdl.specials  import Instance
from migen.fhdl.structure import _Assign

from litex.gen import *

from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect.csr import CSRStatus

from liteiclink.serdes.gtp_7series     import GTPQuadPLL, GTP
from liteiclink.serdes.gtx_7series     import GTXChannelPLL, GTXQuadPLL, GTX
from liteiclink.serdes.gth_7series     import GTHChannelPLL as GTH2ChannelPLL, GTHQuadPLL as GTH2QuadPLL, GTH as GTH2
from liteiclink.serdes.gth3_ultrascale import GTHChannelPLL as GTH3ChannelPLL, GTH3QuadPLL, GTH3
from liteiclink.serdes.gth4_ultrascale import GTHChannelPLL as GTH4ChannelPLL, GTH4QuadPLL, GTH4
from liteiclink.serdes.gty_ultrascale  import GTYChannelPLL, GTYQuadPLL, GTY

# Families -----------------------------------------------------------------------------------------

FAMILIES = {
    # name : (device, channel primitive, common primitive, CPLL, QPLL, Transceiver).
    "gtp"  : ("xc7a200t-sbg484-1",    "GTPE2_CHANNEL", "GTPE2_COMMON", None,           GTPQuadPLL,  GTP),
    "gtx"  : ("xc7k325t-ffg900-2",    "GTXE2_CHANNEL", "GTXE2_COMMON", GTXChannelPLL,  GTXQuadPLL,  GTX),
    "gth2" : ("xc7vx690t-ffg1761-2",  "GTHE2_CHANNEL", "GTHE2_COMMON", GTH2ChannelPLL, GTH2QuadPLL, GTH2),
    "gth3" : ("xcku040-ffva1156-2-e", "GTHE3_CHANNEL", "GTHE3_COMMON", GTH3ChannelPLL, GTH3QuadPLL, GTH3),
    "gth4" : ("xczu9eg-ffvb1156-2-e", "GTHE4_CHANNEL", "GTHE4_COMMON", GTH4ChannelPLL, GTH4QuadPLL, GTH4),
    "gty"  : ("xcvu9p-fsgd2104-2l-e", "GTYE4_CHANNEL", "GTYE4_COMMON", GTYChannelPLL,  GTYQuadPLL,  GTY),
}

# Configurations: (pll, refclk_freq, linerate, transceiver kwargs). Without TX/RX buffer, the
# transceiver clock is derived from the reference clock (refclk_freq/(linerate/data_width) integer).
CONFIGS = {
    "default"       : ("cpll", 125e6,    2.5e9,  {}),
    "qpll"          : ("qpll", 125e6,    2.5e9,  {}),
    "data_width_40" : ("cpll", 125e6,    5e9,    {"data_width": 40}),
    "qpll_dw40"     : ("qpll", 156.25e6, 6.25e9, {"data_width": 40}),
    "buffers"       : ("cpll", 125e6,    2.5e9,  {"tx_buffer_enable": True, "rx_buffer_enable": True}),
    "no_aligner"    : ("cpll", 125e6,    2.5e9,  {"clock_aligner": False}),
    "polarities"    : ("cpll", 125e6,    2.5e9,  {"tx_polarity": 1, "rx_polarity": 1}),
}

# Expected CSRs (add_controls/add_clock_cycles).
CSRS = [
    "tx_enable", "tx_ready", "tx_inhibit", "tx_produce_square_wave",
    "rx_enable", "rx_ready",
    "tx_prbs_config", "rx_prbs_config", "rx_prbs_errors",
    "loopback",
    "tx_polarity", "rx_polarity",
    "tx_diffctrl", "tx_postcursor", "tx_precursor",
    "clock_latch", "clock_tx_cycles", "clock_rx_cycles",
]

# Design -------------------------------------------------------------------------------------------

def _pads():
    return Record([("p", 1), ("n", 1)])

class SerDesTop(LiteXModule):
    def __init__(self, pll_cls, serdes_cls, refclk_freq, linerate, external_clks=False, **kwargs):
        self.cd_sys = ClockDomain()
        self.refclk = refclk = Signal()
        self.pll    = pll    = pll_cls(refclk, refclk_freq, linerate)
        clks = {}
        if external_clks:
            clks = {"tx_clk": Signal(), "rx_clk": Signal()}
        self.serdes = serdes = serdes_cls(pll, _pads(), _pads(), sys_clk_freq=100e6, **clks, **kwargs)
        serdes.add_stream_endpoints()
        serdes.add_controls()
        serdes.add_clock_cycles()

def _instances(fragment, of):
    return [s for s in fragment.specials if isinstance(s, Instance) and s.of == of]

def _parameter(instance, name):
    for item in instance.items:
        if isinstance(item, Instance.Parameter) and item.name == name:
            return item.value.value if hasattr(item.value, "value") else item.value
    raise KeyError(name)

def _inputs(instance):
    return [item.expr for item in instance.items if isinstance(item, Instance.Input)]

def _input(instance, name):
    for item in instance.items:
        if isinstance(item, Instance.Input) and item.name == name:
            return item.expr
    raise KeyError(name)

# Test Xilinx Transceivers -------------------------------------------------------------------------

class TestSerDesXilinx(unittest.TestCase):
    def elaborate(self, family, pll, refclk_freq, linerate, **kwargs):
        device, channel, common, cpll_cls, qpll_cls, serdes_cls = FAMILIES[family]
        pll_cls = {"cpll": cpll_cls, "qpll": qpll_cls}[pll]
        if pll_cls is None:
            pll_cls = qpll_cls # GTP: QPLL only.
        top      = SerDesTop(pll_cls, serdes_cls, refclk_freq, linerate, **kwargs)
        csrs     = [csr.name for csr in top.serdes.get_csrs()]
        fragment = top.get_fragment()
        verilog  = str(XilinxPlatform(device, io=[]).get_verilog(fragment))
        return top, fragment, csrs, verilog

    def check(self, family, pll, refclk_freq, linerate, **kwargs):
        device, channel, common, cpll_cls, qpll_cls, serdes_cls = FAMILIES[family]
        top, fragment, csrs, verilog = self.elaborate(family, pll, refclk_freq, linerate, **kwargs)
        data_width = kwargs.get("data_width", 20)

        # One channel, and one common when using the QPLL (GTP: always).
        channels = _instances(fragment, channel)
        self.assertEqual(len(channels), 1)
        uses_qpll = (pll == "qpll") or (cpll_cls is None)
        self.assertEqual(len(_instances(fragment, common)), 1 if uses_qpll else 0)
        self.assertIn(channel, verilog)

        # Channel clocked from the selected PLL.
        pll_clk = top.pll.clk if uses_qpll else top.pll.refclk
        self.assertTrue(any(expr is pll_clk for expr in _inputs(channels[0])))

        # Data widths and user clocks.
        self.assertEqual(_parameter(channels[0], "TX_DATA_WIDTH"), data_width)
        self.assertEqual(_parameter(channels[0], "RX_DATA_WIDTH"), data_width)
        self.assertEqual(top.serdes.tx_clk_freq, linerate/data_width)
        self.assertEqual(top.serdes.rx_clk_freq, linerate/data_width)
        self.assertEqual(len(top.serdes.sink.data),   8*data_width//10)
        self.assertEqual(len(top.serdes.source.data), 8*data_width//10)

        # Controls.
        for csr in CSRS:
            self.assertIn(csr, csrs)
        self.assertEqual("clock_aligner_disable" in csrs, kwargs.get("clock_aligner", True))
        # Clock cycles: latched by hardware, read-only.
        self.assertIsInstance(top.serdes.clock_tx_cycles, CSRStatus)
        self.assertIsInstance(top.serdes.clock_rx_cycles, CSRStatus)

    def test_configs(self):
        for family in FAMILIES:
            for name, (pll, refclk_freq, linerate, kwargs) in CONFIGS.items():
                with self.subTest(family=family, config=name):
                    self.check(family, pll, refclk_freq, linerate, **kwargs)

    def test_external_clks(self):
        # tx_clk/rx_clk provided by the user (requires TX/RX buffers): transceiver must not generate
        # its own user clocks.
        for family in FAMILIES:
            with self.subTest(family=family):
                top, fragment, csrs, verilog = self.elaborate(family, "cpll", 125e6, 2.5e9,
                    external_clks    = True,
                    tx_buffer_enable = True,
                    rx_buffer_enable = True,
                )
                for buf in ["BUFG", "BUFG_GT", "BUFR"]:
                    self.assertEqual(_instances(fragment, buf), [])
        # Without buffers, external clocks are rejected.
        with self.assertRaises(AssertionError):
            self.elaborate("gtx", "cpll", 125e6, 2.5e9, external_clks=True)

    def test_7series_tx_clock_dividers(self):
        # TX user clock derived from the reference clock: BUFG (/1), BUFR (integer) or PLL
        # (non-integer).
        for family in ["gtp", "gtx", "gth2"]:
            for linerate, data_width, primitive in [
                (2.5e9,  20, None),
                (1.25e9, 20, "BUFR"),
                (5e9,    20, "PLLE2_ADV"),
            ]:
                with self.subTest(family=family, linerate=linerate):
                    top, fragment, csrs, verilog = self.elaborate(family, "cpll", 125e6, linerate,
                        data_width=data_width)
                    for buf in ["BUFR", "PLLE2_ADV"]:
                        self.assertEqual(len(_instances(fragment, buf)), int(buf == primitive))

    def test_gtp_refclk_from_fabric(self):
        # Reference clock from the fabric: GTGREFCLK0/1 (PLL0/1REFCLKSEL=0b111) instead of GTREFCLK0.
        for channel in [0, 1]:
            for refclk_from_fabric in [False, True]:
                with self.subTest(channel=channel, refclk_from_fabric=refclk_from_fabric):
                    refclk   = Signal()
                    pll      = GTPQuadPLL(refclk, 100e6, 5e9, channel=channel, refclk_from_fabric=refclk_from_fabric)
                    common,  = _instances(pll.get_fragment(), "GTPE2_COMMON")
                    gtrefclk  = _input(common, "GTREFCLK0")
                    gtgrefclk = _input(common, f"GTGREFCLK{channel}")
                    refclksel = _input(common, f"PLL{channel}REFCLKSEL")
                    self.assertIs(gtgrefclk if refclk_from_fabric else gtrefclk, refclk)
                    self.assertEqual((gtrefclk if refclk_from_fabric else gtgrefclk).value, 0)
                    self.assertEqual(refclksel.value, 0b111 if refclk_from_fabric else 0b001)

    def test_gty_raw_and_rate(self):
        # Raw RX symbols output, raw TX symbols input (tx_raw_enable) and TXRATE/RXRATE controls.
        top, fragment, csrs, verilog = self.elaborate("gty", "qpll", 156.25e6, 10e9, data_width=40,
            tx_buffer_enable=True, rx_buffer_enable=True)
        serdes  = top.serdes
        channel, = _instances(fragment, "GTYE4_CHANNEL")
        self.assertEqual(len(serdes.rx_raw), 40)
        self.assertEqual(len(serdes.tx_raw), 40)
        for name in ["TXRATE", "RXRATE"]:
            self.assertIsInstance(_input(channel, name), Signal)
        self.assertIn("tx_raw_enable", verilog)

    def test_gtp_raw(self):
        # Raw RX symbols output and raw TX symbols input (tx_raw_enable), same API as the GTY.
        for data_width in [20, 40]:
            with self.subTest(data_width=data_width):
                top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 125e6, 2.5e9,
                    data_width=data_width, clock_aligner=False)
                serdes = top.serdes
                self.assertEqual(len(serdes.rx_raw), data_width)
                self.assertEqual(len(serdes.tx_raw), data_width)
                self.assertEqual(len(_instances(fragment, "GTPE2_CHANNEL")), 1)
                self.assertIn("tx_raw_enable", verilog)

    def test_ultrascale_unaligned_tx_clock_rejected(self):
        # UltraScale(+): TX user clock (without TX buffer) must be an integer division of the
        # reference clock (BUFG_GT divider).
        for family in ["gth3", "gth4", "gty"]:
            with self.subTest(family=family):
                with self.assertRaises(AssertionError):
                    self.elaborate(family, "qpll", 156.25e6, 10.3125e9, data_width=40)

    def test_ultrascale_tx_outclk_sel(self):
        # Without TX buffer, TXOUTCLK is the reference clock (TXPLLREFCLK_DIV1) since the BUFG_GT
        # divider is computed from it (GTY used TXPROGDIVCLK with TX_PROGDIV_CFG=0); TXOUTCLKPMA
        # (BUFG_GT /1) with TX buffer.
        for family, channel in [("gth3", "GTHE3_CHANNEL"), ("gth4", "GTHE4_CHANNEL"), ("gty", "GTYE4_CHANNEL")]:
            for tx_buffer_enable in [False, True]:
                with self.subTest(family=family, tx_buffer_enable=tx_buffer_enable):
                    top, fragment, csrs, verilog = self.elaborate(family, "qpll", 156.25e6, 6.25e9,
                        data_width=40, tx_buffer_enable=tx_buffer_enable)
                    gt,      = _instances(fragment, channel)
                    bufg_gts = [b for b in _instances(fragment, "BUFG_GT") if _input(b, "I") is top.serdes.txoutclk]
                    self.assertEqual(len(bufg_gts), 1)
                    self.assertEqual(_input(gt, "TXOUTCLKSEL").value, 0b010 if tx_buffer_enable else 0b011)
                    div = 1 if tx_buffer_enable else int(156.25e6/top.serdes.tx_clk_freq)
                    self.assertEqual(_input(bufg_gts[0], "DIV").value, div - 1)

    def test_gtp_user_clocks(self):
        # GTP internal datapath is 2-byte: TXUSRCLK/RXUSRCLK at linerate/20, TXUSRCLK2/RXUSRCLK2
        # ("tx"/"rx") at linerate/data_width (UG482: 2:1 with the 4-byte interface, data_width=40).
        for data_width in [20, 40]:
            for buffers in [False, True]:
                with self.subTest(data_width=data_width, buffers=buffers):
                    top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 125e6, 2.5e9,
                        data_width       = data_width,
                        tx_buffer_enable = buffers,
                        rx_buffer_enable = buffers)
                    serdes = top.serdes
                    gt,    = _instances(fragment, "GTPE2_CHANNEL")
                    for d in ["tx", "rx"]:
                        usrclk  = _input(gt, f"{d.upper()}USRCLK")
                        usrclk2 = _input(gt, f"{d.upper()}USRCLK2")
                        self.assertIs(usrclk, getattr(serdes, f"{d}_usrclk"))
                        self.assertIs(usrclk2, getattr(serdes, f"cd_{d}").clk)
                        # USRCLK driver: the domain clock (20-bit) or a PLL output (40-bit).
                        drivers = [st.r for st in fragment.comb if isinstance(st, _Assign) and st.l is usrclk]
                        self.assertEqual(len(drivers), 1)
                        is_usrclk2 = isinstance(drivers[0], ClockSignal) and drivers[0].cd == d
                        self.assertEqual(is_usrclk2, data_width == 20)
                        if data_width == 40:
                            pll = getattr(serdes, f"{d}_usrclk_pll")
                            freqs = [pll.clkouts[n].freq for n in sorted(pll.clkouts)]
                            self.assertEqual(freqs, [2.5e9/20, 2.5e9/40])
                    # PLLE2: CLKOUT1 (USRCLK2) divider twice CLKOUT0 (USRCLK) divider.
                    plls = _instances(fragment, "PLLE2_ADV")
                    self.assertEqual(len(plls), 2 if data_width == 40 else 0)
                    for pll in plls:
                        self.assertEqual(_parameter(pll, "CLKOUT1_DIVIDE"), 2*_parameter(pll, "CLKOUT0_DIVIDE"))

    def test_gtp_external_user_clocks(self):
        # External tx_clk/rx_clk with the 4-byte interface: TXUSRCLK/RXUSRCLK from tx_usrclk/rx_usrclk
        # (linerate/20), tx_clk/rx_clk when not provided (previous behavior).
        for with_usrclk in [False, True]:
            with self.subTest(with_usrclk=with_usrclk):
                usrclks = {"tx_usrclk": Signal(), "rx_usrclk": Signal()} if with_usrclk else {}
                top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 125e6, 2.5e9,
                    data_width       = 40,
                    external_clks    = True,
                    tx_buffer_enable = True,
                    rx_buffer_enable = True,
                    **usrclks)
                self.assertEqual(_instances(fragment, "PLLE2_ADV"), [])
                for d in ["tx", "rx"]:
                    drivers = [st.r for st in fragment.comb if isinstance(st, _Assign) and st.l is getattr(top.serdes, f"{d}_usrclk")]
                    if with_usrclk:
                        self.assertIs(drivers[0], usrclks[f"{d}_usrclk"])
                    else:
                        self.assertEqual(drivers[0].cd, d)

if __name__ == "__main__":
    unittest.main()
