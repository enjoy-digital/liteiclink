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
from migen.fhdl.specials import Instance

from litex.gen import *

from litex.build.xilinx import XilinxPlatform

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

def _output(instance, name):
    for item in instance.items:
        if isinstance(item, Instance.Output) and item.name == name:
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

    def test_gtp_pcie_options(self):
        # GTP options/ports for protocols with Electrical Idle, rate changes and PIPE-style drivers
        # (PCIe): comma alignment on any byte, RXCDR_CFG override, PIPE TX drive mode, PCS PCIe
        # mode, Electrical Idle detection with a fabric clock, raw symbols, TX/RX rates, receiver
        # detection, power-down and status ports.
        oob_clk   = Signal()
        rxcdr_cfg = 0x000107FE406001041010
        top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 100e6, 2.5e9,
            data_width         = 40,
            tx_buffer_enable   = True,
            rx_buffer_enable   = True,
            clock_aligner      = False,
            align_comma_word   = 1,
            rxcdr_cfg          = rxcdr_cfg,
            tx_drive_mode      = "PIPE",
            pcs_pcie_en        = True,
            rx_elecidle_detect = True,
            rx_elecidle_clk    = oob_clk,
        )
        serdes   = top.serdes
        channel, = _instances(fragment, "GTPE2_CHANNEL")
        self.assertEqual(_parameter(channel, "ALIGN_COMMA_WORD"), 1)
        self.assertEqual(_parameter(channel, "RXCDR_CFG"),        rxcdr_cfg)
        self.assertEqual(_parameter(channel, "TX_DRIVE_MODE"),    "PIPE")
        self.assertEqual(_parameter(channel, "TX_DEEMPH0"),       0b10100)
        self.assertEqual(_parameter(channel, "TX_DEEMPH1"),       0b01011)
        self.assertEqual(_parameter(channel, "PCS_PCIE_EN"),      "TRUE")
        self.assertEqual(_parameter(channel, "PCS_RSVD_ATTR"),    0x100)
        self.assertEqual(_parameter(channel, "RXOOB_CLK_CFG"),    "FABRIC")
        self.assertIs(_input(channel, "SIGVALIDCLK"), oob_clk)
        self.assertEqual(_input(channel, "RXELECIDLEMODE").value, 0b00)
        for name, signal in [
            ("TXRATE",      None),
            ("RXRATE",      None),
            ("TXELECIDLE",  serdes.tx_elecidle),
            ("TXDETECTRX",  serdes.tx_detect_rx),
            ("TXPD",        serdes.tx_pd),
            ("TXDEEMPH",    serdes.tx_deemph),
            ("TXMARGIN",    serdes.tx_margin),
            ("TXSWING",     serdes.tx_swing),
            ("TXDIFFCTRL",  serdes.tx_diffctrl),
            ("TXPOSTCURSOR",serdes.tx_postcursor),
            ("TXPRECURSOR", serdes.tx_precursor),
            ("RXCDRHOLD",   serdes.rx_cdr_hold),
        ]:
            self.assertIsInstance(_input(channel, name), Signal)
            if signal is not None:
                self.assertIs(_input(channel, name), signal)
        for name, signal in [
            ("RXELECIDLE",      serdes.rx_elecidle),
            ("PHYSTATUS",       serdes.phy_status),
            ("RXSTATUS",        serdes.rx_status),
            ("RXVALID",         serdes.rx_valid),
            ("RXBYTEISALIGNED", serdes.rx_byte_is_aligned),
            ("RXBUFSTATUS",     serdes.rx_buf_status),
            ("TXRATEDONE",      serdes.tx_rate_done),
            ("RXRATEDONE",      serdes.rx_rate_done),
        ]:
            self.assertIs(_output(channel, name), signal)
        self.assertEqual(len(serdes.rx_raw), 40)
        self.assertEqual(len(serdes.tx_raw), 40)
        self.assertIn("tx_raw_enable", verilog)

    def test_gtp_pcie_options_defaults(self):
        # Without the options, the GTPE2_CHANNEL configuration is unchanged.
        top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 125e6, 2.5e9)
        channel, = _instances(fragment, "GTPE2_CHANNEL")
        self.assertEqual(_parameter(channel, "ALIGN_COMMA_WORD"), 2)
        self.assertEqual(_parameter(channel, "TX_DRIVE_MODE"),    "DIRECT")
        self.assertEqual(_parameter(channel, "TX_DEEMPH0"),       0)
        self.assertEqual(_parameter(channel, "PCS_PCIE_EN"),      "FALSE")
        self.assertEqual(_parameter(channel, "PCS_RSVD_ATTR"),    0)
        self.assertEqual(_parameter(channel, "RXOOB_CLK_CFG"),    "PMA")
        self.assertEqual(_input(channel, "SIGVALIDCLK").value,     0)
        self.assertEqual(_input(channel, "RXELECIDLEMODE").value,  0b11)
        self.assertEqual(_input(channel, "TXOUTCLKSEL").value,     0b011)
        # Reset values of the new control signals keep the previous constants.
        serdes = top.serdes
        self.assertEqual(serdes.tx_diffctrl.reset.value, 0b1000)
        for signal in [serdes.tx_elecidle, serdes.tx_detect_rx, serdes.tx_pd, serdes.tx_deemph,
                       serdes.tx_margin, serdes.tx_swing, serdes.tx_postcursor, serdes.tx_precursor,
                       serdes.tx_rate, serdes.rx_rate, serdes.rx_cdr_hold]:
            self.assertEqual(signal.reset.value, 0)

    def test_gtp_txoutclk_sel(self):
        # TXOUTCLK source override (ex: reference clock to a fabric MMCM), only with an external
        # tx_clk since TXOUTCLK then no longer drives the tx clock domain.
        top, fragment, csrs, verilog = self.elaborate("gtp", "qpll", 100e6, 2.5e9,
            external_clks    = True,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            txoutclk_sel     = "refclk",
        )
        channel, = _instances(fragment, "GTPE2_CHANNEL")
        self.assertEqual(_input(channel, "TXOUTCLKSEL").value, 0b011)
        with self.assertRaises(AssertionError):
            self.elaborate("gtp", "qpll", 100e6, 2.5e9, tx_buffer_enable=True, rx_buffer_enable=True,
                txoutclk_sel="refclk")

    def test_ultrascale_unaligned_tx_clock_rejected(self):
        # UltraScale(+): TX user clock (without TX buffer) must be an integer division of the
        # reference clock (BUFG_GT divider).
        for family in ["gth3", "gth4", "gty"]:
            with self.subTest(family=family):
                with self.assertRaises(AssertionError):
                    self.elaborate(family, "qpll", 156.25e6, 10.3125e9, data_width=40)

if __name__ == "__main__":
    unittest.main()
