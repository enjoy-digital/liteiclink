#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Common transceiver interface: every SerDes wrapper exposes the same names (line rate, data width,
user clocks, raw words, controls), the older names being kept as aliases of the same objects.
"""

import unittest

from migen import *

from litex.gen import *

from liteiclink.serdes.gtp_7series     import GTPQuadPLL, GTP
from liteiclink.serdes.gtx_7series     import GTXChannelPLL, GTX
from liteiclink.serdes.gth_7series     import GTHChannelPLL as GTH2ChannelPLL, GTH as GTH2
from liteiclink.serdes.gth3_ultrascale import GTHChannelPLL as GTH3ChannelPLL, GTH3
from liteiclink.serdes.gth4_ultrascale import GTHChannelPLL as GTH4ChannelPLL, GTH4
from liteiclink.serdes.gty_ultrascale  import GTYChannelPLL, GTY
from liteiclink.serdes.serdes_ecp5     import SerDesECP5PLL, SerDesECP5
from liteiclink.serdes.gtr12_gw5       import GTR12

# Helpers ------------------------------------------------------------------------------------------

LINERATE   = 2.5e9
DATA_WIDTH = 20

def _pads():
    return Record([("p", 1), ("n", 1)])

class _GTR12Toolchain:
    def __init__(self):
        self.additional_tcl_commands = []

class _GTR12Platform:
    def __init__(self):
        self.toolchain  = _GTR12Toolchain()
        self.devicename = "GW5AST-138C"

GTR12_CSR_CONFIG = "upar_write_driver(0x000000,0x00000000)\n"

def wrappers():
    # name: (wrapper, has raw words).
    xilinx = {
        "gtp"  : (GTPQuadPLL,     GTP),
        "gtx"  : (GTXChannelPLL,  GTX),
        "gth2" : (GTH2ChannelPLL, GTH2),
        "gth3" : (GTH3ChannelPLL, GTH3),
        "gth4" : (GTH4ChannelPLL, GTH4),
        "gty"  : (GTYChannelPLL,  GTY),
    }
    r = {}
    for name, (pll_cls, serdes_cls) in xilinx.items():
        pll     = pll_cls(Signal(), 125e6, LINERATE)
        r[name] = (serdes_cls(pll, _pads(), _pads(), sys_clk_freq=100e6, data_width=DATA_WIDTH), name in ["gtp", "gty"])
    pll       = SerDesECP5PLL(Signal(), 125e6, LINERATE)
    r["ecp5"] = (SerDesECP5(pll, _pads(), _pads(), data_width=DATA_WIDTH), True)
    r["gtr12"] = (GTR12(_GTR12Platform(), linerate=LINERATE, csr_config=GTR12_CSR_CONFIG, data_width=DATA_WIDTH), True)
    return r

# Common names -------------------------------------------------------------------------------------

COMMON = [
    "tx_enable", "tx_ready", "tx_produce_pattern", "tx_pattern",
    "rx_enable", "rx_ready", "rx_align",
    "encoder", "decoders", "cd_tx", "cd_rx", "add_stream_endpoints",
]
RAW = ["tx_raw_enable", "tx_raw", "rx_raw"]

class TestSerDesInterface(unittest.TestCase):
    def test_common_names(self):
        for name, (serdes, has_raw) in wrappers().items():
            with self.subTest(wrapper=name):
                self.assertEqual(serdes.linerate,    LINERATE)
                self.assertEqual(serdes.data_width,  DATA_WIDTH)
                self.assertEqual(serdes.nwords,      DATA_WIDTH//10)
                self.assertEqual(serdes.tx_clk_freq, LINERATE/DATA_WIDTH)
                self.assertEqual(serdes.rx_clk_freq, LINERATE/DATA_WIDTH)
                for attr in COMMON:
                    self.assertTrue(hasattr(serdes, attr), attr)
                for attr in RAW:
                    self.assertEqual(hasattr(serdes, attr), has_raw, attr)
                if has_raw:
                    self.assertGreaterEqual(len(serdes.tx_raw), DATA_WIDTH)
                    self.assertGreaterEqual(len(serdes.rx_raw), DATA_WIDTH)

    def test_ecp5_raw_aliases(self):
        # tx_raw_data/rx_raw_data (older names): same signals as tx_raw/rx_raw.
        serdes, _ = wrappers()["ecp5"]
        self.assertIs(serdes.tx_raw, serdes.tx_raw_data)
        self.assertIs(serdes.rx_raw, serdes.rx_raw_data)

    def test_gtr12_linerate_alias(self):
        # line_rate (older name): same as linerate, as argument and attribute.
        for kwargs in [{"linerate": 5e9}, {"line_rate": 5e9}]:
            with self.subTest(**kwargs):
                gtr12 = GTR12(_GTR12Platform(), csr_config=GTR12_CSR_CONFIG, **kwargs)
                self.assertEqual(gtr12.linerate,    5e9)
                self.assertEqual(gtr12.line_rate,   5e9)
                self.assertEqual(gtr12.tx_clk_freq, 125e6)
        # Positional (previous signature).
        gtr12 = GTR12(_GTR12Platform(), 5e9, GTR12_CSR_CONFIG)
        self.assertEqual(gtr12.linerate, 5e9)
        # One of them.
        with self.assertRaises(AssertionError):
            GTR12(_GTR12Platform(), csr_config=GTR12_CSR_CONFIG)
        with self.assertRaises(AssertionError):
            GTR12(_GTR12Platform(), 5e9, GTR12_CSR_CONFIG, linerate=5e9)

if __name__ == "__main__":
    unittest.main()
