#!/usr/bin/env python3

# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import sys

from migen import *
from migen.genlib.io import CRG

from litex.boards.platforms import kc705

from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.transceiver.gtx_7series import GTXChannelPLL, GTXQuadPLL, GTX

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # SMA
    ("sma_tx", 0,
        Subsignal("p", Pins("K2")),
        Subsignal("n", Pins("K1"))
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("K6")),
        Subsignal("n", Pins("K5"))
    ),
    # PCIe
    ("pcie_tx", 0,
        Subsignal("p", Pins("L4")),
        Subsignal("n", Pins("L3"))
    ),
    ("pcie_rx", 0,
        Subsignal("p", Pins("M6")),
        Subsignal("n", Pins("M5"))
    ),
]

# GTXTestSoC ---------------------------------------------------------------------------------------

class GTXTestSoC(SoCMini):
    def __init__(self, platform, connector="pcie", linerate=2.5e9, use_qpll=False):
        assert connector in ["sfp", "sma", "pcie"]
        sys_clk_freq = int(156e9)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("clk156"), platform.request("cpu_reset"))

        # GTX RefClk -------------------------------------------------------------------------------
        refclk      = Signal()
        refclk_pads = platform.request("sgmii_clock")
        self.specials += [
            Instance("IBUFDS_GTE2",
                i_CEB=0,
                i_I=refclk_pads.p,
                i_IB=refclk_pads.n,
                o_O=refclk)
        ]

        # GTX PLL ----------------------------------------------------------------------------------
        pll_cls = GTXQuadPLL if use_qpll else GTXChannelPLL
        pll     = pll_cls(refclk, 125e6, linerate)
        print(pll)
        self.submodules += pll

        # GTX --------------------------------------------------------------------------------------
        if connector == "sfp":
            self.comb += platform.request("sfp_tx_disable_n").eq(1)
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        gtx = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 40,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True)
        self.submodules += gtx

        platform.add_period_constraint(self.crg.cd_sys.clk, platform.default_clk_period)
        platform.add_period_constraint(gtx.cd_tx.clk, 1e9/gtx.tx_clk_freq)
        platform.add_period_constraint(gtx.cd_rx.clk, 1e9/gtx.rx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtx.cd_tx.clk,
            gtx.cd_rx.clk)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            gtx.encoder.k[0].eq(1),
            gtx.encoder.d[0].eq((5 << 5) | 28),
            gtx.encoder.k[1].eq(0),
            gtx.encoder.d[1].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        for i in range(4):
            self.comb += platform.request("user_led", 4 + i).eq(gtx.decoders[1].d[i])

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(tx_counter + 1)
        self.comb += platform.request("user_led", 1).eq(tx_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 2).eq(rx_counter[26])

# Load ---------------------------------------------------------------------------------------------

def load():
    from litex.build.xilinx import VivadoProgrammer
    prog = VivadoProgrammer()
    prog.load_bitstream("build/gateware/kc705.bit")
    exit()

# Build --------------------------------------------------------------------------------------------

def main():
    if "load" in sys.argv[1:]:
        load()
    platform = kc705.Platform()
    platform.add_extension(_transceiver_io)
    soc = GTXTestSoC(platform)
    builder = Builder(soc, output_dir="build")
    builder.build(build_name="kc705")

if __name__ == "__main__":
    main()
