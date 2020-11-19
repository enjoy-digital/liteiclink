#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *
from migen.genlib.io import CRG

from litex_boards.platforms import xcu1525

from litex.build.generic_platform import *

from litex.soc.cores.clock import USPMMCM
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.code_8b10b import K

from liteiclink.serdes.gty_ultrascale import GTYChannelPLL, GTYQuadPLL, GTY

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # QSFP0
    ("qsfp0_refclk", 0,
        Subsignal("p", Pins("M11")),
        Subsignal("n", Pins("M10")),
    ),
    ("qsfp0_fs", 0, Pins("AT20 AU22"), IOStandard("LVCMOS12")),
    ("qsfp0_tx", 0,
        Subsignal("p", Pins("N9")),
        Subsignal("n", Pins("N8"))
    ),
    ("qsfp0_rx", 0,
        Subsignal("p", Pins("N4")),
        Subsignal("n", Pins("N3"))
    ),

    # QSFP1
    ("qsfp1_refclk", 0,
        Subsignal("p", Pins("T11")),
        Subsignal("n", Pins("T10")),
    ),
    ("qsfp1_fs", 0, Pins("AR22 AU20"), IOStandard("LVCMOS12")),
    ("qsfp1_tx", 0,
        Subsignal("p", Pins("U9")),
        Subsignal("n", Pins("U8"))
    ),
    ("qsfp1_rx", 0,
        Subsignal("p", Pins("U4")),
        Subsignal("n", Pins("U3"))
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        # PLL
        self.submodules.pll = pll = USPMMCM(speedgrade=-2)
        pll.register_clkin(platform.request("clk300"), 300e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# GTYTestSoC ---------------------------------------------------------------------------------------

class GTYTestSoC(SoCMini):
    def __init__(self, platform, connector="qsfp0", linerate=2.5e9, use_qpll=False):
        assert connector in ["qsfp0", "qsfp1"]
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LiteICLink bench on XCU1525",
            ident_version = True,
            with_uart     = True,
            uart_name     = "bridge"
        )

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # GTY RefClk -------------------------------------------------------------------------------
        refclk      = Signal()
        refclk_pads = platform.request("qsfp0_refclk")
        self.specials += Instance("IBUFDS_GTE4",
            p_REFCLK_HROW_CK_SEL = 0,
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk)
        self.comb += platform.request(connector + "_fs").eq(0b01)

        # GTY PLL ----------------------------------------------------------------------------------
        if use_qpll:
            pll = GTYQuadPLL(refclk, 156.25e6, linerate)
            print(pll)
            self.submodules.pll = pll
        else:
            pll = GTYChannelPLL(refclk, 156.25e6, linerate)
            print(pll)
            self.submodules += pll

        # GTY --------------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        self.submodules.serdes = serdes = GTY(pll, tx_pads, rx_pads, sys_clk_freq,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            clock_aligner    = False)
        serdes.add_stream_endpoints()
        serdes.add_controls()
        self.add_csr("serdes")

        platform.add_period_constraint(serdes.cd_tx.clk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.cd_rx.clk, 1e9/serdes.rx_clk_freq)
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, serdes.cd_tx.clk, serdes.cd_rx.clk)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            serdes.sink.valid.eq(1),
            serdes.sink.ctrl.eq(0b1),
            serdes.sink.data[:8].eq(K(28, 5)),
            serdes.sink.data[8:].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        #for i in range(4):
        #    self.comb += platform.request("user_led", 4 + i).eq(serdes.source.data[i])

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(tx_counter + 1)
        #self.comb += platform.request("user_led", 1).eq(tx_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        #self.comb += platform.request("user_led", 2).eq(rx_counter[26])

        # Analyzer ---------------------------------------------------------------------------------
        from litescope import LiteScopeAnalyzer
        analyzer_signals = [
            self.serdes.tx_init.done,
            self.serdes.rx_init.done,
            self.serdes.tx_init.fsm,
            self.serdes.rx_init.fsm,
            self.serdes.cd_tx.rst,
            self.serdes.cd_rx.rst,
            tx_counter,
            rx_counter,
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 512,
            clock_domain = "sys",
            csr_csv      = "analyzer.csv")
        self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink transceiver example on XCU1525")
    parser.add_argument("--build",     action="store_true", help="Build bitstream")
    parser.add_argument("--load",      action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--connector", default="qsfp0",     help="Connector: qsfp0 (default) or qsfp1")
    parser.add_argument("--linerate",  default="2.5e9",     help="Linerate (default: 2.5e9)")
    parser.add_argument("--pll",       default="cpll",      help="PLL: cpll (default) or qpll")
    args = parser.parse_args()

    platform = xcu1525.Platform()
    platform.add_extension(_transceiver_io)
    soc = GTYTestSoC(platform,
        connector = args.connector,
        linerate  = float(args.linerate),
        use_qpll  = args.pll == "qpll"
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
