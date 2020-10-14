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

from liteiclink.serdes.gty_ultrascale import GTYChannelPLL, GTYQuadPLL, GTY

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # QSFP0
    ("qsfp0_refclk", 0,
        Subsignal("p", Pins("M11")),
        Subsignal("n", Pins("M10")),
    ),
    ("qsfp0_tx", 0,
        Subsignal("p", Pins("N9")),
        Subsignal("n", Pins("N8"))
    ),
    ("qsfp0_rx", 0,
        Subsignal("p", Pins("N4")),
        Subsignal("n", Pins("N3"))
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
    def __init__(self, platform, connector="qsfp0", linerate=2.5e9):
        assert connector in ["qsfp0"]
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, with_uart=True, uart="bridge")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # GTY RefClk -------------------------------------------------------------------------------
        refclk      = Signal()
        refclk_pads = platform.request("qsfp0_refclk")
        self.specials += Instance("IBUFDS_GTE4",
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk)

        # GTY PLL ----------------------------------------------------------------------------------
        cpll = GTYChannelPLL(refclk, 156.25e6, linerate)
        print(cpll)
        self.submodules += cpll

        # GTY --------------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        self.submodules.gty = gty = GTY(cpll, tx_pads, rx_pads, self.clk_freq,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            clock_aligner    = False)
        gty.add_controls()
        self.add_csr("gty")

        platform.add_period_constraint(gty.cd_tx.clk, 1e9/gty.tx_clk_freq)
        platform.add_period_constraint(gty.cd_rx.clk, 1e9/gty.rx_clk_freq)
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, gty.cd_tx.clk, gty.cd_rx.clk)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            gty.encoder.k[0].eq(1),
            gty.encoder.d[0].eq((5 << 5) | 28),
            gty.encoder.k[1].eq(0),
            gty.encoder.d[1].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        #for i in range(4):
        #    self.comb += platform.request("user_led", 4+ i).eq(gty.decoders[1].d[i])

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])

        #tx_counter = Signal(32)
        #self.sync.tx += tx_counter.eq(tx_counter + 1)
        #self.comb += platform.request("user_led", 1).eq(tx_counter[26])

        #rx_counter = Signal(32)
        #self.sync.rx += rx_counter.eq(rx_counter + 1)
        #self.comb += platform.request("user_led", 2).eq(rx_counter[26])

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink transceiver example on XCU1525")
    parser.add_argument("--build",     action="store_true", help="Build bitstream")
    parser.add_argument("--load",      action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--connector", default="qsfp0",     help="Connector: qsfp0 (default)")
    parser.add_argument("--linerate",  default="2.5e9",     help="Linerate (default: 2.5e9)")
    args = parser.parse_args()

    platform = xcu1525.Platform()
    platform.add_extension(_transceiver_io)
    soc = GTYTestSoC(platform,
        connector = args.connector,
        linerate  = float(args.linerate)
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
