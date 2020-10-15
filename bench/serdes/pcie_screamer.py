#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.boards.platforms import pcie_screamer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # PCIe
    ("pcie_tx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("A6"))
    ),
    ("pcie_rx", 0,
        Subsignal("p", Pins("B10")),
        Subsignal("n", Pins("A10"))
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, clk, rst):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        self.comb += self.cd_sys.clk.eq(clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, rst)

        self.submodules.pll = pll = S7PLL()
        pll.register_clkin(clk, 100e6)
        pll.create_clkout(self.cd_clk125, 125e6)

# GTPTestSoC ---------------------------------------------------------------------------------------

class GTPTestSoC(SoCMini):
    def __init__(self, platform, connector="pcie", linerate=2.5e9):
        assert connector in ["pcie"]
        sys_clk_freq = int(100e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, with_uart=True, uart_name="bridge")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform.request("clk100"), ~platform.request("user_btn", 0))
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/100e6)

        # GTP RefClk -------------------------------------------------------------------------------
        refclk = Signal()
        self.comb += refclk.eq(ClockSignal("clk125"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # GTP PLL ----------------------------------------------------------------------------------
        qpll = GTPQuadPLL(refclk, 125e6, linerate)
        print(qpll)
        self.submodules += qpll

        # GTP --------------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        self.submodules.serdes = serdes = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 20,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True)
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
            serdes.sink.ctrl.eq(0b01),
            serdes.sink.data[0:8].eq((5 << 5) | 28),
            serdes.sink.data[8:16].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        for i in range(2):
            self.comb += platform.request("user_led", i).eq(serdes.source.data[i])

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink transceiver example on PCIe Screamer")
    parser.add_argument("--build",     action="store_true", help="Build bitstream")
    parser.add_argument("--load",      action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--connector", default="pcie",      help="Connector: pcie (default)")
    parser.add_argument("--linerate",  default="2.5e9",     help="Linerate")
    args = parser.parse_args()

    platform = pcie_screamer.Platform()
    platform.add_extension(_transceiver_io)
    soc = GTPTestSoC(platform,
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
