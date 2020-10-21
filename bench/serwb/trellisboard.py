#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex_boards.platforms import trellisboard
from litex_boards.targets.trellisboard import _CRG

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone

from liteiclink.serwb.genphy import SERWBPHY
from liteiclink.serwb.core import SERWBCore

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------

serwb_io = [
    # PMOD loopback
    ("serwb_master", 0,
        Subsignal("clk", Pins("pmoda:0"), IOStandard("LVCMOS33")),
        Subsignal("tx",  Pins("pmoda:1"), IOStandard("LVCMOS33")),
        Subsignal("rx",  Pins("pmoda:2"), IOStandard("LVCMOS33")),
    ),

    ("serwb_slave", 0,
        Subsignal("clk", Pins("pmodb:0"), IOStandard("LVCMOS33")),
        Subsignal("tx",  Pins("pmodb:2"), IOStandard("LVCMOS33")),
        Subsignal("rx",  Pins("pmodb:1"), IOStandard("LVCMOS33")),
    ),
]

# SerWBTestSoC ------------------------------------------------------------------------------------

class SerWBTestSoC(SoCMini):
    mem_map = {
        "serwb": 0x30000000,
    }
    mem_map.update(SoCMini.mem_map)

    def __init__(self, platform, loopback=False, with_analyzer=False):
        sys_clk_freq = int(75e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            csr_data_width = 32,
            ident          = "LiteICLink SerWB bench on Trellisboard",
            ident_version  = True,
            with_uart      = True,
            uart_name      = "bridge")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SerWB ------------------------------------------------------------------------------------
        # SerWB simple test with a SerWB Master added as a Slave peripheral to the SoC and connected
        # to a SerWB Slave with a SRAM attached. Access to this SRAM is then tested from the main
        # SoC through SerWB:
        #                   +--------+    +------+    +------+    +------+
        #                   |        |    |      +-ck->      |    |      |
        #                   |  Test  +----+SerWB +-tx->SerWB +----> Test |
        #                   |   SoC  | WB |Master|    |Slave | WB | SRAM |
        #                   |        +<---+      <-rx-+      <----+      |
        #                   +--------+    +------+    +------+    +------+
        # ------------------------------------------------------------------------------------------

        # Pads
        if loopback:
            serwb_master_pads = Record([("tx", 1), ("rx", 1)])
            serwb_slave_pads  = Record([("tx", 1), ("rx", 1)])
            self.comb += serwb_slave_pads.rx.eq(serwb_master_pads.tx)
            self.comb += serwb_master_pads.rx.eq(serwb_slave_pads.tx)
        else:
            serwb_master_pads = platform.request("serwb_master")
            serwb_slave_pads  = platform.request("serwb_slave")

        # Master
        self.submodules.serwb_master_phy = SERWBPHY(
            device = platform.device,
            pads   = serwb_master_pads,
            mode   = "master")
        self.add_csr("serwb_master_phy")

        # Slave
        self.submodules.serwb_slave_phy = SERWBPHY(
            device = platform.device,
            pads   = serwb_slave_pads,
            mode   ="slave")
        self.add_csr("serwb_slave_phy")

        # Wishbone Slave
        serwb_master_core = SERWBCore(self.serwb_master_phy, self.clk_freq, mode="slave")
        self.submodules += serwb_master_core

        # Wishbone Master
        serwb_slave_core = SERWBCore(self.serwb_slave_phy, self.clk_freq, mode="master")
        self.submodules += serwb_slave_core

        # Wishbone SRAM
        self.submodules.serwb_sram = wishbone.SRAM(8192)
        self.bus.add_slave("serwb", serwb_master_core.bus, SoCRegion(origin=0x30000000, size=8192))
        self.comb += serwb_slave_core.bus.connect(self.serwb_sram.bus)

        # Leds -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("user_led", 0).eq(self.serwb_master_phy.init.ready),
            platform.request("user_led", 1).eq(self.serwb_master_phy.init.error),
            platform.request("user_led", 2).eq(self.serwb_slave_phy.init.ready),
            platform.request("user_led", 3).eq(self.serwb_slave_phy.init.error),
        ]

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [] # TODO
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 256, csr_csv="analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink SerWB bench on Trellisboard")
    parser.add_argument("--build",         action="store_true", help="Build bitstream")
    parser.add_argument("--load",          action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--loopback",      action="store_true", help="Loopback SerWB in FPGA (no IOs)")
    parser.add_argument("--with-analyzer", action="store_true", help="Add LiteScope Analyzer")
    args = parser.parse_args()

    platform = trellisboard.Platform(toolchain="trellis")
    platform.add_extension(serwb_io)
    soc     = SerWBTestSoC(platform, loopback=args.loopback, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))


if __name__ == "__main__":
    main()
