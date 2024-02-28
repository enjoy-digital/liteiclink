#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.build.generic_platform import *

from litex_boards.platforms import icebreaker
from litex_boards.targets.icebreaker import _CRG

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone
from litex.soc.cores.ram import Up5kSPRAM

from liteiclink.serwb.genphy import SERWBPHY
from liteiclink.serwb.core import SERWBCore, SERIOCore

kB = 1024

# IOs ----------------------------------------------------------------------------------------------

serwb_io = [
    ("serwb_slave", 0,
        Subsignal("clk", Pins("PMOD1B:4"), IOStandard("LVCMOS33")),
        Subsignal("tx",  Pins("PMOD1B:6"), IOStandard("LVCMOS33")),
        Subsignal("rx",  Pins("PMOD1B:5"), IOStandard("LVCMOS33")),
    ),
]

# SerWBDemoSoC -------------------------------------------------------------------------------------

class SerWBDemoSoC(SoCMini):
    def __init__(self, platform, with_analyzer=False):
        sys_clk_freq = int(25e6)

        # CRG --------------------------------------------------------------------------------------
        self.cd_sys = ClockDomain()

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            csr_data_width = 32,
            ident          = "LiteICLink SerWB demo on iCEBreaker",
            ident_version  = True,
            with_integrated_main_ram = True,
            integrated_main_ram_size = 1024,
        )

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # SerWB (Slave) ----------------------------------------------------------------------------
        # PHY.
        # ----
        self.serwb_slave_phy = SERWBPHY(
            device = platform.device,
            pads   = platform.request("serwb_slave"),
            mode   = "slave"
        )
        self.comb += self.cd_sys.clk.eq(self.serwb_slave_phy.serdes.clocking.refclk)

        # Core.
        # -----
        self.serwb_slave_core = SERWBCore(self.serwb_slave_phy, self.clk_freq, mode="master",
            etherbone_buffer_depth = 1,
            tx_buffer_depth        = 8,
            rx_buffer_depth        = 8,
        )

        # Add SerWB as Master to SoC.
        # ---------------------------
        self.bus.add_master("serwb", self.serwb_slave_core.bus, SoCRegion(origin=0x00000000, size=0x20000000))

        # Address Remapping.
        # ------------------
        serwb_bus = wishbone.Interface(address_width=32, data_width=32)
        self.submodules += wishbone.Remapper(
            master      = self.bus.masters["serwb"],
            slave       = serwb_bus,
            src_regions = [SoCRegion(origin=0x10000000, size=0x10000000)],
            dst_regions = [SoCRegion(origin=0x40000000, size=0x10000000)],
        )
        self.bus.masters["serwb"] = serwb_bus

        # SerIO ------------------------------------------------------------------------------------

        # SerIO.
        # ------
        self.serio = SERIOCore(serwb_core=self.serwb_slave_core)

        # Increment Inputs every 100ms as PoC.
        # ------------------------------------
        self.serio_timer = WaitTimer(100e-3*sys_clk_freq)
        self.comb += self.serio_timer.wait.eq(~self.serio_timer.done)
        self.sync += If(self.serio_timer.done, self.serio.i.eq(self.serio.i + 1))

        # Leds -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("user_led", 0).eq(self.serwb_slave_phy.init.ready),
            platform.request("user_led", 1).eq(self.serwb_slave_phy.init.error),
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink SerWB demo on iCEBreaker")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream (to SRAM)")
    args = parser.parse_args()

    platform = icebreaker.Platform()
    platform.add_extension(icebreaker.break_off_pmod)
    platform.add_extension(serwb_io)
    soc     = SerWBDemoSoC(platform)
    builder = Builder(soc, csr_csv="csr.csv", csr_json="icebreaker_soc.json")
    builder.build(run=args.build)

    if args.load:
        from litex.build.lattice.programmer import IceStormProgrammer
        prog = IceStormProgrammer()
        prog.flash(0, "build/icebreaker/gateware/icebreaker.bin")


if __name__ == "__main__":
    main()
