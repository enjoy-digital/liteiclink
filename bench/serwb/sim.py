#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone

from liteeth.phy.model import LiteEthPHYModel
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from liteiclink.serwb.genphy import SERWBPHY
from liteiclink.serwb.core import SERWBCore

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data",  Pins(8)),
    ),
    ("eth", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# SoCLinux -----------------------------------------------------------------------------------------

class SerWBSoC(SoCCore):
    def __init__(self):
        platform     = Platform()
        sys_clk_freq = int(1e6)

        self.comb += platform.trace.eq(1)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            integrated_rom_size = 0x10000,
            uart_name           = "sim")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # Etherbone---------------------------------------------------------------------------------
        # phy
        self.submodules.ethphy = LiteEthPHYModel(self.platform.request("eth"))
        self.add_csr("ethphy")
        # core
        ethcore = LiteEthUDPIPCore(self.ethphy,
            mac_address = 0x10e2d5000000,
            ip_address  = "192.168.1.50",
            clk_freq    = sys_clk_freq)
        self.submodules.ethcore = ethcore
        # etherbone
        self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234, mode="master")
        self.add_wb_master(self.etherbone.wishbone.bus)

        # SerWB ------------------------------------------------------------------------------------
        # SerWB simple test with a SerWB Master added as a Slave peripheral to the SoC and connected
        # to a SerWB Slave with a SRAM attached. Access to this SRAM is then tested from the main
        # SoC through SerWB:
        #                   +--------+    +------+    +------+    +------+
        #                   |        |    |      |    |      |    |      |
        #                   |  Test  +----+SerWB +---->SerWB +----> Test |
        #                   |   SoC  | WB |Master|    |Slave | WB | SRAM |
        #                   |        +<---+      <----+      <----+      |
        #                   +--------+    +------+    +------+    +------+
        # ------------------------------------------------------------------------------------------

        # Pads
        serwb_master_pads = Record([("clk", 1), ("tx", 1), ("rx", 1)])
        serwb_slave_pads  = Record([("clk", 1), ("tx", 1), ("rx", 1)])
        self.comb += [
            serwb_slave_pads.clk.eq(serwb_master_pads.clk),
            serwb_slave_pads.rx.eq(serwb_master_pads.tx),
            serwb_master_pads.rx.eq(serwb_slave_pads.tx),
        ]

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

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SerWB LiteX/Verilator Simulation")
    parser.add_argument("--trace",       action="store_true", help="Enable VCD tracing")
    parser.add_argument("--trace-start", default="0",         help="Cycle to start VCD tracing")
    parser.add_argument("--trace-end",   default="-1",        help="Cycle to end VCD tracing")
    args = parser.parse_args()

    sim_config = SimConfig()
    sim_config.add_clocker("sys_clk", freq_hz=1e6)
    sim_config.add_module("serial2console", "serial")
    sim_config.add_module("ethernet", "eth", args={"interface": "tap0", "ip": "192.168.1.100"})

    soc     = SerWBSoC()
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(sim_config=sim_config,
        trace       = True,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end),
    )

if __name__ == "__main__":
    main()
