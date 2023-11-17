#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2023 MoTeC
# Copyright (c) 2023 Gwenhael Goavec-Merou <gwenhael@enjoy-digital.fr>
# Copyright (c) 2017-2023 Florent Kermarrec <florent@enjoy-digital.fr>

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.generic_platform import *
from litex_boards.platforms import efinix_trion_t120_bga576_dev_kit

from litex.soc.cores.clock import TRIONPLL

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *

from litex.soc.interconnect import wishbone

from liteiclink.serwb.phy import SERWBPHY
from liteiclink.serwb.genphy import SERWBPHY as SERWBLowSpeedPHY
from liteiclink.serwb.core import SERWBCore

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------
master_connector = "P2"
slave_connector  = "P3"
serwb_io = [
    ("serwb_master", 0,
        Subsignal("clk_p", Pins(f"{master_connector}:1"),  IOStandard("1.8_V_LVCMOS")),
        Subsignal("clk_n", Pins(f"{master_connector}:3"),  IOStandard("1.8_V_LVCMOS")),
        Subsignal("tx_p",  Pins(f"{master_connector}:7"),  IOStandard("1.8_V_LVCMOS")),
        Subsignal("tx_n",  Pins(f"{master_connector}:9"),  IOStandard("1.8_V_LVCMOS")),
        Subsignal("rx_p",  Pins(f"{slave_connector}:13"),  IOStandard("1.8_V_LVCMOS")),
        Subsignal("rx_n",  Pins(f"{slave_connector}:15"),  IOStandard("1.8_V_LVCMOS")),
    ),

    ("serwb_slave", 0,
        Subsignal("clk_p", Pins(f"{slave_connector}:1"),   IOStandard("1.8_V_LVCMOS")),
        Subsignal("clk_n", Pins(f"{slave_connector}:3"),   IOStandard("1.8_V_LVCMOS")),
        Subsignal("tx_p",  Pins(f"{master_connector}:13"), IOStandard("1.8_V_LVCMOS")),
        Subsignal("tx_n",  Pins(f"{master_connector}:15"), IOStandard("1.8_V_LVCMOS")),
        Subsignal("rx_p",  Pins(f"{slave_connector}:7"),   IOStandard("1.8_V_LVCMOS")),
        Subsignal("rx_n",  Pins(f"{slave_connector}:9"),   IOStandard("1.8_V_LVCMOS")),
    ),
]
 
# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys   = ClockDomain()
        self.cd_sys4x = ClockDomain()

        # # #

        clk40 = platform.request("clk40")
        rst_n = platform.request("user_btn", 0)


        # PLL
        self.pll = pll = TRIONPLL(platform)
        self.comb += pll.reset.eq(~rst_n)
        pll.register_clkin(clk40, 40e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=True, name="sys")
        pll.create_clkout(self.cd_sys4x, 4*sys_clk_freq, phase=90, with_reset=True, name="sys4x")

# SerWBTestSoC ------------------------------------------------------------------------------------

class SerWBTestSoC(SoCMini):
    mem_map = {
        "serwb": 0x30000000,
    }
    mem_map.update(SoCMini.mem_map)

    def __init__(self, platform, with_analyzer=True):
        sys_clk_freq = 40e6

        # USBUART PMOD as Serial--------------------------------------------------------------------
        platform.add_extension(efinix_trion_t120_bga576_dev_kit.usb_pmod_io("pmod_e"))

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            csr_data_with = 32,
            ident         = "LiteICLink SerWB bench on Efinix Trion T120 BGA576 Dev Kit",
            ident_version = True,
            with_uart     = True,
            uart_name     = "uartbone")

        # SerWB ------------------------------------------------------------------------------------
        # SerWB simple test with a SerWB Master added as a Slave peripheral to the SoC and connected
        # to a SerWB Slave with a SRAM attached. Access to this SRAM is then tested from the main
        # SoC through SerWB:
        #                   +--------+    +-------+    +-------+    +------+
        #                   |        |    |       +-ck->       |    |      |
        #                   |  Test  +----+ SerWB +-tx-> SerWB +----> Test |
        #                   |   SoC  | WB |Master |    |Slave  | WB | SRAM |
        #                   |        +<---+       <-rx-+       <----+      |
        #                   +--------+    +-------+    +-------+    +------+
        # ------------------------------------------------------------------------------------------

        # SerWB Master ---------------------------------------------------------------------------
        # PHY
        serwb_master_phy = SERWBPHY(
            device       = self.platform.device,
            pads         = self.platform.request("serwb_master"),
            mode         = "master",
        )
        self.submodules.serwb_master_phy = serwb_master_phy

        # Core
        serwb_master_core = SERWBCore(serwb_master_phy, self.clk_freq, mode="slave",
            etherbone_buffer_depth = 1,
            tx_buffer_depth        = 8,
            rx_buffer_depth        = 8)
        self.submodules += serwb_master_core

        # Connect as peripheral to main SoC.
        self.bus.add_slave("serwb", serwb_master_core.bus, SoCRegion(origin=0x30000000, size=8192))

        # SerWB Slave ----------------------------------------------------------------------------
        # PHY
        serwb_slave_phy = SERWBPHY(
            device       = self.platform.device,
            pads         = self.platform.request("serwb_slave"),
            mode         = "slave",
        )
        self.clock_domains.cd_serwb = ClockDomain()
        if hasattr(serwb_slave_phy.serdes, "clocking"):
            self.comb += self.cd_serwb.clk.eq(serwb_slave_phy.serdes.clocking.refclk)
        else:
            self.comb += self.cd_serwb.clk.eq(ClockSignal("sys"))
        self.specials += AsyncResetSynchronizer(self.cd_serwb, ResetSignal("sys"))
        serwb_slave_phy = ClockDomainsRenamer("serwb")(serwb_slave_phy)
        self.submodules.serwb_slave_phy = serwb_slave_phy

        # Core
        serwb_slave_core = SERWBCore(serwb_slave_phy, self.clk_freq, mode="master",
            etherbone_buffer_depth = 1,
            tx_buffer_depth        = 8,
            rx_buffer_depth        = 8)
        serwb_slave_core = ClockDomainsRenamer("serwb")(serwb_slave_core)
        self.submodules += serwb_slave_core

        # Wishbone SRAM
        serwb_sram = ClockDomainsRenamer("serwb")(wishbone.SRAM(8192))
        self.submodules += serwb_sram
        self.comb += serwb_slave_core.bus.connect(serwb_sram.bus)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                self.serwb_master_phy.init.fsm,
                self.serwb_master_phy.serdes.rx.data,
                self.serwb_master_phy.serdes.rx.comma,
                self.serwb_master_phy.serdes.rx.idle,
                self.serwb_master_phy.serdes.tx.data,
                self.serwb_master_phy.serdes.tx.comma,
                self.serwb_master_phy.serdes.tx.idle,
                self.serwb_master_phy.serdes.rx.datapath.decoder.source,

                self.serwb_slave_phy.init.fsm,
                self.serwb_slave_phy.serdes.rx.data,
                self.serwb_slave_phy.serdes.rx.comma,
                self.serwb_slave_phy.serdes.rx.idle,
                self.serwb_slave_phy.serdes.tx.data,
                self.serwb_slave_phy.serdes.tx.comma,
                self.serwb_slave_phy.serdes.tx.idle,
                self.serwb_slave_phy.serdes.rx.datapath.decoder.source,
            ]
            self.analyzer = LiteScopeAnalyzer(analyzer_signals, 256, csr_csv="analyzer.csv")

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=efinix_trion_t120_bga576_dev_kit.Platform, description="LiteICLink SerWB bench on Efinix Trion T120 BGA576 Dev Kit.")
    parser.add_target_argument("--flash",  action="store_true", help="Flash bitstream.")
    parser.add_argument("--with-analyzer", action="store_true", help="Add LiteScope Analyzer")
    args = parser.parse_args()

    platform = efinix_trion_t120_bga576_dev_kit.Platform()
    platform.add_extension(serwb_io)

    soc     = SerWBTestSoC(platform, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build, **parser.toolchain_argdict)

    if args.load:
        from litex.build.openfpgaloader import OpenFPGALoader
        prog = OpenFPGALoader("trion_t120_bga576_jtag")
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        from litex.build.openfpgaloader import OpenFPGALoader
        prog = OpenFPGALoader("trion_t120_bga576")
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".hex")) # FIXME

if __name__ == "__main__":
    main()

# Use ----------------------------------------------------------------------------------------------

# ./efinix_trion_t120_bga576_dev_kit.py --build --load
# litex_server --uart --uart-port=/dev/ttyUSBX
# ./test_serwb.py --ident --init --sram --access
