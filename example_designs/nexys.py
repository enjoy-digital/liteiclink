#!/usr/bin/env python3
import sys

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.interconnect.csr import *

from litex.build.generic_platform import *
from litex.boards.platforms import nexys_video as nexys

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from liteiclink.serwb.phy import SERWBPHY
from liteiclink.serwb.genphy import SERWBPHY as SERWBLowSpeedPHY
from liteiclink.serwb.core import SERWBCore

from litescope import LiteScopeAnalyzer


serwb_io = [
    # hdmi loopback
    ("serwb_master", 0,
        Subsignal("clk_p", Pins("T1"), IOStandard("TMDS_33")), # hdmi_out clk
        Subsignal("clk_n", Pins("U1"), IOStandard("TMDS_33")), # hdmi_out clk
        Subsignal("tx_p", Pins("W1"), IOStandard("TMDS_33")),  # hdmi_out data0
        Subsignal("tx_n", Pins("Y1"), IOStandard("TMDS_33")),  # hdmi_out data0
        Subsignal("rx_p", Pins("W2"), IOStandard("TMDS_33")),  # hdmi_in data1
        Subsignal("rx_n", Pins("Y2"), IOStandard("TMDS_33")),  # hdmi_in data1
    ),

    ("serwb_slave", 0,
        Subsignal("clk_p", Pins("V4"), IOStandard("TMDS_33")), # hdmi_in clk
        Subsignal("clk_n", Pins("W4"), IOStandard("TMDS_33")), # hdmi_in clk
        Subsignal("tx_p", Pins("AA1"), IOStandard("TMDS_33")), # hdmi_out data1
        Subsignal("tx_n", Pins("AB1"), IOStandard("TMDS_33")), # hdmi_out data1
        Subsignal("rx_p", Pins("Y3"), IOStandard("TMDS_33")),  # hdmi_in data0s
        Subsignal("rx_n", Pins("AA3"), IOStandard("TMDS_33")), # hdmi_in data0
    ),
    ("serwb_enable", 0, Pins("R3"), IOStandard("LVCMOS33")), # hdmi_txen
]


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain()
        self.clock_domains.cd_clk200 = ClockDomain()

        clk100 = platform.request("clk100")
        reset = ~platform.request("cpu_reset")

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys4x = Signal()
        pll_clk200 = Signal()
        self.specials += [
            Instance("MMCME2_BASE",
                p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                # VCO @ 1GHz
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                p_CLKFBOUT_MULT_F=10, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=clk100, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                # 500MHz
                p_CLKOUT0_DIVIDE_F=2, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_sys4x,

                # 200MHz
                p_CLKOUT1_DIVIDE=5, p_CLKOUT1_PHASE=0.0, o_CLKOUT1=pll_clk200
            ),
            Instance("BUFR", p_BUFR_DIVIDE="4", i_I=pll_sys4x, o_O=self.cd_sys.clk),
            Instance("BUFIO", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked | reset),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked | reset)
        ]

        reset_counter = Signal(4, reset=15)
        ic_reset = Signal(reset=1)
        self.sync.clk200 += \
            If(reset_counter != 0,
                reset_counter.eq(reset_counter - 1)
            ).Else(
                ic_reset.eq(0)
            )
        self.specials += Instance("IDELAYCTRL", i_REFCLK=ClockSignal("clk200"), i_RST=ic_reset)


class BaseSoC(SoCCore):
    def __init__(self, platform):
        clk_freq = int(125e6)
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="Nexys Video - LiteICLink Test Design -", ident_version=True,
            with_timer=False
        )
        self.submodules.crg = _CRG(platform)
        self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"),
                                                  clk_freq, baudrate=3e6))
        self.add_wb_master(self.cpu_or_bridge.wishbone)


class SERWBTest(Module, AutoCSR):
    def __init__(self, bus):
        self.do_write = CSR()
        self.do_read = CSR()

        # # #

        self.submodules.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.submodules.timeout = timeout = WaitTimer(2**16)
        self.comb += fsm.reset.eq(self.timeout.done)
        fsm.act("IDLE",
            If(self.do_write.re,
                NextState("WRITE")
            ).Elif(self.do_read.re,
                NextState("READ")
            )
        )
        fsm.act("WRITE",
            timeout.wait.eq(1),
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.we.eq(1),
            bus.adr.eq(0x12345678),
            bus.dat_w.eq(0xdeadbeef),
            If(bus.ack,
                NextState("IDLE")
            )
        )
        fsm.act("READ",
            timeout.wait.eq(1),
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.adr.eq(0x89abcdef),
            If(bus.ack,
                NextState("IDLE")
            )
        )


class SERDESTestSoC(BaseSoC):
    csr_map = {
        "serwb_master_phy": 20,
        "serwb_slave_phy":  21,
        "serwb_test":       22,
        "analyzer":         23
    }
    csr_map.update(BaseSoC.csr_map)

    mem_map = {
        "serwb": 0x30000000,
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, platform, low_speed=True, with_core=True, with_serwb_test=False, with_analyzer=False):
        BaseSoC.__init__(self, platform)

        # serwb enable
        self.comb += platform.request("serwb_enable").eq(1)

        if low_speed:
            # serwb master
            self.submodules.serwb_master_phy = SERWBLowSpeedPHY(platform.request("serwb_master"), mode="master")
            # serwb slave
            self.submodules.serwb_slave_phy = SERWBLowSpeedPHY(platform.request("serwb_slave"), mode="slave")
        else:
            # serwb master
            self.submodules.serwb_master_phy = SERWBPHY(platform.device, platform.request("serwb_master"), mode="master")
            # serwb slave
            self.submodules.serwb_slave_phy = SERWBPHY(platform.device, platform.request("serwb_slave"), mode="slave")

        # leds
        self.comb += [
            platform.request("user_led", 4).eq(self.serwb_master_phy.init.ready),
            platform.request("user_led", 5).eq(self.serwb_master_phy.init.error),
            platform.request("user_led", 6).eq(self.serwb_slave_phy.init.ready),
            platform.request("user_led", 7).eq(self.serwb_slave_phy.init.error),
        ]

        if not with_core:
            # data
            self.sync += [
                If(self.serwb_master_phy.init.ready & self.serwb_master_phy.serdes.tx_ce,
                    self.serwb_master_phy.serdes.tx_d.eq(self.serwb_master_phy.serdes.tx_d + 1)
                ),
                If(self.serwb_slave_phy.init.ready & self.serwb_slave_phy.serdes.tx_ce,
                    self.serwb_slave_phy.serdes.tx_d.eq(self.serwb_slave_phy.serdes.tx_d + 1)
                ),
                If(self.serwb_master_phy.serdes.rx_ce,
                    platform.request("user_led", 0).eq(self.serwb_master_phy.serdes.rx_d[24]),
                    platform.request("user_led", 1).eq(self.serwb_master_phy.serdes.rx_d[25])
                ),
                If(self.serwb_slave_phy.serdes.rx_ce,
                    platform.request("user_led", 2).eq(self.serwb_slave_phy.serdes.rx_d[24]),
                    platform.request("user_led", 3).eq(self.serwb_slave_phy.serdes.rx_d[25])
                ),
            ]
        else:
            # wishbone slave
            serwb_master_core = SERWBCore(self.serwb_master_phy, self.clk_freq, mode="slave")
            self.submodules += serwb_master_core

            # wishbone master
            serwb_slave_core = SERWBCore(self.serwb_slave_phy, self.clk_freq, mode="master")
            self.submodules += serwb_slave_core

            if with_serwb_test:
                # serwb test
                self.submodules.serwb_test = SERWBTest(serwb_master_core.etherbone.wishbone.bus)
                self.comb += [
                    serwb_slave_core.etherbone.wishbone.bus.ack.eq(1),
                    serwb_slave_core.etherbone.wishbone.bus.dat_r.eq(0xdeadbeef)
                ]
            else:
                self.register_mem("serwb", self.mem_map["serwb"], serwb_master_core.etherbone.wishbone.bus, 8192)
                self.submodules.serwb_sram = wishbone.SRAM(8192, init=[i for i in range(8192//4)])
                self.comb += serwb_slave_core.etherbone.wishbone.bus.connect(self.serwb_sram.bus)

            # analyzer
            if with_analyzer:
                analyzer_signals = [
                    serwb_master_core.etherbone.wishbone.bus,
                    #self.serwb_master_phy.serdes.tx_d,
                    #self.serwb_master_phy.serdes.tx_k,
                    #self.serwb_master_phy.serdes.tx_ce,
                    #self.serwb_master_phy.serdes.rx_d,
                    #self.serwb_master_phy.serdes.rx_k,
                    #self.serwb_master_phy.serdes.rx_ce,

                    serwb_slave_core.etherbone.wishbone.bus,
                    self.serwb_slave_phy.serdes.tx_d,
                    self.serwb_slave_phy.serdes.tx_k,
                    self.serwb_slave_phy.serdes.tx_ce,
                    self.serwb_slave_phy.serdes.rx_d,
                    self.serwb_slave_phy.serdes.rx_k,
                    self.serwb_slave_phy.serdes.rx_ce,
                ]
                self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 256)

    def do_exit(self, vns):
        if hasattr(self, "analyzer"):
            self.analyzer.export_csv(vns, "test/analyzer.csv")

def main():
    platform = nexys.Platform()
    platform.add_extension(serwb_io)
    if len(sys.argv) < 2:
        print("missing target (base or serwb)")
        exit()
    if sys.argv[1] == "base":
        soc = BaseSoC(platform)
    elif sys.argv[1] == "serwb":
        soc = SERDESTestSoC(platform)
    else:
        raise KeyError
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    vns = builder.build()
    soc.do_exit(vns)


if __name__ == "__main__":
    main()
