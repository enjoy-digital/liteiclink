#!/usr/bin/env python3

import os
import sys
sys.path.append("../")

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from liteiclink.serwb.phy import SERWBPHY
from liteiclink.serwb.core import SERWBCore


_io = [
    ("clk125", 0, Pins("X")),
    ("serwb_master", 0,
        Subsignal("clk_p", Pins("X")),
        Subsignal("clk_n", Pins("X")),
        Subsignal("tx_p", Pins("X")),
        Subsignal("tx_n", Pins("X")),
        Subsignal("rx_p", Pins("X")),
        Subsignal("rx_n", Pins("X")),
    ),
    ("serwb_slave", 0,
        Subsignal("clk_p", Pins("X")),
        Subsignal("clk_n", Pins("X")),
        Subsignal("tx_p", Pins("X")),
        Subsignal("tx_n", Pins("X")),
        Subsignal("rx_p", Pins("X")),
        Subsignal("rx_n", Pins("X")),
    )
]


class Platform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "", _io)


class CRG(Module):
    def __init__(self, clk125):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain()
        self.clock_domains.cd_clk200 = ClockDomain()

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys = Signal()
        pll_sys4x = Signal()
        pll_clk200 = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                     p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                     # VCO @ 1GHz
                     p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=8.0,
                     p_CLKFBOUT_MULT=8, p_DIVCLK_DIVIDE=1,
                     i_CLKIN1=clk125, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                     # 125MHz
                     p_CLKOUT0_DIVIDE=8, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_sys,

                     # 500MHz
                     p_CLKOUT1_DIVIDE=2, p_CLKOUT1_PHASE=0.0, o_CLKOUT1=pll_sys4x,

                     # 200MHz
                     p_CLKOUT2_DIVIDE=5, p_CLKOUT2_PHASE=0.0, o_CLKOUT2=pll_clk200
            ),
            Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk),
            Instance("BUFG", i_I=pll_sys4x, o_O=self.cd_sys4x.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked),
            AsyncResetSynchronizer(self.cd_sys4x, ~pll_locked),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked)
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


class SERWBSim(Module):
    def __init__(self, platform):
        clk_freq = 125e6
        self.submodules.crg = CRG(platform.request("clk125"))

        # amc
        self.submodules.serwb_phy_amc = SERWBPHY(platform.request("serwb_master"), mode="master", init_timeout=4)

        # rtm
        self.submodules.serwb_phy_rtm = SERWBPHY(platform.request("serwb_slave"), mode="slave", init_timeout=4)


def generate_top():
    platform = Platform()
    soc = SERWBSim(platform)
    platform.build(soc, build_dir="./", run=False)

def generate_top_tb():
    f = open("top_tb.v", "w")
    f.write("""
`timescale 1ns/1ps

module top_tb();

reg clk125;
initial clk125 = 1'b1;
always #4 clk125 = ~clk125;

wire serwb_clk_p;
wire serwb_clk_n;
wire serwb_tx_p;
wire serwb_tx_n;
wire serwb_rx_p;
wire serwb_rx_n;

top dut (
    .clk125(clk125),
    .serwb_amc_clk_p(serwb_clk_p),
    .serwb_amc_clk_n(serwb_clk_n),
    .serwb_amc_tx_p(serwb_tx_p),
    .serwb_amc_tx_n(serwb_tx_n),
    .serwb_amc_rx_p(serwb_rx_p),
    .serwb_amc_rx_n(serwb_rx_n),
    .serwb_rtm_clk_p(serwb_clk_p),
    .serwb_rtm_clk_n(serwb_clk_n),
    .serwb_rtm_tx_p(serwb_rx_p),
    .serwb_rtm_tx_n(serwb_rx_n),
    .serwb_rtm_rx_p(serwb_tx_p),
    .serwb_rtm_rx_n(serwb_tx_n)
);

endmodule""")
    f.close()

def run_sim():
    os.system("rm -rf xsim.dir")
    os.system("call xvlog glbl.v")
    os.system("call xvlog top.v")
    os.system("call xvlog top_tb.v")
    os.system("call xelab -debug typical top_tb glbl -s top_tb_sim -L unisims_ver -L unimacro_ver -L SIMPRIM_VER -L secureip -L $xsimdir/xil_defaultlib -timescale 1ns/1ps")
    os.system("call xsim top_tb_sim -gui")

def main():
    generate_top()
    generate_top_tb()
    run_sim()

if __name__ == "__main__":
    main()
