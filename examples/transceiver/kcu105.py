#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *
from migen.genlib.io import CRG

from litex.boards.platforms import kcu105

from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.transceiver.gth_ultrascale import GTHChannelPLL, GTHQuadPLL, GTH

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # SFP0
    ("sfp0_tx", 0,
        Subsignal("p", Pins("U4")),
        Subsignal("n", Pins("U3"))
    ),
    ("sfp0_rx", 0,
        Subsignal("p", Pins("T2")),
        Subsignal("n", Pins("T1"))
    ),
    # SFP1
    ("sfp1_tx", 0,
        Subsignal("p", Pins("W4")),
        Subsignal("n", Pins("W3"))
    ),
    ("sfp1_rx", 0,
        Subsignal("p", Pins("V2")),
        Subsignal("n", Pins("V1"))
    ),
    # SMA
    ("sma_tx", 0,
        Subsignal("p", Pins("R4")),
        Subsignal("n", Pins("R3"))
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("P2")),
        Subsignal("n", Pins("P1"))
    ),
    # PCIe
    ("pcie_tx", 0,
        Subsignal("p", Pins("AC3")),
        Subsignal("n", Pins("AC4"))
    ),
    ("pcie_rx", 0,
        Subsignal("p", Pins("AB2")),
        Subsignal("n", Pins("AB1"))
    ),
]

# GTXTestSoC ---------------------------------------------------------------------------------------

class GTHTestSoC(SoCMini):
    def __init__(self, platform, connector="pcie", linerate=2.5e9):
        assert connector in ["sfp0", "sfp1", "sma", "pcie"]
        sys_clk_freq = int(125e9)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("clk125"), platform.request("cpu_reset"))
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/125e6)

        # 125Mhz clock -> user_sma --> user_sma_mgt_refclk -----------------------------------------
        user_sma_clock_pads = platform.request("user_sma_clock")
        user_sma_clock      = Signal()
        self.specials += [
            Instance("ODDRE1",
                i_D1=0, i_D2=1, i_SR=0,
                i_C=ClockSignal(),
                o_Q=user_sma_clock),
            Instance("OBUFDS",
                i_I=user_sma_clock,
                o_O=user_sma_clock_pads.p,
                o_OB=user_sma_clock_pads.n)
        ]

        # GTH RefClk -------------------------------------------------------------------------------
        refclk      = Signal()
        refclk_pads = platform.request("user_sma_mgt_refclk")
        self.specials += [
            Instance("IBUFDS_GTE3",
                i_CEB=0,
                i_I=refclk_pads.p,
                i_IB=refclk_pads.n,
                o_O=refclk)
        ]

        # GTH PLL ----------------------------------------------------------------------------------
        cpll = GTHChannelPLL(refclk, 125e6, linerate)
        print(cpll)
        self.submodules += cpll

        # GTH --------------------------------------------------------------------------------------
        if connector == "sfp0":
            self.comb += platform.request("sfp_tx_disable_n", 0).eq(1)
        if connector == "sfp1":
            self.comb += platform.request("sfp_tx_disable_n", 1).eq(1)
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        gth = GTH(cpll, tx_pads, rx_pads, self.clk_freq, clock_aligner=True)
        self.submodules += gth

        platform.add_period_constraint(gth.cd_tx.clk, 1e9/gth.tx_clk_freq)
        platform.add_period_constraint(gth.cd_rx.clk, 1e9/gth.rx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gth.cd_tx.clk,
            gth.cd_rx.clk)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            gth.encoder.k[0].eq(1),
            gth.encoder.d[0].eq((5 << 5) | 28),
            gth.encoder.k[1].eq(0),
            gth.encoder.d[1].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        for i in range(4):
            self.comb += platform.request("user_led", 4+ i).eq(gth.decoders[1].d[i])

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
    prog.load_bitstream("build/gateware/kcu105.bit")
    exit()

# Build --------------------------------------------------------------------------------------------

def main():
    if "load" in sys.argv[1:]:
        load()
    platform = kcu105.Platform()
    platform.add_extension(_transceiver_io)
    soc     = GTHTestSoC(platform)
    builder = Builder(soc, output_dir="build")
    builder.build(build_name="kcu105")

if __name__ == "__main__":
    main()
