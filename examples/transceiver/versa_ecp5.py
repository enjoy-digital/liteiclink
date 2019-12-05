#!/usr/bin/env python3

# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import argparse
import os

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex.boards.platforms import versa_ecp5

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from liteiclink.transceiver.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

_transceiver_io = [
    ("pcie_tx", 0,
        Subsignal("p", Pins("W4")),
        Subsignal("n", Pins("W5")),
    ),
    ("pcie_rx", 0,
        Subsignal("p", Pins("Y5")),
        Subsignal("n", Pins("Y6")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, refclk_from_pll, refclk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain(reset_less=True)
        self.clock_domains.cd_ref = ClockDomain(reset_less=True)

        # # #

        self.cd_sys.clk.attr.add("keep")

        # clk / rst
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(ClockSignal())
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # pll
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if refclk_from_pll:
            pll.create_clkout(self.cd_ref, refclk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | ~pll.locked | ~rst_n)

# SerDesTestSoC ------------------------------------------------------------------------------------

class SerDesTestSoC(SoCMini):
    def __init__(self, toolchain="diamond", connector="pcie", refclk_from_pll=True, refclk_freq=200e6):
        assert connector in ["sma", "pcie"]
        sys_clk_freq = int(100e6)
        platform = versa_ecp5.Platform(toolchain=toolchain)
        platform.add_extension(_transceiver_io)
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq, refclk_from_pll, refclk_freq)

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.bridge.wishbone)

        # SerDes RefClk ----------------------------------------------------------------------------
        if refclk_from_pll:
            refclk = self.crg.cd_ref.clk
        else:
            refclk_pads = platform.request("refclk", 1)
            self.comb += platform.request("refclk_en").eq(1)
            self.comb += platform.request("refclk_rst_n").eq(1)
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP=refclk_pads.p,
                i_REFCLKN=refclk_pads.n,
                o_REFCLKO=refclk,
                p_REFCK_PWDNB="0b1",
                p_REFCK_RTERM="0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # SerDes PLL -------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=5e9)
        self.submodules += serdes_pll

        # SerDes -----------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        channel = 1 if connector == "sma" else 0
        serdes  = SerDesECP5(serdes_pll, tx_pads, rx_pads,
            channel       = channel,
            data_width    = 20)
        serdes.add_stream_endpoints()
        self.submodules += serdes
        platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)

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
        counter = Signal(8)
        self.sync.rx += [
            serdes.rx_align.eq(1),
            serdes.source.ready.eq(1),
            If(serdes.source.data[0:8] == ((5 << 5) | 28),
                counter.eq(serdes.source.data[8:]),
            ).Else(
                counter.eq(serdes.source.data[0:]),
            ),
            platform.request("user_led", 4).eq(counter[0]),
            platform.request("user_led", 5).eq(counter[1]),
            platform.request("user_led", 6).eq(counter[2]),
            platform.request("user_led", 7).eq(counter[3]),
        ]

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 1).eq(rx_counter[26])

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 2).eq(tx_counter[26])

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SerDes Test SoC on Versa ECP5")
    args = parser.parse_args()

    soc = SerDesTestSoC(toolchain="trellis")
    builder = Builder(soc, output_dir="build")
    builder.build(build_name="versa_ecp5")

if __name__ == "__main__":
    main()
