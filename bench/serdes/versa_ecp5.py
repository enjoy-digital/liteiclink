#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex_boards.platforms import versa_ecp5

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.code_8b10b import K

from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

# IOs ----------------------------------------------------------------------------------------------

_transceiver_io = [
    # PCIe
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

        # Clk / Rst
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(ClockSignal())
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=False)
        if refclk_from_pll:
            pll.create_clkout(self.cd_ref, refclk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | ~pll.locked | ~rst_n)

# SerDesTestSoC ------------------------------------------------------------------------------------

class SerDesTestSoC(SoCMini):
    def __init__(self, platform, connector="pcie", linerate=2.5e9):
        assert connector in ["pcie", "sma"]
        assert linerate in [2.5e9, 5e9]
        sys_clk_freq = int(100e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LiteICLink bench on Versa ECP5",
            ident_version = True,
            with_uart     = True,
            uart_name     = "bridge"
        )

        # CRG --------------------------------------------------------------------------------------
        if linerate == 2.5e9:
            refclk_from_pll = False
            refclk_freq     = 156.25e6
        else:
            refclk_from_pll = True
            refclk_freq     = 200e6
        self.submodules.crg = _CRG(platform, sys_clk_freq, refclk_from_pll, refclk_freq)

        # SerDes RefClk ----------------------------------------------------------------------------
        if refclk_from_pll:
            refclk = self.crg.cd_ref.clk
        else:
            refclk_pads = platform.request("refclk", 1)
            self.comb += platform.request("refclk_en").eq(1)
            self.comb += platform.request("refclk_rst_n").eq(1)
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP     = refclk_pads.p,
                i_REFCLKN     = refclk_pads.n,
                o_REFCLKO     = refclk,
                p_REFCK_PWDNB = "0b1",
                p_REFCK_RTERM = "0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # SerDes PLL -------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=linerate)
        self.submodules += serdes_pll

        # SerDes -----------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        channel = 1 if connector == "sma" else 0
        self.submodules.serdes = serdes = SerDesECP5(serdes_pll, tx_pads, rx_pads,
            channel    = channel,
            data_width = 20)
        serdes.add_stream_endpoints()
        serdes.add_controls()
        self.add_csr("serdes")

        platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            serdes.sink.valid.eq(1),
            serdes.sink.ctrl.eq(0b1),
            serdes.sink.data[:8].eq(K(28, 5)),
            serdes.sink.data[8:].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        counter = Signal(8)
        self.sync.rx += [
            serdes.rx_align.eq(1),
            serdes.source.ready.eq(1),
            # No word aligner, so look for K28.5 and redirect the other byte to the leds
            If(serdes.source.data[0:8] == K(28, 5),
                counter.eq(serdes.source.data[8:]),
            ).Else(
                counter.eq(serdes.source.data[0:]),
            ),
            platform.request("user_led", 4).eq(~counter[0]),
            platform.request("user_led", 5).eq(~counter[1]),
            platform.request("user_led", 6).eq(~counter[2]),
            platform.request("user_led", 7).eq(~counter[3]),
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
    parser = argparse.ArgumentParser(description="LiteICLink transceiver example on Versa ECP5")
    parser.add_argument("--build",     action="store_true", help="Build bitstream")
    parser.add_argument("--load",      action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--connector", default="pcie",      help="Connector: pcie (default) or sma")
    parser.add_argument("--linerate",  default="2.5e9",     help="Linerate (default: 2.5e9)")
    args = parser.parse_args()

    platform = versa_ecp5.Platform(toolchain="trellis")
    platform.add_extension(_transceiver_io)
    soc = SerDesTestSoC(platform,
        connector = args.connector,
        linerate  = float(args.linerate)
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))

if __name__ == "__main__":
    main()
