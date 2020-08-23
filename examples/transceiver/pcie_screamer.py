#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.boards.platforms import pcie_screamer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.transceiver.gtp_7series import GTPQuadPLL, GTP

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


class GTPTestSoC(SoCCore):
    def __init__(self, platform, connector="pcie", linerate=2.5e9):
        assert connector in ["pcie"]
        sys_clk_freq = int(100e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, cpu_type=None)
        self.submodules.crg = _CRG(platform.request("clk100"), ~platform.request("user_btn", 0))

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
        gtp = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 20,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True)
        self.submodules += gtp

        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/100e6)
        platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
        platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.rx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtp.cd_tx.clk,
            gtp.cd_rx.clk)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        self.comb += [
            gtp.encoder.k[0].eq(1),
            gtp.encoder.d[0].eq((5 << 5) | 28),
            gtp.encoder.k[1].eq(0),
            gtp.encoder.d[1].eq(counter[26:]),
        ]

       # RX (slow counter) --> Leds
        for i in range(2):
            self.comb += platform.request("user_led", i).eq(gtp.decoders[1].d[i])

# Load ---------------------------------------------------------------------------------------------

def load():
    from litex.build.xilinx import VivadoProgrammer
    prog = VivadoProgrammer()
    prog.load_bitstream("build/gateware/pcie_screamer.bit")
    exit()

# Build --------------------------------------------------------------------------------------------

def main():
    if "load" in sys.argv[1:]:
        load()
    platform = pcie_screamer.Platform()
    platform.add_extension(_transceiver_io)
    soc     = GTPTestSoC(platform)
    builder = Builder(soc, output_dir="build")
    builder.build(build_name="pcie_screamer")

if __name__ == "__main__":
    main()
