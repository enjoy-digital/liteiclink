#!/usr/bin/env python3
import sys

from migen import *
from migen.genlib.io import CRG

from litex.soc.interconnect.csr import *
from litex.build.generic_platform import *
from litex.boards.platforms import kcu105

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from liteiclink.transceiver.gth_ultrascale import GTHChannelPLL, GTHQuadPLL, GTH


class GTHTestSoC(SoCCore):
    def __init__(self, platform, medium="sfp0"):
        sys_clk_freq = int(125e9)
        SoCCore.__init__(self, platform, sys_clk_freq, cpu_type=None)
        clk125 = platform.request("clk125")
        rst = platform.request("cpu_reset")
        self.submodules.crg = CRG(clk125, rst)

        self.crg.cd_sys.clk.attr.add("keep")
        platform.add_period_constraint(self.crg.cd_sys.clk, 8.0)

        # 125Mhz clock -> user_sma --> user_sma_mgt_refclk
        user_sma_clock_pads = platform.request("user_sma_clock")
        user_sma_clock = Signal()
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

        # refclk
        refclk = Signal()
        refclk_pads = platform.request("user_sma_mgt_refclk")
        self.specials += [
            Instance("IBUFDS_GTE3",
                i_CEB=0,
                i_I=refclk_pads.p,
                i_IB=refclk_pads.n,
                o_O=refclk)
        ]

        # pll
        cpll = GTHChannelPLL(refclk, 125e6, 1.25e9)
        print(cpll)
        self.submodules += cpll

        # gth
        if medium == "sfp0":
            self.comb += platform.request("sfp_tx_disable_n", 0).eq(1)
            tx_pads = platform.request("sfp_tx", 0)
            rx_pads = platform.request("sfp_rx", 0)
        elif medium == "sfp1":
            self.comb += platform.request("sfp_tx_disable_n", 1).eq(1)
            tx_pads = platform.request("sfp_tx", 1)
            rx_pads = platform.request("sfp_rx", 1)
        elif medium == "sma":
            tx_pads = platform.request("user_sma_mgt_tx")
            rx_pads = platform.request("user_sma_mgt_rx")
        else:
            raise ValueError
        gth = GTH(cpll, tx_pads, rx_pads, self.clk_freq,
            clock_aligner=True, internal_loopback=False)
        self.submodules += gth

        # led
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        self.comb += [
            gth.encoder.k[0].eq(1),
            gth.encoder.d[0].eq((5 << 5) | 28),
            gth.encoder.k[1].eq(0),
            gth.encoder.d[1].eq(counter[26:]),
        ]

        self.comb += platform.request("user_led", 4).eq(gth.rx_ready)
        for i in range(4):
            self.comb += platform.request("user_led", i).eq(gth.decoders[1].d[i])

        gth.cd_tx.clk.attr.add("keep")
        gth.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(gth.cd_tx.clk, 1e9/gth.tx_clk_freq)
        platform.add_period_constraint(gth.cd_rx.clk, 1e9/gth.tx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gth.cd_tx.clk,
            gth.cd_rx.clk)

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(tx_counter + 1)
        self.comb += platform.request("user_led", 7).eq(tx_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 6).eq(rx_counter[26])


def main():
    if "load" in sys.argv[1:]:
        from litex.build.xilinx import VivadoProgrammer
        prog = VivadoProgrammer()
        prog.load_bitstream("build/gateware/kcu105.bit")
    else:
        platform = kcu105.Platform()
        soc = GTHTestSoC(platform)
        builder = Builder(soc, output_dir="build", compile_gateware=True)
        vns = builder.build(build_name="kcu105")

if __name__ == "__main__":
    main()
