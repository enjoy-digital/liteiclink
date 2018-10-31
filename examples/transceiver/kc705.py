#!/usr/bin/env python3
import sys

from migen import *
from migen.genlib.io import CRG

from litex.boards.platforms import kc705

from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.transceiver.gtx_7series import GTXChannelPLL, GTXQuadPLL, GTX


class GTXTestSoC(SoCCore):
    def __init__(self, platform, use_qpll=True):
        sys_clk_freq = int(156e9)
        SoCCore.__init__(self, platform, sys_clk_freq, cpu_type=None)
        clk156 = platform.request("clk156")
        rst = platform.request("cpu_reset")
        self.submodules.crg = CRG(clk156, rst)

        # refclk
        refclk = Signal()
        refclk_pads = platform.request("sgmii_clock")
        self.specials += [
            Instance("IBUFDS_GTE2",
                i_CEB=0,
                i_I=refclk_pads.p,
                i_IB=refclk_pads.n,
                o_O=refclk)
        ]

        # pll
        pll_cls = GTXQuadPLL if use_qpll else GTXChannelPLL
        pll = pll_cls(refclk, 125e6, 1.25e9)
        print(pll)
        self.submodules += pll

        # gtx
        self.comb += platform.request("sfp_tx_disable_n").eq(1)
        tx_pads = platform.request("sfp_tx")
        rx_pads = platform.request("sfp_rx")
        gtx = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
            clock_aligner=True, internal_loopback=False)
        self.submodules += gtx

        # led blink
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        self.comb += [
            gtx.encoder.k[0].eq(1),
            gtx.encoder.d[0].eq((5 << 5) | 28),
            gtx.encoder.k[1].eq(0),
            gtx.encoder.d[1].eq(counter[26:]),
        ]

        self.comb += platform.request("user_led", 4).eq(gtx.rx_ready)
        for i in range(4):
            self.comb += platform.request("user_led", i).eq(gtx.decoders[1].d[i])

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(tx_counter + 1)
        self.comb += platform.request("user_led", 7).eq(tx_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 6).eq(rx_counter[26])

        # constraints
        self.crg.cd_sys.clk.attr.add("keep")
        gtx.cd_tx.clk.attr.add("keep")
        gtx.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(self.crg.cd_sys.clk, platform.default_clk_period)
        platform.add_period_constraint(gtx.cd_tx.clk, 1e9/gtx.tx_clk_freq)
        platform.add_period_constraint(gtx.cd_rx.clk, 1e9/gtx.tx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtx.cd_tx.clk)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtx.cd_rx.clk)


def main():
    if "load" in sys.argv[1:]:
        from litex.build.xilinx import VivadoProgrammer
        prog = VivadoProgrammer()
        prog.load_bitstream("build/gateware/kc705.bit")
    else:
        platform = kc705.Platform()
        soc = GTXTestSoC(platform)
        builder = Builder(soc, output_dir="build", compile_gateware=True)
        vns = builder.build(build_name="kc705")

if __name__ == "__main__":
    main()
