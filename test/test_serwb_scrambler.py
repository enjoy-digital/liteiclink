#!/usr/bin/env python3

from litex.gen import *

from liteiclink.serwb import scrambler

from litex.soc.interconnect.wishbone import SRAM
from litex.soc.interconnect.stream import Converter


class DUT(Module):
    def __init__(self):
        self.submodules.scrambler = scrambler.SERWBScrambler()
        self.submodules.descrambler = scrambler.SERWBDescrambler()
        self.comb += self.scrambler.source.connect(self.descrambler.sink)


def main_generator(dut):
	for i in range(8192):
		yield dut.scrambler.sink.d.eq(i)
		d = (yield dut.descrambler.source.d)
		print("0x{:08x}".format(d))
		yield


dut = DUT()
run_simulation(dut, main_generator(dut), vcd_name="sim.vcd")
