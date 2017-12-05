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
    i = 0
    errors = 0
    last_d = -1
    while i != 2048:
        # stim
        if (yield dut.scrambler.sink.ready):
            i += 1
        yield dut.scrambler.sink.d.eq(i)

        # check
        if (yield dut.descrambler.source.valid):
            current_d = (yield dut.descrambler.source.d)
            if (current_d != (last_d + 1)):
                errors += 1
            last_d = current_d

        # cycle
        yield

    print("errors: %d" %errors)

dut = DUT()
run_simulation(dut, main_generator(dut), vcd_name="sim.vcd")
