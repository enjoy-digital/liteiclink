#!/usr/bin/env python3

from litex.gen import *

from liteiclink.serwb import scrambler

from litex.soc.interconnect.wishbone import SRAM
from litex.soc.interconnect.stream import Converter


class DUT(Module):
    def __init__(self):
        self.submodules.scrambler = scrambler.Scrambler()
        self.submodules.descrambler = scrambler.Descrambler()
        self.comb += self.scrambler.source.connect(self.descrambler.sink)


def main_generator(dut):
    i = 0
    errors = 0
    last_data = -1
    while i != 2048:
        # stim
        if (yield dut.scrambler.sink.ready):
            i += 1
        yield dut.scrambler.sink.data.eq(i)

        # check
        if (yield dut.descrambler.source.valid):
            current_data = (yield dut.descrambler.source.data)
            if (current_data != (last_data + 1)):
                errors += 1
            last_data = current_data

        # cycle
        yield

    print("errors: %d" %errors)

dut = DUT()
run_simulation(dut, main_generator(dut), vcd_name="sim.vcd")
