#!/usr/bin/env python3

from litex.gen import *

from liteiclink.serwb import scrambler


class DUT(Module):
    def __init__(self):
        self.submodules.scrambler = scrambler.Scrambler(sync_interval=16)
        self.submodules.descrambler = scrambler.Descrambler()
        self.comb += [
            self.scrambler.source.connect(self.descrambler.sink),
            self.descrambler.source.ready.eq(1)
        ]


def main_generator(dut):
    i = 0
    errors = 0
    last_data = -1
    while i != 256:
        print(i)
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
