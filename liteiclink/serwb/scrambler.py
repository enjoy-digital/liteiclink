from functools import reduce
from operator import xor

from litex.gen import *

from litex.soc.interconnect import stream


def K(x, y):
    return (y << 5) | x


@ResetInserter()
class Scrambler(Module):
    def __init__(self, n_io, n_state=23, taps=[17, 22]):
        self.i = Signal(n_io)
        self.o = Signal(n_io)

        # # #

        state = Signal(n_state, reset=1)
        curval = [state[i] for i in range(n_state)]
        for i in reversed(range(n_io)):
            flip = reduce(xor, [curval[tap] for tap in taps])
            self.sync += self.o[i].eq(flip ^ self.i[i])
            curval.insert(0, flip)
            curval.pop()

        self.sync += state.eq(Cat(*curval[:n_state]))


class SERWBScrambler(Module):
    def __init__(self):
        self.sink = sink = stream.Endpoint([("d", 32)])
        self.source = source = stream.Endpoint([("d", 32), ("k", 4)])

        # # #

        # scrambler
        scrambler = Scrambler(32)
        self.submodules += scrambler
        self.comb += scrambler.i.eq(sink.d)

        # insert K.29.7 every 1024 data
        kcount = Signal(10)
        self.sync += kcount.eq(kcount + 1)
        self.comb += [
            If(kcount == 0,
                scrambler.reset.eq(1),
                source.k[0].eq(1),
                source.d[:8].eq(K(29, 7))
            ).Else(
                sink.ready.eq(1),
                source.d.eq(scrambler.o)
            )
        ]

class SERWBDescrambler(Module):
    def __init__(self):
        self.sink = sink = stream.Endpoint([("d", 32), ("k", 4)])
        self.source = source = stream.Endpoint([("d", 32)])

        # # #

        # descrambler
        descrambler = Scrambler(32)
        self.submodules += descrambler
        self.comb += descrambler.i.eq(sink.d)

        # detect and remove K.29.7
        self.comb += [
            If((sink.k[0] == 1) &
               (sink.d[:8] == K(29,7)),
               descrambler.reset.eq(1)
            ).Else(
                source.valid.eq(1),
                source.d.eq(sink.d)
            )
        ]
