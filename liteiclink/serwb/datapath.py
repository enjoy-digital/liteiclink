#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.io import *
from migen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream
from litex.soc.cores.code_8b10b import K, StreamEncoder, StreamDecoder

from liteiclink.serwb.scrambler import Scrambler, Descrambler

# TXDatapath ---------------------------------------------------------------------------------------

class TXDatapath(Module):
    def __init__(self, phy_dw, with_scrambling=False):
        self.idle   = idle   = Signal()
        self.comma  = comma  = Signal()
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", phy_dw)])

        # # #

        # Scrambler
        if with_scrambling:
            self.submodules.scrambler = scrambler = Scrambler()

        # Line coding
        self.submodules.encoder = encoder = StreamEncoder(nwords=4)

        # Converter
        self.submodules.converter = converter = stream.Converter(40, phy_dw)

        # Dataflow
        if with_scrambling:
            self.comb += [
                sink.connect(scrambler.sink),
                If(comma,
                    encoder.sink.valid.eq(1),
                    encoder.sink.k.eq(1),
                    encoder.sink.d.eq(K(28,5))
                ).Else(
                    scrambler.source.connect(encoder.sink)
                )
            ]
        else:
            self.comb += [
                If(comma,
                    encoder.sink.valid.eq(1),
                    encoder.sink.k.eq(1),
                    encoder.sink.d.eq(K(28,5))
                ).Else(
                    sink.connect(encoder.sink, omit={"data"}),
                    encoder.sink.d.eq(sink.data)
                ),
            ]
        self.comb += [
            If(idle,
                converter.sink.valid.eq(1),
                converter.sink.data.eq(0)
            ).Else(
                encoder.source.connect(converter.sink),
            ),
            converter.source.connect(source)
        ]

# RXDatapath ---------------------------------------------------------------------------------------

class RXDatapath(Module):
    def __init__(self, phy_dw, with_scrambling=False):
        self.shift  = shift = Signal(6)
        self.sink   = sink   = stream.Endpoint([("data", phy_dw)])
        self.source = source = stream.Endpoint([("data", 32)])
        self.idle   = idle   = Signal()
        self.comma  = comma  = Signal()

        # # #

        # Converter
        self.submodules.converter = converter = stream.Converter(phy_dw, 40)

        # Shifter
        self.submodules.shifter = shifter = stream.Shifter(dw=40)
        self.comb += shifter.shift.eq(shift)

        # Line Coding
        self.submodules.decoder = decoder = StreamDecoder(nwords=4)

        # Descrambler
        if with_scrambling:
            self.submodules.descrambler = descrambler = Descrambler()

        # Dataflow
        self.comb += [
            sink.connect(converter.sink),
            converter.source.connect(shifter.sink),
            shifter.source.connect(decoder.sink)
        ]
        if with_scrambling:
            self.comb += [
                decoder.source.connect(descrambler.sink),
                descrambler.source.connect(source)
            ]
        else:
            self.comb += [
                decoder.source.connect(source, omit={"d", "k"}),
                source.data.eq(decoder.source.d)
            ]

        # Idle decoding
        idle_timer = WaitTimer(32)
        self.submodules += idle_timer
        self.sync += [
            If(converter.source.valid,
                idle_timer.wait.eq((converter.source.data == 0) | (converter.source.data == (2**40-1)))
            ),
            idle.eq(idle_timer.done)
        ]
        # Comma decoding
        self.sync += [
            If(decoder.source.valid,
                comma.eq((decoder.source.k == 1) & (decoder.source.d == K(28, 5)))
            )
        ]
