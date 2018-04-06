from migen import *

from litex.soc.interconnect import stream
from liteiclink.serwb.scrambler import Scrambler, Descrambler
from liteiclink.serwb.packet import Packetizer, Depacketizer
from liteiclink.serwb.etherbone import Etherbone


class SERWBCore(Module):
    def __init__(self, phy, clk_freq, mode, with_scrambling=True):
        # etherbone
        self.submodules.etherbone = etherbone = Etherbone(mode)

        # packetizer / depacketizer
        depacketizer = Depacketizer(clk_freq)
        packetizer = Packetizer()
        self.submodules += depacketizer, packetizer

        # fifos
        tx_fifo = stream.SyncFIFO([("data", 32)], 16)
        rx_fifo = stream.SyncFIFO([("data", 32)], 16)
        self.submodules += tx_fifo, rx_fifo

        # scrambling
        scrambler =  Scrambler(enable=with_scrambling)
        descrambler = Descrambler(enable=with_scrambling)
        self.submodules += scrambler, descrambler

        # modules connection
        self.comb += [
            # core --> phy
            packetizer.source.connect(tx_fifo.sink),
            tx_fifo.source.connect(scrambler.sink),
            If(phy.init.ready,
                If(scrambler.source.valid,
                    phy.serdes.tx_k.eq(scrambler.source.k),
                    phy.serdes.tx_d.eq(scrambler.source.d)
                ),
                scrambler.source.ready.eq(phy.serdes.tx_ce)
            ),

            # phy --> core
            If(phy.init.ready,
                descrambler.sink.valid.eq(phy.serdes.rx_ce),
                descrambler.sink.k.eq(phy.serdes.rx_k),
                descrambler.sink.d.eq(phy.serdes.rx_d)
            ),
			descrambler.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(depacketizer.sink),

            # etherbone <--> core
            depacketizer.source.connect(etherbone.sink),
            etherbone.source.connect(packetizer.sink)
        ]
