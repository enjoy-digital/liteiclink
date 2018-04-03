from migen import *

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

        # scrambling
        scrambler =  Scrambler(enable=with_scrambling)
        descrambler = Descrambler(enable=with_scrambling)
        self.submodules += scrambler, descrambler

        # modules connection
        self.comb += [
            # core --> phy
            packetizer.source.connect(scrambler.sink),
            If(phy.init.ready,
                If(scrambler.source.valid,
                    phy.serdes.tx_k.eq(scrambler.source.k),
                    phy.serdes.tx_d.eq(scrambler.source.d)
                ),
                scrambler.source.ready.eq(phy.serdes.tx_ready)
            ),

            # phy --> core
            descrambler.sink.valid.eq(phy.init.ready & phy.serdes.rx_valid),
            descrambler.sink.k.eq(phy.serdes.rx_k),
            descrambler.sink.d.eq(phy.serdes.rx_d),
            descrambler.source.connect(depacketizer.sink),

            # etherbone <--> core
            depacketizer.source.connect(etherbone.sink),
            etherbone.source.connect(packetizer.sink)
        ]
