from litex.gen import *

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

        # clock domain crossing
        tx_cdc = stream.AsyncFIFO([("data", 32)], 32) # FIXME: reduce to minimum?
        tx_cdc = ClockDomainsRenamer({"write": "sys", "read": "serwb_serdes"})(tx_cdc)
        rx_cdc = stream.AsyncFIFO([("data", 32)], 32) # FIXME: reduce to minimum?
        rx_cdc = ClockDomainsRenamer({"write": "serwb_serdes", "read": "sys"})(rx_cdc)
        self.submodules += tx_cdc, rx_cdc

        # scrambling
        scrambler =  ClockDomainsRenamer("serwb_serdes")(Scrambler(enable=with_scrambling))
        descrambler = ClockDomainsRenamer("serwb_serdes")(Descrambler(enable=with_scrambling))
        self.submodules += scrambler, descrambler

        # modules connection
        self.comb += [
            # etherbone <--> core
            depacketizer.source.connect(etherbone.sink),
            etherbone.source.connect(packetizer.sink),

            # core --> phy
            packetizer.source.connect(tx_cdc.sink),
            tx_cdc.source.connect(scrambler.sink),
            If(phy.init.ready,
                If(scrambler.source.valid,
                    phy.serdes.tx_k.eq(scrambler.source.k),
                    phy.serdes.tx_d.eq(scrambler.source.d)
                ),
                scrambler.source.ready.eq(1)
            ),

            # phy --> core
            descrambler.sink.valid.eq(phy.init.ready),
            descrambler.sink.k.eq(phy.serdes.rx_k),
            descrambler.sink.d.eq(phy.serdes.rx_d),
            descrambler.source.connect(rx_cdc.sink),
            rx_cdc.source.connect(depacketizer.sink),
        ]
