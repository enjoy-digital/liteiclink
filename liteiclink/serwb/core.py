from litex.gen import *

from litex.soc.interconnect import stream

from liteiclink.serwb.scrambler import Scrambler, Descrambler
from liteiclink.serwb.packet import Packetizer, Depacketizer
from liteiclink.serwb.etherbone import Etherbone


class SERWBCore(Module):
    def __init__(self, phy, clk_freq, mode):
        self.submodules.etherbone = etherbone = Etherbone(mode)
        depacketizer = Depacketizer(clk_freq)
        packetizer = Packetizer()
        self.submodules += depacketizer, packetizer
        tx_cdc = stream.AsyncFIFO([("data", 32)], 32)
        tx_cdc = ClockDomainsRenamer({"write": "sys", "read": "serwb_serdes"})(tx_cdc)
        rx_cdc = stream.AsyncFIFO([("data", 32)], 32)
        rx_cdc = ClockDomainsRenamer({"write": "serwb_serdes", "read": "sys"})(rx_cdc)
        self.submodules += tx_cdc, rx_cdc
        scrambler =  ClockDomainsRenamer("serwb_serdes")(Scrambler())
        descrambler = ClockDomainsRenamer("serwb_serdes")(Descrambler())
        self.submodules += scrambler, descrambler
        self.comb += [
            # core <--> etherbone
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
