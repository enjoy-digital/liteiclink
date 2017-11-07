from litex.gen import *

from litex.soc.interconnect import stream

from liteiclink.serwb.packet import Depacketizer, Packetizer
from liteiclink.serwb.etherbone import Etherbone


class SERWBCore(Module):
    def __init__(self, phy, clk_freq, mode):
        self.submodules.etherbone = etherbone = Etherbone(mode)
        depacketizer = Depacketizer(clk_freq)
        packetizer = Packetizer()
        self.submodules += depacketizer, packetizer
        tx_cdc = stream.AsyncFIFO([("data", 32)], 8)
        tx_cdc = ClockDomainsRenamer({"write": "sys", "read": "serwb_serdes"})(tx_cdc)
        self.submodules += tx_cdc
        rx_cdc = stream.AsyncFIFO([("data", 32)], 8)
        rx_cdc = ClockDomainsRenamer({"write": "serwb_serdes", "read": "sys"})(rx_cdc)
        self.submodules += rx_cdc
        self.comb += [
            # core <--> etherbone
            depacketizer.source.connect(etherbone.sink),
            etherbone.source.connect(packetizer.sink),

            # core --> serdes
            packetizer.source.connect(tx_cdc.sink),
            If(tx_cdc.source.valid & phy.init.ready,
                phy.serdes.tx_data.eq(tx_cdc.source.data)
            ),
            tx_cdc.source.ready.eq(phy.init.ready),

            # serdes --> core
            rx_cdc.sink.valid.eq(phy.init.ready),
            rx_cdc.sink.data.eq(phy.serdes.rx_data),
            rx_cdc.source.connect(depacketizer.sink),
        ]
