#!/usr/bin/env python3

from litex.gen import *

from liteiclink.serwb import packet
from liteiclink.serwb import etherbone
from liteiclink.serwb import scrambler

from litex.soc.interconnect.wishbone import SRAM
from litex.soc.interconnect.stream import StrideConverter


class DUT(Module):
    def __init__(self, with_converter=False):
        # wishbone slave
        slave_depacketizer = packet.Depacketizer(int(100e6)) 
        slave_packetizer = packet.Packetizer()
        self.submodules += slave_depacketizer, slave_packetizer
        slave_descrambler = scrambler.Descrambler()
        slave_scrambler = scrambler.Scrambler()
        self.submodules += slave_descrambler, slave_scrambler
        slave_etherbone = etherbone.Etherbone(mode="slave")
        self.submodules += slave_etherbone
        self.comb += [
            slave_descrambler.source.connect(slave_depacketizer.sink),
            slave_depacketizer.source.connect(slave_etherbone.sink),
            slave_etherbone.source.connect(slave_packetizer.sink),
            slave_packetizer.source.connect(slave_scrambler.sink)
        ]

        # wishbone master
        master_depacketizer = packet.Depacketizer(int(100e6))
        master_packetizer = packet.Packetizer()
        self.submodules += master_depacketizer, master_packetizer
        master_descrambler = scrambler.Descrambler()
        master_scrambler = scrambler.Scrambler()
        self.submodules += master_descrambler, master_scrambler
        master_etherbone = etherbone.Etherbone(mode="master")
        master_sram = SRAM(1024, bus=master_etherbone.wishbone.bus)
        self.submodules += master_etherbone, master_sram
        self.comb += [
            master_descrambler.source.connect(master_depacketizer.sink),
            master_depacketizer.source.connect(master_etherbone.sink),
            master_etherbone.source.connect(master_packetizer.sink),
            master_packetizer.source.connect(master_scrambler.sink)
        ]

        if with_converter:
            s2m_downconverter = StrideConverter(
                [("d", 32), ("k", 4)],
                [("d", 16), ("k", 2)],
                reverse=False)
            s2m_upconverter = StrideConverter(
                [("d", 16), ("k", 2)],
                [("d", 32), ("k", 4)],
                reverse=False)
            self.submodules += s2m_downconverter, s2m_upconverter
            m2s_downconverter = StrideConverter(
                [("d", 32), ("k", 4)],
                [("d", 16), ("k", 2)],
                reverse=False)
            m2s_upconverter = StrideConverter(
                [("d", 16), ("k", 2)],
                [("d", 32), ("k", 4)],
                reverse=False)
            self.submodules += m2s_upconverter, m2s_downconverter
            self.comb += [
                slave_scrambler.source.connect(s2m_downconverter.sink),
                s2m_downconverter.source.connect(s2m_upconverter.sink),
                s2m_upconverter.source.connect(master_descrambler.sink),
    
                master_scrambler.source.connect(m2s_downconverter.sink),
                m2s_downconverter.source.connect(m2s_upconverter.sink),
                m2s_upconverter.source.connect(slave_descrambler.sink)
            ]
        else:
            self.comb += [
                slave_scrambler.source.connect(master_descrambler.sink),
                master_scrambler.source.connect(slave_descrambler.sink)
            ]

        # expose wishbone slave
        self.wishbone = slave_etherbone.wishbone.bus

def main_generator(dut):
    for i in range(8):
        yield from dut.wishbone.write(0x100 + i, i)
    for i in range(8):
        data = (yield from dut.wishbone.read(0x100 + i))
        print("0x{:08x}".format(data))

dut = DUT()
run_simulation(dut, main_generator(dut), vcd_name="sim.vcd")
