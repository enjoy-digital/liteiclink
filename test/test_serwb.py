#!/usr/bin/env python3

import random

from litex.gen import *

from liteiclink.serwb.core import SERWBCore

from litex.soc.interconnect.wishbone import SRAM
from litex.soc.interconnect.stream import StrideConverter


class FakeInit(Module):
    def __init__(self):
        self.ready = 1


class FakeSerdes(Module):
    def __init__(self):
        self.tx_k = Signal(4)
        self.tx_d = Signal(32)
        self.rx_k = Signal(4)
        self.rx_d = Signal(32)


class FakePHY(Module):
    cd = "sys"
    def __init__(self):
        self.init = FakeInit()
        self.serdes = FakeSerdes()


class DUT(Module):
    def __init__(self, **kwargs):
        # wishbone slave
        phy_slave = FakePHY()
        serwb_slave = SERWBCore(phy_slave, int(1e6), "slave", **kwargs)
        self.submodules += phy_slave, serwb_slave

        # wishbone master
        phy_master = FakePHY()
        serwb_master = SERWBCore(phy_master, int(1e6), "master", **kwargs)
        self.submodules += phy_master, serwb_master

        # connect phy
        self.comb += [
            phy_master.serdes.rx_k.eq(phy_slave.serdes.tx_k),
            phy_master.serdes.rx_d.eq(phy_slave.serdes.tx_d),
            phy_slave.serdes.rx_k.eq(phy_master.serdes.tx_k),
            phy_slave.serdes.rx_d.eq(phy_master.serdes.tx_d)
        ]

        # add wishbone sram to wishbone master
        sram = SRAM(1024, bus=serwb_master.etherbone.wishbone.bus)
        self.submodules += sram

        # expose wishbone slave
        self.wishbone = serwb_slave.etherbone.wishbone.bus


def main_generator(dut):
    # prepare test
    prng = random.Random(42)
    data_base = 0x100
    data_length = 4
    datas_w = [prng.randrange(2**32) for i in range(data_length)]
    datas_r = []

    # write
    for i in range(data_length):
        yield from dut.wishbone.write(data_base + i, datas_w[i])

    # read
    for i in range(data_length):
        datas_r.append((yield from dut.wishbone.read(data_base + i)))
        print("0x{:08x}".format(datas_r[i]))

    # check
    errors = 0
    for i in range(data_length):
        if datas_r[i] != datas_w[i]:
            errors += 1
    print("errors: %d" %errors)

dut = DUT(with_scrambling=False)
run_simulation(dut, main_generator(dut), vcd_name="sim.vcd")
