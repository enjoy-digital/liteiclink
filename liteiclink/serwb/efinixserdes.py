#
# This file is part of LiteICLink.
#
# Copyright (c) 2023 MoTeC
# Copyright (c) 2023 Gwenhael Goavec-Merou <gwenhael@enjoy-digital.fr>
# Copyright (c) 2017-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.cores.clock.efinix import TITANIUMPLL,TRIONPLL

from litex.soc.interconnect import stream

from liteiclink.serwb.datapath import TXDatapath, RXDatapath

#from liteiclink.serwb.phy import _SerdesControl
#from liteiclink.serwb.phy import _SerdesMasterInit
#from liteiclink.serwb.phy import _SerdesSlaveInit

# BitSlip ------------------------------------------------------------------------------------------

class BitSlip(Module):
    def __init__(self, dw, rst=None, inc=None, cycles=1):
        self.i = Signal(dw)
        self.o = Signal(dw)
        self.rst = Signal() if rst is None else rst
        self.inc = Signal() if inc is None else inc

        # # #

        self.value = value = Signal(max=(cycles+1)*dw)
        self.sync += [
            If(self.inc,
                value.eq(value + 1),
                If(value == (dw - 1),
                    value.eq(0)
                )
            )
        ]
        self.sync += If(self.rst, value.eq(0))

        self.r = r = Signal((cycles+1)*dw, reset_less=True)
        self.sync += r.eq(Cat(r[dw:], self.i))
        cases = {}
        for i in range(cycles*dw):
            cases[i] = self.o.eq(r[i:dw+i])
        self.comb += Case(value, cases)

# Efinix Serdes Diff TX ----------------------------------------------------------------------------

class EfinixSerdesDiffTx8To1(LiteXModule):
    def __init__(self, data, tx_p, tx_n, slow_clk, fast_clk, platform):
        # only keep _p
        io_name = platform.get_pin_name(tx_p)
        io_pad  = platform.get_pad_name(tx_p) # need real pad name
        io_prop = platform.get_pin_properties(tx_p)

        _data = platform.add_iface_io(io_name + "_gen", 8)
        _oe   = platform.add_iface_io(io_name + "_oe")
        _rst  = platform.add_iface_io(io_name + "_rst")

        assert platform.family in ["Titanium", "Trion"]
        if platform.family == "Titanium":
            # _p has _P_ and _n has _N_ followed by an optional function
            # lvds block needs _PN_
            pad_split = io_pad.split('_')
            assert pad_split[1] == 'P'
            io_pad = f"{pad_split[0]}_PN_{pad_split[2]}"
        if platform.family == "Trion":
            assert "TXP" in io_pad
            # diff output pins are TXPYY and TXNYY
            # lvds block needs TXYY
            io_pad = io_pad.replace("TXP", "TX")
            # remove alternate function to only keep GPIOX_TXYY
            pad_split = io_pad.split('_')
            io_pad = '_'.join(pad_split[:2])

        self.comb += [
            _data.eq(data),
            _rst.eq(0),
            _oe.eq(1),
        ]

        block = {
            "type"      : "LVDS",
            "mode"      : "OUTPUT",
            "tx_mode"   : "DATA",
            "name"      : io_name,
            "sig"       : _data,
            "location"  : io_pad,
            "size"      : 8,
            "slow_clk"  : slow_clk,
            "fast_clk"  : fast_clk,
            "half_rate" : "1",
            "oe_pin"    : _oe,
            "rst_pin"   : _rst,
        }

        platform.toolchain.ifacewriter.blocks.append(block)
        platform.toolchain.excluded_ios.append(tx_p)
        platform.toolchain.excluded_ios.append(tx_n)

# Efinix Serdes Diff RX ----------------------------------------------------------------------------

class EfinixSerdesDiffRx1To8(LiteXModule):
    def __init__(self, rx_p, rx_n, data, slow_clk, fast_clk, platform):
        self.delay_inc = Signal()
        self.delay_rst = Signal()

        # # #

        # only keep _p
        io_name = platform.get_pin_name(rx_p)
        io_pad  = platform.get_pad_name(rx_p) # need real pad name
        io_prop = platform.get_pin_properties(rx_p)

        _data = platform.add_iface_io(io_name + "_gen", 8)
        _ena  = platform.add_iface_io(io_name + "_ena")
        _rst  = platform.add_iface_io(io_name + "_rst")

        assert platform.family in ["Titanium", "Trion"]
        if platform.family == "Titanium":
            # _p has _P_ and _n has _N_ followed by an optional function
            # lvds block needs _PN_
            pad_split = io_pad.split('_')
            assert pad_split[1] == 'P'
            io_pad = f"{pad_split[0]}_PN_{pad_split[2]}"
            # no delay with Trion family
            delay_ena = platform.add_iface_io(io_name + "_delay_ena")
            delay_rst = platform.add_iface_io(io_name + "_delay_rst")
            delay_inc = platform.add_iface_io(io_name + "_delay_inc")

        if platform.family == "Trion":
            assert "RXP" in io_pad
            # diff output pins are RXPYY and RXNYY
            # lvds block needs RXYY
            io_pad = io_pad.replace("RXP", "RX")
            # remove alternate function to only keep GPIOX_RXYY
            pad_split = io_pad.split('_')
            io_pad = '_'.join(pad_split[:2])


        self.comb += [
            _rst.eq(0),
            _ena.eq(1),
            data.eq(_data),
        ]
        if platform.family == "Titanium":
            self.comb +=  [
                delay_inc.eq(1),
                delay_ena.eq(self.delay_inc),
                delay_rst.eq(self.delay_rst),
            ]
        block = {
            "type"      : "LVDS",
            "mode"      : "INPUT",
            "rx_mode"   : "NORMAL",
            "name"      : io_name,
            "sig"       : _data,
            "location"  : io_pad,
            "size"      : 8,
            "slow_clk"  : slow_clk,
            "fast_clk"  : fast_clk,
            "half_rate" : "1",
            "ena_pin"   : _ena,
            "rst_pin"   : _rst,
            "rx_delay"  : "DYNAMIC",
        }

        if platform.family == "Titanium":
            block.update({
                "delay_ena" : delay_ena,
                "delay_rst" : delay_rst,
                "delay_inc" : delay_inc,
            })

        platform.toolchain.ifacewriter.blocks.append(block)
        platform.toolchain.excluded_ios.append(rx_p)
        platform.toolchain.excluded_ios.append(rx_n)

# Efinix SerDes Clocking ---------------------------------------------------------------------------

class _EfinixSerdesClocking(LiteXModule):
    def __init__(self, pads, mode="master", platform=None):
        self.refclk = Signal()

        # # #

        # Parameters.
        # -----------
        sys_clk_freq = LiteXContext.top.sys_clk_freq
        platform     = LiteXContext.platform

        assert platform.family in ["Titanium", "Trion"]

        # Master Mode.
        # ------------
        # Generate the linerate/8 clock. Slave will re-multiply it.
        if mode == "master":
            self.submodules += EfinixSerdesDiffTx8To1(
                data = 0b11110000,
                tx_p     = pads.clk_p,
                tx_n     = pads.clk_n,
                slow_clk = "sys",
                fast_clk = "sys4x",
                platform = platform
            )

        # Slave Mode.
        # -----------
        # Multiply the clock provided by Master with a PLL.
        if mode == "slave":
            self.cd_rx_slow_clk = ClockDomain()
            self.cd_rx_fast_clk = ClockDomain()

            # PLL.
            if platform.family == "Titanium":
                self.pll = pll = TITANIUMPLL(platform)
                pll.register_clkin(pads.clk_p, sys_clk_freq, platform.get_pin_name(pads.clk_p) + "_gen", lvds_input=True)
                pll.create_clkout(None, sys_clk_freq)

            if platform.family == "Trion":
                io_name = platform.get_pin_name(pads.clk_p)
                io_pad  = platform.get_pad_name(pads.clk_p) # need real pad name
                io_prop = platform.get_pin_properties(pads.clk_p)

                if platform.device[:2] == "T2":
                    assert "CLKP" in io_pad
                    io_pad = io_pad.replace("CLKP", "CLK")

                if platform.device[:2] == "T1":
                    assert "RXP" in io_pad
                    # diff output pins are RXPYY and RXNYY
                    # lvds block needs RXYY
                    if "_CLK" in io_pad:
                        io_pad = io_pad.split("_CLK")[0]
                    io_pad = io_pad.replace("RXP", "RX")

                _data = platform.add_iface_io(io_name + "_gen")
                _ena  = platform.add_iface_io(io_name + "_ena")

                self.comb += _ena.eq(1)

                block = {
                    "type"      : "LVDS",
                    "mode"      : "INPUT",
                    "rx_mode"   : "PLL_CLKIN",
                    "name"      : io_name,
                    "sig"       : _data,
                    "ena"       : _ena,
                    "serdes"    : 0,
                    "location"  : io_pad,
                    "size"      : 0,
                    "half_rate" : "0",
                }
                platform.toolchain.ifacewriter.blocks.append(block)
                platform.toolchain.excluded_ios.append(pads.clk_n)

                self.pll = pll = TRIONPLL(platform)
                pll.register_clkin(pads.clk_p, sys_clk_freq,
                    platform.get_pin_name(pads.clk_p) + "_gen2",
                    lvds_input=True,
                    refclk_name=platform.get_pin_name(pads.clk_p) + "_gen",
                )
            pll.create_clkout(self.cd_rx_slow_clk, sys_clk_freq,             name="rx_slow_clk")
            pll.create_clkout(self.cd_rx_fast_clk, sys_clk_freq*4, phase=90, name="rx_fast_clk")

            self.comb += self.refclk.eq(self.cd_rx_slow_clk.clk)
            platform.toolchain.excluded_ios.append(pads.clk_n)

# Efinix SerDes TX ---------------------------------------------------------------------------------

class _EfinixSerdesTX(LiteXModule):
    def __init__(self, pads, slow_cd="sys", fast_cd="sys4x"):
        # Control
        self.idle  = idle  = Signal()
        self.comma = comma = Signal()

        # Datapath.
        self.sink = sink = stream.Endpoint([("data", 32)])

        # # #


        # Datapath.
        # ---------
        self.datapath = datapath = TXDatapath(8)
        self.comb += [
            sink.connect(datapath.sink),
            datapath.source.ready.eq(1),
            datapath.idle.eq(idle),
            datapath.comma.eq(comma)
        ]

        # Output Data (DDR with sys4x).
        # -----------------------------
        self.tx = tx = EfinixSerdesDiffTx8To1(
            data     = datapath.source.data,
            tx_p     = pads.tx_p,
            tx_n     = pads.tx_n,
            slow_clk = slow_cd,
            fast_clk = fast_cd,
            platform = LiteXContext.platform,
        )

# Efinix SerDes RX ---------------------------------------------------------------------------------

class _EfinixSerdesRX(LiteXModule):
    def __init__(self, pads, slow_cd="sys", fast_cd="sys4x"):
        # Control.
        self.delay_rst = delay_rst = Signal()
        self.delay_inc = delay_inc = Signal()
        self.shift_inc = shift_inc = Signal()

        # Status.
        self.idle  =  idle = Signal()
        self.comma = comma = Signal()

        # Datapath
        self.source = source = stream.Endpoint([("data", 32)])

        # # #

        _shift = Signal(3)
        self.sync += [
            If(self.shift_inc,
               _shift.eq(_shift + 1)
            )
        ]
        # Data input (DDR with sys4x).
        # ----------------------------
        self.data = data = Signal(8)
        _data = Signal(8)
        self.rx = rx = EfinixSerdesDiffRx1To8(
            rx_p     = pads.rx_p,
            rx_n     = pads.rx_n,
            data     = _data,
            slow_clk = slow_cd,
            fast_clk = fast_cd,
            platform = LiteXContext.platform,
        )
        self.comb += [
            rx.delay_inc.eq(delay_inc),
            rx.delay_rst.eq(delay_rst),
        ]

        # Datapath.
        # ---------
        self.datapath = datapath = RXDatapath(8)
        self.comb += [
            datapath.sink.valid.eq(1),
            datapath.sink.data.eq(data),
            datapath.shift_inc.eq(self.shift_inc & (_shift == 0b111)),
            datapath.source.connect(source),
            idle.eq(datapath.idle),
            comma.eq(datapath.comma)
        ]

        data_bitslip = BitSlip(8,
            inc    = self.shift_inc,
            cycles = 2)
        self.submodules += data_bitslip
        data_bitslip_o_d = Signal(8)
        self.sync += data_bitslip_o_d.eq(data_bitslip.o)
        self.comb += data_bitslip.i.eq(_data)
        self.comb += data.eq(data_bitslip_o_d)

# Efinix SerDes ------------------------------------------------------------------------------------

@ResetInserter()
class EfinixSerdes(LiteXModule):
    def __init__(self, pads, mode="master"):
        slow_cd = {"master": "sys",   "slave": "rx_slow_clk"}[mode]
        fast_cd = {"master": "sys4x", "slave": "rx_fast_clk"}[mode]
        self.clocking = _EfinixSerdesClocking(pads, mode)
        self.tx       = _EfinixSerdesTX(pads, slow_cd, fast_cd)
        self.rx       = _EfinixSerdesRX(pads, slow_cd, fast_cd)
