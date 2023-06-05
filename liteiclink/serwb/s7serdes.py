#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import BitSlip, WaitTimer

from litex.build.io import *

from litex.soc.interconnect     import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.serwb.datapath import TXDatapath, RXDatapath

# S7 SerDes Clocking -------------------------------------------------------------------------------

class _S7SerdesClocking(LiteXModule):
    def __init__(self, pads, mode="master", data_width=8):
        assert data_width in [4,6,8,10,14] # valid serdese2 ddr rates
        if data_width>=10:
            assert hasattr(pads, "tx_p"), "Width expansion can only be used for differential outputs"
        self.refclk = Signal()

        # # #

        master_shiftin1  = Signal()
        master_shiftin2  = Signal()
        # Master Mode.
        # ------------
        # Generate the linerate/10 clock. Slave will re-multiply it.
        if mode == "master":
            self.converter = converter = stream.Converter(40, data_width)
            self.comb += [
                converter.sink.valid.eq(1),
                converter.source.ready.eq(1),
                converter.sink.data.eq(Replicate(Signal(10, reset=0b1111100000), 4)),
            ]
            self.specials += [
                Instance("OSERDESE2",
                    p_DATA_WIDTH     = data_width,
                    p_TRISTATE_WIDTH = 1,
                    p_DATA_RATE_OQ   = "DDR",
                    p_DATA_RATE_TQ   = "BUF",
                    p_SERDES_MODE    = "MASTER",

                    i_OCE    = 1,
                    i_RST    = ResetSignal("sys"),
                    i_CLK    = ClockSignal(f"sys{data_width//2}x"),
                    i_CLKDIV = ClockSignal("sys"),
                    **{f"i_D{i+1}" : converter.source.data[i] for i in range(min(data_width,8))},
                    i_SHIFTIN1 = master_shiftin1,
                    i_SHIFTIN2 = master_shiftin2,
                    o_OQ     = self.refclk,
                )
            ]
            if data_width>=10:
                slave_shiftout1 = Signal()
                slave_shiftout2 = Signal()
                self.specials += [
                    Instance("OSERDESE2",
                        p_DATA_WIDTH     = data_width,
                        p_TRISTATE_WIDTH = 1,
                        p_DATA_RATE_OQ   = "DDR",
                        p_DATA_RATE_TQ   = "BUF",
                        p_SERDES_MODE    = "SLAVE",

                        i_OCE    = 1,
                        i_RST    = ResetSignal("sys"),
                        i_CLK    = ClockSignal(f"sys{data_width//2}x"),
                        i_CLKDIV = ClockSignal("sys"),
                        **{f"i_D{i+3}" : converter.source.data[i+8] for i in range(data_width-8)}, #d3-d8 see UG471:168
                        o_SHIFTOUT1 = slave_shiftout1,
                        o_SHIFTOUT2 = slave_shiftout2,
                    )
                ]
                self.comb += [master_shiftin1.eq(slave_shiftout1),
                              master_shiftin2.eq(slave_shiftout2)]
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialOutput(self.refclk, pads.clk_p, pads.clk_n)
            else:
                self.comb += pads.clk.eq(self.refclk)

        # Slave Mode.
        # -----------
        # Multiply the clock provided by Master with a PLL/MMCM.
        if mode == "slave":
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialInput(pads.clk_p, pads.clk_n, self.refclk)
            else:
                self.comb += self.refclk.eq(pads.clk)

# S7 SerDes TX -------------------------------------------------------------------------------------

class _S7SerdesTX(LiteXModule):
    def __init__(self, pads, data_width=8):
        assert data_width in [4,6,8,10,14] # valid serdese2 ddr rates
        if data_width>=10:
            assert hasattr(pads, "tx_p"), "Width expansion can only be used for differential outputs"

        # Control
        self.idle  = idle  = Signal()
        self.comma = comma = Signal()

        # Datapath.
        self.sink = sink = stream.Endpoint([("data", 32)])

        # # #


        # Datapath.
        # ---------
        self.datapath = datapath = TXDatapath(data_width)
        self.comb += [
            sink.connect(datapath.sink),
            datapath.source.ready.eq(1),
            datapath.idle.eq(idle),
            datapath.comma.eq(comma),
        ]

        # Output Data (DDR with sys4x).
        # -----------------------------
        self.data = data = Signal(data_width)
        data_serialized  = Signal()
        master_shiftin1  = Signal()
        master_shiftin2  = Signal()
        self.comb += data.eq(datapath.source.data)
        self.specials += [
            Instance("OSERDESE2",
                p_DATA_WIDTH     = data_width,
                p_TRISTATE_WIDTH = 1,
                p_DATA_RATE_OQ   = "DDR",
                p_DATA_RATE_TQ   = "BUF",
                p_SERDES_MODE    = "MASTER",

                i_OCE    = 1,
                i_RST    = ResetSignal("sys"),
                i_CLK    = ClockSignal(f"sys{data_width//2}x"),
                i_CLKDIV = ClockSignal("sys"),
                **{f"i_D{i+1}" : data[i] for i in range(min(data_width,8))},
                i_SHIFTIN1 = master_shiftin1,
                i_SHIFTIN2 = master_shiftin2,
                o_OQ     = data_serialized,
            )
        ]
        if data_width>=10:
            slave_shiftout1 = Signal()
            slave_shiftout2 = Signal()
            self.specials += [
                Instance("OSERDESE2",
                    p_DATA_WIDTH     = data_width,
                    p_TRISTATE_WIDTH = 1,
                    p_DATA_RATE_OQ   = "DDR",
                    p_DATA_RATE_TQ   = "BUF",
                    p_SERDES_MODE    = "SLAVE",

                    i_OCE    = 1,
                    i_RST    = ResetSignal("sys"),
                    i_CLK    = ClockSignal(f"sys{data_width//2}x"),
                    i_CLKDIV = ClockSignal("sys"),
                    **{f"i_D{i+3}" : data[i+8] for i in range(data_width-8)}, #d3-d8 see UG471:168
                    o_SHIFTOUT1 = slave_shiftout1,
                    o_SHIFTOUT2 = slave_shiftout2,
                )
            ]
            self.comb += [master_shiftin1.eq(slave_shiftout1),
                          master_shiftin2.eq(slave_shiftout2)]
        
        if hasattr(pads, "tx_p"):
            self.specials += DifferentialOutput(data_serialized, pads.tx_p, pads.tx_n)
        else:
            self.comb += pads.tx.eq(data_serialized)

# S7 SerDes RX -------------------------------------------------------------------------------------

class _S7SerdesRX(LiteXModule):
    def __init__(self, pads, data_width=8):
        assert data_width in [4,6,8,10,14] # valid serdese2 ddr rates
        if data_width>=10:
            assert hasattr(pads, "rx_p"), "Width expansion can only be used for differential outputs"
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

        _shift = Signal(max=data_width)
        self.sync += If(self.shift_inc, _shift.eq(_shift + 1))

        # Data input (DDR with sys4x).
        # ----------------------------
        data_nodelay      = Signal()
        data_delayed      = Signal()
        master_shiftout1  = Signal()
        master_shiftout2  = Signal()
        self.data = data  = Signal(data_width)
        
        if hasattr(pads, "rx_p"):
            self.specials += DifferentialInput(pads.rx_p, pads.rx_n, data_nodelay)
        else:
            self.comb += data_nodelay.eq(pads.rx)

        self.specials += [
            Instance("IDELAYE2",
                p_DELAY_SRC             = "IDATAIN",
                p_SIGNAL_PATTERN        = "DATA",
                p_CINVCTRL_SEL          = "FALSE",
                p_HIGH_PERFORMANCE_MODE = "TRUE",
                p_REFCLK_FREQUENCY      = 200.0,
                p_PIPE_SEL              = "FALSE",
                p_IDELAY_TYPE           = "VARIABLE",
                p_IDELAY_VALUE          = 0,

                i_C        = ClockSignal(),
                i_LD       = delay_rst,
                i_CE       = delay_inc,
                i_LDPIPEEN = 0,
                i_INC      = 1,
                i_IDATAIN  = data_nodelay,
                o_DATAOUT  = data_delayed,
            ),
            Instance("ISERDESE2",
                p_DATA_WIDTH     = data_width,
                p_DATA_RATE      = "DDR",
                p_SERDES_MODE    = "MASTER",
                p_INTERFACE_TYPE = "NETWORKING",
                p_NUM_CE         = 1,
                p_IOBDELAY       = "IFD",

                i_DDLY    = data_delayed,
                i_CE1     = 1,
                i_RST     = ResetSignal("sys"),
                i_CLK     = ClockSignal(f"sys{data_width//2}x"),
                i_CLKB    =~ClockSignal(f"sys{data_width//2}x"),
                i_CLKDIV  = ClockSignal("sys"),
                i_BITSLIP = shift_inc,
                **{f"o_Q{i+1}" : data[data_width-i-1] for i in range(min(8,data_width))},
                o_SHIFTOUT1 = master_shiftout1,
                o_SHIFTOUT2 = master_shiftout2,
            )
        ]
        if data_width>=10:
            slave_shiftin1 = Signal()
            slave_shiftin2 = Signal()
            remaining_bits = data_width-8
            self.specials += [
                Instance("ISERDESE2",
                p_DATA_WIDTH     = data_width,
                p_DATA_RATE      = "DDR",
                p_SERDES_MODE    = "SLAVE",
                p_INTERFACE_TYPE = "NETWORKING",
                p_NUM_CE         = 1,
                p_IOBDELAY       = "IFD",

                i_CE1     = 1,
                i_RST     = ResetSignal("sys"),
                i_CLK     = ClockSignal(f"sys{data_width//2}x"),
                i_CLKB    =~ClockSignal(f"sys{data_width//2}x"),
                i_CLKDIV  = ClockSignal("sys"),
                i_BITSLIP = shift_inc,
                i_SHIFTIN1 = slave_shiftin1,
                i_SHIFTIN2 = slave_shiftin2,
                **{f"o_Q{i+3}" : data[remaining_bits-i-1] for i in range(remaining_bits)}, #d3-d8 see UG471:155
                )
            ]
            self.comb += [slave_shiftin1.eq(master_shiftout1),
                          slave_shiftin2.eq(master_shiftout2)]

        # Datapath.
        # ---------
        self.datapath = datapath = RXDatapath(data_width)
        self.comb += [
            datapath.sink.valid.eq(1),
            datapath.sink.data.eq(data),
            datapath.shift_inc.eq(self.shift_inc & (_shift == data_width-1)),
            datapath.source.connect(source),
            idle.eq(datapath.idle),
            comma.eq(datapath.comma),
        ]

# S7 SerDes ----------------------------------------------------------------------------------------

@ResetInserter()
class S7Serdes(LiteXModule):
    def __init__(self, pads, mode="master", data_width=8):
        assert mode in ["master", "slave"]
        self.clocking = _S7SerdesClocking(pads, mode, data_width)
        self.tx       = _S7SerdesTX(pads, data_width)
        self.rx       = _S7SerdesRX(pads, data_width)
