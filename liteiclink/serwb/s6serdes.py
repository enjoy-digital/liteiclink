#
# This file is part of LiteICLink.
#
# Copyright (c) 2023 Christian Klarhorst <cklarhor@techfak.uni-bielefeld.de>
# Copyright (c) 2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import BitSlip, WaitTimer

from litex.build.io import *

from litex.soc.interconnect     import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.serwb.datapath import TXDatapath, RXDatapath

# S6 SerDes Clocking -------------------------------------------------------------------------------

class _S6SerdesClockingSimple(LiteXModule):
    def __init__(self, pads, mode="master"):
        self.refclk = Signal()

        # In Master mode, generate the clock with 180° phase shift so that Slave can use this clock
        # to sample data.
        if mode == "master":
            self.specials += DDROutput(0, 1, self.refclk)
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialOutput(self.refclk, pads.clk_p, pads.clk_n)
            else:
                self.comb += pads.clk.eq(self.refclk)

        # In Slave mode, use the clock provided by Master.
        elif mode == "slave":
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialInput(pads.clk_p, pads.clk_n, self.refclk)
            else:
                self.comb += self.refclk.eq(pads.clk)
        else: # Unknown mode
            raise ValueError

class _S6SerdesClocking(LiteXModule):
    def __init__(self, pads, mode="master", data_width=4):
        assert data_width in [1,2,3,4] # valid serdes2 sdr rates
        self.refclk = Signal()

        self.serdes_clk = Signal()
        self.serdes_strobe = Signal()

        # In Master mode, generate the linerate/10 clock. Slave will re-multiply it.
        if mode == "master":
            self.converter = converter = stream.Converter(40, data_width)
            self.comb += [
                converter.sink.valid.eq(1),
                converter.source.ready.eq(1),
                converter.sink.data.eq(Replicate(Signal(10, reset=0b1111100000), 4)),
            ]
            self.test = Signal()
            self.sync += [self.test.eq(~self.test)]
            self.specials += [
                Instance("OSERDES2",
                    p_DATA_WIDTH     = data_width,
                    p_DATA_RATE_OQ   = "SDR",
                    p_DATA_RATE_OT   = "SDR",
                    p_SERDES_MODE    = "None",
                    p_OUTPUT_MODE    = "SINGLE_ENDED",

                    i_OCE    = 1,
                    i_IOCE   = ClockSignal("strobe"),
                    i_RST    = 0,
                    i_CLK0   = ClockSignal(f"sys{data_width}x"),
                    i_CLK1   = 0,
                    i_CLKDIV = ClockSignal("sys"),
                    **{f"i_D{i+1}" : converter.source.data[i] for i in range(data_width)},
                    i_TRAIN  = 0,
                    i_TCE=0,
                    i_SHIFTIN1=0,
                    i_SHIFTIN2=0,
                    i_SHIFTIN3=0,
                    i_SHIFTIN4=0,
                    o_OQ     = self.refclk,
                )
            ]
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialOutput(self.refclk, pads.clk_p, pads.clk_n)
            else:
                self.comb += self.refclk.eq(pads.clk)

        # In Slave mode, multiply the clock provided by Master with a PLL/MMCM.
        elif mode == "slave":
            if hasattr(pads, "clk_p"):
                self.specials += DifferentialInput(pads.clk_p, pads.clk_n, self.refclk)
            else:
                self.comb += self.refclk.eq(pads.clk)

# S6 SerDes TX -------------------------------------------------------------------------------------

class _S6SerdesTX(LiteXModule):
    def __init__(self, pads, data_width=4):
        assert data_width in [1,2,3,4] # valid serdes2 sdr rates
        # Control
        self.idle  = idle  = Signal()
        self.comma = comma = Signal()

        # Datapath
        self.sink = sink = stream.Endpoint([("data", 32)])

        self.datapath = datapath = TXDatapath(data_width)
        self.comb += [
            sink.connect(datapath.sink),
            datapath.source.ready.eq(1),
            datapath.idle.eq(idle),
            datapath.comma.eq(comma)
        ]

        # Data output
        self.data = data = Signal(data_width)
        data_serialized  = Signal()
        self.comb += data.eq(datapath.source.data)
        self.specials += [
            Instance("OSERDES2",
                p_DATA_WIDTH     = data_width,
                p_DATA_RATE_OQ   = "SDR",
                p_DATA_RATE_OT   = "SDR",
                p_SERDES_MODE    = "NONE",
                p_OUTPUT_MODE    = "SINGLE_ENDED",

                i_OCE    = 1,
                i_IOCE   = ClockSignal("strobe"),
                i_RST    = 0,
                i_CLK0   = ClockSignal(f"sys{data_width}x"),
                i_CLK1   = 0,
                i_CLKDIV = ClockSignal("sys"),
                **{f"i_D{i+1}" : data[i] for i in range(data_width)},
                i_TRAIN  = 0,
                i_TCE=0,
                i_SHIFTIN1=0,
                i_SHIFTIN2=0,
                i_SHIFTIN3=0,
                i_SHIFTIN4=0,
                o_OQ     = data_serialized,
            )
        ]
        if hasattr(pads, "tx_p"):
            self.specials += DifferentialOutput(data_serialized, pads.tx_p, pads.tx_n)
        else:
            self.comb += pads.tx.eq(data_serialized)

# S6 SerDes RX -------------------------------------------------------------------------------------

class _S6SerdesRX(LiteXModule):
    def __init__(self, pads, data_width=4):
        assert data_width in [1,2,3,4] # valid serdes2 sdr rates
        # Control
        self.delay_rst = delay_rst  = Signal()
        self.delay_inc = delay_inc  = Signal()
        self.shift_inc     = shift_inc  = Signal()

        # Status
        self.idle  = idle = Signal()
        self.comma = comma = Signal()

        # Datapath
        self.source = source = stream.Endpoint([("data", 32)])

        _shift = Signal(max=data_width)
        self.sync += If(self.shift_inc, _shift.eq(_shift + 1))

        # Data input
        data_nodelay      = Signal()
        data_delayed      = Signal()
        self.data = data  = Signal(data_width)

        if hasattr(pads, "rx_p"):
            self.specials += DifferentialInput(pads.rx_p, pads.rx_n, data_nodelay)
        else:
            self.comb += data_nodelay.eq(pads.rx)

        self.specials += [
            Instance("IODELAY2",
                p_DELAY_SRC             = "IDATAIN",
                p_DATA_RATE             = "SDR",
                p_IDELAY_TYPE           = "VARIABLE_FROM_ZERO",
                p_IDELAY_VALUE          = 0,
                p_IDELAY2_VALUE         = 0,
                p_IDELAY_MODE           = "NORMAL",
                p_ODELAY_VALUE          = 0,
                p_COUNTER_WRAPAROUND    = "STAY_AT_LIMIT",
                p_SERDES_MODE           = "NONE",

                i_T        = 1,
                i_ODATAIN  = 0,
                i_CAL      = ResetSignal("sys"), # calibrate once on reset
                i_CLK      = ClockSignal("sys"),
                i_IOCLK0   = ClockSignal(f"sys{data_width}x"),
                i_IOCLK1   = 0,
                i_RST      = delay_rst,
                i_CE       = delay_inc,
                i_INC      = 1,
                i_IDATAIN  = data_nodelay,
                o_DATAOUT  = data_delayed,
            ),
            Instance("ISERDES2",
                p_BITSLIP_ENABLE = 1,
                p_DATA_WIDTH     = data_width,
                p_DATA_RATE      = "SDR",
                p_SERDES_MODE    = "NONE",
                p_INTERFACE_TYPE = "RETIMED",

                i_CLK0    = ClockSignal(f"sys{data_width}x"),
                i_CLK1    = 0,
                i_CLKDIV  = ClockSignal("sys"),
                i_CE0     = 1,
                i_BITSLIP = shift_inc,
                i_D       = data_delayed,
                i_RST     = ResetSignal("sys"),
                i_IOCE    = ClockSignal("strobe"),
                i_SHIFTIN=0,
                **{f"o_Q{i+1}" : data[i] for i in range(data_width)},
            )
        ]

        # Datapath
        self.datapath = datapath = RXDatapath(data_width)
        self.comb += [
            datapath.sink.valid.eq(1),
            datapath.sink.data.eq(data),
            datapath.shift_inc.eq(self.shift_inc & (_shift == data_width-1)),
            datapath.source.connect(source),
            idle.eq(datapath.idle),
            comma.eq(datapath.comma)
        ]

# S6 SerDes ----------------------------------------------------------------------------------------

@ResetInserter()
class S6Serdes(LiteXModule):
    def __init__(self, pads, mode="master", data_width=4, simple_clk=True):
        if simple_clk:
            self.clocking = _S6SerdesClockingSimple(pads, mode)
        else:
            self.clocking = _S6SerdesClocking(pads, mode, data_width)
        self.tx       = _S6SerdesTX(pads, data_width)
        self.rx       = _S6SerdesRX(pads, data_width)
