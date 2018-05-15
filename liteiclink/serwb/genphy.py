from migen import *
from migen.genlib.io import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.misc import BitSlip, WaitTimer

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.serwb.scrambler import Scrambler, Descrambler


def K(x, y):
    return (y << 5) | x


@ResetInserter()
class _Serdes(Module):
    def __init__(self, pads, mode="master"):
        if mode == "slave":
            self.refclk = Signal()

        self.tx_ce = Signal()
        self.tx_k = Signal(4)
        self.tx_d = Signal(32)

        self.rx_ce = Signal()
        self.rx_k = Signal(4)
        self.rx_d = Signal(32)

        self.tx_idle = Signal()
        self.tx_comma = Signal()
        self.rx_idle = Signal()
        self.rx_comma = Signal()

        self.rx_bitslip_value = Signal(6)

        # # #

        self.submodules.encoder = encoder = CEInserter()(Encoder(4, True))
        self.comb += encoder.ce.eq(self.tx_ce)
        self.submodules.decoders = decoders = [CEInserter()(Decoder(True)) for _ in range(4)]
        self.comb += [decoders[i].ce.eq(self.rx_ce) for i in range(4)]

        # tx clock
        if mode == "master":
            clk_o = Signal()
            self.specials += [
                DDROutput(0, 1, clk_o),
                DifferentialOutput(clk_o, pads.clk_p, pads.clk_n)
           ]

        # tx datapath
        # tx_data -> encoders -> converter -> serdes
        self.submodules.tx_converter = tx_converter = stream.Converter(40, 1)
        self.comb += [
            tx_converter.sink.valid.eq(1),
            self.tx_ce.eq(tx_converter.sink.ready),
            tx_converter.source.ready.eq(1),
            If(self.tx_idle,
                tx_converter.sink.data.eq(0)
            ).Else(
                tx_converter.sink.data.eq(
                    Cat(*[encoder.output[i] for i in range(4)]))
            ),
            If(self.tx_comma,
                encoder.k[0].eq(1),
                encoder.d[0].eq(K(28,5)),
            ).Else(
                encoder.k[0].eq(self.tx_k[0]),
                encoder.k[1].eq(self.tx_k[1]),
                encoder.k[2].eq(self.tx_k[2]),
                encoder.k[3].eq(self.tx_k[3]),
                encoder.d[0].eq(self.tx_d[0:8]),
                encoder.d[1].eq(self.tx_d[8:16]),
                encoder.d[2].eq(self.tx_d[16:24]),
                encoder.d[3].eq(self.tx_d[24:32])
            )
        ]

        serdes_o = Signal()
        self.sync += serdes_o.eq(tx_converter.source.data)
        self.specials += DifferentialOutput(serdes_o, pads.tx_p, pads.tx_n)

        # rx clock
        if mode == "slave":
            self.specials += DifferentialInput(pads.clk_p, pads.clk_n, self.refclk)

        # rx datapath
        # serdes -> converter -> bitslip -> decoders -> rx_data
        self.submodules.rx_converter = rx_converter = stream.Converter(1, 40)
        self.comb += [
            self.rx_ce.eq(rx_converter.source.valid),
            rx_converter.source.ready.eq(1)
        ]
        self.submodules.rx_bitslip = rx_bitslip = CEInserter()(BitSlip(40))
        self.comb += rx_bitslip.ce.eq(self.rx_ce)

        serdes_i = Signal()
        serdes_q = Signal()
        self.specials += DifferentialInput(pads.rx_p, pads.rx_n, serdes_i)
        self.sync += serdes_q.eq(serdes_i)

        self.comb += [
            rx_converter.sink.valid.eq(1),
            rx_converter.sink.data.eq(serdes_q),
            rx_bitslip.value.eq(self.rx_bitslip_value),
            rx_bitslip.i.eq(rx_converter.source.data),
            decoders[0].input.eq(rx_bitslip.o[0:10]),
            decoders[1].input.eq(rx_bitslip.o[10:20]),
            decoders[2].input.eq(rx_bitslip.o[20:30]),
            decoders[3].input.eq(rx_bitslip.o[30:40]),
            self.rx_k.eq(Cat(*[decoders[i].k for i in range(4)])),
            self.rx_d.eq(Cat(*[decoders[i].d for i in range(4)])),
            self.rx_comma.eq(
                (decoders[0].k == 1) & (decoders[0].d == K(28,5)) &
                (decoders[1].k == 0) & (decoders[1].d == 0) &
                (decoders[2].k == 0) & (decoders[2].d == 0) &
                (decoders[3].k == 0) & (decoders[3].d == 0))
        ]

        idle_timer = WaitTimer(256)
        self.submodules += idle_timer
        self.comb += idle_timer.wait.eq(1)
        self.sync += self.rx_idle.eq(idle_timer.done &
            ((rx_bitslip.o == 0) | (rx_bitslip.o == (2**40-1))))


# SERWB Master <--> Slave physical synchronization process:
# 1) Master sends idle patterns (zeroes) to Slave to reset it.
# 2) Master sends K28.5 commas to allow Slave to calibrate, Slave sends idle patterns.
# 3) Slave sends K28.5 commas to allow Master to calibrate, Master sends K28.5 commas.
# 4) Master stops sending K28.5 commas.
# 5) Slave stops sending K28.5 commas.
# 6) Physical link is ready.


@ResetInserter()
class _SerdesMasterInit(Module):
    def __init__(self, serdes, timeout):
        self.ready = Signal()
        self.error = Signal()

        # # #

        self.bitslip = bitslip = Signal(max=40)

        self.submodules.timer = timer = WaitTimer(timeout)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(bitslip, 0),
            NextState("RESET_SLAVE"),
            serdes.tx_idle.eq(1)
        )
        fsm.act("RESET_SLAVE",
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("SEND_PATTERN")
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("SEND_PATTERN",
            If(~serdes.rx_idle,
                timer.wait.eq(1),
                If(timer.done,
                    NextState("CHECK_PATTERN")
                )
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("WAIT_STABLE",
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK_PATTERN")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("CHECK_PATTERN",
            If(serdes.rx_comma,
                timer.wait.eq(1),
                If(timer.done,
                    NextState("READY")
                )
            ).Else(
                NextState("INC_BITSLIP")
            ),
            serdes.tx_comma.eq(1)
        )
        self.comb += serdes.rx_bitslip_value.eq(bitslip)
        fsm.act("INC_BITSLIP",
            NextState("WAIT_STABLE"),
            If(bitslip == (40 - 1),
                NextState("ERROR")
            ).Else(
                NextValue(bitslip, bitslip + 1)
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("READY",
            self.ready.eq(1)
        )
        fsm.act("ERROR",
            self.error.eq(1)
        )


@ResetInserter()
class _SerdesSlaveInit(Module, AutoCSR):
    def __init__(self, serdes, timeout):
        self.ready = Signal()
        self.error = Signal()

        # # #

        self.bitslip = bitslip = Signal(max=40)

        self.submodules.timer = timer = WaitTimer(timeout)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        # reset
        fsm.act("IDLE",
            NextValue(bitslip, 0),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("WAIT_STABLE"),
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("WAIT_STABLE",
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK_PATTERN")
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("CHECK_PATTERN",
            If(serdes.rx_comma,
                timer.wait.eq(1),
                If(timer.done,
                    NextState("SEND_PATTERN")
                )
            ).Else(
                NextState("INC_BITSLIP")
            ),
            serdes.tx_idle.eq(1)
        )
        self.comb += serdes.rx_bitslip_value.eq(bitslip)
        fsm.act("INC_BITSLIP",
            NextState("WAIT_STABLE"),
            If(bitslip == (40 - 1),
                NextState("ERROR")
            ).Else(
                NextValue(bitslip, bitslip + 1)
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("SEND_PATTERN",
            timer.wait.eq(1),
            If(timer.done,
                If(~serdes.rx_comma,
                    NextState("READY")
                )
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("READY",
            self.ready.eq(1)
        )
        fsm.act("ERROR",
            self.error.eq(1)
        )


class _SerdesControl(Module, AutoCSR):
    def __init__(self, serdes, init, mode="master"):
        if mode == "master":
            self.reset = CSR()
        self.ready = CSRStatus()
        self.error = CSRStatus()

        self.bitslip = CSRStatus(6)

        self.scrambling_enable = CSRStorage()

        self.prbs_error = Signal()
        self.prbs_start = CSR()
        self.prbs_cycles = CSRStorage(32)
        self.prbs_errors = CSRStatus(32)

        # # #

        if mode == "master":
            # In Master mode, reset is coming from CSR,
            # it resets the Master that will also reset
            # the Slave by putting the link in idle.
            self.sync += init.reset.eq(self.reset.re)
        else:
            # In Slave mode, reset is coming from link,
            # Master reset the Slave by putting the link
            # in idle.
            self.sync += [
                init.reset.eq(serdes.rx_idle),
                serdes.reset.eq(serdes.rx_idle)
            ]
        self.comb += [
            self.ready.status.eq(init.ready),
            self.error.status.eq(init.error),
            self.bitslip.status.eq(init.bitslip)
        ]

        # prbs
        prbs_cycles = Signal(32)
        prbs_errors = self.prbs_errors.status
        prbs_fsm = FSM(reset_state="IDLE")
        self.submodules += prbs_fsm
        prbs_fsm.act("IDLE",
            NextValue(prbs_cycles, 0),
            If(self.prbs_start.re,
                NextValue(prbs_errors, 0),
                NextState("CHECK")
            )
        )
        prbs_fsm.act("CHECK",
            NextValue(prbs_cycles, prbs_cycles + 1),
            If(self.prbs_error,
                NextValue(prbs_errors, prbs_errors + 1),
            ),
            If(prbs_cycles == self.prbs_cycles.storage,
                NextState("IDLE")
            )
        )


class SERWBPHY(Module, AutoCSR):
    def __init__(self, pads, mode="master", init_timeout=2**16):
        self.sink = sink = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32)])
        assert mode in ["master", "slave"]
        self.submodules.serdes = _Serdes(pads, mode)
        if mode == "master":
            self.submodules.init = _SerdesMasterInit(self.serdes, init_timeout)
        else:
            self.submodules.init = _SerdesSlaveInit(self.serdes, init_timeout)
        self.submodules.control = _SerdesControl(self.serdes, self.init, mode)

        # scrambling
        scrambler =  Scrambler()
        descrambler = Descrambler()
        self.submodules += scrambler, descrambler
        self.comb += [
            scrambler.enable.eq(self.control.scrambling_enable.storage),
            descrambler.enable.eq(self.control.scrambling_enable.storage)
        ]

        # tx dataflow
        self.comb += \
            If(self.init.ready,
                sink.connect(scrambler.sink),
                scrambler.source.ready.eq(self.serdes.tx_ce),
                If(scrambler.source.valid,
                    self.serdes.tx_d.eq(scrambler.source.d),
                    self.serdes.tx_k.eq(scrambler.source.k)
                )
            )

        # rx dataflow
        self.comb += [
            If(self.init.ready,
                descrambler.sink.valid.eq(self.serdes.rx_ce),
                descrambler.sink.d.eq(self.serdes.rx_d),
                descrambler.sink.k.eq(self.serdes.rx_k),
                descrambler.source.connect(source)
            ),
            # For PRBS test we are using the scrambler/descrambler as PRBS,
            # sending 0 to the scrambler and checking that descrambler
            # output is always 0.
            self.control.prbs_error.eq(
                descrambler.source.valid &
                descrambler.source.ready &
                (descrambler.source.data != 0))
        ]
