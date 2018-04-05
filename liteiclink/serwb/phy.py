from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import *

from liteiclink.serwb.kusphy import KUSSerdes
from liteiclink.serwb.s7phy import S7Serdes


# Master <--> Slave synchronization:
# 1) Master sends idle pattern (zeroes) to reset Slave.
# 2) Master sends K28.5 commas to allow Slave to calibrate, Slave sends idle pattern.
# 3) Slave sends K28.5 commas to allow Master to calibrate, Master sends K28.5 commas.
# 4) Master stops sending K28.5 commas.
# 5) Slave stops sending K28.5 commas.
# 6) Link is ready.

class _SerdesMasterInit(Module):
    def __init__(self, serdes, taps, timeout=2**14):
        self.reset_link = Signal()
        self.ready = Signal()
        self.error = Signal()
        self.debug = Signal(8)

        # # #

        self.delay = delay = Signal(max=taps)
        self.delay_min = delay_min = Signal(max=taps)
        self.delay_min_found = delay_min_found = Signal()
        self.delay_max = delay_max = Signal(max=taps)
        self.delay_max_found = delay_max_found = Signal()
        self.bitslip = bitslip = Signal(max=40)

        timer = WaitTimer(timeout)
        self.submodules += timer

        self.submodules.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += self.fsm.reset.eq(self.reset_link)

        self.comb += serdes.rx_delay_inc.eq(1)

        fsm.act("IDLE",
            self.debug.eq(0),
            NextValue(delay, 0),
            NextValue(delay_min, 0),
            NextValue(delay_min_found, 0),
            NextValue(delay_max, 0),
            NextValue(delay_max_found, 0),
            serdes.rx_delay_rst.eq(1),
            NextValue(bitslip, 0),
            NextState("RESET_SLAVE"),
            serdes.tx_idle.eq(1)
        )
        fsm.act("RESET_SLAVE",
            self.debug.eq(1),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("SEND_PATTERN")
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("SEND_PATTERN",
            self.debug.eq(2),
            If(~serdes.rx_idle,
                NextState("WAIT_STABLE")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("WAIT_STABLE",
            self.debug.eq(3),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK_PATTERN")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("CHECK_PATTERN",
            self.debug.eq(4),
            If(~delay_min_found,
                If(serdes.rx_comma,
                    timer.wait.eq(1),
                    If(timer.done,
                        timer.wait.eq(0),
                        NextValue(delay_min, delay),
                        NextValue(delay_min_found, 1)
                    )
                ).Else(
                    NextState("INC_DELAY_BITSLIP")
                ),
            ).Else(
                If(~serdes.rx_comma,
                    NextValue(delay_max, delay),
                    NextValue(delay_max_found, 1),
                    NextState("CHECK_SAMPLING_WINDOW")
                ).Else(
                    NextState("INC_DELAY_BITSLIP")
                )
            ),
            serdes.tx_comma.eq(1)
        )
        self.comb += serdes.rx_bitslip_value.eq(bitslip)
        fsm.act("INC_DELAY_BITSLIP",
            self.debug.eq(5),
            NextState("WAIT_STABLE"),
            If(delay == (taps - 1),
                If(bitslip == (40 - 1),
                    NextState("ERROR")
                ).Else(
                    NextValue(delay_min_found, 0),
                    NextValue(bitslip, bitslip + 1)
                ),
                NextValue(delay, 0),
                serdes.rx_delay_rst.eq(1)
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx_delay_ce.eq(1)
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("CHECK_SAMPLING_WINDOW",
            self.debug.eq(6),
            If((delay_max - delay_min) < taps//16,
               NextValue(delay_min_found, 0),
               NextValue(delay_max_found, 0),
               NextState("WAIT_STABLE")
            ).Else(
                NextState("CONFIGURE_SAMPLING_WINDOW")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("CONFIGURE_SAMPLING_WINDOW",
            self.debug.eq(7),
            If(delay == (delay_min + (delay_max - delay_min)[1:]),
                NextState("READY")
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx_delay_inc.eq(1),
                serdes.rx_delay_ce.eq(1),
                NextState("WAIT_SAMPLING_WINDOW")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("WAIT_SAMPLING_WINDOW",
            self.debug.eq(8),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CONFIGURE_SAMPLING_WINDOW")
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("READY",
            self.debug.eq(9),
            self.ready.eq(1)
        )
        fsm.act("ERROR",
            self.debug.eq(10),
            self.error.eq(1)
        )


class _SerdesSlaveInit(Module, AutoCSR):
    def __init__(self, serdes, taps, timeout=2**14):
        self.ready = Signal()
        self.error = Signal()
        self.link_reset = Signal()
        self.debug = Signal(8)

        # # #

        self.delay = delay = Signal(max=taps)
        self.delay_min = delay_min = Signal(max=taps)
        self.delay_min_found = delay_min_found = Signal()
        self.delay_max = delay_max = Signal(max=taps)
        self.delay_max_found = delay_max_found = Signal()
        self.bitslip = bitslip = Signal(max=40)

        timer = WaitTimer(timeout)
        self.submodules += timer

        self.comb += self.link_reset.eq(serdes.rx_idle)

        self.comb += serdes.rx_delay_inc.eq(1)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.debug.eq(0),
            NextValue(delay, 0),
            NextValue(delay_min, 0),
            NextValue(delay_min_found, 0),
            NextValue(delay_max, 0),
            NextValue(delay_max_found, 0),
            serdes.rx_delay_rst.eq(1),
            NextValue(bitslip, 0),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("WAIT_STABLE"),
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("WAIT_STABLE",
            self.debug.eq(1),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CHECK_PATTERN")
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("CHECK_PATTERN",
            self.debug.eq(2),
            If(~delay_min_found,
                If(serdes.rx_comma,
                    timer.wait.eq(1),
                    If(timer.done,
                        timer.wait.eq(0),
                        NextValue(delay_min, delay),
                        NextValue(delay_min_found, 1)
                    )
                ).Else(
                    NextState("INC_DELAY_BITSLIP")
                ),
            ).Else(
                If(~serdes.rx_comma,
                    NextValue(delay_max, delay),
                    NextValue(delay_max_found, 1),
                    NextState("CHECK_SAMPLING_WINDOW")
                ).Else(
                    NextState("INC_DELAY_BITSLIP")
                )
            ),
            serdes.tx_idle.eq(1)
        )
        self.comb += serdes.rx_bitslip_value.eq(bitslip)
        fsm.act("INC_DELAY_BITSLIP",
            self.debug.eq(3),
            NextState("WAIT_STABLE"),
            If(delay == (taps - 1),
                If(bitslip == (40 - 1),
                    NextState("ERROR")
                ).Else(
                    NextValue(delay_min_found, 0),
                    NextValue(bitslip, bitslip + 1)
                ),
                NextValue(delay, 0),
                serdes.rx_delay_rst.eq(1)
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx_delay_ce.eq(1)
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("CHECK_SAMPLING_WINDOW",
            self.debug.eq(4),
            If((delay_max - delay_min) < taps//16,
               NextValue(delay_min_found, 0),
               NextValue(delay_max_found, 0),
               NextState("WAIT_STABLE")
            ).Else(
                NextState("CONFIGURE_SAMPLING_WINDOW")
            ),
            serdes.tx_idle.eq(1)
        )
        fsm.act("CONFIGURE_SAMPLING_WINDOW",
            self.debug.eq(5),
            If(delay == (delay_min + (delay_max - delay_min)[1:]),
                NextState("SEND_PATTERN")
            ).Else(
                NextValue(delay, delay + 1),
                serdes.rx_delay_inc.eq(1),
                serdes.rx_delay_ce.eq(1),
                NextState("WAIT_SAMPLING_WINDOW")
            )
        )
        fsm.act("WAIT_SAMPLING_WINDOW",
            self.debug.eq(6),
            timer.wait.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("CONFIGURE_SAMPLING_WINDOW")
            )
        )
        fsm.act("SEND_PATTERN",
            self.debug.eq(7),
            timer.wait.eq(1),
            If(timer.done,
                If(~serdes.rx_comma,
                    NextState("READY")
                )
            ),
            serdes.tx_comma.eq(1)
        )
        fsm.act("READY",
            self.debug.eq(8),
            self.ready.eq(1)
        )
        fsm.act("ERROR",
            self.debug.eq(9),
            self.error.eq(1)
        )


class _SerdesControl(Module, AutoCSR):
    def __init__(self, init, mode="master"):
        if mode == "master":
            self.reset = CSR()
        self.ready = CSRStatus()
        self.error = CSRStatus()

        self.delay = CSRStatus(9)
        self.delay_min_found = CSRStatus()
        self.delay_min = CSRStatus(9)
        self.delay_max_found = CSRStatus()
        self.delay_max = CSRStatus(9)
        self.bitslip = CSRStatus(6)

        # # #

        if mode == "master":
            self.comb += init.reset_link.eq(self.reset.re)
        self.comb += [
            self.ready.status.eq(init.ready),
            self.error.status.eq(init.error),
            self.delay.status.eq(init.delay),
            self.delay_min_found.status.eq(init.delay_min_found),
            self.delay_min.status.eq(init.delay_min),
            self.delay_max_found.status.eq(init.delay_max_found),
            self.delay_max.status.eq(init.delay_max),
            self.bitslip.status.eq(init.bitslip)
        ]


class SERWBPHY(Module, AutoCSR):
    def __init__(self, device, pads, mode="master"):
        assert mode in ["master", "slave"]
        if device[:4] == "xcku":
            taps = 512
            self.submodules.serdes = KUSSerdes(pads, mode)
        elif device[:4] == "xc7a":
            taps = 32
            self.submodules.serdes = S7Serdes(pads, mode)
        else:
            raise NotImplementedError
        if mode == "master":
            self.submodules.init = _SerdesMasterInit(self.serdes, taps)
        else:
            self.submodules.init = ResetInserter()(_SerdesSlaveInit(self.serdes, taps))
            self.comb += self.init.reset.eq(self.init.link_reset)
        self.submodules.control = _SerdesControl(self.init, mode)
