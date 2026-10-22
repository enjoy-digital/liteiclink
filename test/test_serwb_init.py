#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen.sim import *

from liteiclink.serwb.phy    import _SerdesInit
from liteiclink.serwb.genphy import _SerdesInit as _GenSerdesInit
from liteiclink.serwb.common import SerdesControl

# Serdes Model -------------------------------------------------------------------------------------

class SerdesModel(Module):
    def __init__(self, taps, mode="slave", with_phase=False):
        self.tx           = Module()
        self.rx           = Module()

        self.tx.idle      = Signal()
        self.tx.comma     = Signal()
        self.rx.idle      = Signal()
        self.rx.comma     = Signal()

        self.rx.shift_inc = Signal()
        self.rx.delay_rst = Signal()
        self.rx.delay_inc = Signal()

        self.valid_shift  = Signal(6)
        self.valid_delays = Signal(taps)
        self.valid_phase  = Signal(2)
        if with_phase:
            self.rx.phase_sel = Signal(2)

        # # #

        delay   = Signal(max=taps)
        shift = Signal(6)

        valid_delays = Array(Signal() for i in range(taps))
        for i in range(taps):
            self.comb += valid_delays[taps-1-i].eq(self.valid_delays[i])

        # Word alignment is cyclic (40 bit positions).
        self.sync += [
            If(self.rx.shift_inc,
                shift.eq(Mux(shift == 39, 0, shift + 1))
            ),
            If(self.rx.delay_rst,
                delay.eq(0)
            ).Elif(self.rx.delay_inc,
                delay.eq(delay + 1)
            )
        ]

        if mode == "master":
            self.submodules.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
            self.comb += self.fsm.reset.eq(self.tx.idle)
            fsm.act("IDLE",
                If(self.tx.comma,
                    NextState("SEND_COMMA")
                ),
                self.rx.idle.eq(1)
            )
            fsm.act("SEND_COMMA",
                If(valid_delays[delay] &
                   (shift == self.valid_shift) &
                   (getattr(self.rx, "phase_sel", 0) == self.valid_phase),
                        self.rx.comma.eq(1)
                ),
                If(~self.tx.comma,
                    NextState("READY")
                )
            )
            fsm.act("READY")
        elif mode == "slave":
            self.submodules.fsm = fsm = FSM(reset_state="IDLE")
            fsm.act("IDLE",
                self.rx.idle.eq(1),
                NextState("SEND_COMMA")
            )
            fsm.act("SEND_COMMA",
                If(valid_delays[delay] &
                   (shift == self.valid_shift) &
                   (getattr(self.rx, "phase_sel", 0) == self.valid_phase),
                        self.rx.comma.eq(1)
                ),
                If(~self.tx.idle,
                    NextState("READY")
                )
            )
            fsm.act("READY")

# DUT Master ---------------------------------------------------------------------------------------

class DUTMaster(Module):
    def __init__(self, taps=32, clk_ratio="1:1"):
        self.submodules.serdes = SerdesModel(taps, mode="master", with_phase=(clk_ratio != "1:1"))
        self.submodules.init   = _SerdesInit(self.serdes, taps, timeout=1, clk_ratio=clk_ratio, mode="master")

# DUT Slave ----------------------------------------------------------------------------------------

class DUTSlave(Module):
    def __init__(self, taps=32, clk_ratio="1:1"):
        self.submodules.serdes = SerdesModel(taps, mode="slave", with_phase=(clk_ratio != "1:1"))
        self.submodules.init   = _SerdesInit(self.serdes, taps, timeout=1, clk_ratio=clk_ratio, mode="slave")

# TestSERWBInit ------------------------------------------------------------------------------------

def generator(test, dut, valid_shift, valid_delays, check_success):
    yield dut.serdes.valid_shift.eq(valid_shift)
    yield dut.serdes.valid_delays.eq(valid_delays)
    while not ((yield dut.init.ready) or
               (yield dut.init.error)):
        yield
    if check_success:
        ready     = (yield dut.init.ready)
        error     = (yield dut.init.error)
        delay_min = (yield dut.init.delay_min)
        delay_max = (yield dut.init.delay_max)
        delay     = (yield dut.init.delay)
        shift     = (yield dut.init.shift)
        test.assertEqual(ready, 1)
        test.assertEqual(error, 0)
        test.assertEqual(delay_min, 4)
        test.assertEqual(delay_max, 9)
        test.assertEqual(delay, 6)
        test.assertEqual(shift, valid_shift)
    else:
        ready = (yield dut.init.ready)
        error = (yield dut.init.error)
        test.assertEqual(ready, 0)
        test.assertEqual(error, 1)

class TestSERWBInit(unittest.TestCase):
    def test_master_init_success(self):
        dut = DUTMaster()
        valid_shift  = 2
        valid_delays = 0b10001111100000111110000011111000
        run_simulation(dut, generator(self, dut, valid_shift, valid_delays, True))

    def test_master_init_failure(self):
        # too small window
        dut = DUTMaster()
        valid_shift  = 2
        valid_delays = 0b00000000000000010000000000000000
        run_simulation(dut, generator(self, dut, valid_shift, valid_delays, False))

    def test_slave_init_success(self):
        dut = DUTSlave()
        valid_shift  = 2
        valid_delays = 0b10001111100000111110000011111000
        run_simulation(dut, generator(self, dut, valid_shift, valid_delays, True))

    def test_slave_init_failure(self):
        # too small window
        dut = DUTSlave()
        valid_shift  = 2
        valid_delays = 0b00000000000000010000000000000000
        run_simulation(dut, generator(self, dut, valid_shift, valid_delays, False))

# Test SERWB Init Phase (Efinix clk_ratio) ---------------------------------------------------------

class TestSERWBInitPhase(unittest.TestCase):
    def phase_test(self, dut_cls, clk_ratio, valid_phase):
        dut     = dut_cls(clk_ratio=clk_ratio)
        results = {}
        def generator():
            yield dut.serdes.valid_shift.eq(2)
            yield dut.serdes.valid_delays.eq(0b10001111100000111110000011111000)
            yield dut.serdes.valid_phase.eq(valid_phase)
            for _ in range(200000):
                if (yield dut.init.ready) or (yield dut.init.error):
                    break
                yield
            results["ready"] = (yield dut.init.ready)
            results["phase"] = (yield dut.init.phase_sel)
        run_simulation(dut, generator())
        return results

    def test_master_phase_found(self):
        r = self.phase_test(DUTMaster, "1:4", valid_phase=2)
        self.assertEqual((r["ready"], r["phase"]), (1, 2))

    def test_slave_phase_found(self):
        r = self.phase_test(DUTSlave, "1:2", valid_phase=1)
        self.assertEqual((r["ready"], r["phase"]), (1, 1))

    def test_master_phase_not_found(self):
        r = self.phase_test(DUTMaster, "1:2", valid_phase=3)
        self.assertEqual(r["ready"], 0)

# Generic PHY Init ---------------------------------------------------------------------------------

class GenSerdesModel(Module):
    """Generic PHY SerDes model: comma received when word alignment (shift) is valid."""
    def __init__(self, mode="slave"):
        self.tx, self.rx = Module(), Module()
        self.tx.idle    = Signal()
        self.tx.comma   = Signal()
        self.rx.idle    = Signal()
        self.rx.comma   = Signal()
        self.rx.shift   = Signal()
        self.valid_shift = Signal(6)

        # # #

        shift = Signal(6)
        self.sync += If(self.rx.shift, shift.eq(Mux(shift == 39, 0, shift + 1)))
        aligned = Signal()
        self.comb += aligned.eq(shift == self.valid_shift)
        # Remote side: Master sends commas, Slave sends idle then commas (see init sequence).
        if mode == "master":
            # Remote Slave: idle until master sends commas, then commas when aligned.
            self.comb += self.rx.idle.eq(~self.tx.comma)
            self.comb += self.rx.comma.eq(self.tx.comma & aligned)
        else:
            # Remote Master: commas until local slave sends commas.
            self.comb += self.rx.comma.eq(~self.tx.comma & aligned)

class TestSERWBGenInit(unittest.TestCase):
    def gen_init_test(self, mode, valid_shift):
        serdes  = GenSerdesModel(mode)
        init    = _GenSerdesInit(serdes, timeout=4, mode=mode)
        dut     = Module()
        dut.submodules += serdes, init
        results = {}
        def generator():
            yield serdes.valid_shift.eq(valid_shift)
            for _ in range(10000):
                if (yield init.ready) or (yield init.error):
                    break
                yield
            results.update(ready=(yield init.ready), error=(yield init.error), shift=(yield init.shift))
        run_simulation(dut, generator())
        return results

    def test_master_init(self):
        self.assertEqual(self.gen_init_test("master", 7), dict(ready=1, error=0, shift=7))

    def test_slave_init(self):
        self.assertEqual(self.gen_init_test("slave", 13), dict(ready=1, error=0, shift=13))

    def test_init_failure(self):
        # No valid alignment.
        r = self.gen_init_test("slave", 63)
        self.assertEqual((r["ready"], r["error"]), (0, 1))

# SerDes Control -----------------------------------------------------------------------------------

class TestSERWBControl(unittest.TestCase):
    def test_control(self):
        serdes = Module()
        serdes.rx, serdes.rx.datapath = Module(), Module()
        serdes.reset                   = Signal()
        serdes.rx.idle                 = Signal()
        serdes.rx.datapath.invalid     = Signal()
        init = Module()
        for name in ["ready", "error", "reset", "delay_min_found", "delay_max_found"]:
            setattr(init, name, Signal())
        for name in ["delay", "delay_min", "delay_max", "shift", "phase_sel"]:
            setattr(init, name, Signal(9))
        control = SerdesControl(serdes, init, mode="master")
        dut     = Module()
        dut.submodules += control
        results = {}
        def generator():
            # Reset CSR -> init reset.
            yield control.reset.wr_stb.eq(1)
            yield
            yield control.reset.wr_stb.eq(0)
            yield
            results["reset"] = (yield init.reset)
            # Status.
            yield init.ready.eq(1)
            yield init.shift.eq(5)
            yield
            yield
            results["ready"] = (yield control.ready.status)
            results["shift"] = (yield control.shift.status)
            # PRBS: count errors during prbs_cycles.
            yield control.prbs_cycles.storage.eq(10)
            yield control.prbs_start.wr_stb.eq(1)
            yield
            yield control.prbs_start.wr_stb.eq(0)
            for i in range(20):
                yield control.prbs_error.eq(i in [2, 3, 7, 15])
                yield
            results["prbs_errors"] = (yield control.prbs_errors.status)
            # Invalid 8b10b words: counted when link ready, cleared on link down.
            for i in range(3):
                yield serdes.rx.datapath.invalid.eq(1)
                yield
            yield serdes.rx.datapath.invalid.eq(0)
            yield
            results["invalid_errors"] = (yield control.invalid_errors.status)
            yield init.ready.eq(0)
            yield
            yield
            results["invalid_errors_cleared"] = (yield control.invalid_errors.status)
        run_simulation(dut, generator())
        self.assertEqual(results, dict(reset=1, ready=1, shift=5, prbs_errors=3, invalid_errors=3, invalid_errors_cleared=0))
