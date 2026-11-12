#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
GTP user DRP port: the GTP wrapper is simulated with a GTPE2_CHANNEL DRP/RX PMA reset model (other
primitives empty, QPLL lock driven by the test). DRPControl accesses on serdes.drp must reach the
GTP DRP with the default DRP mux selection, while the RX initialization keeps the DRP during its
RX PMA reset workaround (read/mask/restore of DRP 0x011), without DRP protocol violations.
"""

import unittest

from migen import *
from migen.fhdl.specials import Instance

from litex.gen import *

from liteiclink.serdes.common import DRPControl

from test.test_serdes_xilinx import SerDesTop
from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP

# GTP Model ----------------------------------------------------------------------------------------

DRP_REGS = {
    0x011 : 0x0c5a, # RX_DATA_WIDTH (bit 11 cleared/restored by the RX initialization).
    0x05e : 0x0000, # User register.
}

class _Ports:
    def __init__(self, instance):
        self.ports = {item.name: item.expr for item in instance.items
            if isinstance(item, (Instance.Input, Instance.Output))}

    def __getattr__(self, name):
        return self.ports[name]

class _GTPModel(Module):
    def __init__(self, instance, qpll_lock):
        self.regs      = {addr: Signal(16, reset=value, name=f"reg_{addr:03x}") for addr, value in DRP_REGS.items()}
        self.accesses  = Signal(16) # DRP accesses (en).
        self.violation = Signal()   # en while an access is pending.
        if instance.of == "GTPE2_COMMON":
            self.comb += _Ports(instance).PLL0LOCK.eq(qpll_lock)
        if instance.of != "GTPE2_CHANNEL":
            return
        p = _Ports(instance)

        # DRP (sys): rdy 3 cycles after en.
        pending = Signal(2)
        addr    = Signal(9)
        self.comb += p.DRPRDY.eq(pending == 1)
        self.sync += [
            If(pending != 0, pending.eq(pending - 1)),
            If(p.DRPEN,
                self.accesses.eq(self.accesses + 1),
                If(pending != 0, self.violation.eq(1)),
                pending.eq(3),
                addr.eq(p.DRPADDR),
                Case(p.DRPADDR, {a: If(p.DRPWE, r.eq(p.DRPDI)) for a, r in self.regs.items()}),
            ),
            Case(addr, {a: p.DRPDO.eq(r) for a, r in self.regs.items()}),
        ]

        # RX PMA reset done: pulse after the RX reset release.
        count = Signal(8)
        self.sync += If(p.GTRXRESET, count.eq(0)).Elif(count != 255, count.eq(count + 1))
        self.comb += p.RXPMARESETDONE.eq((count >= 8) & (count < 32))

class GTPSimTop(LiteXModule):
    def __init__(self):
        self.qpll_lock = Signal()
        self.top       = SerDesTop(GTPQuadPLL, GTP, 125e6, 2.5e9)
        self.serdes    = serdes = self.top.serdes
        self.drp       = DRPControl(serdes.drp)
        self.models    = []

    def overrides(self):
        top = self
        class _InstanceLowerer:
            @staticmethod
            def lower(instance):
                model = _GTPModel(instance, top.qpll_lock)
                if instance.of == "GTPE2_CHANNEL":
                    top.channel = model
                return model
        return {Instance: _InstanceLowerer}

# Helpers ------------------------------------------------------------------------------------------

def drp_access(dut, addr, we=0, di=0, timeout=200):
    yield dut.drp._addr.storage.eq(addr)
    yield dut.drp._we.storage.eq(we)
    yield dut.drp._di.storage.eq(di)
    yield dut.drp._start.re.eq(1)
    yield
    yield dut.drp._start.re.eq(0)
    for i in range(timeout):
        yield
        if (yield dut.drp._done.status):
            return (yield dut.drp._do.status)
    return None

def run(dut, generator):
    run_simulation(dut, generator,
        clocks            = {"sys": 10, "tx": 10, "rx": 10},
        special_overrides = dut.overrides(),
    )

# Test GTP DRP -------------------------------------------------------------------------------------

class TestSerDesGTPDRP(unittest.TestCase):
    def test_user_drp_reaches_gtp(self):
        # No QPLL lock: the RX initialization never uses the DRP, user accesses reach the GTP.
        dut     = GTPSimTop()
        results = {}
        def generator():
            for i in range(20):
                yield
            results["write"] = yield from drp_access(dut, 0x05e, we=1, di=0xbeef)
            results["read"]  = yield from drp_access(dut, 0x05e)
            results["reg"]   = yield dut.channel.regs[0x05e]
        run(dut, generator())
        self.assertIsNotNone(results["write"], "user DRP write never completed")
        self.assertEqual(results["read"], 0xbeef)
        self.assertEqual(results["reg"],  0xbeef)

    def test_user_drp_during_rx_init(self):
        # QPLL lock: the RX initialization does its DRP read/mask/restore of 0x011 while the user
        # does back-to-back accesses: all of them complete, 0x011 is restored, no DRP violation.
        dut     = GTPSimTop()
        results = {"reads": [], "reg_011": []}
        def generator():
            for i in range(8):
                yield
            yield dut.qpll_lock.eq(1)
            for n in range(40):
                yield from drp_access(dut, 0x05e, we=1, di=n)
                results["reads"].append((yield from drp_access(dut, 0x05e)))
            results["violation"] = yield dut.channel.violation
            results["state"]     = yield dut.serdes.rx_init.fsm.state
            results["end"]       = True
        def monitor():
            while "end" not in results:
                value = yield dut.channel.regs[0x011]
                if value not in results["reg_011"][-1:]:
                    results["reg_011"].append(value)
                if (yield dut.serdes.drp_mux.hold):
                    results["held"] = True # User access issued during the RX init DRP sequence.
                yield
        run(dut, [generator(), monitor()])
        self.assertTrue(results.get("held", False), "no user access during the RX init DRP sequence")
        self.assertEqual(results["reads"], list(range(40)))
        self.assertFalse(results["violation"])
        # 0x011 modified (bit 11 cleared) then restored.
        self.assertEqual(results["reg_011"], [DRP_REGS[0x011], DRP_REGS[0x011] & 0xf7ff, DRP_REGS[0x011]])
        rx_init_fsm = dut.serdes.rx_init.fsm
        self.assertNotIn(results["state"], [rx_init_fsm.encoding[s] for s in [
            "POWER-DOWN", "WAIT-INIT-DELAY", "DRP_READ_ISSUE", "DRP_READ_WAIT", "DRP_MOD_ISSUE",
            "DRP_MOD_WAIT", "WAIT_PMARST_FALL", "DRP_RESTORE_ISSUE", "DRP_RESTORE_WAIT"]])

if __name__ == "__main__":
    unittest.main()
