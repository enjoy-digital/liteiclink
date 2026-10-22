#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Xilinx transceivers init FSMs tests, against a behavioral model of the transceiver reset/alignment
signals (PLL lock, reset done, buffer bypass delay/phase alignment).
"""

import unittest

from migen import *

from litex.gen import *
from litex.gen.sim import *

from liteiclink.serdes.gtp_7series_init    import GTPTXInit, GTPRXInit
from liteiclink.serdes.gtx_7series_init    import GTXTXInit, GTXRXInit
from liteiclink.serdes.gth_7series_init    import GTHTXInit as GTH2TXInit, GTHRXInit as GTH2RXInit
from liteiclink.serdes.gth_ultrascale_init import GTHTXInit as GTHUSTXInit, GTHRXInit as GTHUSRXInit
from liteiclink.serdes.gty_ultrascale_init import GTYTXInit, GTYRXInit

SYS_CLK_FREQ = 10e6 # Small timers (watchdogs: 1ms = 10k cycles).

# Inits: (class, kind, watchdog cycles).
INITS = {
    "gtx_tx"    : (GTXTXInit,   "Xx", 10000),
    "gtx_rx"    : (GTXRXInit,   "Xx", 10000),
    "gth2_tx"   : (GTH2TXInit,  "Xx", 10000),
    "gth2_rx"   : (GTH2RXInit,  "Xx", 10000),
    "gthus_tx"  : (GTHUSTXInit, "Xx", 100000),
    "gthus_rx"  : (GTHUSRXInit, "Xx", 100000),
    "gty_tx"    : (GTYTXInit,   "Xx", 100000),
    "gty_rx"    : (GTYRXInit,   "Xx", 100000),
    "gtp_tx"    : (GTPTXInit,   "tx", 10000),
    "gtp_rx"    : (GTPRXInit,   "rx", 40000),
}

# Transceiver Model --------------------------------------------------------------------------------

class TransceiverModel:
    """Behavioral model of the transceiver signals used by the init FSMs.

    - PLL locks `pll_lock_delay` cycles after PLL reset release (shared PLL: locked when no PLL
      reset is driven).
    - Reset done `reset_done_delay` cycles after GT reset release (never with `reset_done=False`).
    - Buffer bypass alignment: delay reset done, phase init done, phase align done (2 rising edges)
      and sync done, a few cycles after the requests.
    """
    def __init__(self, init, kind, reset_done=True, pll_lock_delay=20, reset_done_delay=20):
        self.init             = init
        self.kind             = kind
        self.reset_done       = reset_done
        self.pll_lock_delay   = pll_lock_delay
        self.reset_done_delay = reset_done_delay
        self.events           = []
        self.drp_writes       = []

    def sig(self, name):
        # Generic inits use Xx-prefixed names, GTP inits tx/rx-prefixed names.
        if self.kind != "Xx":
            name = name.replace("Xx", self.kind)
        return getattr(self.init, name, None)

    @passive
    def generator(self):
        init = self.init
        pllreset   = getattr(init, "pllreset", None)
        gtreset    = self.sig("gtXxreset")
        dlysreset  = self.sig("Xxdlysreset")
        pll_count   = 0
        reset_count = 0
        align       = None # Cycles since alignment request.
        dlysreset_d = 0
        drp_pending = 0
        drp_reg     = 0xffff
        pmaresetdone_count = 0
        while True:
            # PLL.
            if pllreset is not None and (yield pllreset):
                pll_count = 0
            else:
                pll_count += 1
            yield init.plllock.eq(pll_count >= self.pll_lock_delay)

            # GT reset / reset done.
            if (yield gtreset):
                reset_count = 0
                align       = None
                pmaresetdone_count = 0
            else:
                reset_count += 1
            done = self.reset_done and (reset_count >= self.reset_done_delay)
            yield self.sig("Xxresetdone").eq(done)

            # GTP RX: PMA reset done pulses after GT reset release (falling edge awaited).
            if self.kind == "rx":
                pmaresetdone_count += (reset_count > 0)
                yield init.rxpmaresetdone.eq(1 <= pmaresetdone_count <= 8)

            # Buffer bypass alignment (started on delay reset request, pulse or level).
            _dlysreset = (yield dlysreset) if dlysreset is not None else 0
            if _dlysreset and not dlysreset_d:
                align = 0
                self.events.append("align")
            elif align is not None:
                align += 1
            dlysreset_d = _dlysreset
            if align is not None:
                yield self.sig("Xxdlysresetdone").eq(align >= 4)
                if self.sig("Xxphinitdone") is not None:
                    yield self.sig("Xxphinitdone").eq(align >= 8)
                if self.sig("Xxphaligndone") is not None:
                    # Two rising edges.
                    yield self.sig("Xxphaligndone").eq((12 <= align < 16) | (align >= 20))
                if self.sig("Xxsyncdone") is not None:
                    yield self.sig("Xxsyncdone").eq(align >= 12)
            else:
                for name in ["Xxdlysresetdone", "Xxphinitdone", "Xxphaligndone", "Xxsyncdone"]:
                    if self.sig(name) is not None:
                        yield self.sig(name).eq(0)

            # DRP (GTP RX): register model, rdy one cycle after en.
            if hasattr(init, "drp") and self.kind == "rx":
                yield init.drp.rdy.eq(drp_pending == 1)
                if drp_pending == 1:
                    drp_pending = 0
                if (yield init.drp.en):
                    drp_pending = 1
                    if (yield init.drp.we):
                        drp_reg = (yield init.drp.di)
                        self.drp_writes.append(drp_reg)
                    yield init.drp.do.eq(drp_reg)
            yield

# Helpers ------------------------------------------------------------------------------------------

def wait_done(init, timeout):
    for i in range(timeout):
        if (yield init.done):
            return i
        yield
    return None

# Test Inits ---------------------------------------------------------------------------------------

class TestSerDesInit(unittest.TestCase):
    def run_init(self, name, buffer_enable=False, reset_done=True, cycles=2000, restart=False):
        cls, kind, watchdog = INITS[name]
        init    = cls(SYS_CLK_FREQ, buffer_enable=buffer_enable)
        model   = TransceiverModel(init, kind, reset_done=reset_done)
        results = {}
        def generator():
            results["done"] = yield from wait_done(init, cycles)
            if restart and results["done"] is not None:
                yield init.restart.eq(1)
                yield
                yield init.restart.eq(0)
                yield
                results["done_after_restart"] = (yield init.done)
                results["redone"] = yield from wait_done(init, cycles)
        run_simulation(init, [generator(), model.generator()])
        return results, model

    def test_init_done(self):
        for name in INITS:
            for buffer_enable in [False, True]:
                with self.subTest(init=name, buffer_enable=buffer_enable):
                    results, model = self.run_init(name, buffer_enable=buffer_enable)
                    self.assertIsNotNone(results["done"])
                    # Buffer bypass alignment only when buffer is disabled.
                    self.assertEqual("align" in model.events, not buffer_enable)

    def test_restart(self):
        for name in INITS:
            with self.subTest(init=name):
                results, model = self.run_init(name, restart=True)
                self.assertIsNotNone(results["done"])
                self.assertEqual(results["done_after_restart"], 0)
                self.assertIsNotNone(results["redone"])

    def test_no_reset_done(self):
        # Transceiver never reports reset done: init must not complete, watchdog restarts the FSM
        # (PLL/GT reset asserted again).
        for name in ["gtx_tx", "gthus_rx", "gtp_tx"]:
            with self.subTest(init=name):
                cls, kind, watchdog = INITS[name]
                init    = cls(SYS_CLK_FREQ, buffer_enable=True)
                model   = TransceiverModel(init, kind, reset_done=False)
                gtreset = model.sig("gtXxreset")
                resets  = []
                def generator():
                    reset_d = 1
                    for _ in range(100): # Skip initial reset.
                        yield
                    for i in range(watchdog + 200):
                        self.assertEqual((yield init.done), 0)
                        reset = (yield gtreset)
                        if reset and not reset_d:
                            resets.append(i)
                        reset_d = reset
                        yield
                run_simulation(init, [generator(), model.generator()])
                self.assertEqual(len(resets), 1)
                self.assertGreater(resets[0], watchdog - 300)

    def test_gtp_rx_drp_sequence(self):
        # GTP RX init: DRP register 0x011 bit 11 cleared during PMA reset, then restored.
        results, model = self.run_init("gtp_rx")
        self.assertIsNotNone(results["done"])
        self.assertEqual(model.drp_writes, [0xffff & 0xf7ff, 0xffff])

if __name__ == "__main__":
    unittest.main()
