#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *
from litex.gen.sim import *

from liteiclink.serdes.clock_aligner import BruteforceClockAligner

COMMA   = 0b0101111100 # K28.5 (LSB first, as received from Xilinx transceivers).
COMMA_N = ~COMMA & 0b1111111111
VALID   = 0b0101010101 # Valid 8b10b symbol (5 ones), not a comma.
INVALID = 0b0000000011 # Invalid 8b10b symbol (2 ones).

TX_CLK_FREQ  = 1e6
CHECK_PERIOD = 100e-6 # 100 cycles.

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self):
        self.cd_tx = ClockDomain()
        self.cd_rx = ClockDomain()
        self.aligner = BruteforceClockAligner(COMMA, TX_CLK_FREQ, check_period=CHECK_PERIOD)

# Test Clock Aligner -------------------------------------------------------------------------------

class TestClockAligner(unittest.TestCase):
    def run_aligner(self, symbols, cycles=1000):
        """Drive rxdata with symbols(i) (lower 10-bit) and return (ready, restarts) per cycle."""
        dut     = DUT()
        results = {"ready": [], "restarts": []}
        def rx_generator():
            for i in range(cycles):
                yield dut.aligner.rxdata.eq(symbols(i))
                yield
        def tx_generator():
            for i in range(cycles):
                results["ready"].append((yield dut.aligner.ready))
                # Ignore first check (check counter starts at 0: immediate check after reset).
                if (yield dut.aligner.restart) and i > 2:
                    results["restarts"].append(i)
                yield
        run_simulation(dut,
            generators = {"rx": rx_generator(), "tx": tx_generator()},
            clocks     = {"rx": 10, "tx": 10},
        )
        return results

    def test_aligned(self):
        # Commas (both disparities) and valid symbols: ready after the comma and no-error checks.
        results = self.run_aligner(lambda i: [COMMA, VALID, COMMA_N, VALID][i%4])
        self.assertEqual(results["restarts"], [])
        self.assertEqual(results["ready"][-1], 1)
        self.assertGreater(results["ready"].index(1), 150) # At least 2 check periods.

    def test_no_comma(self):
        # No comma (wrong phase): transceiver restarted at each check, never ready.
        results = self.run_aligner(lambda i: VALID)
        self.assertEqual(sum(results["ready"]), 0)
        self.assertGreaterEqual(len(results["restarts"]), 5)

    def test_errors_after_ready(self):
        # Errors once aligned: ready dropped and transceiver restarted.
        results = self.run_aligner(lambda i: INVALID if 600 <= i < 610 else [COMMA, VALID][i%2])
        self.assertEqual(results["ready"][590], 1)
        self.assertEqual(results["ready"][-1], 1)  # Re-aligned.
        self.assertIn(0, results["ready"][600:700])
        self.assertTrue(any(600 <= i < 700 for i in results["restarts"]))

    def test_disable(self):
        # Disabled: no checks, no restart.
        dut      = DUT()
        restarts = []
        def generator():
            yield dut.aligner.disable.eq(1)
            for i in range(500):
                if (yield dut.aligner.restart) and i > 2:
                    restarts.append(i)
                yield
        run_simulation(dut, generators={"tx": generator()}, clocks={"rx": 10, "tx": 10})
        self.assertEqual(restarts, [])

if __name__ == "__main__":
    unittest.main()
