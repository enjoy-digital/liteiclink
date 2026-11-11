#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from test import test_serwb_init as init

# Test SERWB Init Phase Error ----------------------------------------------------------------------

class TestSERWBInitPhaseError(unittest.TestCase):
    def test_phase_after_error(self):
        # After a failed initialization, phase_sel must show the last phase tried (no wrap).
        for dut_cls in [init.DUTMaster, init.DUTSlave]:
            with self.subTest(dut=dut_cls.__name__):
                r = init.TestSERWBInitPhase.phase_test(self, dut_cls, "1:2", valid_phase=3)
                self.assertEqual((r["ready"], r["phase"]), (0, 1))

if __name__ == "__main__":
    unittest.main()
