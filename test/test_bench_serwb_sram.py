#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import io
import unittest
import contextlib
from types import SimpleNamespace

from test.test_bench_serwb import load_test_serwb

# Test SerWB Bench SRAM ----------------------------------------------------------------------------

class TestBenchSerWBSRAM(unittest.TestCase):
    def test_sram_dump(self):
        # --sram writes 128 words: both dumps must read the 128 words back.
        test_serwb = load_test_serwb()
        base  = 0x30000000
        reads = []
        class RemoteClient:
            def __init__(self, **kwargs):
                self.mems = SimpleNamespace(serwb=SimpleNamespace(base=base))
            def open(self):  pass
            def close(self): pass
            def write(self, addr, data): pass
            def read(self, addr):
                reads.append(addr)
                return 0
        test_serwb.RemoteClient = RemoteClient
        with contextlib.redirect_stdout(io.StringIO()):
            test_serwb.sram_test(port=1234)
        self.assertEqual(reads, 2*[base + 4*i for i in range(128)])

if __name__ == "__main__":
    unittest.main()
