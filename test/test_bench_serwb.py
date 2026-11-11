#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import io
import os
import unittest
import contextlib
import importlib.util
from types import SimpleNamespace

# Helpers ------------------------------------------------------------------------------------------

def load_test_serwb():
    path = os.path.join(os.path.dirname(__file__), "..", "bench", "serwb", "test_serwb.py")
    spec = importlib.util.spec_from_file_location("test_serwb_bench", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class FakeRegister:
    def __init__(self, value=0):
        self.value = value

    def write(self, value):
        pass

    def read(self):
        return self.value

# Test SerWB Bench ---------------------------------------------------------------------------------

class TestBenchSerWB(unittest.TestCase):
    def init_test(self, ready, error):
        test_serwb = load_test_serwb()
        regs = SimpleNamespace()
        for side in ["master", "slave"]:
            for name in ["reset", "ready", "error", "shift"]:
                setattr(regs, f"serwb_{side}_phy_control_{name}", FakeRegister())
        regs.serwb_master_phy_control_ready.value = ready
        regs.serwb_master_phy_control_error.value = error
        class RemoteClient:
            def __init__(self, **kwargs):
                self.regs = regs
            def open(self):  pass
            def close(self): pass
        test_serwb.RemoteClient = RemoteClient
        test_serwb.time         = SimpleNamespace(sleep=lambda t: None)
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            test_serwb.init_test(port=1234)
        return stdout.getvalue()

    def test_init_success(self):
        self.assertIn("Success.", self.init_test(ready=1, error=0))

    def test_init_error(self):
        # Init FSM in error: the wait loop exits before the timeout, must not report a success.
        output = self.init_test(ready=0, error=1)
        self.assertNotIn("Success.", output)
        self.assertIn("Failed", output)

if __name__ == "__main__":
    unittest.main()
