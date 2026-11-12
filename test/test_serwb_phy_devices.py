#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
from unittest import mock

from migen import *

from liteiclink.serwb          import phy as serwb_phy
from liteiclink.serwb.phy      import SERWBPHY
from liteiclink.serwb.kuserdes import KUSerdes

# Test SerWB PHY Devices ---------------------------------------------------------------------------

class _Selected(Exception):
    pass

class TestSERWBPHYDevices(unittest.TestCase):
    def pads(self):
        return Record([(name, 1) for name in ["clk_p", "clk_n", "tx_p", "tx_n", "rx_p", "rx_n"]])

    def test_efinix_devices(self):
        # Every Trion (T4 to T120) and Titanium device must select the Efinix SerDes (stubbed: the
        # real one needs an Efinix platform).
        def efinix_serdes(*args, **kwargs):
            raise _Selected()
        devices = ["T4F81C2", "T8F81C2", "T13F256C3", "T20F256C3", "T35F324C3", "T55F324C3",
            "T85F324C3", "T120F576I4", "Ti60F225C3", "Ti180J484C4"]
        with mock.patch.object(serwb_phy, "EfinixSerdes", efinix_serdes):
            for device in devices:
                with self.subTest(device=device):
                    with self.assertRaises(_Selected):
                        SERWBPHY(device, self.pads(), mode="master")

    def test_artix_ultrascale_plus(self):
        # Artix UltraScale+ selects the UltraScale SerDes, always with the UltraScale+ settings.
        phy = SERWBPHY("xcau15p-ffvb676-2-e", self.pads(), mode="master")
        self.assertIsInstance(phy.serdes, KUSerdes)
        sim_devices = set()
        for module in [phy.serdes.tx, phy.serdes.rx]:
            for special in module.get_fragment().specials:
                if hasattr(special, "items"):
                    for item in special.items:
                        if getattr(item, "name", None) == "SIM_DEVICE":
                            sim_devices.add(item.value.value if hasattr(item.value, "value") else item.value)
        self.assertEqual(sim_devices, {"ULTRASCALE_PLUS"})

    def test_unknown_device(self):
        with self.assertRaises(NotImplementedError):
            SERWBPHY("LFE5UM5G-45F", self.pads(), mode="master")

if __name__ == "__main__":
    unittest.main()
