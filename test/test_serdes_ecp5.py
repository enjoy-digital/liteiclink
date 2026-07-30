#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.sim import run_simulation

from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.serdes.serdes_ecp5 import SerDesECP5, SerDesECP5WordAligner


class TestSerDesECP5(unittest.TestCase):
    @staticmethod
    def serdes(with_oob=False):
        class PLL:
            refclk = Signal()
            config = {
                "m":        20,
                "d":        1,
                "linerate": 3e9,
            }

        class Pads:
            p = Signal()
            n = Signal()

        return SerDesECP5(
            pll      = PLL(),
            tx_pads  = Pads(),
            rx_pads  = Pads(),
            with_oob = with_oob,
        )

    def test_oob_is_opt_in(self):
        legacy = self.serdes()
        oob    = self.serdes(with_oob=True)

        # Existing designs retain the DCU aligner, SCI idle control, and unbuffered datapath.
        self.assertFalse(hasattr(legacy, "rx_aligner"))
        self.assertNotIn("p_D_SYNC_LOCAL_EN", legacy.serdes_params)
        self.assertNotIn("p_CHX_LSM_DISABLE", legacy.serdes_params)
        self.assertNotIn("i_CHX_FFC_EI_EN", legacy.serdes_params)

        # OOB users explicitly select the fabric aligner and direct electrical-idle path.
        self.assertTrue(hasattr(oob, "rx_aligner"))
        self.assertEqual(oob.serdes_params["p_D_SYNC_LOCAL_EN"], "0b1")
        self.assertEqual(oob.serdes_params["p_CHX_LSM_DISABLE"], "0b1")
        self.assertIn("i_CHX_FFC_EI_EN", oob.serdes_params)

    def test_legacy_init_interface_is_preserved(self):
        init = self.serdes().init
        for name in ["rst", "tx_rst", "rx_rst", "pcs_rst", "ready"]:
            self.assertTrue(hasattr(init, name))

    def test_word_aligner(self):
        encoded = []
        encoder = Encoder(1, True)
        symbols = [(0xbc, 1), (0x4a, 0), (0x4a, 0), (0x7b, 0)]

        def encode():
            for n in range(84):
                data, control = symbols[n % len(symbols)]
                yield encoder.d[0].eq(data)
                yield encoder.k[0].eq(control)
                yield
                encoded.append((yield encoder.output[0]))

        run_simulation(encoder, encode())
        bits = []
        for symbol in encoded[4:]:
            bits += [(symbol >> n) & 1 for n in range(10)]

        class DUT(Module):
            def __init__(self):
                self.submodules.aligner = SerDesECP5WordAligner()
                self.decoders = [Decoder(True) for _ in range(2)]
                self.submodules += self.decoders
                self.sync += [
                    self.decoders[0].input.eq(self.aligner.source[:10]),
                    self.decoders[1].input.eq(self.aligner.source[10:]),
                ]

        for offset in range(20):
            dut = DUT()
            decoded = []

            def check():
                stream = bits[offset:] + bits[:offset]
                words = [
                    stream[n:n + 20]
                    for n in range(0, len(stream) - 20, 20)
                ]
                for word in words*3:
                    yield dut.aligner.sink.eq(sum(bit << n for n, bit in enumerate(word)))
                    yield
                    decoded.append([
                        ((yield dut.decoders[0].d), (yield dut.decoders[0].k)),
                        ((yield dut.decoders[1].d), (yield dut.decoders[1].k)),
                    ])

            run_simulation(dut, check())
            valid = [(0xbc, 1), (0x4a, 0), (0x7b, 0)]
            clean = sum(all(symbol in valid for symbol in word) for word in decoded[20:])
            self.assertGreaterEqual(clean, 0.9*len(decoded[20:]), f"offset {offset}")


if __name__ == "__main__":
    unittest.main()
