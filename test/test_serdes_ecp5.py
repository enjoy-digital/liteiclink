#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *
from migen.sim import run_simulation

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus
from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5, SerDesECP5WordAligner
from liteiclink.serdes.serdes_ecp5 import SerDesECP5TXGearbox, SerDesECP5RXGearbox
from liteiclink.serdes.serdes_ecp5 import SerDesECP5SymbolAligner


class TestSerDesECP5(unittest.TestCase):
    @staticmethod
    def serdes(with_oob=False, data_width=20):
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
            with_oob   = with_oob,
            data_width = data_width,
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


# 40-bit datapath ----------------------------------------------------------------------------------

# 10-bit symbols (LSB first): K28.5 (RD-/RD+) and comma-free data symbols (D21.5, D10.2).
K28_5   = [0x17c, 0x283]
SYMBOLS = [0x2aa, 0x155]

CLOCKS = {"full": 4, "half": 8}

def bits_of(words, width):
    return [(w >> i) & 1 for w in words for i in range(width)]

def words_of(bits, width):
    return [sum(b << i for i, b in enumerate(bits[n:n + width]))
        for n in range(0, len(bits) - width + 1, width)]

class TestSerDesECP540Bit(unittest.TestCase):
    def test_pll_5gbps(self):
        config = SerDesECP5PLL.compute_config(250e6, 5e9)
        self.assertEqual((config["m"], config["d"]), (20, 1))
        with self.assertRaises(ValueError):
            SerDesECP5PLL.compute_config(100e6, 5e9) # x25 max: 2.5Gbps.

    def test_clock_cycles_read_only(self):
        # Clock cycles: latched by hardware, read-only.
        serdes = TestSerDesECP5.serdes()
        serdes.add_clock_cycles()
        self.assertIsInstance(serdes.clock_tx_cycles, CSRStatus)
        self.assertIsInstance(serdes.clock_rx_cycles, CSRStatus)

    def test_40bit_requires_oob(self):
        with self.assertRaises(AssertionError):
            TestSerDesECP5.serdes(data_width=40)

    def test_40bit_interface(self):
        serdes = TestSerDesECP5.serdes(with_oob=True, data_width=40)
        self.assertEqual(serdes.nwords, 4)
        self.assertEqual(len(serdes.tx_pattern),  40)
        self.assertEqual(len(serdes.rx_raw_data), 40)
        self.assertEqual(serdes.tx_clk_freq, 3e9/40)
        self.assertTrue(hasattr(serdes, "cd_tx_full"))
        self.assertTrue(hasattr(serdes, "cd_rx_full"))
        self.assertIsInstance(serdes.rx_aligner, SerDesECP5SymbolAligner)

    def test_tx_gearbox_order(self):
        random.seed(0)
        dut   = SerDesECP5TXGearbox()
        words = [random.getrandbits(40) for _ in range(64)]
        out   = []
        def half_gen():
            for w in words:
                yield dut.sink.eq(w)
                yield
        def full_gen():
            for _ in range(2*len(words) + 8):
                out.append((yield dut.source))
                yield
        run_simulation(dut, {"half": half_gen(), "full": full_gen()}, clocks=CLOCKS)
        # 20-bit words: LSBs then MSBs of each 40-bit word, in order (after the gearbox latency).
        expected = [h for w in words for h in [w & 0xfffff, w >> 20]]
        stream   = "".join(f"{w:05x}" for w in out)
        self.assertIn("".join(f"{w:05x}" for w in expected[:100]), stream)

    def check_rx_offset(self, offset):
        class DUT(LiteXModule):
            def __init__(self):
                self.gearbox = SerDesECP5RXGearbox()
                self.aligner = ClockDomainsRenamer("half")(SerDesECP5SymbolAligner())
                self.comb += self.aligner.sink.eq(self.gearbox.source)

        # Symbols stream: Ordered Sets like (4 COMs) and data, shifted by offset bits.
        symbols = []
        rd = 0
        for _ in range(48):
            for _ in range(4):
                symbols.append(K28_5[rd])
                rd ^= 1
            symbols += [random.choice(SYMBOLS) for _ in range(12)]
        bits     = [random.getrandbits(1) for _ in range(offset)] + bits_of(symbols, 10)
        rx_words = words_of(bits, 20)

        dut = DUT()
        out = []
        def full_gen():
            for w in rx_words:
                yield dut.gearbox.sink.eq(w)
                yield
        def half_gen():
            for _ in range(len(rx_words)//2):
                out.append((yield dut.aligner.source))
                yield
        run_simulation(dut, {"full": full_gen(), "half": half_gen()}, clocks=CLOCKS)

        # After the alignment (first Ordered Sets), the output symbols are the transmitted ones.
        rx_symbols = words_of(bits_of(out[len(out)//4:], 40), 10)
        expected   = "".join(f"{s:03x}" for s in symbols)
        received   = "".join(f"{s:03x}" for s in rx_symbols[:len(rx_symbols) - 8])
        self.assertGreater(len(rx_symbols), len(symbols)//2)
        self.assertIn(received, expected)

    def test_rx_gearbox_symbol_aligner(self):
        random.seed(0)
        for offset in range(40):
            with self.subTest(offset=offset):
                self.check_rx_offset(offset)


if __name__ == "__main__":
    unittest.main()


class TestSerDesECP5PCIe(unittest.TestCase):
    @staticmethod
    def serdes(linerate=2.5e9, **kwargs):
        class PLL:
            refclk = Signal()
            config = {
                "m":        25,
                "d":        1,
                "linerate": linerate,
            }

        class Pads:
            p = Signal()
            n = Signal()

        return SerDesECP5(
            pll        = PLL(),
            tx_pads    = Pads(),
            rx_pads    = Pads(),
            with_oob   = True,
            data_width = 40,
            **kwargs
        )

    def test_pcie_is_opt_in(self):
        serdes = self.serdes()
        for name in ["p_CHX_PCIE_MODE", "i_CHX_FFC_PCIE_DET_EN", "i_CHX_FFC_PCIE_CT", "o_CHX_FFS_PCIE_DONE", "o_CHX_FFS_PCIE_CON"]:
            self.assertNotIn(name, serdes.serdes_params)

    def test_pcie_mode_and_detection_ports(self):
        serdes = self.serdes(pcie=True)
        params = serdes.serdes_params
        self.assertEqual(params["p_CHX_PCIE_MODE"],    "0b1")
        self.assertEqual(params["p_D_TX_MAX_RATE"],    "2.5")
        self.assertEqual(params["p_CHX_CDR_MAX_RATE"], "2.5")
        self.assertIs(params["i_CHX_FFC_PCIE_DET_EN"], serdes.pcie_det_en)
        self.assertIs(params["i_CHX_FFC_PCIE_CT"],     serdes.pcie_ct)
        self.assertIs(params["o_CHX_FFS_PCIE_DONE"],   serdes.pcie_done)
        self.assertIs(params["o_CHX_FFS_PCIE_CON"],    serdes.pcie_con)
        self.assertIs(params["o_CHX_FFS_RLOS"],        serdes.rx_los)

    def test_pcie_is_gen1_only(self):
        with self.assertRaises(AssertionError):
            self.serdes(linerate=3e9, pcie=True)

    def test_pcie_analog_options(self):
        with self.assertRaises(AssertionError):
            self.serdes(pcie=True, pcie_analog="unknown")
        default = self.serdes(pcie=True, pcie_analog="default").serdes_params
        wizard  = self.serdes(pcie=True, pcie_analog="wizard").serdes_params
        # "wizard" (the default option) replaces the 10BSER sample code attributes.
        self.assertEqual(self.serdes(pcie=True).serdes_params["p_CHX_RTERM_RX"], wizard["p_CHX_RTERM_RX"])
        self.assertEqual(wizard["p_CHX_RTERM_RX"], "0d22")
        self.assertNotEqual(default["p_CHX_RTERM_RX"], "0d22")
