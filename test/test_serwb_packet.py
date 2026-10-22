#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
import random

from migen import *

from litex.gen import *
from litex.gen.sim import *

from litex.soc.interconnect import stream

from liteiclink.serwb.packet import Packetizer, Depacketizer, CRCDepacketizer, packet_description

# DUT ----------------------------------------------------------------------------------------------

class DUTPacket(LiteXModule):
    def __init__(self, timeout=1e-3, with_crc=False, corrupt_word=None, corrupt_mask=1):
        self.packetizer = Packetizer(with_crc=with_crc)
        if with_crc:
            self.depacketizer = CRCDepacketizer(clk_freq=int(1e6), timeout=timeout, max_words=16)
        else:
            self.depacketizer = Depacketizer(clk_freq=int(1e6), timeout=timeout)
        self.comb += self.packetizer.source.connect(self.depacketizer.sink)
        # Optional corruption of the Nth word transmitted (corrupt_mask bits flipped).
        if corrupt_word is not None:
            count = Signal(16)
            self.sync += If(self.packetizer.source.valid & self.packetizer.source.ready, count.eq(count + 1))
            self.comb += If(count == corrupt_word,
                self.depacketizer.sink.data.eq(self.packetizer.source.data ^ corrupt_mask)
            )

# Helpers ------------------------------------------------------------------------------------------

def packet_generator(sink, packets, prng, valid_rand=50):
    for port, datas in packets:
        for i, data in enumerate(datas):
            # Random bubbles, with last/data held high during bubbles to stress FSM conditions.
            while prng.randrange(100) < valid_rand:
                yield sink.valid.eq(0)
                yield sink.last.eq(1)
                yield
            yield sink.valid.eq(1)
            yield sink.port.eq(port)
            yield sink.length.eq(4*len(datas))
            yield sink.data.eq(data)
            yield sink.last.eq(i == len(datas) - 1)
            yield
            while not (yield sink.ready):
                yield
        yield sink.valid.eq(0)
        yield sink.last.eq(0)

def packet_checker(source, packets, results, prng, ready_rand=50, timeout=10000, expected=None):
    current  = []
    cycles   = 0
    expected = len(packets) if expected is None else expected
    while (len(results) < expected) and (cycles < timeout):
        yield source.ready.eq(prng.randrange(100) >= ready_rand)
        yield
        cycles += 1
        if (yield source.valid) and (yield source.ready):
            current.append((yield source.data))
            if (yield source.last):
                results.append(((yield source.port), current))
                current = []

def words_generator(sink, words):
    for word in words:
        yield sink.valid.eq(1)
        yield sink.data.eq(word)
        yield
        while not (yield sink.ready):
            yield
    yield sink.valid.eq(0)

def words_checker(source, results, cycles, delay=0):
    for _ in range(delay):
        yield
    yield source.ready.eq(1)
    yield
    for _ in range(cycles):
        if (yield source.valid) and (yield source.ready):
            results.append(((yield source.port), (yield source.data), (yield source.last)))
        yield

# Test Packet --------------------------------------------------------------------------------------

class TestSERWBPacket(unittest.TestCase):
    def test_packet_loopback(self):
        prng    = random.Random(42)
        packets = [(prng.randrange(4), [prng.randrange(2**32) for _ in range(prng.randrange(1, 9))])
            for _ in range(32)]
        results = []
        dut     = DUTPacket()
        run_simulation(dut, [
            packet_generator(dut.packetizer.sink, packets, random.Random(1)),
            packet_checker(dut.depacketizer.source, packets, results, random.Random(2)),
        ])
        self.assertEqual(results, packets)

    def crc_loopback_test(self, packets, expected=None, **kwargs):
        results = []
        dut     = DUTPacket(with_crc=True, **kwargs)
        crc_errors = []
        @passive
        def monitor():
            while True:
                if (yield dut.depacketizer.crc_error):
                    crc_errors.append(1)
                yield
        run_simulation(dut, [
            packet_generator(dut.packetizer.sink, packets, random.Random(1)),
            packet_checker(dut.depacketizer.source, packets, results, random.Random(2), timeout=4000, expected=expected),
            monitor(),
        ])
        return results, len(crc_errors)

    def test_crc_packet_loopback(self):
        prng    = random.Random(42)
        packets = [(prng.randrange(4), [prng.randrange(2**32) for _ in range(prng.randrange(1, 9))])
            for _ in range(32)]
        results, crc_errors = self.crc_loopback_test(packets)
        self.assertEqual((results, crc_errors), (packets, 0))

    def test_crc_corrupted_packet_dropped(self):
        # Packet 1 (words 7..13: preamble, header, 4 data, crc) has a corrupted data word.
        packets = [(1, [0x100 + i for i in range(4)]) for _ in range(3)]
        results, crc_errors = self.crc_loopback_test(packets, expected=2, corrupt_word=7 + 3)
        self.assertEqual((results, crc_errors), ([packets[0], packets[2]], 1))

    def test_crc_corrupted_crc_upper_bits_dropped(self):
        # CRC16 is zero-extended: upper bits of the CRC word (word 13) must also be checked.
        packets = [(1, [0x100 + i for i in range(4)]) for _ in range(3)]
        results, crc_errors = self.crc_loopback_test(packets, expected=2, corrupt_word=7 + 6, corrupt_mask=1 << 31)
        self.assertEqual((results, crc_errors), ([packets[0], packets[2]], 1))

    def test_crc_oversized_packet_dropped(self):
        # Payload larger than max_words (16) must be dropped, next packet received.
        packets = [(1, list(range(20))), (2, [0xcafe])]
        results, _ = self.crc_loopback_test(packets, expected=1)
        self.assertEqual(results, [packets[1]])

    def test_depacketizer_zero_length(self):
        # A zero-length header must not swallow the following packet.
        dut     = Depacketizer(clk_freq=int(1e6))
        results = []
        words   = [0x5aa55aa5, (0 << 8) | 0, 0x5aa55aa5, (8 << 8) | 1, 0x11111111, 0x22222222]
        run_simulation(dut, [words_generator(dut.sink, words), words_checker(dut.source, results, 32)])
        self.assertEqual(results, [(1, 0x11111111, 0), (1, 0x22222222, 1)])

    def test_depacketizer_no_timeout_on_backpressure(self):
        # A packet stalled by downstream (longer than timeout) must not be dropped.
        dut     = Depacketizer(clk_freq=int(1e6), timeout=20e-6)
        results = []
        words   = [0x5aa55aa5, (8 << 8) | 3, 0x11111111, 0x22222222]
        run_simulation(dut, [words_generator(dut.sink, words), words_checker(dut.source, results, 16, delay=100)])
        self.assertEqual(results, [(3, 0x11111111, 0), (3, 0x22222222, 1)])

    def test_depacketizer_timeout(self):
        # A truncated packet must be dropped after timeout and the next packet received.
        dut     = Depacketizer(clk_freq=int(1e6), timeout=50e-6)
        results = []
        def generator():
            yield from words_generator(dut.sink, [0x5aa55aa5, (16 << 8) | 0, 0xdeadbeef])
            for _ in range(100):
                yield
            yield from words_generator(dut.sink, [0x5aa55aa5, (4 << 8) | 2, 0xcafecafe])
        run_simulation(dut, [generator(), words_checker(dut.source, results, 200)])
        self.assertEqual(results, [(0, 0xdeadbeef, 0), (2, 0xcafecafe, 1)])

if __name__ == "__main__":
    unittest.main()
