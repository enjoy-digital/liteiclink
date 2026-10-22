#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest
import random

from migen import *

from litex.gen import *
from litex.gen.sim import *

from litex.soc.interconnect     import wishbone
from litex.soc.interconnect.axi import AXILiteInterface, AXILite2Wishbone, RESP_OKAY

from litex.soc.interconnect import stream

from liteiclink.serwb.core     import SERWBCore, SERIOCore
from liteiclink.serwb.datapath import TXDatapath, RXDatapath, FlowControl

# Slow tests (skipped with LITEICLINK_QUICK_TESTS=1, for quick local iterations) ------------------

slow_test = unittest.skipIf(os.environ.get("LITEICLINK_QUICK_TESTS", "0") == "1", "Slow test (LITEICLINK_QUICK_TESTS=1).")

# Loopback PHY -------------------------------------------------------------------------------------

class LoopbackInit(LiteXModule):
    def __init__(self):
        self.ready = Signal()

class LoopbackPHY(LiteXModule):
    """Word-level PHY model: like the real PHYs, the link carries one word every `ratio` cycles
    and has no backpressure (RX words are pushed to the core whatever its ready)."""
    def __init__(self, ratio=5):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32)])
        self.init   = LoopbackInit()
        self.tx     = stream.Endpoint([("data", 32)])
        self.rx     = stream.Endpoint([("data", 32)])

        # # #

        ce = Signal(ratio, reset=1)
        self.sync += ce.eq(Cat(ce[1:], ce[0]))

        self.comb += [
            # TX: Always transmitting (data=0 when idle), as real PHYs do.
            self.tx.valid.eq(ce[0]),
            If(self.init.ready,
                sink.ready.eq(ce[0]),
                If(sink.valid, self.tx.data.eq(sink.data)),
            ),
            # RX.
            If(self.init.ready,
                source.valid.eq(self.rx.valid),
                source.data.eq(self.rx.data),
            ),
        ]

# Datapath PHY -------------------------------------------------------------------------------------

class DatapathPHY(LiteXModule):
    """PHY model using the real TX/RX datapaths (8b10b, K characters, optional flow control) with
    an 8-bit parallel wire. As on hardware, RX datapath has no backpressure."""
    def __init__(self, with_flow_control=False):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32)])
        self.init   = LoopbackInit()
        self.tx     = TXDatapath(8)
        self.rx     = RXDatapath(8)

        # # #

        self.comb += self.tx.source.ready.eq(1)
        if with_flow_control:
            serdes = Module() # FlowControl expects SerDes-like tx/rx with datapath/endpoints.
            serdes.tx, serdes.rx = Module(), Module()
            serdes.tx.datapath, serdes.tx.sink   = self.tx, self.tx.sink
            serdes.rx.datapath, serdes.rx.source = self.rx, self.rx.source
            self.flow_control = FlowControl(serdes, self.init.ready)
            self.pause = self.flow_control.pause
            self.comb += [
                sink.connect(self.flow_control.sink),
                self.flow_control.source.connect(source),
            ]
        else:
            # Same data-path than SERWBPHY.
            self.comb += [
                If(self.init.ready,
                    If(sink.valid, sink.connect(self.tx.sink)),
                    self.rx.source.connect(source),
                ).Else(
                    self.rx.source.ready.eq(1),
                ),
                self.tx.sink.valid.eq(1),
            ]

    def connect_to(self, other, flip=0):
        # flip: Optional bit errors injection mask (8-bit wire).
        return [
            other.rx.sink.valid.eq(self.tx.source.valid),
            other.rx.sink.data.eq(self.tx.source.data ^ flip),
        ]

# Slow Wishbone Slave ------------------------------------------------------------------------------

class SlowAck(LiteXModule):
    """Delays Wishbone acks by `latency` cycles."""
    def __init__(self, master, slave, latency=64):
        count = Signal(max=latency + 1)
        self.comb += [
            master.connect(slave, omit={"stb", "ack"}),
            slave.stb.eq(master.stb & (count == latency)),
            master.ack.eq(slave.ack),
        ]
        self.sync += [
            If(~(master.cyc & master.stb) | slave.ack,
                count.eq(0)
            ).Elif(count != latency,
                count.eq(count + 1)
            )
        ]

# Error Wishbone Slave -----------------------------------------------------------------------------

class ErrorAbove(LiteXModule):
    """Returns Wishbone errors for accesses at/above `adr` (forwards others)."""
    def __init__(self, master, slave, adr=32):
        error = Signal()
        self.comb += [
            error.eq(master.adr >= adr),
            master.connect(slave, omit={"stb", "cyc", "ack", "err"}),
            slave.cyc.eq(master.cyc & ~error),
            slave.stb.eq(master.stb & ~error),
            master.ack.eq(slave.ack),
            master.err.eq(master.cyc & master.stb & error),
        ]

# DUT ----------------------------------------------------------------------------------------------

class DUTLink(LiteXModule):
    """SerWB link between two PHYs:

    bus -> SERWBCore("slave") -> PHY <-> PHY -> SERWBCore("master") -> SRAM
    """
    def __init__(self, connected=True, with_serio=False, serio_extra_port=False,
        phy="loopback", with_flow_control=False, sram_latency=0, error_above=None, **kwargs):
        # Link state (link up when connected), and bit errors injection (datapath PHY, SoC to remote).
        self.link_up = Signal(reset=1)
        self.flip    = Signal(8)

        # PHYs.
        if phy == "loopback":
            self.master_phy = LoopbackPHY()
            self.slave_phy  = LoopbackPHY()
            if connected:
                self.comb += [
                    self.master_phy.tx.connect(self.slave_phy.rx),
                    self.slave_phy.tx.connect(self.master_phy.rx),
                ]
        else:
            self.master_phy = DatapathPHY(with_flow_control)
            self.slave_phy  = DatapathPHY(with_flow_control)
            if connected:
                self.comb += self.master_phy.connect_to(self.slave_phy, flip=self.flip)
                self.comb += self.slave_phy.connect_to(self.master_phy)
        if connected:
            self.comb += [
                self.master_phy.init.ready.eq(self.link_up),
                self.slave_phy.init.ready.eq(self.link_up),
            ]

        # Cores.
        self.master_core = SERWBCore(self.master_phy, clk_freq=int(1e6), mode="slave",  **kwargs)
        self.slave_core  = SERWBCore(self.slave_phy,  clk_freq=int(1e6), mode="master", **kwargs)

        # SRAM.
        sram_bus = self.slave_core.bus
        if error_above is not None:
            _sram_bus  = wishbone.Interface()
            self.error = ErrorAbove(sram_bus, _sram_bus, adr=error_above)
            sram_bus   = _sram_bus
        if sram_latency:
            _sram_bus = wishbone.Interface()
            self.slow = SlowAck(sram_bus, _sram_bus, latency=sram_latency)
            sram_bus  = _sram_bus
        self.sram = wishbone.SRAM(128, bus=sram_bus) # 32 words.

        # SerIO.
        if with_serio:
            self.master_serio = SERIOCore(self.master_core)
            self.slave_serio  = SERIOCore(self.slave_core)

        # Extra SerIO on a port unknown to the remote side (packets must be dropped there).
        if serio_extra_port:
            self.extra_serio = SERIOCore(self.master_core, port=2)

        self.bus = self.master_core.bus

class DUTLinkAXILite(DUTLink):
    """AXI-Lite SoC side, through the standard LiteX AXILite2Wishbone bridge."""
    def __init__(self, **kwargs):
        DUTLink.__init__(self, **kwargs)
        self.axi_lite = AXILiteInterface(data_width=32, address_width=32)
        self.axi_lite2wishbone = AXILite2Wishbone(self.axi_lite, self.master_core.bus)

# Helpers ------------------------------------------------------------------------------------------

def wait_link_ready(dut, timeout=100000):
    for _ in range(timeout):
        if (yield dut.master_phy.init.ready) and (yield dut.slave_phy.init.ready):
            return
        yield
    raise TimeoutError("SerWB link not ready.")

def bus_access(bus, adr, we=0, dat_w=0, sel=0b1111, timeout=100000):
    """Single Wishbone access, returns (dat_r, err)."""
    yield bus.adr.eq(adr)
    yield bus.we.eq(we)
    yield bus.dat_w.eq(dat_w)
    yield bus.sel.eq(sel)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(timeout):
        if (yield bus.ack) or (yield bus.err):
            break
        yield
    else:
        raise TimeoutError("Wishbone access timeout.")
    dat_r = (yield bus.dat_r)
    err   = (yield bus.err)
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield
    return dat_r, err

def sel_mask(sel):
    return sum(0xff << (8*i) for i in range(4) if sel & (1 << i))

# Test SERWB Link --------------------------------------------------------------------------------------

class TestSERWBLink(unittest.TestCase):
    def wishbone_test(self, n=8, with_sel=False, **kwargs):
        prng   = random.Random(42)
        errors = []
        dut    = DUTLink(**kwargs)
        def generator():
            yield from wait_link_ready(dut)
            ref = {}
            # Full word writes.
            for i in range(n):
                ref[i] = prng.randrange(2**32)
                _, err = yield from bus_access(dut.bus, i, we=1, dat_w=ref[i])
                errors.append(err)
            # Partial writes (byte enables).
            if with_sel:
                for i in range(n):
                    sel   = prng.randrange(1, 16)
                    dat_w = prng.randrange(2**32)
                    mask  = sel_mask(sel)
                    ref[i] = (ref[i] & ~mask) | (dat_w & mask)
                    _, err = yield from bus_access(dut.bus, i, we=1, dat_w=dat_w, sel=sel)
                    errors.append(err)
            # Reads.
            for i in range(n):
                dat_r, err = yield from bus_access(dut.bus, i)
                errors.append(err)
                self.assertEqual(dat_r, ref[i], f"Mismatch at {i}")
        run_simulation(dut, generator())
        self.assertEqual(sum(errors), 0)
        return dut

    def test_wishbone(self):
        self.wishbone_test()

    def test_wishbone_byte_enables(self):
        self.wishbone_test(with_sel=True)

    def test_wishbone_etherbone_buffer_depth(self):
        dut = self.wishbone_test(n=4, etherbone_buffer_depth=8)
        # Etherbone record receiver must be sized from etherbone_buffer_depth (8 data words + base
        # address -> 16 entries).
        mems = [m for m in dut.master_core.etherbone.record.receiver.fifo._fragment.specials if isinstance(m, Memory)]
        self.assertEqual([m.depth for m in mems], [16])
        self.assertEqual(dut.master_core.tx_fifo.depth, 2 + 2 + 1 + 1 + 8)

    def test_wishbone_unknown_port(self):
        # SerIO packets sent to a port not present on the remote side must not disturb Wishbone.
        dut = DUTLink(serio_extra_port=True)
        def generator():
            yield from wait_link_ready(dut)
            for i in range(4):
                yield dut.extra_serio.i.eq(0x100 + i)
                _, err = yield from bus_access(dut.bus, i, we=1, dat_w=0x1000 + i)
                self.assertEqual(err, 0)
            for i in range(4):
                dat_r, err = yield from bus_access(dut.bus, i)
                self.assertEqual((dat_r, err), (0x1000 + i, 0))
        run_simulation(dut, generator())

    @slow_test
    def test_serio(self):
        dut     = DUTLink(with_serio=True)
        results = []
        def generator():
            yield from wait_link_ready(dut)
            for value in [0x12345678, 0xdeadbeef]:
                yield dut.master_serio.i.eq(value)
                yield dut.slave_serio.i.eq(~value & 0xffffffff)
                for _ in range(2000):
                    yield
                results.append(((yield dut.slave_serio.o), (yield dut.master_serio.o)))
        run_simulation(dut, generator())
        self.assertEqual(results, [(v, ~v & 0xffffffff) for v in [0x12345678, 0xdeadbeef]])

    def test_axi_lite(self):
        prng = random.Random(42)
        dut  = DUTLinkAXILite()
        def generator():
            ref = {}
            for i in range(2):
                ref[i] = prng.randrange(2**32)
                resp = yield from dut.axi_lite.write(4*i, ref[i])
                self.assertEqual(resp, RESP_OKAY)
            # Partial write (strobes -> byte enables).
            resp = yield from dut.axi_lite.write(0, 0xaabbccdd, strb=0b0101)
            self.assertEqual(resp, RESP_OKAY)
            ref[0] = (ref[0] & ~sel_mask(0b0101)) | (0xaabbccdd & sel_mask(0b0101))
            for i in range(2):
                data, resp = yield from dut.axi_lite.read(4*i)
                self.assertEqual((data, resp), (ref[i], RESP_OKAY))
        run_simulation(dut, generator())

    def test_datapath_phy(self):
        # Real TX/RX datapaths (8b10b) with and without flow control.
        self.wishbone_test(n=2, phy="datapath")
        self.wishbone_test(n=2, phy="datapath", with_flow_control=True)

    @slow_test
    def test_datapath_phy_crc(self):
        # CRC + flow control over real TX/RX datapaths.
        self.wishbone_test(n=2, phy="datapath", with_flow_control=True, with_crc=True)

    def write_stress_test(self, n=8, **kwargs):
        # Back-to-back posted writes to a slow remote slave: without flow control the remote RX
        # buffer overflows and writes are lost.
        dut     = DUTLink(phy="datapath", sram_latency=64, **kwargs)
        results = []
        def generator():
            for i in range(n):
                yield from bus_access(dut.bus, i, we=1, dat_w=0x1000 + i, timeout=2000)
            # Let remote side drain and check SRAM content.
            for _ in range(n*80):
                yield
            for i in range(n):
                results.append((yield dut.sram.mem[i]))
        run_simulation(dut, generator())
        return results == [0x1000 + i for i in range(n)]

    @slow_test
    def test_flow_control_write_stress(self):
        self.assertTrue(self.write_stress_test(with_flow_control=True))
        self.assertFalse(self.write_stress_test(with_flow_control=False))

    # Native protocol ------------------------------------------------------------------------------

    def test_native_wishbone(self):
        self.wishbone_test(protocol="native", with_sel=True)

    def test_native_datapath_phy(self):
        # Real TX/RX datapaths with flow control + CRC.
        self.wishbone_test(n=2, protocol="native", phy="datapath", with_flow_control=True, with_crc=True)

    def test_native_write_coalescing(self):
        # Back-to-back consecutive writes must be coalesced in burst_depth requests, and a read must
        # see previous (pending) writes.
        dut      = DUTLink(protocol="native", burst_depth=4)
        requests = []
        @passive
        def monitor():
            source = dut.master_core.mmap.source
            while True:
                if (yield source.valid) and (yield source.ready) and (yield source.last):
                    requests.append(1)
                yield
        def generator():
            for i in range(8):
                _, err = yield from bus_access(dut.bus, i, we=1, dat_w=0x100 + i)
                self.assertEqual(err, 0)
            # Read right after writes: pending writes are sent first.
            dat_r, err = yield from bus_access(dut.bus, 7)
            self.assertEqual((dat_r, err), (0x107, 0))
            for i in [0, 3, 4]:
                dat_r, err = yield from bus_access(dut.bus, i)
                self.assertEqual((dat_r, err), (0x100 + i, 0))
        run_simulation(dut, [generator(), monitor()])
        # 2 write requests (2x4 words) + 4 read requests.
        self.assertEqual(len(requests), 2 + 4)

    def test_native_flow_control_write_stress(self):
        # Coalesced writes (bursts) to a slow remote slave with flow control.
        self.assertTrue(self.write_stress_test(protocol="native", with_flow_control=True))

    def test_native_remote_error(self):
        # Remote Wishbone errors are reported on reads (Etherbone does not report them).
        dut = DUTLink(protocol="native", error_above=32)
        def generator():
            _, err = yield from bus_access(dut.bus, 1, we=1, dat_w=0x1234)
            dat_r, err = yield from bus_access(dut.bus, 1)
            self.assertEqual((dat_r, err), (0x1234, 0))
            _, err = yield from bus_access(dut.bus, 40)
            self.assertEqual(err, 1)
            dat_r, err = yield from bus_access(dut.bus, 1)
            self.assertEqual((dat_r, err), (0x1234, 0))
        run_simulation(dut, generator())

    def test_native_link_down_error(self):
        dut = DUTLink(connected=False, protocol="native")
        def generator():
            for _ in range(100):
                yield
            _, err = yield from bus_access(dut.bus, 0, timeout=100)
            self.assertEqual(err, 1)
        run_simulation(dut, generator())

    def test_link_recovery(self):
        # Link down during traffic: accesses return err, then link recovers when back up.
        dut = DUTLink(protocol="native")
        def generator():
            _, err = yield from bus_access(dut.bus, 0, we=1, dat_w=0x1111)
            self.assertEqual(err, 0)
            # Let posted write reach remote side (writes in flight are lost on link down).
            for _ in range(200):
                yield
            # Read in flight when link goes down.
            yield dut.bus.adr.eq(0)
            yield dut.bus.we.eq(0)
            yield dut.bus.cyc.eq(1)
            yield dut.bus.stb.eq(1)
            for _ in range(10):
                yield
            yield dut.link_up.eq(0)
            for _ in range(100):
                if (yield dut.bus.err) or (yield dut.bus.ack):
                    break
                yield
            self.assertEqual((yield dut.bus.err), 1)
            yield dut.bus.cyc.eq(0)
            yield dut.bus.stb.eq(0)
            for _ in range(10):
                yield
            # Link back up.
            yield dut.link_up.eq(1)
            for _ in range(10):
                yield
            _, err = yield from bus_access(dut.bus, 1, we=1, dat_w=0x2222)
            self.assertEqual(err, 0)
            for i, v in [(0, 0x1111), (1, 0x2222)]:
                dat_r, err = yield from bus_access(dut.bus, i)
                self.assertEqual((dat_r, err), (v, 0))
        run_simulation(dut, generator())

    def bit_errors_test(self, with_crc, n=12, rate=0.03, seed=1):
        # Random bit errors on the link (SoC to remote side), native protocol, writes only. Returns
        # the remote SRAM content.
        dut     = DUTLink(phy="datapath", protocol="native", with_crc=with_crc)
        prng    = random.Random(seed)
        results = []
        flipping = [True]
        @passive
        def injector():
            while True:
                flip = 0
                if flipping[0] and (prng.random() < rate):
                    flip = 1 << prng.randrange(8)
                yield dut.flip.eq(flip)
                yield
        def generator():
            for i in range(n):
                yield from bus_access(dut.bus, i, we=1, dat_w=0x1000 + i, timeout=2000)
                for _ in range(prng.randrange(16)):
                    yield
            for _ in range(400):
                yield
            flipping[0] = False
            for i in range(n):
                results.append((yield dut.sram.mem[i]))
        run_simulation(dut, [generator(), injector()])
        return results

    @slow_test
    def test_bit_errors_crc(self):
        # With CRC, corrupted requests are dropped: remote SRAM words are either written correctly
        # or not written (0), never corrupted.
        n       = 12
        results = self.bit_errors_test(with_crc=True, n=n)
        self.assertTrue(all(r in [0, 0x1000 + i] for i, r in enumerate(results)), [hex(r) for r in results])
        self.assertGreater(sum(r == 0x1000 + i for i, r in enumerate(results)), 0)
        # Without CRC, same errors corrupt remote SRAM (checks errors are effectively injected).
        results = self.bit_errors_test(with_crc=False, n=n)
        self.assertFalse(all(r in [0, 0x1000 + i] for i, r in enumerate(results)), [hex(r) for r in results])

    def test_link_down_error(self):
        # Without link, accesses must return an error instead of stalling the bus.
        dut = DUTLink(connected=False)
        def generator():
            for _ in range(100):
                yield
            _, err = yield from bus_access(dut.bus, 0, we=1, dat_w=0x12345678, timeout=100)
            self.assertEqual(err, 1)
            _, err = yield from bus_access(dut.bus, 0, timeout=100)
            self.assertEqual(err, 1)
        run_simulation(dut, generator())

if __name__ == "__main__":
    unittest.main()
