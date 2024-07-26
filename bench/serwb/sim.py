#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.sim              import SimPlatform
from litex.build.sim.config       import SimConfig

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import wishbone

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.builder  import *

from liteeth.phy.model          import LiteEthPHYModel
from liteeth.core               import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from liteiclink.serwb.genphy import SERWBPHY
from liteiclink.serwb.core   import SERWBCore, SERIOCore

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),

    # Serial.
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data",  Pins(8)),
    ),
    # Ethernet.
    ("eth", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# SerWBMinSoC --------------------------------------------------------------------------------------

class SerWBMinSoC(SoCMini):
    def __init__(self):
        platform     = Platform()
        sys_clk_freq = int(1e6)

        self.comb += platform.trace.eq(1)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform.request("sys_clk"))

        # SerWB ------------------------------------------------------------------------------------

        # Pads.
        # -----
        serwb_master_pads = Record([("tx", 1), ("rx", 1)])
        serwb_slave_pads  = Record([("tx", 1), ("rx", 1)])
        self.comb += serwb_slave_pads.rx.eq(serwb_master_pads.tx)
        self.comb += serwb_master_pads.rx.eq(serwb_slave_pads.tx)

        # Master.
        # -------
        self.serwb_master_phy = serwb_master_phy = SERWBPHY(
            device       = platform.device,
            pads         = serwb_master_pads,
            mode         = "master",
            init_timeout = 128
        )

        # Slave.
        # ------
        self.serwb_slave_phy = serwb_slave_phy = SERWBPHY(
            device       = platform.device,
            pads         = serwb_slave_pads,
            mode         ="slave",
            init_timeout = 128,
        )

        # Sim Timer.
        # ----------
        timer = Signal(32)
        self.sync += timer.eq(timer + 1)

        # Sim Gen/Check.
        # --------------
        class EndpointCounterGenerator(Module):
            def __init__(self, name, ready, endpoint):
                counter = Signal(32)
                self.sync += [
                    endpoint.valid.eq(ready),
                    If(endpoint.valid & endpoint.ready,
                        endpoint.data.eq(endpoint.data + 1)
                    )
                ]

        class EndpointCounterChecker(Module):
            def __init__(self, name, ready, endpoint):
                last_data = Signal(32, reset=2**32-1)
                self.sync += [
                    endpoint.ready.eq(ready),
                    If(endpoint.valid & endpoint.ready,
                        #Display("[%08d] {} %08x".format(name), timer, endpoint.data),
                        If((endpoint.data - 1) != last_data,
                            Display("[%08d] {} check error.".format(name), timer)
                        ),
                        last_data.eq(endpoint.data),
                    )
                ]

        ready = serwb_master_phy.init.ready & serwb_slave_phy.init.ready

        # Master gen/check.
        # -----------------
        self.submodules += EndpointCounterGenerator("Master", ready=ready, endpoint=serwb_master_phy.sink)
        self.submodules += EndpointCounterChecker("Master",  ready=ready, endpoint=serwb_master_phy.source)

        # Slave gen/check.
        # ----------------
        self.submodules += EndpointCounterGenerator("Slave",  ready=ready, endpoint=serwb_slave_phy.sink)
        self.submodules += EndpointCounterChecker("Slave",  ready=ready, endpoint=serwb_slave_phy.source)


        # Simu Status.
        # ------------
        serwb_master_phy.init.fsm.finalize()
        serwb_slave_phy.init.fsm.finalize()
        for phy, fsm in [
            ["master",  serwb_master_phy.init.fsm],
            ["slave ",  serwb_slave_phy.init.fsm]]:
            for state, value in fsm.encoding.items():
                self.sync += [
                    If(fsm.next_state != fsm.state,
                        If(fsm.next_state == value,
                            Display("[%08d] {} entering {} state.".format(phy.upper(), state), timer)
                        )
                    )
                ]

        # Simulation End
        self.sync += If(timer > 10000, Finish())

# SerWBSoC -----------------------------------------------------------------------------------------

class SerWBSoC(SoCCore):
    def __init__(self, with_serio=False, with_debug=False, write_stress=False, read_stress=False, alt_stress=False):
        platform     = Platform()
        sys_clk_freq = int(1e6)

        self.comb += platform.trace.eq(1)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq,
            ident         = "LiteICLink SerWB bench simulation",
            ident_version = True,
            with_uart     = True,
            uart_name     = "sim",
        )

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform.request("sys_clk"))

        # Etherbone---------------------------------------------------------------------------------
        self.ethphy = LiteEthPHYModel(self.platform.request("eth"))
        self.add_etherbone(phy=self.ethphy)

        # SerWB ------------------------------------------------------------------------------------
        # SerWB simple test with a SerWB Master added as a Slave peripheral to the SoC and connected
        # to a SerWB Slave with a SRAM attached. Access to this SRAM is then tested from the main
        # SoC through SerWB:
        #                   +--------+    +------+    +------+    +------+
        #                   |        |    |      +-ck->      |    |      |
        #                   |  Test  +----+SerWB +-tx->SerWB +----> Test |
        #                   |   SoC  | WB |Master|    |Slave | WB | SRAM |
        #                   |        +<---+      <-rx-+      <----+      |
        #                   +--------+    +------+    +------+    +------+
        # ------------------------------------------------------------------------------------------

        # Pads.
        # -----
        serwb_master_pads = Record([("tx", 1), ("rx", 1)])
        serwb_slave_pads  = Record([("tx", 1), ("rx", 1)])
        self.comb += serwb_slave_pads.rx.eq(serwb_master_pads.tx)
        self.comb += serwb_master_pads.rx.eq(serwb_slave_pads.tx)

        # Master.
        # -------
        self.serwb_master_phy = SERWBPHY(
            device       = platform.device,
            pads         = serwb_master_pads,
            mode         = "master",
            init_timeout = 128,
        )

        # Slave.
        # ------
        self.serwb_slave_phy = SERWBPHY(
            device       = platform.device,
            pads         = serwb_slave_pads,
            mode         ="slave",
            init_timeout = 128,
        )

        # Wishbone Slave.
        # ---------------
        self.serwb_master_core = serwb_master_core = SERWBCore(
            phy      = self.serwb_master_phy,
            clk_freq = self.clk_freq,
            mode     = "slave",
            etherbone_buffer_depth = 4,
            tx_buffer_depth        = 8,
            rx_buffer_depth        = 8,
        )

        # Wishbone Master.
        # ----------------
        self.serwb_slave_core = serwb_slave_core = SERWBCore(
            phy      = self.serwb_slave_phy,
            clk_freq = self.clk_freq,
            mode     = "master",
            etherbone_buffer_depth = 4,
            tx_buffer_depth        = 8,
            rx_buffer_depth        = 8,

        )

        # Wishbone SRAM.
        # --------------
        self.serwb_sram = wishbone.SRAM(8192)
        self.bus.add_slave("serwb", serwb_master_core.bus, SoCRegion(origin=0x30000000, size=8192))
        self.comb += serwb_slave_core.bus.connect(self.serwb_sram.bus)

        # SerIO ------------------------------------------------------------------------------------
        if with_serio:
            # IOs Master.
            # ----------
            self.serio_master_core = serio_master_core = SERIOCore(serwb_core=serwb_master_core)

            # IOs Slave.
            # ----------
            self.serio_slave_core = serio_slave_core = SERIOCore(serwb_core=serwb_slave_core)

            # IOs Test.
            # ---------
            count = Signal(14)
            self.sync += count.eq(count + 1)
            self.sync += If(count == 0, serio_master_core.i.eq(serio_master_core.i + 1))

            o_d = Signal(32)
            self.sync += o_d.eq(serio_slave_core.o)
            self.sync += If(serio_slave_core.o != o_d, Display("o %d", serio_slave_core.o))

        # Debug ------------------------------------------------------------------------------------

        if with_debug:
            # Latency measurements.
            latency_m2s_display = Signal()
            latency_ack_display = Signal()
            latency_s2m_display = Signal()
            latency_count       = Signal(32)
            self.latency_fsm = latency_fsm = FSM(reset_state="IDLE")
            latency_fsm.act("IDLE",
                NextValue(latency_count, 0),
                If(serwb_master_core.bus.stb & serwb_master_core.bus.cyc,
                    NextState("M2S-MEASURE")
                )
            )
            latency_fsm.act("M2S-MEASURE",
                NextValue(latency_count, latency_count + 1),
                If(serwb_slave_core.bus.stb & serwb_slave_core.bus.cyc,
                    latency_m2s_display.eq(1),
                    NextValue(latency_count, 0),
                    NextState("ACK-MEASURE")
                )
            )
            self.sync += If(latency_m2s_display, Display("M2S Latency: %d Cycles.", latency_count))
            latency_fsm.act("ACK-MEASURE",
                NextValue(latency_count, latency_count + 1),
                If(serwb_slave_core.bus.ack,
                    latency_ack_display.eq(1),
                    NextValue(latency_count, 0),
                    NextState("S2M-MEASURE")
                )
            )
            self.sync += If(latency_ack_display, Display("ACk Latency: %d Cycles.", latency_count))
            latency_fsm.act("S2M-MEASURE",
                NextValue(latency_count, latency_count + 1),
                If(serwb_master_core.bus.ack,
                    latency_s2m_display.eq(1),
                    NextValue(latency_count, 0),
                    NextState("IDLE")
                )
            )
            self.sync += If(latency_s2m_display, Display("S2M Latency: %d Cycles.", latency_count))

        # Stress -----------------------------------------------------------------------------------

        if write_stress or read_stress:
            # Delay SRAM Ack.
            self.comb += self.serwb_sram.bus.ack.eq(0)
            serwb_fsm_count = Signal(32)
            self.serwb_fsm = serwb_fsm = FSM(reset_state="IDLE")
            serwb_fsm.act("IDLE",
                If(serwb_slave_core.bus.stb & serwb_slave_core.bus.cyc,
                    NextValue(serwb_fsm_count, 0),
                    NextState("WAIT")
                )
            )
            serwb_fsm.act("WAIT",
                NextValue(serwb_fsm_count, serwb_fsm_count + 1),
                If(serwb_fsm_count == (16 - 1),
                    NextState("ACK")
                )
            )
            serwb_fsm.act("ACK",
                self.serwb_sram.bus.ack.eq(1),
                NextState("IDLE")
            )

        if write_stress:
            write_bus = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
            self.bus.add_master("write_stress", write_bus)
            self.write_fsm = write_fsm = FSM(reset_state="IDLE")
            write_fsm.act("IDLE",
                NextState("WRITE")
            )
            write_count = Signal(32)
            write_fsm.act("WRITE",
                write_bus.stb.eq(1),
                write_bus.cyc.eq(1),
                write_bus.we.eq(1),
                write_bus.adr.eq(0x30000000),
                write_bus.sel.eq(0b1111),
                write_bus.dat_w.eq(0x12345678),
                If(write_bus.ack,
                    NextValue(write_count, write_count + 1),
                    NextState("IDLE")
                )
            )
            self.sync += If(write_bus.stb & write_bus.cyc & write_bus.ack, Display("Write %d", write_count))

        if read_stress:
            read_bus = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
            self.bus.add_master("read_stress", read_bus)
            self.read_fsm = read_fsm = FSM(reset_state="IDLE")
            read_fsm.act("IDLE",
                NextState("WRITE")
            )
            read_count = Signal(32)
            read_fsm.act("WRITE",
                read_bus.stb.eq(1),
                read_bus.cyc.eq(1),
                read_bus.we.eq(0),
                read_bus.adr.eq(0x30000000),
                read_bus.sel.eq(0b1111),
                If(read_bus.ack,
                    NextValue(read_count, read_count + 1),
                    NextState("IDLE")
                )
            )
            self.sync += If(read_bus.stb & read_bus.cyc & read_bus.ack, Display("Read %d", read_count))

        if alt_stress:
            alt_bus = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
            self.bus.add_master("alt_stress", alt_bus)
            self.alt_fsm = alt_fsm = FSM(reset_state="IDLE")
            alt_fsm.act("IDLE",
                NextState("WRITE")
            )
            write_count = Signal(32)
            alt_fsm.act("WRITE",
                alt_bus.stb.eq(1),
                alt_bus.cyc.eq(1),
                alt_bus.we.eq(1),
                alt_bus.adr.eq(0x30000000),
                alt_bus.sel.eq(0b1111),
                alt_bus.dat_w.eq(0x12345678),
                If(alt_bus.ack,
                    NextValue(write_count, write_count + 1),
                    NextState("WRITE-CHECK"),
                )
            )
            alt_fsm.act("WRITE-CHECK",
                If(write_count == 4,
                    NextValue(write_count, 0),
                    NextState("READ")
                ).Else(
                    NextState("WRITE")
                )
            )
            self.sync += If(alt_bus.stb & alt_bus.cyc & alt_bus.ack, Display("Write %d", write_count))

            read_count = Signal(32)
            alt_fsm.act("READ",
                alt_bus.stb.eq(1),
                alt_bus.cyc.eq(1),
                alt_bus.we.eq(0),
                alt_bus.adr.eq(0x30000000),
                alt_bus.sel.eq(0b1111),
                If(alt_bus.ack,
                    NextValue(read_count, read_count + 1),
                    NextState("READ-CHECK")
                )
            )
            alt_fsm.act("READ-CHECK",
                If(read_count == 4,
                    NextValue(read_count, 0),
                    NextState("IDLE")
                ).Else(
                    NextState("READ")
                )
            )
            self.sync += If(alt_bus.stb & alt_bus.cyc & alt_bus.ack, Display("Read %d", read_count))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SerWB LiteX/Verilator Simulation.")
    parser.add_argument("--min",          action="store_true", help="Run Minimal simulation (SerSBMinSoC).")
    parser.add_argument("--with-serio",   action="store_true", help="Enable SerIO.")
    parser.add_argument("--with-debug",   action="store_true", help="Enable Debug traces.")
    parser.add_argument("--write-stress", action="store_true", help="Enable Write Stress.")
    parser.add_argument("--read-stress",  action="store_true", help="Enable Read Stress.")
    parser.add_argument("--alt-stress",   action="store_true", help="Enable Alternate Write/Read Stress.")
    parser.add_argument("--trace",        action="store_true", help="Enable tracing.")
    parser.add_argument("--trace-start",  default="0",         help="Cycle to start VCD tracing.")
    parser.add_argument("--trace-end",    default="-1",        help="Cycle to end VCD tracing.")
    parser.add_argument("--trace-fst",    action="store_true", help="Use .fst format for tracing.")
    args = parser.parse_args()

    sim_config = SimConfig()
    sim_config.add_clocker("sys_clk", freq_hz=1e6)
    if not args.min:
        sim_config.add_module("serial2console", "serial")
        sim_config.add_module("ethernet", "eth", args={"interface": "tap0", "ip": "192.168.1.100"})

    soc     = SerWBMinSoC() if args.min else SerWBSoC(
        with_serio   = args.with_serio,
        with_debug   = args.with_debug,
        write_stress = args.write_stress,
        read_stress  = args.read_stress,
        alt_stress   = args.alt_stress,
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(sim_config=sim_config,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end),
        trace_fst   = int(args.trace_fst),
    )

if __name__ == "__main__":
    main()
