#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# LiteICLink SerWB test utility.

import sys
import time
import argparse

from litex import RemoteClient

from litescope import LiteScopeAnalyzerDriver

# Identifier Test ----------------------------------------------------------------------------------

def ident_test(port):
    bus = RemoteClient(port=port)
    bus.open()

    fpga_identifier = ""

    for i in range(256):
        c = chr(bus.read(bus.bases.identifier_mem + 4*i) & 0xff)
        fpga_identifier += c
        if c == "\0":
            break

    print(fpga_identifier)

    bus.close()

# Init Test ----------------------------------------------------------------------------------------

def init_test(port):
    bus = RemoteClient(port=port)
    bus.open()

    # Reset SerWB Master.
    # -------------------
    print("Reseting SerWB Master...")
    bus.regs.serwb_master_phy_control_reset.write(1)

    # Initialize SerWB Master.
    # ------------------------
    timeout = 4
    print("Initializing SerWB Master...")
    while (bus.regs.serwb_master_phy_control_ready.read() == 0 and
           bus.regs.serwb_master_phy_control_error.read() == 0 and
           timeout > 0):
        time.sleep(0.1)
        print(f"{timeout:2.2f}s", end="\r")
        sys.stdout.flush()
        timeout -= 0.1
    print("")
    if (timeout <= 0):
        print("Failed.")
        return
    else:
        print("Success.")

    # Show Master Config.
    # -------------------
    print("Master config")
    print("-------------")
    if hasattr(bus.regs, "serwb_master_phy_control_delay"):
        print(f"delay_min_found: {bus.regs.serwb_master_phy_control_delay_min_found.read()}")
        print(f"delay_min: {bus.regs.serwb_master_phy_control_delay_min.read()}")
        print(f"delay_max_found: {bus.regs.serwb_master_phy_control_delay_max_found.read()}")
        print(f"delay_max: {bus.regs.serwb_master_phy_control_delay_max.read()}")
        print(f"delay: {bus.regs.serwb_master_phy_control_delay.read()}")
    print(f"bitslip: {bus.regs.serwb_master_phy_control_shift.read()}")
    if hasattr(bus.regs, "serwb_master_phy_control_phase"):
        print(f"phase: {bus.regs.serwb_master_phy_control_phase.read()}")
    print(f"ready: {bus.regs.serwb_master_phy_control_ready.read()}")
    print(f"error: {bus.regs.serwb_master_phy_control_error.read()}")
    print("")

    # Show Slave Config.
    # ------------------
    print("Slave config")
    print("------------")
    if hasattr(bus.regs, "serwb_slave_phy_control_delay"):
        print(f"delay_min_found: {bus.regs.serwb_slave_phy_control_delay_min_found.read()}")
        print(f"delay_min: {bus.regs.serwb_slave_phy_control_delay_min.read()}")
        print(f"delay_max_found: {bus.regs.serwb_slave_phy_control_delay_max_found.read()}")
        print(f"delay_max: {bus.regs.serwb_slave_phy_control_delay_max.read()}")
        print(f"delay: {bus.regs.serwb_slave_phy_control_delay.read()}")
    print(f"bitslip: {bus.regs.serwb_slave_phy_control_shift.read()}")
    if hasattr(bus.regs, "serwb_slave_phy_control_phase"):
        print(f"phase: {bus.regs.serwb_slave_phy_control_phase.read()}")
    print(f"ready: {bus.regs.serwb_slave_phy_control_ready.read()}")
    print(f"error: {bus.regs.serwb_slave_phy_control_error.read()}")

    bus.close()

# SRAM Test ----------------------------------------------------------------------------------------

def sram_test(port):
    bus = RemoteClient(port=port)
    bus.open()

    def mem_dump(base, length):
        for addr in range(base, base + length, 4):
            if (addr % 16 == 0):
                if addr != base:
                    print("")
                print(f"0x{addr:08x}", end="  ")
            data = bus.read(addr)
            for i in reversed(range(4)):
                print(f"{(data >> (8*i)) & 0xff:02x}", end=" ")
        print("")

    def mem_write(base, datas):
        for n, addr in enumerate(range(base, base + 4*len(datas), 4)):
            if (addr % 16 == 0):
                if addr != base:
                    print("")
                print(f"0x{addr:08x}", end="  ")
            data = datas[n]
            for i in reversed(range(4)):
                print(f"{(data >> (8*i)) & 0xff:02x}", end=" ")
            bus.write(addr, data)
        print("")

    print("Fill SRAM with counter:")
    mem_write(bus.mems.serwb.base, [i for i in range(128)])
    print("")

    print("Dump SRAM:")
    mem_dump(bus.mems.serwb.base, 128)
    print("")

    print("Fill SRAM with 4 32-bit words:")
    mem_write(bus.mems.serwb.base, [0x01234567, 0x89abcdef, 0x5aa55aa5, 0xa55aa55a])
    print("")

    print("Dump SRAM:")
    mem_dump(bus.mems.serwb.base, 128)
    print("")

    bus.close()

# Access Test ----------------------------------------------------------------------------------------

def access_test(port):
    bus = RemoteClient(port=port)
    bus.open()

    data = 0x12345678
    addr = 0x100

    print(f"Write over SerWB at 0x{addr:08x}: 0x{data:08x}.")
    bus.write(bus.mems.serwb.base + addr, data)
    print(f"Read  over SerWB at 0x{addr:08x}: 0x{bus.read(bus.mems.serwb.base + addr):08x}.")

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink SerWB test utility")
    parser.add_argument("--port",   default="1234",      help="Host bind port")
    parser.add_argument("--ident",  action="store_true", help="Read FPGA identifier")
    parser.add_argument("--init",   action="store_true", help="Initialize SerWB Link")
    parser.add_argument("--sram",   action="store_true", help="Test SRAM access over SerWB")
    parser.add_argument("--access", action="store_true", help="Test single Write/Read access over SerWB")
    args = parser.parse_args()

    port = int(args.port, 0)

    if args.ident:
        ident_test(port=port)

    if args.init:
        init_test(port=port)

    if args.sram:
        sram_test(port=port)

    if args.access:
        access_test(port=port)

if __name__ == "__main__":
    main()
