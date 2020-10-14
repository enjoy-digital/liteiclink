#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# LiteICLink PRBS/BER test utility.

import sys
import time
import argparse

from litex import RemoteClient

# Constants ----------------------------------------------------------------------------------------

prbs_modes = {
    "prbs7":  0b01,
    "prbs15": 0b10,
    "prbs31": 0b11,
}

near_end_pma_loopback = 0b10

# PRBS Test ----------------------------------------------------------------------------------------

def prbs_test(port=1234, mode="prbs7", loopback=False, duration=60):
    wb = RemoteClient(port=port)
    wb.open()
    wb.regs.ctrl_scratch.read()

    def serdes_reset():
        print("Reseting SerDes...")
        if hasattr(wb.regs, "serdes_clock_aligner_disable"):
            wb.regs.serdes_clock_aligner_disable.write(0)
        wb.regs.serdes_tx_prbs_config.write(0)
        wb.regs.serdes_rx_prbs_config.write(0)
        time.sleep(1)

    # Enable Loopback
    if loopback:
        print("Enabling SerDes loopback...")
        wb.regs.serdes_loopback.write(near_end_pma_loopback)

    # Reset SerDes
    serdes_reset()

    # Configure PRBS
    print(f"Configuring SerDes for {mode.upper()}...")
    wb.regs.serdes_tx_prbs_config.write(prbs_modes[mode])
    wb.regs.serdes_rx_prbs_config.write(prbs_modes[mode])

    # Run PRBS/BER Test
    print("Running PRBS/BER test...")
    errors_current   = 0
    errors_last      = 0
    errors_total     = 0
    duration_current = 0
    interval         = 1
    try:
        while duration_current < duration:
            # Interval / Duration
            time.sleep(interval)
            duration_current += interval
            # Errors
            errors_current = (wb.regs.serdes_rx_prbs_errors.read() - errors_last)
            errors_total  += errors_current
            errors_last = errors_current
            # Log
            print("Errors: {:12d} / Duration: {:8d}s / BER: {:1.3e} ".format(
                errors_current,
                duration_current,
                errors_total/(duration_current*5e9)))
    except KeyboardInterrupt:
        pass

    # Reset SerDes
    serdes_reset()

    # Disable Loopback
    if loopback:
        print("Disabling SerDes loopback...")
        wb.regs.serdes_loopback.write(0)

    wb.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteICLink PRBS/BER test utility")
    parser.add_argument("--port",      default="1234",      help="Host bind port")
    parser.add_argument("--mode",      default="prbs7",     help="PRBS mode: prbs7 (default), prbs15 or prbs31")
    parser.add_argument("--duration",  default="60",        help="Test duration (default=10)")
    parser.add_argument("--loopback",  action="store_true", help="Enable internal loopback")
    args = parser.parse_args()

    prbs_test(
        port     = int(args.port, 0),
        mode     = args.mode,
        loopback = args.loopback,
        duration = int(args.duration, 0)
    )

if __name__ == "__main__":
    main()
