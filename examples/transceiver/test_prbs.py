#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Proof of Concept to test PRBS/BER with LiteICLink.

import time
import sys

from litex import RemoteClient

PRBS7  = 0b01
PRBS15 = 0b10
PRBS31 = 0b11

def main():
    wb = RemoteClient()
    wb.open()

    print("{:08x}".format(wb.regs.ctrl_scratch.read()))
    exit()


    # # #

    class GTYTester:
        def reset(self):
            if hasattr(wb.regs, "gty_clock_aligner_disable"):
                wb.regs.gty_clock_aligner_disable.write(0)
            wb.regs.gty_tx_prbs_config.write(0)
            wb.regs.gty_rx_prbs_config.write(0)
            wb.regs.gty_loopback.write(0)
            time.sleep(1)

        def prbs_test(self, config):
            print("PRBS test ({})... ".format(config))
            prbs_configs = {
                "prbs7":  PRBS7,
                "prbs15": PRBS15,
                "prbs31": PRBS31
            }
            if hasattr(wb.regs, "gty_clock_aligner_disable"):
                wb.regs.gty_clock_aligner_disable.write(1)
            wb.regs.gty_tx_prbs_config.write(prbs_configs[config])
            wb.regs.gty_rx_prbs_config.write(prbs_configs[config])
            errors_current = 0
            errors_last    = 0
            errors_total   = 0
            duration       = 0
            interval       = 10
            while True:
                duration += interval
                errors_current = (wb.regs.gty_rx_prbs_errors.read() - errors_last)
                errors_total  += errors_current
                errors_last = errors_current
                print("Errors: {:8d} / Duration: {:8d}s / BER: {:1.3e} ".format(
                    errors_current,
                    duration,
                    errors_total/(duration*5e9)))
                time.sleep(interval)

    gty_tester = GTYTester()
    gty_tester.reset()
    if "prbs7" in sys.argv[1:]:
        gty_tester.prbs_test("prbs7")
    elif "prbs15" in sys.argv[1:]:
        gty_tester.prbs_test("prbs15")
    elif "prbs31" in sys.argv[1:]:
        gty_tester.prbs_test("prbs31")
    else:
        print("missing test, supported: (prbs7, prbs15, prbs31)")

    # # #

    wb.close()

if __name__ == "__main__":
    main()