#!/usr/bin/env python3
import sys
import time

from litex.soc.tools.remote import RemoteClient

wb = RemoteClient()
wb.open()

# # #

if len(sys.argv) < 2:
    print("missing test (init)")
    wb.close()
    exit()

if sys.argv[1] == "init":
    wb.regs.serwb_master_phy_control_reset.write(1)
    timeout = 40
    print("SERWB init", end="")
    while (wb.regs.serwb_master_phy_control_ready.read() == 0 and
           wb.regs.serwb_master_phy_control_error.read() == 0 and
           timeout > 0):
        time.sleep(0.1)
        print(".", end="")
        sys.stdout.flush()
        timeout -= 1
    print("")
    print("Master configuration")
    print("-----------------")
    print("delay_min_found: {:d}".format(wb.regs.serwb_master_phy_control_delay_min_found.read()))
    print("delay_min: {:d}".format(wb.regs.serwb_master_phy_control_delay_min.read()))
    print("delay_max_found: {:d}".format(wb.regs.serwb_master_phy_control_delay_max_found.read()))
    print("delay_max: {:d}".format(wb.regs.serwb_master_phy_control_delay_max.read()))    
    print("delay: {:d}".format(wb.regs.serwb_master_phy_control_delay.read()))
    print("bitslip: {:d}".format(wb.regs.serwb_master_phy_control_bitslip.read()))
    print("ready: {:d}".format(wb.regs.serwb_master_phy_control_ready.read()))
    print("error: {:d}".format(wb.regs.serwb_master_phy_control_error.read()))
    print("")
    print("Slave configuration")
    print("-----------------")
    print("delay_min_found: {:d}".format(wb.regs.serwb_slave_phy_control_delay_min_found.read()))
    print("delay_min: {:d}".format(wb.regs.serwb_slave_phy_control_delay_min.read()))
    print("delay_max_found: {:d}".format(wb.regs.serwb_slave_phy_control_delay_max_found.read()))
    print("delay_max: {:d}".format(wb.regs.serwb_slave_phy_control_delay_max.read()))    
    print("delay: {:d}".format(wb.regs.serwb_slave_phy_control_delay.read()))
    print("bitslip: {:d}".format(wb.regs.serwb_slave_phy_control_bitslip.read()))
    print("ready: {:d}".format(wb.regs.serwb_slave_phy_control_ready.read()))
    print("error: {:d}".format(wb.regs.serwb_slave_phy_control_error.read()))

else:
    raise ValueError

# # #

wb.close()
