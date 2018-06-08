#!/usr/bin/env python3
import sys
import time

from litex.soc.tools.remote import RemoteClient
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver

wb = RemoteClient()
wb.open()

# # #

def help():
    print("Supported triggers:")
    print(" - converter")
    print(" - encoder")
    print(" - fsm")
    print(" ")
    print(" - continous")
    exit()

if len(sys.argv) < 2:
    help()
elif sys.argv[1] == "help":
    help()

if len(sys.argv) < 3:
    length = 256
else:
    length = int(sys.argv[2])

groups = {
    "converter":  0,
    "encoder":    1,
    "control":    2
}

group = sys.argv[1]

analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
if group == "converter":
    analyzer.configure_group(groups["converter"])
    analyzer.configure_trigger(cond={})
elif group == "encoder":
    analyzer.configure_group(groups["encoder"])
    analyzer.configure_trigger(cond={})
elif group == "control":
    analyzer.configure_group(groups["fsm"])
    analyzer.configure_trigger(cond={})
elif group == "continuous":
    continuous_items = [
        # group, name
        (2, "soc_serwb_master_phy_serdes_reset"),
        (2, "soc_serwb_master_phy_init_state"),
        (2, "soc_serwb_master_phy_serdes_tx_idle0"),
        (2, "soc_serwb_master_phy_serdes_tx_comma0"),
        (2, "soc_serwb_master_phy_serdes_rx_idle0"),
        (2, "soc_serwb_master_phy_serdes_rx_comma0"),
        (2, "soc_serwb_slave_phy_serdes_reset"),
        (2, "soc_serwb_slave_phy_init_state"),
        (2, "soc_serwb_slave_phy_serdes_tx_idle0"),
        (2, "soc_serwb_slave_phy_serdes_tx_comma0"),
        (2, "soc_serwb_slave_phy_serdes_rx_idle0"),
        (2, "soc_serwb_slave_phy_serdes_rx_comma0"),
    ]
    while True:
        print("="*40)
        for (group, name) in continuous_items:
            value = analyzer.get_instant_value(group, name)
            print("{} : 0x{:x}".format(name, value))
else:
    raise ValueError

analyzer.run(offset=32, length=length)
analyzer.wait_done()
analyzer.upload()
analyzer.save(group + ".vcd")
analyzer.save(group + ".py")

# # #

wb.close()
