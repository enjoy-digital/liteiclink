#!/usr/bin/env python3
import sys
import time

from litex.soc.tools.remote import RemoteClient

from litescope import LiteScopeAnalyzerDriver

wb = RemoteClient()
wb.open()

# # #

def seed_to_data(seed, random=True):
    if random:
        return (1664525*seed + 1013904223) & 0xffffffff
    else:
        return seed

def write_pattern(length):
    for i in range(length):
        wb.write(wb.mems.serwb.base + 4*i, seed_to_data(i))

def check_pattern(length, debug=False):
    errors = 0
    for i in range(length):
        error = 0
        read_data = wb.read(wb.mems.serwb.base + 4*i)
        if read_data != seed_to_data(i):
            error = 1
            if debug:
                print("{}: 0x{:08x}, 0x{:08x}   KO".format(i, read_data, seed_to_data(i)))
        else:
            if debug:
                print("{}: 0x{:08x}, 0x{:08x} OK".format(i, read_data, seed_to_data(i)))
        errors += error
    return errors

if len(sys.argv) < 2:
    print("missing test (init, prbs, wishbone, analyzer)")
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

elif sys.argv[1] == "prbs":
    print("PRBS Master --> Slave test:")
    wb.regs.serwb_master_phy_control_prbs_tx_config.write(0b01)
    wb.regs.serwb_slave_phy_control_prbs_rx_config.write(0b01)
    for i in range(10):
        print("errors : %d" %wb.regs.serwb_slave_phy_control_prbs_rx_errors.read())
        time.sleep(1)
    wb.regs.serwb_master_phy_control_prbs_tx_config.write(0b00)
    wb.regs.serwb_slave_phy_control_prbs_rx_config.write(0b00)

    print("PRBS Slave --> Master test:")
    wb.regs.serwb_slave_phy_control_prbs_tx_config.write(0b01)
    wb.regs.serwb_master_phy_control_prbs_rx_config.write(0b01)
    for i in range(10):
        print("errors : %d" %wb.regs.serwb_master_phy_control_prbs_rx_errors.read())
        time.sleep(1)
    wb.regs.serwb_slave_phy_control_prbs_tx_config.write(0b00)
    wb.regs.serwb_master_phy_control_prbs_rx_config.write(0b00)


elif sys.argv[1] == "wishbone":
    write_pattern(128)
    errors = check_pattern(128, debug=False)
    print("errors: {:d}".format(errors))

elif sys.argv[1] == "analyzer":
    analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
    analyzer.configure_trigger(cond={"serwb_master_core_wishbone_bus_cyc" : 1})
    #analyzer.configure_trigger(cond={"serwb_slave_phy_serdes_rx_d" : 0xfd})    
    analyzer.run(offset=32, length=256)

    # test write
    #wb.regs.serwb_test_do_write.write(1)
    
    # read
    wb.regs.serwb_test_do_read.write(1)

    analyzer.wait_done()
    analyzer.upload()
    analyzer.save("dump.vcd")

else:
    raise ValueError

# # #

wb.close()
