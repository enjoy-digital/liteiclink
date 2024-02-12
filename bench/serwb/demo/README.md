[> LiteICLink's SerWB Demo
--------------------------

This demo showcases the SerWB protocol to link two FPGA boards - the ECPIX-5 and iCEBreaker - using
a minimal three-wire interface. It's a hands-on guide for FPGA developers looking to explore
efficient FPGA-to-FPGA communication via the Wishbone bus with LiteICLink's SerWB.

![](https://github.com/enjoy-digital/liteiclink/assets/1450143/09e0597a-b4f0-49c7-a3b5-ab9ea61900e5)


[> Build and Load iCEBreaker Peripheral SoC
-------------------------------------------

./icebreaker.py --build --load

[> Build and Load ECPIX-5 Main SoC
----------------------------------

./ecpix5.py --build --load

While building the ECPIX-5 design, the SoC mapping of the iCEBreaker FPGA is re-integrated in the
LiteX Builder and mapping of the combined peripheral + main SoC can be generated to regular LiteX
exports (.h, .csv, .json, etc...)


[> Test the Design
------------------

Once the two FPGAs are loaded, the ECPIX-5 should respond to ping at 192.168.1.50 and the two red
leds of the ECPIX-5 and iCEBreaker should be up, indicating a LiteICLink SerWB's Link up.

![](https://github.com/enjoy-digital/liteiclink/assets/1450143/8045d0f0-3977-4891-bd45-da6d05a1d48f)


Access over Etherbone to the peripherlas of the ECXPIX-5 and iCEBreaker can be tested with the
following commands:

    litex_server --udp
    litex_cli --regs
    0x00000000 : 0x00000000 ctrl_reset
    0x00000004 : 0x12345678 ctrl_scratch
    0x00000008 : 0x00000000 ctrl_bus_errors
    0x00000800 : 0x00000000 ethphy_crg_reset
    0x00000804 : 0x00000005 ethphy_rx_inband_status
    0x00000808 : 0x00000000 ethphy_mdio_w
    0x0000080c : 0x00000001 ethphy_mdio_r
    0x00001800 : 0x00000000 serwb_master_phy_control_reset
    0x00001804 : 0x00000001 serwb_master_phy_control_ready
    0x00001808 : 0x00000000 serwb_master_phy_control_error
    0x0000180c : 0x00000014 serwb_master_phy_control_shift
    0x00001810 : 0x00000000 serwb_master_phy_control_prbs_start
    0x00001814 : 0x00000000 serwb_master_phy_control_prbs_cycles
    0x00001818 : 0x00000000 serwb_master_phy_control_prbs_errors
    0x30000000 : 0x00000000 icebreaker_soc_ctrl_reset
    0x30000004 : 0x12345678 icebreaker_soc_ctrl_scratch
    0x30000008 : 0x00000000 icebreaker_soc_ctrl_bus_errors
    0x30001000 : 0x00000001 icebreaker_soc_serwb_slave_phy_control_ready
    0x30001004 : 0x00000000 icebreaker_soc_serwb_slave_phy_control_error
    0x30001008 : 0x00000018 icebreaker_soc_serwb_slave_phy_control_shift
    0x3000100c : 0x00000000 icebreaker_soc_serwb_slave_phy_control_prbs_start
    0x30001010 : 0x00000000 icebreaker_soc_serwb_slave_phy_control_prbs_cycles