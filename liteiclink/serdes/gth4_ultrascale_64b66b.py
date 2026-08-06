#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
UltraScale+ GTHE4 64b66b transceiver wrapper (asynchronous gearbox).

Exposes a raw 64-bit data + 2-bit sync header interface per direction, suitable
for 64b66b based protocols (10GBASE-R style PCS, JESD204C, ...):
- TX: 64-bit data + 2-bit header consumed every TXUSRCLK2 cycle.
- RX: 64-bit data + 2-bit header produced every RXUSRCLK2 cycle, with a gearbox
  slip control (rx_slip) for sync header alignment.

Clocking (GTH 64b66b async gearbox, TX/RX_INT_DATAWIDTH=32-bit):
- TX/RXUSRCLK  = linerate/33 (internal PCS clock, from TX/RXPROGDIVCLK /16.5).
- TX/RXUSRCLK2 = linerate/66 (fabric clock, one 66-bit block per cycle).

The GTHE4_CHANNEL attributes are generated with the Vivado GT wizard
(gtwizard_ultrascale v1.7) for: 16.22016 Gbps linerate, 245.76 MHz refclk,
QPLL0 (FBDIV=66, VCO=16.22016 GHz, HALF rate), 64B66B_ASYNC encoding/decoding,
DFE equalization, on an UltraScale+ -2 speed grade. They are expected to hold
for nearby linerates (~12-16.375 Gbps); regenerate for distant rates.

Hardware PRBS generators/checkers (PMA-side, gearbox-bypassed) are exposed for
line-level testing with the GT internal loopbacks.
"""

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc       import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from liteiclink.serdes.common              import *
from liteiclink.serdes.gth_ultrascale_init import GTHTXInit, GTHRXInit
from liteiclink.serdes.gth4_ultrascale     import GTHQuadPLLBase, GTH4QuadPLL

# GTH4 Quad PLL (high VCO rates) -------------------------------------------------------------------

class GTH4QuadPLLHighRate(GTH4QuadPLL):
    """GTH4QuadPLL with QPLL0 analog settings adjusted for high VCO frequencies.

    The GTH4QuadPLL QPLL0 analog settings are tuned for VCO frequencies up to
    ~13 GHz; the values below come from the Vivado GT wizard for a 16.22016 GHz
    QPLL0 VCO and are applied for VCO frequencies above 14 GHz.
    """
    def __init__(self, refclk, refclk_freq, linerate):
        super().__init__(refclk, refclk_freq, linerate)
        if (self.config["qpll"] == "qpll0") and (self.config["vco_freq"] > 14e9):
            self.gth_params.update(
                p_PPF0_CFG   = 0b0000111100000000,
                p_QPLL0_CFG0 = 0b0011001100111100,
                p_QPLL0_CFG4 = 0b0000000001000101,
                p_QPLL0_LPF  = 0b1100011101,
            )

# GTH4 64b66b --------------------------------------------------------------------------------------

class GTH4_64B66B(LiteXModule):
    def __init__(self, pll, tx_pads, rx_pads, sys_clk_freq,
        tx_polarity = 0,
        rx_polarity = 0,
        pll_master  = True):
        # TX interface.
        self.tx_data   = tx_data   = Signal(64)
        self.tx_header = tx_header = Signal(2)

        # RX interface.
        self.rx_data   = rx_data   = Signal(64)
        self.rx_header = rx_header = Signal(2)
        self.rx_slip   = rx_slip   = Signal()

        # TX controls.
        self.tx_enable  = Signal(reset=1)
        self.tx_ready   = Signal()
        self.tx_inhibit = Signal()

        # RX controls.
        self.rx_enable = Signal(reset=1)
        self.rx_ready  = Signal()

        # Hardware PRBS (PMA-side, bypasses the gearbox; PRBSSEL encoding:
        # 0b0000=off, 0b0001=PRBS7, 0b0011=PRBS15, 0b0100=PRBS23, 0b0101=PRBS31).
        self.tx_prbs_config    = Signal(4)
        self.rx_prbs_config    = Signal(4)
        self.rx_prbs_cnt_reset = Signal()
        self.rx_prbs_errors    = Signal(32)
        self.rx_prbs_locked    = Signal()

        # DRP.
        self.drp = DRPInterface()

        # Loopback (0b000=off, 0b001=Near-End PCS, 0b010=Near-End PMA,
        #           0b100=Far-End PMA, 0b110=Far-End PCS).
        self.loopback = Signal(3)

        # Polarity inversion (lane routing); reset from the constructor parameters.
        self.tx_polarity = Signal(reset=tx_polarity)
        self.rx_polarity = Signal(reset=rx_polarity)

        # Streams.
        self.sink   = stream.Endpoint([("data", 64), ("header", 2)])
        self.source = stream.Endpoint([("data", 64), ("header", 2)])

        # # #

        use_qpll0 = isinstance(pll, GTHQuadPLLBase) and pll.config["qpll"] == "qpll0"
        use_qpll1 = isinstance(pll, GTHQuadPLLBase) and pll.config["qpll"] == "qpll1"
        assert (use_qpll0 or use_qpll1)

        # Transceiver direct clock outputs (useful to specify clock constraints).
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        # Fabric clocks (one 66-bit block per cycle).
        self.tx_clk_freq = pll.config["linerate"]/66
        self.rx_clk_freq = pll.config["linerate"]/66
        # Internal PCS user clocks (constraints only, no fabric logic).
        self.txusrclk_freq = pll.config["linerate"]/33
        self.rxusrclk_freq = pll.config["linerate"]/33

        # Control/Status CDC.
        tx_prbs_config    = Signal(4)
        rx_prbs_config    = Signal(4)
        rx_prbs_cnt_reset = Signal()
        rx_prbs_errors    = Signal(32)
        rx_prbs_locked    = Signal()
        self.specials += [
            MultiReg(self.tx_prbs_config, tx_prbs_config,      "tx"),
            MultiReg(self.rx_prbs_config, rx_prbs_config,      "rx"),
            MultiReg(rx_prbs_errors,      self.rx_prbs_errors, "sys"),
            MultiReg(rx_prbs_locked,      self.rx_prbs_locked, "sys"),
        ]
        self.rx_prbs_cnt_reset_ps = PulseSynchronizer("sys", "rx")
        self.comb += self.rx_prbs_cnt_reset_ps.i.eq(self.rx_prbs_cnt_reset)
        self.comb += rx_prbs_cnt_reset.eq(self.rx_prbs_cnt_reset_ps.o)

        # TX init ----------------------------------------------------------------------------------
        self.tx_init = tx_init = GTHTXInit(sys_clk_freq, buffer_enable=False)
        self.comb += [
            self.tx_ready.eq(tx_init.done),
            tx_init.restart.eq(~self.tx_enable),
        ]

        # RX init ----------------------------------------------------------------------------------
        self.rx_init = rx_init = GTHRXInit(sys_clk_freq, buffer_enable=False)
        self.comb += [
            self.rx_ready.eq(rx_init.done),
            rx_init.restart.eq(~self.rx_enable),
        ]

        # PLL --------------------------------------------------------------------------------------
        self.comb += [
            tx_init.plllock.eq(pll.lock),
            rx_init.plllock.eq(pll.lock),
        ]
        if pll_master:
            self.comb += pll.reset.eq(tx_init.pllreset)

        # DRP mux ----------------------------------------------------------------------------------
        self.drp_mux = drp_mux = DRPMux()
        drp_mux.add_interface(self.drp)

        # GTHE4_CHANNEL instance -------------------------------------------------------------------
        rxphaligndone = Signal()
        txdata_int    = Signal(64)
        txheader_int  = Signal(2)
        txusrclk      = Signal() # linerate/33.
        rxusrclk      = Signal() # linerate/33.
        rx_prbs_err   = Signal()
        # Attributes from the Vivado GT wizard (see module docstring).
        self.gth_params = dict(
            p_ACJTAG_DEBUG_MODE            = 0b0,
            p_ACJTAG_MODE                  = 0b0,
            p_ACJTAG_RESET                 = 0b0,
            p_ADAPT_CFG0                   = 0b0001000000000000,
            p_ADAPT_CFG1                   = 0b1100100000000000,
            p_ADAPT_CFG2                   = 0b0000000000000000,
            p_ALIGN_COMMA_DOUBLE           = "FALSE",
            p_ALIGN_COMMA_ENABLE           = 0b0000000000,
            p_ALIGN_COMMA_WORD             = 1,
            p_ALIGN_MCOMMA_DET             = "FALSE",
            p_ALIGN_MCOMMA_VALUE           = 0b1010000011,
            p_ALIGN_PCOMMA_DET             = "FALSE",
            p_ALIGN_PCOMMA_VALUE           = 0b0101111100,
            p_A_RXOSCALRESET               = 0b0,
            p_A_RXPROGDIVRESET             = 0b0,
            p_A_RXTERMINATION              = 0b1,
            p_A_TXDIFFCTRL                 = 0b01100,
            p_A_TXPROGDIVRESET             = 0b0,
            p_CAPBYPASS_FORCE              = 0b0,
            p_CBCC_DATA_SOURCE_SEL         = "ENCODED",
            p_CDR_SWAP_MODE_EN             = 0b0,
            p_CFOK_PWRSVE_EN               = 0b1,
            p_CHAN_BOND_KEEP_ALIGN         = "FALSE",
            p_CHAN_BOND_MAX_SKEW           = 1,
            p_CHAN_BOND_SEQ_1_1            = 0b0000000000,
            p_CHAN_BOND_SEQ_1_2            = 0b0000000000,
            p_CHAN_BOND_SEQ_1_3            = 0b0000000000,
            p_CHAN_BOND_SEQ_1_4            = 0b0000000000,
            p_CHAN_BOND_SEQ_1_ENABLE       = 0b1111,
            p_CHAN_BOND_SEQ_2_1            = 0b0000000000,
            p_CHAN_BOND_SEQ_2_2            = 0b0000000000,
            p_CHAN_BOND_SEQ_2_3            = 0b0000000000,
            p_CHAN_BOND_SEQ_2_4            = 0b0000000000,
            p_CHAN_BOND_SEQ_2_ENABLE       = 0b1111,
            p_CHAN_BOND_SEQ_2_USE          = "FALSE",
            p_CHAN_BOND_SEQ_LEN            = 1,
            p_CH_HSPMUX                    = 0b0110100001101000,
            p_CKCAL1_CFG_0                 = 0b1100000011000000,
            p_CKCAL1_CFG_1                 = 0b0101000011000000,
            p_CKCAL1_CFG_2                 = 0b0000000000001010,
            p_CKCAL1_CFG_3                 = 0b0000000000000000,
            p_CKCAL2_CFG_0                 = 0b1100000011000000,
            p_CKCAL2_CFG_1                 = 0b1000000011000000,
            p_CKCAL2_CFG_2                 = 0b0000000000000000,
            p_CKCAL2_CFG_3                 = 0b0000000000000000,
            p_CKCAL2_CFG_4                 = 0b0000000000000000,
            p_CKCAL_RSVD0                  = 0b0000000010000000,
            p_CKCAL_RSVD1                  = 0b0000010000000000,
            p_CLK_CORRECT_USE              = "FALSE",
            p_CLK_COR_KEEP_IDLE            = "FALSE",
            p_CLK_COR_MAX_LAT              = 20,
            p_CLK_COR_MIN_LAT              = 18,
            p_CLK_COR_PRECEDENCE           = "TRUE",
            p_CLK_COR_REPEAT_WAIT          = 0,
            p_CLK_COR_SEQ_1_1              = 0b0000000000,
            p_CLK_COR_SEQ_1_2              = 0b0000000000,
            p_CLK_COR_SEQ_1_3              = 0b0000000000,
            p_CLK_COR_SEQ_1_4              = 0b0000000000,
            p_CLK_COR_SEQ_1_ENABLE         = 0b1111,
            p_CLK_COR_SEQ_2_1              = 0b0000000000,
            p_CLK_COR_SEQ_2_2              = 0b0000000000,
            p_CLK_COR_SEQ_2_3              = 0b0000000000,
            p_CLK_COR_SEQ_2_4              = 0b0000000000,
            p_CLK_COR_SEQ_2_ENABLE         = 0b1111,
            p_CLK_COR_SEQ_2_USE            = "FALSE",
            p_CLK_COR_SEQ_LEN              = 1,
            p_CPLL_CFG0                    = 0b0000000111111010,
            p_CPLL_CFG1                    = 0b0000000000100011,
            p_CPLL_CFG2                    = 0b0000000000000010,
            p_CPLL_CFG3                    = 0b0000000000000000,
            p_CPLL_FBDIV                   = 2,
            p_CPLL_FBDIV_45                = 5,
            p_CPLL_INIT_CFG0               = 0b0000001010110010,
            p_CPLL_LOCK_CFG                = 0b0000000111101000,
            p_CPLL_REFCLK_DIV              = 1,
            p_CTLE3_OCAP_EXT_CTRL          = 0b000,
            p_CTLE3_OCAP_EXT_EN            = 0b0,
            p_DDI_CTRL                     = 0b00,
            p_DDI_REALIGN_WAIT             = 15,
            p_DEC_MCOMMA_DETECT            = "FALSE",
            p_DEC_PCOMMA_DETECT            = "FALSE",
            p_DEC_VALID_COMMA_ONLY         = "FALSE",
            p_DELAY_ELEC                   = 0b0,
            p_DMONITOR_CFG0                = 0b0000000000,
            p_DMONITOR_CFG1                = 0b00000000,
            p_ES_CLK_PHASE_SEL             = 0b0,
            p_ES_CONTROL                   = 0b000000,
            p_ES_ERRDET_EN                 = "FALSE",
            p_ES_EYE_SCAN_EN               = "FALSE",
            p_ES_HORZ_OFFSET               = 0b000000000000,
            p_ES_PRESCALE                  = 0b00000,
            p_ES_QUALIFIER0                = 0b0000000000000000,
            p_ES_QUALIFIER1                = 0b0000000000000000,
            p_ES_QUALIFIER2                = 0b0000000000000000,
            p_ES_QUALIFIER3                = 0b0000000000000000,
            p_ES_QUALIFIER4                = 0b0000000000000000,
            p_ES_QUALIFIER5                = 0b0000000000000000,
            p_ES_QUALIFIER6                = 0b0000000000000000,
            p_ES_QUALIFIER7                = 0b0000000000000000,
            p_ES_QUALIFIER8                = 0b0000000000000000,
            p_ES_QUALIFIER9                = 0b0000000000000000,
            p_ES_QUAL_MASK0                = 0b0000000000000000,
            p_ES_QUAL_MASK1                = 0b0000000000000000,
            p_ES_QUAL_MASK2                = 0b0000000000000000,
            p_ES_QUAL_MASK3                = 0b0000000000000000,
            p_ES_QUAL_MASK4                = 0b0000000000000000,
            p_ES_QUAL_MASK5                = 0b0000000000000000,
            p_ES_QUAL_MASK6                = 0b0000000000000000,
            p_ES_QUAL_MASK7                = 0b0000000000000000,
            p_ES_QUAL_MASK8                = 0b0000000000000000,
            p_ES_QUAL_MASK9                = 0b0000000000000000,
            p_ES_SDATA_MASK0               = 0b0000000000000000,
            p_ES_SDATA_MASK1               = 0b0000000000000000,
            p_ES_SDATA_MASK2               = 0b0000000000000000,
            p_ES_SDATA_MASK3               = 0b0000000000000000,
            p_ES_SDATA_MASK4               = 0b0000000000000000,
            p_ES_SDATA_MASK5               = 0b0000000000000000,
            p_ES_SDATA_MASK6               = 0b0000000000000000,
            p_ES_SDATA_MASK7               = 0b0000000000000000,
            p_ES_SDATA_MASK8               = 0b0000000000000000,
            p_ES_SDATA_MASK9               = 0b0000000000000000,
            p_EYE_SCAN_SWAP_EN             = 0b0,
            p_FTS_DESKEW_SEQ_ENABLE        = 0b1111,
            p_FTS_LANE_DESKEW_CFG          = 0b1111,
            p_FTS_LANE_DESKEW_EN           = "FALSE",
            p_GEARBOX_MODE                 = 0b10001,
            p_ISCAN_CK_PH_SEL2             = 0b0,
            p_LOCAL_MASTER                 = 0b1,
            p_LPBK_BIAS_CTRL               = 0b100,
            p_LPBK_EN_RCAL_B               = 0b0,
            p_LPBK_EXT_RCAL                = 0b1000,
            p_LPBK_IND_CTRL0               = 0b000,
            p_LPBK_IND_CTRL1               = 0b000,
            p_LPBK_IND_CTRL2               = 0b000,
            p_LPBK_RG_CTRL                 = 0b1110,
            p_OOBDIVCTL                    = 0b00,
            p_OOB_PWRUP                    = 0b0,
            p_PCI3_AUTO_REALIGN            = "OVR_1K_BLK",
            p_PCI3_PIPE_RX_ELECIDLE        = 0b0,
            p_PCI3_RX_ASYNC_EBUF_BYPASS    = 0b00,
            p_PCI3_RX_ELECIDLE_EI2_ENABLE  = 0b0,
            p_PCI3_RX_ELECIDLE_H2L_COUNT   = 0b000000,
            p_PCI3_RX_ELECIDLE_H2L_DISABLE = 0b000,
            p_PCI3_RX_ELECIDLE_HI_COUNT    = 0b000000,
            p_PCI3_RX_ELECIDLE_LP4_DISABLE = 0b0,
            p_PCI3_RX_FIFO_DISABLE         = 0b0,
            p_PCIE3_CLK_COR_EMPTY_THRSH    = 0b00000,
            p_PCIE3_CLK_COR_FULL_THRSH     = 0b010000,
            p_PCIE3_CLK_COR_MAX_LAT        = 0b00100,
            p_PCIE3_CLK_COR_MIN_LAT        = 0b00000,
            p_PCIE3_CLK_COR_THRSH_TIMER    = 0b001000,
            p_PCIE_BUFG_DIV_CTRL           = 0b0011010100000000,
            p_PCIE_PLL_SEL_MODE_GEN12      = 0b10,
            p_PCIE_PLL_SEL_MODE_GEN3       = 0b10,
            p_PCIE_PLL_SEL_MODE_GEN4       = 0b10,
            p_PCIE_RXPCS_CFG_GEN3          = 0b0000101010100101,
            p_PCIE_RXPMA_CFG               = 0b0010100000001010,
            p_PCIE_TXPCS_CFG_GEN3          = 0b0010010010100100,
            p_PCIE_TXPMA_CFG               = 0b0010100000001010,
            p_PCS_PCIE_EN                  = "FALSE",
            p_PCS_RSVD0                    = 0b0000000000000000,
            p_PD_TRANS_TIME_FROM_P2        = 0b000000111100,
            p_PD_TRANS_TIME_NONE_P2        = 0b00011001,
            p_PD_TRANS_TIME_TO_P2          = 0b01100100,
            p_PREIQ_FREQ_BST               = 1,
            p_PROCESS_PAR                  = 0b010,
            p_RATE_SW_USE_DRP              = 0b1,
            p_RCLK_SIPO_DLY_ENB            = 0b0,
            p_RCLK_SIPO_INV_EN             = 0b0,
            p_RESET_POWERSAVE_DISABLE      = 0b0,
            p_RTX_BUF_CML_CTRL             = 0b010,
            p_RTX_BUF_TERM_CTRL            = 0b00,
            p_RXBUFRESET_TIME              = 0b00011,
            p_RXBUF_ADDR_MODE              = "FAST",
            p_RXBUF_EIDLE_HI_CNT           = 0b1000,
            p_RXBUF_EIDLE_LO_CNT           = 0b0000,
            p_RXBUF_EN                     = "FALSE",
            p_RXBUF_RESET_ON_CB_CHANGE     = "TRUE",
            p_RXBUF_RESET_ON_COMMAALIGN    = "FALSE",
            p_RXBUF_RESET_ON_EIDLE         = "FALSE",
            p_RXBUF_RESET_ON_RATE_CHANGE   = "TRUE",
            p_RXBUF_THRESH_OVFLW           = 0,
            p_RXBUF_THRESH_OVRD            = "FALSE",
            p_RXBUF_THRESH_UNDFLW          = 4,
            p_RXCDRFREQRESET_TIME          = 0b00001,
            p_RXCDRPHRESET_TIME            = 0b00001,
            p_RXCDR_CFG0                   = 0b0000000000000011,
            p_RXCDR_CFG0_GEN3              = 0b0000000000000011,
            p_RXCDR_CFG1                   = 0b0000000000000000,
            p_RXCDR_CFG1_GEN3              = 0b0000000000000000,
            p_RXCDR_CFG2                   = 0b0000001001101001,
            p_RXCDR_CFG2_GEN2              = 0b1001101001,
            p_RXCDR_CFG2_GEN3              = 0b0000001001101001,
            p_RXCDR_CFG2_GEN4              = 0b0000000101100100,
            p_RXCDR_CFG3                   = 0b0000000000010000,
            p_RXCDR_CFG3_GEN2              = 0b010000,
            p_RXCDR_CFG3_GEN3              = 0b0000000000010000,
            p_RXCDR_CFG3_GEN4              = 0b0000000000010010,
            p_RXCDR_CFG4                   = 0b0101110011110110,
            p_RXCDR_CFG4_GEN3              = 0b0101110011110110,
            p_RXCDR_CFG5                   = 0b1011010001101011,
            p_RXCDR_CFG5_GEN3              = 0b0001010001101011,
            p_RXCDR_FR_RESET_ON_EIDLE      = 0b0,
            p_RXCDR_HOLD_DURING_EIDLE      = 0b0,
            p_RXCDR_LOCK_CFG0              = 0b0010001000000001,
            p_RXCDR_LOCK_CFG1              = 0b1001111111111111,
            p_RXCDR_LOCK_CFG2              = 0b0111011111000011,
            p_RXCDR_LOCK_CFG3              = 0b0000000000000001,
            p_RXCDR_LOCK_CFG4              = 0b0000000000000000,
            p_RXCDR_PH_RESET_ON_EIDLE      = 0b0,
            p_RXCFOK_CFG0                  = 0b0000000000000000,
            p_RXCFOK_CFG1                  = 0b1000000000010101,
            p_RXCFOK_CFG2                  = 0b0000001010101110,
            p_RXCKCAL1_IQ_LOOP_RST_CFG     = 0b0000000000000100,
            p_RXCKCAL1_I_LOOP_RST_CFG      = 0b0000000000000100,
            p_RXCKCAL1_Q_LOOP_RST_CFG      = 0b0000000000000100,
            p_RXCKCAL2_DX_LOOP_RST_CFG     = 0b0000000000000100,
            p_RXCKCAL2_D_LOOP_RST_CFG      = 0b0000000000000100,
            p_RXCKCAL2_S_LOOP_RST_CFG      = 0b0000000000000100,
            p_RXCKCAL2_X_LOOP_RST_CFG      = 0b0000000000000100,
            p_RXDFELPMRESET_TIME           = 0b0001111,
            p_RXDFELPM_KL_CFG0             = 0b0000000000000000,
            p_RXDFELPM_KL_CFG1             = 0b1010000011100010,
            p_RXDFELPM_KL_CFG2             = 0b0000000100000000,
            p_RXDFE_CFG0                   = 0b0000101000000000,
            p_RXDFE_CFG1                   = 0b0000000000000000,
            p_RXDFE_GC_CFG0                = 0b0000000000000000,
            p_RXDFE_GC_CFG1                = 0b1000000000000000,
            p_RXDFE_GC_CFG2                = 0b1111111111100000,
            p_RXDFE_H2_CFG0                = 0b0000000000000000,
            p_RXDFE_H2_CFG1                = 0b0000000000000010,
            p_RXDFE_H3_CFG0                = 0b0000000000000000,
            p_RXDFE_H3_CFG1                = 0b1000000000000010,
            p_RXDFE_H4_CFG0                = 0b0000000000000000,
            p_RXDFE_H4_CFG1                = 0b1000000000000010,
            p_RXDFE_H5_CFG0                = 0b0000000000000000,
            p_RXDFE_H5_CFG1                = 0b1000000000000010,
            p_RXDFE_H6_CFG0                = 0b0000000000000000,
            p_RXDFE_H6_CFG1                = 0b1000000000000010,
            p_RXDFE_H7_CFG0                = 0b0000000000000000,
            p_RXDFE_H7_CFG1                = 0b1000000000000010,
            p_RXDFE_H8_CFG0                = 0b0000000000000000,
            p_RXDFE_H8_CFG1                = 0b1000000000000010,
            p_RXDFE_H9_CFG0                = 0b0000000000000000,
            p_RXDFE_H9_CFG1                = 0b1000000000000010,
            p_RXDFE_HA_CFG0                = 0b0000000000000000,
            p_RXDFE_HA_CFG1                = 0b1000000000000010,
            p_RXDFE_HB_CFG0                = 0b0000000000000000,
            p_RXDFE_HB_CFG1                = 0b1000000000000010,
            p_RXDFE_HC_CFG0                = 0b0000000000000000,
            p_RXDFE_HC_CFG1                = 0b1000000000000010,
            p_RXDFE_HD_CFG0                = 0b0000000000000000,
            p_RXDFE_HD_CFG1                = 0b1000000000000010,
            p_RXDFE_HE_CFG0                = 0b0000000000000000,
            p_RXDFE_HE_CFG1                = 0b1000000000000010,
            p_RXDFE_HF_CFG0                = 0b0000000000000000,
            p_RXDFE_HF_CFG1                = 0b1000000000000010,
            p_RXDFE_KH_CFG0                = 0b0000000000000000,
            p_RXDFE_KH_CFG1                = 0b1000000000000000,
            p_RXDFE_KH_CFG2                = 0b0010011000010011,
            p_RXDFE_KH_CFG3                = 0b0100000100011100,
            p_RXDFE_OS_CFG0                = 0b0000000000000000,
            p_RXDFE_OS_CFG1                = 0b1000000000000010,
            p_RXDFE_PWR_SAVING             = 0b0,
            p_RXDFE_UT_CFG0                = 0b0000000000000000,
            p_RXDFE_UT_CFG1                = 0b0000000000000011,
            p_RXDFE_UT_CFG2                = 0b0000000000000000,
            p_RXDFE_VP_CFG0                = 0b0000000000000000,
            p_RXDFE_VP_CFG1                = 0b1000000000110011,
            p_RXDLY_CFG                    = 0b0000000000010000,
            p_RXDLY_LCFG                   = 0b0000000000110000,
            p_RXELECIDLE_CFG               = "SIGCFG_4",
            p_RXGBOX_FIFO_INIT_RD_ADDR     = 4,
            p_RXGEARBOX_EN                 = "TRUE",
            p_RXISCANRESET_TIME            = 0b00001,
            p_RXLPM_CFG                    = 0b0000000000000000,
            p_RXLPM_GC_CFG                 = 0b1000000000000000,
            p_RXLPM_KH_CFG0                = 0b0000000000000000,
            p_RXLPM_KH_CFG1                = 0b0000000000000010,
            p_RXLPM_OS_CFG0                = 0b0000000000000000,
            p_RXLPM_OS_CFG1                = 0b1000000000000010,
            p_RXOOB_CFG                    = 0b000000110,
            p_RXOOB_CLK_CFG                = "PMA",
            p_RXOSCALRESET_TIME            = 0b00011,
            p_RXOUT_DIV                    = 1,
            p_RXPCSRESET_TIME              = 0b00011,
            p_RXPHBEACON_CFG               = 0b0000000000000000,
            p_RXPHDLY_CFG                  = 0b0010000001110000,
            p_RXPHSAMP_CFG                 = 0b0010000100000000,
            p_RXPHSLIP_CFG                 = 0b1001100100110011,
            p_RXPH_MONITOR_SEL             = 0b00000,
            p_RXPI_AUTO_BW_SEL_BYPASS      = 0b0,
            p_RXPI_CFG0                    = 0b0000000000000100,
            p_RXPI_CFG1                    = 0b0000000000000000,
            p_RXPI_LPM                     = 0b0,
            p_RXPI_SEL_LC                  = 0b00,
            p_RXPI_STARTCODE               = 0b00,
            p_RXPI_VREFSEL                 = 0b0,
            p_RXPMACLK_SEL                 = "DATA",
            p_RXPMARESET_TIME              = 0b00011,
            p_RXPRBS_ERR_LOOPBACK          = 0b0,
            p_RXPRBS_LINKACQ_CNT           = 15,
            p_RXREFCLKDIV2_SEL             = 0b0,
            p_RXSLIDE_AUTO_WAIT            = 7,
            p_RXSLIDE_MODE                 = "OFF",
            p_RXSYNC_MULTILANE             = 0b0,
            p_RXSYNC_OVRD                  = 0b0,
            p_RXSYNC_SKIP_DA               = 0b0,
            p_RX_AFE_CM_EN                 = 0b0,
            p_RX_BIAS_CFG0                 = 0b0001010101010100,
            p_RX_BUFFER_CFG                = 0b000000,
            p_RX_CAPFF_SARC_ENB            = 0b0,
            p_RX_CLK25_DIV                 = 10,
            p_RX_CLKMUX_EN                 = 0b1,
            p_RX_CLK_SLIP_OVRD             = 0b00000,
            p_RX_CM_BUF_CFG                = 0b1010,
            p_RX_CM_BUF_PD                 = 0b0,
            p_RX_CM_SEL                    = 3,
            p_RX_CM_TRIM                   = 10,
            p_RX_CTLE3_LPF                 = 0b11111111,
            p_RX_DATA_WIDTH                = 64,
            p_RX_DDI_SEL                   = 0b000000,
            p_RX_DEFER_RESET_BUF_EN        = "TRUE",
            p_RX_DEGEN_CTRL                = 0b011,
            p_RX_DFELPM_CFG0               = 6,
            p_RX_DFELPM_CFG1               = 0b1,
            p_RX_DFELPM_KLKH_AGC_STUP_EN   = 0b1,
            p_RX_DFE_AGC_CFG0              = 0b10,
            p_RX_DFE_AGC_CFG1              = 4,
            p_RX_DFE_KL_LPM_KH_CFG0        = 1,
            p_RX_DFE_KL_LPM_KH_CFG1        = 4,
            p_RX_DFE_KL_LPM_KL_CFG0        = 0b01,
            p_RX_DFE_KL_LPM_KL_CFG1        = 4,
            p_RX_DFE_LPM_HOLD_DURING_EIDLE = 0b0,
            p_RX_DISPERR_SEQ_MATCH         = "TRUE",
            p_RX_DIV2_MODE_B               = 0b0,
            p_RX_DIVRESET_TIME             = 0b00001,
            p_RX_EN_CTLE_RCAL_B            = 0b0,
            p_RX_EN_HI_LR                  = 0b1,
            p_RX_EXT_RL_CTRL               = 0b000000000,
            p_RX_EYESCAN_VS_CODE           = 0b0000000,
            p_RX_EYESCAN_VS_NEG_DIR        = 0b0,
            p_RX_EYESCAN_VS_RANGE          = 0b00,
            p_RX_EYESCAN_VS_UT_SIGN        = 0b0,
            p_RX_FABINT_USRCLK_FLOP        = 0b0,
            p_RX_INT_DATAWIDTH             = 1,
            p_RX_PMA_POWER_SAVE            = 0b0,
            p_RX_PMA_RSV0                  = 0b0000000000000000,
            p_RX_PROGDIV_CFG               = 16.5,
            p_RX_PROGDIV_RATE              = 0b0000000000000001,
            p_RX_RESLOAD_CTRL              = 0b0000,
            p_RX_RESLOAD_OVRD              = 0b0,
            p_RX_SAMPLE_PERIOD             = 0b111,
            p_RX_SIG_VALID_DLY             = 11,
            p_RX_SUM_DFETAPREP_EN          = 0b0,
            p_RX_SUM_IREF_TUNE             = 0b1001,
            p_RX_SUM_RESLOAD_CTRL          = 0b0011,
            p_RX_SUM_VCMTUNE               = 0b1010,
            p_RX_SUM_VCM_OVWR              = 0b0,
            p_RX_SUM_VREF_TUNE             = 0b100,
            p_RX_TUNE_AFE_OS               = 0b00,
            p_RX_VREG_CTRL                 = 0b101,
            p_RX_VREG_PDB                  = 0b1,
            p_RX_WIDEMODE_CDR              = 0b01,
            p_RX_WIDEMODE_CDR_GEN3         = 0b00,
            p_RX_WIDEMODE_CDR_GEN4         = 0b01,
            p_RX_XCLK_SEL                  = "RXDES",
            p_RX_XMODE_SEL                 = 0b0,
            p_SAMPLE_CLK_PHASE             = 0b0,
            p_SAS_12G_MODE                 = 0b0,
            p_SATA_BURST_SEQ_LEN           = 0b1111,
            p_SATA_BURST_VAL               = 0b100,
            p_SATA_CPLL_CFG                = "VCO_3000MHZ",
            p_SATA_EIDLE_VAL               = 0b100,
            p_SHOW_REALIGN_COMMA           = "TRUE",
            p_SIM_DEVICE                   = "ULTRASCALE_PLUS",
            p_SIM_MODE                     = "FAST",
            p_SIM_RECEIVER_DETECT_PASS     = "TRUE",
            p_SIM_RESET_SPEEDUP            = "TRUE",
            p_SIM_TX_EIDLE_DRIVE_LEVEL     = "Z",
            p_SRSTMODE                     = 0b0,
            p_TAPDLY_SET_TX                = 0b00,
            p_TEMPERATURE_PAR              = 0b0010,
            p_TERM_RCAL_CFG                = 0b100001000010001,
            p_TERM_RCAL_OVRD               = 0b000,
            p_TRANS_TIME_RATE              = 0b00001110,
            p_TST_RSV0                     = 0b00000000,
            p_TST_RSV1                     = 0b00000000,
            p_TXBUF_EN                     = "FALSE",
            p_TXBUF_RESET_ON_RATE_CHANGE   = "TRUE",
            p_TXDLY_CFG                    = 0b1000000000010000,
            p_TXDLY_LCFG                   = 0b0000000000110000,
            p_TXDRVBIAS_N                  = 0b1010,
            p_TXFIFO_ADDR_CFG              = "LOW",
            p_TXGBOX_FIFO_INIT_RD_ADDR     = 4,
            p_TXGEARBOX_EN                 = "TRUE",
            p_TXOUT_DIV                    = 1,
            p_TXPCSRESET_TIME              = 0b00011,
            p_TXPHDLY_CFG0                 = 0b0110000001110000,
            p_TXPHDLY_CFG1                 = 0b0000000000001110,
            p_TXPH_CFG                     = 0b0000001100100011,
            p_TXPH_CFG2                    = 0b0000000000000000,
            p_TXPH_MONITOR_SEL             = 0b00000,
            p_TXPI_CFG                     = 0b0000000000000000,
            p_TXPI_CFG0                    = 0b00,
            p_TXPI_CFG1                    = 0b00,
            p_TXPI_CFG2                    = 0b00,
            p_TXPI_CFG3                    = 0b0,
            p_TXPI_CFG4                    = 0b0,
            p_TXPI_CFG5                    = 0b000,
            p_TXPI_GRAY_SEL                = 0b0,
            p_TXPI_INVSTROBE_SEL           = 0b0,
            p_TXPI_LPM                     = 0b0,
            p_TXPI_PPM                     = 0b0,
            p_TXPI_PPMCLK_SEL              = "TXUSRCLK2",
            p_TXPI_PPM_CFG                 = 0b00000000,
            p_TXPI_SYNFREQ_PPM             = 0b001,
            p_TXPI_VREFSEL                 = 0b0,
            p_TXPMARESET_TIME              = 0b00011,
            p_TXREFCLKDIV2_SEL             = 0b0,
            p_TXSYNC_MULTILANE             = 0b0,
            p_TXSYNC_OVRD                  = 0b0,
            p_TXSYNC_SKIP_DA               = 0b0,
            p_TX_CLK25_DIV                 = 10,
            p_TX_CLKMUX_EN                 = 0b1,
            p_TX_DATA_WIDTH                = 64,
            p_TX_DCC_LOOP_RST_CFG          = 0b0000000000000100,
            p_TX_DEEMPH0                   = 0b000000,
            p_TX_DEEMPH1                   = 0b000000,
            p_TX_DEEMPH2                   = 0b000000,
            p_TX_DEEMPH3                   = 0b000000,
            p_TX_DIVRESET_TIME             = 0b00001,
            p_TX_DRIVE_MODE                = "DIRECT",
            p_TX_DRVMUX_CTRL               = 2,
            p_TX_EIDLE_ASSERT_DELAY        = 0b100,
            p_TX_EIDLE_DEASSERT_DELAY      = 0b011,
            p_TX_FABINT_USRCLK_FLOP        = 0b0,
            p_TX_FIFO_BYP_EN               = 0b0,
            p_TX_IDLE_DATA_ZERO            = 0b0,
            p_TX_INT_DATAWIDTH             = 1,
            p_TX_LOOPBACK_DRIVE_HIZ        = "FALSE",
            p_TX_MAINCURSOR_SEL            = 0b0,
            p_TX_MARGIN_FULL_0             = 0b1011111,
            p_TX_MARGIN_FULL_1             = 0b1011110,
            p_TX_MARGIN_FULL_2             = 0b1011100,
            p_TX_MARGIN_FULL_3             = 0b1011010,
            p_TX_MARGIN_FULL_4             = 0b1011000,
            p_TX_MARGIN_LOW_0              = 0b1000110,
            p_TX_MARGIN_LOW_1              = 0b1000101,
            p_TX_MARGIN_LOW_2              = 0b1000011,
            p_TX_MARGIN_LOW_3              = 0b1000010,
            p_TX_MARGIN_LOW_4              = 0b1000000,
            p_TX_PHICAL_CFG0               = 0b0000000000000000,
            p_TX_PHICAL_CFG1               = 0b0111111000000000,
            p_TX_PHICAL_CFG2               = 0b0000001000000001,
            p_TX_PI_BIASSET                = 3,
            p_TX_PI_IBIAS_MID              = 0b00,
            p_TX_PMADATA_OPT               = 0b0,
            p_TX_PMA_POWER_SAVE            = 0b0,
            p_TX_PMA_RSV0                  = 0b0000000000001000,
            p_TX_PREDRV_CTRL               = 2,
            p_TX_PROGCLK_SEL               = "PREPI",
            p_TX_PROGDIV_CFG               = 16.5,
            p_TX_PROGDIV_RATE              = 0b0000000000000001,
            p_TX_QPI_STATUS_EN             = 0b0,
            p_TX_RXDETECT_CFG              = 0b00000000110010,
            p_TX_RXDETECT_REF              = 4,
            p_TX_SAMPLE_PERIOD             = 0b111,
            p_TX_SARC_LPBK_ENB             = 0b0,
            p_TX_SW_MEAS                   = 0b00,
            p_TX_VREG_CTRL                 = 0b000,
            p_TX_VREG_PDB                  = 0b0,
            p_TX_VREG_VREFSEL              = 0b00,
            p_TX_XCLK_SEL                  = "TXOUT",
            p_USB_BOTH_BURST_IDLE          = 0b0,
            p_USB_BURSTMAX_U3WAKE          = 0b1111111,
            p_USB_BURSTMIN_U3WAKE          = 0b1100011,
            p_USB_CLK_COR_EQ_EN            = 0b0,
            p_USB_EXT_CNTL                 = 0b1,
            p_USB_IDLEMAX_POLLING          = 0b1010111011,
            p_USB_IDLEMIN_POLLING          = 0b0100101011,
            p_USB_LFPSPING_BURST           = 0b000000101,
            p_USB_LFPSPOLLING_BURST        = 0b000110001,
            p_USB_LFPSPOLLING_IDLE_MS      = 0b000000100,
            p_USB_LFPSU1EXIT_BURST         = 0b000011101,
            p_USB_LFPSU2LPEXIT_BURST_MS    = 0b001100011,
            p_USB_LFPSU3WAKE_BURST_MS      = 0b111110011,
            p_USB_LFPS_TPERIOD             = 0b0011,
            p_USB_LFPS_TPERIOD_ACCURATE    = 0b1,
            p_USB_MODE                     = 0b0,
            p_USB_PCIE_ERR_REP_DIS         = 0b0,
            p_USB_PING_SATA_MAX_INIT       = 21,
            p_USB_PING_SATA_MIN_INIT       = 12,
            p_USB_POLL_SATA_MAX_BURST      = 8,
            p_USB_POLL_SATA_MIN_BURST      = 4,
            p_USB_RAW_ELEC                 = 0b0,
            p_USB_RXIDLE_P0_CTRL           = 0b1,
            p_USB_TXIDLE_TUNE_ENABLE       = 0b1,
            p_USB_U1_SATA_MAX_WAKE         = 7,
            p_USB_U1_SATA_MIN_WAKE         = 4,
            p_USB_U2_SAS_MAX_COM           = 64,
            p_USB_U2_SAS_MIN_COM           = 36,
            p_USE_PCS_CLK_PHASE_SEL        = 0b0,
            p_Y_ALL_MODE                   = 0b0,
        )
        self.gth_params.update(
            # Reset modes.
            i_GTTXRESETSEL    = 0,
            i_GTRXRESETSEL    = 0,
            i_RESETOVRD       = 0,

            # DRP.
            i_DRPADDR         = drp_mux.addr,
            i_DRPCLK          = drp_mux.clk,
            i_DRPDI           = drp_mux.di,
            o_DRPDO           = drp_mux.do,
            i_DRPEN           = drp_mux.en,
            o_DRPRDY          = drp_mux.rdy,
            i_DRPWE           = drp_mux.we,

            # CPLL (unused, QPLL only).
            i_GTREFCLK0       = 0,
            i_GTREFCLK1       = 0,
            i_CPLLRESET       = 1,
            i_CPLLPD          = 1,
            o_CPLLLOCK        = Open(),
            i_CPLLLOCKEN      = 0,
            i_CPLLREFCLKSEL   = 0b001,
            i_TSTIN           = 0,
            i_CPLLFREQLOCK    = 0,
            i_CPLLLOCKDETCLK  = 0,

            # QPLL.
            i_QPLL0CLK        = pll.clk    if use_qpll0 else 0,
            i_QPLL0REFCLK     = pll.refclk if use_qpll0 else 0,
            i_QPLL1CLK        = pll.clk    if use_qpll1 else 0,
            i_QPLL1REFCLK     = pll.refclk if use_qpll1 else 0,
            i_QPLL0FREQLOCK   = 0,
            i_QPLL1FREQLOCK   = 0,

            # 8B10B (unused).
            i_RX8B10BEN       = 0,
            i_TX8B10BEN       = 0,

            # TX clock.
            i_TXRATE          = 0b000,
            o_TXOUTCLK        = self.txoutclk,
            i_TXSYSCLKSEL     = 0b10 if use_qpll0 else 0b11,
            i_TXPLLCLKSEL     = 0b11 if use_qpll0 else 0b10,
            i_TXOUTCLKSEL     = 0b101, # TXPROGDIVCLK.

            # TX Startup/Reset.
            i_GTTXRESET       = tx_init.gtXxreset,
            i_TXPMARESET      = 0,
            i_TXPCSRESET      = 0,
            o_TXRESETDONE     = tx_init.Xxresetdone,
            i_TXPHDLYRESET    = 0,
            i_TXDLYSRESET     = tx_init.Xxdlysreset,
            o_TXDLYSRESETDONE = tx_init.Xxdlysresetdone,
            i_TXPHALIGN       = 0,
            i_TXPHALIGNEN     = 0,
            i_TXPHINIT        = 0,
            o_TXPHINITDONE    = Open(),
            o_TXPHALIGNDONE   = tx_init.Xxphaligndone,
            i_TXUSERRDY       = tx_init.Xxuserrdy,
            i_TXPHDLYPD       = 0,
            i_TXPHOVRDEN      = 0,
            i_TXMAINCURSOR    = 0x50,
            i_TXSYNCMODE      = 1,

            # TX Buffer bypass.
            i_TXDLYBYPASS     = 0,
            i_TXDLYEN         = 0,

            # TX data.
            i_TXCTRL0         = 0,
            i_TXCTRL1         = 0,
            i_TXDATA          = txdata_int,
            i_TXHEADER        = txheader_int,
            i_TXSEQUENCE      = 0, # Async gearbox: no pause sequencing.
            i_TXUSRCLK        = txusrclk,
            i_TXUSRCLK2       = ClockSignal("tx"),

            # TX PRBS.
            i_TXPRBSSEL       = tx_prbs_config,
            i_TXPRBSFORCEERR  = 0,

            # TXPI.
            i_TXPIPPMEN       = 0,
            i_TXPIPPMOVRDEN   = 0,
            i_TXPIPPMPD       = 0,
            i_TXPIPPMSEL      = 1,
            i_TXPIPPMSTEPSIZE = 0b00000,
            i_TXPISOPD        = 0,

            # TX electrical.
            i_TXPD            = Cat(tx_init.gtXxpd, tx_init.gtXxpd),
            i_TXDIFFCTRL      = 0b11000,
            i_TXINHIBIT       = self.tx_inhibit,

            # Internal Loopback.
            i_LOOPBACK        = self.loopback,

            # RX Startup/Reset.
            i_GTRXRESET       = rx_init.gtXxreset,
            i_RXPMARESET      = 0,
            i_RXPCSRESET      = 0,
            o_RXRESETDONE     = rx_init.Xxresetdone,
            i_RXDLYSRESET     = rx_init.Xxdlysreset,
            o_RXPHALIGNDONE   = rxphaligndone,
            i_RXSYNCALLIN     = rxphaligndone,
            i_RXUSERRDY       = rx_init.Xxuserrdy,
            i_RXSYNCIN        = 0,
            i_RXSYNCMODE      = 1,
            o_RXSYNCDONE      = rx_init.Xxsyncdone,
            i_RXDLYBYPASS     = 0,
            i_RXPHDLYPD       = 0,
            i_RXBUFRESET      = 0,
            i_RXDLYEN         = 0,
            o_GTPOWERGOOD     = Open(),

            # CDR.
            i_RXCDRFREQRESET  = 0,
            i_RXCDRHOLD       = 0,
            i_RXCDROVRDEN     = 0,
            i_RXCDRRESET      = 0,
            i_RXCHBONDEN      = 0,

            # RX AFE.
            i_RXDFEXYDEN      = 1,
            i_RXLPMEN         = 0, # DFE.

            # RX clock.
            i_RXRATE          = 0b000,
            i_RXSYSCLKSEL     = 0b10 if use_qpll0 else 0b11,
            i_RXOUTCLKSEL     = 0b101, # RXPROGDIVCLK.
            i_RXPLLCLKSEL     = 0b11 if use_qpll0 else 0b10,
            o_RXOUTCLK        = self.rxoutclk,
            i_RXUSRCLK        = rxusrclk,
            i_RXUSRCLK2       = ClockSignal("rx"),

            # RX Byte and Word Alignment (unused, sync header based).
            o_RXBYTEISALIGNED = Open(),
            o_RXBYTEREALIGN   = Open(),
            o_RXCOMMADET      = Open(),
            i_RXCOMMADETEN    = 0,
            i_RXMCOMMAALIGNEN = 0,
            i_RXPCOMMAALIGNEN = 0,
            i_RXSLIDE         = 0,

            # RX data.
            o_RXCTRL0         = Open(),
            o_RXCTRL1         = Open(),
            o_RXDATA          = rx_data,
            o_RXHEADER        = rx_header,
            i_RXGEARBOXSLIP   = rx_slip,

            # RX PRBS.
            i_RXPRBSSEL       = rx_prbs_config,
            i_RXPRBSCNTRESET  = rx_prbs_cnt_reset,
            o_RXPRBSERR       = rx_prbs_err,
            o_RXPRBSLOCKED    = rx_prbs_locked,

            # RX electrical.
            i_RXPD            = Cat(rx_init.gtXxpd, rx_init.gtXxpd),
            i_RXELECIDLEMODE  = 0b11,

            # Polarity.
            i_TXPOLARITY      = self.tx_polarity,
            i_RXPOLARITY      = self.rx_polarity,

            # Pads.
            i_GTHRXP          = rx_pads.p,
            i_GTHRXN          = rx_pads.n,
            o_GTHTXP          = tx_pads.p,
            o_GTHTXN          = tx_pads.n,
        )

        # TX clocking ------------------------------------------------------------------------------
        # TXUSRCLK: linerate/33 (PCS, no fabric logic); TXUSRCLK2: linerate/66 (fabric).
        tx_reset_deglitched = Signal()
        tx_reset_deglitched.attr.add("no_retiming")
        self.sync += tx_reset_deglitched.eq(~tx_init.done)
        self.cd_tx = ClockDomain()
        self.specials += [
            Instance("BUFG_GT", i_I=self.txoutclk, i_DIV=0b000, o_O=txusrclk),
            Instance("BUFG_GT", i_I=self.txoutclk, i_DIV=0b001, o_O=self.cd_tx.clk),
            AsyncResetSynchronizer(self.cd_tx, tx_reset_deglitched),
        ]

        # RX clocking ------------------------------------------------------------------------------
        # RXUSRCLK: linerate/33 (PCS, no fabric logic); RXUSRCLK2: linerate/66 (fabric).
        rx_reset_deglitched = Signal()
        rx_reset_deglitched.attr.add("no_retiming")
        self.sync += rx_reset_deglitched.eq(~rx_init.done)
        self.cd_rx = ClockDomain()
        self.specials += [
            Instance("BUFG_GT", i_I=self.rxoutclk, i_DIV=0b000, o_O=rxusrclk),
            Instance("BUFG_GT", i_I=self.rxoutclk, i_DIV=0b001, o_O=self.cd_rx.clk),
            AsyncResetSynchronizer(self.cd_rx, rx_reset_deglitched),
        ]

        # TX Datapath ------------------------------------------------------------------------------
        self.comb += [
            self.sink.ready.eq(1),
            If(self.sink.valid,
                txdata_int.eq(self.sink.data),
                txheader_int.eq(self.sink.header),
            ).Else(
                txdata_int.eq(tx_data),
                txheader_int.eq(tx_header),
            ),
        ]

        # RX Datapath ------------------------------------------------------------------------------
        self.comb += [
            self.source.valid.eq(1),
            self.source.data.eq(rx_data),
            self.source.header.eq(rx_header),
        ]

        # RX PRBS error counter (RXUSRCLK2 domain) -------------------------------------------------
        rx_prbs_errors_count = Signal(32, reset_less=True)
        self.sync.rx += [
            If(rx_prbs_cnt_reset,
                rx_prbs_errors_count.eq(0),
            ).Elif(rx_prbs_err & (rx_prbs_errors_count != (2**32-1)),
                rx_prbs_errors_count.eq(rx_prbs_errors_count + 1),
            ),
        ]
        self.comb += rx_prbs_errors.eq(rx_prbs_errors_count)

    def add_controls(self):
        self._tx_enable         = CSRStorage(reset=1, description="TX Enable.")
        self._tx_ready          = CSRStatus(description="TX Ready.")
        self._tx_inhibit        = CSRStorage(description="TX Inhibit.")
        self._rx_enable         = CSRStorage(reset=1, description="RX Enable.")
        self._rx_ready          = CSRStatus(description="RX Ready.")
        self._loopback          = CSRStorage(3, description="Loopback mode.")
        self._tx_prbs_config    = CSRStorage(4, description="TX Hardware PRBS config (PRBSSEL encoding).")
        self._rx_prbs_config    = CSRStorage(4, description="RX Hardware PRBS config (PRBSSEL encoding).")
        self._rx_prbs_cnt_reset = CSRStorage(description="RX PRBS error counter reset.")
        self._rx_prbs_errors    = CSRStatus(32, description="RX PRBS error counter.")
        self._rx_prbs_locked    = CSRStatus(description="RX PRBS checker locked.")
        self._tx_polarity       = CSRStorage(reset=self.tx_polarity.reset, description="TX polarity inversion.")
        self._rx_polarity       = CSRStorage(reset=self.rx_polarity.reset, description="RX polarity inversion.")
        self.comb += [
            self.tx_enable.eq(self._tx_enable.storage),
            self._tx_ready.status.eq(self.tx_ready),
            self.tx_inhibit.eq(self._tx_inhibit.storage),
            self.rx_enable.eq(self._rx_enable.storage),
            self._rx_ready.status.eq(self.rx_ready),
            self.loopback.eq(self._loopback.storage),
            self.tx_prbs_config.eq(self._tx_prbs_config.storage),
            self.rx_prbs_config.eq(self._rx_prbs_config.storage),
            self.rx_prbs_cnt_reset.eq(self._rx_prbs_cnt_reset.re),
            self._rx_prbs_errors.status.eq(self.rx_prbs_errors),
            self._rx_prbs_locked.status.eq(self.rx_prbs_locked),
            self.tx_polarity.eq(self._tx_polarity.storage),
            self.rx_polarity.eq(self._rx_polarity.storage),
        ]

    def do_finalize(self):
        self.specials += Instance("GTHE4_CHANNEL", **self.gth_params)
