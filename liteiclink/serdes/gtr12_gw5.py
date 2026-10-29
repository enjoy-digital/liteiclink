#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST GTR12 transceiver (raw PCS, fabric 8b/10b), LiteICLink transceiver interface.

The GTR12 quad is configured by register writes generated from a TOML description with Gowin's
serdes_toml_to_csr tool (Gowin Customized PHY IP, IPUG1024): line rate, PLL/CDR, PMA width and
TX/RX buffers gear (fabric data width), comma alignment... The configuration is given by the
caller (csr_config) and loaded by the Gowin toolchain (set_csr). Registers can also be read/written
at runtime through the UPAR (DRP) port of the quad (GTR12UPAR).

Fabric side: data_width-bit raw words (8b/10b symbols, 10 bits each, first symbol on bits 0-9) on
the TX/RX PCS fabric clocks (line rate/data_width), 8b/10b encoding/decoding and polarity inversion
in the fabric.
"""

from migen import *
from migen.genlib.cdc       import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from litex.soc.cores.code_8b10b import Encoder, Decoder

# GTR12 8b/10b Decoder ----------------------------------------------------------------------------

class GTR12Decoder(LiteXModule):
    """8b/10b decoder (Decoder interface: input, d, k, invalid, 1 cycle of latency) with the lookups
    in logic and registered outputs: no block ROM (slow clock-to-output; GowinSynthesis infers block
    ROMs from the registered lookups and can merge them with other lookups on the same address into
    wrong contents: syn_romstyle "logic")."""
    def __init__(self, lsb_first=True):
        self.input   = Signal(10)
        self.d       = Signal(8, attr={("syn_romstyle", "logic")})
        self.k       = Signal(attr={("syn_romstyle", "logic")})
        self.invalid = Signal()

        # # #

        from litex.soc.cores.code_8b10b import table_6b5b, table_4b3b, table_4b3b_kn, table_4b3b_kp

        input_msb_first = Signal(10)
        if lsb_first:
            self.comb += [input_msb_first[i].eq(self.input[9-i]) for i in range(10)]
        else:
            self.comb += input_msb_first.eq(self.input)
        code6b = input_msb_first[4:]
        code4b = input_msb_first[:4]
        code5b = Signal(5)
        code3b = Signal(3)
        k      = Signal()
        self.comb += [
            Case(code6b, {i: code5b.eq(v) for i, v in enumerate(table_6b5b)}),
            k.eq(0),
            If(code6b == 0b001111,
                k.eq(1),
                Case(code4b, {i: code3b.eq(v) for i, v in enumerate(table_4b3b_kn)}),
            ).Elif(code6b == 0b110000,
                k.eq(1),
                Case(code4b, {i: code3b.eq(v) for i, v in enumerate(table_4b3b_kp)}),
            ).Else(
                If((code4b == 0b0111) | (code4b == 0b1000), # D.x.A7/K.x.7
                    If((code6b != 0b100011) &
                       (code6b != 0b010011) &
                       (code6b != 0b001011) &
                       (code6b != 0b110100) &
                       (code6b != 0b101100) &
                       (code6b != 0b011100), k.eq(1))
                ),
                Case(code4b, {i: code3b.eq(v) for i, v in enumerate(table_4b3b)}),
            ),
        ]
        # Basic invalid symbols detection (as Decoder): 4, 5 or 6 ones in the symbol.
        ones = Signal(4)
        self.comb += ones.eq(Reduce("ADD", [self.input[i] for i in range(10)]))
        self.sync += [
            self.d.eq(Cat(code5b, code3b)),
            self.k.eq(k),
            self.invalid.eq((ones != 4) & (ones != 5) & (ones != 6)),
        ]

# GTR12 Quad Helpers -------------------------------------------------------------------------------

def gtr12_quad_params(position="Q1"):
    """GTR12_QUAD parameters with all lanes and fabric controls tied off (the register
    configuration selects their internal controls), the caller overrides the lane it uses."""
    params = dict(p_POSITION=position)
    for name, width in [
        ("FABRIC_CLK_LIFE_DIV_I", 2), ("FABRIC_CM0_RXCLK_OE_L_I", 1), ("FABRIC_CM0_RXCLK_OE_R_I", 1),
        ("FABRIC_PMA_PD_REFHCLK_I", 1), ("FABRIC_REFCLK1_INPUT_SEL_I", 3), ("FABRIC_REFCLK_INPUT_SEL_I", 3),
        ("FABRIC_REFCLK_OE_L_I", 1), ("FABRIC_REFCLK_OE_R_I", 1), ("FABRIC_REFCLK_OUTPUT_SEL_I", 5),
        ("REFCLKM0_I", 1), ("REFCLKM1_I", 1), ("REFCLKP0_I", 1), ("REFCLKP1_I", 1),
        ("FABRIC_BURN_IN_I", 1), ("FABRIC_CK_SOC_DIV_I", 2), ("FABRIC_CLK_REF_CORE_I", 1),
        ("FABRIC_CMU1_REFCLK_GATE_I", 1), ("FABRIC_CMU_REFCLK_GATE_I", 1), ("FABRIC_GLUE_MAC_INIT_INFO_I", 1),
        ("FABRIC_REFCLK_GATE_I", 1), ("FABRIC_CMU0_RESETN_I", 1), ("FABRIC_CMU0_PD_I", 1),
        ("FABRIC_CMU0_IDDQ_I", 1), ("FABRIC_CMU1_RESETN_I", 1), ("FABRIC_CMU1_PD_I", 1),
        ("FABRIC_CMU1_IDDQ_I", 1), ("FABRIC_PLL_CDN_I", 1), ("FABRIC_CM1_PD_REFCLK_DET_I", 1),
        ("FABRIC_CM0_PD_REFCLK_DET_I", 1), ("FABRIC_POR_N_I", 1), ("FABRIC_QUAD_MCU_REQ_I", 1),
        ("CK_AHB_I", 1), ("AHB_RSTN", 1), ("TEST_DEC_EN", 1), ("QUAD_PCIE_CLK", 1),
        ("PCIE_DIV2_REG", 1), ("PCIE_DIV4_REG", 1), ("PMAC_LN_RSTN", 1)]:
        params[f"i_{name}"] = Constant(0, width)
    for n in range(4):
        for name, width in [
            ("LN{n}_RXM_I", 1), ("LN{n}_RXP_I", 1), ("FABRIC_LN{n}_CTRL_I", 43), ("FABRIC_LN{n}_IDDQ_I", 1),
            ("FABRIC_LN{n}_PD_I", 3), ("FABRIC_LN{n}_RATE_I", 2), ("FABRIC_LN{n}_RSTN_I", 1),
            ("FABRIC_LN{n}_TXDATA_I", 80), ("LANE{n}_PCS_RX_RST", 1), ("LANE{n}_ALIGN_TRIGGER", 1),
            ("LANE{n}_CHBOND_START", 1), ("LANE{n}_PCS_TX_RST", 1), ("LANE{n}_FABRIC_RX_CLK", 1),
            ("LANE{n}_FABRIC_C2I_CLK", 1), ("LANE{n}_FABRIC_TX_CLK", 1), ("LANE{n}_RX_IF_FIFO_RDEN", 1),
            ("FABRIC_LN{n}_CPLL_RESETN_I", 1), ("FABRIC_LN{n}_CPLL_PD_I", 1), ("FABRIC_LN{n}_CPLL_IDDQ_I", 1),
            ("FABRIC_LN{n}_CTRL_I_H", 43), ("FABRIC_LN{n}_PD_I_H", 3), ("FABRIC_LN{n}_RATE_I_H", 2),
            ("FABRIC_LN{n}_TX_VLD_IN", 1)]:
            params["i_" + name.format(n=n)] = Constant(0, width)
    return params

def gtr12_add_csr_config(platform, filename, csr_config):
    """Register configuration of the quad, written in the gateware directory and loaded by the
    Gowin toolchain (set_csr)."""
    platform.toolchain.additional_tcl_commands += [
        f'set serdes_csr [open "{filename}" w]',
        'puts -nonewline $serdes_csr {' + csr_config + '}',
        'close $serdes_csr',
        f'set_csr {filename}',
    ]

# GTR12 UPAR (Dynamic Register Access) -------------------------------------------------------------

class GTR12UPAR(LiteXModule):
    """GTR12 registers runtime access (UPAR/DRP, Gowin IPUG1024 Dynamic Reconfiguration): one access
    at a time, addr/wrdata/we/re with done/rddata (upar clock domain = clk). The user requests (ex:
    rate change sequencer) have priority over the CSR requests (add_csr), held until served."""
    def __init__(self, inet_upar_q, clk, rst=0, position="Q1"):
        self.test_dec_en = Signal() # o: To the quad TEST_DEC_EN (same net required).
        self.ahb_rstn    = Signal() # o: To the quad AHB_RSTN (same net required).
        self.addr   = Signal(24) # i
        self.wrdata = Signal(32) # i
        self.we     = Signal()   # i: Write request (when ready, addr/wrdata taken with it).
        self.re     = Signal()   # i: Read request (when ready, addr taken with it).
        self.ready  = Signal()   # o: Idle.
        self.done   = Signal()   # o: Access done (pulse, user requests).
        self.rddata = Signal(32) # o
        self.abort  = Signal()   # i: Abort the access in progress.

        # # #

        # CSR requests (add_csr), served when no user request.
        self.csr_addr   = Signal(24)
        self.csr_wrdata = Signal(32)
        self.csr_we     = Signal()
        self.csr_re     = Signal()
        self.csr_done   = Signal()

        wren       = Signal()
        rden       = Signal()
        rdvld      = Signal()
        upar_ready = Signal()
        self.upar_ready_s = upar_ready
        self.upar_rdvld_s = rdvld
        rddata = Signal(32)
        addr   = Signal(24)
        wrdata = Signal(32)
        is_csr = Signal()
        self.specials += Instance("GTR12_UPAR",
            i_UPAR_CLK        = clk,
            i_UPAR_RST        = rst,
            i_UPAR_WREN_S     = wren,
            i_UPAR_ADDR_S     = addr,
            i_UPAR_WRDATA_S   = wrdata,
            i_UPAR_RDEN_S     = rden,
            i_UPAR_STRB_S     = Constant(0xff, 8),
            i_UPAR_BUS_WIDTH_S = 0,
            # Registers access through UPAR (CSR_MODE as in the Gowin SerDes IP generated wrappers,
            # else the accesses do not complete).
            i_CSR_MODE        = 0b10100,
            o_UPAR_RDDATA_S   = rddata,
            o_UPAR_RDVLD_S    = rdvld,
            o_UPAR_READY_S    = upar_ready,
            o_QUAD_CFG_TEST_DEC_EN = self.test_dec_en,
            o_AHB_RSTN_O      = self.ahb_rstn,
            io_INET_UPAR_Q1   = inet_upar_q if position == "Q1" else Signal(421),
            io_INET_UPAR_Q0   = inet_upar_q if position == "Q0" else Signal(421),
        )
        # Write: wren held until ready. Read: rden held until rdvld.
        done = Signal()
        self.comb += [
            self.done.eq(done & ~is_csr),
            self.csr_done.eq(done & is_csr),
        ]
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.ready.eq(1),
            If(self.we | self.re,
                NextValue(addr,   self.addr),
                NextValue(wrdata, self.wrdata),
                NextValue(is_csr, 0),
                If(self.we, NextState("WRITE")).Else(NextState("READ"))
            ).Elif(self.csr_we | self.csr_re,
                NextValue(addr,   self.csr_addr),
                NextValue(wrdata, self.csr_wrdata),
                NextValue(is_csr, 1),
                If(self.csr_we, NextState("WRITE")).Else(NextState("READ"))
            )
        )
        fsm.act("WRITE",
            wren.eq(1),
            If(upar_ready | self.abort, done.eq(1), NextState("IDLE"))
        )
        fsm.act("READ",
            rden.eq(1),
            If(rdvld | self.abort,
                done.eq(1),
                NextValue(self.rddata, rddata),
                NextState("IDLE")
            )
        )

    def add_csr(self):
        """CSR access (sys clock domain = upar clock domain)."""
        self._addr   = CSRStorage(24, description="Register address.")
        self._wrdata = CSRStorage(32, description="Register write data.")
        self._ctrl   = CSR(2)
        self._status = CSRStatus(fields=[
            CSRField("ready",      size=1, description="No CSR access pending/in progress."),
            CSRField("upar_ready", size=1, description="UPAR_READY_S."),
            CSRField("upar_rdvld", size=1, description="UPAR_RDVLD_S."),
        ])
        self._cfg    = CSRStorage(fields=[
            CSRField("abort", size=1, description="Abort the access in progress (FSM back to idle)."),
        ])
        self._rddata = CSRStatus(32, description="Register read data.")
        pending_we = Signal()
        pending_re = Signal()
        busy       = Signal()
        self.sync += [
            If(self._ctrl.re,
                pending_we.eq(self._ctrl.r[0]),
                pending_re.eq(self._ctrl.r[1]),
                busy.eq(self._ctrl.r != 0),
            ).Elif(self._cfg.fields.abort | (self.ready & ~(self.we | self.re)),
                # Granted (no user request) or aborted.
                pending_we.eq(0),
                pending_re.eq(0),
            ),
            If(self.csr_done | self._cfg.fields.abort, busy.eq(0)),
        ]
        self.comb += [
            self.csr_addr.eq(self._addr.storage),
            self.csr_wrdata.eq(self._wrdata.storage),
            self.csr_we.eq(pending_we),
            self.csr_re.eq(pending_re),
            self._status.fields.ready.eq(~busy),
            self._status.fields.upar_ready.eq(self.upar_ready_s),
            self._status.fields.upar_rdvld.eq(self.upar_rdvld_s),
            self.abort.eq(self._cfg.fields.abort),
            self._rddata.status.eq(self.rddata),
        ]

# GTR12 Transceiver --------------------------------------------------------------------------------

class GTR12(LiteXModule):
    """GTR12 lane with a raw PCS (data_width-bit fabric words) and fabric 8b/10b.

    LiteICLink transceiver interface: tx_enable/tx_ready, rx_enable/rx_ready, encoder/decoders,
    tx_produce_pattern/tx_pattern, cd_tx/cd_rx, add_stream_endpoints(). rx_align enables the PCS
    comma alignment (when configured). Lane status (pll_lock, cdr_lock, aligned, rx_elecidle,
    astat, stat) for the users/bring-up.
    """
    def __init__(self, platform, line_rate, csr_config, data_width=40, position="Q1", lane=0, pll="QPLL1",
        tx_polarity=0, rx_polarity=0, csr_name="gtr12.csr", with_upar=True, upar_clk=None):
        assert data_width in [20, 40, 80]
        assert pll in ["QPLL0", "QPLL1", "CPLL"] # PLL used by the configuration (lock status).
        self.nwords     = nwords = data_width//10
        self.data_width = data_width
        self.line_rate  = line_rate

        self.tx_enable          = Signal(reset=1) # i
        self.tx_ready           = Signal()        # o
        self.tx_produce_pattern = Signal()        # i
        self.tx_pattern         = Signal(data_width)
        self.tx_raw_enable      = Signal()        # i: Transmit tx_raw (8b/10b bypassed).
        self.tx_raw             = Signal(data_width)

        self.rx_enable = Signal(reset=1) # i
        self.rx_ready  = Signal()        # o
        self.rx_align  = Signal(reset=1) # i
        self.rx_raw    = Signal(data_width) # o: Raw RX words (rx clock domain).
        self.rx_valid  = Signal()        # o: RX words valid (rx clock domain).

        # Lane status (raw, quad clock domains).
        self.pll_lock    = Signal()
        self.cdr_lock    = Signal()
        self.aligned     = Signal()
        self.k_lock      = Signal()
        self.rx_elecidle = Signal()
        self.astat       = Signal(6)
        self.stat        = Signal(13)
        self.refclk_det  = Signal(2) # Quad reference clocks detected (CM0/CM1).
        self.plls_lock   = Signal(3) # QPLL0 (CMU), QPLL1 (CMU1), lane CPLL locks.

        self.tx_clk_freq = self.rx_clk_freq = line_rate/data_width

        # # #

        # Clocks -----------------------------------------------------------------------------------
        # The PCS fabric clocks outputs drive the clock domains clocks directly (no alias: kept
        # names for the timing constraints, GowinSynthesis otherwise merges the aliases).
        self.cd_tx = ClockDomain()
        self.cd_rx = ClockDomain()
        tx_clk = self.cd_tx.clk
        rx_clk = self.cd_rx.clk
        tx_clk.attr.add(("syn_keep", 1))
        rx_clk.attr.add(("syn_keep", 1))

        # 8b/10b -----------------------------------------------------------------------------------
        self.encoder  = ClockDomainsRenamer("tx")(Encoder(nwords, True))
        self.decoders = [ClockDomainsRenamer("rx")(GTR12Decoder(True)) for _ in range(nwords)]
        self.submodules += self.decoders

        # Init / Resets ----------------------------------------------------------------------------
        self.comb += self.pll_lock.eq(self.plls_lock[{"QPLL0": 0, "QPLL1": 1, "CPLL": 2}[pll]])
        # Lane reset (PMA) while not enabled, then PCS TX/RX resets until the PLL is locked (the CDR
        # lock depends on the received signal: cdr_lock status).
        lane_rstn  = Signal()
        pll_lock   = Signal()
        cdr_lock   = Signal()
        self.specials += [
            MultiReg(self.pll_lock, pll_lock),
            MultiReg(self.cdr_lock, cdr_lock),
        ]
        self.comb += [
            lane_rstn.eq(self.tx_enable | self.rx_enable),
            self.tx_ready.eq(self.tx_enable & pll_lock),
            self.rx_ready.eq(self.rx_enable & pll_lock),
        ]
        pcs_tx_rst = Signal()
        pcs_rx_rst = Signal()
        self.comb += [
            pcs_tx_rst.eq(~self.tx_ready),
            pcs_rx_rst.eq(~self.rx_ready),
        ]
        self.specials += [
            AsyncResetSynchronizer(self.cd_tx, pcs_tx_rst),
            AsyncResetSynchronizer(self.cd_rx, pcs_rx_rst),
        ]

        # TX Data ----------------------------------------------------------------------------------
        tx_polarity_tx        = Signal()
        tx_produce_pattern_tx = Signal()
        tx_pattern_tx         = Signal(data_width)
        self.specials += [
            MultiReg(tx_polarity,             tx_polarity_tx,        "tx"),
            MultiReg(self.tx_produce_pattern, tx_produce_pattern_tx, "tx"),
            MultiReg(self.tx_pattern,         tx_pattern_tx,         "tx"),
        ]
        txdata = Signal(data_width)
        self.sync.tx += [
            If(self.tx_raw_enable,
                txdata.eq(self.tx_raw)
            ).Elif(tx_produce_pattern_tx,
                txdata.eq(tx_pattern_tx)
            ).Else(
                txdata.eq(Cat(*[self.encoder.output[i] for i in range(nwords)]))
            )
        ]
        txdata_pol = Signal(data_width)
        self.comb += txdata_pol.eq(txdata ^ Replicate(tx_polarity_tx, data_width))

        # RX Data ----------------------------------------------------------------------------------
        rx_polarity_rx = Signal()
        self.specials += MultiReg(rx_polarity, rx_polarity_rx, "rx")
        rxdata     = Signal(88)
        rx_valid   = Signal()
        rx_empty   = Signal()
        rxdata_r   = Signal(data_width)
        self.sync.rx += [
            self.rx_valid.eq(rx_valid),
            rxdata_r.eq(rxdata[:data_width] ^ Replicate(rx_polarity_rx, data_width)),
        ]
        self.comb += self.rx_raw.eq(rxdata_r)
        for i in range(nwords):
            self.comb += self.decoders[i].input.eq(rxdata_r[10*i:10*(i+1)])

        # GTR12 Quad -------------------------------------------------------------------------------
        self.inet_upar_q = inet_upar_q = Signal(421)
        rx_align = Signal()
        self.specials += MultiReg(self.rx_align, rx_align, "rx")
        n = lane
        self.gtr12_params = params = gtr12_quad_params(position)
        params.update({
            f"i_FABRIC_LN{n}_RSTN_I"         : lane_rstn,
            f"i_LANE{n}_PCS_TX_RST"          : pcs_tx_rst,
            f"i_LANE{n}_PCS_RX_RST"          : pcs_rx_rst,
            f"i_LANE{n}_FABRIC_TX_CLK"       : tx_clk,
            f"i_LANE{n}_FABRIC_RX_CLK"       : rx_clk,
            f"i_FABRIC_LN{n}_TXDATA_I"       : Cat(txdata_pol, Constant(0, 80 - data_width)),
            f"i_FABRIC_LN{n}_TX_VLD_IN"      : 1,
            f"i_LANE{n}_RX_IF_FIFO_RDEN"     : ~rx_empty,
            f"i_LANE{n}_ALIGN_TRIGGER"       : rx_align,
            f"o_LANE{n}_PCS_TX_O_FABRIC_CLK" : tx_clk,
            f"o_LANE{n}_PCS_RX_O_FABRIC_CLK" : rx_clk,
            f"o_FABRIC_LN{n}_RXDATA_O"       : rxdata,
            f"o_FABRIC_LN{n}_RX_VLD_OUT"     : rx_valid,
            f"o_LANE{n}_RX_IF_FIFO_EMPTY"    : rx_empty,
            f"o_FABRIC_LANE{n}_CMU_OK_O"     : self.plls_lock[2],
            "o_FABRIC_CMU_OK_O"              : self.plls_lock[0],
            "o_FABRIC_CMU1_OK_O"             : self.plls_lock[1],
            f"o_FABRIC_LN{n}_PMA_RX_LOCK_O"  : self.cdr_lock,
            f"o_LANE{n}_ALIGN_LINK"          : self.aligned,
            f"o_LANE{n}_K_LOCK"              : self.k_lock,
            f"o_FABRIC_LN{n}_RXELECIDLE_O"   : self.rx_elecidle,
            f"o_FABRIC_LN{n}_ASTAT_O"        : self.astat,
            f"o_FABRIC_LN{n}_STAT_O"         : self.stat,
            "o_FABRIC_PMA_CM0_DR_REFCLK_DET_O" : self.refclk_det[0],
            "o_FABRIC_PMA_CM1_DR_REFCLK_DET_O" : self.refclk_det[1],
            "io_INET_Q_UPAR"                 : inet_upar_q,
        })
        gtr12_add_csr_config(platform, csr_name, csr_config)

        # UPAR -------------------------------------------------------------------------------------
        if with_upar:
            upar_clk = ClockSignal("sys") if upar_clk is None else upar_clk
            self.upar = GTR12UPAR(inet_upar_q, clk=upar_clk, position=position)
            # The quad AHB clock/reset and test decoder enable come from the UPAR (same nets).
            params["i_CK_AHB_I"]  = upar_clk
            params["i_AHB_RSTN"]  = self.upar.ahb_rstn
            params["i_TEST_DEC_EN"] = self.upar.test_dec_en

    def do_finalize(self):
        self.specials += Instance("GTR12_QUAD", **self.gtr12_params)

    def add_stream_endpoints(self):
        self.sink   = sink   = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.source = source = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.comb += [
            sink.ready.eq(1),
            source.valid.eq(1),
        ]
        for i in range(self.nwords):
            self.comb += [
                self.encoder.k[i].eq(sink.ctrl[i]),
                self.encoder.d[i].eq(sink.data[8*i:8*(i+1)]),
                source.ctrl[i].eq(self.decoders[i].k),
                source.data[8*i:8*(i+1)].eq(self.decoders[i].d),
            ]

    def add_status_csr(self):
        """Lane status (bring-up): pll_lock, cdr_lock, aligned, k_lock, rx_elecidle, astat, stat."""
        status = Cat(self.pll_lock, self.cdr_lock, self.aligned, self.k_lock, self.rx_elecidle,
            self.astat, self.stat, self.refclk_det, self.plls_lock)
        self._status = CSRStatus(len(status), description="pll_lock[0], cdr_lock[1], aligned[2], "
            "k_lock[3], rx_elecidle[4], astat[10:5], stat[23:11], refclk_det[25:24], "
            "plls_lock[28:26] (QPLL0, QPLL1, CPLL).")
        self.specials += MultiReg(status, self._status.status)
