from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.interconnect.csr import *
from litex.soc.cores.code_8b10b import Encoder, Decoder

from liteiclink.transceiver.gtx_7series_init import GTXInit
from liteiclink.transceiver.clock_aligner import BruteforceClockAligner

from liteiclink.transceiver.prbs import *


class GTXChannelPLL(Module):
    def __init__(self, refclk, refclk_freq, linerate):
        self.refclk = refclk
        self.reset = Signal()
        self.lock = Signal()
        self.config = self.compute_config(refclk_freq, linerate)

    @staticmethod
    def compute_config(refclk_freq, linerate):
        for n1 in 4, 5:
            for n2 in 1, 2, 3, 4, 5:
                for m in 1, 2:
                    vco_freq = refclk_freq*(n1*n2)/m
                    if 1.6e9 <= vco_freq <= 3.3e9:
                        for d in 1, 2, 4, 8, 16:
                            current_linerate = vco_freq*2/d
                            if current_linerate == linerate:
                                return {"n1": n1, "n2": n2, "m": m, "d": d,
                                        "vco_freq": vco_freq,
                                        "clkin": refclk_freq,
                                        "linerate": linerate}
        msg = "No config found for {:3.2f} MHz refclk / {:3.2f} Gbps linerate."
        raise ValueError(msg.format(refclk_freq/1e6, linerate/1e9))

    def __repr__(self):
        r = """
GTXChannelPLL
==============
  overview:
  ---------
       +--------------------------------------------------+
       |                                                  |
       |   +-----+  +---------------------------+ +-----+ |
       |   |     |  | Phase Frequency Detector  | |     | |
CLKIN +----> /M  +-->       Charge Pump         +-> VCO +---> CLKOUT
       |   |     |  |       Loop Filter         | |     | |
       |   +-----+  +---------------------------+ +--+--+ |
       |              ^                              |    |
       |              |    +-------+    +-------+    |    |
       |              +----+  /N2  <----+  /N1  <----+    |
       |                   +-------+    +-------+         |
       +--------------------------------------------------+
                            +-------+
                   CLKOUT +->  2/D  +-> LINERATE
                            +-------+
  config:
  -------
    CLKIN    = {clkin}MHz
    CLKOUT   = CLKIN x (N1 x N2) / M = {clkin}MHz x ({n1} x {n2}) / {m}
             = {vco_freq}GHz
    LINERATE = CLKOUT x 2 / D = {vco_freq}GHz x 2 / {d}
             = {linerate}GHz
""".format(clkin=self.config["clkin"]/1e6,
           n1=self.config["n1"],
           n2=self.config["n2"],
           m=self.config["m"],
           vco_freq=self.config["vco_freq"]/1e9,
           d=self.config["d"],
           linerate=self.config["linerate"]/1e9)
        return r


class GTXQuadPLL(Module):
    def __init__(self, refclk, refclk_freq, linerate):
        self.clk = Signal()
        self.refclk = Signal()
        self.reset = Signal()
        self.lock = Signal()
        self.config = self.compute_config(refclk_freq, linerate)

        # # #

        fbdiv_ratios = {
            16:  1,
            20:  1,
            32:  1,
            40:  1,
            64:  1,
            66:  0,
            80:  1,
            100: 1
        }
        fbdivs = {
            16:  0b0000100000,
            20:  0b0000110000,
            32:  0b0001100000,
            40:  0b0010000000,
            64:  0b0011100000,
            66:  0b0101000000,
            80:  0b0100100000,
            100: 0b0101110000
        }

        self.specials += \
            Instance("GTXE2_COMMON",
                p_QPLL_CFG=0x0680181 if self.config["vco_band"] == "upper" else
                           0x06801c1,
                p_QPLL_FBDIV=fbdivs[self.config["n"]],
                p_QPLL_FBDIV_RATIO=fbdiv_ratios[self.config["n"]],
                p_QPLL_REFCLK_DIV=self.config["m"],
                i_GTREFCLK0=refclk,
                i_QPLLRESET=self.reset,

                o_QPLLOUTCLK=self.clk,
                o_QPLLOUTREFCLK=self.refclk,
                i_QPLLLOCKEN=1,
                o_QPLLLOCK=self.lock,
                i_QPLLREFCLKSEL=0b001
            )

    @staticmethod
    def compute_config(refclk_freq, linerate):
        for n in 16, 20, 32, 40, 64, 66, 80, 100:
            for m in 1, 2, 3, 4:
                vco_freq = refclk_freq*n/m
                if 5.93e9 <= vco_freq <= 8e9:
                    vco_band = "lower"
                elif 9.8e9 <= vco_freq <= 12.5e9:
                    vco_band = "upper"
                else:
                    vco_band = None
                if vco_band is not None:
                    for d in [1, 2, 4, 8, 16]:
                        current_linerate = (vco_freq/2)*2/d
                        if current_linerate == linerate:
                            return {"n": n, "m": m, "d": d,
                                    "vco_freq": vco_freq,
                                    "vco_band": vco_band,
                                    "clkin": refclk_freq,
                                    "clkout": vco_freq/2,
                                    "linerate": linerate}
        msg = "No config found for {:3.2f} MHz refclk / {:3.2f} Gbps linerate."
        raise ValueError(msg.format(refclk_freq/1e6, linerate/1e9))

    def __repr__(self):
        r = """
GTXQuadPLL
===========
  overview:
  ---------
       +-------------------------------------------------------------++
       |                                          +------------+      |
       |   +-----+  +---------------------------+ | Upper Band | +--+ |
       |   |     |  | Phase Frequency Detector  +->    VCO     | |  | |
CLKIN +----> /M  +-->       Charge Pump         | +------------+->/2+--> CLKOUT
       |   |     |  |       Loop Filter         +-> Lower Band | |  | |
       |   +-----+  +---------------------------+ |    VCO     | +--+ |
       |              ^                           +-----+------+      |
       |              |        +-------+                |             |
       |              +--------+  /N   <----------------+             |
       |                       +-------+                              |
       +--------------------------------------------------------------+
                               +-------+
                      CLKOUT +->  2/D  +-> LINERATE
                               +-------+
  config:
  -------
    CLKIN    = {clkin}MHz
    CLKOUT   = CLKIN x N / (2 x M) = {clkin}MHz x {n} / (2 x {m})
             = {clkout}GHz
    VCO      = {vco_freq}GHz ({vco_band})
    LINERATE = CLKOUT x 2 / D = {clkout}GHz x 2 / {d}
             = {linerate}GHz
""".format(clkin=self.config["clkin"]/1e6,
           n=self.config["n"],
           m=self.config["m"],
           clkout=self.config["clkout"]/1e9,
           vco_freq=self.config["vco_freq"]/1e9,
           vco_band=self.config["vco_band"],
           d=self.config["d"],
           linerate=self.config["linerate"]/1e9)
        return r


class GTX(Module, AutoCSR):
    def __init__(self, pll, tx_pads, rx_pads, sys_clk_freq,
                 clock_aligner=True, internal_loopback=False,
                 tx_polarity=0, rx_polarity=0):
        self.tx_produce_square_wave = CSRStorage()
        self.tx_prbs_config = CSRStorage(2)

        self.rx_prbs_config = CSRStorage(2)
        self.rx_prbs_errors = CSRStatus(32)

        # # #

        self.submodules.encoder = ClockDomainsRenamer("tx")(
            Encoder(2, True))
        self.decoders = [ClockDomainsRenamer("rx")(
            Decoder(True)) for _ in range(2)]
        self.submodules += self.decoders

        self.rx_ready = Signal()

        # transceiver direct clock outputs
        # useful to specify clock constraints in a way palatable to Vivado
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        self.tx_clk_freq = pll.config["linerate"]/20

        # control/status cdc
        tx_produce_square_wave = Signal()
        tx_prbs_config = Signal(2)

        rx_prbs_config = Signal(2)
        rx_prbs_errors = Signal(32)


        self.specials += [
            MultiReg(self.tx_produce_square_wave.storage, tx_produce_square_wave, "tx"),
            MultiReg(self.tx_prbs_config.storage, tx_prbs_config, "tx"),
        ]

        self.specials += [
            MultiReg(self.rx_prbs_config.storage, rx_prbs_config, "rx"),
            MultiReg(rx_prbs_errors, self.rx_prbs_errors.status, "sys"), # FIXME
        ]

        # # #

        use_cpll = isinstance(pll, GTXChannelPLL)
        use_qpll = isinstance(pll, GTXQuadPLL)

        # TX generates TX clock, init must be in system domain
        tx_init = GTXInit(sys_clk_freq, False)
        # RX receives restart commands from TX domain
        rx_init = ClockDomainsRenamer("tx")(
            GTXInit(self.tx_clk_freq, True))
        self.submodules += tx_init, rx_init
        self.comb += [
            tx_init.plllock.eq(pll.lock),
            rx_init.plllock.eq(pll.lock),
            pll.reset.eq(tx_init.pllreset)
        ]

        rxcdr_cfgs = {
            1 : 0x03000023ff10400020,
            2 : 0x03000023ff10200020,
            4 : 0x03000023ff10100020,
            8 : 0x03000023ff10080020
        }

        txdata = Signal(20)
        rxdata = Signal(20)
        self.specials += \
            Instance("GTXE2_CHANNEL",
                # PMA Attributes
                p_PMA_RSV=0x00018480,
                p_PMA_RSV2=0x2050,
                p_PMA_RSV3=0,
                p_PMA_RSV4=0,
                p_RX_BIAS_CFG=0b100,
                p_RX_CM_TRIM=0b010,
                p_RX_OS_CFG=0b10000000,
                p_RX_CLK25_DIV=5,
                p_TX_CLK25_DIV=5,

                # Power-Down Attributes
                p_PD_TRANS_TIME_FROM_P2=0x3c,
                p_PD_TRANS_TIME_NONE_P2=0x3c,
                p_PD_TRANS_TIME_TO_P2=0x64,

                # CPLL
                p_CPLL_CFG=0xBC07DC,
                p_CPLL_FBDIV=1 if use_qpll else pll.config["n2"],
                p_CPLL_FBDIV_45=4 if use_qpll else pll.config["n1"],
                p_CPLL_REFCLK_DIV=1 if use_qpll else pll.config["m"],
                p_RXOUT_DIV=pll.config["d"],
                p_TXOUT_DIV=pll.config["d"],
                i_CPLLRESET=0 if use_qpll else pll.reset,
                o_CPLLLOCK=Signal() if use_qpll else pll.lock,
                i_CPLLLOCKEN=1,
                i_CPLLREFCLKSEL=0b001,
                i_TSTIN=2**20-1,
                i_GTREFCLK0=0 if use_qpll else pll.refclk,

                # QPLL
                i_QPLLCLK=0 if use_cpll else pll.clk,
                i_QPLLREFCLK=0 if use_cpll else pll.refclk,

                # TX clock
                p_TXBUF_EN="FALSE",
                p_TX_XCLK_SEL="TXUSR",
                o_TXOUTCLK=self.txoutclk,
                i_TXSYSCLKSEL=0b11 if use_qpll else 0b00,
                i_TXOUTCLKSEL=0b11,

                # TX Startup/Reset
                i_GTTXRESET=tx_init.gtXxreset,
                o_TXRESETDONE=tx_init.Xxresetdone,
                i_TXDLYSRESET=tx_init.Xxdlysreset,
                o_TXDLYSRESETDONE=tx_init.Xxdlysresetdone,
                o_TXPHALIGNDONE=tx_init.Xxphaligndone,
                i_TXUSERRDY=tx_init.Xxuserrdy,

                # TX data
                p_TX_DATA_WIDTH=20,
                p_TX_INT_DATAWIDTH=0,
                i_TXCHARDISPMODE=Cat(txdata[9], txdata[19]),
                i_TXCHARDISPVAL=Cat(txdata[8], txdata[18]),
                i_TXDATA=Cat(txdata[:8], txdata[10:18]),
                i_TXUSRCLK=ClockSignal("tx"),
                i_TXUSRCLK2=ClockSignal("tx"),

                # TX electrical
                i_TXBUFDIFFCTRL=0b100,
                i_TXDIFFCTRL=0b1000,

                # Internal Loopback
                i_LOOPBACK=0b010 if internal_loopback else 0b000,

                # RX Startup/Reset
                i_GTRXRESET=rx_init.gtXxreset,
                o_RXRESETDONE=rx_init.Xxresetdone,
                i_RXDLYSRESET=rx_init.Xxdlysreset,
                o_RXDLYSRESETDONE=rx_init.Xxdlysresetdone,
                o_RXPHALIGNDONE=rx_init.Xxphaligndone,
                i_RXUSERRDY=rx_init.Xxuserrdy,

                # RX AFE
                p_RX_DFE_XYD_CFG=0,
                # tests results @ 1.25Gbps:
                # (1) SFP 10GbE optical loopback
                # (2) SFP copper loopback
                # Xilinx's default value: 0x3008e56a
                #p_RX_DFE_KL_CFG2=0x3008e56a, # (1): errors+++, (2): errors+
                p_RX_DFE_KL_CFG2=0x3310180c, # (1): working, (2): not working
                #p_RX_DFE_KL_CFG2=0x3010d90c, # (1): errors+, (2): not working
                #p_RX_DFE_KL_CFG2=0x301148ac, # (1): errors+, (2): not working
                i_RXDFEXYDEN=1,
                i_RXDFEXYDHOLD=0,
                i_RXDFEXYDOVRDEN=0,
                i_RXLPMEN=0,

                # RX clock
                p_RXBUF_EN="FALSE",
                p_RX_XCLK_SEL="RXUSR",
                i_RXDDIEN=1,
                i_RXSYSCLKSEL=0b00,
                i_RXOUTCLKSEL=0b010,
                o_RXOUTCLK=self.rxoutclk,
                i_RXUSRCLK=ClockSignal("rx"),
                i_RXUSRCLK2=ClockSignal("rx"),
                p_RXCDR_CFG=rxcdr_cfgs[pll.config["d"]],

                # RX Clock Correction Attributes
                p_CLK_CORRECT_USE="FALSE",
                p_CLK_COR_SEQ_1_1=0b0100000000,
                p_CLK_COR_SEQ_2_1=0b0100000000,
                p_CLK_COR_SEQ_1_ENABLE=0b1111,
                p_CLK_COR_SEQ_2_ENABLE=0b1111,

                # RX data
                p_RX_DATA_WIDTH=20,
                p_RX_INT_DATAWIDTH=0,
                o_RXDISPERR=Cat(rxdata[9], rxdata[19]),
                o_RXCHARISK=Cat(rxdata[8], rxdata[18]),
                o_RXDATA=Cat(rxdata[:8], rxdata[10:18]),

                # Polarity
                i_TXPOLARITY=tx_polarity,
                i_RXPOLARITY=rx_polarity,

                # Pads
                i_GTXRXP=rx_pads.p,
                i_GTXRXN=rx_pads.n,
                o_GTXTXP=tx_pads.p,
                o_GTXTXN=tx_pads.n,
            )

        # tx clocking
        tx_reset_deglitched = Signal()
        tx_reset_deglitched.attr.add("no_retiming")
        self.sync += tx_reset_deglitched.eq(~tx_init.done)
        self.clock_domains.cd_tx = ClockDomain()
        txoutclk_bufg = Signal()
        txoutclk_bufr = Signal()
        tx_bufr_div = pll.config["clkin"]/self.tx_clk_freq
        assert tx_bufr_div == int(tx_bufr_div)
        self.specials += [
            Instance("BUFG", i_I=self.txoutclk, o_O=txoutclk_bufg),
            # TODO: use MMCM instead?
            Instance("BUFR", i_I=txoutclk_bufg, o_O=txoutclk_bufr,
                i_CE=1, p_BUFR_DIVIDE=str(int(tx_bufr_div))),
            Instance("BUFG", i_I=txoutclk_bufr, o_O=self.cd_tx.clk),
            AsyncResetSynchronizer(self.cd_tx, tx_reset_deglitched)
        ]

        # rx clocking
        rx_reset_deglitched = Signal()
        rx_reset_deglitched.attr.add("no_retiming")
        self.sync.tx += rx_reset_deglitched.eq(~rx_init.done)
        self.clock_domains.cd_rx = ClockDomain()
        self.specials += [
            Instance("BUFG", i_I=self.rxoutclk, o_O=self.cd_rx.clk),
            AsyncResetSynchronizer(self.cd_rx, rx_reset_deglitched)
        ]

        # tx data and prbs
        self.submodules.tx_prbs = ClockDomainsRenamer("tx")(PRBSTX(20, True))
        self.comb += self.tx_prbs.config.eq(tx_prbs_config)
        self.comb += [
            self.tx_prbs.i.eq(Cat(*[self.encoder.output[i] for i in range(2)])),
            If(tx_produce_square_wave,
                # square wave @ linerate/20 for scope observation
                txdata.eq(0b11111111110000000000)
            ).Else(
                txdata.eq(self.tx_prbs.o)
            )
        ]

        # rx data and prbs
        self.submodules.rx_prbs = ClockDomainsRenamer("rx")(PRBSRX(20, True))
        self.comb += [
            self.rx_prbs.config.eq(rx_prbs_config),
            rx_prbs_errors.eq(self.rx_prbs.errors)
        ]
        self.comb += [
            self.decoders[0].input.eq(rxdata[:10]),
            self.decoders[1].input.eq(rxdata[10:]),
            self.rx_prbs.i.eq(rxdata)
        ]

        # clock alignment
        if clock_aligner:
            clock_aligner = BruteforceClockAligner(0b0101111100, self.tx_clk_freq)
            self.submodules += clock_aligner
            self.comb += [
                clock_aligner.rxdata.eq(rxdata),
                rx_init.restart.eq(clock_aligner.restart),
                self.rx_ready.eq(clock_aligner.ready)
            ]
        else:
            self.comb += self.rx_ready.eq(rx_init.done)
