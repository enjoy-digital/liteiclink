#
# This file is part of LiteICLink project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register appendix of the manual: which designs tools/csr_doc.py elaborates and which register banks
# it documents (the engine is generic, part of the manual kit).

import json
import os
import sys
import tempfile

# Configurations: "manual" and the name of a function of this file building the SoC. LiteICLink has
# no generator and its bench targets leave some registers out (no clock aligner, no DRP access, no
# SerWB PRBS or CRC), so every bank comes from a small SoC defined below, elaborated only for this
# appendix on a LiteX-Boards platform: one per transceiver family, and the SerWB PHYs and core with
# every register option enabled. These SoCs are not designs to build: their clocks and pads are
# placeholders.
CONFIGS = {
    "gtp"       : ("manual", "soc_gtp"),
    "gtx_serwb" : ("manual", "soc_gtx_serwb"),
    "gth3"      : ("manual", "soc_gth3"),
    "gty"       : ("manual", "soc_gty"),
    "ecp5"      : ("manual", "soc_ecp5"),
    "gtr12"     : ("manual", "soc_gtr12"),
    "serwb_gen" : ("manual", "soc_serwb_gen"),
}

# Banks documented in the appendix (prefix match on the bank name), in order, with their title and
# the Python class that defines them. The longest matching prefix owns a bank.
BANKS = [
    ("serdes_gtp",    "GTP (Artix-7)",            "GTP",           "liteiclink/serdes/gtp_7series.py"),
    ("serdes_gtx",    "GTX (Kintex-7, Virtex-7)", "GTX",           "liteiclink/serdes/gtx_7series.py"),
    ("serdes_gth3",   "GTH (UltraScale)",         "GTH3",          "liteiclink/serdes/gth3_ultrascale.py"),
    ("serdes_gty",    "GTY (UltraScale+)",        "GTY",           "liteiclink/serdes/gty_ultrascale.py"),
    ("serdes_drp",    "DRP access",               "DRPControl",    "liteiclink/serdes/common.py"),
    ("serdes_ecp5",   "ECP5 DCU",                 "SerDesECP5",    "liteiclink/serdes/serdes_ecp5.py"),
    ("serdes_gtr12",  "GTR12 (Gowin GW5)",        "GTR12",         "liteiclink/serdes/gtr12_gw5.py"),
    ("serwb_phy_s7",  "SerWB PHY, 7-Series",      "SerdesControl", "liteiclink/serwb/common.py"),
    ("serwb_phy_gen", "SerWB PHY, generic",       "SerdesControl", "liteiclink/serwb/common.py"),
    ("serwb_core",    "SerWB core",               "SERWBCore",     "liteiclink/serwb/core.py"),
]

# Bank renames: the SerDes wrappers of the benches are all named `serdes0` and the SerWB PHYs
# `serwb_master_phy`; each family gets its own name in the appendix so that each has its own table.
RENAMES = {
    "gtp"       : {"serdes0": "serdes_gtp"},
    "gtx_serwb" : {"serdes0": "serdes_gtx", "serdes0_drp": "serdes_drp",
                   "serwb_master_phy": "serwb_phy_s7", "serwb_master_core": "serwb_core"},
    "gth3"      : {"serdes0": "serdes_gth3"},
    "gty"       : {"serdes0": "serdes_gty"},
    "ecp5"      : {"serdes0": "serdes_ecp5"},
    "gtr12"     : {"serdes0": "serdes_gtr12"},
    "serwb_gen" : {"serwb_master_phy": "serwb_phy_gen"},
}

# Descriptions of the registers that the gateware declares without one. Keys: (group, register) or
# (group, register, field); a group covers the banks of _DESC_GROUPS. A description declared in
# the gateware takes precedence. Fields declared with `values` and no description get the list of
# their values (see _describe).
DESCRIPTIONS = {
    ("gtr12", "upar_ctrl") : "Write strobe: bit 0 requests a write of `upar_wrdata` at `upar_addr`, bit 1 a read.",

    ("serwb_phy", "control_reset")           : "Write strobe (link master only): restart the link initialization of both sides.",
    ("serwb_phy", "control_ready")           : "1: link initialized, words are exchanged.",
    ("serwb_phy", "control_error")           : "1: the initialization failed: no sampling window or word alignment found.",
    ("serwb_phy", "control_delay")           : "Input delay (taps) chosen at the centre of the sampling window.",
    ("serwb_phy", "control_delay_min_found") : "1: the lower edge of the sampling window was found.",
    ("serwb_phy", "control_delay_min")       : "Lower edge of the sampling window (taps).",
    ("serwb_phy", "control_delay_max_found") : "1: the upper edge of the sampling window was found.",
    ("serwb_phy", "control_delay_max")       : "Upper edge of the sampling window (taps).",
    ("serwb_phy", "control_shift")           : "Word alignment found by the initialization (bitslip).",
    ("serwb_phy", "control_phase")           : "Sampling phase found by the initialization, the last one tried after an error (Efinix PHY).",
    ("serwb_phy", "control_prbs_start")      : "Write strobe: clear `control_prbs_errors` and count errors for `control_prbs_cycles` cycles.",
    ("serwb_phy", "control_prbs_cycles")     : "Duration of a link test, in `sys` cycles.",
    ("serwb_phy", "control_prbs_errors")     : "Errors counted by the last link test.",
}

_DESC_GROUPS = {
    "serdes_ecp5"   : "ecp5",
    "serdes_gtr12"  : "gtr12",
    "serwb_phy_s7"  : "serwb_phy",
    "serwb_phy_gen" : "serwb_phy",
}

# Notes printed after the title of a bank: what the register descriptions do not say.
NOTES = {
    "serdes_gtp": [
        "The registers of `add_controls()` and `add_clock_cycles()`, the same on every Xilinx",
        "wrapper except the electrical controls (see the notes of the other banks).",
        "`clock_aligner_disable` exists only with `clock_aligner=True`, the default; the bench",
        "targets build without the clock aligner. `clock_tx_cycles` and `clock_rx_cycles` are",
        "read-only and take the counts of the TX and RX user clocks when software writes `clock_latch`.",
        "Bit 2 of `rx_prbs_config` (`pause`) freezes `rx_prbs_errors` for a coherent read.",
    ],
    "serdes_gtx": [
        "Same layout as the GTP bank. The Virtex-7 GTH wrapper (`GTH` of",
        "`liteiclink/serdes/gth_7series.py`) has the same registers, with the same reset values.",
    ],
    "serdes_gth3": [
        "Same layout as the GTP bank, with `tx_diffctrl` reset to `0xc`. The UltraScale+ GTH",
        "wrapper (`GTH4`) has the same registers without `tx_postcursor_inv` and",
        "`tx_precursor_inv`.",
    ],
    "serdes_gty": [
        "No `tx_postcursor_inv` and `tx_precursor_inv`, and a 5-bit `tx_diffctrl`. The raw",
        "symbol interface and the TXRATE/RXRATE inputs of the GTY are gateware signals, not",
        "registers.",
    ],
    "serdes_drp": [
        "Added by the design, not by the wrappers: `DRPControl(serdes.drp)` gives software access",
        "to the dynamic reconfiguration port of a Xilinx channel. Software writes `addr`, `di`",
        "and `we`, then any value to `start`, and polls `done` before reading `do`.",
    ],
    "serdes_ecp5": [
        "The base, PRBS and clock-cycle registers have the layout of the Xilinx banks, without",
        "polarity and electrical controls; `loopback` takes the 3-bit modes of the Xilinx banks,",
        "mapped to the DCU loopbacks, and `tx_inhibit` puts the TX output in electrical idle",
        "([Lattice ECP5 DCU](#ecp5-controls)). The `sci_reconfig_` registers",
        "are always present: they give software byte access to the SCI configuration registers",
        "of the DCU (`sel` 0 for the channel, 1 for the dual), and `pause` stops the hardware",
        "reconfiguration sequence while software uses them.",
    ],
    "serdes_gtr12": [
        "The GTR12 wrapper has no `add_controls()`: `add_status_csr()` adds `status` and",
        "`upar.add_csr()` the `upar_` registers. The `upar_` registers reach the quad registers",
        "at run time; requests of the user logic have priority over them",
        "([Gowin GW5 GTR12](#gowin-gw5)).",
    ],
    "serwb_phy_s7": [
        "The `control_` registers of a high-speed SerWB PHY, here a 7-Series link master built",
        "with `with_prbs=True` and `with_auto_retrain=True`. The UltraScale and Efinix PHYs have",
        "the same bank; the Efinix PHY adds `control_phase`, its RX phase selection. A link slave has no",
        "`control_reset`: the master resets it through the link. `control_prbs_tx` and",
        "`control_prbs_rx` exist only with `with_prbs=True`; without them, a link test counts the",
        "received idle words that are not zero, which measures the link only without flow control",
        "(with flow control the fill words are removed before the counter). `control_prbs_cycles`",
        "counts `sys` cycles, and `control_prbs_errors` the words with one or more wrong bits.",
        "`control_invalid_errors` counts the words with a symbol of an invalid ones count; it is cleared while the link",
        "is not ready; it and `control_link_drops` stop at their maximum value.",
    ],
    "serwb_phy_gen": [
        "The generic PHY has no input delay: no `control_delay_` and no `control_phase`",
        "registers. The other registers are those of the high-speed PHYs.",
    ],
    "serwb_core": [
        "`crc_errors` exists only with `with_crc=True`, the option enabled here.",
        "`rx_overflows` counts the events where a received packet word was lost because the RX",
        "buffer of the core was full, which flow control avoids; lost idle words are not counted",
        "([SerWB architecture](#serwb-architecture)).",
    ],
}

# Opening paragraph of the appendix and the sentence that ends the access paragraph.
INTRO = ["This appendix lists the registers of LiteICLink: the controls of each SerDes wrapper,",
         "the DRP access block, and the SerWB PHYs and core. It is generated from the register",
         "definitions of the gateware, elaborated without running an FPGA toolchain. Every",
         "configuration below is a design of the manual only (`doc/manual/registers.py`),",
         "elaborated for this appendix on a LiteX-Boards platform: its clocks and pads are",
         "placeholders and it is not a design to build. Offsets are in bytes from the base of",
         "the bank; the base addresses depend on the SoC and are in the `csr.csv` of a build.",
         "A build names a bank after the module of the SoC, `serdes0` for the SerDes wrapper of",
         "the benches and `serwb_master_phy` for a SerWB PHY; this appendix names each one after",
         "its family (`serdes_gtx`, `serwb_phy_s7`) to keep them apart. The Efinix SerWB PHY,",
         "the Virtex-7 GTH and the UltraScale+ GTH are not elaborated: the notes describe their",
         "banks by difference. The configurations, as the docstrings of `registers.py` describe",
         "them:",
         "",
         "- `gtp`: Artix-7 GTP on the quad PLL, reference clock from the fabric, clock aligner on",
         "  (SQRL Acorn platform).",
         "- `gtx_serwb`: Kintex-7 GTX on the CPLL with DRP access, and a 7-Series SerWB link master",
         "  with every register option and a core with `with_crc=True` (KC705 platform).",
         "- `gth3`: UltraScale GTH on the CPLL (KCU105 platform).",
         "- `gty`: UltraScale+ GTY on the quad PLL, 40-bit words (XCU1525 platform).",
         "- `ecp5`: ECP5 DCU 1, channel 0, at 2.5 Gbps (ECPIX-5 platform).",
         "- `gtr12`: GW5AST-138 GTR12 lane with the status and UPAR registers, on a placeholder",
         "  quad configuration (Tang Mega 138K Pro platform).",
         "- `serwb_gen`: generic SerWB PHY as link master with every register option (iCEBreaker",
         "  platform)."]
USAGE = ["The host scripts of `bench/` use these", "names through the `csr.csv` of a build."]
MAIN_BANK = "Registers of the top-level `main` bank of the SoC."

# Manual-only designs ------------------------------------------------------------------------------

def _crg(platform):
    """sys clock from the default clock of the board: enough to elaborate, never built."""
    from litex.build.io import CRG
    return CRG(platform.request(platform.default_clk_name))


def _pads():
    """Differential pads of a transceiver lane (placeholders, never built)."""
    from migen import Record
    return Record([("p", 1), ("n", 1)]), Record([("p", 1), ("n", 1)])


def _soc(platform, sys_clk_freq):
    from litex.soc.integration.soc_core import SoCMini
    soc     = SoCMini(platform, clk_freq=sys_clk_freq, ident="LiteICLink registers")
    soc.crg = _crg(platform)
    return soc


def _add_serdes(soc, serdes):
    soc.serdes0 = serdes
    serdes.add_stream_endpoints()
    serdes.add_controls()
    serdes.add_clock_cycles()


def soc_gtp():
    """Artix-7 GTP on the QPLL (reference clock from the fabric), clock aligner on (SQRL Acorn)."""
    from migen import Signal
    from litex_boards.platforms import sqrl_acorn
    from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP
    platform = sqrl_acorn.Platform()
    soc      = _soc(platform, int(100e6))
    pll      = GTPQuadPLL(Signal(), 125e6, 5e9, refclk_from_fabric=True)
    _add_serdes(soc, GTP(pll, *_pads(), int(100e6)))
    return soc


def soc_gtx_serwb():
    """Kintex-7 GTX on the CPLL with DRP access, and a 7-Series SerWB link master with every
    register option (KC705)."""
    from migen import Signal, Record, ClockDomain
    from litex.soc.integration.soc import SoCRegion
    from litex_boards.platforms import xilinx_kc705
    from liteiclink.serdes.gtx_7series import GTXChannelPLL, GTX
    from liteiclink.serdes.common import DRPControl
    from liteiclink.serwb.phy import SERWBPHY
    from liteiclink.serwb.core import SERWBCore
    platform = xilinx_kc705.Platform()
    soc      = _soc(platform, int(125e6))
    soc.cd_sys4x = ClockDomain()
    pll      = GTXChannelPLL(Signal(), 125e6, 2.5e9)
    _add_serdes(soc, GTX(pll, *_pads(), int(125e6)))
    soc.serdes0_drp = DRPControl(soc.serdes0.drp)
    pads = Record([(n, 1) for n in ["clk_p", "clk_n", "tx_p", "tx_n", "rx_p", "rx_n"]])
    soc.serwb_master_phy  = SERWBPHY(platform.device, pads, mode="master",
        with_prbs         = True,
        with_auto_retrain = True,
    )
    soc.serwb_master_core = SERWBCore(soc.serwb_master_phy, int(125e6), mode="slave", with_crc=True)
    soc.bus.add_slave("serwb", soc.serwb_master_core.bus, SoCRegion(origin=0x30000000, size=0x10000000))
    return soc


def soc_gth3():
    """UltraScale GTH on the CPLL (KCU105)."""
    from migen import Signal
    from litex_boards.platforms import xilinx_kcu105
    from liteiclink.serdes.gth3_ultrascale import GTHChannelPLL, GTH3
    platform = xilinx_kcu105.Platform()
    soc      = _soc(platform, int(125e6))
    pll      = GTHChannelPLL(Signal(), 125e6, 2.5e9)
    _add_serdes(soc, GTH3(pll, *_pads(), int(125e6)))
    return soc


def soc_gty():
    """UltraScale+ GTY on the QPLL, 40-bit words (SQRL XCU1525)."""
    from migen import Signal
    from litex_boards.platforms import sqrl_xcu1525
    from liteiclink.serdes.gty_ultrascale import GTYQuadPLL, GTY
    platform = sqrl_xcu1525.Platform()
    soc      = _soc(platform, int(125e6))
    pll      = GTYQuadPLL(Signal(), 125e6, 5e9)
    _add_serdes(soc, GTY(pll, *_pads(), int(125e6), data_width=40))
    return soc


def soc_ecp5():
    """ECP5 DCU1 channel 0 at 2.5 Gbps (ECPIX-5)."""
    from migen import Signal
    from litex_boards.platforms import lambdaconcept_ecpix5
    from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5
    platform = lambdaconcept_ecpix5.Platform(toolchain="trellis")
    soc      = _soc(platform, int(100e6))
    pll      = SerDesECP5PLL(Signal(), 100e6, 2.5e9)
    _add_serdes(soc, SerDesECP5(pll, *_pads(), dual=1, channel=0))
    return soc


def soc_gtr12():
    """GW5AST-138 GTR12 lane with the status and UPAR registers (Tang Mega 138K Pro). The quad
    configuration is a placeholder (one write), not a working configuration."""
    from litex_boards.platforms import sipeed_tang_mega_138k_pro
    from liteiclink.serdes.gtr12_gw5 import GTR12
    platform = sipeed_tang_mega_138k_pro.Platform()
    soc      = _soc(platform, int(100e6))
    soc.serdes0 = serdes = GTR12(platform, 5e9, "upar_write_driver(0x000000,0x00000000)\n")
    serdes.add_stream_endpoints()
    serdes.add_status_csr()
    serdes.upar.add_csr()
    return soc


def soc_serwb_gen():
    """Generic SerWB PHY as link master with every register option (iCEBreaker)."""
    from migen import Record
    from litex_boards.platforms import icebreaker
    from liteiclink.serwb.genphy import SERWBPHY
    platform = icebreaker.Platform()
    soc      = _soc(platform, int(12e6))
    pads     = Record([("clk", 1), ("tx", 1), ("rx", 1)])
    soc.serwb_master_phy = SERWBPHY(platform.device, pads, mode="master",
        with_prbs         = True,
        with_auto_retrain = True,
    )
    return soc

# Elaboration --------------------------------------------------------------------------------------

def _rename(soc, csr_json, old, new):
    soc.csr.regions[new] = soc.csr.regions.pop(old)
    csr_json["csr_bases"][new] = csr_json["csr_bases"][old]
    for reg in list(csr_json["csr_registers"]):
        if reg.startswith(old + "_"):
            csr_json["csr_registers"][new + reg[len(old):]] = csr_json["csr_registers"][reg]


def _values(values):
    return " ".join(f"`{v[0].strip('`')}`: {' '.join(v[1].split())}" for v in values)


def _sentence(text):
    """End a description with a period, so that the reset value that follows reads as a sentence."""
    text = " ".join(text.split())
    return text if not text or text.endswith((".", ")")) else text + "."


def _describe(soc):
    """Attach the descriptions of DESCRIPTIONS, or the values of a field, to the registers and
    fields that have none in the gateware."""
    for bank, region in soc.csr.regions.items():
        group = _DESC_GROUPS.get(bank, bank)
        for csr in region.obj:
            desc = DESCRIPTIONS.get((group, csr.name))
            if desc and not (getattr(csr, "description", None) or "").strip():
                csr.description = desc
            if getattr(csr, "description", None):
                csr.description = _sentence(csr.description)
            fields = getattr(csr, "fields", None)
            for f in (fields.fields if fields else []):
                desc = DESCRIPTIONS.get((group, csr.name, f.name))
                if not desc and f.values:
                    desc = _values(f.values)
                if desc and not (f.description or "").strip():
                    f.description = desc


def elaborate(name, repo_root):
    """Build a design of this file and elaborate it without the toolchain; return (soc, csr_json,
    module, argv)."""
    script, func = CONFIGS[name]
    sys.path.insert(0, repo_root)
    from litex.soc.integration.builder import Builder
    old_cwd = os.getcwd()

    with tempfile.TemporaryDirectory() as tmp:
        os.chdir(tmp)
        try:
            soc = globals()[func]()
            builder = Builder(soc, output_dir=tmp, csr_csv=os.path.join(tmp, "csr.csv"),
                csr_json=os.path.join(tmp, "csr.json"), compile_software=False,
                compile_gateware=False)
            builder.build(run=False)
        finally:
            os.chdir(old_cwd)
        csr_json = json.load(open(os.path.join(tmp, "csr.json")))

    # Memories mapped on the CSR bus have no register definitions: keep the register banks only.
    for bank, region in list(soc.csr.regions.items()):
        if not isinstance(region.obj, list):
            del soc.csr.regions[bank]
    for old, new in RENAMES[name].items():
        _rename(soc, csr_json, old, new)
    _describe(soc)
    return soc, csr_json, "doc/manual/registers.py", [func]
