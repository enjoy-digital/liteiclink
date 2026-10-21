#!/usr/bin/env python3

#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
SerWB resource usage estimation (Yosys synth_xilinx, 7-Series cells).

Synthesizes SerWB PHY + Core configurations and reports LUT/FF/LUTRAM/CARRY usage, to track
resource usage of changes. Absolute numbers are estimations (Yosys), use them to compare.

Usage:
    ./resources.py                      # All configurations.
    ./resources.py --phy gen --json r.json
    ./resources.py --compare r.json     # Compare against a previous run.
"""

import os
import re
import json
import argparse
import subprocess
import tempfile

from migen import *

from litex.gen import *

from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect.csr import CSR, CSRStatus, CSRStorage

from liteiclink.serwb.phy    import SERWBPHY
from liteiclink.serwb.genphy import SERWBPHY as SERWBGenPHY
from liteiclink.serwb.core   import SERWBCore

# Configurations -----------------------------------------------------------------------------------

PHYS = {
    # name : (PHY class, device, differential pads, kwargs).
    "gen" : (SERWBGenPHY, "xc7a200t-sbg484-1",    False, {}),
    "s7"  : (SERWBPHY,    "xc7a200t-sbg484-1",    True,  {}),
    "ku"  : (SERWBPHY,    "xcku040-ffva1156-2-e", True,  {}),
}

OPTIONS = {
    # name : (PHY kwargs, Core kwargs).
    "default"      : ({},                          {}),
    "flow_control" : ({"with_flow_control": True}, {}),
    "crc"          : ({},                          {"with_crc": True}),
    "native"       : ({},                          {"protocol": "native"}),
    "native_fc"    : ({"with_flow_control": True}, {"protocol": "native"}),
    "prbs"         : ({"with_prbs": True},         {}),
}

# Design -------------------------------------------------------------------------------------------

class SERWBTop(LiteXModule):
    """SerWB PHY + Core, PHY "master" side (core "slave": bus exposed to the SoC) or opposite."""
    def __init__(self, phy_cls, device, diff, mode, phy_kwargs={}, core_kwargs={}):
        names = ["clk_p", "clk_n", "tx_p", "tx_n", "rx_p", "rx_n"] if diff else ["clk", "tx", "rx"]
        self.pads = pads = Record([(n, 1) for n in names])
        self.phy  = phy  = phy_cls(device=device, pads=pads, mode=mode, **phy_kwargs)
        self.core = core = SERWBCore(phy, int(100e6),
            mode = {"master": "slave", "slave": "master"}[mode],
            **core_kwargs
        )
        self.ios = set(pads.flatten()) | set(core.bus.flatten())
        # Keep CSR logic (as read/written by a CSR bank in a SoC), otherwise pruned by synthesis.
        for csr in phy.get_csrs() + core.get_csrs():
            if isinstance(csr, CSRStatus):
                self.ios.add(csr.status)
            elif isinstance(csr, CSRStorage):
                self.ios |= {csr.storage, csr.re}
            elif isinstance(csr, CSR):
                self.ios |= {csr.r, csr.re}
        for name in ["sys", "sys4x", "sys4x_dly", "idelay"]:
            cd = ClockDomain(name)
            setattr(self, f"cd_{name}", cd)
            self.ios |= {cd.clk, cd.rst}

# Synthesis ----------------------------------------------------------------------------------------

CELLS = {
    "LUT"    : r"LUT[1-6]",
    "FF"     : r"FD[A-Z]{2}",
    "LUTRAM" : r"RAM\w+",
    "SRL"    : r"SRL\w+",
    "CARRY"  : r"CARRY\w+",
}

def synthesize(top, device):
    platform = XilinxPlatform(device, io=[])
    with tempfile.TemporaryDirectory() as tmp:
        v = os.path.join(tmp, "top.v")
        platform.get_verilog(top, ios=top.ios, name="top").write(v)
        log = subprocess.run(["yosys", "-q", "-p", f"read_verilog {v}; synth_xilinx -top top -flatten; tee -o {tmp}/stat.txt stat"],
            capture_output=True, text=True, check=True)
        stat = open(os.path.join(tmp, "stat.txt")).read()
    results = {k: 0 for k in CELLS}
    for cell, count in re.findall(r"^\s+(\S+)\s+(\d+)\s*$", stat, re.M):
        for k, pattern in CELLS.items():
            if re.fullmatch(pattern, cell):
                results[k] += int(count)
    return results

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SerWB resource usage estimation (Yosys).")
    parser.add_argument("--phy",     default="all", help=f"PHY(s) ({', '.join(PHYS)}) or all.")
    parser.add_argument("--options", default="all", help=f"Option(s) ({', '.join(OPTIONS)}) or all.")
    parser.add_argument("--mode",    default="master", choices=["master", "slave"], help="PHY mode.")
    parser.add_argument("--json",    default=None,  help="Save results to JSON file.")
    parser.add_argument("--compare", default=None,  help="Compare with results from JSON file.")
    args = parser.parse_args()

    phys    = list(PHYS)    if args.phy     == "all" else args.phy.split(",")
    options = list(OPTIONS) if args.options == "all" else args.options.split(",")
    ref     = json.load(open(args.compare)) if args.compare else {}

    results = {}
    print(f"{'Configuration':28s}" + "".join(f"{k:>14s}" for k in CELLS))
    for phy in phys:
        phy_cls, device, diff, phy_kwargs = PHYS[phy]
        for option in options:
            opt_phy_kwargs, opt_core_kwargs = OPTIONS[option]
            name = f"{phy}/{args.mode}/{option}"
            top  = SERWBTop(phy_cls, device, diff, args.mode,
                phy_kwargs  = {**phy_kwargs, **opt_phy_kwargs},
                core_kwargs = opt_core_kwargs,
            )
            r = results[name] = synthesize(top, device)
            line = f"{name:28s}"
            for k in CELLS:
                cell = f"{r[k]}"
                if name in ref:
                    cell += f" ({r[k] - ref[name][k]:+d})"
                line += f"{cell:>14s}"
            print(line, flush=True)

    if args.json:
        json.dump(results, open(args.json, "w"), indent=2)

if __name__ == "__main__":
    main()
