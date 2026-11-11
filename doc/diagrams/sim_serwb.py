#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB Verilator simulation (bench/serwb/sim.py): both sides of a link in one simulated SoC, the
# two generic PHYs wired back to back, a remote SRAM, and the host or a test FSM as bus master.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(170, 20, 570, 250, title="Verilator: sim.py", title_pos="bl"))
    host = d.add(Box(20, 70, 120, 50, label="Host\nlitex_server", fill=COLOR_HARDWARE))
    mst  = d.add(Box(195, 70, 150, 50, label="Etherbone or\ntest FSM"))
    cs   = d.add(Box(380, 70, 140, 50, label="SERWBCore\nslave", fill=COLOR_ACCENT_LIGHT))
    pm   = d.add(Box(570, 70, 140, 50, label="Generic PHY\nmaster", fill=COLOR_ACCENT))
    ps   = d.add(Box(570, 180, 140, 50, label="Generic PHY\nslave", fill=COLOR_ACCENT))
    cm   = d.add(Box(380, 180, 140, 50, label="SERWBCore\nmaster", fill=COLOR_ACCENT_LIGHT))
    sram = d.add(Box(195, 180, 150, 50, label="SRAM"))
    d.add(Arrow((host, "r"), (mst, "l"), kind="data", heads="both"))
    d.add(Text(80, 140, "interactive mode only", size=FONT_SIZE_SM))
    d.add(Arrow((mst, "r"), (cs, "l"), kind="data", heads="both"))
    d.add(Arrow((cs, "r"), (pm, "l"), kind="data", heads="both"))
    d.add(Arrow((pm, "b", 0.3), (ps, "t", 0.3), kind="accent", label="tx"))
    d.add(Arrow((ps, "t", 0.7), (pm, "b", 0.7), kind="accent", label="rx"))
    d.add(Arrow((ps, "l"), (cm, "r"), kind="data", heads="both"))
    d.add(Arrow((cm, "l"), (sram, "r"), kind="data", heads="both"))
    return d
