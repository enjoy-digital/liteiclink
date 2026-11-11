#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB between two LiteX SoCs: the SoC side maps a region to the link, the remote side masters
# its own bus.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(20, 20, 330, 260, title="FPGA A: SoC side", title_pos="bl"))
    d.add(Container(410, 20, 330, 260, title="FPGA B: remote side", title_pos="br"))
    mst   = d.add(Box(45, 45, 130, 50, label="CPU or\nhost bridge"))
    busa  = d.add(Box(45, 135, 130, 50, label="SoC bus\nSerWB region"))
    corea = d.add(Box(205, 45, 120, 50, label="SERWBCore\nslave", fill=COLOR_ACCENT_LIGHT))
    phya  = d.add(Box(205, 135, 120, 100, label="SERWBPHY\nmaster", fill=COLOR_ACCENT))
    phyb  = d.add(Box(435, 135, 120, 100, label="SERWBPHY\nslave", fill=COLOR_ACCENT))
    coreb = d.add(Box(435, 45, 120, 50, label="SERWBCore\nmaster", fill=COLOR_ACCENT_LIGHT))
    busb  = d.add(Box(585, 45, 130, 50, label="Remote\nbus"))
    perb  = d.add(Box(585, 135, 130, 50, label="Peripherals,\nmemory"))
    d.add(Arrow((mst, "b"), (busa, "t"), kind="data", heads="both"))
    d.add(Arrow((busa, "r", 0.3), (corea, "l"), kind="data", heads="both"))
    d.add(Arrow((corea, "b"), (phya, "t"), kind="data", heads="both"))
    d.add(Arrow((phya, "r", 0.2), (phyb, "l", 0.2), kind="accent", label="clock"))
    d.add(Arrow((phya, "r", 0.5), (phyb, "l", 0.5), kind="accent", label="requests"))
    d.add(Arrow((phyb, "l", 0.8), (phya, "r", 0.8), kind="accent", label="responses"))
    d.add(Arrow((phyb, "t"), (coreb, "b"), kind="data", heads="both"))
    d.add(Arrow((coreb, "r"), (busb, "l"), kind="data", heads="both"))
    d.add(Arrow((busb, "b"), (perb, "t"), kind="data", heads="both"))
    return d
