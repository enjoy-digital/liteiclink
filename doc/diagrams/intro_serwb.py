#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB between two FPGAs: SoC, SoC side core, link master PHY; link slave PHY, remote side core,
# peripherals; clock, TX and RX links between the PHYs.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 210)
    d.add(Container(10, 10, 325, 190, title="FPGA A", title_pos="bl"))
    d.add(Container(425, 10, 325, 190, title="FPGA B", title_pos="br"))
    soc  = d.add(Box(25, 50, 80, 60, label="SoC"))
    core = d.add(Box(120, 50, 95, 60, label="Core\nSoC side", fill=COLOR_ACCENT))
    phym = d.add(Box(230, 50, 95, 60, label="PHY\nlink master", fill=COLOR_ACCENT_LIGHT))
    phys = d.add(Box(435, 50, 95, 60, label="PHY\nlink slave", fill=COLOR_ACCENT_LIGHT))
    corr = d.add(Box(545, 50, 95, 60, label="Core\nremote side", fill=COLOR_ACCENT))
    per  = d.add(Box(655, 50, 85, 60, label="Peripherals"))
    d.add(Arrow((soc, "r"), (core, "l"), kind="data", heads="both"))
    d.add(Arrow((core, "r"), (phym, "l"), kind="data", heads="both"))
    d.add(Arrow((phym, "r", 0.2), (phys, "l", 0.2), kind="control", label="clock"))
    d.add(Arrow((phym, "r", 0.55), (phys, "l", 0.55), kind="accent", label="TX"))
    d.add(Arrow((phys, "l", 0.9), (phym, "r", 0.9), kind="accent", label="RX", label_side="below"))
    d.add(Arrow((phys, "r"), (corr, "l"), kind="data", heads="both"))
    d.add(Arrow((corr, "r"), (per, "l"), kind="data", heads="both"))
    d.add(Text(70, 130, "Wishbone\nslave", size=FONT_SIZE_SM))
    d.add(Text(697, 130, "Wishbone\nmaster", size=FONT_SIZE_SM))
    return d
