#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Anatomy of an UltraScale transceiver quad: common block with the two quad PLLs, each feeding every
# channel; channel 0 with its channel PLL (fed by the reference clock), PMA and PCS; channels 1 to 3.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 265)
    d.add(Container(125, 10, 515, 245, title="Quad", title_pos="bl"))
    d.add(Container(140, 30, 160, 175, title="Common", title_pos="bl", title_size=15, fill="#f6f6f6"))
    d.add(Container(330, 30, 290, 125, title="Channel 0", title_pos="tr", title_size=15, fill="#f6f6f6"))
    q0   = d.add(Box(160, 50, 120, 40, label="QPLL0", fill=COLOR_ACCENT_LIGHT))
    q1   = d.add(Box(160, 115, 120, 40, label="QPLL1", fill=COLOR_ACCENT_LIGHT))
    cpll = d.add(Box(345, 50, 90, 34, label="CPLL", fill=COLOR_ACCENT_LIGHT))
    pma  = d.add(Box(345, 100, 100, 40, label="PMA", fill=COLOR_ACCENT))
    pcs  = d.add(Box(490, 100, 100, 40, label="PCS", fill=COLOR_ACCENT))
    chs  = d.add(Box(330, 180, 270, 44, label="Channels 1 to 3", stack=2))
    d.add(Arrow((15, 70), (q0, "l"), kind="control"))
    d.add(Text(65, 48, "reference\nclocks", size=FONT_SIZE_SM))
    d.add(Arrow((15, 135), (q1, "l"), kind="control"))
    d.add(Arrow((q0, "r"), (pma, "l", 0.3), kind="control", via=[(306, 70), (306, 112)]))
    d.add(Arrow((q0, "r"), (chs, "l", 0.3), kind="control", via=[(306, 70), (306, 193.2)]))
    d.add(Arrow((q1, "r"), (pma, "l", 0.7), kind="control", via=[(318, 135), (318, 128)]))
    d.add(Arrow((q1, "r"), (chs, "l", 0.7), kind="control", via=[(318, 135), (318, 210.8)]))
    d.add(Arrow((15, 20), (cpll, "t"), kind="control", via=[(390, 20)]))
    d.add(Arrow((cpll, "b", 0.44), (pma, "t", 0.4), kind="control"))
    d.add(Arrow((pma, "r"), (pcs, "l"), kind="data", heads="both"))
    d.add(Arrow((pcs, "r"), (740, 120), kind="data", heads="both", label="fabric"))
    return d
