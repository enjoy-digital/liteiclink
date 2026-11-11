#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# GTY quad PLL: reference divider, QPLL0 or QPLL1 VCO, each with its own fractional feedback
# (sigma-delta modulator), full or half rate output, then the channel /D divider.

from svgdiag import *
from sd_helpers import frac, hfrac


def build():
    d = Diagram(CANVAS_WIDTH, 285)
    ref  = d.add(Box(15, 110, 105, 60, label="Reference\nclock", fill=COLOR_HARDWARE))
    m    = d.add(Box(155, 110, 70, 60, label="/M"))
    sdm0 = d.add(Box(265, 15, 140, 44, label="N + fraction (SDM0)", fill=COLOR_ACCENT_LIGHT))
    q0   = d.add(Box(265, 80, 140, 44, label="QPLL0 VCO", fill=COLOR_ACCENT))
    q1   = d.add(Box(265, 156, 140, 44, label="QPLL1 VCO", fill=COLOR_ACCENT))
    sdm1 = d.add(Box(265, 222, 140, 44, label="N + fraction (SDM1)", fill=COLOR_ACCENT_LIGHT))
    rate = d.add(Box(445, 105, 115, 70, label="Full or\nhalf rate"))
    div  = d.add(Box(595, 105, 70, 70, label="/D"))
    d.add(Arrow((ref, "r"), (m, "l")))
    d.add(Arrow((m, "r", 0.25), (q0, "l")))
    d.add(Arrow((m, "r", 0.75), (q1, "l")))
    d.add(Arrow((q0, "r"), (rate, "l", 0.25)))
    d.add(Arrow((q1, "r"), (rate, "l", 0.75)))
    d.add(Arrow((sdm0, "b"), (q0, "t"), kind="feedback"))
    d.add(Arrow((sdm1, "t"), (q1, "b"), kind="feedback"))
    d.add(Arrow((rate, "r"), (div, "l"), kind="accent"))
    d.add(Arrow((div, "r"), (745, div.cy), kind="accent", label="line rate", label_pos=(705, div.cy - 10)))
    return d
