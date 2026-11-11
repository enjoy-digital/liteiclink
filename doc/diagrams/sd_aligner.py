#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Brute-force clock aligner: wait for the comma on the first symbol, then for a period without
# symbol errors; any failure restarts the RX init FSM.

from svgdiag import *
from sd_helpers import frac, hfrac


def build():
    d = Diagram(CANVAS_WIDTH, 240)
    w, h = 160, 56
    wc  = d.add(Box(15,  30, w, h, label="Wait comma"))
    wne = d.add(Box(300, 30, w, h, label="Wait no error"))
    rdy = d.add(Box(585, 30, w, h, label="Ready", fill=COLOR_ACCENT))
    rst = d.add(Box(300, 165, w, h, label="Restart RX init", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((wc, "r"), (wne, "l"), kind="accent", label="comma", label_pos=(237, 48)))
    d.add(Arrow((wne, "r"), (rdy, "l"), kind="accent", label="no error", label_pos=(522, 48)))
    yr = rst.cy
    d.add(Arrow((wc, "b", hfrac(wc, 95)), (rst, "l"), via=[(95, yr)], label="no comma",
        label_pos=(190, yr - 8)))
    d.add(Arrow((wne, "b"), (rst, "t"), label="error", label_pos=(388, 128), label_anchor="start"))
    d.add(Arrow((rdy, "b", hfrac(rdy, 665)), (rst, "r"), via=[(665, yr)], label="error",
        label_pos=(560, yr - 8)))
    return d
