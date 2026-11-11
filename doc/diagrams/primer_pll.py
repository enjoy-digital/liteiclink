#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A transceiver PLL: reference clock, /M, phase detector and loop filter, VCO, /N feedback, /D.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 185)
    m   = d.add(Box(140, 50, 60, 44, label="/M"))
    pfd = d.add(Box(230, 50, 150, 44, label="Phase detector\nloop filter", size=13, fill=COLOR_ACCENT_LIGHT))
    vco = d.add(Box(410, 50, 80, 44, label="VCO", fill=COLOR_ACCENT))
    dd  = d.add(Box(540, 50, 60, 44, label="/D"))
    n   = d.add(Box(345, 130, 60, 36, label="/N"))
    d.add(Arrow((15, 72), (m, "l"), kind="control", label="reference clock"))
    d.add(Arrow((m, "r"), (pfd, "l"), kind="control"))
    d.add(Arrow((pfd, "r"), (vco, "l"), kind="control"))
    d.add(Arrow((vco, "r"), (dd, "l"), kind="accent"))
    d.add(Arrow((dd, "r"), (740, 72), kind="accent", label="line rate"))
    d.add(Arrow((vco, "b"), (n, "r"), kind="feedback", via=[(450, 148)]))
    d.add(Arrow((n, "l"), (pfd, "b", 0.4), kind="feedback", via=[(290, 148)]))
    return d
