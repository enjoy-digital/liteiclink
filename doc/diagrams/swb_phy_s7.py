#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# 7-Series SerWB PHY: S7Serdes (TX datapath and OSERDESE2, IDELAYE2 and ISERDESE2, RX datapath) inside
# SERWBPHY, whose init FSM drives the delay taps and the bitslip. The clock lane is not drawn.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    d.add(Container(10, 10, 610, 360, title="SERWBPHY", title_pos="br"))
    d.add(Container(22, 25, 586, 230, title="S7Serdes", title_pos="br"))
    txd = d.add(Box(35, 40, 150, 56, label="TX datapath\n8b/10b, 40 to 8", fill=COLOR_ACCENT_LIGHT))
    ose = d.add(Box(420, 40, 150, 56, label="OSERDESE2\n8:1 DDR", fill=COLOR_ACCENT))
    rxd = d.add(Box(35, 150, 150, 56, label="RX datapath\n8 to 40, 8b/10b", fill=COLOR_ACCENT_LIGHT))
    ise = d.add(Box(235, 150, 135, 56, label="ISERDESE2\n1:8 DDR", fill=COLOR_ACCENT))
    idl = d.add(Box(420, 150, 150, 56, label="IDELAYE2\n32 taps", fill=COLOR_ACCENT))
    ini = d.add(Box(235, 280, 135, 50, label="Init FSM"))
    lnk = d.add(Box(650, 40, 95, 166, label="LVDS\npairs", fill=COLOR_HARDWARE))
    d.add(Arrow((txd, "r"), (ose, "l"), kind="data", label="8 bits per sys cycle"))
    d.add(Arrow((ose, "r"), (lnk, "l", 28/166), kind="data"))
    d.add(Arrow((lnk, "l", 138/166), (idl, "r"), kind="data"))
    d.add(Arrow((idl, "l"), (ise, "r"), kind="data"))
    d.add(Arrow((ise, "l"), (rxd, "r"), kind="data"))
    d.add(Arrow((ini, "t"), (ise, "b"), kind="control", label="bitslip", label_side="left"))
    d.add(Arrow((ini, "r"), (idl, "b"), kind="control", label="delay", label_pos=(470, 295)))
    return d
