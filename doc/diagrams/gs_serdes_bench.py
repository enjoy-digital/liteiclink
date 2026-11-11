#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# First SerDes bench: host scripts, UART bridge, bench SoC registers, transceiver, loopback.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(20, 20, 200, 210, title="Host", title_pos="bl"))
    d.add(Container(260, 20, 340, 210, title="Bench SoC", title_pos="bl"))
    test   = d.add(Box(45, 45, 150, 50, label="test_prbs.py"))
    server = d.add(Box(45, 135, 150, 50, label="litex_server"))
    bridge = d.add(Box(285, 135, 120, 50, label="UARTBone"))
    regs   = d.add(Box(285, 45, 120, 50, label="serdes0\nregisters", fill=COLOR_ACCENT_LIGHT))
    serdes = d.add(Box(450, 45, 125, 140, label="SerDes\nwrapper\nPRBS", fill=COLOR_ACCENT))
    loop   = d.add(Box(640, 65, 100, 90, label="Loopback\nor cable", fill=COLOR_HARDWARE))
    d.add(Arrow((test, "b"), (server, "t"), kind="control", heads="both", label="TCP", label_side="right"))
    d.add(Arrow((server, "r"), (bridge, "l"), kind="control", heads="both", label="UART"))
    d.add(Arrow((bridge, "t"), (regs, "b"), kind="control", heads="both"))
    d.add(Arrow((regs, "r"), (serdes, "l", 0.179), kind="control", heads="both"))
    d.add(Arrow((serdes, "r", 0.357), (loop, "l", 0.333), kind="accent", label="TX", label_pos=(615, 85)))
    d.add(Arrow((loop, "l", 0.667), (serdes, "r", 0.5714), kind="accent", label="RX", label_pos=(615, 140)))
    return d
