#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Overview of a SerDes wrapper: user core, wrapper, transceiver, line to the other end.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    d.add(Container(10, 10, 510, 210, title="FPGA", title_pos="br"))
    user = d.add(Box(25, 50, 120, 50, label="User core", fill=COLOR_ACCENT_LIGHT))
    wrap = d.add(Box(200, 50, 140, 50, label="SerDes wrapper", fill=COLOR_ACCENT))
    xcvr = d.add(Box(380, 50, 125, 50, label="Transceiver", fill=COLOR_HARDWARE))
    regs = d.add(Box(200, 140, 140, 44, label="Registers"))
    peer = d.add(Box(610, 50, 130, 50, label="Other end", fill=COLOR_HARDWARE))
    d.add(Arrow((user, "r"), (wrap, "l"), kind="accent", heads="both", label="words"))
    d.add(Arrow((wrap, "r"), (xcvr, "l"), kind="data", heads="both"))
    d.add(Arrow((xcvr, "r"), (peer, "l"), kind="data", heads="both", label="serial pairs"))
    d.add(Arrow((wrap, "b"), (regs, "t"), kind="control", heads="both"))
    return d
