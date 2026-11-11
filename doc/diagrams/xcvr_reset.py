#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Init FSM of a Xilinx channel: power down, PLL lock, init delay, reset done, CDR lock, phase
# alignment (buffer bypass), ready; the watchdog restarts the sequence.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 255)
    pd   = d.add(Box(20, 30, 130, 44, label="Power down"))
    lock = d.add(Box(200, 30, 130, 44, label="PLL lock"))
    dly  = d.add(Box(380, 30, 130, 44, label="Init delay"))
    done = d.add(Box(560, 30, 130, 44, label="Reset done"))
    cdr  = d.add(Box(200, 150, 130, 44, label="CDR lock"))
    phal = d.add(Box(380, 150, 130, 44, label="Phase align", fill=COLOR_ACCENT_LIGHT))
    rdy  = d.add(Box(560, 150, 130, 44, label="Ready", fill=COLOR_ACCENT))
    wd   = d.add(Box(20, 150, 130, 44, label="Watchdog", fill=COLOR_SHADE))
    d.add(Arrow((pd, "r"), (lock, "l"), kind="data"))
    d.add(Arrow((lock, "r"), (dly, "l"), kind="data"))
    d.add(Arrow((dly, "r"), (done, "l"), kind="data"))
    d.add(Arrow((done, "b"), (cdr, "t"), kind="data", via=[(625, 112), (265, 112)]))
    d.add(Arrow((cdr, "r"), (phal, "l"), kind="data"))
    d.add(Arrow((phal, "r"), (rdy, "l"), kind="data"))
    d.add(Arrow((cdr, "b"), (rdy, "b"), kind="control", via=[(265, 222), (625, 222)]))
    d.add(Text(445, 236, "buffers enabled", size=FONT_SIZE_SM))
    d.add(Arrow((wd, "t"), (pd, "b"), kind="feedback", label="restart", label_side="right"))
    d.add(Mark(705, 172))
    return d
