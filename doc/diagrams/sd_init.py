#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Init FSM of the Xilinx SerDes wrappers (TX and RX): power down, optional DRP access, PLL lock,
# transceiver reset, buffer bypass alignment, ready; restart and watchdog back to power down.

from svgdiag import *
from sd_helpers import frac, hfrac


def build():
    d = Diagram(CANVAS_WIDTH, 265)
    w, h = 150, 56
    pd   = d.add(Box(15,  30, w, h, label="Power down,\nPLL reset"))
    drp  = d.add(Box(205, 30, w, h, label="DRP access"))
    lock = d.add(Box(395, 30, w, h, label="Wait PLL\nlock"))
    rel  = d.add(Box(585, 30, w, h, label="Release\nreset"))
    done = d.add(Box(585, 150, w, h, label="Wait reset\ndone"))
    aln  = d.add(Box(300, 150, w, h, label="Delay and\nphase align"))
    rdy  = d.add(Box(15, 150, w, h, label="Ready", fill=COLOR_ACCENT))
    for a, b in ((pd, drp), (drp, lock), (lock, rel)):
        d.add(Arrow((a, "r"), (b, "l"), kind="accent"))
    d.add(Arrow((rel, "b"), (done, "t"), kind="accent"))
    d.add(Arrow((done, "l"), (aln, "r"), kind="accent", label="buffers off", label_pos=(518, 168)))
    d.add(Arrow((aln, "l"), (rdy, "r"), kind="accent"))
    d.add(Arrow((done, "b", hfrac(done, 660)), (rdy, "b", hfrac(rdy, 90)), via=[(660, 240), (90, 240)],
        label="buffers on", label_pos=(375, 232)))
    d.add(Arrow((rdy, "t", hfrac(rdy, 60)), (pd, "b", hfrac(pd, 60)), kind="feedback",
        label="restart", label_pos=(70, 118), label_anchor="start"))
    return d
