#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Blocks of a LiteICLink SerDes wrapper: fabric 8b/10b encoder and decoders, TX source mux, init
# FSMs, around the transceiver channel and its PLL.

from svgdiag import *
from sd_helpers import frac, hfrac


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(150, 15, 420, 275, title="SerDes wrapper", title_pos="bl"))
    user = d.add(Box(15, 40, 95, 160, label="User core", fill=COLOR_ACCENT_LIGHT))
    enc  = d.add(Box(170, 50, 120, 50, label="Encoder", fill=COLOR_ACCENT))
    mux  = d.add(Box(330, 50, 120, 50, label="TX source\nmux", fill=COLOR_ACCENT))
    dec  = d.add(Box(330, 140, 120, 50, label="Decoders", fill=COLOR_ACCENT))
    init = d.add(Box(330, 215, 120, 60, label="TX and RX\ninit FSMs"))
    gt   = d.add(Box(605, 40, 140, 150, label="Transceiver\nchannel", fill=COLOR_HARDWARE))
    pll  = d.add(Box(605, 240, 140, 45, label="CPLL or QPLL", fill=COLOR_HARDWARE))
    yt, yr = 75, 165
    d.add(Arrow((user, "r", frac(user, yt)), (enc, "l"), kind="accent", label="sink", label_pos=(130, 65)))
    d.add(Arrow((enc, "r"), (mux, "l"), kind="accent"))
    d.add(Arrow((mux, "r"), (gt, "l", frac(gt, yt)), kind="accent", label="TXDATA", label_pos=(520, 65)))
    d.add(Arrow((gt, "l", frac(gt, yr)), (dec, "r"), kind="data", label="RXDATA", label_pos=(520, 155)))
    d.add(Arrow((dec, "l"), (user, "r", frac(user, yr)), kind="data", label="source", label_pos=(130, 155)))
    d.add(Arrow((init, "r", frac(init, 228)), (gt, "b", hfrac(gt, 640)), via=[(640, 228)],
        label="resets", label_pos=(520, 220)))
    d.add(Arrow((init, "r", frac(init, 262)), (pll, "l", frac(pll, 262)), label="reset", label_pos=(520, 254)))
    d.add(Arrow((pll, "t", hfrac(pll, 705)), (gt, "b", hfrac(gt, 705)), label="clock",
        label_side="right"))
    return d
