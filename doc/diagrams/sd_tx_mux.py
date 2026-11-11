#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# TX source mux of the Xilinx wrappers: square wave, pattern, raw symbols (GTY) and the PRBS
# generator, which passes the encoder output through when PRBS is off.

from svgdiag import *
from sd_helpers import frac


def build():
    d = Diagram(CANVAS_WIDTH, 270)
    enc  = d.add(Box(15, 200, 120, 44, label="Encoder", fill=COLOR_ACCENT_LIGHT))
    sq   = d.add(Box(175, 20, 160, 44, label="Square wave"))
    pat  = d.add(Box(175, 80, 160, 44, label="tx_pattern"))
    raw  = d.add(Box(175, 140, 160, 44, label="Raw symbols (GTY)"))
    prbs = d.add(Box(175, 200, 160, 44, label="PRBS generator", fill=COLOR_ACCENT))
    mux  = d.add(Box(410, 20, 100, 224, label="TX source\nmux", fill=COLOR_ACCENT))
    gt   = d.add(Box(590, 92, 155, 80, label="Transceiver\nchannel", fill=COLOR_HARDWARE))
    d.add(Arrow((enc, "r"), (prbs, "l"), kind="accent"))
    for i, b in enumerate((sq, pat, raw, prbs)):
        d.add(Arrow((b, "r"), (mux, "l", frac(mux, b.cy)), kind="accent" if b is prbs else "data",
            label=str(i + 1), label_pos=(372, b.cy - 10)))
    d.add(Arrow((mux, "r", frac(mux, gt.cy)), (gt, "l"), kind="accent", label="TXDATA",
        label_pos=(550, gt.cy - 10)))
    return d
