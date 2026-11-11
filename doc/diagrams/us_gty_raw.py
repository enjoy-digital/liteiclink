#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# GTY raw symbol path beside the fabric 8b/10b: tx_raw selected by tx_raw_enable in the TX source
# mux, rx_raw taken from the received symbols before the decoders.

from svgdiag import *
from sd_helpers import frac


def build():
    d = Diagram(CANVAS_WIDTH, 320)
    enc  = d.add(Box(170, 30, 120, 50, label="Encoder", fill=COLOR_ACCENT_LIGHT))
    rtx  = d.add(Box(170, 110, 120, 50, label="Raw TX\nsymbols", fill=COLOR_ACCENT))
    mux  = d.add(Box(360, 30, 110, 130, label="TX source\nmux", fill=COLOR_ACCENT_LIGHT))
    dec  = d.add(Box(360, 185, 110, 50, label="Decoders", fill=COLOR_ACCENT_LIGHT))
    rrx  = d.add(Box(360, 255, 110, 50, label="Raw RX\nsymbols", fill=COLOR_ACCENT))
    gt   = d.add(Box(600, 30, 145, 275, label="GTYE4_CHANNEL\n8b/10b bypassed", fill=COLOR_HARDWARE))
    d.add(Arrow((15, enc.cy), (enc, "l"), kind="data", label="sink", label_pos=(90, enc.cy - 10)))
    d.add(Arrow((15, rtx.cy), (rtx, "l"), kind="accent", label="tx_raw", label_pos=(90, rtx.cy - 10)))
    d.add(Arrow((enc, "r"), (mux, "l", frac(mux, enc.cy)), kind="data"))
    d.add(Arrow((rtx, "r"), (mux, "l", frac(mux, rtx.cy)), kind="accent"))
    d.add(Arrow((mux, "r", frac(mux, 95)), (gt, "l", frac(gt, 95)), kind="accent", label="TXDATA",
        label_pos=(535, 85)))
    d.add(Arrow((gt, "l", frac(gt, dec.cy)), (dec, "r"), kind="data", label="RXDATA",
        label_pos=(535, dec.cy - 10)))
    d.add(Arrow((gt, "l", frac(gt, rrx.cy)), (rrx, "r"), kind="accent"))
    d.add(Arrow((dec, "l"), (15, dec.cy), kind="data", label="source", label_pos=(90, dec.cy - 10)))
    d.add(Arrow((rrx, "l"), (15, rrx.cy), kind="accent", label="rx_raw", label_pos=(90, rrx.cy - 10)))
    return d
