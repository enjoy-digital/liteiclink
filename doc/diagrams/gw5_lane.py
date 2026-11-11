#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# GTR12 lane datapath: fabric 8b/10b, TX source selection and polarity inversion around the raw
# PCS words of the quad lane.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 270)
    d.add(Container(15, 15, 515, 240, title="Fabric (GTR12)", title_pos="br"))
    lane = d.add(Box(615, 40, 130, 170, label="GTR12 lane\nraw PCS", fill=COLOR_HARDWARE))
    # TX row (left to right).
    enc = d.add(Box(35, 45, 120, 56, label="8b/10b\nencoder", fill=COLOR_ACCENT_LIGHT))
    mux = d.add(Box(215, 45, 130, 56, label="Raw, pattern,\nencoder", fill=COLOR_ACCENT_LIGHT))
    txp = d.add(Box(400, 45, 110, 56, label="Polarity\nXOR", fill=COLOR_ACCENT))
    d.add(Arrow((enc, "r"), (mux, "l"), kind="data"))
    d.add(Arrow((mux, "r"), (txp, "l"), kind="data"))
    d.add(Arrow((txp, "r"), (lane, "l", 33/170), kind="data", label="TXDATA"))
    # RX row (right to left).
    rxp = d.add(Box(400, 145, 110, 56, label="Polarity\nXOR", fill=COLOR_ACCENT))
    dec = d.add(Box(215, 145, 130, 56, label="8b/10b decoders\n(logic lookups)", fill=COLOR_ACCENT_LIGHT))
    raw = d.add(Box(35, 145, 120, 56, label="source\ndata, ctrl", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((lane, "l", 133/170), (rxp, "r"), kind="data", label="RXDATA"))
    d.add(Arrow((rxp, "l"), (dec, "r"), kind="data"))
    d.add(Arrow((dec, "l"), (raw, "r"), kind="data"))
    return d
