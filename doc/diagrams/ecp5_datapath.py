#
# This file is part of LiteICLink.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECP5 DCU datapath with data_width=40: fabric 8b/10b, gearboxes and symbol aligner around the
# 20-bit raw 10BSER bus of the DCU channel.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 270)
    d.add(Container(15, 15, 515, 240, title="Fabric (SerDesECP5)", title_pos="br"))
    dcu = d.add(Box(615, 40, 130, 170, label="DCU\nchannel\n10BSER", fill=COLOR_HARDWARE))
    # TX row (left to right).
    enc = d.add(Box(35, 45, 120, 56, label="8b/10b\nencoder", fill=COLOR_ACCENT_LIGHT))
    mux = d.add(Box(215, 45, 130, 56, label="Square, pattern,\nPRBS, raw", fill=COLOR_ACCENT_LIGHT))
    txg = d.add(Box(400, 45, 110, 56, label="TX gearbox", fill=COLOR_ACCENT))
    d.add(Arrow((enc, "r"), (mux, "l"), kind="data"))
    d.add(Arrow((mux, "r"), (txg, "l"), kind="data", label="40 bits"))
    d.add(Arrow((txg, "r"), (dcu, "l", 33/170), kind="data", label="20 bits"))
    # RX row (right to left).
    rxg = d.add(Box(400, 145, 110, 56, label="RX gearbox", fill=COLOR_ACCENT))
    sal = d.add(Box(215, 145, 130, 56, label="Symbol\naligner", fill=COLOR_ACCENT))
    dec = d.add(Box(35, 145, 120, 56, label="8b/10b\ndecoders", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((dcu, "l", 133/170), (rxg, "r"), kind="data", label="20 bits"))
    d.add(Arrow((rxg, "l"), (sal, "r"), kind="data", label="40 bits"))
    d.add(Arrow((sal, "l"), (dec, "r"), kind="data"))
    return d
