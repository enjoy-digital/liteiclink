#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A serial link: parallel word, serializer and its PLL, line, CDR, deserializer, parallel word.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 205)
    d.add(Container(10, 10, 250, 185, title="TX", title_pos="bl", title_size=15))
    d.add(Container(340, 10, 410, 185, title="RX", title_pos="bl", title_size=15))
    pll  = d.add(Box(135, 25, 105, 36, label="PLL", fill=COLOR_ACCENT_LIGHT))
    win  = d.add(Box(20, 90, 95, 44, label="Word"))
    ser  = d.add(Box(135, 90, 105, 44, label="Serializer", fill=COLOR_ACCENT))
    cdr  = d.add(Box(355, 90, 90, 44, label="CDR", fill=COLOR_ACCENT))
    des  = d.add(Box(470, 90, 115, 44, label="Deserializer", fill=COLOR_ACCENT))
    wout = d.add(Box(615, 90, 110, 44, label="Word"))
    d.add(Arrow((pll, "b"), (ser, "t"), kind="control"))
    d.add(Arrow((win, "r"), (ser, "l"), kind="data"))
    d.add(Arrow((ser, "r"), (cdr, "l"), kind="accent", label="serial line"))
    d.add(Arrow((cdr, "r"), (des, "l"), kind="data"))
    d.add(Arrow((des, "r"), (wout, "l"), kind="data"))
    d.add(Arrow((cdr, "b"), (wout, "b"), kind="feedback", via=[(400, 165), (670, 165)]))
    d.add(Text(560, 177, "recovered clock", size=FONT_SIZE_SM))
    return d
