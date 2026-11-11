#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Word alignment: the received words straddle the K28.5 symbol; slipping the boundary by 15 bits
# puts the symbol at the start of word 0 (bits [9:0]).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 205)
    d.add(Text(105, 35, "bit stream", anchor="end"))
    d.add(Text(105, 95, "words before", anchor="end"))
    d.add(Text(105, 175, "words after", anchor="end"))
    syms = [(120, 50, "D"), (170, 100, "D"), (270, 100, "K28.5"), (370, 100, "D"), (470, 100, "D"),
            (570, 100, "D"), (670, 50, "D")]
    for x, w, l in syms:
        d.add(Box(x, 20, w, 30, label=l, radius=0, fill=COLOR_ACCENT if l == "K28.5" else COLOR_FILL))
    for i, x in enumerate([120, 320, 520]):
        d.add(Box(x, 80, 200, 30, label=f"word {i}", radius=0))
    d.add(Box(120, 160, 150, 30, label="previous", radius=0))
    for i, x in enumerate([270, 470]):
        d.add(Box(x, 160, 200, 30, label=f"word {i}", radius=0, fill=COLOR_ACCENT_LIGHT if i == 0 else COLOR_FILL))
    d.add(Arrow((20, 105), (20, 165), kind="accent"))
    d.add(Text(30, 135, "slip 15 bits", size=FONT_SIZE_SM, anchor="start"))
    return d
