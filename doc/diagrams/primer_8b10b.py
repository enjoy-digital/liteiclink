#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# 8b/10b: the byte (bits A to H) is coded by 5b/6b and 3b/4b into the symbol abcdei fghj; K28.5
# with negative running disparity and its comma.

from svgdiag import *


def cells(d, x, y, w, labels, fill=COLOR_FILL, fills=None):
    out = []
    for i, l in enumerate(labels):
        f = fills[i] if fills else fill
        out.append(d.add(Box(x + i*w, y, w, 30, label=l, radius=0, fill=f, size=13)))
    return out


def build():
    d = Diagram(CANVAS_WIDTH, 215)
    d.add(Text(165, 45, "byte", anchor="end"))
    d.add(Text(165, 110, "coders", anchor="end"))
    d.add(Text(165, 175, "symbol", anchor="end"))
    lo = cells(d, 180, 30, 30, list("ABCDE"))
    hi = cells(d, 350, 30, 30, list("FGH"))
    c5 = d.add(Box(180, 90, 150, 40, label="5b/6b", fill=COLOR_ACCENT))
    c3 = d.add(Box(350, 90, 90, 40, label="3b/4b", fill=COLOR_ACCENT))
    cells(d, 180, 160, 25, list("abcdei"), fill=COLOR_ACCENT_LIGHT)
    cells(d, 350, 160, 22.5, list("fghj"), fill=COLOR_ACCENT_LIGHT)
    d.add(Arrow((lo[2], "b"), (c5, "t"), kind="data"))
    d.add(Arrow((hi[1], "b"), (c3, "t"), kind="data"))
    d.add(Arrow((c5, "b"), (255, 160), kind="data"))
    d.add(Arrow((c3, "b"), (395, 160), kind="data"))
    # K28.5, RD-.
    d.add(Text(610, 45, "K28.5, RD-"))
    bits = "0011111010"
    for i, (n, b) in enumerate(zip("abcdeifghj", bits)):
        d.add(Text(493 + i*26, 82, n, size=13))
    cells(d, 480, 95, 26, list(bits), fills=[COLOR_ACCENT if i < 7 else COLOR_FILL for i in range(10)])
    d.add(Bus([(480, 140), (662, 140)]))
    d.add(Text(571, 155, "comma", size=FONT_SIZE_SM))
    return d
