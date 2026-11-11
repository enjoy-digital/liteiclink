#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# TX de-emphasis: a bit pattern sent at full swing, then with the repeated bits lowered.

from svgdiag import *

BITS  = [0, 1, 1, 1, 0, 1, 0, 0]
X0, W = 190, 65


def wave(d, yc, deemph):
    pts, prev = [], None
    for i, b in enumerate(BITS):
        full = prev is None or b != prev
        amp  = 22 if (full or not deemph) else 11
        y    = yc - amp if b else yc + amp
        x    = X0 + i*W
        if pts and abs(pts[-1][1] - y) > 0.5:
            pts.append((x, y))
        elif not pts:
            pts.append((x, y))
        pts.append((x + W, y))
        prev = b
    d.add(Bus(pts, kind="data", color=COLOR_CYAN if deemph else None))


def build():
    d = Diagram(CANVAS_WIDTH, 225)
    d.add(Text(175, 35, "bits", anchor="end"))
    d.add(Text(175, 100, "full swing", anchor="end"))
    d.add(Text(175, 175, "de-emphasis", anchor="end"))
    for i, b in enumerate(BITS):
        d.add(Box(X0 + i*W, 20, W, 30, label=str(b), radius=0, size=13))
    wave(d, 100, False)
    wave(d, 175, True)
    d.add(Bus([(X0, 100), (X0 + 8*W, 100)], dash="2,4"))
    d.add(Bus([(X0, 175), (X0 + 8*W, 175)], dash="2,4"))
    return d
