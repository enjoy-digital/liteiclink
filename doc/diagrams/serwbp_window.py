#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB sampling window scan: delay taps, the taps where the comma pattern is received (min to
# max), and the centre where the delay is set.

from svgdiag import *

X0, W, N = 60, 20, 32
MIN, MAX = 9, 22


def build():
    d = Diagram(CANVAS_WIDTH, 175)
    cx = lambda i: X0 + i*W + W/2
    for i in range(N):
        d.add(Box(X0 + i*W, 70, W, 30, radius=0, fill=COLOR_ACCENT if MIN <= i <= MAX else COLOR_FILL))
    d.add(Bus([(X0 + MIN*W, 55), (X0 + (MAX + 1)*W, 55)], color=COLOR_CYAN))
    d.add(Text((X0 + MIN*W + X0 + (MAX + 1)*W)/2, 40, "pattern received", size=FONT_SIZE_SM))
    d.add(Text((X0 + X0 + MIN*W)/2, 40, "no pattern", size=FONT_SIZE_SM))
    d.add(Text((X0 + (MAX + 1)*W + X0 + N*W)/2, 40, "no pattern", size=FONT_SIZE_SM))
    centre = MIN + (MAX - MIN)//2
    for i, t in ((0, "0"), (N - 1, "31")):
        d.add(Text(cx(i), 115, t, size=FONT_SIZE_SM))
    for i, t in ((MIN, "min"), (centre, "centre"), (MAX, "max")):
        d.add(Bus([(cx(i), 102), (cx(i), 125)], color=COLOR_CYAN if t == "centre" else None))
        d.add(Text(cx(i), 140, t, size=FONT_SIZE_SM))
    d.add(Mark(cx(centre), 85))
    d.add(Text(cx(N//2), 162, "delay taps", size=FONT_SIZE_SM, italic=True))
    return d
