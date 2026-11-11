#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PRBS test: generator, link under test, checker, error counter; below, the LFSR of PRBS7.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 235)
    gen = d.add(Box(25, 25, 110, 50, label="LFSR\ngenerator", fill=COLOR_ACCENT))
    lnk = d.add(Box(175, 25, 110, 50, label="Link\nunder test", fill=COLOR_HARDWARE))
    chk = d.add(Box(325, 25, 110, 50, label="LFSR\nchecker", fill=COLOR_ACCENT))
    cnt = d.add(Box(475, 25, 110, 50, label="Error\ncounter", fill=COLOR_ACCENT_LIGHT))
    reg = d.add(Box(625, 25, 110, 50, label="Register"))
    d.add(Arrow((gen, "r"), (lnk, "l"), kind="data"))
    d.add(Arrow((lnk, "r"), (chk, "l"), kind="data"))
    d.add(Arrow((chk, "r"), (cnt, "l"), kind="control"))
    d.add(Arrow((cnt, "r"), (reg, "l"), kind="control"))
    # PRBS7 LFSR.
    d.add(Text(320, 112, "LFSR of PRBS7", italic=True))
    cs = [d.add(Box(180 + 40*i, 130, 40, 30, label=str(i + 1), radius=0, fill=COLOR_ACCENT_LIGHT, size=13))
          for i in range(7)]
    xor = d.add(Box(290, 180, 50, 36, label="XOR"))
    d.add(Arrow((cs[6], "r"), (560, 145), kind="data", label="sequence"))
    d.add(Arrow((cs[5], "b"), (xor, "r", 0.3), kind="control", via=[(400, 191)]))
    d.add(Arrow((cs[6], "b"), (xor, "r", 0.8), kind="control", via=[(440, 209)]))
    d.add(Arrow((xor, "l"), (cs[0], "b"), kind="control", via=[(200, 198)]))
    return d
