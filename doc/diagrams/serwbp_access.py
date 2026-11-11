#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Accesses over SerWB, time going down: a posted write, then a blocking read.

from svgdiag import *

CPU, SOC, REM = 120, 380, 640


def build():
    d = Diagram(CANVAS_WIDTH, 345)
    for x, l in ((CPU, "CPU"), (SOC, "SoC side"), (REM, "Remote side")):
        d.add(Box(x - 60, 10, 120, 36, label=l, fill=COLOR_ACCENT_LIGHT if x != CPU else COLOR_FILL))
        d.add(Bus([(x, 46), (x, 335)], dash="4,3"))
    d.add(Arrow((CPU, 85), (SOC, 85), label="write"))
    d.add(Arrow((SOC, 115), (CPU, 115), label="ack"))
    d.add(Arrow((SOC, 145), (REM, 145), kind="accent", label="write request"))
    d.add(Text(REM + 10, 170, "bus write", size=FONT_SIZE_SM, anchor="start"))
    d.add(Arrow((CPU, 215), (SOC, 215), label="read"))
    d.add(Arrow((SOC, 245), (REM, 245), kind="accent", label="read request"))
    d.add(Text(REM + 10, 265, "bus read", size=FONT_SIZE_SM, anchor="start"))
    d.add(Arrow((REM, 285), (SOC, 285), kind="accent", label="read response"))
    d.add(Arrow((SOC, 315), (CPU, 315), label="ack and data"))
    d.add(Text(CPU - 10, 265, "CPU waits", size=FONT_SIZE_SM, anchor="end"))
    return d
