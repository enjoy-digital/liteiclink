#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Clocking of a Xilinx channel: reference clock pads, IBUFDS_GTE2, PLL, channel, TXOUTCLK and
# RXOUTCLK buffered and returned as the user clocks.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    pads = d.add(Box(15, 95, 95, 50, label="Clock pads", fill=COLOR_HARDWARE))
    ibuf = d.add(Box(135, 95, 120, 50, label="IBUFDS_GTE2"))
    pll  = d.add(Box(280, 95, 80, 50, label="PLL", fill=COLOR_ACCENT_LIGHT))
    ch   = d.add(Box(395, 40, 125, 170, label="Channel", fill=COLOR_ACCENT))
    btx  = d.add(Box(615, 50, 70, 36, label="BUFG"))
    brx  = d.add(Box(615, 160, 70, 36, label="BUFG"))
    d.add(Arrow((pads, "r"), (ibuf, "l"), kind="control"))
    d.add(Arrow((ibuf, "r"), (pll, "l"), kind="control"))
    d.add(Arrow((pll, "r"), (395, 120), kind="control"))
    d.add(Arrow((520, 68), (btx, "l"), kind="control", label="TXOUTCLK"))
    d.add(Arrow((btx, "b"), (520, 112), kind="control", via=[(650, 112)], label="TXUSRCLK", label_pos=(567, 104)))
    d.add(Arrow((520, 178), (brx, "l"), kind="control", label="RXOUTCLK"))
    d.add(Arrow((brx, "t"), (520, 140), kind="control", via=[(650, 140)], label="RXUSRCLK", label_pos=(567, 132)))
    d.add(Arrow((btx, "r"), (745, 68), kind="control", label="tx"))
    d.add(Arrow((brx, "r"), (745, 178), kind="control", label="rx"))
    return d
