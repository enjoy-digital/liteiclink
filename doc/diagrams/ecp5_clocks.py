#
# This file is part of LiteICLink.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECP5 DCU clocking: reference clock to the DCU PLL (x M, / D) and to the CDR, DCU word clocks
# (line rate / 20) and the CLKDIVF /2 of the 40-bit mode (line rate / 40).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    ref = d.add(Box(15, 90, 120, 60, label="Reference\nclock", fill=COLOR_HARDWARE))
    d.add(Container(165, 15, 280, 220, title="DCU", title_pos="tl"))
    pll = d.add(Box(185, 90, 100, 60, label="PLL\nx M, / D", fill=COLOR_ACCENT))
    txc = d.add(Box(320, 35, 105, 56, label="TX word\nclock"))
    rxc = d.add(Box(320, 150, 105, 56, label="CDR, RX\nword clock"))
    txd = d.add(Box(500, 35, 100, 56, label="CLKDIVF\n/ 2", fill=COLOR_ACCENT_LIGHT))
    rxd = d.add(Box(500, 150, 100, 56, label="CLKDIVF\n/ 2", fill=COLOR_ACCENT_LIGHT))
    tx  = d.add(Box(655, 35, 90, 56, label="tx", fill=COLOR_ACCENT))
    rx  = d.add(Box(655, 150, 90, 56, label="rx", fill=COLOR_ACCENT))
    d.add(Arrow((ref, "r"), (pll, "l"), kind="control"))
    d.add(Arrow((pll, "t"), (txc, "l"), kind="control"))
    d.add(Arrow((ref, "b"), (rxc, "l"), kind="control", via=[(ref.cx, rxc.cy)]))
    d.add(Arrow((txc, "r"), (txd, "l"), kind="control"))
    d.add(Arrow((rxc, "r"), (rxd, "l"), kind="control"))
    d.add(Arrow((txd, "r"), (tx, "l"), kind="control"))
    d.add(Arrow((rxd, "r"), (rx, "l"), kind="control"))
    d.add(Text(472, 50, "tx_full", size=12, color="#444444"))
    d.add(Text(472, 165, "rx_full", size=12, color="#444444"))
    return d
