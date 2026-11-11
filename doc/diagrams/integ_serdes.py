#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A SerDes wrapper in a LiteX SoC: reference clock, PLL, wrapper, user logic in tx/rx domains,
# registers on the CSR bus.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(150, 20, 590, 290, title="LiteX SoC", title_pos="br"))
    ref   = d.add(Box(20, 60, 100, 50, label="Reference\nclock", fill=COLOR_HARDWARE))
    pads  = d.add(Box(20, 187, 100, 70, label="TX, RX\npads", fill=COLOR_HARDWARE))
    pll   = d.add(Box(180, 60, 120, 50, label="PLL\nCPLL, QPLL", fill=COLOR_ACCENT_LIGHT))
    serd  = d.add(Box(350, 60, 140, 200, label="SerDes\nwrapper", fill=COLOR_ACCENT))
    txl   = d.add(Box(570, 60, 140, 50, label="User TX logic"))
    rxl   = d.add(Box(570, 135, 140, 50, label="User RX logic"))
    csr   = d.add(Box(570, 210, 140, 50, label="CSR bus"))
    d.add(Arrow((ref, "r"), (pll, "l"), kind="control"))
    d.add(Arrow((pll, "r"), (serd, "l", 0.125), kind="control"))
    d.add(Arrow((pads, "r"), (serd, "l", 0.81), kind="data", heads="both"))
    d.add(Arrow((txl, "l"), (serd, "r", 0.125), kind="accent", label="sink, tx"))
    d.add(Arrow((serd, "r", 0.5), (rxl, "l"), kind="accent", label="source, rx"))
    d.add(Arrow((csr, "l"), (serd, "r", 0.875), kind="control", heads="both", label="registers"))
    return d
