#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# User clocks of a Xilinx SerDes wrapper: TXOUTCLK and RXOUTCLK through clock buffers (and a
# divider or a PLL on TX when the buffers are bypassed) to the tx and rx clock domains.

from svgdiag import *
from sd_helpers import frac


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    ref  = d.add(Box(15, 95, 110, 60, label="Reference\nclock", fill=COLOR_HARDWARE))
    gt   = d.add(Box(175, 30, 150, 190, label="Transceiver\nchannel", fill=COLOR_HARDWARE))
    txb  = d.add(Box(400, 30, 170, 70, label="BUFG(_GT), divider\nor PLL", fill=COLOR_ACCENT))
    rxb  = d.add(Box(400, 150, 170, 70, label="BUFG or\nBUFG_GT", fill=COLOR_ACCENT_LIGHT))
    tx   = d.add(Box(630, 30, 115, 70, label="tx domain", fill=COLOR_ACCENT_LIGHT))
    rx   = d.add(Box(630, 150, 115, 70, label="rx domain", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((ref, "r"), (gt, "l", frac(gt, ref.cy))))
    d.add(Arrow((gt, "r", frac(gt, txb.cy)), (txb, "l"), kind="accent", label="TXOUTCLK",
        label_pos=(362, txb.cy - 10)))
    d.add(Arrow((gt, "r", frac(gt, rxb.cy)), (rxb, "l"), label="RXOUTCLK", label_pos=(362, rxb.cy - 8)))
    d.add(Arrow((txb, "r"), (tx, "l"), kind="accent"))
    d.add(Arrow((rxb, "r"), (rx, "l")))
    return d
