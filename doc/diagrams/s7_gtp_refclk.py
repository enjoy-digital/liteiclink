#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# GTP reference clock inputs: dedicated pads into GTREFCLK0, or a fabric clock through the global
# clock network into GTGREFCLK0/1 (refclk_from_fabric).

from svgdiag import *
from sd_helpers import frac


def build():
    d = Diagram(CANVAS_WIDTH, 240)
    pads = d.add(Box(15, 30, 165, 60, label="Reference\nclock pads", fill=COLOR_HARDWARE))
    ibuf = d.add(Box(225, 30, 150, 60, label="IBUFDS_GTE2", fill=COLOR_HARDWARE))
    fpll = d.add(Box(15, 160, 165, 60, label="Fabric clock\n(S7PLL)", fill=COLOR_ACCENT_LIGHT))
    bufg = d.add(Box(225, 160, 150, 60, label="Clock network", fill=COLOR_ACCENT))
    com  = d.add(Box(510, 30, 235, 190, label="GTPE2_COMMON\nPLL0 or PLL1", fill=COLOR_HARDWARE))
    d.add(Arrow((pads, "r"), (ibuf, "l")))
    d.add(Arrow((ibuf, "r"), (com, "l", frac(com, ibuf.cy)), label="GTREFCLK0", label_pos=(442, 50)))
    d.add(Arrow((fpll, "r"), (bufg, "l"), kind="accent"))
    d.add(Arrow((bufg, "r"), (com, "l", frac(com, bufg.cy)), kind="accent", label="GTGREFCLK0/1",
        label_pos=(442, 180)))
    return d
