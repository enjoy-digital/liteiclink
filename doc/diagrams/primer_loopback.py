#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The four loopbacks of a transceiver: near-end PCS and PMA, far-end PMA and PCS.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 275)
    d.add(Container(140, 15, 475, 250, title="Transceiver", title_pos="br"))
    utx  = d.add(Box(15, 40, 110, 44, label="User TX"))
    urx  = d.add(Box(15, 180, 110, 44, label="User RX"))
    tpcs = d.add(Box(190, 40, 130, 44, label="TX PCS", fill=COLOR_ACCENT_LIGHT))
    rpcs = d.add(Box(190, 180, 130, 44, label="RX PCS", fill=COLOR_ACCENT_LIGHT))
    tpma = d.add(Box(440, 40, 130, 44, label="TX PMA", fill=COLOR_ACCENT_LIGHT))
    rpma = d.add(Box(440, 180, 130, 44, label="RX PMA", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((utx, "r"), (tpcs, "l"), kind="data"))
    d.add(Arrow((tpcs, "r"), (tpma, "l"), kind="data"))
    d.add(Arrow((tpma, "r"), (740, 62), kind="data", label="to remote"))
    d.add(Arrow((740, 202), (rpma, "r"), kind="data", label="from remote"))
    d.add(Arrow((rpma, "l"), (rpcs, "r"), kind="data"))
    d.add(Arrow((rpcs, "l"), (urx, "r"), kind="data"))
    d.add(Arrow((tpcs, "b", 0.3), (rpcs, "t", 0.3), kind="accent", label="near-end PCS", label_side="left"))
    d.add(Arrow((rpcs, "t", 0.7), (tpcs, "b", 0.7), kind="accent", label="far-end PCS", label_side="right"))
    d.add(Arrow((tpma, "b", 0.3), (rpma, "t", 0.3), kind="accent", label="near-end PMA", label_side="left"))
    d.add(Arrow((rpma, "t", 0.7), (tpma, "b", 0.7), kind="accent", label="far-end PMA", label_side="right"))
    return d
