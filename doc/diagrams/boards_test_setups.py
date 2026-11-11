#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Closing a SerDes link for a PRBS test: near-end loopback inside the transceiver, or a loopback
# module/cable on the board connector.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(20, 20, 420, 260, title="FPGA", title_pos="bl"))
    gen   = d.add(Box(45, 50, 130, 50, label="PRBS\ngenerator", fill=COLOR_ACCENT_LIGHT))
    chk   = d.add(Box(45, 150, 130, 50, label="PRBS\nchecker", fill=COLOR_ACCENT_LIGHT))
    d.add(Container(240, 30, 180, 240, title="Transceiver lane", title_pos="tl", chamfer=("tr", "bl"), fill=COLOR_PANEL))
    xcvr  = d.add(Box(260, 65, 140, 120, label="Serializer\nand\ndeserializer", fill=COLOR_HARDWARE))
    near  = d.add(Box(260, 210, 140, 40, label="Near-end loopback", fill=COLOR_ACCENT))
    conn  = d.add(Box(480, 75, 110, 100, label="Connector\nSFP, SMA,\nPCIe, SATA", fill=COLOR_HARDWARE))
    loop  = d.add(Box(630, 75, 110, 100, label="Loopback\nmodule\nor cable", fill=COLOR_HARDWARE))
    d.add(Arrow((gen, "r"), (xcvr, "l", 0.1), kind="data"))
    d.add(Arrow((xcvr, "l", 0.9), (chk, "r"), kind="data"))
    d.add(Arrow((xcvr, "r", 0.333), (conn, "l", 0.25), kind="accent", label="TX", label_pos=(460, 98)))
    d.add(Arrow((conn, "l", 0.75), (xcvr, "r", 0.667), kind="accent", label="RX", label_pos=(460, 166)))
    d.add(Arrow((conn, "r", 0.25), (loop, "l", 0.25), kind="accent"))
    d.add(Arrow((loop, "l", 0.75), (conn, "r", 0.75), kind="accent"))
    d.add(Arrow((xcvr, "b", 0.3), (near, "t", 0.3), kind="feedback"))
    d.add(Arrow((near, "t", 0.7), (xcvr, "b", 0.7), kind="feedback"))
    return d
