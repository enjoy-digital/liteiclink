#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB core of one side: bus protocol on port 0, port arbiter and dispatcher, packetizer and
# depacketizer, TX and RX buffers in front of the PHY.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(15, 15, 590, 260, title="SERWBCore", title_pos="br"))
    mm  = d.add(Box(35, 60, 110, 140, label="Bus protocol\n(MMAP)\nport 0", fill=COLOR_ACCENT))
    arb = d.add(Box(185, 40, 110, 50, label="Arbiter", fill=COLOR_ACCENT_LIGHT))
    pkt = d.add(Box(325, 40, 115, 50, label="Packetizer", fill=COLOR_ACCENT_LIGHT))
    txf = d.add(Box(470, 40, 110, 50, label="TX buffer"))
    dsp = d.add(Box(185, 170, 110, 50, label="Dispatcher", fill=COLOR_ACCENT_LIGHT))
    dpk = d.add(Box(325, 170, 115, 50, label="Depacketizer", fill=COLOR_ACCENT_LIGHT))
    rxf = d.add(Box(470, 170, 110, 50, label="RX buffer"))
    phy = d.add(Box(650, 40, 95, 180, label="SERWBPHY", fill=COLOR_HARDWARE))
    d.add(Arrow((mm, "r", 25/140), (arb, "l"), kind="data"))
    d.add(Arrow((arb, "r"), (pkt, "l"), kind="data"))
    d.add(Arrow((pkt, "r"), (txf, "l"), kind="data"))
    d.add(Arrow((txf, "r"), (phy, "l", 25/180), kind="data"))
    d.add(Arrow((phy, "l", 155/180), (rxf, "r"), kind="data"))
    d.add(Arrow((rxf, "l"), (dpk, "r"), kind="data"))
    d.add(Arrow((dpk, "l"), (dsp, "r"), kind="data"))
    d.add(Arrow((dsp, "l"), (mm, "r", 135/140), kind="data"))
    return d
