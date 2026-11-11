#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Upstream dispatching of the SerWB core: received packets go to the endpoint of their port, packets
# for unknown ports to a drop endpoint.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    dpk = d.add(Box(15, 115, 130, 60, label="Depacketizer", fill=COLOR_ACCENT_LIGHT))
    dsp = d.add(Box(200, 115, 130, 60, label="Dispatcher\n(port field)", fill=COLOR_ACCENT))
    d.add(Arrow((dpk, "r"), (dsp, "l"), kind="data"))
    ports = [
        ("Port 0: bus protocol", COLOR_ACCENT_LIGHT),
        ("Port 1: SerIO",        COLOR_ACCENT_LIGHT),
        ("Port n: user endpoint", COLOR_ACCENT_LIGHT),
        ("Other ports: drop",    COLOR_SHADE),
    ]
    for i, (label, fill) in enumerate(ports):
        b = d.add(Box(470, 25 + 65*i, 230, 45, label=label, fill=fill))
        d.add(Arrow((dsp, "r"), (b, "l"), kind="data", via=[(400, 145), (400, b.cy)] if abs(b.cy - 145) > 0.5 else ()))
    return d
