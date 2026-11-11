#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB packet: preamble, port/length header word, payload words, optional CRC16 word; fields of the
# header word.

from swb_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    d.add(Text(30, 25, "Packet (32-bit words, first word on the left)", size=14, anchor="start", bold=True))
    bitfield(d, 30, 40, [
        (150, "Preamble\n0x5aa55aa5", COLOR_SHADE),
        (150, "Header\nport, length", COLOR_ACCENT),
        (250, "Payload\nlength / 4 words", COLOR_ACCENT_LIGHT),
        (150, "CRC16\n(with_crc)", COLOR_FILL),
    ], h=50)
    d.add(Text(30, 130, "Header word", size=14, anchor="start", bold=True))
    for x, t in ((30, "31"), (205, "24"), (230, "23"), (555, "8"), (580, "7"), (720, "0")):
        d.add(Text(x, 155, t, size=12, anchor="start" if x in (30, 230, 580) else "end", color="#444444"))
    bitfield(d, 30, 168, [
        (200, "Reserved (0)", COLOR_SHADE),
        (350, "Length in bytes", COLOR_ACCENT),
        (150, "Port", COLOR_ACCENT),
    ], h=40)
    return d
