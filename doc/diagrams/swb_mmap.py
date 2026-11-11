#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB native MMAP protocol: request and read response payload words, fields of the command and
# status words.

from swb_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    d.add(Text(30, 25, "Request payload (SoC side to remote side)", size=14, anchor="start", bold=True))
    bitfield(d, 30, 40, [
        (180, "Command", COLOR_ACCENT),
        (180, "Word address", COLOR_ACCENT_LIGHT),
        (340, "Write data (count words, writes only)", COLOR_FILL),
    ], h=40)
    d.add(Text(30, 115, "Command word", size=14, anchor="start", bold=True))
    bitfield(d, 30, 130, [
        (280, "Magic 0x5357 [31:16]", COLOR_SHADE),
        (180, "Count - 1 [15:8]", COLOR_ACCENT),
        (110, "Sel [7:4]", COLOR_ACCENT),
        (70,  "[3:1]", COLOR_SHADE),
        (60,  "Op [0]", COLOR_ACCENT),
    ], h=40)
    d.add(Text(30, 205, "Read response payload (remote side to SoC side)", size=14, anchor="start", bold=True))
    bitfield(d, 30, 220, [
        (180, "Status", COLOR_ACCENT),
        (520, "Read data (count words)", COLOR_FILL),
    ], h=40)
    d.add(Text(30, 295, "Status word", size=14, anchor="start", bold=True))
    bitfield(d, 30, 310, [
        (280, "Magic 0x5357 [31:16]", COLOR_SHADE),
        (360, "[15:1]", COLOR_SHADE),
        (60,  "Err [0]", COLOR_ACCENT),
    ], h=40)
    return d
