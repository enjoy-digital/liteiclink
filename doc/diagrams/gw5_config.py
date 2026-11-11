#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# GTR12 configuration flow: hardware-validated register writes held by the user core, csr_config
# given to GTR12, set_csr in the toolchain script, quad registers; UPAR for runtime accesses.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(15, 15, 300, 220, title="User core", title_pos="bl"))
    sets = d.add(Box(185, 40, 110, 56, label="Validated\nsettings", fill=COLOR_ACCENT_LIGHT))
    cfg  = d.add(Box(185, 130, 110, 56, label="csr_config\ntext", fill=COLOR_ACCENT))
    gtr  = d.add(Box(370, 130, 120, 56, label="GTR12", fill=COLOR_ACCENT))
    tcl  = d.add(Box(370, 40, 120, 56, label="set_csr\n(TCL script)"))
    gw   = d.add(Box(545, 40, 200, 56, label="Gowin toolchain", fill=COLOR_HARDWARE))
    quad = d.add(Box(545, 130, 200, 56, label="Quad registers", fill=COLOR_HARDWARE))
    d.add(Arrow((sets, "b"), (cfg, "t"), kind="data"))
    d.add(Arrow((cfg, "r"), (gtr, "l"), kind="data"))
    d.add(Arrow((gtr, "t"), (tcl, "b"), kind="data"))
    d.add(Arrow((tcl, "r"), (gw, "l"), kind="data"))
    d.add(Arrow((gw, "b"), (quad, "t"), kind="data", label="bitstream", label_side="right"))
    d.add(Arrow((gtr, "r"), (quad, "l"), kind="control", label="UPAR", heads="both"))
    return d
