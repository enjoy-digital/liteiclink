#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB link initialization: the master resets the slave with an idle line, each side calibrates on
# the commas of the other (the master sends commas until it is ready), then both stop sending commas.

from swb_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    e = Exchange(d, "Master PHY", "Slave PHY")
    e.msg("Idle (line at zero)", to="right", note="slave held in reset")
    e.msg("K28.5 commas", to="right", kind="accent", note="slave calibrates")
    e.msg("Idle (line at zero)", to="left")
    e.msg("K28.5 commas", to="left", kind="accent", note="master calibrates")
    e.msg("K28.5 commas, until master ready", to="right", kind="accent")
    e.msg("Idle words", to="right", note="master ready", note_side="left")
    e.msg("Idle words", to="left", note="slave ready", note_side="right")
    return d
