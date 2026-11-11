#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB link flow control: XOFF/XON characters from the receiving side pause and resume the data of
# the sending side, which sends K28.1 fill characters while paused.

from swb_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 360)
    e = Exchange(d, "Side A", "Side B")
    e.msg("Data words", to="right", kind="accent", note="RX buffer filling")
    e.msg("XOFF (K28.3)", to="left", note="buffer almost full", note_side="right")
    e.msg("K28.1 fill", to="right", note="paused", note_side="left")
    e.msg("XON (K28.4)", to="left", note="buffer below threshold", note_side="right")
    e.msg("Data words", to="right", kind="accent", note="resumed", note_side="left")
    e.msg("XON (K28.4)", to="left", note="sent again periodically", note_side="right")
    return d
