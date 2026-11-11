#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SerWB native protocol: posted writes acknowledged locally and merged into one request; a read is a
# blocking round trip.

from swb_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    e = Exchange(d, "SoC side core", "Remote side core")
    e.note("write A: acked", side="left", step=26)
    e.note("write A+1: acked", side="left", step=26)
    e.note("write A+2: acked", side="left", step=26)
    e.note("write A+3: acked", side="left", step=36)
    e.msg("Request: 4 writes from A", to="right", kind="accent", note="4 bus writes")
    e.note("read B: waits", side="left", step=34)
    e.msg("Request: 1 read at B", to="right", kind="accent", note="1 bus read", step=44)
    e.msg("Response: status, data", to="left", note="read acked")
    return d
