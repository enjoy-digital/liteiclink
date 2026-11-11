#
# This file is part of LiteICLink.
#
# Copyright (c) 2017-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Shared helpers of the SerDes wrapper figures (sd_*, s7_*, us_*); no build(): not a figure.

from svgdiag import *


def frac(box, y):
    """Fraction of the height of box at which an horizontal arrow at ordinate y meets its side."""
    return (y - box.y)/box.h


def hfrac(box, x):
    """Fraction of the width of box at which a vertical arrow at abscissa x meets its side."""
    return (x - box.x)/box.w
