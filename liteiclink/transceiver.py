# Retro-compatibility 2020-10-14 (liteiclink.transceiver renamed to liteiclink.serdes).
import sys
import warnings

from liteiclink import serdes

warnings.warn(
    "liteiclink.transceiver is deprecated and will be removed, please use liteiclink.serdes.",
    FutureWarning,
    stacklevel=2,
)

sys.modules[__name__] = serdes
