"""The board's management address, in one place.

Dante Controller can now SET this address (firmware/dante_info.c, 0x13 write),
so a literal baked into a dozen tools is a trap: the next change from the
controller silently points every one of them at an address nobody is listening
on. 2026-11-07 moved the board from .200 to .201 from Controller, and eighteen
defaults across twelve files went stale in one step.

So: one constant, overridable without editing anything.

    DANTE_BOARD=169.254.9.77 tools/stats.py

Every tool still takes the address as an argument; this is only the default.
flash.sh honours the same variable with the same fallback.
"""
import os

BOARD = os.environ.get("DANTE_BOARD", "169.254.9.201")
