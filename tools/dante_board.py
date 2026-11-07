"""The board's management address, in one place.

Dante Controller can now SET this address (firmware/dante_info.c, 0x13 write),
so a literal baked into a dozen tools is a trap: the next change from the
controller silently points every one of them at an address nobody is listening
on. 2026-11-07 moved the board from .200 to .201 from Controller and back again,
and the first move left eighteen stale defaults across twelve files in one step.

The fallback is the DOCUMENTED default, 169.254.9.200 -- the address the README,
the evidence files and every captured exchange refer to. It is not "wherever the
board happens to be": when Controller moves it, set DANTE_BOARD rather than
editing this line, so the repo keeps meaning one thing.

So: one constant, overridable without editing anything.

    DANTE_BOARD=169.254.9.77 tools/stats.py

Every tool still takes the address as an argument; this is only the default.
flash.sh honours the same variable with the same fallback.
"""
import os

BOARD = os.environ.get("DANTE_BOARD", "169.254.9.200")
