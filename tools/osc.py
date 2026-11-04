#!/usr/bin/env python3
"""Send OSC to the board's control port (UDP 9000) and prove the mux moved.

    tools/osc.py main                 switch the USB source to Main
    tools/osc.py backup               switch to Backup
    tools/osc.py source 1             same, via /usb/source
    tools/osc.py rate 96000           48000 or 96000 (replug USB afterwards)
    tools/osc.py ping                 wait for /pong
    tools/osc.py /usb/source 1        any address, explicitly
    tools/osc.py --float backup       send the argument as ",f" instead of ",i"
    tools/osc.py --board 169.254.9.200 main

WHY THIS EXISTS AT ALL. firmware/osc.c documents the address list beautifully
and there was no way to exercise it from the repo -- every OSC test ran from a
throwaway script in a temp directory, which is how a control surface ends up
documented but unverified. A desk is the real client; this is the bench stand-in.

WHY IT READS THE COUNTERS BACK. OSC is fire-and-forget UDP: the board does not
acknowledge a switch, so "I sent it" is not "it moved". After a source command
this reads the `U` opcode on 7779 and prints `select`, `switches` and `forced`,
which is the only honest confirmation. `forced` incrementing is not a fault --
it means the candidate offered no frame marker and the mux went anyway, which
is the asked-for behaviour: you do not switch for fun, so when you say go, it
goes. Expect it when you switch to a source that is not streaming.
"""
import argparse
import socket
import time
import struct
import sys

OSC_PORT = 9000
CTRL_PORT = 7779
BOARD_DEFAULT = "169.254.9.200"
TAG_DUAL = 0x55534232      # 'USB2'
TAG_SINGLE = 0x55534231    # 'USB1'

# Shorthands -> (address, needs_argument). The firmware ignores the argument on
# /usb/main and /usb/backup, so they are sent bare: a desk that fires a trigger
# with no type tag at all must work, and that path is only exercised if we also
# send it that way sometimes.
ALIASES = {
    "main":   ("/usb/main",   False),
    "backup": ("/usb/backup", False),
    "source": ("/usb/source", True),
    "rate":   ("/usb/rate",   True),
    "ping":   ("/ping",       False),
}


def _pad4(b: bytes) -> bytes:
    """OSC strings are null-terminated AND padded to 4 bytes INCLUDING the null.

    "/abc" is therefore 8 bytes on the wire, not 5. Getting this wrong makes the
    receiver read the type tag out of the middle of the address, and then every
    message looks malformed -- the same trap firmware/osc.c calls out.
    """
    b = b + b"\0"
    return b + b"\0" * ((4 - len(b) % 4) % 4)


def build(addr: str, args, as_float: bool) -> bytes:
    msg = _pad4(addr.encode())
    if not args:
        # A message with no type tag is legal and some desks send exactly that
        # for a trigger. Send the empty tag "," rather than nothing: it is the
        # more common shape and the firmware accepts both.
        return msg + _pad4(b",")
    tags = ","
    blob = b""
    for a in args:
        if as_float:
            tags += "f"
            blob += struct.pack(">f", float(a))
        else:
            tags += "i"
            blob += struct.pack(">i", int(a))
    return msg + _pad4(tags.encode()) + blob


def mux_state(board: str, timeout: float = 3.0):
    """Read select / switches / forced over the 7779 `U` opcode, or None."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        s.sendto(b"U", (board, CTRL_PORT))
        d, _ = s.recvfrom(512)
        w = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
        if w[0] == TAG_SINGLE:
            return "single"
        if w[0] != TAG_DUAL:
            return None
        return {"select": w[1], "live": w[2], "switches": w[3], "forced": w[10]}
    except Exception:                                      # noqa: BLE001
        return None


def main() -> int:
    p = argparse.ArgumentParser(add_help=True, description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--board", default=BOARD_DEFAULT)
    p.add_argument("--float", dest="as_float", action="store_true",
                   help="send numbers as ',f'; plenty of desks only do floats")
    p.add_argument("--no-verify", action="store_true",
                   help="skip the 7779 read-back after a source command")
    p.add_argument("what", help="main | backup | source | rate | ping | /an/address")
    p.add_argument("args", nargs="*")
    a = p.parse_args()

    if a.what.startswith("/"):
        addr, needs = a.what, bool(a.args)
    elif a.what in ALIASES:
        addr, needs = ALIASES[a.what]
    else:
        print("unknown: %s   (use main|backup|source|rate|ping or /an/address)"
              % a.what, file=sys.stderr)
        return 2

    args = a.args
    if needs and not args:
        print("%s needs an argument" % addr, file=sys.stderr)
        return 2
    if not needs:
        args = []        # the firmware ignores it; do not invent one

    before = None
    is_source = addr in ("/usb/main", "/usb/backup", "/usb/source")
    target = None
    if addr == "/usb/main":
        target = 0
    elif addr == "/usb/backup":
        target = 1
    elif addr == "/usb/source" and args:
        target = 1 if int(args[0]) else 0
    if is_source and not a.no_verify:
        before = mux_state(a.board)

    msg = build(addr, args, a.as_float)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3.0)
    s.sendto(msg, (a.board, OSC_PORT))
    print("sent %s %s  (%d bytes, %s)"
          % (addr, args or "", len(msg), "f" if a.as_float and args else
             ("i" if args else "no arg")))

    if addr == "/ping":
        try:
            d, src = s.recvfrom(512)
            name = d.split(b"\0", 1)[0].decode(errors="replace")
            print("reply %s from %s" % (name, src[0]))
            return 0 if name == "/pong" else 1
        except socket.timeout:
            print("no /pong within 3 s -- is the firmware current and port 9000 bound?",
                  file=sys.stderr)
            return 1

    if before is None:
        return 0
    if before == "single":
        print("  (this bitstream is single-USB: nothing to switch)")
        return 0

    # SETTLE BEFORE READING, OR YOU WILL MISREPORT A GOOD SWITCH. The CSR write
    # lands immediately but the handover itself completes on a frame boundary,
    # so an instant read-back catches `select` already updated while `switches`
    # has not ticked yet. Measured 2026-11-04: a switch back to Main reported
    # "select now Main, handovers +0" and this tool called it a failure. Poll
    # until the counter moves, then stop.
    after = None
    for _ in range(12):                       # up to ~1.8 s
        time.sleep(0.15)
        after = mux_state(a.board)
        if isinstance(after, dict) and after["switches"] != before["switches"]:
            break
    if not isinstance(after, dict):
        print("  could not read the mux back on 7779", file=sys.stderr)
        return 1

    name = ("Main", "Backup")[after["select"] & 1]
    moved = after["switches"] - before["switches"]
    forced = after["forced"] - before["forced"]
    print("  select now %s   live %d   handovers %+d   forced %+d"
          % (name, after["live"], moved, forced))

    already = target is not None and (before["select"] & 1) == target
    if moved == 0 and already:
        print("  already on %s, so there was nothing to hand over." % name)
    elif moved == 0:
        print("  NOTE: handovers did not move and the source was not already "
              "%s -- the command did not land." % name)
    elif forced:
        print("  forced: the candidate offered no frame marker and the mux went "
              "anyway. Expected when switching to a source that is not streaming.")
    if target is not None and (after["select"] & 1) != target:
        print("  MISMATCH: asked for %s, board reports %s"
              % (("Main", "Backup")[target], name), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
