#!/usr/bin/env python3
"""Drive and measure the gateware RX MAC allow-list, over UDP:7779.

The gateware classifies every RX frame whether or not the filter is armed, so
there is a real dry run available before anything is risked:

    tools/rx_gate.py status          # what WOULD be dropped, and at what rate
    tools/rx_gate.py watch 10        # per-second deltas, filter untouched
    tools/rx_gate.py on              # arm -- PROVISIONAL, auto-reverts in 30 s
    tools/rx_gate.py commit          # "networking survived, keep it"
    tools/rx_gate.py off             # back out now

`on` is provisional on purpose. If the allow-list is wrong the board stops
answering, and the packet that would undo it is the one being dropped -- so the
board undoes it by itself unless `commit` arrives first. If you lose contact,
wait 30 s and it comes back. The console ('x') is the other way out, and a MAC
filter cannot lock the UART out.

The reply is a separate, smaller record from the main stats reply, which stays
byte-identical: growing that one from 200 to 208 bytes once killed the port.
Fields are parsed by name against a version tag, never by hand-counted offset.
"""
import socket
import struct
import sys
import time

PORT = 7779
TAG = 0x52584731  # 'RXG1'

FIELDS = [
    "enabled", "pending_revert", "revert_in_ms",
    "match", "nomatch", "discarded",
    "last_drop_hi", "last_drop_lo",
    "writer_errors", "sw_filtered",
]


def query(board, arg=b"", timeout=3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(b"g" + arg, (board, PORT))
    d, _ = s.recvfrom(512)
    if len(d) < 4 * (1 + len(FIELDS)):
        raise ValueError(f"short reply: {len(d)} bytes")
    words = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
    if words[0] != TAG:
        raise ValueError(f"bad tag {words[0]:#010x}; is this firmware current?")
    out = dict(zip(FIELDS, words[1:]))
    hi, lo = out.pop("last_drop_hi"), out.pop("last_drop_lo")
    out["last_drop"] = ":".join(
        f"{b:02x}" for b in (
            (hi >> 24) & 0xFF, (hi >> 16) & 0xFF, (hi >> 8) & 0xFF, hi & 0xFF,
            (lo >> 8) & 0xFF, lo & 0xFF,
        )
    )
    return out


def show(st):
    print(f"  armed          {'YES' if st['enabled'] else 'no'}")
    if st["pending_revert"]:
        print(f"  auto-revert    in {st['revert_in_ms']} ms  "
              f"(run 'commit' to keep it)")
    print(f"  match          {st['match']}")
    print(f"  nomatch        {st['nomatch']}   "
          f"{'(dry run: what arming would drop)' if not st['enabled'] else ''}")
    print(f"  discarded      {st['discarded']}   "
          f"{'(gateware is dropping)' if st['discarded'] else '(gateware dropping nothing yet)'}")
    print(f"  last dropped   {st['last_drop']}")
    print(f"  writer_errors  {st['writer_errors']}   <- the counter being fixed")
    print(f"  sw_filtered    {st['sw_filtered']}   <- software filter, one level up")


def watch(board, secs):
    a = query(board)
    t0 = time.time()
    time.sleep(secs)
    b = query(board)
    dt = time.time() - t0
    print(f"over {dt:.1f}s (armed={'YES' if b['enabled'] else 'no'}):")
    for k in ("match", "nomatch", "discarded", "writer_errors", "sw_filtered"):
        d = b[k] - a[k]
        print(f"  {k:14s} {b[k]:<12} delta {d:+8d}   {d / dt:8.1f}/s")
    print(f"  last dropped   {b['last_drop']}")
    # The whole point, stated plainly rather than left to the reader.
    we = (b["writer_errors"] - a["writer_errors"]) / dt
    if b["enabled"]:
        print(f"\n  => writer_errors {we:.1f}/s while armed"
              f"{'  -- GOOD' if we < 1.0 else '  -- still losing frames, investigate'}")
    else:
        print(f"\n  => baseline writer_errors {we:.1f}/s with the filter OFF")


def main():
    board = "169.254.9.200"
    args = sys.argv[1:]
    if args and args[0].count(".") == 3:
        board = args.pop(0)
    cmd = args[0] if args else "status"

    if cmd == "status":
        show(query(board))
    elif cmd == "watch":
        watch(board, int(args[1]) if len(args) > 1 else 10)
    elif cmd == "on":
        st = query(board, b"1")
        print("armed (provisionally -- commit within 30 s or it reverts):")
        show(st)
    elif cmd == "off":
        print("disarmed:")
        show(query(board, b"0"))
    elif cmd == "commit":
        print("committed:")
        show(query(board, b"c"))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
