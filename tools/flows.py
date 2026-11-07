#!/usr/bin/env python3
"""Read the board's per-flow table over UDP 7779, opcode 'f'.

    tools/flows.py                    one snapshot
    tools/flows.py --watch 15         re-read every 15 s until Ctrl-C
    tools/flows.py --board 169.254.9.200

WHY THIS EXISTS. The 'f' reply has been in dstats.c for months and nothing in
the repo read it -- every measurement of it was a throwaway script in a temp
directory, which is how `age_ms` ended up being quoted from memory in one
argument and re-derived from scratch in the next. The numbers it carries are
the ones that decide whether a flow is stale, so they need a tool that is
committed next to the firmware that produces them.

WHAT age_ms ACTUALLY MEANS. dante_tx.c writes `last_ms` in the BIND PATHS only,
and every flow-control request runs through dante_tx_bind_unicast, so age_ms is
the time since the receiver last asked for this flow -- the keepalive age. An
age that climbs monotonically means the receiver is not refreshing. Measured on
this bench 2026-11-05: one flow to :14337 climbed from 1147813 ms to 1207821 ms
over 60 s, exactly +15002 ms per 15 s poll, i.e. zero refreshes in 19 minutes.

WHAT ka COLUMNS MEAN. Datagrams that peer sent to OUR media port 4321, counted
by media_rx in dante_tx.c. OBSERVATION ONLY -- nothing expires a flow on them.
A device reported to keepalive on 4321 shows a rising `ka` and a small `ka_age`;
a device that never sends there shows `ka 0  ka_age never`, and for that device
only an explicit 0x0101 stop or slot pressure will ever free its context.

DO NOT WIRE A TIMEOUT TO THESE NUMBERS WITHOUT MEASURING EVERY DEVICE FIRST.
dante_tx_expire() now releases a flow after 5 silent seconds, but ONLY one that
has ever sent to 4321 (`ka` non-zero) -- a device that never does is left alone
for exactly the reason the older timeouts were removed: 300 s and 600 s each
silenced a flow that was still playing audio.

WHY-COUNTERS (FLW3). `disables` and `rebinds` say how often, never by what, and
the console on this bench is dead. 2026-11-05: two receivers lost audio TOGETHER
for ~10 s every 3-5 minutes and both bindings jumped afterwards. That shape has
two very different causes -- the talker going off (every receiver stops, because
`want` in dante_tx.c needs PTP settled AND a bound flow AND the rate ready), or
individual flows being released (only those receivers stop). The trailer below
tells them apart:

    off_unlock  the talker went off because PTP was not settled   <- clock
    off_noflow  ...because no flow was bound any more             <- consequence
    off_rate    ...because mcr_dante had not reached rate
    ub_stop     a receiver sent 0x0101                            <- receiver
    ub_expire   keepalives stopped for 5 s                        <- our expiry
    ub_drop     dante_tx_drop_all: a latency or rate change
    ub_arc      ARC 0x2202, or any caller that did not say
    ptp_unlocks lock -> unlock transitions; each one holds the talker off for
                PTP_SETTLE_MS (4 s) after relock, so each is >=4 s of silence
                for EVERY receiver at once
    ptp_steps   absolute clock steps; each re-anchors the media clock
    rx_ts_resyncs  stale RX-timestamp-ring entries discarded by dispatch_rx.
                Each resync shifts the ring, so a timestamp can end up paired
                with a DIFFERENT frame -- and the error is then the gap between
                those two frames, ~17 us for Sync -> FollowUp. Measured
                2026-11-05: six consecutive offset samples at -15 to -17 us,
                1.5 s after a lost FollowUp, tripped the 5 us unlock threshold.
    rx_pops     pops issued, for the ratio

off_noflow rising WITH ub_stop/ub_expire is the benign order (receivers left,
so the talker had nothing to send). off_unlock rising is the opposite story:
we silenced them.
"""
import argparse
import socket
import struct
import sys
import time
from dante_board import BOARD as _BOARD

CTRL_PORT = 7779
BOARD_DEFAULT = _BOARD
TAG_FLW3 = 0x464C5733          # stride 11 + a 7-word why-counter trailer
TAG_FLW2 = 0x464C5732          # per-flow stride 11 words
TAG_FLW1 = 0x464C5731          # older firmware: stride 8, no cookie/ka columns
NEVER = 0xFFFFFFFF
WHY = ["off_unlock", "off_noflow", "off_rate",
       "ub_arc", "ub_stop", "ub_expire", "ub_drop",
       "ptp_unlocks", "ptp_steps", "rx_ts_resyncs", "rx_pops"]


def fetch(board: str, timeout: float = 3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(b"f", (board, CTRL_PORT))
    d, _ = s.recvfrom(2048)
    if len(d) < 8:
        raise ValueError("short reply: %d bytes" % len(d))
    w = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
    tag, n = w[0], w[1]
    if tag == TAG_FLW1:
        stride, have_ka = 8, False
    elif tag in (TAG_FLW2, TAG_FLW3):
        stride, have_ka = 11, True
    else:
        raise ValueError("unexpected tag %#010x -- not an 'f' reply" % tag)

    out = []
    for f in range(n):
        o = 2 + f * stride
        if o + stride > len(w):
            break
        in_use = (w[o] >> 24) & 0xFF
        rec = {
            "f": f,
            "in_use": in_use,
            "nslots": (w[o] >> 16) & 0xFF,
            "fpp": (w[o] >> 8) & 0xFF,
            "mcast": w[o] & 0xFF,
            "dst": ".".join(str((w[o + 1] >> sh) & 0xFF) for sh in (24, 16, 8, 0)),
            "dport": w[o + 2],
            "age_ms": w[o + 3],
        }
        if have_ka:
            rec.update(cookie=w[o + 4], ka_rx=w[o + 5], ka_age_ms=w[o + 6])
            ch = w[o + 7 : o + 11]
        else:
            rec.update(cookie=None, ka_rx=None, ka_age_ms=None)
            ch = w[o + 4 : o + 8]
        chans = []
        for word in ch:
            chans += [word >> 16, word & 0xFFFF]
        rec["chans"] = chans[: rec["nslots"]] if in_use else []
        out.append(rec)
    why = None
    if tag == TAG_FLW3:
        o = 2 + n * stride
        if o + len(WHY) <= len(w):
            why = dict(zip(WHY, w[o : o + len(WHY)]))
    return out, have_ka, why


def _age(ms):
    if ms is None:
        return "-"
    if ms == NEVER:
        return "never"
    return "%.1f s" % (ms / 1000.0)


def show_why(why, prev):
    if not why:
        return
    # Print the counters, and the DELTA next to them. A total of 36 disables
    # says nothing about whether it is still happening; a delta does.
    parts = []
    for k in WHY:
        d = why[k] - prev[k] if prev and k in prev else 0
        parts.append("%s=%d%s" % (k, why[k], " (+%d)" % d if d else ""))
    print("  why: " + "  ".join(parts))


def show(rows, have_ka):
    live = [r for r in rows if r["in_use"]]
    if not live:
        print("no flows bound")
        return
    hdr = "  f  kind       destination            slots fpp  age"
    if have_ka:
        hdr += "        handle    ka   ka_age"
    print(hdr)
    for r in live:
        line = "  %d  %-9s  %-21s  %2d   %3d  %-10s" % (
            r["f"], "multicast" if r["mcast"] else "unicast",
            "%s:%d" % (r["dst"], r["dport"]), r["nslots"], r["fpp"],
            _age(r["age_ms"]))
        if have_ka:
            # A multicast flow is created once by ARC 0x2201 and nothing ever
            # refreshes it, so its age is meaningless by construction -- say so
            # rather than inviting someone to read it as staleness.
            line += "  %d/%-6d  %-4s %s" % (
                r["f"], r["cookie"], r["ka_rx"], _age(r["ka_age_ms"]))
        print(line)
        print("       channels %s%s" % (
            r["chans"],
            "   (age is not staleness: multicast is never refreshed)"
            if r["mcast"] else ""))


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--board", default=BOARD_DEFAULT)
    p.add_argument("--watch", type=float, metavar="SEC",
                   help="re-read every SEC seconds; shows whether age CLIMBS, "
                        "which is the whole question")
    a = p.parse_args()

    prev = {}
    prev_why = None
    while True:
        try:
            rows, have_ka, why = fetch(a.board)
        except socket.timeout:
            # Under watch this must not be fatal: a missed reply is exactly what
            # happens during the event being hunted, and exiting there throws
            # away the rest of the run.
            print("no reply from %s:%d" % (a.board, CTRL_PORT), file=sys.stderr)
            if not a.watch:
                return 1
            time.sleep(a.watch)
            continue
        except ValueError as e:
            print(str(e), file=sys.stderr)
            return 1
        if a.watch:
            print("--- %s" % time.strftime("%H:%M:%S"))
        show(rows, have_ka)
        show_why(why, prev_why)
        if why:
            prev_why = dict(why)
        for r in rows:
            if not r["in_use"]:
                continue
            was = prev.get(r["f"])
            if was is not None and r["age_ms"] < was:
                print("       flow %d REFRESHED (age fell %d -> %d ms)"
                      % (r["f"], was, r["age_ms"]))
            prev[r["f"]] = r["age_ms"]
        if not a.watch:
            return 0
        time.sleep(a.watch)


if __name__ == "__main__":
    sys.exit(main())
