#!/usr/bin/env python3
"""Dual-USB A/B source: select and report, over UDP:7779.

    tools/usbsrc.py                 # status
    tools/usbsrc.py main            # request Main   (USB1)
    tools/usbsrc.py backup          # request Backup (USB2)
    tools/usbsrc.py watch 60        # poll status for 60 s

THE SWITCH IS FRAME-ALIGNED IN GATEWARE. A request sets `select`; `live` follows
only when the candidate source presents a channel-0-of-frame marker, so the
packetizer always receives the new source starting at channel 0. A blind
mid-frame handover would hand the ring a partial 48-channel frame and rotate
every channel permanently -- the "48ch corrupt after a while" failure recorded
in dante_packetizer.py.

So `select != live` for a few microseconds is NORMAL. `select != live`
PERSISTENTLY means the candidate is not delivering frames, and the handover is
correctly refusing to complete. That is the gateware protecting you, not a bug.

CHECK THE BACKUP IS ALIVE BEFORE SWITCHING. rx_beats/ep_out for the Backup must
be climbing at ~9.216 MB/s (48 ch x 4 B x 48 kHz). A hot standby you have not
measured is a standby on trust, and switching to a dead one trades working audio
for silence.
"""
import socket
import struct
import sys
import time

PORT = 7779
BOARD_DEFAULT = "169.254.9.200"
TAG_DUAL   = 0x55534232   # 'USB2' - dual-USB build
TAG_SINGLE = 0x55534231   # 'USB1' - built without USB2=1

FIELDS = ["select", "live", "switches",
          "main_rx_beats", "main_ep_out", "main_overflow",
          "bkp_rx_beats", "bkp_ep_out", "bkp_overflow"]
# 48 ch x 4 B x fs. WAS HARDCODED OP 48 kHz, en op een 96 kHz-bord meldde dit
# "+116% vs nominal" voor een stroom die 8% boven nominaal lag -- een getal dat
# paniek suggereert waar niets aan de hand is. De rate komt uit de 'R'-opcode
# (firmware/dstats.c); faalt die, dan val je terug op 48 kHz zoals voorheen.
_NOM = 48 * 4 * 48000


def _nominal():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.sendto(b"R", (BOARD_DEFAULT, PORT))
        d, _ = s.recvfrom(512)
        fs = struct.unpack(">I", d[4:8])[0]
        if fs in (48000, 96000):
            return 48 * 4 * fs, fs
    except Exception:                                 # noqa: BLE001
        pass
    return 48 * 4 * 48000, 48000


def query(board, arg=b"", timeout=3.0, tries=4):
    last = None
    for _ in range(tries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(timeout)
            s.sendto(b"U" + arg, (board, PORT))
            d, _ = s.recvfrom(512)
            w = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
            if w[0] == TAG_SINGLE:
                return None
            if w[0] != TAG_DUAL:
                raise ValueError(f"bad tag {w[0]:#010x}; firmware current?")
            return dict(zip(FIELDS, w[1:]))
        except Exception as e:                      # noqa: BLE001
            last = e
    raise last


def name(v):
    return "Backup(USB2)" if v else "Main(USB1)"


def show(board):
    global _NOM
    _NOM, _fs = _nominal()
    print("  rate       %d Hz   (nominaal %d B/s per bron)" % (_fs, _NOM))
    a = query(board)
    if a is None:
        print("  this bitstream was built WITHOUT dual USB.")
        print("  rebuild with:  USB2=1 ./build.sh --seed <n>")
        return 2
    time.sleep(2.0)
    b = query(board)
    print(f"  select     {a['select']} = {name(a['select'])}   (requested)")
    print(f"  live       {a['live']} = {name(a['live'])}   (actually feeding)")
    if a["select"] != a["live"]:
        print("    -> HANDOVER PENDING. Normal for microseconds. If it persists,")
        print("       the candidate is not delivering frames and the gateware is")
        print("       correctly refusing to switch mid-frame.")
    print(f"  handovers  {a['switches']}")
    for tag, pfx in (("Main  ", "main"), ("Backup", "bkp")):
        rx = ((b[f"{pfx}_rx_beats"] - a[f"{pfx}_rx_beats"]) & 0xffffffff) / 2.0
        ep = ((b[f"{pfx}_ep_out"] - a[f"{pfx}_ep_out"]) & 0xffffffff) / 2.0
        leak = (rx - ep) / rx * 100 if rx else float("nan")
        live = "LIVE" if rx > 1_000_000 else "NO INGRESS"
        off = (rx - _NOM) / _NOM * 100 if rx else float("nan")
        print(f"  {tag}  {rx:>11,.0f} B/s  {off:+6.2f}% vs nominal  "
              f"leak {leak:+5.2f}%  overflow {b[f'{pfx}_overflow']:,}  {live}")
    if query(board) and b["bkp_rx_beats"] == a["bkp_rx_beats"]:
        print("    -> the BACKUP is not receiving. Do NOT switch to it.")
    return 0


def main():
    board = "169.254.9.200"
    args = sys.argv[1:]
    if args and args[0].count(".") == 3:
        board = args.pop(0)
    cmd = args[0] if args else "status"

    if cmd == "status":
        return show(board)
    if cmd in ("main", "backup"):
        if query(board) is None:
            print("  this bitstream was built WITHOUT dual USB.")
            return 2
        print(f"requesting {cmd}:")
        query(board, b"1" if cmd == "backup" else b"0")
        return show(board)
    if cmd == "watch":
        secs = int(args[1]) if len(args) > 1 else 60
        t0 = time.time()
        while time.time() - t0 < secs:
            r = query(board)
            if r is None:
                print("  not a dual-USB build"); return 2
            print(f"  t+{time.time()-t0:5.0f}s  select={r['select']} live={r['live']} "
                  f"handovers={r['switches']} bkp_ovf={r['bkp_overflow']}")
            time.sleep(5.0)
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
