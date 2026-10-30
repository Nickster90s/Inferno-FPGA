#!/usr/bin/env python3
"""Sweep a ULPI input IDELAY tap and report what each tap enumerates as.

    sudo tools/tapsweep.py                  # sweep Main, hub port 8
    sudo tools/tapsweep.py --phy backup     # sweep Backup, hub port 10
    sudo tools/tapsweep.py --port 8 --phy main --board 169.254.9.200

NEEDS ROOT: it power-cycles the hub port through
/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port<N>/disable so a 32-tap sweep runs
unattended. The console 'u' key does the same stepping but only increments and
tells you to "re-plug USB to test" -- and on 2026-10-30 the serial console went
silent entirely (nothing at 1 Mbaud or 115200, while UDP kept answering), which
left the one documented knob for the 60 MHz ULPI eye unreachable. Hence the 'T'
opcode in firmware/dstats.c and this tool.

READ THIS BEFORE BELIEVING A RESULT:

  * A tap that enumerates at 480 is inside the eye. speed=12 (full-speed) means
    the PHY took the connect but the HS chirp failed; that is the SAME symptom
    as a wrong data-bit order, so a sweep that finds NO 480 tap anywhere is
    evidence the problem is not timing at all -- see the pinout traps in README.
  * The two buses need INDEPENDENT taps. avb-usb-host measured tap 8 working on
    the Backup PHY while the same tap left Main decoding SOFs cleanly yet never
    asserting its D+ pull-up: reads tolerate ~0.6 ns of extra nxt/data delay,
    the write handshake does not.
  * The eye centre depends on PLACEMENT, so it moves when the floorplan or the
    netlist changes. A tap measured on one bitstream is not a constant of the
    hardware. Pin the winner as the CSR reset in avb_soc.py and re-measure
    after any gateware change.
"""
import argparse
import os
import socket
import struct
import subprocess
import sys
import time

PORT = 7779
TAG_TAP = 0x54415031          # 'TAP1'


def set_tap(board, phy, tap, timeout=3.0, tries=4):
    """Set one bus's tap; returns (main_tap, backup_tap, dual_built)."""
    if phy == "query":
        arg = b""
    elif phy == "main":
        arg = bytes([tap])
    else:
        # Main's byte is mandatory in the 3-byte form, so read it back first and
        # write it unchanged -- otherwise sweeping Backup would silently retune
        # Main underneath the measurement.
        cur = set_tap(board, "query", 0)
        arg = bytes([cur[0], tap])
    last = None
    for _ in range(tries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(timeout)
            s.sendto(b"T" + arg, (board, PORT))
            d, _ = s.recvfrom(512)
            if struct.unpack(">I", d[:4])[0] != TAG_TAP:
                raise ValueError("bad tag; is the firmware current?")
            return d[4], d[5], d[6]
        except Exception as e:                      # noqa: BLE001
            last = e
    raise last


def port_cycle(port):
    path = ("/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port%d/disable" % port)
    if not os.path.exists(path):
        sys.exit("no %s -- is the board on hub port %d of bus 1?" % (path, port))
    for v in ("1", "0"):
        with open(path, "w") as f:
            f.write(v)
        time.sleep(0.6)


def speed(port, timeout=5.0):
    p = "/sys/bus/usb/devices/1-%d/speed" % port
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with open(p) as f:
                return f.read().strip()
        except OSError:
            time.sleep(0.25)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="169.254.9.200")
    ap.add_argument("--phy", default="main", choices=("main", "backup"))
    ap.add_argument("--port", type=int, default=None,
                    help="hub port (default 8 for main, 10 for backup)")
    ap.add_argument("--taps", default="0-31", help="e.g. 0-31 or 6,8,10")
    a = ap.parse_args()
    port = a.port if a.port is not None else (8 if a.phy == "main" else 10)

    if "-" in a.taps:
        lo, hi = (int(v) for v in a.taps.split("-"))
        taps = range(lo, hi + 1)
    else:
        taps = [int(v) for v in a.taps.split(",")]

    m, b, dual = set_tap(a.board, "query", 0)
    print("start: main tap %d, backup tap %d, dual=%d" % (m, b, dual))
    if a.phy == "backup" and not dual:
        sys.exit("this bitstream has no 2nd ULPI (build with USB2=1)")
    print("sweeping %s on hub port %d\n" % (a.phy, port))

    found = []
    print("tap  speed")
    for t in taps:
        set_tap(a.board, a.phy, t)
        port_cycle(port)
        sp = speed(port)
        print("%3d  %s" % (t, sp or "-"), flush=True)
        if sp == "480":
            found.append(t)

    print()
    if found:
        print("HIGH SPEED on taps: %s" % found)
        print("Pin the centre of that run as the CSR reset in avb_soc.py")
        print("(ulpi_idelay_tap / ulpi2_idelay_tap) and re-measure after any")
        print("gateware change -- the eye centre moves with placement.")
    else:
        print("NO tap enumerated at high speed.")
        print("A phase-INVARIANT failure is not a timing problem. Check the")
        print("pinout and the DIR/NXT wiring (README, 'USB3300 ULPI pinouts').")
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
