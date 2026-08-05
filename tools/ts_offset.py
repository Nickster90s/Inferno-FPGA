#!/usr/bin/env python3
"""Read, set, or CALIBRATE the TX timestamp offset against a real Dante device.

    tools/ts_offset.py                 # read the value in force
    tools/ts_offset.py -88             # set it (re-anchors; may click)
    tools/ts_offset.py --calibrate     # two-point fit, then solve and apply

WHY CALIBRATE INSTEAD OF COMPUTING. The obvious model is

    lag = (fpp - 1) - offset

i.e. the anchor offset moves the emitted timestamp 1:1. Measured on the bench it
does not: changing the offset by -162 moved the on-wire lag by +366, a ratio of
2.26. Two separate attempts to pick the constant from the model missed, once
short and once long, and each cost a rebuild, a flash, a reboot and a hand
re-created multicast flow -- so each produced ONE data point, at a different
board uptime, with any accumulated phase drift folded in and inseparable.

So do not model it. Measure the slope on the actual hardware, in one sitting,
with nothing else changing, and solve.

TARGET. A real Dante transmitter in the same capture is the reference: match
where IT sits, rather than any absolute figure. tools/ts_lag.py's absolute
column depends on a host->PTP fit whose slope comes back implausible (+28 ppm)
and on PTPv1 Sync field offsets that were inferred rather than verified. The
DIFFERENCE between two streams in one capture is free of both.

CAVEAT: every set re-anchors, which is a timestamp discontinuity -- one packet
advances by other than fpp. Expect a click. Do not run this while anyone is
listening to the result.
"""
import re
import struct
import subprocess
import sys
import socket

BOARD = "169.254.9.200"
PORT = 7779
IFACE = "ens5"
SCRATCH = "/tmp/ts_offset_cal.pcap"


def rpc(payload, timeout=3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(payload, (BOARD, PORT))
    d, _ = s.recvfrom(1500)
    tag, val = struct.unpack(">II", d[:8])
    if tag != 0x54534F31:
        raise SystemExit(f"unexpected reply tag {tag:#x}")
    return struct.unpack(">i", d[4:8])[0]


def measure(n=12000):
    """Return (ours, reference, delta) mean lag in samples, or None."""
    subprocess.run(
        ["sudo", "tcpdump", "-i", IFACE, "-n", "-s0", "-w", SCRATCH,
         "(udp port 4321) or (udp port 319)", "-c", str(n)],
        check=True, capture_output=True, timeout=180)
    out = subprocess.run([sys.executable, "tools/ts_lag.py", SCRATCH],
                         check=True, capture_output=True, text=True).stdout
    rows = {}
    for line in out.splitlines():
        m = re.match(r"^(\d+\.\d+\.\d+\.\d+)\s+\S+\s+\d+\s+(-?\d+\.\d)", line)
        if m:
            rows[m.group(1)] = float(m.group(2))
    if BOARD not in rows or len(rows) < 2:
        print(out)
        return None
    ours = rows[BOARD]
    ref_ip = [k for k in rows if k != BOARD][0]
    return ours, rows[ref_ip], ours - rows[ref_ip]


def main():
    args = sys.argv[1:]
    if not args:
        print(f"ts offset in force: {rpc(b'o')} samples")
        return 0

    if args[0] != "--calibrate":
        v = int(args[0])
        print(f"ts offset now: {rpc(b'o' + str(v).encode())} samples")
        return 0

    cur = rpc(b"o")
    print(f"starting from offset {cur}\n")

    pts = []
    for probe in (cur, cur + 200):
        rpc(b"o" + str(probe).encode())
        r = measure()
        if r is None:
            print("could not see both our stream and a reference device.")
            print("Is the multicast flow up, and is a real Dante device "
                  "transmitting multicast on this segment?")
            return 1
        ours, ref, delta = r
        print(f"  offset {probe:>6}   ours {ours:8.1f}   ref {ref:6.1f}   "
              f"delta {delta:+8.1f}")
        pts.append((probe, delta))

    (x0, d0), (x1, d1) = pts
    if x1 == x0 or d1 == d0:
        print("\nno response to the offset -- knob not taking effect")
        return 1
    slope = (d1 - d0) / (x1 - x0)          # samples of delta per sample of offset
    solved = int(round(x0 - d0 / slope))   # want delta == 0
    print(f"\nslope {slope:+.3f} delta-samples per offset-sample "
          f"(a 1:1 model would be {-1.0:+.3f})")
    print(f"solving delta -> 0 gives offset {solved}")

    rpc(b"o" + str(solved).encode())
    r = measure()
    if r is None:
        return 1
    ours, ref, delta = r
    print(f"\n  applied {solved:>6}   ours {ours:8.1f}   ref {ref:6.1f}   "
          f"delta {delta:+8.1f}")
    print("\nfold this into DANTE_TX_TS_OFFSET in firmware/dante_tx.h "
          "once a listening test agrees."
          if abs(delta) < 8 else
          "\ndelta still large -- the relationship may not be linear; "
          "re-run and compare slopes before trusting either.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
