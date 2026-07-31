#!/usr/bin/env python3
"""Fetch and decode the FPGA's on-device control-plane capture.

Dante Controller talks to the board over UNICAST, and the bench switch is
unmanaged, so it forwards that traffic only to the board's port. A tcpdump on
the build host therefore sees our multicast and NONE of the controller's
requests or our replies -- which is why several "DC sent us nothing"
conclusions during bring-up were wrong. The board is the only vantage point
that sees both directions.

    ./tools/cap_fetch.py [board_ip] [--reset]

Sends a request to :7778; the board ships its ring back on :9997.
"""
import socket
import struct
import sys

BOARD = "169.254.9.200"
REQ_PORT = 7778
OUT_PORT = 9997

ARC_OPCODES = {
    0x1000: "channel/flow counts", 0x1002: "device name",
    0x1003: "device names",        0x1100: "property values",
    0x1102: "property types",      0x2000: "tx channels",
    0x2010: "tx friendly names",   0x2200: "tx flows",
    0x2204: "tx flow detail?",     0x2320: "?",
    0x3000: "rx channels",         0x3200: "rx flows",
    0x3300: "clock domain",        0x4100: "?",
}
PORTNAMES = {4440: "ARC", 8800: "CMC", 8700: "INFO", 4455: "FLOW",
             319: "PTP-EVT", 320: "PTP-GEN", 8702: "DEVINFO", 8708: "HEARTBEAT"}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    board = args[0] if args else BOARD
    reset = "--reset" in sys.argv

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("", OUT_PORT))
    rx.settimeout(2.0)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(b"r" if reset else b"?", (board, REQ_PORT))

    entries, total, expected = {}, None, None
    while True:
        try:
            data, _ = rx.recvfrom(4096)
        except socket.timeout:
            break
        if len(data) < 8:
            continue
        total, idx, count = struct.unpack(">IHH", data[:8])
        expected = count
        o = 8
        n = idx
        while o + 9 <= len(data):
            t_ms, d, ln, cl = struct.unpack(">IBHH", data[o:o + 9])
            o += 9
            if o + cl > len(data):
                break
            entries[n] = (t_ms, d, ln, data[o:o + cl])
            o += cl
            n += 1

    if not entries:
        print(f"no capture returned from {board} -- is the firmware running?")
        return 1

    print(f"{len(entries)} of {expected} frames ({total} recorded since reset)\n")
    for i in sorted(entries):
        t_ms, d, ln, raw = entries[i]
        ihl = (raw[14] & 0x0F) * 4
        u = 14 + ihl
        sp, dp = struct.unpack(">HH", raw[u:u + 4])
        src = ".".join(str(x) for x in raw[26:30])
        dst = ".".join(str(x) for x in raw[30:34])
        pay = raw[u + 8:]
        svc = PORTNAMES.get(dp) or PORTNAMES.get(sp) or "?"
        arrow = "TX" if d else "RX"

        note = ""
        if sp in (4440, 8800, 4455) or dp in (4440, 8800, 4455):
            if len(pay) >= 10:
                sc, tl, seq, o1, o2 = struct.unpack(">HHHHH", pay[:10])
                name = ARC_OPCODES.get(o1, "")
                st = "OK" if o2 == 1 else ("req" if o2 == 0 else f"code=0x{o2:04x}")
                note = f"  op=0x{o1:04x} {name:<22s} {st} seq={seq}"
        elif sp == 8700 or dp == 8700:
            if len(pay) >= 32:
                note = f"  query=0x{pay[27]:02x} op={pay[24:32].hex()}"

        print(f"[{t_ms:>8} ms] {arrow} {svc:<9s} {src}:{sp} -> {dst}:{dp} "
              f"len={ln}{note}")
        if note and len(pay) > 10:
            print(f"             {pay[:64].hex()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
