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
        # DECODEER OP PROTOCOL, NIET BLIND ALS UDP.
        # Dit leesde de eerste 4 bytes na de IP-header ALTIJD als src/dst poort.
        # Bij ICMP zijn dat type, code en checksum, en dan krijg je een
        # verzonnen "poort 771 -> 46077" die er volstrekt echt uitziet. Op
        # 2026-11-05 heeft dat tot de conclusie geleid dat een DAC8 ons met
        # 5000 UDP-pakketten/s bombardeerde, terwijl het ICMP type 3 code 3
        # (port unreachable) was -- 0x0303 = 771 en de checksum 0xb3fd = 46077.
        # De werkelijke oorzaak was omgekeerd: WIJ bleven 17 uur naar een dode
        # flow zenden en kregen op elk pakket een foutmelding terug.
        proto = raw[23] if len(raw) > 23 else 0
        PNAME = {1: "ICMP", 2: "IGMP", 6: "TCP", 17: "UDP"}
        if proto != 17:
            u0 = 14 + ihl
            src = ".".join(str(x) for x in raw[26:30])
            dst = ".".join(str(x) for x in raw[30:34])
            extra = ""
            if proto == 1 and len(raw) >= u0 + 8:
                t, c = raw[u0], raw[u0 + 1]
                extra = " type=%d code=%d" % (t, c)
                inner = raw[u0 + 8:]
                if len(inner) >= 28:
                    iihl = (inner[0] & 0x0F) * 4
                    isrc = ".".join(str(x) for x in inner[12:16])
                    idst = ".".join(str(x) for x in inner[16:20])
                    isp, idp = struct.unpack(">HH", inner[iihl:iihl + 4])
                    extra += "  [afgewezen: %s:%d -> %s:%d]" % (isrc, isp, idst, idp)
            print("[%d ms] %s %-9s %s -> %s len=%d%s"
                  % (t_ms, "TX" if d else "RX", PNAME.get(proto, "proto%d" % proto),
                     src, dst, ln, extra))
            continue
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
