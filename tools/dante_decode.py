#!/usr/bin/env python3
"""Decode Dante audio multicast from a pcap: timestamps, continuity, content.

    sudo tcpdump -i ens5 -n -s0 -w mc.pcap 'udp port 4321' -c 4000
    tools/dante_decode.py mc.pcap

WHY THIS EXISTS. Unicast flows to a receiver are invisible from this host -- the
switch learns the MAC and never floods them here -- so for a long time the only
view of our own audio was the transmitter's own counters, which are exactly what
had been lying (a media clock 227 ms in the future read healthy on every one).
Putting a couple of channels on MULTICAST makes the stream readable from ens5,
and a real Dante device transmitting its own multicast alongside gives a
reference implementation to compare against, byte for byte.

Wire format (flows_tx.rs:139-197), 9-byte header, no RTP:

    [0]      0x02                    constant
    [1..5]   seconds        u32 BE
    [5..9]   subsec_samples u32 BE   0 .. sample_rate-1
    [9..]    interleaved samples, BIG-endian, MSB-justified

    timestamp = seconds * sample_rate + subsec_samples, in units of SAMPLES

WHAT TO LOOK FOR:
  * timestamp advancing by exactly fpp per packet, no gaps, no duplicates
  * timestamp a multiple of fpp (flows_tx.rs bootstraps it that way)
  * our stream's timestamp tracking a real device's with a stable offset
  * samples actually non-zero -- a perfectly-timestamped stream of digital
    silence looks identical to working audio in every counter on the box
"""
import struct
import sys
from collections import defaultdict

SR = 48000


def pcap_packets(path):
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            return
        magic = struct.unpack("<I", gh[:4])[0]
        if magic in (0xA1B2C3D4, 0xA1B23C4D):
            endian, nano = "<", magic == 0xA1B23C4D
        elif magic in (0xD4C3B2A1, 0x4D3CB2A1):
            endian, nano = ">", magic == 0x4D3CB2A1
        else:
            raise SystemExit(f"not a pcap: magic {magic:#x}")
        while True:
            ph = f.read(16)
            if len(ph) < 16:
                return
            ts_s, ts_f, caplen, _ = struct.unpack(endian + "IIII", ph)
            data = f.read(caplen)
            if len(data) < caplen:
                return
            yield ts_s + ts_f / (1e9 if nano else 1e6), data


def parse(data):
    """Ethernet -> IPv4 -> UDP -> (src, dst, dport, payload). None if not ours."""
    if len(data) < 14 or data[12:14] != b"\x08\x00":
        return None
    ihl = (data[14] & 0x0F) * 4
    ip = data[14:14 + ihl]
    if len(ip) < 20 or ip[9] != 17:
        return None
    src = ".".join(str(b) for b in ip[12:16])
    dst = ".".join(str(b) for b in ip[16:20])
    udp = data[14 + ihl:]
    if len(udp) < 8:
        return None
    sport, dport = struct.unpack(">HH", udp[0:4])
    return src, dst, dport, udp[8:]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    flows = defaultdict(lambda: {"n": 0, "prev": None, "gaps": 0, "dups": 0,
                                 "steps": defaultdict(int), "nonzero": 0,
                                 "peak": 0, "bad_const": 0, "misalign": 0,
                                 "first_ts": None, "last_ts": None,
                                 "first_wall": None, "last_wall": None,
                                 "nch": None})
    for wall, data in pcap_packets(sys.argv[1]):
        p = parse(data)
        if not p:
            continue
        src, dst, dport, pl = p
        if dport != 4321 or len(pl) < 9:
            continue
        f = flows[(src, dst)]
        if pl[0] != 0x02:
            f["bad_const"] += 1
        sec, sub = struct.unpack(">II", pl[1:9])
        ts = sec * SR + sub
        body = pl[9:]
        f["n"] += 1
        if f["first_ts"] is None:
            f["first_ts"], f["first_wall"] = ts, wall
        f["last_ts"], f["last_wall"] = ts, wall
        if f["prev"] is not None:
            d = ts - f["prev"]
            f["steps"][d] += 1
            if d == 0:
                f["dups"] += 1
            elif d < 0:
                f["gaps"] += 1
        f["prev"] = ts
        # 24-bit big-endian MSB-justified samples
        for i in range(0, len(body) - 2, 3):
            v = (body[i] << 16) | (body[i + 1] << 8) | body[i + 2]
            if v & 0x800000:
                v -= 1 << 24
            if v:
                f["nonzero"] += 1
                f["peak"] = max(f["peak"], abs(v))

    for (src, dst), f in sorted(flows.items()):
        if not f["n"]:
            continue
        steps = sorted(f["steps"].items(), key=lambda kv: -kv[1])
        fpp = steps[0][0] if steps else 0
        span = (f["last_wall"] - f["first_wall"]) or 1e-9
        pps = (f["n"] - 1) / span
        ts_span = f["last_ts"] - f["first_ts"]
        # Emitted sample rate implied by the timestamps vs wall clock.
        implied = ts_span / span if span else 0
        print(f"{src} -> {dst}")
        print(f"  packets      {f['n']}  ({pps:.0f} pps over {span:.2f}s)")
        print(f"  fpp          {fpp}   step histogram: "
              f"{', '.join(f'{k}x{v}' for k, v in steps[:4])}")
        print(f"  continuity   gaps(backwards)={f['gaps']} duplicates={f['dups']}"
              f"  const!=0x02: {f['bad_const']}")
        print(f"  ts multiple of fpp: "
              f"{'yes' if fpp and f['first_ts'] % fpp == 0 else 'NO'}")
        print(f"  implied rate {implied:.1f} samples/s "
              f"({(implied / SR - 1) * 1e6:+.1f} ppm vs 48000)")
        print(f"  audio        {f['nonzero']} non-zero samples, "
              f"peak {f['peak']} ({20 * (len(str(f['peak'])) and __import__('math').log10(max(f['peak'],1)) - 6.92) if f['peak'] else 0:.0f} dBFS)"
              if f["peak"] else f"  audio        ALL ZERO (digital silence)")
        print()

    # Cross-compare: our stream against any other device's, in sample units.
    if len(flows) >= 2:
        ks = sorted(flows.keys())
        print("timestamp offsets between streams (samples, at capture start):")
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                a, b = flows[ks[i]], flows[ks[j]]
                if a["first_ts"] and b["first_ts"]:
                    d = a["first_ts"] - b["first_ts"]
                    print(f"  {ks[i][0]} - {ks[j][0]} = {d:+d} samples "
                          f"({d / SR * 1000:+.2f} ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
