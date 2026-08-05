#!/usr/bin/env python3
"""Measure ABSOLUTE transmit timestamp lag, in samples, against the PTP timeline.

    sudo tcpdump -i ens5 -n -s0 -w lag.pcap '(udp port 4321) or (udp port 319)' -c 8000
    tools/ts_lag.py lag.pcap

WHY THIS EXISTS. The firmware's own `drift` statistic is
`last_emitted_timestamp - ptp_now`, sampled in the main loop. That mixes the
quantity we care about (how far behind the PTP timeline our packets are stamped)
with something we do not (how long firmware took to get around to reading it).
It read -39 while the true on-wire figure was needed to design a latency budget,
and a latency budget built on the wrong number is worthless.

The quantity that matters is: at the instant a packet hits the wire, how far in
the PAST is the timestamp inside it? That is exactly what a receiver's latency
setting has to cover, together with network transit. A receiver at 0.25 ms
latency (12 samples) discards anything stamped more than ~12 samples ago.

METHOD. The capture also contains PTPv1 Sync messages from the grandmaster,
whose originTimestamp is grandmaster time. Fitting host-capture-clock against
those gives a host->PTP mapping, after which every audio packet's arrival can be
expressed in PTP samples and compared with the timestamp it carries.

    lag_samples = ptp_time_at_arrival - packet_timestamp

Accuracy is limited by Sync transit delay (tens of us) and capture timestamping,
so treat this as good to a few samples -- which is the resolution the question
needs. A real Dante device in the same capture is the control: whatever
systematic error the method has applies equally to it, so the DIFFERENCE between
its lag and ours is exact even if the absolute is not.

READ THE PER-PACKET SPREAD, NOT JUST THE MEAN. A low mean with a wide spread is
worse than a slightly higher mean that is tight: the receiver discards on the
WORST packet, not the average one.
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
    _, dport = struct.unpack(">HH", udp[0:4])
    return src, dst, dport, udp[8:]


def ptpv1_sync_seconds(pl):
    """originTimestamp (seconds, nanoseconds) from a PTPv1 Sync, or None.

    IEEE 1588-2002 header is 40 bytes fixed; Sync's originTimestamp is the first
    field of the body. control == 0 marks Sync; messageType lives in the low
    nibble of byte 20 in the v1 layout used on the wire by Dante devices.
    """
    if len(pl) < 52:
        return None
    control = pl[32]
    if control != 0:                      # 0 = Sync
        return None
    sec, nsec = struct.unpack(">II", pl[40:48])
    if sec == 0:
        return None
    return sec + nsec / 1e9


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    syncs = []                                   # (host_wall, ptp_seconds)
    audio = defaultdict(list)                    # src -> [(host_wall, ts_samples)]

    for wall, data in pcap_packets(sys.argv[1]):
        p = parse(data)
        if not p:
            continue
        src, dst, dport, pl = p
        if dport == 319:
            t = ptpv1_sync_seconds(pl)
            if t is not None:
                syncs.append((wall, t))
        elif dport == 4321 and len(pl) >= 9 and pl[0] == 0x02:
            sec, sub = struct.unpack(">II", pl[1:9])
            audio[(src, dst)].append((wall, sec * SR + sub))

    if len(syncs) < 2:
        print(f"need >=2 PTPv1 Sync messages to build a PTP timeline, got {len(syncs)}")
        print("capture with:  '(udp port 4321) or (udp port 319)'")
        return 1

    # host_wall -> ptp_seconds, least squares over the capture.
    n = len(syncs)
    mx = sum(w for w, _ in syncs) / n
    my = sum(t for _, t in syncs) / n
    sxx = sum((w - mx) ** 2 for w, _ in syncs)
    sxy = sum((w - mx) * (t - my) for w, t in syncs)
    slope = sxy / sxx if sxx else 1.0
    print(f"PTP timeline from {n} Sync messages   host->PTP slope {slope:.9f} "
          f"({(slope - 1) * 1e6:+.1f} ppm)")
    if abs(slope - 1) > 200e-6:
        print("  WARNING: slope implausible; capture too short or Syncs too sparse")
    print()

    def ptp_at(wall):
        return my + (wall - mx) * slope

    rows = []
    for (src, dst), pkts in sorted(audio.items()):
        if len(pkts) < 20:
            continue
        lags = [(ptp_at(w) * SR) - ts for w, ts in pkts]
        lags.sort()
        mean = sum(lags) / len(lags)
        p50 = lags[len(lags) // 2]
        p99 = lags[int(len(lags) * 0.99)]
        rows.append((src, dst, len(pkts), mean, lags[0], p50, p99, lags[-1]))

    print(f"{'source':<16} {'group':<16} {'n':>5} {'mean':>8} {'min':>8} "
          f"{'p50':>8} {'p99':>8} {'max':>8}   (samples behind PTP)")
    for src, dst, n_, mean, lo, p50, p99, hi in rows:
        print(f"{src:<16} {dst:<16} {n_:>5} {mean:8.1f} {lo:8.1f} "
              f"{p50:8.1f} {p99:8.1f} {hi:8.1f}")

    print()
    print("what a receive-latency setting must cover (worst packet, + transit):")
    for src, dst, n_, mean, lo, p50, p99, hi in rows:
        ms = hi / SR * 1000
        verdict = []
        for lat_ms in (0.25, 0.5, 1.0, 2.0):
            verdict.append(f"{lat_ms}ms:{'OK ' if ms < lat_ms else 'MISS'}")
        print(f"  {src:<16} worst {hi:7.1f} samples = {ms:5.2f} ms   "
              + "  ".join(verdict))

    if len(rows) >= 2:
        print()
        print("relative to each other (method error cancels here -- this is exact):")
        for i in range(len(rows)):
            for j in range(i + 1, len(rows)):
                d = rows[i][3] - rows[j][3]
                print(f"  {rows[i][0]} - {rows[j][0]} = {d:+.1f} samples "
                      f"({d / SR * 1000:+.3f} ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
