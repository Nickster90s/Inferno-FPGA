#!/usr/bin/env python3
"""Read every Dante device's OWN measured RX latency, straight off the wire.

    sudo tcpdump -i ens5 -n -s0 -w hb.pcap 'udp port 8708' -c 200
    tools/dante_latency.py hb.pcap

BENCH RULE: read from a pcap captured on ens5. Joining the multicast group with
a plain socket binds on the DEFAULT ROUTE, which here is eno1 -- the first run of
this tool reported two devices on 192.168.2.x that are not on the bench at all.

WHY THIS EXISTS. Dante Controller's "Latency Status" square is the only
authoritative statement of whether a receiver is happy, and until now the only
way to read it was to ask the operator to look at a GUI on another machine.
Everything measurable from the transmitter -- packet rate, fpp, continuity,
non-zero samples, ring level, underrun, PTP lock -- stayed green through hours
of silence, because a packet discarded for arriving outside the playout window
is discarded SILENTLY.

Devices broadcast the number themselves. inferno's info_mcast_server.rs:227-250
builds a block in the heartbeat multicast (224.0.0.233:8708):

    u16 length        = 24 + flows*4
    u16 0x8003                          <- block type
    u16 4
    u16 content_len   = 12 + flows*4
    u16 counter
    u16 0
    u16 flows_count
    u16 0
    u16 24
    u16 0
    u32 sample_rate
    u32 latency_samples[flows_count]    <- per flow

The value is `actual_latency_samples`, defined at flows_rx.rs:122 as

    max over the reporting interval of (now - packet_timestamp) at receipt

i.e. exactly the quantity a receive-latency setting has to cover, measured by
the receiver on its own media clock. `.swap(0)` on read makes each report a
PEAK-SINCE-LAST-REPORT, not an average -- which is the right statistic, because
a receiver drops on the worst packet, not the mean one.

READ IT AGAINST THE DEVICE'S LATENCY SETTING. A device set to 1 ms (48 samples)
reporting 60 is dropping audio; the same device reporting 12 is fine.
"""
import struct
import sys
from collections import defaultdict

PORT = 8708
SR_DEFAULT = 48000


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
            _, _, caplen, _ = struct.unpack(endian + "IIII", ph)
            data = f.read(caplen)
            if len(data) < caplen:
                return
            yield data


def udp_payload(data):
    """(src_ip, dport, payload) for an IPv4/UDP frame, else None."""
    if len(data) < 14 or data[12:14] != b"\x08\x00":
        return None
    ihl = (data[14] & 0x0F) * 4
    ip = data[14:14 + ihl]
    if len(ip) < 20 or ip[9] != 17:
        return None
    src = ".".join(str(b) for b in ip[12:16])
    udp = data[14 + ihl:]
    if len(udp) < 8:
        return None
    _, dport = struct.unpack(">HH", udp[0:4])
    return src, dport, udp[8:]


def find_latency_blocks(buf):
    """Yield (sample_rate, [latency,...]) for every 0x8003 block in a datagram.

    Scanned rather than walked from a fixed header offset: the surrounding
    message carries a variable number of preceding blocks (peaks, clock status),
    and guessing that layout is how a decoder silently returns nothing. Each
    candidate is validated against its own internally-redundant length fields, so
    a false positive would have to satisfy four independent constraints.
    """
    n = len(buf)
    for i in range(0, n - 24, 2):
        if buf[i + 2:i + 4] != b"\x80\x03":
            continue
        blk_len, _, _, content_len = struct.unpack(">HHHH", buf[i:i + 8])
        flows, = struct.unpack(">H", buf[i + 12:i + 14])
        if flows > 64:
            continue
        if blk_len != 24 + flows * 4 or content_len != 12 + flows * 4:
            continue
        if i + 24 + flows * 4 > n:
            continue
        sr, = struct.unpack(">I", buf[i + 20:i + 24])
        if sr not in (44100, 48000, 88200, 96000, 176400, 192000):
            continue
        lat = list(struct.unpack(">%dI" % flows, buf[i + 24:i + 24 + flows * 4]))
        yield sr, lat


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    seen = defaultdict(lambda: {"n": 0, "sr": SR_DEFAULT, "peak": [], "last": []})
    for data in pcap_packets(sys.argv[1]):
        p = udp_payload(data)
        if not p:
            continue
        src, dport, pl = p
        if dport != PORT:
            continue
        for sr, lat in find_latency_blocks(pl):
            d = seen[src]
            d["n"] += 1
            d["sr"] = sr
            d["last"] = lat
            if len(d["peak"]) < len(lat):
                d["peak"] += [0] * (len(lat) - len(d["peak"]))
            for i, v in enumerate(lat):
                d["peak"][i] = max(d["peak"][i], v)

    if not seen:
        print("no 0x8003 latency blocks seen.")
        print("Devices report this only for flows they RECEIVE; a transmitter-only")
        print("device publishes none. Check that receivers are subscribed.")
        return 1

    print(f"{'device':<18} {'rate':>6} {'flow':>5} {'peak':>8} {'last':>8}   verdict")
    for ip in sorted(seen):
        d = seen[ip]
        sr = d["sr"]
        for i, pk in enumerate(d["peak"]):
            last = d["last"][i] if i < len(d["last"]) else 0
            ms = pk / sr * 1000.0
            fits = [f"{L}ms:{'OK ' if ms < L else 'MISS'}"
                    for L in (0.25, 0.5, 1.0, 2.0, 5.0)]
            print(f"{ip:<18} {sr:>6} {i:>5} {pk:>8} {last:>8}   "
                  f"{ms:5.2f} ms  " + " ".join(fits))
    print("\npeak = worst packet since that device's previous report.")
    print("A device whose latency SETTING is below its peak is dropping audio.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
