#!/usr/bin/env python3
# Controlled-stimulus probe of the FPGA RX timestamp path, reading the firmware's
# report out of a capture rather than a multicast socket (the earlier socket
# version joined the group on the wrong interface and saw nothing).
#
# We send UDP to :7777 at deliberately UNEVEN gaps and compare the gaps the
# firmware timestamped against the gaps we actually sent. Absolute offset does
# not matter -- only whether the SHAPE is reproduced.
import os, socket, struct, subprocess, sys, time

D = os.path.dirname(os.path.abspath(__file__))
PCAP = os.path.join(D, "probe2.pcap")
FPGA = sys.argv[1] if len(sys.argv) > 1 else "169.254.9.200"

GAPS_MS = [5, 40, 7, 60, 12, 5, 90, 20, 5, 35, 8, 70, 15, 5, 50, 25]

td = subprocess.Popen(["sudo", "tcpdump", "-i", "ens5", "-n", "-s0",
                       "-w", PCAP, "udp port 9998"],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2.0)

tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sent = {}
seq = 0x2000
for g in GAPS_MS:
    tx.sendto(struct.pack(">I", seq) + b"probe", (FPGA, 7777))
    sent[seq] = time.time()
    seq += 1
    time.sleep(g / 1000.0)
print(f"sent {len(sent)} probes with uneven gaps")

time.sleep(3.0)          # let at least one 1 Hz report carry the whole ring
td.terminate(); td.wait()
time.sleep(0.3)

got = {}
d = open(PCAP, "rb").read()
e = "<" if d[:4] == b"\xd4\xc3\xb2\xa1" else ">"
o = 24
while o + 16 <= len(d):
    ts, tu, cl, ol = struct.unpack(e + "IIII", d[o:o + 16]); o += 16
    p = d[o:o + cl]; o += cl
    if len(p) < 42 or p[23] != 17:
        continue
    ihl = (p[14] & 0xF) * 4
    pl = p[14 + ihl + 8:]
    if len(pl) < 192:
        continue
    for i in range(16):
        s, ts_s, ts_n = struct.unpack(">III", pl[i * 12:(i + 1) * 12])
        if s in sent:
            got[s] = ts_s * 10**9 + ts_n

common = sorted(got)
print(f"matched {len(common)} of {len(sent)}\n")
print("  seq   host_gap_ms   fpga_gap_ms     error_us")
errs = []
prev = None
for s in common:
    if prev is not None and s == prev + 1:
        hg = (sent[s] - sent[prev]) * 1000.0
        fg = (got[s] - got[prev]) / 1e6
        errs.append((fg - hg) * 1000.0)
        print(f"{s:#06x} {hg:12.3f} {fg:13.3f} {(fg - hg) * 1000:12.1f}")
    prev = s

if len(errs) >= 3:
    import statistics
    print(f"\nerror vs host: mean {statistics.mean(errs):+.1f} us, "
          f"stdev {statistics.pstdev(errs):.1f} us")
    hgaps = [(sent[s] - sent[p]) * 1000 for p, s in zip(common, common[1:])
             if s == p + 1]
    fgaps = [(got[s] - got[p]) / 1e6 for p, s in zip(common, common[1:])
             if s == p + 1]
    print(f"host gap spread: {min(hgaps):.1f}..{max(hgaps):.1f} ms")
    print(f"fpga gap spread: {min(fgaps):.1f}..{max(fgaps):.1f} ms")
    print("\nIf the fpga spread mirrors the host spread, arrival timestamping")
    print("works and the PTP offset noise originates elsewhere.")
