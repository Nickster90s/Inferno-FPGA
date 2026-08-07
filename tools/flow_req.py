#!/usr/bin/env python3
# Ask the board for a UNICAST flow, the way a real receiver does.
#
# WHY THIS EXISTS. DVS unicast at 4 slots x fpp=60 is dead while AM2 unicast
# (fpp=16) is clean and DVS MULTICAST at the same 4 x fpp=60 is also clean. So
# the frame at that size is already proven to reach the wire, and the failing
# variable is unicast-specific -- but every test so far needed the user to click
# in Dante Controller, which mixed "our unicast path" with "whatever DVS asks
# for" in a single un-splittable experiment.
#
# This sends the request ourselves, to an address we control, so the two can be
# separated:
#
#   frames arrive here  -> unicast at 4 x fpp=60 works; the fault is in what DVS
#                          specifically requests (read the [flow] console line)
#   nothing arrives     -> reproduced with no DVS involved, on a bench we own
#
# Request layout, from firmware/dante_flows.c:8-24 (which took it from inferno
# flows_control_server.rs). Offsets marked "absolute" are from the START of the
# packet, i.e. they include the 10-byte header -- getting that wrong points the
# socket descriptor into the channel list and the board reads a garbage
# destination.
#
#    header, 10 B:  start_code 0x2714 | total_len | seq | opcode1 | opcode2
#    content, at absolute 10:
#       +0   2  hostname_offset          absolute
#       +2   4  sample_rate
#       +6   4  bits_per_sample
#      +10   2  1
#      +12   2  num_channels
#      +14   2  remote_descriptor_offset absolute
#      +16  .. channel indices, u16, 1-BASED
#      then 6 unidentified bytes, then fpp u16
#    remote descriptor, 8 B:  0x0802 | dport | dst ip
#
#   run:  tools/flow_req.py [board] [--slots N] [--fpp N] [--port N] [--to IP]

import argparse
import socket
import struct
import sys

ap = argparse.ArgumentParser()
ap.add_argument("board", nargs="?", default="169.254.9.200")
ap.add_argument("--slots", type=int, default=4)
ap.add_argument("--fpp", type=int, default=60)
ap.add_argument("--port", type=int, default=14336, help="destination UDP port")
ap.add_argument("--to", default=None, help="destination IP (default: our ens5 address)")
ap.add_argument("--rate", type=int, default=48000)
ap.add_argument("--bits", type=int, default=24)
a = ap.parse_args()

if a.to is None:
    # Bind a scratch socket toward the board so the kernel picks the interface
    # that actually reaches it. Hardcoding our address here would silently test
    # the wrong NIC -- the bench has two, and only ens5 is on the Dante fabric.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.connect((a.board, 4455))
    a.to = probe.getsockname()[0]
    probe.close()

nch = a.slots
HDR = 10
content = bytearray()
content += struct.pack(">H", 0)          # hostname_offset, patched below
content += struct.pack(">I", a.rate)
content += struct.pack(">I", a.bits)
content += struct.pack(">H", 1)
content += struct.pack(">H", nch)
content += struct.pack(">H", 0)          # remote_descriptor_offset, patched below
for i in range(nch):
    content += struct.pack(">H", i + 1)  # tx channels are 1-BASED
content += b"\0" * 6                     # the six unidentified bytes
content += struct.pack(">H", a.fpp)

rdo = HDR + len(content)                 # absolute
desc = bytes([0x08, 0x02, (a.port >> 8) & 0xFF, a.port & 0xFF]) + \
       bytes(int(x) for x in a.to.split("."))
name_off = rdo + len(desc)               # absolute
name = b"flowreq\0"

struct.pack_into(">H", content, 0, name_off)
struct.pack_into(">H", content, 14, rdo)

body = bytes(content) + desc + name
total = HDR + len(body)
pkt = struct.pack(">HHHHH", 0x2714, total, 0x2001, 0x0100, 0) + body

print(f"requesting {nch} slots, fpp={a.fpp} -> {a.to}:{a.port}  from {a.board}")
print(f"  packet {len(pkt)} B, rdo={rdo}, name_off={name_off}")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.bind(("", 0))
s.sendto(pkt, (a.board, 4455))
try:
    rep, src = s.recvfrom(2048)
except socket.timeout:
    print("  NO REPLY from the flow-control server")
    sys.exit(1)

sc, tl, seq, o1, o2 = struct.unpack(">HHHHH", rep[:10])
# o2 is the status: 1 = OK, and the error codes are the real ones from
# flows_control.rs, so a non-1 value is the board telling us exactly why.
CODES = {0x0301: "sample rate / fpp unsupported", 0x0315: "too many TX flows",
         0x2201: "no free context", 1: "OK"}
print(f"  reply opcode1=0x{o1:04x} code=0x{o2:04x} ({CODES.get(o2, 'unknown')}) "
      f"len={len(rep)}")
sys.exit(0 if o2 == 1 else 2)
