#!/usr/bin/env python3
# Query a real Dante device's ARC server directly, so we can see what a working
# device returns for opcodes we currently stub with zeros.
#
# Dante Controller shows our PTPv2 domain as 0 and priority as 0/0 -- literal
# zeros -- while real devices show N/A there and report v1 Leader/Follower. We
# return 0x1100 as 110 zero bytes and 0x1102 as 94 zero bytes (as inferno does).
# If either carries clock capability, our zeros would read exactly as
# "PTPv2, domain 0, priority 0/0".
#
# Framing (firmware/dante_msg.h): 10-byte BE header
#   start_code | total_length | seqnum | opcode1 | opcode2 | content
import socket, struct, sys

# THE START CODE SELECTS A MESSAGE REVISION. This script had 0x2714 hardcoded
# with a comment claiming it was "what DC uses on the wire". It is not, and the
# difference is not cosmetic -- measured 2026-11-06 against a DirectOut PRODIGY,
# reproducibly, same device and same opcode:
#
#     0x1000 with 0x2714 -> 46 bytes, capability word 0x0ff9
#     0x1000 with 0x2809 -> 86 bytes, capability word 0x1ff9
#
# The on-device capture (tools/cap_fetch.py) shows Dante Controller sending
# 0x2809. So every comparison made with this script before that date was
# reading the OLD revision, and twice sent me after the wrong capability bytes.
# 0x2809 is now the default; pass --old to get the previous behaviour, and both
# are printed so a reply can never be attributed to the wrong revision.
SC_DC  = 0x2809
SC_OLD = 0x2714
START_CODE = SC_OLD if "--old" in sys.argv else SC_DC

TARGETS = [a for a in sys.argv[1:] if not a.startswith("-")] or \
          ["169.254.60.249", "169.254.61.114"]
OPCODES = [0x1000, 0x1100, 0x1102, 0x3300, 0x2204, 0x4100, 0x2200, 0x2320, 0x3320]

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1.5)
seq = 0x1000

for ip in TARGETS:
    print(f"\n===== {ip}   (start_code {START_CODE:#06x})")
    for op in OPCODES:
        seq += 1
        pkt = struct.pack(">HHHHH", START_CODE, 10, seq, op, 0)
        try:
            s.sendto(pkt, (ip, 4440))
            data, _ = s.recvfrom(2048)
        except socket.timeout:
            print(f"  op=0x{op:04x}  <no reply>")
            continue
        if len(data) < 10:
            print(f"  op=0x{op:04x}  short: {data.hex()}")
            continue
        sc, tl, rseq, o1, o2 = struct.unpack(">HHHHH", data[:10])
        body = data[10:]
        status = "OK" if o2 == 1 else f"code=0x{o2:04x}"
        nz = sum(1 for b in body if b)
        print(f"  op=0x{op:04x} -> o1=0x{o1:04x} {status} len={len(body)} nonzero={nz}")
        if body:
            for i in range(0, len(body), 32):
                print("      " + body[i:i + 32].hex())
