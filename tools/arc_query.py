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

TARGETS = sys.argv[1:] or ["169.254.60.249", "169.254.61.114"]
OPCODES = [0x1000, 0x1100, 0x1102, 0x3300, 0x2204, 0x4100, 0x2200, 0x2320]

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1.5)
seq = 0x1000

for ip in TARGETS:
    print(f"\n===== {ip} =====")
    for op in OPCODES:
        seq += 1
        # start_code 0x2714 is what DC uses on the wire; content empty.
        pkt = struct.pack(">HHHHH", 0x2714, 10, seq, op, 0)
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
