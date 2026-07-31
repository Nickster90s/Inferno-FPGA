#!/usr/bin/env python3
"""Read the board's runtime counters over UDP (the UART belongs to picocom)."""
import socket, struct, sys, time
NAMES = ["talker","packets","underrun","overrun","fifo_level","last_sec","last_ts",
         "anchors","enables","disables","ptp_locked","offset_ns","path_delay_ns",
         "mac_writer_err","rx_crc_err",
         "flows_active","flows_requests","flows_rejected"]
for _f in range(6):
    NAMES += [f"f{_f}_in_use", f"f{_f}_age_ms", f"f{_f}_rebinds"]
SIGNED = {"offset_ns","path_delay_ns"}
def read(board):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(3)
    s.sendto(b"?", (board, 7779))
    d, _ = s.recvfrom(512)
    out = {}
    for i, n in enumerate(NAMES):
        if (i+1)*4 > len(d): break
        v = struct.unpack(">I", d[i*4:(i+1)*4])[0]
        if n in SIGNED and v >= 1 << 31: v -= 1 << 32
        out[n] = v
    return out
if __name__ == "__main__":
    board = sys.argv[1] if len(sys.argv) > 1 else "169.254.9.200"
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    a = read(board)
    if not secs:
        for k, v in a.items(): print(f"  {k:16s} {v}")
    else:
        time.sleep(secs); b = read(board)
        print(f"over {secs}s:")
        for k in a:
            d = b[k] - a[k]
            print(f"  {k:16s} {b[k]:<12} {'delta %+d' % d if d else ''}")
