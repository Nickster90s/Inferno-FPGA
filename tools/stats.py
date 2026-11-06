#!/usr/bin/env python3
"""Read the board's runtime counters over UDP (the UART belongs to picocom)."""
import socket, struct, sys, time
# "sendreq_busy" HEETTE "overrun". Het is send_req dat midden in een traversal
# landt, wat pend[] alsnog serveert: BENIGNE, en op een gezonde 96 kHz-stroom
# loopt het met 11k-28k/s. Onder de oude naam las dat als massale sample-loss en
# dat heeft op 2026-11-03 een halve sessie gekost. De teller die wel audio-verlies
# betekent is write_drops, en die staat in tools/usbsrc.py (U-antwoord).
NAMES = ["talker","packets","underrun","sendreq_busy","fifo_level","last_sec","last_ts",
         "anchors","enables","disables","ptp_locked","offset_ns","path_delay_ns",
         "mac_writer_err","rx_crc_err",
         "ptp_now_sec","ptp_now_sub",
         "flows_active","flows_requests","flows_rejected"]
for _f in range(6):
    NAMES += [f"f{_f}_in_use", f"f{_f}_age_ms", f"f{_f}_rebinds",
              f"f{_f}_mac_hi", f"f{_f}_mac_lo"]
# LET OP: offset_ns IS DE RUWE OFFSET, NIET DE KLOKFOUT.
# firmware/ptpv1.c:368 zet g_ptpv1.offset_ns op het ruwe sample; de servo rekent
# met median_of() daarvan (:502) en de lock/outlier-logica ook. dstats.c:764
# stuurt de RUWE waarde in dit antwoord. Losse uitschieters van 15-50 us zijn
# daarin dus NORMAAL -- de filter houdt ze uit de klok, en de console zwijgt
# (geen "outlier ignored", geen "unlocked", geen "re-anchor").
# Op 2026-11-04 heb ik die ruwe pieken een halve dag als klokverstoring
# gerapporteerd. Wil je de echte klokfout zien, gebruik tools/telemetry.py: die
# stream heeft raw EN filtered per servo-update.
SIGNED = {"offset_ns","path_delay_ns","trim_ppb","drift_samples"}
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
# RING HEALTH ('r' -> 'RNG1'), its own reply because the main one must not grow.
# fifo_level in the list above is the CLAMPED level: block_level turns a negative
# ring level into 0, and that clamp is why four wedges in 2026-08 each presented
# as "fifo_level 0, every ingress counter perfect" and were misread three times
# as "the decoder is producing nothing". ring_level_raw is the same number
# without the clamp.
RING_NAMES  = ["ring_level_raw", "ring_level_min", "ring_underflow",
               "ring_unprime", "fifo_level_clamped", "underrun_ticks",
               "talker_enables"]
RING_SIGNED = {"ring_level_raw", "ring_level_min"}

def read_ring(board):
    """None if the firmware predates the 'r' opcode (it simply will not reply)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(2)
    s.sendto(b"r", (board, 7779))
    try:
        d, _ = s.recvfrom(512)
    except socket.timeout:
        return None
    if len(d) < 4 or d[:4] != b"RNG1":
        return None
    out = {}
    for i, n in enumerate(RING_NAMES):
        off = 4 + i * 4
        if off + 4 > len(d): break
        v = struct.unpack(">I", d[off:off+4])[0]
        if n in RING_SIGNED and v >= 1 << 31: v -= 1 << 32
        out[n] = v
    return out

def print_ring(r):
    if r is None:
        print("  (no RNG1 reply -- firmware predates the 'r' opcode)")
        return
    for k, v in r.items(): print(f"  {k:20s} {v}")
    # NOTHING HERE MEANS ANYTHING UNTIL THE TALKER HAS RUN ONCE. With the
    # gateware enable low, `rd` is parked at its reset value while `wr` climbs
    # with USB samples, and the level is modulo 4*SRING_DEPTH -- so it sweeps
    # its whole +-4096 range every ~10.7 ms at 96 kHz and reads as a deep
    # inversion. On the first boot of this firmware that is exactly what this
    # function reported. Say so instead of crying wolf.
    if r.get("talker_enables") is None:
        print("  VERDICT: firmware predates talker_enables in this reply -- "
              "check `enables` in the main reply before believing level_min.")
        return
    if r["talker_enables"] == 0:
        print("  VERDICT: the talker has never been enabled, so the ring level "
              "is not occupancy yet (rd is parked, the level is modular). "
              "ring_level_min is meaningless until enables >= 1.")
        return
    bad = []
    if r.get("ring_underflow"):   bad.append("ring_underflow is NOT 0")
    if r.get("ring_level_min", 0) < 0: bad.append("ring_level_min went negative")
    # Stated as the fault it means, not as a number to interpret later.
    print("  VERDICT: " + ("ring inverted at least once -- " + ", ".join(bad) +
                           ". The hysteresis recovers in ~11 ms, so this is a"
                           " glitch, but the route is supposed to be impossible"
                           " with 48-bit pointers."
                           if bad else
                           "the ring has never inverted since boot."))

if __name__ == "__main__":
    board = sys.argv[1] if len(sys.argv) > 1 else "169.254.9.200"
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    if board == "--ring" or (len(sys.argv) > 2 and sys.argv[2] == "--ring"):
        if board == "--ring": board = "169.254.9.200"
        print_ring(read_ring(board))
        sys.exit(0)
    a = read(board)
    if not secs:
        for k, v in a.items(): print(f"  {k:16s} {v}")
    else:
        time.sleep(secs); b = read(board)
        print(f"over {secs}s:")
        for k in a:
            d = b[k] - a[k]
            print(f"  {k:16s} {b[k]:<12} {'delta %+d' % d if d else ''}")
    print("ring health:")
    print_ring(read_ring(board))
