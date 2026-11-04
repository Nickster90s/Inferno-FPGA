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
