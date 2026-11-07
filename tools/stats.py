#!/usr/bin/env python3
"""Read the board's runtime counters over UDP (the UART belongs to picocom)."""
import socket, struct, sys, time
from dante_board import BOARD as _BOARD
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
               "talker_enables",
               # Raw gateware peak registers, to tell "detector never fired"
               # from "readout broken". peak_select_rb must read back 3.
               "peak_ch0", "peak_ch1", "peak_ch2", "peak_ch3",
               "peak_select_rb"]
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

# INFO-PORT STATS ('I' -> 'INF1'). Separate reply for the same reason as --ring.
INFO_NAMES = ["info_rx", "info_rx_unknown", "info_tx", "info_tx_heartbeat",
              "reboot_req", "reboot_odd", "reboot_refused"]

def read_info(board):
    """{} of counters, "silent" if nothing answered, "old" if something did but
    not with INF1.

    THOSE TWO ARE DIFFERENT and this function used to return None for both, so
    `--info` printed "firmware predates the 'I' opcode" at a board that had
    reset and was sitting dead in the loader. That misread cost a round of
    looking on 2026-11-07, in a tool written the same hour to fix exactly this
    class of ambiguity elsewhere. A timeout is not a version.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(2)
    s.sendto(b"I", (board, 7779))
    try:
        d, _ = s.recvfrom(512)
    except socket.timeout:
        return "silent"
    if len(d) < 4 or d[:4] != b"INF1":
        return "old"
    out = {}
    for i, n in enumerate(INFO_NAMES):
        off = 4 + i * 4
        if off + 4 > len(d): break
        out[n] = struct.unpack(">I", d[off:off+4])[0]
    return out

def print_info(r):
    if r == "silent":
        print(f"  NOTHING ANSWERED on {_BOARD}:7779. The board is off, on another"
              " address, or sitting in the loader after a reset with no image to"
              " load. This is NOT a statement about its firmware version.")
        return
    if r == "old":
        print("  answered, but not with INF1 -- that firmware predates the 'I'"
              " opcode.")
        return
    for k, v in r.items(): print(f"  {k:20s} {v}")
    # Dante Controller's reboot button, made legible. The shipping build has no
    # console, so these counters are the only way to tell three very different
    # situations apart -- and a board that stays up looks the same in all of
    # them.
    rq, ro, rr = r.get("reboot_req", 0), r.get("reboot_odd", 0), r.get("reboot_refused", 0)
    if rq == 0 and ro == 0:
        print("  VERDICT: no reboot command has reached this board since boot."
              " If Controller's button was pressed, either it sent nothing or"
              " the frame never arrived -- check the capture ring for an"
              " inbound 0x90 on the info port before blaming the firmware.")
    elif rr:
        print(f"  VERDICT: {rq} reboot command(s) arrived and {rr} were REFUSED"
              " -- there is no usable firmware image in SPI flash at 0x300000,"
              " so a reset would land in the loader and halt. Run"
              " ./flash.sh --permanent, then try again.")
    elif ro:
        print(f"  VERDICT: {ro} command(s) on opcode 0x90 carried an action"
              " other than 0x0001 and were ignored on purpose. Read the bytes"
              " out of the capture ring: 0x90 may be a family of system"
              " commands and only reboot is implemented.")
    else:
        print(f"  VERDICT: {rq} reboot command(s) accepted. The board reset, so"
              " every counter here is from AFTER that -- which is why this one"
              " reads 0 again on a board that did reboot.")

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
    # THE THREE COUNTERS MEAN THREE DIFFERENT THINGS, AND I HAVE NOW GOT THIS
    # WRONG IN BOTH DIRECTIONS. Written out so the next reader does not have to
    # rediscover it:
    #
    #   ring_underflow  episodes where the level went NEGATIVE -- rd passed wr,
    #                   i.e. samples were emitted that were never written. THIS
    #                   IS THE FAULT. It must be 0.
    #   ring_unprime    episodes where the prime hysteresis dropped out, which
    #                   happens at level < FRAME_SAMPLES -- a LOW level, not
    #                   necessarily a negative one. This is the SAFETY NET
    #                   FIRING, not the fault. It holds rd until wr catches up
    #                   and re-primes at centre in ~11 ms.
    #   ring_level_min  a sticky minimum with no edge behind it, so it also
    #                   records the one sample taken as `en` goes high, when rd
    #                   is still parked at its reset value and the difference is
    #                   modulo 4*SRING_DEPTH. That can read deeply negative on a
    #                   ring that then runs for hours clean.
    #
    # Measured 2026-11-07, two boots of the same bitstream: one showed
    # level_min -2856 with both episode counters 0, the other level_min 0 with
    # ring_unprime 1 -- in both cases level_raw was healthy, audio was flowing
    # and the peak meter was correct. First this function called the first case
    # an inversion, then it called the second one an inversion. Neither was.
    uf = r.get("ring_underflow", 0)
    up = r.get("ring_unprime", 0)
    mn = r.get("ring_level_min", 0)

    if uf:
        print(f"  VERDICT: THE RING INVERTED -- ring_underflow {uf}. rd passed"
              " wr, so samples went out that were never written. The hysteresis"
              " recovers in ~11 ms, but with 48-bit pointers this route is"
              " supposed to be impossible: find out how it was reached.")
        return
    if up:
        print(f"  VERDICT: no inversion (ring_underflow 0), but the prime guard"
              f" fired {up}x -- the level dipped below one frame and rd was held"
              " until wr caught up. ONE episode at the talker's first enable is"
              " expected, because rd starts parked. A count that CLIMBS while"
              " running means the level is grazing the floor and a re-prime will"
              " eventually be audible. Read this twice to tell those apart.")
    elif mn < 0:
        print(f"  VERDICT: no episode at all (ring_underflow 0, ring_unprime 0)"
              f" but ring_level_min is {mn}. That is the sample taken as the"
              " talker was enabled, with rd still parked -- not an inversion"
              " during operation. If level_min keeps dropping while both episode"
              " counters stay 0, the edge detector is what is wrong.")
    else:
        print("  VERDICT: the ring has never inverted and the prime guard has"
              " never fired since boot.")

if __name__ == "__main__":
    board = sys.argv[1] if len(sys.argv) > 1 else _BOARD
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    if board == "--ring" or (len(sys.argv) > 2 and sys.argv[2] == "--ring"):
        if board == "--ring": board = _BOARD
        print_ring(read_ring(board))
        sys.exit(0)
    if board == "--info" or (len(sys.argv) > 2 and sys.argv[2] == "--info"):
        if board == "--info": board = _BOARD
        print_info(read_info(board))
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
