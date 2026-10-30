#!/usr/bin/env python3
"""Read the USB->ring path, for diagnosing the ring-loses-prime wedge.

    tools/usbdiag.py [board]

Run it WHILE THE BOARD IS WEDGED. If the board is running the older firmware
the last two fields are absent; flash with `./flash.sh --fw-only`, which leaves
gateware state (and therefore the wedge) intact -- proven twice, 2026-08-12 and
2026-08-13.
"""
import socket, struct, sys, time

FIELDS = ["tag", "rx_beats", "ep_out", "overrun", "underrun",
          "sample_overflow", "fb_ovr", "block_level",
          "head_valid_of_256", "head_raw",   # these two: see the note below
          "frame_holdover", "rx_ts_resyncs", "rx_pops", "ts_ring_level"]


def read(board):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    s.sendto(b"u", (board, 7779))
    d, _ = s.recvfrom(512)
    w = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
    return dict(zip(FIELDS, w))


def main():
    board = sys.argv[1] if len(sys.argv) > 1 else "169.254.9.200"
    a = read(board); t0 = time.time(); time.sleep(5.0); b = read(board)
    dt = time.time() - t0
    for k in FIELDS[1:]:
        if k not in b:
            continue
        if k in ("block_level", "head_valid_of_256", "head_raw"):
            print(f"  {k:18} {b[k]:>14,}")
        else:
            print(f"  {k:18} {b[k]:>14,}  {((b[k]-a[k]) & 0xffffffff)/dt:>12,.1f}/s")

    print()
    ing = ((b["rx_beats"] - a["rx_beats"]) & 0xffffffff) / dt
    ur  = ((b["underrun"] - a["underrun"]) & 0xffffffff) / dt

    # NO VERDICT ON head_valid_of_256. Measured 2026-08-13, it reads 0 on a
    # HEALTHY board (ring 66, 0 underruns, nominal ingress) and 1/256 on a
    # wedged one -- identical, so it discriminates nothing. usb_sample_data is
    # a FIRMWARE DRAIN interface and this design drains the FIFO in hardware at
    # 48 kHz, so the CPU-visible head is essentially never valid either way.
    # The fields are still printed because they cost nothing, but a verdict was
    # built on them before a healthy baseline was taken, and it fired on a
    # healthy board. Do not reinstate one without a baseline.
    rs = b.get("rx_ts_resyncs")
    if rs is not None:
        d = (rs - a.get("rx_ts_resyncs", rs)) & 0xffffffff
        pops = (b.get("rx_pops", 0) - a.get("rx_pops", 0)) & 0xffffffff
        print(f"  RX TS RING: {rs:,} resyncs ({d/dt:,.2f}/s), level {b.get('ts_ring_level')}, "
              f"pops {pops/dt:,.0f}/s")
        if d:
            print("    NONZERO = the RX timestamp ring is falling out of lock-step with")
            print("    frame consumption. Frames then carry the PREVIOUS frame's timestamp,")
            print("    which injects microseconds into every PTP offset with NO packet loss.")

    hold = b.get("frame_holdover")
    if hold is not None:
        dh = (hold - a.get("frame_holdover", hold)) & 0xffffffff
        if hold:
            print(f"  FRAME MARKER LOST: {hold:,} synthesised frame starts "
                  f"({dh/dt:,.1f}/s right now).")
            print("    This is the fault that wedged the ring three times before")
            print("    the flywheel existed. Audio should still be flowing -- if it")
            print("    is, the fix is working and this counter names the cause.")
        else:
            print("  frame marker healthy (0 synthesised frame starts).")

    if ing < 1000:
        print("  no USB ingress -- the host is not sending. Not the ring-wedge path.")
    elif ur > 40000 and b["block_level"] == 0:
        print("  WEDGED: ingress %,.0f B/s, ring 0, underrun every strobe." % ing
              if False else
              f"  WEDGED: ingress {ing:,.0f} B/s, ring 0, underrun on every strobe.")
        print("    underrun on every strobe => primed=0 => rd is FROZEN.")
        print("    rd frozen and level still 0 => wr is frozen too: nothing is")
        print("    reaching the ring. sample_overflow 0 => the FIFO write port")
        print("    is not dropping either, so the decoder is not even trying.")
        print()
        print("    TRY THIS BEFORE a bitstream reload -- it has never been tested,")
        print("    because `talker` stayed 1 through all three wedges so `en`")
        print("    never cycled:")
        print("        tools/stats.py <board>        # confirm talker == 1")
        print("        # cycle the talker (en), then re-run this tool")
        print("    If the ring re-primes, the wedge is clearable in firmware and")
        print("    does not need JTAG. If not, only a bitstream reload clears it.")
    else:
        print(f"  USB->ring path healthy: ingress {ing:,.0f} B/s, block_level "
              f"{b['block_level']}, underrun {ur:,.0f}/s.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
