#!/usr/bin/env python3
"""Drive and measure the Dante media clock, over UDP:7779.

    tools/mclk.py status        # dry run: what discipline WOULD do
    tools/mclk.py watch 60      # drift rate + ring level min/max over a window
    tools/mclk.py on            # arm the discipline
    tools/mclk.py off           # back to nominal, immediately

THE DRY-RUN CHECK, before arming anything:

`target_ppb` is what PTPv1's servo integral says this board's crystal error is.
`drift` is an INDEPENDENT measurement -- the emitted media timestamp minus PTP,
in samples, differentiated over the watch window. If discipline is correct they
must agree in magnitude and be opposite in sign: a crystal running +4.7 ppm fast
should show drift +4.7 ppm and target_ppb about -4700.

If they disagree, the sign or the scaling is wrong and arming would drive the
clock the wrong way twice as fast. That check costs one `watch` and is the whole
reason this reports before it writes.

RING LEVEL is reported as min/avg/max over 1 s windows, sampled at 1 kHz on the
board. Watch MIN, not avg. The previous two attempts at this were read through a
1 Hz snapshot of the same value, which is why "40547 underruns with the level at
centre" looked like a paradox: underrun_count ticks at 48 kHz while the ring is
un-primed, so 40547 ticks is 0.85 s of brief, repeated dips that a slow sample
lands in almost never.
"""
import socket
import struct
import sys
import time

PORT = 7779
TAG = 0x4D434C4B  # 'MCLK'
SIGNED = {"target_ppb", "applied_ppb", "drift_samples"}

FIELDS = [
    "enabled", "ptp_locked", "target_ppb", "applied_ppb",
    "base_inc", "applied_inc", "nco_writes", "trips",
    "lvl_min", "lvl_avg", "lvl_max", "underrun_per_s",
    "drift_samples", "underrun_total", "overrun_total",
]


def query(board, arg=b"", timeout=3.0, tries=4):
    last = None
    for _ in range(tries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(timeout)
            s.sendto(b"m" + arg, (board, PORT))
            d, _ = s.recvfrom(512)
            w = struct.unpack(">%dI" % (len(d) // 4), d[: (len(d) // 4) * 4])
            if w[0] != TAG:
                raise ValueError(f"bad tag {w[0]:#010x}; firmware current?")
            out = dict(zip(FIELDS, w[1:]))
            for k in SIGNED:
                if out[k] >= 1 << 31:
                    out[k] -= 1 << 32
            return out
        except Exception as e:      # noqa: BLE001 - report the last failure
            last = e
    raise last


def show(st):
    print(f"  discipline     {'ARMED' if st['enabled'] else 'off (dry run)'}"
          f"    ptp_locked={st['ptp_locked']}")
    print(f"  target         {st['target_ppb']:+d} ppb"
          f"   ({st['target_ppb'] / 1000.0:+.2f} ppm, from PTP servo integral)")
    print(f"  applied        {st['applied_ppb']:+d} ppb"
          f"   ({st['applied_ppb'] / 1000.0:+.2f} ppm, slew-limited)")
    print(f"  NCO inc        base={st['base_inc']} applied={st['applied_inc']}"
          f"  writes={st['nco_writes']} trips={st['trips']}")
    print(f"  ring level     min={st['lvl_min']} avg={st['lvl_avg']}"
          f" max={st['lvl_max']}   (centre 64, prime floor near 0)")
    print(f"  underrun       {st['underrun_per_s']}/s"
          f"   {'<- ring empty, no USB source' if st['underrun_per_s'] > 40000 else ''}")
    print(f"  drift          {st['drift_samples']:+d} samples (emitted - PTP)")


def watch(board, secs):
    a = query(board)
    t0 = time.time()
    lo, hi = a["lvl_min"], a["lvl_max"]
    worst_ur = a["underrun_per_s"]
    while time.time() - t0 < secs:
        time.sleep(2)
        try:
            r = query(board, timeout=2.0, tries=2)
        except Exception:
            continue
        lo = min(lo, r["lvl_min"])
        hi = max(hi, r["lvl_max"])
        worst_ur = max(worst_ur, r["underrun_per_s"])
    b = query(board)
    dt = time.time() - t0
    slip = b["drift_samples"] - a["drift_samples"]
    ppm = slip / dt * 1e6 / 48000
    print(f"over {dt:.0f}s ({'ARMED' if b['enabled'] else 'dry run'}):")
    print(f"  drift slip     {slip:+d} samples  ->  {ppm:+.2f} ppm"
          f"  ({slip / dt / 48000 * 3600 * 1000:+.1f} ms/hour)")
    print(f"  target_ppb     {b['target_ppb']:+d}"
          f"   ({b['target_ppb'] / 1000.0:+.2f} ppm)")
    print(f"  applied_ppb    {b['applied_ppb']:+d}"
          f"   ({b['applied_ppb'] / 1000.0:+.2f} ppm)")
    print(f"  ring level     min={lo} max={hi}   worst underrun {worst_ur}/s")
    print(f"  underrun total {b['underrun_total']}"
          f" (delta {b['underrun_total'] - a['underrun_total']:+d})")
    print(f"  overrun  total {b['overrun_total']}"
          f" (delta {b['overrun_total'] - a['overrun_total']:+d})")
    print(f"  trips          {b['trips']}")
    # State the check rather than leaving it to the reader.
    if not b["enabled"]:
        agree = (abs(ppm) > 0.5 and
                 abs(abs(ppm * 1000) - abs(b["target_ppb"])) < abs(b["target_ppb"]) * 0.5
                 and (ppm > 0) != (b["target_ppb"] > 0))
        print(f"\n  => dry-run cross-check: measured drift {ppm:+.2f} ppm vs "
              f"PTP estimate {b['target_ppb'] / 1000.0:+.2f} ppm")
        print("     " + ("CONSISTENT (opposite sign, similar magnitude) -- safe to arm"
                         if agree else
                         "DISAGREE -- do NOT arm; check sign/scaling first"))


def main():
    board = "169.254.9.200"
    args = sys.argv[1:]
    if args and args[0].count(".") == 3:
        board = args.pop(0)
    cmd = args[0] if args else "status"

    if cmd == "status":
        show(query(board))
    elif cmd == "watch":
        watch(board, int(args[1]) if len(args) > 1 else 60)
    elif cmd == "on":
        print("armed:")
        show(query(board, b"1"))
    elif cmd == "off":
        print("disarmed, NCO back to nominal:")
        show(query(board, b"0"))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
