#!/usr/bin/env python3
"""Drive and measure the Dante media clock, over UDP:7779.

    tools/mclk.py status        # dry run: what discipline WOULD do
    tools/mclk.py watch 60      # drift rate + ring level min/max over a window
    tools/mclk.py log 3600 f    # 1 Hz drift samples; proves the signal is clean
    tools/mclk.py on            # arm the discipline
    tools/mclk.py off           # back to nominal, immediately

BEFORE ENABLING THE PHASE TERM, run `log 3600`. It is step 2 of
DRIFT_HANDOFF.md: an hour of 1 Hz samples with not one |drift| > 100. The phase
loop ran away once on a drift signal that went whole seconds out, and a control
loop cannot be validated on a signal nobody sampled fast enough to see.

THE DRY-RUN CHECK, before arming anything:

`target_ppb` is what PTPv1's servo integral says this board's crystal error is.
`drift` is an INDEPENDENT measurement -- the media clock counter minus PTP, in
samples, differentiated over the watch window. If discipline is correct they
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
SIGNED = {"target_ppb", "applied_ppb", "drift_samples", "phase_ppb"}

FIELDS = [
    "enabled", "ptp_locked", "target_ppb", "applied_ppb",
    "base_inc", "applied_inc", "nco_writes", "trips",
    "lvl_min", "lvl_avg", "lvl_max", "underrun_per_s",
    "drift_samples", "underrun_total", "overrun_total",
    "phase_enabled", "phase_ppb",
    "drift_valid", "drift_rejects", "phase_rejects",
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
            # Tolerate a firmware older than a field: zip() truncates, so a
            # short reply simply omits the tail rather than crashing the tool.
            for k in SIGNED:
                if k in out and out[k] >= 1 << 31:
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
    # "media - PTP", not "emitted - PTP": since the atomic-measurement fix this
    # is the media clock COUNTER, which every flow's emitted timestamp sits a
    # further (fpp-1) behind. Constant per flow, so the RATE is unchanged.
    stale = "" if st.get("drift_valid", 1) else "   <- STALE, no sample this poll"
    print(f"  drift          {st['drift_samples']:+d} samples (media - PTP){stale}")
    print(f"  phase term     {'ENABLED' if st.get('phase_enabled') else 'off'}"
          f"   contributing {st.get('phase_ppb', 0):+d} ppb")
    if st.get("drift_rejects") or st.get("phase_rejects"):
        print(f"  measurement    {st.get('drift_rejects', 0)} polls with no sample"
              f"  (talker off or media strobe stopped)")
        print(f"                 {st.get('phase_rejects', 0)} rejected by the"
              f" magnitude guard  <- SHOULD BE 0; if not, the signal is wrong again")


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
    # Differentiating a HELD value invents a drift rate. drift_valid says the
    # reading is this second's measurement rather than the last good one carried
    # over because the talker was off.
    if not (a.get("drift_valid", 1) and b.get("drift_valid", 1)):
        print("  drift slip     NOT MEASURABLE -- an endpoint reading was stale")
        print("                 (talker off, or the media strobe stopped)")
        return
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


def log(board, secs, path=None, limit=100):
    """Sample drift at 1 Hz and report every excursion.

    This is step 2 of DRIFT_HANDOFF.md, made runnable: the phase term must not
    be enabled until an hour of samples contains not one |drift| > 100. The
    failure it exists to catch is a whole-second artefact (48000 samples), which
    a single sample per minute would miss entirely -- hence 1 Hz, and hence
    reporting the WORST sample rather than an average, which would bury it.
    """
    t0 = time.time()
    n = bad = stale = 0
    worst = 0
    first = None
    fh = open(path, "w") if path else None
    if fh:
        fh.write("# t_s drift_samples valid applied_ppb phase_ppb lvl_min underrun_per_s\n")
    print(f"logging drift at 1 Hz for {secs}s; excursions past +-{limit} samples "
          f"are printed as they happen")
    try:
        while time.time() - t0 < secs:
            try:
                st = query(board, timeout=2.0, tries=2)
            except Exception:
                time.sleep(1.0)
                continue
            t = time.time() - t0
            d = st["drift_samples"]
            ok = st.get("drift_valid", 1)
            if fh:
                fh.write(f"{t:.1f} {d} {int(ok)} {st['applied_ppb']} "
                         f"{st.get('phase_ppb', 0)} {st['lvl_min']} "
                         f"{st['underrun_per_s']}\n")
                fh.flush()
            if not ok:
                stale += 1
                print(f"  t+{t:6.0f}s  NO SAMPLE (talker off or media strobe stopped)")
            else:
                n += 1
                if first is None:
                    first = d
                if abs(d) > abs(worst):
                    worst = d
                if abs(d) > limit:
                    bad += 1
                    print(f"  t+{t:6.0f}s  drift {d:+d} samples  <- EXCURSION"
                          f"{'  (~1 second: the old artefact is back)' if abs(abs(d) - 48000) < 200 else ''}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("  (interrupted)")
    finally:
        if fh:
            fh.close()

    dt = time.time() - t0
    print(f"\nover {dt:.0f}s: {n} samples, {stale} with no sample, "
          f"{bad} past +-{limit}")
    print(f"  worst |drift|  {worst:+d} samples")
    st = query(board)
    print(f"  guard rejects  {st.get('phase_rejects', 0)}"
          f"   (the magnitude guard in mcr_dante.c; must be 0)")
    if bad == 0 and st.get("phase_rejects", 0) == 0:
        print("  => the drift signal is clean over this window."
              " DRIFT_HANDOFF.md step 2 satisfied for this run.")
    else:
        print("  => NOT clean. Do NOT enable the phase term; the signal is"
              " still wrong.")
    if fh is None:
        print("  (pass a filename to keep the raw samples: mclk.py log 3600 drift.log)")


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
    elif cmd == "log":
        log(board,
            int(args[1]) if len(args) > 1 else 3600,
            args[2] if len(args) > 2 else None)
    elif cmd == "on":
        print("armed:")
        show(query(board, b"1"))
    elif cmd == "phase-on":
        print("phase term ENABLED (experimental - it caused clock artifacts once):")
        show(query(board, b"P"))
    elif cmd == "phase-off":
        print("phase term disabled:")
        show(query(board, b"p"))
    elif cmd == "off":
        print("disarmed, NCO back to nominal:")
        show(query(board, b"0"))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
