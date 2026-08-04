#!/usr/bin/env python3
"""Stream the board's telemetry ring to CSV, and summarise it live.

    tools/telemetry.py                    # live summary, Ctrl-C to stop
    tools/telemetry.py --csv run.csv      # ...and log every record
    tools/telemetry.py --csv run.csv --secs 3600

WHY THIS EXISTS. Three faults in this project were invisible to snapshot
counters, each costing a bench session:

  * the media clock sat 227 ms in the future for two days while every counter
    read healthy and no audio played -- nothing measured PHASE;
  * a phase servo hunted +/-0.5 ppm in a limit cycle, visible only as `drift`
    "wandering" between reads minutes apart;
  * two wrong diagnoses in one evening, both from inferring behaviour through
    0.5 Hz samples of already-median-filtered values.

So this pulls fixed-size records pushed ON EVENT by the device, parses them BY
NAME against a version tag (never by hand-counted offset -- two wrong
conclusions in this project came from stale offsets), and writes them out for
offline analysis.

WHAT TO LOOK AT:
  * `ptp` rows: raw vs filtered offset side by side. Their difference is what
    the median filter is actually doing. Outliers are flagged, not hidden.
  * `mclk` rows: drift_samples (PHASE) and applied_ppb (RATE) together. A servo
    that is hunting shows as drift oscillating while applied_ppb chases it --
    that signature is what convicted the phase term.
  * `event` rows: lock/unlock, anchor, talker on/off, flow bind/unbind. Levels
    hide events; these are the events.
  * Sequence gaps are REPORTED, not silently skipped. A gap means the ring
    wrapped before this tool drained it.
"""
import argparse
import csv
import socket
import struct
import sys
import time

PORT = 7779
TAG = 0x544C4D31  # 'TLM1'
HDR = 20

T_PTP, T_MCLK, T_EVENT = 1, 2, 3

EVENTS = {
    1: "ptp_lock", 2: "ptp_unlock", 3: "anchor", 4: "talker_on",
    5: "talker_off", 6: "flow_bind", 7: "flow_unbind", 8: "mclk_trip",
    9: "mclk_arm", 10: "mclk_disarm",
}
PTP_F = {0x01: "locked", 0x02: "OUTLIER", 0x04: "no_followup", 0x08: "mispaired"}
MCLK_F = {0x01: "disciplined", 0x02: "phase_on", 0x04: "talker_on"}


def s32(v):
    return v - (1 << 32) if v >= 1 << 31 else v


def drain(board, want, timeout=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(b"t" + struct.pack(">I", want), (board, PORT))
    d, _ = s.recvfrom(1500)
    if len(d) < HDR:
        raise ValueError(f"short reply: {len(d)} bytes")
    tag, rec_bytes, oldest, nxt, dropped = struct.unpack(">5I", d[:HDR])
    if tag != TAG:
        raise ValueError(f"bad tag {tag:#010x} -- is the firmware current?")
    recs = []
    body = d[HDR:]
    for i in range(0, len(body) - rec_bytes + 1, rec_bytes):
        seq, t_ms, packed, v0, v1, v2, v3 = struct.unpack(">7I", body[i:i + 28])
        recs.append({
            "seq": seq, "t_ms": t_ms,
            "type": packed >> 24, "flags": (packed >> 16) & 0xFF,
            "aux": packed & 0xFFFF,
            "v0": s32(v0), "v1": s32(v1), "v2": s32(v2), "v3": s32(v3),
        })
    return oldest, nxt, dropped, recs


def decode(r):
    """Turn a raw record into named fields. The device's layout is documented
    in telem.h; this is the only place that knows it."""
    t = r["type"]
    out = {"seq": r["seq"], "t_ms": r["t_ms"], "kind": "?"}
    if t == T_PTP:
        out.update(
            kind="ptp",
            sync_seq=r["aux"],
            offset_ns=r["v0"],
            filtered_ns=r["v1"],
            path_delay_ns=r["v2"],
            rate_ppb=r["v3"],
            flags="|".join(n for b, n in PTP_F.items() if r["flags"] & b),
        )
    elif t == T_MCLK:
        lvl = r["v2"] & 0xFFFFFFFF
        out.update(
            kind="mclk",
            drift_samples=r["v0"],          # PHASE axis
            applied_ppb=r["v1"],            # RATE axis
            lvl_min=(lvl >> 16) & 0xFFFF,
            lvl_max=lvl & 0xFFFF,
            lvl_avg=r["aux"],
            underrun_per_s=r["v3"],
            flags="|".join(n for b, n in MCLK_F.items() if r["flags"] & b),
        )
    elif t == T_EVENT:
        out.update(
            kind="event",
            event=EVENTS.get(r["aux"], f"id{r['aux']}"),
            a=r["v0"], b=r["v1"],
        )
    return out


COLS = ["seq", "t_ms", "kind", "flags", "event", "a", "b",
        "offset_ns", "filtered_ns", "path_delay_ns", "rate_ppb", "sync_seq",
        "drift_samples", "applied_ppb", "lvl_min", "lvl_avg", "lvl_max",
        "underrun_per_s"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board", nargs="?", default="169.254.9.200")
    ap.add_argument("--csv")
    ap.add_argument("--secs", type=float, default=0, help="0 = until Ctrl-C")
    ap.add_argument("--poll", type=float, default=0.5)
    a = ap.parse_args()

    writer = fh = None
    if a.csv:
        fh = open(a.csv, "w", newline="")
        writer = csv.DictWriter(fh, fieldnames=COLS, extrasaction="ignore")
        writer.writeheader()

    want = 0
    t0 = time.time()
    n_ptp = n_mclk = n_evt = gaps = 0
    off = []
    drift = []
    last_report = 0.0
    print("draining telemetry (Ctrl-C to stop)")
    try:
        while a.secs == 0 or time.time() - t0 < a.secs:
            try:
                oldest, nxt, dropped, recs = drain(a.board, want)
            except Exception as e:
                print(f"  [drain failed: {e}]")
                time.sleep(a.poll)
                continue
            if want and oldest > want:
                gaps += oldest - want
                print(f"  ** GAP: {oldest - want} records lost "
                      f"(ring wrapped before drain; device dropped={dropped})")
            for r in recs:
                d = decode(r)
                want = r["seq"] + 1
                if writer:
                    writer.writerow(d)
                if d["kind"] == "ptp":
                    n_ptp += 1
                    off.append(d["filtered_ns"])
                    if "OUTLIER" in d["flags"]:
                        print(f"  t={d['t_ms']}ms OUTLIER raw={d['offset_ns']} "
                              f"filt={d['filtered_ns']} ns")
                elif d["kind"] == "mclk":
                    n_mclk += 1
                    # Ignore absurd phase values: before the talker anchors,
                    # dbg_last_sec is stale and drift is meaningless. Without
                    # this the live range is dominated by one startup sample.
                    if abs(d["drift_samples"]) < 48000 * 60:
                        drift.append(d["drift_samples"])
                elif d["kind"] == "event":
                    n_evt += 1
                    print(f"  t={d['t_ms']}ms EVENT {d['event']} "
                          f"a={d['a']} b={d['b']}")
            if not recs:
                time.sleep(a.poll)

            now = time.time()
            if now - last_report >= 10:
                last_report = now
                if off and drift:  # both axes, always reported together
                    print(f"  [{now - t0:5.0f}s] ptp n={n_ptp} "
                          f"offset {min(off):+d}..{max(off):+d} ns  |  "
                          f"phase {min(drift):+d}..{max(drift):+d} samples "
                          f"(span {max(drift) - min(drift)})  |  "
                          f"events {n_evt} gaps {gaps}")
                off, drift = off[-200:], drift[-200:]
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()
            print(f"\nwrote {a.csv}")
    print(f"records: ptp={n_ptp} mclk={n_mclk} events={n_evt} gaps={gaps}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
