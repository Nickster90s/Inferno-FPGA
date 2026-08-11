# Handoff: the +0.35 ppm media-clock drift

Everything a fresh session needs to fix the residual drift. Written 2026-08-11
at commit `842bccb`. Read this before touching `mcr_dante.c`.

---

## The problem in one line

With rate discipline armed and healthy, the emitted media timestamp still gains
**+0.35 ppm** on PTP — **1 sample per minute** — so a receiver's measured latency
walks down until it clamps at zero, and the re-anchor backstop fires every ~8
minutes with a timestamp discontinuity each time.

## What it looks like on the bench

A RedNet A16R subscribed at a 0.25 ms latency setting (budget = 12 samples):

```
reported latency  134 µs  →  decaying  →  0  →  Dante Controller shows GREY
```

Grey means the receiver measured zero: our packets arrive at or before their own
timestamp, so `now − timestamp` clamps. It is not a fault — it is the drift
having walked the error past zero — but it costs the health indicator, and if
the drift ever ran the other way the same walk would end in RED.

`anchors` in `tools/stats.py` climbing (5 → 7 in ~30 min was measured) is the
same phenomenon seen from our side.

## The measurement

```
$ tools/mclk.py watch 120
over 120s (ARMED):
  drift slip     +2 samples  ->  +0.35 ppm  (+1.2 ms/hour)
  target_ppb     -4354   (-4.35 ppm)
  applied_ppb    -4354   (-4.35 ppm)
  ring level     min=63 max=67   worst underrun 0/s
  trips          0
```

`drift` is `emitted timestamp − PTP`. **Positive slip means our timestamps run
AHEAD**, which is why a receiver's measured latency *falls*.

The rate loop is doing its job: it removes the crystal's −4.35 ppm. What it
cannot do is remove what it leaves behind, because it has no term that looks at
where the timestamp actually *is*. A small rate error integrates into unbounded
phase error, and only the re-anchor stops it.

---

## The intended fix, and why it is currently unusable

`mcr_dante.c` already contains a phase term:

```
firmware/mcr_dante.c:136   PHASE_TARGET_SAMPLES  (-24)   inferno CLOCK_OFFSET_NS
firmware/mcr_dante.c:157   PHASE_MAX_PPB         2000
firmware/mcr_dante.c:182   PHASE_KI              10      (was 40)
firmware/mcr_dante.c:187   PHASE_KP_NUM          30      (was 2, with DEN 5)
firmware/mcr_dante.c:218   phase_enabled                 DEFAULTS OFF
firmware/mcr_dante.c:439   the PI itself
```

### Problem 1 — the shipped gains could not hold phase (FIXED)

`Kp` was `2/5 = 0.4` ppb per sample. The plant is an **integrator**: 1 ppb
becomes `4.8e-5` samples/s. So `Kp=0.4` is a **14.5 hour** time constant — the
proportional term does nothing on any useful timescale, the integral runs the
loop alone, and it limit-cycles over **20 samples** against a 12-sample budget.

`sims/sim_mclk_phase_trim.py` demonstrates this and searches the gain space.
`Kp=30, Ki=10` settles within 1 sample with a 1-sample steady-state swing, peaks
at 734 ppb against the 2000 clamp, and never exceeds the 100 ppb/s slew limiter.
Those gains are now in the tree.

### Problem 2 — the drift SIGNAL has 48000-sample outliers (NOT FIXED)

**This is the actual blocker.** `firmware/mcr_dante.c:348-358`:

```c
ptp_timestamp_t t = gptp_read_time();                 // instant A
int64_t ptp_smp = (int64_t)t.seconds * 48000
                + (int64_t)((t.nanoseconds * 3u) / 62500u);
uint32_t esec, esub;
dante_tx_read_emitted(&esec, &esub);                  // instant B
int64_t emit    = (int64_t)esec * 48000 + (int64_t)esub;
drift_samples = (int32_t)(emit - ptp_smp);
```

The two reads happen at **different instants**. The `(esec, esub)` pair is
atomic with respect to *itself*, but not with respect to `gptp_read_time()`. So
when the two straddle a one-second boundary, the difference jumps by a whole
second — **48000 samples**.

Harmless for the dry-run cross-check, which differentiates over a long window
and averages it out. **Fatal in a 1 Hz control loop.**

Measured 2026-08-11, enabling the phase term with the corrected gains:

```
t+0    phase term ENABLED, contributing +0 ppb, drift +2 samples
t+60   phase term contributing +2000 ppb (the rail), drift -335977 samples
```

`-47959` was also seen, which is `-48000 + 41` — the fingerprint. It cost ten
re-anchors and an audio interruption before it was switched off.

A guard is now in place (`mcr_dante.c:439`, skip the update when
`|err| > 2400`), so the landmine will not fire on anyone else. **That is a
safety net, not the fix.** The signal is still wrong; the loop just ignores it
when it is obviously wrong.

---

## What to do next

1. **Make the drift sample atomic across the second boundary.** Options, in
   rough order of preference:
   - read PTP and the emitted timestamp inside one critical section, or
   - read `gptp_read_time()` twice, before and after
     `dante_tx_read_emitted()`, and reject the sample if the seconds field
     changed between them, or
   - compute the difference in a way that cannot straddle: subtract the seconds
     fields first and sanity-check the result is in `{-1, 0, +1}` before
     folding in the subsec difference.

2. **Prove it.** Log `drift_samples` at 1 Hz for an hour with the phase term
   still OFF and confirm there is not a single |value| > 100. If any survive,
   the fix is incomplete and the loop must not be enabled.

3. **Extend the sim before re-enabling.** `sims/sim_mclk_phase_trim.py` models
   drift as a clean float — that is exactly why it passed while the real thing
   ran away. Add the second-boundary outlier to the model and show the loop
   survives it, then re-check the gains.

4. **Only then enable**, with nobody listening, and watch
   `tools/mclk.py` for `phase term ENABLED / contributing N ppb`, plus ring
   level and underruns.

---

## Tools

| what | how |
|---|---|
| clock status, drift, ring | `tools/mclk.py` |
| drift RATE over a window | `tools/mclk.py watch 120` |
| arm / disarm rate discipline | `tools/mclk.py on` / `off` |
| enable / disable phase term | `tools/mclk.py phase-on` / `phase-off` |
| runtime counters | `tools/stats.py` (`anchors`, `fifo_level`, `underrun`) |
| **what receivers actually measure** | `tools/dante_latency.py <pcap>` on an 8708 capture |
| true on-wire timestamp lag vs PTP | `tools/ts_lag.py <pcap>` |
| timestamp offset, live | UDP `'o'` + signed decimal, e.g. `o-2` |
| flash firmware only | `./flash.sh --fw-only` |

Capture for `dante_latency.py`:
`sudo tcpdump -i ens5 -nn "udp port 8708" -w hb.pcap`

---

## Things that will waste your time if you do not know them

- **Transmit-side counters stay green through total audio loss.** This happened
  three separate times in one session: a frame-size bug where `packet_count` was
  perfect, an AM2 degrading to red while our side ran 48,001 pps with zero
  overruns, and a ring that lost prime while `rx_beats == ep_out` with 0% leak.
  **Only `fifo_level` and the receivers' own reported latency are trustworthy.**

- **`DANTE_TX_TS_OFFSET` is a red herring for this.** It biases where the walk
  starts, not whether it walks. Measured: offset 0 → A16R reads 0/12 (grey, full
  margin); −2 → 8-10/12; −4 → 9-11/12 (one excursion from red). Changing it also
  calls `ts_anchor()`, which is a timestamp discontinuity — do not do it while
  anyone is listening.

- **`DANTE_TX_REANCHOR_SAMPLES` is now 8** (`firmware/dante_tx.h:149`), reduced
  from 24 because 24 samples is 0.5 ms — double a 0.25 ms receiver's entire
  budget. It makes each discontinuity smaller; it does not stop them.

- **The ring can lose prime and never recover**, with data still arriving at the
  correct rate (`fifo_level 0`, `underrun 48001/s` — one per sample tick). A
  firmware reload recovers it. Trigger not established; suspected to be repeated
  `ts_anchor()` / `dante_tx_drop_all()` churn. See README Open bugs.

- **`mclk.py phase-on` prints its own warning** that it caused clock artifacts
  once. That warning was correct and was ignored. Do not repeat that.

---

## Current state at `842bccb`

- rate discipline **armed**, `-4354…-4411 ppb`, 0 trips, ring 63-67, 0 underruns
- phase term **off** by default, gains corrected, outlier guard in place
- `DANTE_TX_TS_OFFSET = 0`, `DANTE_TX_REANCHOR_SAMPLES = 8`
- three receivers healthy: A16R `fpp=4` @ 0.25 ms, AM2 `fpp=16` @ 1 ms,
  DVS `fpp=60` @ 4 ms
- the board runs a **volatile** bitstream (`build_seed8/`); a power cycle
  reverts to whatever is in SPI flash
