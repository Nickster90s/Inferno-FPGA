# Handoff: the +0.35 ppm media-clock drift

Everything a fresh session needs to fix the residual drift. Written 2026-08-11
at commit `842bccb`. Read this before touching `mcr_dante.c`.

> # RESOLVED, 2026-08-12. The drift was NCO INTEGER ROUNDING.
>
> Not a servo error, not a measurement error, and not something a phase loop
> was needed for. `tools/mclk.py log 540` on the board, discipline armed:
>
> ```
> over 541s: 534 samples, 0 with no sample, 0 past +-100
>   least-squares slope  +0.00 samples/hour  =  +0.0000 ppm
> ```
>
> **534 consecutive samples, every one reading exactly -3.** Against +22.5
> samples/hour measured on the same board an hour earlier. Ring 63, zero
> underruns, `anchors` stuck at 1 (the boot anchor), and all three receivers
> parked and stable: A16R 8/12 at 0.25 ms, AM2 21/48 at 1 ms, DVS 88/192 at
> 4 ms. **The phase term is still OFF and is no longer needed to hold rate.**
>
> The cause, the fix and the credit for spotting it are in *"The actual root
> cause"* below. Everything above that section is the investigation as it stood
> before, kept because two of its intermediate findings are real bugs that were
> fixed on the way and would otherwise be lost.

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

### Problem 2 — the drift SIGNAL had 48000-sample outliers (FIXED, needs a bench run)

This was the blocker. `firmware/mcr_dante.c:348-358` used to read:

```c
ptp_timestamp_t t = gptp_read_time();                 // instant A
int64_t ptp_smp = (int64_t)t.seconds * 48000
                + (int64_t)((t.nanoseconds * 3u) / 62500u);
uint32_t esec, esub;
dante_tx_read_emitted(&esec, &esub);                  // instant B
int64_t emit    = (int64_t)esec * 48000 + (int64_t)esub;
drift_samples = (int32_t)(emit - ptp_smp);
```

Measured 2026-08-11, enabling the phase term with the corrected gains:

```
t+0    phase term ENABLED, contributing +0 ppb, drift +2 samples
t+60   phase term contributing +2000 ppb (the rail), drift -335977 samples
```

**Correcting the diagnosis this document originally gave.** It blamed a
one-second read straddle between instants A and B. That mechanism is real, but
it was *already handled* and it cannot explain what was seen:

- `dante_tx_read_emitted()` seqlocks the emitted pair (read sec, sub, sec, retry
  on disagreement), and `gptp_read_time()` is latched in gateware — reading
  `seconds_hi` snapshots seconds and nanoseconds together (`avb_soc.py:153`).
  Both pairs were already atomic when the loop ran away.
- **A straddle can only ever be ±1 second.** `-335977` is `-7 × 48000 + 23`.
  Seven seconds is not a straddle.

The real mechanism is **staleness**. `dbg_last_sec`/`dbg_last_ts` hold the *last
emitted* packet's timestamp, and the media counter behind them is gated on `en`
(`dante_packetizer.py`: `strobe = mcr.sample_strobe & en`). Drop the talker — a
flow expiring, `dante_tx_drop_all()`, the re-anchor path — and the register
freezes while PTP runs on, so the reported drift walks at exactly **−48000
samples per second of silence**. `-47959` is one second of it; `-335977` is
seven. Nothing guarded that.

**The `|err| > 2400` guard is not sufficient, and the sim now proves it.** It
only catches staleness once the freeze exceeds 50 ms, and the talker drops at an
arbitrary point inside the 1 Hz poll interval, so 6 landings in 100 report a
completely wrong number small enough to pass — worth up to 2400 samples of pure
error, which kicks the NCO by 687 ppb for a correction the phase error never
asked for.

### The fix: `dante_tx_media_phase()` (`firmware/dante_tx.c`)

```c
int dante_tx_media_phase(int32_t *out_samples);   // 1 = sample, 0 = none
```

- **No seconds field on either side.** The media side reads the *live* subsecond
  counter (`ts_now_sub`, 0..47999); the PTP side uses only `nanoseconds`. A
  ±48000 error is not expressible. Folding into ±24000 discards nothing real:
  the re-anchor backstop holds the true phase within 8 samples, and an entirely
  undisciplined crystal needs 28 hours to reach half a second.
- **Liveness is proven, not assumed.** A frozen counter does not look like a
  slow ramp — against a running PTP it sweeps the whole ±24000 range once a
  second, so ~10% of readings pass a magnitude guard *with a random sign*
  (measured, see the host test in the commit). The function waits for the
  counter to actually tick and returns **no sample** if it never does.
- **Bounded residual.** The counter is read either side of the PTP read and a
  disagreement retries, so the error is under one sample.

`mcr_dante.c` **holds** the whole phase term on a poll with no sample rather
than zeroing it — zeroing would step the applied rate by whatever the integral
had accumulated. `drift_valid`, `drift_rejects` and `phase_rejects` are now in
the `MCLK` readout, and `phase_rejects` must stay 0: if the magnitude guard ever
fires again, the measurement is wrong again.

**Note the change of reference.** `drift_samples` is now the media clock
**counter** against PTP, not the emitted timestamp, which sits a further
`(fpp-1)` behind — 3 samples at the A16R's `fpp=4`, 59 at DVS's `fpp=60`. That
is deliberate: the counter is fpp-independent, so a receiver rebinding at a
different fpp no longer steps the number a control loop is closing on. Absolute
readings shift by a few samples against anything recorded before this change;
rates do not.

---

## What to do next

1. ~~Make the drift sample atomic.~~ **Done** — `dante_tx_media_phase()`.
2. ~~Prove it on hardware.~~ **Done** — 534/534 clean samples, 0 rejects.
3. ~~Extend the sim.~~ **Done** — check 6 models the measurement itself.
4. ~~Enable the phase term.~~ **Not needed.** The drift it existed to remove was
   integer rounding, now fixed at source. It stays off.

Genuinely outstanding:

- **A long run.** The longest measurement so far is 9 minutes. Leave it
  overnight and confirm drift is still flat and `anchors` has not moved. The
  fix is exact arithmetic, so nothing should accumulate — but this project's
  own history says the difference between an argument and a measurement is
  where the bugs live.
- **`sims/sim_mclk_phase_trim.py` still models `MEASURED_RESIDUAL_PPM = 0.35`,
  which no longer exists.** Re-point it at whatever the long run measures.
- **The `overrun` counter runs at ~200/s (1.27% of 15,848 pps) with fpp 4, 16
  and 60 bound together.** Not investigated. `send_req` arriving while the
  single frame builder is busy, plausibly structural for mixed fpp, but no
  before-baseline was captured and README records a 0 overrun/s run at
  18,000 pps with uniform fpp.
- **`PHASE_TARGET_SAMPLES` is now -4, derived from bench-measured receiver
  constants** (A16R 6, AM2 19, DVS 83, all tracking drift 1:1). If a receiver
  tighter than 0.25 ms ever binds, that and `DANTE_TX_REANCHOR_SAMPLES` must be
  revisited together.

## The actual root cause: the NCO increment is an integer

**Found by asking why `mcr` from the AVB project never drifts.** It has a pure
gPTP clock mode (`cs=0`) alongside its CRF mode, and the pure-gPTP path does not
drift. That question is what cracked this.

The media-clock NCO increment is a 32-bit integer of about 4.12e6, so **one LSB
is 242 ppb**. Two roundings sat inside it, both in the same direction:

| where | what | cost |
|---|---|---|
| `mcr_dante_init()` | `(fs << 32) / sys_clk` = 4123168.604, **rounded up** to 4123169 | **+96 ppb, permanently** |
| `inc_for_ppb()` | truncates the ppb correction toward zero | **+37 ppb** |
| | | **+133 ppb = +0.133 ppm** |

Measured drift before the fix: **+0.13 ppm**. Predicted from the constants
alone: **+0.133 ppm**. It is the whole residual, to two decimal places.

**The PTP servo is structurally blind to it.** `g_ptpv1.rate_ppb` measures this
board's crystal against the Leader. It knows nothing about how coarsely we then
render that estimate into an integer, so the rate loop reports itself converged,
0 trips, healthy ring — while the media clock runs 0.13 ppm fast and every
receiver's measured latency walks down. That is exactly the symptom this
document was written about.

The error is **positive across the entire operating range** (+0.036 to
+0.275 ppm, verified by sweeping ppb), which is why every drift observation in
this project has had the same sign: timestamps early, receiver latency falling,
never the reverse. That consistency should have been a clue much earlier — a
servo error would wander.

### Why `mcr` does not have it

`firmware/mcr.c:219`, the `cs=0` path:

```c
int64_t d    = g->current_addend_full - g->base_addend_full;
int64_t corr = (m->base_increment * d) / g->base_addend_full;
```

It takes the ratio from the **52-bit gPTP addend** (32-bit integer + 20-bit
fraction) and recomputes it in `mcr_watchdog_tick()` at main-loop rate — ~4 kHz
— behind a 2-unit deadband. The addend moves continuously, so the computed
increment **dithers across the grid point** and the quantisation averages to
zero. It gets sub-LSB resolution for free, as a side effect of being fast and
ratio-based.

`mcr_dante` gave that up when it went to one slew-limited write per second: a
single integer, computed once, parked 0.55 LSB off for good.

### The fix

Carry the increment in **16.16 fixed point, unrounded**, and first-order
sigma-delta the fraction — emit N or N+1 at a duty cycle equal to the fractional
part, so the *mean* increment is exact even though every written value is an
integer. `firmware/mcr_dante.c`, `nco_render()`.

Host-verified across the operating range: mean error **under 0.1 ppb**, peak
phase excursion **0.012 samples** inside any one second.

Safe where the gateware fractional NCO was not. Attempt 2 put the fraction in
hardware and dithered **strobe timing at 50 MHz**; the USB feedback servo
measures strobes-per-SOF, so it saw the jitter and the ring starved (level 36,
~9700 underrun/s). This dithers at **1 Hz** between two rates 242 ppb apart —
0.012 samples against a 2048-sample ring, against a startup slew that already
moves 100 ppb every second with zero underruns. The block comment at
`PHASE_MAX_PPB` had reached this same conclusion and named it as the right
approach; it just never got built.

Cost: the NCO is now written about once a second instead of ~17 times total.
Three orders of magnitude below the thousands-per-second path this module
replaced, and the writes are what make the average exact.

### What this means for the phase term — and why it is now ON

The dither fixed the RATE. It did not, and could not, hold absolute POSITION:
with nothing pulling phase back, whatever residual is left integrates freely.
Measured after the dither: drift crept -3 -> -4 over ~40 minutes, about
**1.5 samples/hour** — fifteen times slower than the +22.5 samples/hour before,
but still unbounded, and an A16R at 0.25 ms only has 12 samples to give.

So the phase term was enabled 2026-08-12, and the dither changed its arithmetic
in a way that had to be re-derived first:

**The 242 ppb actuator LSB was why the loop parked 2..5 samples above target**
— the `(LSB/2)/Kp` quantisation floor. `nco_render()` dithers that away, so the
actuator is now effectively continuous and the offset disappears. At the -4
chosen to cancel it, the loop parks at exactly -4 and leaves the A16R at 10 of
12, near the top of its budget. **`PHASE_TARGET_SAMPLES` is therefore -1**, which
parks drift at 0 — A16R at 6 of 12, dead centre, zero steady-state swing.

Measured over the first 5 minutes with it running:

```
  t     drift  phase_ppb  applied_ppb  ring    ur/s  rejects
   15s     -3        +0       -4790  64..67     0  0/13
  120s     -4       +92       -4373  64..66     0  0/13
  300s     -3       +65       -4390  64..66     0  0/13
```

**It contributes 65 ppb against a 2000 ppb clamp** — 3% of its authority, and
0.065 ppm against the USB feedback servo's 20800 ppm. That is the difference the
dither made: before it, the same loop needed ~400 ppb just to fight the rounding
error. Ring 63..67 throughout, zero underruns, zero rejects, `anchors` still 1.

**Converged after ~50 minutes, exactly as the time constant predicted.** The
standing -2 sample error worked out of the P term and into the integral, and the
A16R walked 8 -> 7 -> 6 of 12 and stopped:

```
$ tools/mclk.py log 270
over 270s: 270 samples, 0 with no sample, 0 past +-100
  drift distribution: -1 x270          <- every sample, ON TARGET
  guard rejects  0
```

Settled state: drift **-1**, the target, held for 270 consecutive samples. Phase
term contributing **+16 ppb** — 0.8% of its 2000 ppb clamp, and now almost pure
integral since the proportional term has nothing left to correct. Ring 63..67,
`underrun_per_s` 0 throughout, `anchors` still 1 (the boot anchor — the backstop
has never fired). Receivers parked: **A16R 6/12 at 0.25 ms, dead centre**, AM2
20/48 at 1 ms, DVS 81/192 at 4 ms.

One counter to be aware of: `underrun` total jumped to 15236 across the last
firmware reload and then froze. 15112 ticks is 0.31 s of un-primed ring, which
is consistent with the prime after a reload, and the per-second rate has read 0
in every sample since. It was not observed happening, so it is recorded rather
than explained.

---

## Tools

| what | how |
|---|---|
| clock status, drift, ring | `tools/mclk.py` |
| drift RATE over a window | `tools/mclk.py watch 120` |
| **1 Hz drift log, to prove the signal** | `tools/mclk.py log 3600 drift.log` |
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

## Current state

Updated 2026-08-12 with the measurement fix; the bench numbers below are still
those measured at `842bccb`, before it.

- rate discipline **armed**, `-4354…-4411 ppb`, 0 trips, ring 63-67, 0 underruns
- phase term **off** by default, gains corrected, outlier guard in place
- the drift signal is now `dante_tx_media_phase()` — **not yet run on hardware**
- `DANTE_TX_TS_OFFSET = 0`, `DANTE_TX_REANCHOR_SAMPLES = 8`
- three receivers healthy: A16R `fpp=4` @ 0.25 ms, AM2 `fpp=16` @ 1 ms,
  DVS `fpp=60` @ 4 ms
- the board runs a **volatile** bitstream (`build_seed8/`); a power cycle
  reverts to whatever is in SPI flash
