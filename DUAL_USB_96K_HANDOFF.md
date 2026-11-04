# Dual USB + 48/96 kHz — where this stands

Written 2026-10-30, at the end of the session that brought both PHYs up and
made the sample rate selectable. Companion to `DRIFT_HANDOFF.md`, same purpose:
everything a fresh session needs, including the things that are **not** solved
and the explanations that were tried and killed by measurement.

Read `README.md` §"Dual USB: the build that works" and §"USB3300 ULPI pinouts"
first for the mechanics. This file is the state and the open ends.

---

## Where we are

**Dual USB works.** Both USB3300 PHYs enumerate at 480 Mbit/s and the host sees
two distinct, correctly named cards:

```
 2 [Main   ]: USB-Audio - N-Series Dante Main    at usb-...-8   1209:eab1  serial 0
 3 [Backup ]: USB-Audio - N-Series Dante Backup  at usb-...-10  1209:eab2  serial 1
```

**96 kHz works, and the data path is now measured, not inferred.** Both cards
report `Format: S32_LE  Channels: 48  Rates: 96000`, and under a real
48-channel stream, two minutes continuous:

```
rx_beats 18,431,959/s   ep_out 18,431,959/s   leak -0.01%
overrun 0/s             underrun 0.0/s        block_level 65 (target 64)
ingress exactly nominal (18,432,000 B/s)
```

> **The `overrun 0/s` above does not reproduce, and that is a counter problem,
> not a regression.** On 2026-11-03, on a stream measured as healthy by every
> other number, `overrun` ran at 11k-28k/s. `overrun_count` summed a BENIGN
> event (`send_req` landing mid-traversal, which `pend[]` re-serves) with an
> AUDIBLE one (a USB sample dropped on a full ring). It is now two CSRs:
> `overrun_count` (benign, and load-dependent -- it doubled while builds ran on
> the host and `packets` did not move) and `write_drop_count`, which is the one
> that means lost audio and must read 0. Read it with `tools/usbsrc.py`.
> Whether `write_drops` is genuinely 0 is **not yet proven** -- the bitstream
> that separates them is the one being built now.

Getting there took finding **five** constants that were tuned as absolutes on
48 kHz and had to be relative: the NCO increment, the Dante timestamp
arithmetic, the packetizer's subsecond wrap, the feedback clamp
(`FB_LO/FB_HI`, which pinned 96 kHz feedback at 6.5 samples/µframe so Linux
rejected it and ran open-loop), and the feedback gains (`err<<6`, half the
relative authority at 96 kHz). Plus one that was not a constant at all: LUNA's
iso OUT endpoint `buffer_size`, defaulted to two transactions while 96 kHz
delivers three, silently discarding the third.

**That is the pattern to expect for the next one: something tuned as an
absolute value on one rate that should have been relative to it.** Enumeration
and descriptors tell you nothing about any of them — all five were found by
playing audio and reading `rx_beats` against `ep_out`.
The rate is chosen on the Dante side, persisted in `cfg.sample_rate`, and
applied live — `g_fs`, the media-clock NCO, the packetizer's subsecond wrap and
the USB clock source are all rewritten without a reset. Switched 48↔96 three
times in a row with the board staying up.

**Measured, 8 power cycles per PHY, taps from firmware:** Main 8/8, Backup 8/8.

### The build

```bash
# descriptors first -- a rate/identity change alters the netlist
tools/gen_usb_subsystem.py --dual-rate

SYS_PERIOD_NS=20 BOARD=breakout USB2=1 USB_DUAL_RATE=1 \
  NEXTPNR_USB_FP_LIGHT=1 ./build.sh --seed 7

./flash.sh --bit build/colorlight_i9plus/gateware/colorlight_i9plus.bit
```

Three switches each matter and each was a dead end first:

| | why |
|---|---|
| `BOARD=breakout` | the fitted carrier. The maps were hardcoded to the July `ext` board, which broke the **single** USB too |
| `NEXTPNR_USB_FP_LIGHT=1` | the full floorplan box is infeasible: 11 seeds stall with no error, and `PLACER=sa` names the reason (`failed to place chain ... PACKED_CARRY4`) |
| `--seed <n>` | the 60 MHz ULPI domain is tight. Only 4 of 11 seeds clear all five clock requirements |
| `USB_DUAL_RATE=1` | sizes the ISO endpoint for 96 kHz and exposes `rate_48k_i` |

---

## What we are running into

### 1. ~~The clock-source class requests fail on Main~~ — FIXED 2026-11-03

```
usb 1-8: 1:1: cannot get freq (v2/v3): err -71
usb 1-8: parse_audio_format_rates_v2v3(): unable to retrieve number of sample rates (clock 1)
```

**It was the IDELAY tap, and it was self-inflicted.** The tap had been set to
25, the middle of an eye measured with "does the device enumerate". That
criterion is too coarse — it only says the first few control transfers got
through. Re-measured with the clock-source request as the criterion, 6 power
cycles per tap:

| tap | enumerates | failures |
|---|---|---|
| 22 | 6/6 | 2 |
| **25** (what we had set) | 6/6 | **5** |
| 28 | 6/6 | 0 |
| 30 | 6/6 | 0 |
| 31 | 6/6 | 0 |

So the eye for **correct transfers** is 28..31 while the eye for *enumerating*
is 22..31. Picking the middle of the wider one landed outside the narrower one.
Tap 30 — the value that had been in the gateware reset before I "improved" it —
is right. Now **10/10 enumerations, 0 failures**, and both PHYs 8/8 with both
ALSA cards at 48ch @ 96000.

`tools/tapsweep.py` now scores on these kernel messages by default
(`--criterion errors`, `--trials 3`), because the old criterion would lead the
next session to exactly the same wrong conclusion.

Four explanations preceded this and each was killed by a measurement — they are
listed under "ruled out" below. The one that finally isolated it was the
operator's: swap the two cables. The failure followed the **device**, not the
port (port 1-8 went from 5/9 failures with Main to 0/8 with Backup), which
removed the host side and left only the per-PHY settings.

### 2. The USB device does not appear in a Linux desktop's sound settings

On a second machine the kernel enumerates it and ALSA has the card, but
Elementary OS's sound panel does not list it. Our 48 channels carry
`bmChannelConfig = 0` (discrete, no spatial positions — the honest choice for
arbitrary Dante streams), so PipeWire builds no stereo-like profile and the
device belongs in **Pro Audio**, which simple panels skip. Check with
`pactl list cards | grep -A25 Backup`; if the only profiles are `off` and
`pro-audio`, that is the whole story, and `pactl set-card-profile <name>
pro-audio` exposes the 48 channels to pro tools (Carla, qpwgraph, Ardour).

Not expected to matter for the production target: macOS + QLab handles a
48-channel single-rate device through Audio MIDI Setup.

*(A first hypothesis — PipeWire's `default.clock.allowed-rates` not containing
96000 — is weakened: Inferno DVS runs at 96 kHz on that same machine.)*

### 3. ~~Bench bandwidth~~ — the limit I claimed does not exist

I had written that two PHYs streaming 48ch/96k at once "does not fit one host
controller", on the arithmetic that each reserves 2880 B/microframe = ~23 MB/s.
**Measured 2026-11-03 and it is wrong:** both stream simultaneously on this
bench, on one controller, bus 001:

```
Main   18,446,844 B/s  +0.08% vs nominal  leak +0.00%  LIVE
Backup 18,446,824 B/s  +0.08% vs nominal  leak +0.00%  LIVE
```

36.9 MB/s together. The error was comparing a per-endpoint reservation against
the controller's total as if 2 × 23 had to fit in ~24; high-speed isochronous
has roughly 48 MB/s of budget, and 2 × 2880 B per microframe fits inside it.

What does remain true: many USB hubs and some older xHCI controllers refuse
high-bandwidth isochronous endpoints altogether. Plug into a motherboard port,
not a hub or a dock.

### 4. ~~The A/B mux has never been exercised end to end~~ — DONE 2026-11-03

Exercised in both directions at 96 kHz with both sources live:

```
to Backup:   select 1  live 1  handovers 1
back to Main: select 0  live 0  handovers 2
```

Both sources kept streaming at +0.08% with zero leak, `block_level` stayed at
target (65-66) and `overrun` stayed 0. The first handover showed one brief
underrun burst (412/s, back to 0 immediately) — that is the frame-aligned
commit waiting a frame for the candidate's channel-0 marker, which is what it
is for. The second was clean.

The prerequisite is the one that made this look broken before: the handover
cannot complete unless the candidate is **actually delivering frames**. With
nothing streaming to it, `select` moves and `live` does not, permanently — the
gateware protecting the audio, not a fault. Start a stream to the standby
first (`aplay -D hw:<Backup>,0 -c 48 -r 96000 -f S32_LE ...`) and check its
ingress is at nominal before switching.

**Three more things verified on hardware later the same day**, all of which had
until then only been reasoned about:

- **The forced handover works on a DEAD source.** With the Backup stream killed
  and its ingress at 0, `/usb/backup` over OSC still moved the mux: `handovers
  3, forced 1`. That is the behaviour that was asked for -- you do not switch
  for fun, so when you say go, it goes, and the `forced` counter tells you
  afterwards that the candidate offered no frame marker.
- **OSC works under a live double stream.** Both PHYs at 48 ch / 96 kHz
  (18.47 and 18.44 MB/s, 36.9 MB/s total on one controller, leak ~0) while
  `/usb/backup` and `/usb/main` were sent on UDP 9000. Aligned, `forced 0`.
- **The PHY hold fixes the enumeration-before-firmware race, provably.** The
  gateware default is 48 kHz; the descriptor advertised `Rates: 96000` only.
  That can only happen if the PHY left reset *after* firmware applied the rate
  stored in `cfg.sample_rate`. Before the hold, the host read the 48 kHz
  default and the rate change needed a replug.

### 5. ~~Builds were a lottery~~ — SOLVED 2026-11-03, root cause found

Every hard build session on this project ended in "try more seeds". The user's
read was that something underneath was wrong rather than that the placer was
moody, and that was correct.

`SoCCore(ident_version=True)` makes LiteX append `build_time()` -- date **and
time to the second** -- to the ident string and bake it into the `identifier`
memory. That lands in `colorlight_i9plus_mem.init` and is pulled in by
`$readmemh`, so yosys constant-folds it. Two builds started in different
seconds get **different netlists**:

| | hunt build | serial build |
|---|---|---|
| ident ROM tail | `...19:01:16` | `...19:27:51` |
| cells | 20496 | 20443 |
| names in common | | ~11500 of 20496 |

Everything else was byte-identical: RTL, `.ys`, loader ROM, `csr.h`, the XDC
periods. And yosys itself is deterministic -- run twice on identical input in
the same directory it produced a **byte-identical 36 MB JSON**.

So a seed only names a result for a given netlist. This is the single
explanation for: the 52-65 MHz scatter on "one seed", sweeps that never
predicted real builds, `build_seed8` being irreproducible, and
`floorplan_usb.py` constraining 1419 / 1427 / 1436 cells on "the same" design
(it matches on cell names, and the names moved).

Fixed with `ident_version=False` (long comment at the call site). Verified: two
generations two seconds apart now give byte-identical `.init` files and a `.v`
differing only in its date comment.

What this buys, beyond calm: a **P&R-only sweep from one fixed `.json` is a
valid predictor again**. Copy `colorlight_i9plus.json` + `.xdc` into a scratch
dir, vary only `--seed`, keep `--pre-place floorplan_usb.py --freq 125
--timing-allow-fail`, and parse the LAST timing report per clock. That is
minutes per seed instead of a full build, and it now agrees with the real build
because the real build synthesises the same netlist.

### 6. The left IO column is oversubscribed — fourteen P&R runs, no closure yet

Read this before trying a floorplan change; it is the whole search space I
covered on 2026-11-03 so nobody repeats it. All runs are P&R-only on ONE fixed
netlist (valid now that the netlist is reproducible — see item 5), seed varied,
`--freq 125`, last timing report per clock.

Required: sys 50, usb 60, usb2 60, eth_rx 125, eth_tx 125 MHz.

| config | seeds | eth_rx | eth_tx | usb | usb2 |
|---|---|---|---|---|---|
| USB box X0..48 Y0..156 (shipping default) | 1, 2 | 103.5 / 124.7 **FAIL** | 150 / 137 | 70 / 71 | 66 / 64 |
| same | 3, 7, 8 | not finished in 30 min (3 = stall, 7/8 = slow) | | | |
| USB box capped Y0..50, 1386 cells | 8, 2 | **140 / 161** | 144 / 138 | 58 **FAIL** / 65 | 47 / 48 **FAIL** |
| USB box Y30..52 (Main's own pad band) | 8, 8w, 2 | genuine STALL, all three | | | |
| capped Y0..50, pad-nets only (716 cells) | 2 | 159.6 | 138.7 | **34.3 FAIL** | 57.5 FAIL |
| eth-RX box X50..75 Y50..95 (508 cells) | 8 | 121.5 FAIL | **95.8 FAIL** | **78.9** | **68.1** |
| whole MAC box X2..45 Y52..110 (1487 cells) | 8, 2, 1 | genuine STALL, all three | | | |

**What the numbers mean.** Three blocks compete for the same left IO column and
whichever gets displaced is the clock that fails — which is exactly why one
clock failed per build and a different one each time. The pads are fixed and
measured: USB at X1 Y1-49 (Main Y23-49, Backup Y1-32), ethernet at X1 Y51-72
(TX Y51-54, RX Y55-72).

- eth_rx is **geometry, not seed luck**: 103-124 with USB holding the full
  column, 140-161 the moment USB vacates Y51+. Its critical path is 1.1 ns
  logic against 6.9 ns routing, through one net
  (`liteethmaccrc32checker_fifo_out`, fan-out 48) whose users sat tens of tiles
  from their driver — the same net on every failing seed.
- usb/usb2 are the same story mirrored: 78.9 / 68.1, the widest margins
  measured all day, as soon as the eth RX cells left their half.
- RX and TX share the MAC core, so boxing RX alone **tears TX apart**
  (140 -> 95.8). Box the MAC or neither.
- The column below Y50 (X0..48 = 2499 tiles) holds about 1375 cells at the
  density HeAP can legalise. Main needs all of that on its own (with fewer
  cells `usb` collapses to 34.3), so Main and Backup **cannot both** live below
  the eth pads. Measured three ways.
- Density is necessary but not sufficient: the whole-MAC box at 0.57 cells/tile
  stalled on all three seeds, so box SHAPE and the CARRY4 chains matter too.

**Where to go next, in order of expected value.** (a) The MAC box at
`X2..45 Y52..110` is the right idea at the wrong size — it is the only layout
that matches the pad geometry, and it stalled on area. Try it WIDER in X
(regions may overlap; cfgflash already spans X50..114 with 29 cells) rather
than abandoning it, e.g. `NEXTPNR_ETH_RX_REGION=2,52,70,120`. (b) Shorten the
path instead of the route: that fan-out-48 net is LiteEth's RX async-FIFO
pointer compare folded into the CRC checker's stream gating, and pipelining it
is the classic fix — see `LITEETH_PATCHES.md`, patching LiteEth is already
normal here. (c) Only then hunt seeds, which is now a legitimate predictor.

**Knobs, all OFF by default so the shipping build is unchanged:**
`NEXTPNR_ETH_RX_REGION="x0,y0,x1,y1"`, `NEXTPNR_ETH_RX_SUBSTR="a,b,c"`,
`NEXTPNR_USB_REGION`, `NEXTPNR_USB_FP_LIGHT_YMAX`, `NEXTPNR_USB_FP_CTRL`.

**Method notes that cost time.** `rc=124` means two different things and they
must be told apart: a run stuck AT `Running main analytical placer` with zero
`type ALL: wirelen solved` lines has **no legal solution**, while one that is
iterating is merely **slow** (seeds 7 and 8 iterate for over 30 min with the
full box). `$SCRATCH/report.py` prints which. And P&R here is ~5-25 min
depending on seed and box on a 4-core machine, so run at most 3 and never
conclude from a timeout under contention.

### 7. The 3-hour wedge is still open

Deferred by the operator until dual USB worked. The mechanism was identified to
0.3% (a 32-bit ring pointer wrapping at 2³²/384000 = 3.107 h) and fixed, and
the wedge **recurred with the fix in the running bitstream**. A full audit found
no remaining 2³² hazard. Unexplained.

Corroboration from the sibling receiver project (`Inferno-ESP32-P4`), which
lists our bug among its lessons: *"64-bit sample indices everywhere; the FPGA's
32-bit ring pointer wrapped after 3.1 hours."* Their fix is structural — 64-bit
indices throughout — rather than patching one wrap.

---

## The floorplan work that came out of chasing it

Not the fix, but a real improvement that stands: the `cd_usb` critical path is
LUNA's `get_descriptor` byte counter and its carry chain, and that net was
running across the die — source (70,138), sink (33,100), 2.4 ns of routing on a
0.327 ns budget. Pulling `get_descriptor` into the USB box raised `usb` from
62.23 to 70.70 MHz.

| cell set | cells | places | usb |
|---|---|---|---|
| ULPI pad nets only | 690 | every seed | ~62 MHz |
| **+ `get_descriptor`** | **1335** | **seeds place** | **70.7 MHz** |
| + whole `USBControlEndpoint` | 2276 | **only seed 7** | 80.5, but eth_rx 117 ✗ |

`NEXTPNR_USB_FP_CTRL` selects the set. The wide version is both tighter and
unnecessary — the last ~950 cells buy under a MHz and cost eth_rx its corner
plus eleven of twelve seeds. Y-capping the box below the eth pads does not work
(a rectangle cannot avoid a 22-row band and stay feasible); eth_rx is recovered
through the seed, which swings it 109–173 MHz.

## How we thought we would fix the clock-source failures

The critical path in `cd_usb` is **measured**, not guessed. From the build log:

```
Source StandardRequestHandler.get_descriptor.bytes_sent[2].Q
  Net ... budget 0.327 ns, route 2.4 ns, (70,138) -> (33,100)
  Sink  ... $PACKED_CARRY4$.S2        -> a CARRY4 chain
```

So it is LUNA's `get_descriptor` byte counter and its carry chain — exactly what
`avb_soc.py` already said it was. And that net runs **across the die**: source
at (70,138), sink at (33,100), 2.4 ns of routing against a 0.33 ns budget.

That is the light floorplan being too light. It pulls only the nets touching the
ULPI **pads**, because the floorplan was added in May for pad timing — but the
binding path is in the control endpoint, which was left free to scatter.

**The change under test:** `NEXTPNR_USB_FP_LIGHT=1` now pulls two groups, the
ULPI pad nets *and* `USBControlEndpoint`. That is the middle ground between the
light box (~690 cells) and the full box (~3000, which stalls HeAP).

**Success criterion:** `usb` above ~67 MHz, the figure at which Backup records
zero failures. Then re-count the failures per PHY; if Main goes to zero, the
correlation was causal and this is closed.

**If `usb` does not move:** the domain is not placement-limited and the counter
itself is next — but read the critical path again first, because it may have
moved to a different net.

### Things already ruled out for this symptom

| tried | result |
|---|---|
| iso bandwidth | fails with Backup disabled too |
| a sample-rate dependence | 0/6 at both 48 and 96 kHz |
| `g_fs` reading 0 | the whole UDP stats path was returning 200 zero bytes; the board was wedged |
| the unsynchronised rate CSR | real defect, fixed (`MultiReg` per target domain) — but the failures continue |
| registering `cur_rate` off the critical path | **not** the critical path, per the log above. The attempt also broke twice on hazards the upstream file already documents: `m.d.sync` is not the usb domain there, and an `m.d` statement breaks the `If/Elif` chain |
| relaxing the sys constraint (`SYS_PERIOD_NS=20`) | `usb` 62.23 → 62.37 MHz, i.e. nothing. It did help `usb2` (67.59 → 72.32) and `sys` stays fine, so it is kept |
| more `usb` margin (62.23 → 70.70 MHz via the floorplan) | **the failure ratio did not move**: still 5 of 7 on Main, 0 of 8 on Backup. This is what retired the Fmax hypothesis |
| the hub port / the cable (operator's test) | the failure followed the **device**, not the port — host side ruled out |

**What it actually was:** the tap, measured with the wrong criterion. See §1.

---

## Then, in order

1. Close the clock-source failures (above).
2. **Turn on the Dante Controller menu.** `firmware/dante_info.c` already
   advertises 48000 and 96000 and handles the `0x81` SET; capability byte
   `0x17` is `0xdb`. Expect one correction round: our `0x0080` header carried
   `0` where an AM2 puts the rate (offset 8) and `0` where it puts `2`
   (offset 12) — fixed, but unverified against a real controller. If the menu
   does not appear, capture an AM2 and diff rather than guessing.
3. **Exercise the A/B mux** (§4 above).
4. **The 3-hour wedge** (§5), with the sibling project's 64-bit-indices
   approach as the reference.
5. Port the sibling project's **low-edge PTP filter**. It replaced the median +
   outlier gate that this project still runs, on the reasoning that queueing
   only ever makes a Sync late, so the true offset is the low edge of the
   scatter. Their host test on identical recorded jitter: max offset 25–53 µs →
   2.4–4.5 µs, rms 3.5–6.6 → 0.2–0.5 µs, lock losses 2–4 → 0.

---

## Traps that cost real time here

Each of these is in the source as a comment too; they are collected because
they are about *method*, and the next session will meet them again.

**A `VAR_LOAD` IDELAY CSR read-back is not the hardware.** The tap only reaches
the delay line when the load register is written; until then the delay holds
`IDELAY_VALUE = 0`. Telemetry reported tap 30 while the hardware sat at 0, so
Main failed after every boot and worked during a tap sweep (which loads each
step). This looked like intermittent hardware and was explained three different
wrong ways. `firmware/main.c` now writes tap-then-load at boot for both buses.
An unloaded tap and a shifted eye look identical from outside — rule the first
out before concluding the second.

**The tap values live in firmware, not in the gateware reset.** Changing one is
then a netload of seconds instead of a build that also re-rolls placement and
costs a second measurement.

**A hanging placer has no legal solution.** HeAP never reports infeasibility —
it re-solves forever and looks hung. `PLACER=sa` terminates and names the cell.
Run that before resizing a floorplan box.

**A P&R-only seed sweep is a shortlist, not a prediction.** It is five times
faster than full builds (synthesis is seed-independent), but the netlist must be
the one the final build produces, and `build.sh` rebuilds the loader every run.
A sweep taken from a build that printed `STALE LOADER` measures a netlist that
no longer exists. Every sweep this session over-predicted `usb` by 3–5 MHz.
**Decide on the real build's numbers.**

**Rank seeds by the worst margin across all five clocks.** The binding clock
changes per seed: seed 9 had the best `usb` of twelve and failed `eth_rx` at
109.71 MHz against the 125 gigabit RGMII needs. Sorting on the clock that was
the problem last time nearly shipped that.

**A rate change must not reset the SoC.** The firmware lives in coderam, not
flash, so a reset lands in the loader and the board is gone — netload could not
reach it either, only a bitstream reload.

**`cfg_t`'s layout is load-bearing.** `cfg_load()` requires
`t.size == sizeof(cfg_t)`, so a new field comes out of `reserved`. It must also
not move existing fields: a first attempt kept `sizeof` at 120 and still shifted
`mclk_ppb_valid` from offset 60 to 64, which would have silently lost the
media-clock warm start on every stored blob.

**`build.sh`'s stall watchdog used to `pkill -9 -f nextpnr-xilinx`**, killing
every nextpnr on the machine — it took out a parallel 20-minute run and reported
only `Killed`. Now scoped to its own `cwd`. The same unscoped-pattern mistake
then killed the shell that was running the kill. Kill by PID.

**`tools/gen_usb_subsystem.py` writes atomically now.** It used to open the
output before elaborating, so a generator error left a 0-byte `.v` and the next
build failed with `Module 'usb_avb_subsystem2' is not part of the design` —
a message that says nothing about the real cause.

**`flash.sh` pointed at the single-USB bitstream, and the dual-USB one lived in
a temp dir.** Found 2026-11-04. `BIT_VALIDATED` was still
`build_seed8/gateware/colorlight_i9plus.bit`, which has no `usb2` in its XDC —
it is the single-PHY build that was validated in its day. So `./flash.sh`
without `--bit` quietly put one PHY on the board. Meanwhile the bitstream that
actually passes dual USB at 96 kHz existed only as a copy in a session
scratchpad, one cleanup away from gone. It is now
`bitstreams/dual-usb-96k_2026-11-03_seed8_VERIFIED.bit` (md5
`fd9a5ca00238aa4365208b18946fc8ae`) with a `.info` beside it recording the
timing and the hardware measurements, and `BIT_VALIDATED` points there. Both
are gitignored by the existing `bitstreams/*` rules — they are local artifacts,
like their neighbours.

**The firmware must be compiled against the bitstream's OWN csr.h, and the
build directory does not guarantee that.** This nearly went into SPI on
2026-11-04. In `build/colorlight_i9plus/` the `.bit` was 19:27:50 while
`csr.h`, the `.v` and the `.xdc` were 19:47:54 — a later, abandoned build. That
newer `csr.h` carries one extra CSR (`aaf_pkt write_drop_count` at `0x68`, from
the counter split), which pushes `fifo_level`, `dbg_last_ts`, `dbg_last_sec`,
`holdover_count` and `ts_subsec_max` up by four bytes. CSR banks sit on fixed
`0x800` boundaries, so nothing outside the `aaf_pkt` bank moves — but that is
not reassuring, because `fifo_level` feeds the USB isochronous feedback
accumulator (`firmware/main.c:1185`) and `dbg_last_sec`/`dbg_last_ts` compute
the Dante emit timestamp (`firmware/dante_tx.c:465`). A four-byte shift
therefore buys host sample-rate drift plus a missed playout deadline: "no
audio" with perfect packets on the wire, which is two separate bugs this
project has already chased. `flash.sh` runs `make -C firmware` itself, so
`--permanent` and `--fw-only` will happily ship the mismatch.

Reconstructing the right header is quick and exact, because `--soft-only`
generates headers without P&R:

    git worktree add /tmp/pre <commit that built the .bit>
    cd /tmp/pre && BOARD=breakout USB2=1 USB_DUAL_RATE=1 \
      python3 ./avb_soc.py --soft-only --firmware firmware/loader/loader.bin \
      --output-dir /tmp/pre-out

Validate the method before trusting it: generate from the CURRENT tree the same
way and check it reproduces the build directory's `csr.h` under
`grep -v "Auto-generated by LiteX" | md5sum` (it did, `daf43257939f…`). Then
copy **only** `csr.h`. `soc.h` differs just in the ident string and
`variables.mak` carries absolute paths into the worktree, so copying it breaks
the firmware build. Finish with `make -C firmware clean && make -C firmware`,
and verify in the machine code rather than by inspection:

    riscv64-unknown-elf-objdump -dS firmware/firmware.elf | grep -A2 fifo_level_read
    # want: lw a0,104(a5)  with a5 = 0xf0000000  ->  0xf0000068

**SPI now holds the verified pair** (bitstream at `0x0`, firmware image at
`0x300000`), written 2026-11-04. Note what `--permanent` leaves behind: the
firmware step ends with the SPI-proxy bitstream in the FPGA, so the board is
NOT running the design until a power cycle. A volatile `./flash.sh` afterwards
restores service immediately and validates the same pair; that was done, and
the board came up with PTP locked in 40 s, `talker 1` at 60 s, 23998 pkt/s
(= 96000/4), `rx_crc_err 0`, offset converging through −1.1 µs. The one counter
that looks alarming after a cold boot is `mac_writer_err`: it accrues a few
tens of thousands while the rx-gate is still disabled at reset, then settles to
~3/s once firmware arms it. Measure the RATE, not the total.

## 8. Why Main is tight, and why its ingress leaks (2026-11-04)

Reported as "audio on Backup is good, on Main it is very bad". It is real, it is
measurable, and the cause is not what the first measurement suggested.

**Use the LEAK, never `overrun_count`/`sendreq_busy`.** `main_rx_beats` against
`main_ep_out` are two INDEPENDENT counters, so their difference means exactly
one thing: bytes that arrived on the ULPI wire and never reached the OUT
endpoint. `overrun_count` in the shipping bitstream still SUMS benign overruns
with ring-full write-drops, and during this investigation it read anywhere
between 0 and 3.5 million/s for the same situation on BOTH PHYs. I built a tap
theory on an early Main-515k/Backup-0 reading and the control measurement on
Backup demolished it. The leak's noise floor is +/-30 B/s over a 15 s window
(+/-0.0003%); one reading came out at -0.0003% because the two counters are not
sampled atomically.

**Measured leak against the IDELAY tap, 48 ch @ 96 kHz, 15 s per point:**

| tap | Main | Backup |
|---|---|---|
| 0 | — | stream will not open (4/4) |
| 8 | — | +0.0139%  (6.7 samples/s/ch) |
| 16 | — | +0.0001% |
| 18–28 | — | at the noise floor |
| 24 / 28 / 31 | — | 0.0000% |
| 26 | +2.5237% (1267 samples/s/ch) | — |
| 27 | +1.2925% (637) | — |
| 28 | +0.5918% (288) | — |
| 29 | +0.1750% (84.6) | — |
| 30 | +0.0231% (11.1) | — |
| 31 | +0.0010% (0.3) | 0.0000% |

Monotone on both buses: higher tap is better, and **there is no upper edge
within reach** — Main still improves from 30 to 31, so both eyes continue past
what IDELAYE2 can deliver (31 taps, ~78 ps each, ~2.4 ns total). "Pick the
middle of the eye" therefore does not apply here. The only edge we can touch is
the lower one, so the rule is: **as high as the delay line goes.**

**The asymmetry: Main's eye sits ~16–19 taps (~1.3 ns) HIGHER than Backup's.**
Backup is flat zero from ~16 to 31 and only fails below ~8, so it has ~15 taps
of margin. Main's clean region starts exactly at 31, so it has none — its eye
begins where the delay line stops. That is the whole of "Main is tight".

**It is not a gateware asymmetry.** Both PLLs are `create_clkout(..., 60e6,
phase=0)`, both ULPI buses are in bank 34, and Main's clock pin is the BETTER
one: T4 is `IO_L13N_T2_MRCC_34` against Backup's AA4 `IO_L11N_T1_SRCC_34`. Pin
spread is 10 IOB rows on both. (The CRG comment still calls ulpi2_clock "V4, a
bank-34 MRCC pin"; the map has used AA4/SRCC since 2026-10-28.)

**I FIRST WROTE THAT THE ~1.3 ns IS BOARD WIRING, "NOT PLACEMENT". THAT WAS
WRONG, OR AT LEAST UNSUPPORTED.** The list above rules out a gateware asymmetry
between the two buses; it does NOT rule out placement. The eye's position is set
by when data reaches the capture flop against when the clock does, and BOTH of
those terms are placement-dependent: the IOB-to-flop routing on one side, the
PLL/BUFG insertion delay and clock-tree path on the other. With a critical path
that is 86% routing and cells smeared from X=23 to X=84, placement is exactly
where that skew can come from. The repo's own rule already said so — "the eye
centre depends on PLACEMENT, so it moves when the floorplan or the netlist
changes" (tools/tapsweep.py), and "a narrow eye is a symptom of too little Fmax
margin in cd_usb, so look there BEFORE reaching for a sampling-phase change"
(README). I proposed the phase shift first and skipped past that rule.

What supports placement as the cause: `usb` Fmax spans **34.3 to 78.9 MHz**
across the configurations in section 6, and the flashed build sits at 60.52 —
the bottom of that range. The same floorplan config that ships reached usb
70 / 71 on seeds 1 and 2. So there is 15-30% of margin available without
touching a single line of source.

One thing NOT to over-read: the firmware comment records Main's eye as 28..31 on
the seed-2 build while we measure only 31 clean on seed 8. Suggestive, but the
two used DIFFERENT CRITERIA — class-request errors then, the leak now, and the
leak is far more sensitive. It is a hint that the eye moved, not proof.

**Separately, both USB domains are routing-bound and smeared across the die.**
From the verified build's own log (`ser_s8.log`):

* `usb` (Main, s7pll1): **2.3 ns logic, 14.2 ns routing**, 16.5 ns total,
  hopping (84,84) → (84,94) → (81,103) → (30,103) → (70,103) → (33,89) →
  (70,109) → (30,110) → (23,125) → (70,111) — X=23 to X=84, 0.9–1.5 ns a hop.
* `usb2` (Backup, s7pll2): **2.0 ns logic, 13.4 ns routing**, 15.3 ns total,
  same character: (91,110) → (30,134) → (71,134) → (33,106) → (81,127).

86% of both paths is wire, through `usb.USBControlEndpoint`,
`endpoint_mux.tx_mux` and `ulpi_wrap`. `NEXTPNR_USB_FP_LIGHT=1` constrains only
the ~690 cells on nets touching the ULPI pads, so the LUNA core itself is free
to scatter. Main is 1.2 ns worse than Backup — 60.52 vs 65.04 MHz — purely on
placement luck, not architecture. Neither has the margin a 60 MHz source
capture deserves.

### What was done now, and what is still needed

Firmware pins **both taps at 31** (was Main 30, Backup 16). Measured after:
Main +0.0004%/+0.0007% (39 and 68 B/s), Backup +0.0001%/-0.0001% (6 and -9 B/s,
i.e. the noise floor). Main went from ~11 samples/s/channel — a dropout every
~90 ms, continuous crackle — to ~0.1–0.3. Written to SPI, so a power cycle
keeps it.

That is the best firmware can do, and it is **not** the same as Backup's zero.
Main sits a factor ~2 above the measurement noise floor with zero margin above
it, which is not a production footing.

**The real fix is to move the capture phase, not the data.** Main needs ~16
taps (~1.3 ns, ~27 deg at 60 MHz) more effective delay than the line can give.
Shifting `cd_usb`'s PLL output phase moves the sampling edge instead, which
drops Main's eye down into the middle of the tap range and gives it margin on
both sides. Shift both domains by the same amount and Backup's whole range goes
clean too, so no asymmetry is introduced:

    usb_pll.create_clkout(self.cd_usb,  60e6, phase=333)   # -27 deg
    usb2_pll.create_clkout(self.cd_usb2, 60e6, phase=333)

**The sign needs one build to confirm.** More data delay is equivalent to an
EARLIER sampling edge, hence the negative shift, but verify by re-running the
tap sweep: the eye's lower edge should move DOWN from 31 toward ~15. If it
moves up instead, use `phase=27`. Two builds settle it, and the eye measurement
reads out immediately — do not guess from the Fmax number.

Second, give each USB subsystem a compact region so the 14 ns of routing comes
down. That is the same left-IO-column contention as section 6, now with a
priority order the audio decides: **usb/usb2 margin matters more than eth_rx
headroom** — eth_rx needs 125 MHz and reaches 135–161, while usb needs 60 and
sits at 60.52.

### Two traps that cost time here

**`firmware.bin`'s crc32 is a constant.** The Makefile runs
`litex.soc.software.crcfbigen`, which patches the binary so its CRC32 is always
`0x2144df1c`. `mkimage.py` and `netload.py` both print that value, so it is
useful for spotting corruption and **useless for telling two builds apart** —
and it will not change when your edit lands. Compare `md5sum firmware/firmware.bin`
instead, or read the constant out of the disassembly
(`objdump -d firmware/firmware.elf | grep -B1 "sw.*-2016(a"` shows `li a4,31`
before the tap CSR write). I nearly concluded a rebuild had not happened.

**`./flash.sh --fw-only` cannot reset this board by itself.** It prints
"waiting for loader... (reset the board now: 'r' on the console)" and then
fails with "no loader responded". Use plain `./flash.sh`, whose `pld load`
restarts into the loader; it reloads the same bitstream plus the new firmware in
one step.

### The experiment that decides it (started 2026-11-04)

    BOARD=breakout USB2=1 USB_DUAL_RATE=1 NEXTPNR_USB_FP_LIGHT=1 \
    USB_PERIOD_NS=14 STALL_SECS=900 \
    ./build.sh --seed 1 --output-dir build_usbmargin_s1

`USB_PERIOD_NS=14` (71 MHz) gives the placer a real gradient on both USB
domains without being unreachable — 70/71 is measured, so this is "ambitious
but inside the measured range", not the 8 ns that killed HeAP. Verified in the
generated XDC: `usb_clk` and `usb2_clk` both at 14.000 ns, sys still 20.000.

Read the result as a fork, not as a number:

* **Main's eye drops into the middle of the tap range and the leak reaches the
  noise floor** → it was placement. Done, and no phase shift is needed.
* **The eye stays pinned at 31 even though usb now closes near 70** → the skew
  really is board-level, and the phase shift comes back on the table with
  evidence behind it instead of a guess.

Acceptance for production, either way: Main's leak at the noise floor with at
least ~8 taps of margin BELOW the pinned value, so a future placement shift
cannot walk it back off the edge. A single tap of margin is what we have now,
and it is why this had to be re-opened.

Watch `eth_rx`: this config measured 103.5 and 124.7 against a 125 MHz
requirement. That is the trade. 124.7 is a 0.2% shortfall on a clock whose
health we can check directly on hardware (`rx_crc_err`, `mac_writer_err` rate),
while usb's 0.9% shortfall is already proven to cost audio. Measure before
rejecting it.

#### Result: USB_PERIOD_NS=14 WORKED. usb 60.52 -> 69.98.

**READ THE LAST TIMING REPORT, NOT THE FIRST.** nextpnr prints one per clock
after PLACEMENT and again after ROUTING, and the two differ a lot. I reported
the post-placement numbers (usb 54.69, usb2 55.91, eth_rx 120.66) as the result
and wrote a whole "it backfired" section on them. `tools/`-side, `report.py`
in the scratchpad was written in the previous session to take the LAST report
per clock for exactly this reason, and I bypassed it by reading a monitor event.
Grep it properly:

    for c in s7pll0_clkout_buf0 s7pll1_clkout_buf s7pll2_clkout_buf \
             eth_rx_clk eth_tx_clk; do
      grep -a "Max frequency for clock" build.log | grep -a "$c'" | tail -1
    done

| clock | required | final (USB_PERIOD_NS=14) | the flashed build |
|---|---|---|---|
| sys | 50 | 55.52 PASS | 50.48 |
| usb (Main) | 60 | **69.98 (+16.6%)** | 60.52 (+0.9%) |
| usb2 (Backup) | 60 | 61.18 (+2.0%) | 65.04 (+8.4%) |
| eth_rx | 125 | **140.96 PASS** | 135.61 |
| eth_tx | 125 | 142.63 PASS | 160.38 |

Against the REAL requirements all five pass. The two "FAIL" lines are only
against the artificial 71.43 MHz target, which is not a requirement — so when
sweeping with a deliberately tight target, score on the MHz number, never on
PASS/FAIL.

So Main's domain gained 15.6% and `eth_rx` closed at the same time, which the
section 6 table said could not happen in this configuration (it had eth_rx at
103.5 / 124.7 FAIL). The difference is that those were P&R-only runs on the
2026-11-03 netlist; this is a full build on the current one. Treat the section 6
table as a map of the old netlist, not of this one.

Note usb2 at 61.18 is only +2.0%, worse than the flashed build's +8.4%. The
margin moved from Backup to Main rather than appearing from nowhere, which is
consistent with the column contention: the two USB subsystems trade with each
other as well as with ethernet. Backup's eye has ~15 taps of slack, so it can
afford to lose some — but verify it on hardware rather than assuming.

#### Control run, in parallel

    BOARD=breakout USB2=1 USB_DUAL_RATE=1 NEXTPNR_USB_FP_LIGHT=1 \
    STALL_SECS=900 ./build.sh --seed 1 --output-dir build_usbdefault_s1

Same configuration at DEFAULT constraints, to tell whether the 14 ns target did
the work or seed 1 simply places well on this netlist. That answer decides
whether `USB_PERIOD_NS` stays a useful knob or is noise. (Both builds ran
concurrently on a 4-core machine, so wall-clock times from these two runs are
not comparable with each other.)

### Outcome: Main's leak is gone. Measured on build_usbmargin_s1.

Flashed volatile, taps 31/31, three 30 s windows per PHY:

| window | Main | Backup |
|---|---|---|
| 1 | +0.0001% (14 B/s) | 0.0000% (0 B/s) |
| 2 | **0.0000% (0 B/s)** | +0.0001% (13 B/s) |
| 3 | +0.0001% (7 B/s) | −0.0001% (−13 B/s) |

Main and Backup are now indistinguishable, both at the measurement noise floor,
each with one window at exactly zero. Before: Main 84–93 B/s against Backup
5–13. Dual stream, both PHYs at 48 ch / 96 kHz simultaneously: 18,458,592 B/s
each, **leak +0.00% on both**, overflow 0, `packets` 24000.5/s, `rx_crc_err` 0,
`mac_writer_err` 2.9/s, PTP locked at 213 ns. `eth_rx` at 140.96 MHz is healthy
on hardware, which answers the trade this build was worried about.

**And the eye did NOT move — it got cleaner where it already was.** Sweeping
Main's tap on this build: 31 → +0.0003%, 28 → +0.3498%, 24 → +23.9% (3/4 opens
failed), ≤20 will not open at all. So the lower edge is still at ~30 and the
fall-off below it is STEEPER than on the 60.52 MHz build, while the leak AT tap
31 dropped from 84 to ~0 B/s. Conclusion: **eye POSITION is set by board skew,
eye QUALITY at that position is set by Fmax margin.** Neither of my two
hypotheses was right alone — placement did not shift the eye into mid-range, and
skew alone did not explain why more Fmax fixed the leak.

Consequence for production: the audible defect is fixed, but the pin still sits
~2 taps above the cliff rather than the ~8 I set as the acceptance bar. The
phase shift from the previous section is still the way to buy that margin, and
it is now a *robustness* change rather than the fix — different priority.

**USB_PERIOD_NS=14 is a FLOORPLAN lever, not a timing lever.** The control build
at default constraints gave usb 69.67 / usb2 65.96 — the same usb as the 14 ns
build (69.98), so the tight target did NOT produce Main's margin; seed 1 simply
places usb well on this netlist. What it did produce is `eth_rx` 140.96 against
the control's **103.46 FAIL**: constraining both USB domains harder compacts
them and frees the left column, exactly the mechanism section 6 describes. Keep
it for that reason, and note the control's usb2 was BETTER (65.96 vs 61.18), so
there is a better point still to find.

### A defect I introduced and then measured: write_drop_count is broken

The counter split of commit 7df0575 was right in principle and wrong in this
instance. `write_drop_count` reads **~4,608,000/s in every configuration** —
only Main, only Backup, or both — and that is exactly 96000 × 48. A counter
equal to the sample rate counts SAMPLES, not drops. Meanwhile the leak was
0.0000% on both PHYs, `packets` exactly 24000/s and `overflow` 0, so nothing was
actually being lost. `tools/usbsrc.py` was printing "<-- HOORBARE SAMPLE-DROPS"
at it, which is a false alarm; it now prints the number flagged as broken.

Where to look: the increment is `en & samp_vld & (started | first) & ~have_space`
and should be the exact complement of `do_write`. `samp_vld` is `do_pop`, a true
one-cycle strobe, and none of these signals sit inside the `for _s in
range(streams)` loop (which starts after them), so it is not a scope mistake.
That leaves `have_space = level < (SRING_DEPTH - 2)`: `~have_space` is true on
every sample while `do_write` still writes, so `level` is not the quantity it
appears to be — note the `fifo_level` CSR reads 7 to 95, far below 2046. Fix
`level`, not the counter.

So the standing rule for "are we losing audio" remains: **the leak, two
independent counters.** I split one conflated counter and walked straight into a
second bad one; the leak is the thing that has never lied here.

### Retraction: write_drop_count was never broken, and the "new defect" was my own doing

Both claims in the previous two sections are wrong. Recorded in full because the
reasoning error is the reusable part.

**The counter is correct.** It read ~4,608,000/s and I concluded "a counter
equal to the sample rate counts samples, not drops". It does not: that is
exactly what a FULL ring produces, because then every write is refused. The
confirmation was sitting beside it and I did not read it — `fifo_level` was
pinned at **128**, and `_bl_max = SRING_DEPTH >> _bl_sh = 128 = full` with the
servo target at 64. Once the ring sat at its centre again (fifo_level 63..67),
the counter read **exactly 0** with two simultaneous 48 ch / 96 kHz streams.
So `write_drop_count == sample rate` means "ring pinned full", not "counter
broken". Read `fifo_level` next to it, always.

**And the ring-full state was not a property of build_usbmargin_s1.** I measured
underrun 387-516/s on Main and 645-774/s on Backup with the ring pinned, and
wrote that up as a regression in the new bitstream. After a plain firmware
reload, on the same bitstream, both PHYs sit at fifo_level 63..67 with
**underrun 0/s** and `write_drops` 0, dual stream, leak -0.00% on both.

The likely cause of the disturbed state is me: within the hour before, I had
swept Main's IDELAY tap down through 28, 24, 20, 16, 12, 8 and Backup's through
24, 16, 8, 0 — values that produce 24% byte loss or kill the link outright — and
then swept the USB async feedback across +/-3%. That is enough to drive the ring
into a stuck full state that survives until the firmware is reloaded. **The
board was live and being listened to while I did it**, and "music, beep, off,
repeating" is a fair description of what a tap sweep sounds like from the other
end. Do destructive sweeps on a board nobody is listening to, and reload the
firmware before trusting any measurement taken after one.

**What survives from that detour, as a genuine latent bug:** `main_usb2_fb_ovr`
exists (avb_soc.py:1094, resets to 0 = auto) and firmware never wrote it —
`main.c` only wrote `main_usb_fb_ovr`. The Backup PHY was therefore permanently
on the wrapper's measured loop and could not be influenced from outside at all,
so the `F` sweep over UDP silently covered only Main. That is fixed now (both
CSRs get `usb_fb_manual`), but note it was NOT the cause of what was heard —
both PHYs are healthy on auto. It matters for diagnosis, not for today's audio.

Also noted while in there: `USB_FB_NOMINAL` in main.c is `(6 << 16)` = 6.0
samples per microframe, the **48 kHz** value. At 96 kHz nominal is 12.0 =
786432, so the percentage the FBSWEEP handler prints is a factor two out at
96 kHz. Read the raw value, not the percentage. The F sweep itself was still
informative: on Main, held at the true 96 kHz nominal, fifo 90 / underrun 0 /
packets exactly 24000, while +1% pinned the ring full with underrun 1575/s.

### Where build_usbmargin_s1 actually stands

All five clocks pass their real requirements (sys 55.52, usb 69.98, usb2 61.18,
eth_rx 140.96, eth_tx 142.63). On hardware, after a clean firmware load:

* Main leak -0.00%, Backup leak -0.00%, both with ingress at +0.06% of nominal
* `write_drops` 0 with both PHYs streaming 48 ch / 96 kHz simultaneously
* ring centred, fifo_level 63..67, `underrun` 0/s on either source
* `rx_crc_err` 0, `mac_writer_err` ~3/s, PTP locked

That is the first build in this project with both PHYs clean at the same time.
Still open, in order: the eye has only ~2 taps of margin above the cliff (the
phase shift, as robustness); usb2 at 61.18 is thinner than the control build's
65.96, so a better seed/floorplan point exists; and this is running VOLATILE —
SPI still holds the previous bitstream.

### 9. The leak is Main's input eye, and /dev/zero cannot see it (2026-11-04)

**Test ULPI with random data. Never with `/dev/zero`.** Same bitstream, same
pinned tap 31, same 48 ch @ 96 kHz, Main:

| source | rx vs nominal | leak | fifo | underrun |
|---|---|---|---|---|
| `/dev/zero` | +0.06% | **0.0000%** | 90 | 0/s |
| random data | +1.2% | **+1.25%** | 30–45 | 39–127/s |
| mpv, internet radio | +1.2–1.3% | **+1.18–1.23%** | 56–59 | 12/s |

Backup measures 0.0000% with zeros AND with random. Constant zero bytes make
almost no transitions on the data bus, so a marginal setup/hold window stays
invisible; I spent half a day concluding Main was clean while 1.28% was walking
out of it. Every "leak = 0" result taken with `/dev/zero` in the sections above
is worthless for timing margin.

Tap sweep on Main WITH RANDOM DATA, which is what settles it:

| tap | leak | lost |
|---|---|---|
| **31 = the hardware maximum** | **+1.28%** | 624 samples/s/ch |
| 30 | +4.99% | 2428 |
| 29 | +14.21% | 6895 |
| 28 | +29.95% | 14490 |

Monotone and steep, and the highest tap IDELAYE2 has still leaks 1.28%. 31 taps
x ~78 ps = ~2.4 ns is simply not enough delay. **No tap can fix Main.**

**It is not the media clock and not the feedback loop.** Measured against the
board's own PTP clock over 180 s: 24002.09 packets/s, **+0.0087%** — the media
clock is exact. And holding the feedback at true nominal made things worse
(fifo 13, underrun 1124/s) because the host was already starving: the user's
mpv log showed its network cache draining 28 s -> 0 over ~44 minutes, which is
~1.06% and matches the over-delivery. The chain is: Main's ULPI input loses
~1.25% of real data -> the endpoint passes roughly nominal -> the host is asked
for ~1.2% more -> a network-sourced player drains its buffer until it underruns.
"Audio loopt" was exactly that.

### The phase shift was never fairly tested — I flashed over a live stream

`USB_PHASE_DEG=333` on `cd_usb` builds fine and closes identically (a PLL phase
is a parameter, so same netlist, same placement, same Fmax), and both PHYs still
enumerate. Then, under isochronous traffic:

    xhci_hcd 0000:00:14.0: ERROR Transfer event TRB DMA ptr not part of current TD
    xhci_hcd 0000:00:14.0: xHCI host controller not responding, assume dead
    xhci_hcd 0000:00:14.0: HC died; cleaning up

All USB on the machine went away — both PHYs and the CH347, so the JTAG/UART
bridge you would reflash with disappears too.

**I blamed the phase for that, and I was wrong.** The real cause was that
AUDIO WAS STILL STREAMING ON ONE OF THE PHYS WHEN I FLASHED. `pld load`
reconfigures the FPGA, so the USB device vanishes mid-transfer, and "TRB DMA ptr
not part of current TD" is exactly that failure. `flash.sh` now refuses to flash
while any card reports `state: RUNNING` (override with
`FLASH_ALLOW_STREAMING=1`). So the phase shift is not refuted — it has never
been measured under normal conditions.

**What remains a real risk, to check first rather than assume:** `cd_usb` clocks
the RX capture AND the TX side driving the PHY. ULPI is source-synchronous, so
the shift moves our OUTPUT timing relative to the PHY's clock too. 27 degrees is
1.25 ns; whether that still meets the USB3300's input setup/hold is NOT
established and was not measured. If TX breaks it shows up as enumeration or
transfer errors, not as a leak — so verify both PHYs still work normally BEFORE
drawing any conclusion about the eye. I wrote this up as a proven mechanism when
it is an untested one.

**Recovery without a reboot:**

    echo 0000:00:14.0 | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind
    sleep 3
    echo 0000:00:14.0 | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind

Both PHYs and the CH347 come back and flashing works again. Verified.

**How to test it properly, in this order:** stop every player, flash, wait for
PTP, confirm both PHYs enumerate and that Backup still measures 0.0000% leak,
and only then repeat the tap sweep on Main WITH RANDOM DATA. The eye's lower
edge should move DOWN from ~31 toward ~15; if it moves up, use 27 instead of 333.

**If the phase genuinely cannot work, then in order of expected value.** (a) A second PLL output
at a shifted phase clocking ONLY the RX capture flops, which means separating
the ULPI wrapper's RX capture from the rest of `cd_usb` — the real fix, and a
contained RTL change. (b) The physical skew: Main's eye sits ~1.3 ns higher than
Backup's, both PLLs are at phase 0 and both buses are in bank 34 with Main on
the BETTER clock pin (T4 MRCC vs AA4 SRCC), so ~1.3 ns of data-vs-clock skew on
the Main bus is a wiring property worth measuring at the board — Main is on J4
while J5 was hand-soldered, so an over-long clock wire on Main is a real
candidate. (c) Until either lands, **Backup is the clean source** and measures
0.0000% with real data; using it as the primary is the honest interim position.
