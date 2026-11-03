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

**96 kHz works.** Both cards report `Format: S32_LE  Channels: 48  Rates: 96000`.
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

### 3. Bench bandwidth, production topology

At 96 kHz each PHY needs high-bandwidth isochronous: 3 transactions per
microframe, 2880 B, ~23 MB/s. Two PHYs **streaming** at once does not fit one
host controller. On this bench both sit on bus 001, so 48ch/96k on both
simultaneously cannot be verified here. In production each PHY has its own Mac,
so it is not a limit — but note that many USB hubs and some older xHCI
controllers refuse high-bandwidth iso endpoints. Plug into a motherboard port.

Enumeration and configuration of both at once **do** work on one controller;
it is the streaming reservation that will not.

### 4. The A/B mux has never been exercised end to end

`tools/usbsrc.py backup` sets the request, but the handover is frame-aligned in
gateware: `live` follows only when the candidate presents a channel-0-of-frame
marker. With nothing streaming, both sources read `NO INGRESS` and the request
stays pending — correct behaviour, but it means the switch itself is untested.
To test: stream 48ch/96k into Backup, confirm ingress climbs to ~9.216 MB/s per
8-channel block, then switch and check `live` follows and `handovers` increments.

### 5. The 3-hour wedge is still open

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
