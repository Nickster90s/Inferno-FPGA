# Inferno-FPGA — 48-channel USB → AoIP interface

A standalone hardware audio interface: a host streams 48 channels over USB, and
an FPGA puts them on the network as a native AoIP transmitter, interoperating
with Audinate's Dante protocol. No PC-side driver stack, no Dante Virtual
Soundcard, no licensed Audinate module.

**Board:** Colorlight i9plus v6.1 (Xilinx Artix-7 XC7A50T-fgg484-1)
**Toolchain:** openXC7 (yosys + nextpnr-xilinx) — fully FOSS, no Vivado
**SoC:** LiteX + VexRiscv, bare-metal C firmware
**USB:** LUNA/Amaranth UAC2 high-speed device on a USB3300 ULPI PHY

> ### Status: audio works, down to 0.25 ms. Phases 0-5 and 7 complete on the bench.
>
> The device appears in Dante Controller, locks to PTPv1, and streams audio to
> real Dante hardware over **unicast flows** negotiated on port 4455 — verified
> against a Focusrite RedNet A16R, a RedNet AM2 and Dante Virtual Soundcard
> **simultaneously**, at different `fpp` and channel counts, with zero underruns
> and USB ingress within +0.002% of nominal. Multicast flows can be created and
> deleted from Dante Controller.
>
> **0.25 ms latency is selectable and measured working** — Dante Controller
> offers 0.25/0.5/1/2/5 ms, sets the value over ARC `0x1101`, and the change
> takes effect immediately. At a 0.25 ms setting a RedNet A16R negotiates
> `fpp=8` (0.167 ms of packetization) and reports a **125 µs peak — half the
> budget**. On the transmit side `tools/ts_lag.py` puts our timestamps 8 samples
> EARLY against the PTP timeline with a 2.3-sample spread; a RedNet A16R
> transmitting on the same bench measures +12.5 samples and cannot itself meet
> 0.25 ms.
>
> **Latency is per-flow, not per-device.** A packet cannot exist until `fpp`
> samples after its own timestamp, so `fpp/48000` is a hard floor for each
> subscription:
>
> | receiver | negotiated `fpp` | floor | tested at |
> |---|---|---|---|
> | RedNet A16R (Brooklyn-3) | 8 | 0.167 ms | **0.25 ms, 125 µs peak** |
> | RedNet AM2 (UltimoX2) | 16 | 0.333 ms | 1 ms (its own minimum) |
> | Dante Virtual Soundcard | 60 | 1.25 ms | **4 ms** |
>
> DVS streams cleanly but **always requests `fpp=60`** and therefore cannot run
> at 0.25 or 0.5 ms — measured, not assumed: with the device advertising 250 µs,
> its flow still binds at `fpp=60` (`tools/stats.py` opcode `f`). It is tested
> at 4 ms. Low latency is a console/Brooklyn-3-class capability here, not a
> device-wide one.
>
> Long-run hardening (Phase 6) has not been done; see [Open bugs](#open-bugs)
> for what remains.

---

## Legal and naming

**This project is not affiliated with, authorized by, or approved by Audinate.**
"Dante" is Audinate's trademark. The Dante protocol is undocumented and
unlicensed; everything here is derived from public reverse-engineering work.
Bench and research use only.

**On the name.** This repository is *not* a fork or port of
[inferno](https://gitlab.com/lumifaza/inferno), despite the name. Inferno is a
Rust implementation of the Dante protocol for Linux, licensed GPL-3/AGPL-3, and
it is used here strictly as a **specification to read** — a description of wire
formats, opcodes and record layouts. No inferno code is copied into this
repository. See [Protocol references](#protocol-references) and
[`DANTE_CONVERSION.md`](DANTE_CONVERSION.md).

**Licensing.** Original code in this repository is Apache-2.0 (see `LICENSE`,
`NOTICE`, `LICENSING.md`). Note that `rtl/ulpi_wrapper.v` is vendored from
[ultraembedded](https://github.com/ultraembedded) and is **GPL**, so any
*bitstream* built from this tree is GPL-encumbered regardless of how the rest
is licensed.

---

## Why Dante-native rather than AES67

Dante devices have an AES67 mode, and an AES67 bridge would be less work and
legally cleaner. It was considered and rejected for this project because Dante's
AES67 mode is multicast-only, locked to 48 kHz, and unsupported on Dante Virtual
Soundcard, Dante Via, and older hardware. Native Dante talks to everything.

The trade is that the work moves from the data plane to the control plane:

| | Dante-native | AES67 |
|---|---|---|
| Audio header | **9 bytes**, no RTP | RTP 12 B + SDP + SAP |
| Clock | PTPv1 (40-byte header, new code) | PTPv2 (reuse existing gPTP plumbing) |
| Discovery | mDNS `_netaudio-*` + ARC + CMC + info multicast | SAP/SDP |
| Interop | all Dante devices | AES67-capable Dante only |

Dante's audio wire format is genuinely simpler than AES67's — one constant byte
and two big-endian `u32`s, then big-endian MSB-justified samples:

```
[0]      0x02                    constant
[1..5]   seconds        u32 BE
[5..9]   subsec_samples u32 BE   0 .. sample_rate-1
[9..]    interleaved samples, big-endian, MSB-justified

timestamp = seconds * sample_rate + subsec_samples   (in units of samples)
```

Milestone-1 parameters: 24-bit, `fpp = 16`, 8 channels per flow, 6 multicast
flows = 48 channels, to `239.255.x.y:4321`. 435-byte frames, 3000 pps per flow.

---

## Architecture

```
   host (DAW)                    FPGA — XC7A50T                        network
  ┌──────────┐   USB HS    ┌───────────────────────────────┐
  │ 48 ch    │  iso async  │  USB3300 ULPI ──► LUNA UAC2   │
  │ 48 kHz   │────────────►│         (Verilog leaf, cd_usb)│
  │ 32-bit   │◄────────────│              │ AsyncFIFO      │
  └──────────┘  feedback   │              ▼                │
                           │   packetizer: 6 × 8 ch rings  │
                           │   paced by the media clock    │
                           │              │                │   6 × AoIP 
                           │              ▼                │   multicast
                           │   TXFrameArbiter ──► LiteEth ─┼──► flows
                           │                        ▲      │   (UDP 4321)
                           │  VexRiscv ─────────────┘      │
                           │   • PTPv1 slave (TSU stamps)  │◄── PTPv1
                           │   • mDNS / ARC / CMC / info   │◄── control
                           │   • media-clock NCO servo     │
                           └───────────────────────────────┘
```

Two properties are load-bearing and must survive every future change:

- **The CPU is never in the per-sample path.** The gateware packetizer pops the
  USB FIFO and emits frames itself, paced by the media-clock NCO. The main loop
  runs at ~4000 iterations/s; it cannot touch 48 × 48 kHz of audio. Any design
  that puts firmware on the sample path has already failed — see
  `BENCHMARK_BASELINE.md`.
- **The host is slaved to our clock, not the reverse.** The USB wrapper measures
  its own NCO-strobes-per-SOF and reports that as async isochronous feedback, so
  the host paces itself to the media clock. There is no sample-rate converter.

  **Correction (2026-08-03): "nothing chases the FIFO" was wrong**, and believing
  it cost two bench sessions. The wrapper's feedback is a **PI servo on ring
  level**, not a pure rate report — `rtl/usb_avb_subsystem.v`: `err = 64 -
  block_level`, `integ += err`, P gain `err<<6`, updated every SOF. So the ring
  already has a controller, and anything that disciplines the NCO becomes a
  *second* controller on the same buffer. That is why the media-clock NCO must
  be **slew-limited** (`mcr_dante.c`) rather than stepped.

---

## Roadmap

| # | Phase | Exit signal | State |
|---|---|---|---|
| 0 | Fork `avb-aes3`, strip the AVB stack | boots, clock locks, USB enumerates | **code complete**, HW check pending |
| 0.5 | Frozen loader ROM + `coderam` + netload | firmware edit → running in seconds | **code complete**, HW check pending |
| 1 | LiteDRAM on the 8 MB SDRAM (`--with-sdram`) | 8 MB memtest passes | planned |
| 2 | `net.c` — IPv4 / UDP / ICMP / IGMP / ARP | **FPGA answers `ping`** | **done** |
| 3 | Dante discovery (mDNS + ARC + CMC + info) | **device appears in Dante Controller** | **done** |
| 4 | `ptpv1.c` — PTPv1 slave | locks to a PTPv1 master | **done** — locks in ~30 s, offset sub-µs |
| 5 | `dante_packetizer.py` + `dante_tx.c` | audio received by real hardware | **done** — clean audio on the bench |
| 6 | Hardening / long-run | 24 h, no dropouts | **partial** — see below |
| 8 | Persistent settings + RX patch | survives a power cycle | **not started** |
| 7 | Unicast flows + subscriptions | subscribe from Dante Controller | **done** — per-flow map, slots and fpp |

Discovery deliberately precedes clock and audio: it is firmware-only, needs no
PTP, and getting the device to appear in Dante Controller is the earliest
external confirmation that the reverse-engineered assumptions hold — before any
gateware is written against them.

Milestone 1 is **48 channels out only**. USB capture (device→host) has never
carried a sample in this lineage and is out of scope until the TX path is solid.

Phase 7 was reached earlier than planned, because it turned out to be necessary
rather than optional: multicast bundles sourced all six flows unconditionally,
which measured **65.5 Mbit/s of 69.6 Mbit/s on the segment** — 94% of all
traffic, flooded to every port by an unmanaged switch. Unicast made transmission
proportional to actual subscriptions (0.03 Mbit/s idle).

---

## Open bugs

**Fixed (2026-08-07): `netload.py` gave up on a NAK the loader wanted retried.**
One push was NAKed (`NAKed EXEC ... CRC mismatch or short image`); the tool
exited, and the board then answered neither UDP nor a further netload, needing a
JTAG bitstream reload. The loader sets `started = 0` on that path with the
comment *"let the host retry cleanly"* — it was explicitly asking for another
attempt, and the sender was calling `sys.exit()` instead. It now retries the
whole image up to three times.

A lost or duplicated DATA frame already self-corrected, because the loader
reports where it actually is and the sender resyncs. What does not self-correct
is a frame accepted at the RIGHT offset with damaged payload: offsets stay
consistent, every ACK looks normal, and it only surfaces as a CRC mismatch at
EXEC.

> Note for anyone reading the logs: `netload.py` prints `crc32 0x2144df1c` for
> **every** image, and that is correct, not a bug. `0x2144DF1C` is the CRC-32
> residue — the value you get computing CRC32 over a message that already has
> its own CRC appended, which `firmware/Makefile` does via `crcfbigen`. It was
> briefly recorded here as a defect on exactly that misreading.

**Fixed (2026-08-07): frames larger than 512 bytes never left the device.**
`rd_idx_next` and `fr_adr` in `dante_packetizer.py` addressed the 512-word frame
memory with `Signal(max=128)` — 7 bits. 128 words is 512 bytes, and that was a
hard ceiling: 6 slots × fpp=24 (483 B) worked, 5 × fpp=32 (531 B) did not.

It presented as "the frame never reaches the wire", which was wrong twice over.
The frames were transmitted the whole time at the correct length and rate — the
truncated *write* address wrapped past word 127 onto words 0..k, which hold the
destination MAC, source MAC and ethertype, so each frame left with its own
headers overwritten by audio. Every filter used to look for them (`udp port N`,
`ether src <board>`) keyed on a field the bug destroys, and `packet_count` —
which counts the packetizer's own last beat — was correct throughout. Capturing
unfiltered showed 771-byte frames, `ethertype 0xee00`, garbage MACs, at exactly
the paced 800/s.

This blocked every receiver needing a large packet: DVS at 4 slots × fpp=60 is
771 bytes. `f_last_idx` three lines above already carried a comment about the
same `max=128` truncation, so the bug class had been found once and these two
were missed.


**Media clock: rate discipline now works (2026-08-03), audio interaction still
untested.** `mcr_dante.c` replaces `mcr`'s ownership of the NCO — one writer,
rate only (`g_ptpv1.rate_ppb`, the servo integral, never the phase term),
slew-limited to 100 ppb/s at 1 Hz. Measured by toggling it at runtime:
**+4.86 ppm disarmed → +0.17 ppm armed** (17.5 → 0.6 ms/hour), causal and
reversible. The whole correction costs **17 CSR writes**, against the old path's
thousands per second.

Root cause of the two previous failures, finally identified: an **AVB CRF-loss
watchdog** was the only live NCO writer, rewriting it at ~4 kHz — *and* the USB
async feedback is not a rate report but a **PI servo on ring level** (traced in
`rtl/usb_avb_subsystem.v`), so the ring already had a controller. Stepping the
NCO underneath it was the fault; the missing constraint was slew rate, not the
choice of rate estimate. See `MCR_REPLACEMENT.md`.

**Validated with audio (2026-08-04).** MacBook streaming, two unicast flows,
discipline toggled at runtime:

| | disarmed | armed |
|---|---|---|
| drift | +3.61 ppm (13.0 ms/h) | **+0.39 ppm (1.4 ms/h)** |
| ring level | 45…77 | **48…78** |
| underrun / overrun | 0 / 0 | **0 / 0** |

The ring did not care — level range while armed is indistinguishable from
undisciplined, against a centre of 64, and the worst level minimum sampled
through the whole 44 s slew was 49. PTP stayed locked with no re-anchors. That
is what the slew limiter buys: both earlier attempts stepped the NCO at
main-loop rate and produced underrun storms.

**Then audio stopped entirely, and rate discipline could not have caught it.**
Hours later: no audio at any receiver, patch **fully green** in Dante
Controller, and every counter healthy — 9001 pps, ring centred, zero underruns,
PTP locked. The emitted timestamp was **+10908 samples (227 ms) in the future**,
147× `DANTE_TX_TS_OFFSET`. Receivers negotiate the subscription fine and then
discard every audio packet as far-future.

Two failures lined up: `ts_anchor()` only runs when the talker *enables*, and
the talker had stayed on for two days of free-running drift; and the re-anchor
watchdog compared only the **seconds** field with `diff > 1`, so a 0.227 s error
was structurally invisible to it. Rate discipline reported a flat +0.53 ppm
throughout — correct, and irrelevant. **Rate and phase are separate axes**, and
only one had an instrument. Fixed in both `dante_tx.c` (sample-based re-anchor,
5 ms backstop) and `mcr_dante.c` (slow phase term, ±2 ppm, 1 ms deadband).
Phase now −4 samples; audio confirmed by ear. See `MCR_REPLACEMENT.md`.

**Remaining gap:** the auto-disable guard needs `lvl_max >= 32` to tell "ring
active" from "no USB source", so it cannot catch a clock that stops the ring
priming at all. Nothing has made it fire yet.

An overnight unattended run has since happened and audio was still playing in
the morning, so this has not bitten in practice — but that is one run and the
guard is still blind to the case it was written for, so it stays open.

**(historical) Media clock is UNDISCIPLINED under PTPv1 — drifts ~15 ms/hour.**
Was the most serious issue; fixed above. Left running the stream dies silently: every counter
reads healthy while the timestamp walks out of the receiver's ~1 ms buffer.

Root cause is understood. `mcr_compute_gptp_base()` takes its rate from
`m->gptp`, gated on `g->servo_locked` — a flag set only by `gptp_servo_update()`,
the **802.1AS** servo. This device runs PTPv1, which keeps its addend state in
`g_ptpv1` and writes nothing to the gPTP struct, so the gate is never true and
the function always returns the undisciplined `base_increment`. The +4.34 ppm is
the raw crystal error.

**Pointing `mcr` at `g_ptpv1` fixes the drift and breaks the audio.** Measured
both ways:

    drift     4.34 ppm -> 0.17 ppm     (25x better)
    underrun  ~130 total -> 40547 in 4 minutes (~170/s, sustained)

The packetizer emits silence on an underrun, so the audible result was worse
than the slow drift it cured — and `fifo_level` still read centre throughout,
which nobody has explained. Reverted; the tree is back to clean audio that
drifts.

**Start the next attempt from the underrun, not the drift.** The question is why
correcting the NCO rate starves a ring whose level looks correct. Two other
unexplained observations are probably the same fault seen from different angles:
applying a rate trim made the timestamp fall 5.3 s behind in 115 s (a thousand
times more than the ±50 ppm clamp permits), and following the servo integral
alone instead of the full addend produced the same underrun storm. All three say
something in the media-clock/USB-feedback loop reacts far more violently to an
NCO change than the arithmetic suggests it should.

**Fixed:** *audio was mangled for the first seconds of every stream.* Two
separate causes, both now addressed. The talker used to toggle ON → OFF → ON at
startup because flows expired on a PTP-derived timer that jumps when PTP steps;
expiry is gone entirely (contexts are reclaimed by peer IP, or by evicting the
least-recently-bound when all six are taken). And the talker was enabled on the
*first* PTP lock edge, which is not trustworthy — PTP locks, drops out, relocks,
then applies its path delay and a phase correction. It now waits for the lock to
hold 4 s **and** the path delay to be known before anchoring: `enables 1,
disables 0, anchors 1`, against 3/2/3 before.

**Fixed:** *unicast reached only one receiver.* Never a transmit-side fault —
the rate was already exactly 9001 pps with both contexts emitting correctly. A
stale `b.<flow>=` key from a deleted multicast flow kept receivers chasing a
group that no longer existed, so they never asked for a unicast flow. TXT
records now carry the mDNS cache-flush bit (RFC 6762 §10.2) and a 120 s TTL.
Confirmed: two receivers, separate contexts, both playing.

**`mac_writer_err` climbs at ~25/s.** Received frames dropped because the CPU
cannot drain two RX slots against flooded multicast. This is the documented
collapse mode (risk 8); PTP stays locked despite it, but it is real frame loss —
and it is the *source* of the ±5–10 µs PTP offset outliers that `ptpv1.c`'s
unlock hysteresis currently masks, since a lost FollowUp or DelayResp mispairs
with the wrong Sync.

**Fixed by `rx_gate`** (2026-08-01) — see `RX_GATE.md`. Measured A/B on one
bitstream with the filter toggled at runtime: **21.0% of unicast round-trips to
the board were lost with the filter off (84 of 400), and 0 of 400 with it on.**
`writer_errors` falls 61.1 → 8.4/s; the residual is flood frames aborted while
the status FIFO is briefly full, counted before the MAC is classified, and is
*not* lost control traffic — see RX_GATE.md for why, and why round-trip loss is
the honest instrument. Underruns, overruns, PTP unlocks and re-anchors were all
zero in both arms.

The filter is disabled at reset and armed explicitly (`tools/rx_gate.py on`, or
`x` on the console), so the bitstream behaves exactly as before until asked. The
software allow-list in `dispatch_rx()` could never have fixed this on its own: it
runs after the frame has already taken a slot, and the slot is what is exhausted.

**sys Fmax is not reproducible — the artifact is a BITSTREAM, not a seed.**
`avb_soc.py` re-execs with `PYTHONHASHSEED=0` to stop the same seed scattering
52-65 MHz run to run. That reduced the scatter; it did not remove it. Measured
2026-08-07 on one netlist: seed 8 gave **54.97 MHz** in a serial build and
**63.26 MHz** in a sweep, same source, same loader, same parsing. So
`--seed 8` does not name a result, and rebuilding a validated seed may not
reproduce the bitstream that was validated. Keep `build_seed8/`.

Two consequences. A sweep table is a snapshot, not a spec — re-sweep after any
gateware or loader change. And Fmax must never pick the seed on its own: seed 1
had the highest clean sys Fmax (63.54) and the LOWEST USB margin (79.62), and
flashed, the MacBook enumerated the audio device and showed **no outputs at
all**. Prefer the higher `USB_MHz` when seeds tie, then confirm on hardware --
see the header of `tools/seed_sweep.sh`.

**Settings and the RX patch do not survive a reboot.** `cfgflash.c` / `config.c`
already persist a versioned, CRC'd blob in SPI flash — static IP, media-clock
source, the converged PTP addend and media-clock rate — but three things an
operator sets are NOT in it, and come back at their compiled defaults:

- **advertised latency** (`g_latency_ns`). Select 0.25 ms in Dante Controller,
  power cycle, and the device is back at the 1 ms default while Controller still
  believes it chose 0.25.
- **device name**, if renamed from Controller.
- **the RX patch** — `sub_tx_name[]` / `sub_tx_host[]` in `dante_arc.c` are plain
  statics, so every subscription made into this device is lost.

Latency is trivial to add: `config.h` documents the procedure (new field before
`reserved`, shrink `reserved` by the same size, bump `CFG_VERSION`) and there are
55 reserved bytes. The RX patch is the awkward one — `DANTE_RX_CHANNELS` x 2
name strings will not fit in what is spare, so it needs either a second flash
region or a larger config record.

Worth doing together with a **write policy**: a setting that is saved on every
Controller poll would write flash constantly, so saves must be debounced and
only on actual change.

**Long-run (phase 6) is unproven, but three days of uptime did not break it.**
Observed 2026-08-11: the board had been powered for **3 days**. Starting the host
gave **instantly clean audio** — no re-lock wait, no startup artefacts. That is
real evidence that PTPv1 lock and the media-clock discipline survive long
uptime, which is what the drift work was for.

It is NOT the 24 h test. Audio was not streaming for those three days — the host
was off — so it says the clock is healthy after long idle, not that a stream
survives 24 h. The soak that phase 6 asks for is continuous audio, watching
`underrun` / `overrun` / receiver-reported peaks, and it has not been run.

**Per-receiver latency works.** Verified 2026-08-11 with three receivers
subscribed at once, each at its OWN latency setting, all green in Dante
Controller and audio clean by ear on the AM2 and DVS:

| receiver | `fpp` | its setting | measured | notes |
|---|---|---|---|---|
| RedNet A16R | 4 (its own choice) | 0.25 ms | **20 µs** | peak 7 samples |
| RedNet AM2 | 16 (its own choice) | 1.0 ms | 280 µs | |
| Dante Virtual Soundcard | **16, clamped from 60** | 4.0 ms | **<700 µs** | was 1.77 ms |

18,000 pps, 0 underrun/s, 0 overrun/s, ring 64.

Three things had to be right, and each was wrong in a different way.

**1. Advertise a small `fpp`, not a large one.** We advertised `fpp=8,2` and
accepted up to 60, so receivers negotiated packets we could not deliver quickly
— an AM2 on `fpp=16` needs 333 µs of packetization and cannot be served inside
a 250 µs budget, so it went red and looked like a latency fault. A real A16R
advertises **`fpp=4,2`**. Receivers then take `max(our value, their own setting)`
and sort themselves out, which is what the Dante documentation says all along.
Note a receiver picks `fpp` from its OWN latency, roughly latency ÷ 3: the A16R
went 8 → 4 by itself when its setting went 0.5 → 0.25 ms.

**2. Serve an oversized `fpp`, do not refuse it** (`g_fpp_clamp`, cap
`g_fpp_max_accept = 16`). DVS cannot be talked down — its own minimum latency is
4 ms so it always asks for `fpp=60`. Rejecting is useless: capped at 4, DVS
retried the IDENTICAL request 16 times and never renegotiated, and so did the
AM2; both just stop subscribing. Serving a SMALLER packet than was asked for
works, because a receiver reassembles by TIMESTAMP rather than by the `fpp` it
requested. That is how the A16R gets away with capping at 4.

The cap is 16 and not 4, and the difference was measured with audio playing:

| cap | DVS | AM2 |
|---|---|---|
| 60 (accept anything) | `fpp=60`, 1.77 ms | 0.46 ms, green |
| 4 (copy the A16R) | `fpp=4`, 0.42 ms | **1.10 ms, RED** |
| **16** | `fpp=16`, 0.70 ms | **0.28 ms, green** |

A receiver's requested `fpp` encodes its own CAPABILITY, not just its latency.
The AM2 asks for 16 because it cannot process 12,000 packets/s; at `fpp=4` its
rate tripled and it went red while OUR side stayed clean at 48,001 pps with zero
overruns. DVS, a PC, handled 12,000 pps fine. Copying the A16R's cap of 4 is
wrong for a mixed bench.

**3. `DANTE_TX_TS_OFFSET = 0`**, so `lag = fpp-1`. The old value of 6 was
calibrated at a large `fpp` and does not survive a small one: when the A16R
renegotiated to `fpp=4` the lag became 3-6 = **-3 samples** — our timestamps
AHEAD of arrival — the measurement clamped to zero and Controller showed a GREY
Latency Status. 0 is correct by construction, not by tuning: the timestamp marks
the packet's first sample and the packet leaves when its last sample arrives.

> A low measured latency is margin, not risk. 20 µs of a 250 µs budget means we
> deliver early and the receiver keeps its slack. Playout alignment depends on
> timestamp CORRECTNESS, not delivery time. The cost is that the average sits
> near the measurement floor — watch the PEAK for early warning.

`dante_tx_latency_effective()` (raise the advertised latency to cover the
largest bound `fpp` window) was built when this was misdiagnosed and is OFF by
default; with the `fpp` cap it only dragged fast receivers up to the slowest.

**Clipping at full source volume** is unconfirmed as ours. The digital path is
a bit-exact MSB-justified truncation with no gain stage, so it cannot create
clipping that is not already in the source — but this has not been verified at
high level, because it needs a multicast flow to make the payload visible.

**USB is only partly retested** on the current gateware. 2 channels were
verified end to end after the unicast gateware work. All 48 have NOT been
exercised on this bitstream — the historical failure mode in this lineage was
48-channel-specific (channel rotation after a dropped sample, fixed by
channel-addressed ingress), so a 2-channel pass does not clear it.

USB ingress itself measures exact on the validated bitstream: `ep_out`
9,216,205/s against a 9,216,000 nominal (**+0.002%**), leak +0.00%, 0
underruns/s. That is the transport, not the channel mapping, and it is the
mapping the 48-channel failure mode lived in.

---

## Quickstart

The bitstream and the firmware are **separate artifacts**. The BRAM ROM holds
only a small frozen loader; application firmware is delivered at runtime. So a
firmware change costs seconds, not a 6–10 minute place-and-route.

```sh
# Toolchain environment (openXC7 + LiteX/migen)
export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db

# --- once per gateware change (~6-10 min) --------------------------------
make -C firmware/loader          # the frozen loader -> baked into the ROM
./build.sh --seed 7              # bitstream
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 build/colorlight_i9plus/gateware/colorlight_i9plus.bit; exit"

# --- every firmware iteration (seconds, no P&R) --------------------------
make -C firmware
sudo ./tools/netload.py ens5 firmware/firmware.bin    # then reset the board
```

`netload.py` repeats its START frame until the loader answers, so start it and
then reset the board — `r` on the console, a power cycle, or a JTAG reconfigure.
The loader opens a ~400 ms window at every reset.

For standalone boot with no host attached, put the firmware in SPI flash:

```sh
./tools/mkimage.py firmware/firmware.bin firmware/firmware.img
TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl -f $TOOLS/ch347.cfg -c "
  init; pld load 0 $TOOLS/bscan_spi_xc7a50t.bit; reset halt
  flash probe 0; flash protect 0 0 50 off
  flash write_image erase firmware/firmware.img 0x300000 bin
" -c exit
```

That writes only the firmware image at 3 MB — it does not touch the bitstream at
`0x0` or the config sector at the top of flash.

Console: `sudo picocom -b 1000000 /dev/ttyACM0` (Ctrl-A Ctrl-X to exit). openocd
and picocom cannot both hold `/dev/ttyACM0` — close picocom before flashing.

### Things that will bite you

`build.sh` sets `PYTHONHASHSEED=0` and runs under `setarch -R` (ASLR off).
**Both are required** for reproducible placement — without them the same
`--seed` scatters Fmax across ~52–65 MHz run to run.

Always `make clean && make` in `firmware/` after touching CSRs. A link failure
otherwise leaves the *previous* `firmware.bin` in place against the *new* CSR
layout, so every register access silently lands at the wrong address.

`./build.sh --bake-firmware --firmware firmware/firmware.bin` restores the legacy
single-image map (96 KB ROM, no loader) for a standalone shippable bitstream, or
for recovery if the loader boot path breaks. In that mode firmware changes *do*
re-roll placement again.

After any gateware or loader change, sweep seeds and **test USB on hardware** —
global Fmax does not predict USB quality:

```sh
JOBS=3 tools/seed_sweep.sh 1 9      # builds each seed, tabulates Fmax per clock
```

### The short way

```sh
./flash.sh                 # volatile: pld load + firmware over Ethernet
./flash.sh --permanent     # SPI: bitstream at 0x0 AND firmware image at 3 MB
./flash.sh --unlock        # once per board, before the first --permanent
./flash.sh --fw-only       # firmware only, no bitstream
```

It rebuilds the firmware first, refuses to run while picocom holds
`/dev/ttyACM0`, and checks the board answers afterwards. It defaults to
`build_seed8/` — the bitstream VALIDATED on hardware — and says so, warning if
`build/colorlight_i9plus/` is newer rather than silently picking either. Pass
`--bit PATH` to override. The manual steps below are what it runs.

### Persistent flashing (SPI — survives power cycle)

`pld load` alone is **not** persistent. Writing SPI needs the bscan proxy:

```sh
# Step 1 — unlock, once per board (MX25L128 ships write-protected)
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl \
  -f /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/ch347.cfg \
  -c "init; pld load 0 /home/lisp/FPGA/Colorlight-FPGA-Projects/tools/unlock_flash_xc7a50t.bit; exit"

# Step 2 — write + reconfigure from flash
TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
BIT=build/colorlight_i9plus/gateware/colorlight_i9plus.bit
sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl -f $TOOLS/ch347.cfg -c "
  set XC7_JSHUTDOWN 0x0d; set XC7_JPROGRAM 0x0b; set XC7_BYPASS 0x3f
  init; pld load 0 $TOOLS/bscan_spi_xc7a50t.bit; reset halt
  flash probe 0; flash protect 0 0 50 off
  flash write_image erase $BIT 0x0 bin
  irscan xc7.tap \$XC7_JSHUTDOWN; irscan xc7.tap \$XC7_JPROGRAM
  runtest 60000; runtest 2000; irscan xc7.tap \$XC7_BYPASS; runtest 2000; reset
" -c exit
```

openocd and picocom cannot both hold `/dev/ttyACM0` — close picocom first. SPI
is only read at **power-up**: a warm reset does not reload it.

---

## Hardware

### Board

Colorlight i9plus v6.1, XC7A50T-FGG484. 25 MHz crystal on **K4**.
Board pinout: `/home/lisp/FPGA/Colorlight-FPGA-Projects/colorlight_i9plus_v6.1.md`

The cabled RJ45 jack on this board is **U9 = PHY1, MDIO address 1**, so LiteEth
uses `eth_clocks`/`eth` **index 1**. Getting this wrong yields a link that
negotiates but never passes frames.

8 MB of SDR SDRAM (M12L64322A) is wired to the FPGA and currently unused;
Phase 1 brings it up for the capture ring and cold heap.

### CH347T — JTAG + UART on one USB-C

| CH347T | Signal | FPGA pin |
|---|---|---|
| TXD1 | UART | M3 |
| RXD1 | UART | R3 |
| TCK/TMS/TDI/TDO | JTAG | header |

UART runs at **1 Mbaud** with a 64-byte HW FIFO so periodic status prints never
stall the main loop. Both JTAG and UART share `/dev/ttyACM0`.

**The UART drops output under load.** A `printf` in a UDP handler was verified
not to reach the console while the opcode demonstrably ran and returned the
right value, and `[flow]` lines from the same path came through. Do not build a
diagnostic that depends on the console alone -- use UDP (below).

### UDP control surface (port 7779)

`tools/stats.py` speaks this. One-byte opcode, optional payload, binary reply.
It is the reliable channel: it works while the console is dropping output, and
it does not need the CH347 (which JTAG and the UART share).

| op | payload | does |
|---|---|---|
| `?` | | all runtime counters (the default `stats.py` read) |
| `f` | | per-flow detail: dst, port, slots, **fpp**, age |
| `u` | | USB ingress counters -- `rx_beats`, `ep_out`, over/underrun |
| `s` `g` `p` `t` `m` | | PTP servo status, gPTP, phase, telemetry, media clock |
| `L` | u32 ns | advertised latency; no payload = read it back |
| `M` | | multicast fpp / create |
| `P` `o` `F` | | phase adjust, TX timestamp offset, USB feedback override |
| `A` | 4-byte IP | mirror every ARC request to that host on port 7780 |
| `K` | u16 key, u16 val | patch an inline key in the ARC 0x1100 table |
| `B` | u16 key, u32 val | patch a u32 in the 0x1100 **data blob** |
| `G` | u8, u8 | patch the 0x1000 capability bytes |
| `V` | u16, u16 | patch the 0x1003 router / arcp version fields |

`A`/`K`/`B`/`G`/`V` are protocol-archaeology probes. They are RAM-only and a
reboot reverts them.

### USB3300 ULPI breakout (P2 / SODIMM header)

| Signal | FPGA pin | SODIMM | Notes |
|---|---|---|---|
| CLK | T4 | 51 | must be MRCC/SRCC — clock-capable |
| DIR | T3 | 49 | |
| NXT | U2 | 57 | |
| STP | U3 | 59 | |
| RST | R2 | 41 | **active HIGH** — use `Pins`, not `PinsN` |
| D0–D7 | V2 V3 W1 W2 Y1 AA1 AB1 Y2 | 61–75 | bidirectional |

The 60 MHz ULPI clock is **PHY-sourced**; a PLLE2 re-emits it.

**Wiring is not optional detail.** Twist CLK with **its own ground**, never
paired with another active signal. Twenty gateware variations failed identically
until the clock was rewired this way. A marginal ULPI crystal presents as a
touch-sensitive, intermittent link.

---

## Console

| Key | Action |
|---|---|
| `s` | status — PTP servo, grandmaster, RX message counts |
| `m` | media-clock servo state (NCO increment, offset) |
| `a` | packetizer + USB ingress state |
| `e` | RX ethertype counters + LiteEth heartbeat |
| `b` | 1-second rate window (loop rate, writer errors) |
| `t` / `T` | enable / disable the gateware talker |
| `f` / `F` | toggle USB NCO freeze / sweep USB feedback value |
| `k` / `j` | cycle SRC servo KP / KI (live tuning) |
| `u` | step the ULPI IDELAY tap (re-plug USB to test) |
| `G` / `C` | dump PTP / media-clock convergence ring-log |
| `R` / `z` | dump / clear the on-FPGA packet capture ring |
| `P` | sweep the presentation-time offset (AVB-only; removed in Phase 5) |
| `N` | clear a stale inherited AVB CRF binding from NV |
| `g` / `x` | RX MAC allow-list state / arm-disarm it (see `RX_GATE.md`) |
| `c` / `d` | media-clock state / arm-disarm discipline (see `MCR_REPLACEMENT.md`) |
| `v` | toggle verbose prints (default off) |
| `r` | reboot |
| `h` / `?` | help |

`b` is the early-warning instrument. Targets are **>10,000 iterations/s and 0
writer errors**. If iterations/s falls below ~2000 or writer errors climb once
real Dante multicast is on the wire, the CPU is being swamped by flooded audio
and needs a MAC-level RX allow-list — not a protocol parser. That allow-list now
exists in gateware: `g` shows what it would drop while still disabled, `x` arms
it. `RX_GATE.md` has the procedure.

---

## Repository layout

```
avb_soc.py              LiteX SoC top: CRG, LiteEth+TSU, MCR NCO, USB instance,
                        packetizer wiring, clock constraints, floorplan hooks
aaf_packetizer.py       gateware packetizer: 6×8ch rings → frame builder →
                        TXFrameArbiter.  Phase 5 → dante_packetizer.py
rx_gate.py              RX destination-MAC allow-list: drops the flooded audio
                        before it takes an RX slot.  Off at reset -- RX_GATE.md
floorplan_usb.py        --pre-place region constraint for the USB block
build.sh                deterministic build wrapper (PYTHONHASHSEED + setarch -R)
flash.sh                flash the board.  Volatile by default (a power cycle
                        reverts); --permanent writes SPI, --unlock once per board

tools/
  netload.py            push firmware into coderam over raw Ethernet (dev loop)
  stats.py              runtime counters over UDP:7779 -- the main telemetry
  telemetry.py          event stream decoder
  rx_gate.py            arm / back out / measure the RX MAC allow-list
  mclk.py               arm / back out / measure the media-clock discipline
  mkimage.py            wrap firmware in the loader's header for SPI flash
  seed_sweep.sh         build N seeds, tabulate Fmax per clock
  arc_query.py          query ANY device's ARC server -- the tool that decoded
                        the property tables by comparing real hardware
  flow_req.py           send a real flow-control request from the bench, so
                        unicast can be exercised without Dante Controller
  dante_latency.py      decode receivers' own reported latency from the 8708
                        heartbeat -- the authoritative "is the receiver happy"
  ts_lag.py             absolute transmit timestamp lag against the PTP timeline
  ts_offset.py          two-point runtime calibration of DANTE_TX_TS_OFFSET
  dante_decode.py       pcap -> Dante audio header fields
  cap_fetch.py          pull the on-FPGA capture ring
  rx_ts_probe.py        RX timestamp probe
  test_netload_protocol.py
                        host-side conformance test: a model of the loader's
                        receive state machine driven by the real sender

firmware/
  loader/               FROZEN boot loader baked into the 12 KB ROM. Loads an
                        image into coderam from Ethernet or SPI flash and jumps.
                        Changing it re-rolls placement -- avoid.
  main.c                boot, RX dispatcher, UART CLI, main polling loop
  net.c                 IPv4 / UDP / ICMP / IGMP / ARP.  Replaced osc.c
  gptp.c                802.1AS gPTP slave + PI servo on the 52-bit TSU addend.
                        Phase 4 factors the servo out for reuse by ptpv1.c
  ptpv1.c               PTPv1 slave -- the clock Dante actually speaks
  mcr.c                 media-clock NCO servo
  mcr_dante.c           the AoIP media clock: sole NCO owner, rate-only,
                        slew-limited.  Replaces mcr.c's ownership

  --- AoIP control plane ---
  mdns.c                _netaudio-{arc,cmc,chan,bund} discovery records
  dante_dev.c           device identity + g_latency_ns, the ONE advertised
                        latency all three publishers read
  dante_msg.c           the 10-byte control header and message builder
  dante_arc.c           ARC server (4440): channels, flows, subscriptions, the
                        0x1100/0x1102 property tables, 0x1003 device names and
                        0x1101 latency set
  dante_cmc.c           CMC server (8800) -- device advertisement
  dante_info.c          device / product info records + the 8708 heartbeat.
                        Manufacturer, Model Name and Dante Model live here
  dante_flows.c         flow-control server (4455): parses a receiver's request
                        for channels, fpp and destination socket
  dante_tx.c            binds those requests to packetizer contexts; owns the
                        timestamp anchor and flow eviction

  cap.c                 on-FPGA packet capture ring
  rx_gate.c             CSR programming + auto-revert for the RX allow-list
  dstats.c              the UDP:7779 control surface (see Console)
  telem.c               event stream
  cfgflash.c config.c   NV config in SPI flash (versioned + CRC)
  pkt_geom.h            packetizer stream geometry

rtl/                    Verilog leaves: usb_avb_subsystem.v (LUNA-emitted UAC2),
                        ulpi_wrapper.v (GPL), rgmii_var_delay.py
sims/                   Migen simulations — the reference for ring addressing
                        and time-mux correctness.  Read these before changing
                        the packetizer.
_avb_reference/         the stripped AVB stack, kept readable, out of the build

DANTE_CONVERSION.md     conversion log: what was removed, why, measurements
LITEETH_PATCHES.md      the four LiteEth patches this design depends on
BENCHMARK_BASELINE.md   why the CPU must stay off the sample path
TOOLCHAIN.md            exact toolchain versions + fresh-machine reproduction
```

---

## Protocol references

Neither is a dependency; both are read as specifications.

- **[inferno](https://gitlab.com/lumifaza/inferno)** (GPL-3/AGPL-3) — the Dante
  protocol reference. Audio wire format in `device_server/flows_tx.rs`; ARC
  opcodes in `protocol/proto_arc.rs` + `device_server/arc_server.rs`; mDNS
  records in `device_server/mdns_server.rs`; the multicast-bundle discovery path
  in `mdns_client.rs`. Also the eventual **test peer**: run it on Linux to
  record our 48 channels and verify bit-exactness.
- **[statime](https://github.com/pendulum-project/statime)** (Apache-2.0/MIT) —
  PTP. Upstream is PTPv2 only; the **PTPv1** wire format Dante needs lives on
  the `inferno-dev` branch of the [teodly fork](https://github.com/teodly/statime)
  under `statime/src/datastructures/messages_v1/`. Permissively licensed, so
  safe to reference or vendor. `statime-stm32/src/ptp_clock.rs` is a good
  template for mapping a PTP servo onto a hardware addend register.

### Decoded on this bench — not in inferno

inferno is the best reference available, but it is not complete. These were
worked out here by querying real hardware with `tools/arc_query.py` and
comparing devices that differ in the behaviour of interest — a RedNet A16R
(Brooklyn-3) against a RedNet AM2 (UltimoX2). Probing our own device could not
have found any of them; the comparison had to be against hardware that works.

- **ARC `0x1101` — Dante Controller's latency SET.** Carries the latency in
  nanoseconds as u32, **twice**; both copies have always been equal, and we
  require that before applying, so a misparsed message is rejected rather than
  believed. inferno does not implement this opcode. Answering `0x22` to it is
  why a latency selection would not stick.
- **The latency CAPABILITY lives in the `0x1100` DATA BLOB**, addressed by the
  offset keys — not in the inline key/value table:

  | key | A16R | AM2 | |
  |---|---|---|---|
  | `0x8306` | 250,000 | 1,000,000 | **minimum supported latency** |
  | `0x8205` / `0x8301` | 500,000 | 1,000,000 | |

  A16R minimum 0.25 ms → Controller offers 0.25/0.5/1/2/5. AM2 minimum 1 ms →
  1/2/5. Found by sweeping opcodes against both devices and searching the
  replies for the latency values as nanoseconds.
- **`0x1003` is not inferno's `get_device_names` layout.** Both RedNets emit a
  48-byte header, 2 pad bytes, then FIXED 32-byte name fields, with the names at
  `+20/+22/+24` (inferno has a 38-byte header and `+12/+14/+16`) and
  router/arcp versions at `+34`/`+38`. Confirmed rather than assumed: those
  version fields decode to 4.4.0 / 2.8.12 for the A16R and 4.3.0 / 2.8.9 for the
  AM2, matching each device's own mDNS. Serving the wrong layout made Controller
  fail with "Cannot retrieve Device Latency" and retry ~1300 times a minute.
- **`0x2032`** — a request Controller makes once per Device View open; both
  RedNets answer OK with two zero bytes.

Dead ends recorded so they are not retried: the `0x1000` capability word, the
inline `0x1100` keys, the missing `0x0064`/`0x00f0` keys, the arcp version, and
the `(mf, model)` identity all left Controller's latency list unchanged.

Standards worth having open: IEEE 1588-2002 (PTPv1), IEEE 1588-2008,
RFC 1112/2236 (IGMP), RFC 6762/6763 (mDNS/DNS-SD).

---

## openXC7 landmines

Hard-won, each one cost real debugging time. Read before changing gateware.

- **Inject clock constraints yourself.** openXC7 emits no `create_clock` for
  PLL-derived clocks, so every CDC gets timed as a single-cycle path. `avb_soc.py`
  injects `create_clock` + `set_clock_groups` onto the PLL output *nets*. Without
  this you get CDC failures that present as audio corruption.
- **Determinism needs both knobs.** `PYTHONHASHSEED=0` *and* ASLR off.
- **Seed roulette is real, and Fmax does not predict USB quality.** A firmware
  change alters ROM contents and re-rolls placement. A seed must be validated on
  two independent axes: it places without a HeAP stall, *and* USB works on real
  hardware. Phase 0.5 exists to remove this tax entirely.
- **No usable DSP48.** nextpnr-xilinx cannot route the inferred DSP48 cascade on
  this part, so two constant multiplies are hand-expanded into shift-add trees.
  Budget zero DSPs; design arithmetic to avoid them.
- **Wide CSR banks destroy sys_clk.** One debug CSR bank capped it at 43.97 MHz,
  and sub-50 MHz setup violations **silently corrupted audio data paths** rather
  than failing loudly. Use indirect addressing (`ctx_select`) instead of
  per-channel registers, and keep the CSR bridge registered.
- **Wide muxes too.** A 109:1 32-bit read mux belongs in a BRAM, not an `Array`
  of registers.
- **USB placement is fragile.** Any new logic in `cd_usb` outside the wrapper
  hierarchy can re-break 60 MHz HS chirp. Keep new gateware in `sys`. Even the
  UART FIFO depth is pinned at 64 because 1024 shifted placement and broke USB.
- **`last_be` is a one-hot of the last valid byte, not a byte mask.** Getting
  this wrong produces a bad FCS on every single frame.
- **`nrxslots` cannot exceed 2.** Raising it to 4 *silently* kills LiteEth TX.
  Root cause unknown; it would need patching LiteEthMAC.
- **Never drive the CH347 UART from the build host** — it crashes the CH347 off
  USB. That port belongs to the interactive console.

---

## Related projects

- [TerminalDanteControl](https://github.com/Nickster90s/TerminalDanteControl) —
  `dantectl`, a terminal Dante controller: Discover, Sync and a Routing
  patchbay. Python 3 stdlib only. Written against this firmware and the captures
  in `captures/`, and it is how the device's control plane, PTP lock and
  subscriptions get checked from the bench without Dante Controller. Its README
  records the measurement showing this firmware drops ARC requests that arrive
  in a burst with info queries (0/12, against 12/12 on a RedNet A16R under the
  same load)
- [inferno](https://gitlab.com/lumifaza/inferno) — Dante for Linux, in Rust
- [statime](https://github.com/pendulum-project/statime) — `no_std` PTP in Rust
- [network-audio-controller](https://github.com/chris-ritsen/network-audio-controller) —
  open Dante device/routing control (`netaudio` CLI), useful for testing
- [LiteX](https://github.com/enjoy-digital/litex) / [LiteEth](https://github.com/enjoy-digital/liteeth)
- [LUNA](https://github.com/greatscottgadgets/luna) — USB gateware in Amaranth
- [openXC7](https://github.com/openXC7) — FOSS Xilinx 7-series flow
- [aes67-linux-daemon](https://github.com/bondagit/aes67-linux-daemon) — the
  AES67 alternative, if standards-based interop suits you better
