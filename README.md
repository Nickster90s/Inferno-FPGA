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
| 3 | Dante discovery (mDNS + ARC + CMC + info) | **device appears in Dante Controller** | **done** — incl. live TX flows (`0x2200`) and the Signal meter (`0x8002`) |
| 4 | `ptpv1.c` — PTPv1 slave | locks to a PTPv1 master | **done** — locks in ~30 s, offset sub-µs |
| 5 | `dante_packetizer.py` + `dante_tx.c` | audio received by real hardware | **done** — clean audio on the bench |
| 6 | Hardening / long-run | 24 h, no dropouts | **partial** — 16 h 8 min clean on 2026-11-05/06, see below |
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

**Open: `ptp_locked` drops and comes back, and two independent sources see it.**
Controller's own log has this device repeatedly losing and regaining clock sync
(2026-11-06 18:07–18:43, 2026-11-07 09:41–09:45), and `tools/stats.py` caught
`ptp_locked` going 1 → 0 within 20 s on 2026-11-07 at a raw offset of about
−12 µs. Audio kept flowing through all of it — an 18-hour run with `underrun`
stable and `disables 0` — so it costs no samples that we can measure, but it
should not happen. Not investigated.

Note that `offset_ns` in the `?` reply is the **raw** PTP sample, not the servo
output, so isolated 15–50 µs excursions in it are normal and are not evidence
of anything; `tools/telemetry.py` carries raw **and** filtered per servo update,
and that is the right source for the real clock error. Half a day was spent in
2026-11-04 reporting those raw spikes as clock disturbance.

**Fixed (2026-11-04): Main's ULPI input eye sat beyond the delay line — both
PHYs are now clean at once.** The Backup PHY measured perfect while Main lost
~1.3% of real audio, which a listener heard as crackle and then as drift: the
loss made the host over-deliver by the same ~1.2%, so a network-sourced player
drained its buffer (28 s of cache to zero in ~44 minutes) and stuttered.

Main's leak per IDELAY tap, 48 ch @ 96 kHz, **measured with random data**:

| tap | phase 0 | phase 333 |
|---|---|---|
| 31 (the hardware maximum) | **+1.2841%** | **+0.0002%** |
| 30 | +4.9970% | — |
| 29 | +14.2055% | — |
| 28 | +29.9451% | — |
| 24 / 20 / 12 / 8 / 0 | — | **0.0000%** |

At phase 0 even the highest tap IDELAYE2 has still leaked 1.28%, so no tap could
fix it: 31 taps x ~78 ps is ~2.4 ns and the eye lay past that. The fix is to move
the sampling edge instead of the data — `USB_PHASE_DEG` (default **333**, i.e.
−27°) on `cd_usb` only. Every tap is then clean, including tap 0, so the eye is
wider than the whole delay line. `cd_usb2` stays at phase 0: Backup runs off its
own PLL, was already well placed, and still measures −0.0002%.

Both PHYs streaming 48 ch / 96 kHz simultaneously with random data: leak +0.00%
on each, `write_drops` 0, `underrun` 0.0/s, ring at 66 against a servo target of
64, 24016 packets/s, `rx_crc_err` 0, PTP locked.

> **Four measurement rules, each one paid for.**
>
> 1. **Test a clocked bus with RANDOM data, never `/dev/zero`.** The same
>    bitstream and the same tap measured 0.0000% with zeros and +1.28% with
>    random: constant bytes barely toggle the lines, so a marginal setup/hold
>    window is invisible. Pre-generate a file (`head -c 150M /dev/urandom >
>    rand.raw`) and loop it; `/dev/urandom` live is too slow for 18 MB/s.
> 2. **The leak is `dbg_rx_beats` vs `dbg_ep_out`** — two independent counters,
>    so the difference means one thing. `overrun_count` sums a benign cause with
>    a harmful one and reads anywhere from 0 to millions per second;
>    `write_drop_count` equal to the sample rate means the ring is pinned full
>    (check `fifo_level`: 128 is full, 64 is the servo target), not that the
>    counter is broken.
> 3. **Never flash while audio is streaming.** `pld load` reconfigures the FPGA,
>    so the USB device vanishes mid-transfer and xHCI can take the whole
>    controller down with it (`TRB DMA ptr not part of current TD` → `HC died`),
>    including the CH347 you would reflash with. `flash.sh` now refuses. Recovery
>    without a reboot: unbind then bind `0000:00:14.0` on
>    `/sys/bus/pci/drivers/xhci_hcd`.
> 4. **Clear the host's ARP entry before concluding the board is dead.** Added
>    2026-11-07, after it produced a confident and completely wrong diagnosis
>    ("the board is sitting in the loader") while the firmware had been running
>    the whole time. `jtagspi_program` loads a proxy bitstream to reach the
>    flash, so the design stops and restarts; on the way back the cached entry
>    goes stale while still printing `REACHABLE`, and ping returns 100% loss
>    with `pipe 2`. The loader and the application firmware share one MAC
>    (`02:00:00:00:00:42`), so nothing about the entry looks wrong. One command
>    settles it: `sudo ip neigh del 169.254.9.200 dev ens5`. Check USB too — a
>    board whose USB devices are freshly enumerated has booted, whatever the
>    network says.

Build that produced this (reproducible — see the seed entry below):

```
BOARD=breakout USB2=1 USB_DUAL_RATE=1 NEXTPNR_USB_FP_LIGHT=1 \
USB_PERIOD_NS=14 USB_PHASE_DEG=333 STALL_SECS=900 \
./build.sh --seed 1 --output-dir build_phase333_s1
```

**That recipe is now `build.sh`'s default, and it stopped being optional on
2026-11-07.** Two consecutive eight-seed sweeps were lost to assembling it from
memory — three hours. The first set none of it and built the single-USB 48 kHz
chip (every log healthy, zero `ulpi2` lines in the XDC). The second set three of
the four `NEXTPNR_*`/`USB*` switches and hung all eight seeds, because without
`NEXTPNR_USB_FP_LIGHT=1` the full USB floorplan box is infeasible by
construction: the build that places has 1429 cells in `usb_fp`, the eight that
hung had 3109 in the same region, and nothing else differed.

So `./build.sh` with no arguments now builds this configuration; any variable
set explicitly is kept, and `DANTE_PROFILE=none` opts out entirely. Every build
writes **all 23** switches the design reads to `$OUT_DIR/build_config.txt`,
derived from `avb_soc.py` and `floorplan_usb.py` rather than hand-listed, so a
new switch cannot quietly fall out of the record and a build directory can
always be asked what it is. Reporting a mistake in a log that is read after the
build is weaker than making the mistake impossible.

Timing: sys 55.52, usb 69.98, usb2 61.18, eth_rx 140.96, eth_tx 142.63 MHz —
all five past their real requirements. The `FAIL` lines in that log are against
the artificial 71.43 MHz target `USB_PERIOD_NS=14` sets, which is not a
requirement; score the MHz, not PASS/FAIL. And read the **last** timing report
per clock: nextpnr prints one after placement and another after routing, and
they differ by more than 10 MHz.


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


**ROOT CAUSE FOUND 2026-08-14: the ring pointers wrap after 3.107 hours, and
the occupancy calculation could not survive it.** Audio died four times --
2026-08-12, twice on 08-13, again on 08-14 -- every time needing a JTAG
bitstream reload. The period was the clue that named it:

    rd advances by `channels` (8) per media strobe at 48 kHz = 384,000/s
    rd is Signal(32)  ->  2^32 / 384,000 = 11,185 s = 3.107 HOURS

`dante_packetizer.py` computed `level = wr - rd` as a PLAIN difference into a
34-bit signed signal, which is only correct while neither pointer has wrapped:

1. `wr` leads `rd` by the ring level (~64), so **`wr` wraps first**.
2. For the ~167 us until `rd` wraps too, `wr - rd` evaluates to
   **-4,294,967,232** instead of +64.
3. `level < FRAME_SAMPLES` un-primes the ring, and `rd` only advances under
   `strobe & primed` -- so **`rd` freezes exactly when it was about to wrap**,
   and can never wrap. `level` stays ~-2^32 permanently.

IT HID BEHIND ITS OWN SAFETY CLAMP. `block_level` clamps a negative level to 0
(`.Elif(level < 0, self.block_level.eq(0))`), so `fifo_level` read 0 rather than
-4.29 billion: the single number that would have named this on the first
occurrence was the one deliberately thrown away. Meanwhile `wr`'s update is
gated on neither `primed` nor `en`, so every ingress counter stayed perfect --
`rx_beats = ep_out`, `sample_overflow` 0, no leak, no drops. Three separate
investigations concluded "the decoder is producing nothing" from exactly that
evidence, and all three were wrong.

CORRECTION (2026-08-15): the first version of this entry said avb-aes3 never
saw it because its `rd` advanced by +64 per block traversal. **That was wrong on
both the number and the inference.** avb-aes3 advances `rd` by FRAME_SAMPLES=48
per block at 8000 blocks/s = 384,000 Sa/s -- identical to ours, since
consumption must match production. The +64 was dante-usb's own earlier fpp=8
behaviour.

**avb-aes3 has the identical bug**: `aaf_packetizer.py:287-291` has the same
`Signal(32)` pointers and the same `level = Signal((34, True)); level.eq(wr-rd)`,
the same un-prime latch, the same `level < 0` clamp hiding it, and the same
3.107-hour period. It survives by accident -- its firmware drops
`aaf_pkt_enable` on every gPTP/CRF unlock (`main.c:1512-1544`), and each toggle
reloads `rd` from `rd_anchor`, so routine unlocks act as an unintended watchdog.
Our talker never drops once locked, and `ts_anchor()` reloads only
`ts_sec`/`ts_sub`, never `rd`. **The fix should be ported back to avb-aes3.**

Also corrected: the ring is 2048 samples (`fifo_depth=256`), not 512, so the
window where `wr` has wrapped and `rd` has not is ~2.7 ms, not 167 us. And the
state is not strictly permanent -- `frame_base` free-runs, so `level` climbs
back through zero after ANOTHER 3.107 h. Nobody ever waited that out.

THE FIX: compute the difference MODULO instead of widening it. 2^32 is divisible
by any power of two, so the low bits of `wr - rd` are exact across a wrap.
Modulus 4 x SRING_DEPTH keeps real occupancy in the lower half and preserves
"consumer overtook producer" as a negative in the upper half.
`sims/sim_ring_wrap.py` drives the pointers straight at the boundary and fails
without the fix (level +64 read as -4,294,967,232), passes with it.

NO SIMULATION IN THIS TREE EVER CROSSED THE WRAP -- `sim_ring_accounting` and
`sim_ring_sample_rd` both model occupancy over seconds, and this needs 2^32
increments. That is why it shipped.

**2026-11-06, THE WIDTH IS GONE AS A CLASS.** The modulo fix above was
verified in hardware first, which is why the widening could be done calmly: the
board ran 16 h 8 min at 96 kHz with `en` high throughout, and at 8 ch x 96 kHz
the wrap is 2^32/768000 = **1.553 h**, not 3.107 h, so the pointers crossed
2^32 about ten times with `underrun` pinned at 129. (The 3.107-hour number
belongs to 48 kHz. At 96 kHz the old bug would have fired more than twice as
often, not less -- worth knowing before anyone tries to reproduce it.) The
record is `bitstreams/dual-usb-96k_2026-11-04_phase333_BOTH-CLEAN.VERIFIED-16h.md`.

The pointers are now `PTR_BITS = 48`, which puts the wrap at 2^48/768000 =
11.6 years, and the receiver project's own statement of the lesson is the reason
it is a width and not another guard: *"64-bit sample indices everywhere. Here
the failure mode is removed by construction rather than defended against."*
It is nearly free, because every consumer already truncates -- `_diff` keeps
only `_LVL_BITS`, `rdf[0:log2depth]` only the address bits, and `f_base` is a
difference that is exact modulo any power of two. Nothing compares a pointer
against a constant.

And four CSRs now report what the clamp used to eat: `ring_level_raw` (signed,
unclamped), `ring_level_min` (sticky worst), `ring_underflow` (episodes, not
cycles) and `ring_unprime`. Read them with `tools/stats.py --ring`. NO new
recovery was added, deliberately: the prime hysteresis already is one -- a
negative level un-primes, `rd` holds, `wr` catches up, and the ring re-primes
at centre in ~11 ms, which is the same end state as the talker cycle verified
below. What was missing was never the recovery. It was knowing that anything
had happened.

THE CSR CHANGE IS PURELY ADDITIVE, AND THAT TOOK ONE MEASUREMENT TO GET RIGHT.
Declared next to `fifo_level`, where they belong by subject, the four registers
pushed `dbg_last_ts`, `dbg_last_sec`, `holdover_count` and `ts_subsec_max` up by
0x10 -- so an old firmware on the new bitstream would have read `ring_level_raw`
where it expected `dbg_last_ts`, i.e. fed garbage straight into the re-anchor
phase check in `dante_tx.c`. A CSR's address is its declaration order, so they
are declared LAST instead, and `csr.h` generated both ways confirms the
difference is now four appended addresses and nothing else:

    32a33,36
    > CSR_AAF_PKT_RING_LEVEL_RAW_ADDR (CSR_BASE + 0x80L)
    > CSR_AAF_PKT_RING_LEVEL_MIN_ADDR (CSR_BASE + 0x84L)
    > CSR_AAF_PKT_RING_UNDERFLOW_ADDR (CSR_BASE + 0x88L)
    > CSR_AAF_PKT_RING_UNPRIME_ADDR   (CSR_BASE + 0x8cL)

Generate the header without P&R to check this yourself:
`BOARD=breakout USB2=1 USB_DUAL_RATE=1 python3 avb_soc.py --soft-only
--output-dir build_csrcheck` -- and use an `--output-dir`, never the live
`build/`, whose `csr.h` is the one the running firmware was compiled against.

The firmware side is `#ifdef CSR_AAF_PKT_RING_LEVEL_RAW_ADDR`-guarded so it
compiles against an older header too, and `tools/stats.py --ring` prints
"firmware predates the 'r' opcode" instead of failing. Note the headroom:
coderam is at 92108/98304 bytes (93%) with the `'r'` reply in.

A TALKER CYCLE ALSO CLEARS IT, without JTAG: dropping the flows takes `en` low,
and `If(~aligned_once, NextValue(rd, rd_anchor))` reloads `rd` from a pointer
that tracks `wr`. Verified on the wedged board 2026-08-14 -- the ring re-primed
and held at centre after the flows rebound. Worth keeping as an auto-recovery
even with the arithmetic fixed.

A FAILED INSTRUMENT AND A FIX FOR THE WRONG FAULT, both recorded so nobody
repeats them: `usb_sample_data` bit 7 (`valid`) reads 0 on a HEALTHY board (it
is a firmware drain path; the hardware drains the FIFO at 48 kHz), so it
discriminates nothing -- a verdict built on it fired on a healthy board. And the
frame-marker flywheel in `dante_packetizer.py` was built on the inference that
`first` had stopped; `frame_holdover` read **0** through the fourth wedge, so it
never had. The flywheel is harmless and stays (it is inert when healthy, proven
in `sims/sim_frame_flywheel.py`), but it guards a fault that has never occurred.

**The residual drift is +0.35 ppm, and it is what makes a receiver's Latency
Status walk.** Full handoff for fixing it: [`DRIFT_HANDOFF.md`](DRIFT_HANDOFF.md). Measured 2026-08-11 with `tools/mclk.py watch 120`, discipline
ARMED and healthy (target and applied both -4354 ppb, 0 trips, ring 63-67, 0
underruns):

    drift slip  +2 samples / 120 s  =  +0.35 ppm  =  +1.2 ms/hour

`drift` is `emitted timestamp - PTP`, so a positive slip means our timestamps
run AHEAD and a receiver's measured latency falls by **1 sample per minute**. A
RedNet A16R at 0.25 ms was watched doing exactly that: 134 µs, decaying, then
clamped at 0 and its Latency Status went grey. At that rate the re-anchor
backstop (now 8 samples) fires about every 8 minutes, and every fire is a
timestamp discontinuity.

This is why neither a timestamp bias nor a tighter re-anchor threshold fixes it.
The error does not settle anywhere — it walks — so any bias only chooses where
in the walk you start:

| `DANTE_TX_TS_OFFSET` | A16R reads (budget 12 samples) | verdict |
|---|---|---|
| 0 | 0/12 | grey, but full margin — we always arrive early |
| -2 | 8-10/12 | green, ~25% margin |
| -4 | 9-11/12 | green, one excursion from red |

**FIXED 2026-08-12: the drift was NCO integer rounding, not a servo error.**
The increment is a 32-bit integer of ~4.12e6, so one LSB is 242 ppb, and two
roundings sat inside it -- `(fs << 32) / sys_clk` = 4123168.604 rounded UP to
4123169 (+96 ppb, permanently) and `inc_for_ppb()` truncating (+37 ppb). Total
+133 ppb; measured drift +0.13 ppm. The PTP servo is structurally blind to it:
`rate_ppb` measures the crystal against the Leader, not how coarsely we render
that estimate into an integer, so the rate loop reported itself converged with
0 trips and a healthy ring while the clock ran fast. The error is positive
across the whole operating range, which is why every drift observation in this
project has had the same sign.

Found by asking why `mcr` from the AVB project never drifts: its pure-gPTP path
takes the ratio from the 52-bit gPTP addend and rewrites the increment at ~4 kHz,
so the value dithers across the grid point and the quantisation averages out.
`mcr_dante` lost that when it went to one slew-limited write per second.

The fix carries the increment in 16.16 fixed point and sigma-deltas the fraction
(`nco_render()`), so the MEAN increment is exact. Measured after: **534
consecutive drift samples all reading -3, least-squares slope +0.00 ppm**,
against +22.5 samples/hour an hour earlier on the same board. Ring 63, zero
underruns, `anchors` 1. A16R 8/12 at 0.25 ms, AM2 21/48 at 1 ms, DVS 88/192 at
4 ms, all parked. The phase term is now **ON** as well, but for its real job — holding absolute
position, which the dither cannot do. After the dither the drift still crept
~1.5 samples/hour with nothing pulling it back. Enabled 2026-08-12 with
`PHASE_TARGET_SAMPLES = -1`: it contributes **65 ppb against a 2000 ppb clamp**,
holds the A16R at 6/12 dead centre, and ran with ring 63..67, zero underruns and
zero rejects. Before the dither the same loop needed ~400 ppb just to fight the
rounding error, and its target had to be skewed to -4 to cancel the actuator
quantisation the dither has since removed.

Two other real bugs were found on the way and are fixed: the drift measurement
could go whole seconds wrong when the talker dropped (`dante_tx_media_phase()`),
and the re-anchor band was `8 + (fmax - fmin)` = **64 samples** with fpp 4 and 60
both bound -- 5.3x an A16R's entire budget, so subscribing DVS silently disabled
the backstop for every low-latency receiver. Full account:
[`DRIFT_HANDOFF.md`](DRIFT_HANDOFF.md).

The original analysis, kept because the reasoning about bias-vs-walk is still
correct:

THE FIX IS TO NULL THE RESIDUAL, not to bias around it: a slow outer trim that
closes the loop on the MEASURED drift (the quantity `mclk.py watch` already
computes) rather than trusting the PTP servo integral alone. It needs to be slow
and low-authority — pointing `mcr` at `g_ptpv1` has broken audio twice before,
see below — but +-0.5 ppm of authority updated once a minute would hold the
error still, and a still error is what lets a 0.25 ms receiver keep both its
margin and its indicator.

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

**The seed lottery: SOLVED 2026-11-03. A build timestamp was in a ROM.**
For months the same `--seed` scattered sys Fmax across 52-65 MHz run to run,
sweeps never predicted real builds, and rebuilding a validated seed did not
reproduce the validated bitstream. `PYTHONHASHSEED=0` reduced the scatter but
never removed it, and the residue was written off as toolchain noise.

It was not noise. `SoCCore(ident_version=True)` makes LiteX append
`build_time()` -- date **and time to the second** -- to the ident string and
bake it into the `identifier` memory. That is emitted as
`colorlight_i9plus_mem.init` and pulled in by `$readmemh`, so yosys
constant-folds it. Two builds started in different seconds therefore hand
nextpnr DIFFERENT NETLISTS. Measured, with identical RTL, identical `.ys`,
byte-identical loader ROM and byte-identical `csr.h`:

| | hunt build | serial build |
|---|---|---|
| ident ROM | `...19:01:16` | `...19:27:51` |
| cells in the netlist | 20496 | 20443 |
| cell names in common | | ~11500 of 20496 |

A seed is only a seed FOR A GIVEN NETLIST. Re-rolling the netlist every build
made `--seed 8` mean something different each time, which is why the sweep was
never predictive and why `floorplan_usb.py` -- which matches on cell NAMES --
constrained 1419 / 1427 / 1436 cells on "the same" design.

Supporting measurement, so this is not a story: yosys run twice on identical
input in the same directory produced a **byte-identical 36 MB JSON**. The tool
was always deterministic; the input was not.

The fix is `ident_version=False` in `avb_soc.py` (see the comment block there).
Nothing reads the identifier at runtime, so the timestamp bought nothing. Do NOT
substitute a git hash: that re-rolls the netlist on any commit, including one
that never touches the gateware.

Consequences, all good: the same seed now reproduces, a P&R-only sweep from a
fixed `.json` is a valid predictor again (`$SCRATCH/pnr_sweep.sh` pattern -- copy
the json+xdc, vary only `--seed`), and "rebuild seed N" gives back the bitstream
you validated. One caveat survives: Fmax must still never pick the seed on its
own. Seed 1 once had the highest clean sys Fmax (63.54) and the LOWEST USB
margin (79.62), and flashed, the MacBook enumerated the audio device and showed
**no outputs at all**. Prefer the higher `USB_MHz` when seeds tie, then confirm
on hardware -- see the header of `tools/seed_sweep.sh`.

**Current build: seed 5, 2026-11-07, with ICAP in the netlist.** Six of eight
seeds passed all five clock floors on the corrected recipe (seed 6 failed on
both USB domains, seed 1 was a genuine HeAP stall after two iterations and
912 s of silence).

| seed | sys | usb | usb2 | weakest USB | eth_tx | eth_rx |
|---|---|---|---|---|---|---|
| 2 | 65.09 | 67.15 | 67.91 | 67.15 | 152.16 | 135.26 |
| 3 | 63.69 | 68.64 | 66.77 | 66.77 | 145.48 | 177.02 |
| 4 | 58.95 | 68.94 | 66.16 | 66.16 | 139.86 | 134.44 |
| **5** | **61.90** | **67.71** | **70.73** | **67.71** | **167.59** | **132.64** |
| 7 | 55.97 | 61.83 | 67.27 | 61.83 | 154.92 | 147.71 |
| 8 | 55.44 | 64.52 | 67.41 | 64.52 | 151.75 | 151.13 |

Picked on the **weakest USB domain**, which is how "prefer the higher `USB_MHz`"
generalises once there are two PHYs: both carry a 60 MHz ULPI bus and the
failure mode (ingress leak) is per-PHY, so the worst of the two is what decides.
Seed 4 has the highest Main and the lowest Backup — the wrong trade. Confirmed
on hardware, both PHYs streaming 48 ch at 96 kHz simultaneously **with random
data**: leak `+0.0000%` on each, `ep_out` 18,432,197 and 18,432,190 B/s against
an expected 18,432,000 (**+0.001%**), `sample_overflow` 0, and unchanged across
an OSC handover. Written to SPI with `--permanent`; `build_seed5/` is the
validated artifact, not the number 5.

One trap on the way, worth the line: `STALL_SECS` defaulted to 240 in the sweep
while the recipe uses 900, and that killed seeds 1 and 3 mid-HeAP. Seed 3 then
passed cleanly on the retry with `HeAP Placer Time: 79.99s`. HeAP takes 49-80 s
here and the log does not grow while it solves, so a short quiet window throws
away good seeds. And the sweep reported those kills as `NOBUILD` rather than
`HEAP-STALL`, because the marker was written into the same file the dying build
still held an open fd on and got overwritten -- the tell being that all eight
logs came out byte-identical in size. The marker now lives in its own file.

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
**Both are required** for reproducible placement. They were never sufficient on
their own: the 52–65 MHz scatter that survived them was the ident ROM's build
timestamp re-rolling the netlist — fixed by `ident_version=False`, see the seed
entry above. If scatter ever comes back, diff the generated `*.init` files
between two builds before blaming the placer; that is the measurement that
found it.

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
| `r` | | ring health -- `ring_level_raw` (**unclamped**, signed), `ring_level_min`, `ring_underflow`, `ring_unprime`, plus four raw peak registers. `tools/stats.py --ring` |
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

**Read `r` before believing `fifo_level` from `?`.** `fifo_level` is
`block_level`, which CLAMPS a negative ring level to 0 — and that clamp is why
the four ring-pointer wedges of 2026-08 each presented as "fifo_level 0, every
ingress counter perfect" and were misread three times as "the decoder is
producing nothing". `ring_underflow` counts episodes where the consumer
overtook the producer and **must stay 0**; `tools/stats.py --ring` refuses a
verdict while the talker has never been enabled, because with the gateware
enable low `rd` is parked and the level is modular rather than occupancy.

### OSC control surface (port 9000)

`firmware/osc.c`, exercised with `tools/osc.py`. The A/B select was already
reachable over 7779, but only through this project's own binary opcodes — fine
for a bench script, useless from QLab or any OSC-speaking desk. This binds a
second port and speaks the subset of OSC that a switch needs.

| address | types | does |
|---|---|---|
| `/usb/source` | `i` `f` | 0 = Main, 1 = Backup |
| `/usb/main` | — | switch to Main (argument ignored) |
| `/usb/backup` | — | switch to Backup (argument ignored) |
| `/usb/rate` | `i` `f` | 48000 or 96000 |
| `/ping` | — | replies `/pong`, so a desk can prove it has the board |

```
tools/osc.py backup                 # switch, then read the mux back
tools/osc.py --float source 0       # as a desk would send it
tools/osc.py rate 96000             # then replug USB so the host re-reads it
tools/osc.py ping
```

**Both `i` and `f` are accepted, deliberately.** Plenty of controllers send
every number as a float — TouchOSC and many QLab cue setups do. Accepting only
`,i` works on the bench and then silently ignores the desk, which is the worst
way for this to fail. A missing type tag is treated as "no argument", because
some desks send exactly that for a trigger. An unknown type tag is logged, not
dropped. `#bundle` is **detected and reported, not parsed**: a single message
covers a switch, and a half-supported bundle parser is a worse place to find a
bug than an explicit "not supported" line on the console.

`/usb/rate` writes `cfg.sample_rate` to NV config and applies it live, but the
host has already read the descriptor — **replug USB** (or power-cycle) before
expecting the new rate. 48/96 kHz from Dante Controller goes through the info
records, not through here.

**The switch is hard, and `forced` is how you know.** OSC is fire-and-forget
UDP: the board does not acknowledge, so "I sent it" is not "it moved". That is
why `tools/osc.py` reads the `U` opcode on 7779 afterwards and prints `select`,
`switches` and `forced`. A `forced` increment is **not** a fault — it means the
candidate offered no frame marker and the mux went anyway, which is the asked-for
behaviour: you do not switch for fun, so when you say go, it goes. Measured
2026-11-04 with nothing streaming, every switch is `forced +1`, and with the
Backup stream killed outright the mux still moved (`handovers 3, forced 1`).
Under a live double stream — both PHYs at 48 ch / 96 kHz, 36.9 MB/s on one
controller — handovers were aligned and `forced 0`.

**Read the counters back after a settle, not immediately.** The CSR write lands
at once but the handover completes on a frame boundary, so an instant read-back
catches `select` already updated while `switches` has not ticked. That made
`tools/osc.py` report a perfectly good switch as a failure until it was taught
to poll (~1.8 s) for the counter.

### Dual USB: the build that works

```
BOARD=breakout USB2=1 NEXTPNR_USB_FP_LIGHT=1 ./build.sh --output-dir build_dante_s5 --seed 5
```

Measured 2026-10-30, both PHYs enumerating at **480 Mbit/s, 6/6 power cycles**,
taps from the gateware reset (no runtime command), two distinct ALSA cards:

```
 2 [Main   ]: USB-Audio - N-Series Dante Main     at usb-...-8,  high speed
 3 [Backup ]: USB-Audio - N-Series Dante Backup   at usb-...-10, high speed
```

Fmax: sys 62.2, **usb 70.1**, usb2 63.8, eth_rx 169.9 MHz.

**Three things each had to be right, and each was a separate dead end first.**

1. **`NEXTPNR_USB_FP_LIGHT=1`, not the full floorplan.** The default box pulls
   ~3000 cells, because the net-connectivity pass grabs every net carrying the
   subsystem prefix and so drags in the whole LUNA core, which has no
   pad-timing relationship at all. HeAP never completes an iteration on that:
   **11 seeds, all stalled, none with a placement error**. SA on the same box
   does print the reason -- `failed to place chain ... PACKED_CARRY4`: a carry
   chain needs contiguous vertical SLICEs in one column and cannot be split
   across a region boundary, so the regions were impossible, not merely tight.
   The light mode constrains only the ~690 cells on nets that touch the ULPI
   **pads** -- which is what the floorplan was added for in the first place --
   and places in 25-35 s.
2. **Seed 5.** The 60 MHz ULPI domain is systematically tight here. Over 12
   seeds on one netlist, only seed 5 cleared 60 MHz on `usb`; the rest landed
   49.8-59.5, i.e. **failing**, and earlier builds will have been running with
   negative slack on that domain. If USB behaves intermittently, check this
   number before anything else.
3. **The pinout** (below), including the physically swapped DIR/NXT.

**An IDELAY tap is a property of a placement, not of the hardware.** On this
build both PHYs enumerate at *every* tap 0..31. On a seed-8 build with usb at
60.14 MHz the eyes narrowed to 28..31 (Main) and 9..31 (Backup) -- and when a
later change moved placement across Backup's edge, Backup vanished from the bus
entirely, because its pinned reset was 8 and its eye started at 9. Never pin an
edge value, and re-measure with `tools/tapsweep.py` after any gateware change.
A narrow eye is a symptom of too little Fmax margin in `cd_usb`, so look there
before reaching for a sampling-phase change.

`tools/tapsweep.py` power-cycles the hub port through
`/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port<N>/disable`, so a 32-tap sweep runs
unattended; it needs the `T` opcode in `firmware/dstats.c` because the serial
console went silent on this bitstream while UDP kept answering.

### USB3300 ULPI pinouts (`BOARD=`)

Two physical carriers exist and **their pinouts differ**. `avb_soc.py` selects
with `BOARD=` (default `breakout`, the board currently fitted); the single
source of truth is `_ULPI_PINS` there — read it, don't copy it:

```
BOARD=breakout python3 -c 'import avb_soc; print(avb_soc._ULPI_PINS)'
```

| | `ext` (old carrier) | `breakout` (current) |
|---|---|---|
| Main CLK/DIR/NXT/STP/RST | T4 / T3 / U2 / U3 / R2 | *same* |
| Main D0–D7 | V2 V3 W1 **W2 Y1 AA1 AB1** Y2 | V2 V3 W1 **Y1 AA1 AB1 W2** Y2 |
| Backup | bank 14 — **known bad** | bank 34, all-odd SODIMM |

RST is **active HIGH** — `Pins`, not `PinsN`. The 60 MHz ULPI clock is
**PHY-sourced**; a PLLE2 re-emits it. `USB_BACKUP=1` puts the single (proven)
USB subsystem on the Backup pins — ~14k cells, places in ~3 min — to validate
Backup hardware without the dual-USB netlist.

**Three traps, each of which has cost a session:**

1. **The wrong data order is silent.** The bus toggles, the PHY clocks, you get
   garbage. `breakout` wires D[7:0] in ascending SODIMM order, which rotates
   four bits relative to `ext`. Read the ULPI vendor ID (regs 0x00/0x01,
   `0x0424` on SMSC) before suspecting anything else.
2. **DIR and NXT are physically transposed at position B.** J5's pin order
   differs from J4's, so the hand-soldered leads crossed; the gateware swaps
   them back (`dir=AB7`, `nxt=AB6`). This — not timing — is why the 2026-07-09
   dual-USB bring-up saw `error -71` at *every* clock phase: `oe = ~dir_i` was
   driven from the NXT pin, so the bus was never released during the PHY's
   turnaround and we read back our own TXCMD bytes. **A phase-invariant failure
   is a wiring permutation, never a timing problem** — turning the IDELAY tap
   cannot fix one.
3. **SODIMM 88 (V4) / 90 (R4) / 89 (Y4) must stay NC.** They are the
   differential partners of the two ULPI clocks; a switching signal on any of
   them couples straight into a 60 MHz clock. Until 2026-10-30 this repo's
   `ulpi2_io()` used V4 *as* the clock and Y4 as RST.

**Wiring is not optional detail.** Twist CLK with **its own ground**, never
paired with another active signal. Twenty gateware variations failed identically
until the clock was rewired this way. A marginal ULPI crystal presents as a
touch-sensitive, intermittent link.

---

## Console

**THE CONSOLE IS QUIET BY DEFAULT SINCE 2026-11-06, AND THE KEYS BELOW STILL
WORK.** Only the printing is gone. The narrative was the biggest single thing in
coderam and it was the format strings, not the UART:

    203 printf format strings    10084 bytes = 77% of ALL .rodata
    vfprintf                      1712 bytes
    uart_* driver + printf          676 bytes

    quiet    coderam 72364 / 98304 (73%)
    verbose  coderam 91980 / 98304 (93%)

`firmware/log.h` makes `printf` a macro that does not evaluate its arguments, so
quiet the literals are never referenced and never reach `.rodata`. The text
stays in the source on purpose — in this project the messages are the record of
what was measured. `make DANTE_VERBOSE=1` brings all 203 lines back.

The UART itself stays: `firmware/loader/loader.c` reads it (that is what
`tools/netload.py` talks to), an RP2350 link is planned on it, and every key
below with a side effect still works — `r` reboot, `t`/`T` talker, `x` rx_gate,
`d` mcr_dante, `z` cap_reset. The keys that only PRINT are inert unless you
build verbose. `g_verbose` ('v') now gates lines that no longer exist: a runtime
flag cannot save a format string, only compile-time removal can.

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
  osc.py                send OSC on 9000 and read the mux back on 7779
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
  net.c                 IPv4 / UDP / ICMP / IGMP / ARP.  Took over the IP/UDP
                        handling that the original osc.c carried itself (it
                        predated net.c); osc.c was then rewritten on top of
                        it and is the OSC surface described under Console
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
  osc.c                 the UDP:9000 OSC surface: A/B select + rate,
                        for a show controller (see Console)
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

### Setting the IP from Dante Controller

Controller's Network Config tab said **"This device does not support static
addressing"**, and that was the device's own answer, not a network problem.
`IVersions.hasStaticIp()` in `DanteController.jar` is
`inferredCapabilities & 0x4000` — **bit 14 of the capabilities u32 at
`content[0x14]` of the `0x60` versions message**. A PRODIGY1-32 sends
`0x0e7cd65b` with that bit set; we sent `0x000010db` without it. One byte in
`firmware/dante_info.c`:

```c
c[0x16] = 0x10 | 0x40;      /* 0x40 == bit 14 == network configurable */
```

Everything tried before that was in the wrong message, and each was ruled out
by measurement: four extra `0x1100` keys, a unicast mDNS reply, `arcp_vers`
2.8.15 (a regression — audio kept flowing but Controller stopped showing the
subscription as patched), the `dbc` and `chan` services, the `0x3320` code, the
extended `0x1000` form, CMC registration, and a second interface in `0x11`.
That last one made Controller draw primary **and** secondary, which looked like
progress; both halves stayed grey.

**`0x11` interface status** and **`0x13` interface control** are laid out in
`STATIC_ADDRESSING_HANDOFF.md`, derived from the
`conmon_audinate_interface_*` accessors in `libDanteController.dylib` and then
checked byte for byte against a real PRODIGY. Four things the decompile left
open were settled by capturing Controller's own Apply — unicast, so the
board's capture ring (`tools/cap_fetch.py`) is the only vantage point:

- the **interface index counts from 0**,
- the static setter sends flag values `0x0002` **and** flags-valid `0x0002`,
- the record is `0x1c` bytes with `+22..+27` zero and the domain-name offset at
  `+24` equal to 0,
- Controller **retries the write every ~2 s** until the answer reflects it —
  which is why `cfg_save()` is gated on the stored value changing, or an open
  dialog would grind the SPI flash.

We report **one** interface, because the gateware instantiates one RGMII PHY.
The PRODIGY reports two because it has a redundant port.

**A new address is applied by a reboot, not live.** It goes to `g_cfg.osc_ip`,
which `main.c` already feeds to `net_init()` at boot, and is reported in the
`0x11` reboot records — that is what makes Controller say the device must be
rebooted, and it is how real Dante hardware behaves. Writing `g_net_ip` under a
running stack would strand every bound flow, every ARP cache entry on the
segment and the reply still in flight. Setting the address back to the live one
cancels a stored change.

**The reboot is not self-service yet.** The firmware lives in coderam, not
flash, so a reset lands in the loader and does not come back without
`tools/netload.py`; the UART RX is not wired on this breakout, so `'r'` on the
console does nothing. A JTAG reconfigure (`./flash.sh`) is the only reset, and
it requires the audio to be stopped.

**This device is statically configured by construction** — it reads its address
out of `g_cfg` at every boot and has no DHCP client and no link-local
negotiation — so the `0x0002` interface flag is unconditional, and a write
asking to stop being static, or to turn on DHCP or link-local assignment, is
refused rather than accepted and ignored. 169.254.x.x sitting in the link-local
*range* is not the same as link-local *assigned*.

Because Controller can move the address, the board's management IP lives in one
place: `tools/dante_board.py`, overridable per invocation.

```bash
DANTE_BOARD=169.254.9.77 tools/stats.py      # flash.sh honours it too
```

### Controller's Reboot button: info opcode `0x90`

Measured off the wire 2026-11-07, which was the only way: the command is unicast
to our info port, and it was invisible for one round of looking because the
capture ring dropped all multicast and so could not tell "DC sent nothing" from
"we cannot see it".

```
RX INFO 169.254.228.170:8700 -> :8700  len 78
ffff 0024 6a5d 0000 c8a362ebd51a0000 "Audinate" 07 3d 00 90 00 00 00 64
00 01 00 00
```

36 bytes, four of content, version `0x073d` (a **controller** asking for
something, against `0x072a` which is a device talking about itself), preceded by
a CMC `0x1001` device-advertisement request — the same two-step DC uses before
the `0x13` interface write. **Sent exactly once, with no retry**, unlike that
write which it repeats every ~2 s: a reboot needs no acknowledgement, because
the device going away is the acknowledgement. So we answer nothing.

inferno does not handle `0x90` at all, so there is no reference for it. Only
action `0x0001` is honoured here; the capability byte at `content[0x17]` of
`0x60` is annotated "identify / rate config / reboot / factory reset" with the
bits never isolated, so `0x90` may be a family of system commands with the u16
selecting which. Anything else is counted and logged, not obeyed.

**A reboot is not what this ends up doing, and the first reason is that
`ctrl_reset` does not come back on this design — measured before today.** `firmware/dstats.c`, at the `'R'`
rate opcode: *"Een reset landt in de loader omdat de firmware in coderam staat
en niet in flash — gemeten: het bord kwam niet terug, **ook niet met
netload**."* That note is why the sample rate is applied live rather than by
resetting, and it is the whole reason a reboot command cannot simply pulse the
SoC reset. It was walked past three times on 2026-11-07; all three attempts left
the board silent on the wire (no ARP, nothing from its MAC under `tcpdump`) and
each needed the JTAG cable.

The SPI image was **not** the problem, and ruling that out is worth recording:
the ROM loader demonstrably contains the flash fallback (its `[ldr] flash image`
strings are present in `build_seed8`'s `rom.init`) and the image at `0x300000`
passes its own CRC after `--permanent`. The reset itself does not come back,
presumably because a warm CPU reset leaves the clocking or the PHY in a state
`phy_init()` does not restore.

#### ICAPE2 with IPROG looked like the answer. It cannot work on this flow.

The textbook mechanism on a 7-series part is ICAPE2 with IPROG: write `0x0f` to
the CMD register (`0b00100`) and the FPGA reconfigures itself out of SPI flash,
a full clean boot. LiteX has the core, it was instantiated (`icap`, pinned at
`csr_id=13`), and the firmware did everything the references ask for — WBSTAR
then CMD=IPROG, through a core that sends the full sync sequence and
bit-reverses every byte as ICAPE2 requires.

**It does not reconfigure. Measured 2026-11-07 on the first real press of the
button:**

- The FPGA did **not** reset. USB stayed enumerated on the same device numbers,
  so the fabric was untouched; only the CPU stopped, in the infinite loop that
  used to follow the ICAP write.
- The SPI read path was not at fault: the interlock immediately before had
  streamed the whole image out of flash and verified its CRC.
- `tools/icap.py` then read **every** ICAP register as `0x00000000` — IDCODE and
  STAT included — **while `icap_done` arrived every single time**. So the
  divided ICAP clock runs, LiteX's FSM completes, and ICAPE2's `O` bus returns
  nothing.

The reason is in prjxray, and it is structural. The whole `CFG_CENTER_MID` tile
has **nine** documented features where a CLB tile has 680:

```
CFG_CENTER_MID.BSCAN.JTAG_CHAIN_1..4
CFG_CENTER_MID.DCIRESET.ENABLED              <- activation bit for DCIRESET
CFG_CENTER_MID.ICAP.ICAP_WIDTH_X16
CFG_CENTER_MID.ICAP.ICAP_WIDTH_X8            <- width attribute only
CFG_CENTER_MID.STARTUP.PROG_USR
CFG_CENTER_MID.STARTUP.USRCCLKO_CONNECTED    <- activation bit for STARTUPE2
```

DCIRESET has an `ENABLED` bit. STARTUPE2 has `USRCCLKO_CONNECTED` — which is
exactly the "Vivado-only property" the open-source community hits on flash
access, and having it characterised is *why* the `S7SPIFlash` core works here.
**ICAP has no activation or connected bit at all.** The site is routed — the
FASM wires all 32 `I` and 32 `O` pins plus `CLK`, `CSIB` and `RDWRB` — and never
switched on, and no openXC7 bitstream can switch it on, because nobody ever
fuzzed the bit. That is an inference from the sibling sites in the same tile
rather than a proof, and it fits every measurement.

Nor is there a counter-example in the wild: nobody reports a working IPROG on an
open-source 7-series bitstream, openXC7's own fork confirms ICAPE2 only as far
as place-and-route, and the one project that documents the warm-boot steps
[records that the commands were never
run](https://github.com/fpgas-online/fpgas.online-docs/issues/126).

A smaller lesson came with it. The ICAP write was followed by `for (;;) { }`,
justified in a comment: a successful IPROG takes the die down mid-sequence, so
there is nothing to synchronise with and polling `icap_done` could only hang.
True of a *successful* IPROG — and it ignored the other case entirely, which
turned one failed ICAP write into a dead board until a power cycle. **An
unbounded wait on hardware that may not answer is not robustness, it is a
latch.** The wait is now bounded and the handler survives.

#### So the Reboot button does not reboot: it applies the address live.

Which is the better answer regardless. A reconfiguration drops the USB device
mid-transfer, and 48 channels of USB audio are the product — xHCI can take the
whole host controller down when a device vanishes underneath it. A real Dante
chip reboots for an address change because that is how its firmware is built,
not because a network stack needs it.

This costs almost nothing, because **nothing in this firmware caches our IP**:
`mdns.c`, `dante_cmc.c`, `dante_tx.c`, `dante_info.c` and `net.c` all read
`g_net_ip` at the moment they build a packet. The one cached copy is the AAF
packetizer's `src_ip` CSR — it emits audio with no CPU in the loop — so that
write was factored out of `dante_tx_init()` into `dante_tx_set_src_ip()` and is
re-latched. The mDNS A record goes out with the cache-flush class
(`DNS_C_IN_FLUSH`), so a single `mdns_announce()` replaces the old address in
every resolver on the link. PTPv1's group is re-joined because a membership
report carries the source address.

It is applied from `dante_info_poll()`, not in the packet path: the `0x13` write
that stored the address was acknowledged long before, `0x90` gets no reply, and
changing `g_net_ip` underneath a handler still parsing a frame addressed to the
old IP is the kind of re-entrancy that costs an evening.

**Measured, .200 → .201 → .200, with both PHYs streaming 48 ch at 96 kHz
throughout:**

| | before | after both changes |
|---|---|---|
| USB device numbers | 110 / 111 | **110 / 111 — no re-enumeration** |
| USB stream status | `Running` 96 kHz | `Running` 96 kHz |
| `packets` | 1,042,129 | 2,355,767 — **continued, never reset** |
| `underrun` | 126 | **126 — not one new underrun** |
| mDNS | `.200` | followed to `.201` and back |

`flows_active` dips 2 → 1 for a few seconds, because a receiver's flow was bound
to the old source IP; it returns to 2 on its own, `flows_rejected 0`. A reboot
would do that too, and take the audio with it.

So a self-service address change needs **no** gateware change. `reboot_applied`
counts it; `reboot_nochange` counts a press with nothing queued.

**The interlock, and why it needed two attempts.** This firmware runs from
coderam. A reset enters the loader, which waits for netload and falls through to
the SPI image at `0x300000` — and **halts** if that image is not loadable. So
honouring a reboot on a board that has only had volatile loads turns one click
in Controller into a JTAG recovery.

The first version checked only the magic and a plausible length, with a comment
admitting it did not cover "flashed but since corrupted" and arguing that case
was far-fetched. It was not: the first test passed that check, reset the board,
and the board did not come back. It now streams the image out of SPI in
256-byte chunks and verifies the CRC with the same bitwise IEEE CRC-32 the
loader applies — an interlock that does not check what the loader checks is not
an interlock. It costs ~100 ms of stalled main loop and will glitch the flows,
which is the right trade for a rare operator action that ends in a reset anyway.

**And the limit that bit next:** the interlock proves an image is *loadable*, not
that it *matches the running gateware*. This board's SPI already held an `INFN`
image with a valid CRC, from older firmware built against a different CSR map,
which comes up mute. The image header carries no map identity, so nothing in the
firmware can tell. What closes it is `./flash.sh --permanent`, which writes the
bitstream and the firmware from one tree and refuses a `csr.h` mismatch.

**Observability**, because the shipping build has no console:

```bash
tools/stats.py --info      # 'I' -> 'INF1'
```

reports `reboot_req` / `reboot_odd` / `reboot_refused` /
`reboot_unsupported` / `reboot_applied` / `reboot_nochange`, and prints a
verdict that names which of those cases happened. Without them a board that
stays up after the button is pressed is indistinguishable between "DC sent
nothing", "we did not recognise it" and "we refused". It has its own reply
rather than extra words on the main one: growing that from 200 to 208 bytes once
killed the port outright.

### Secondary network: why there is only a primary

**Decision 2026-11-07: primary only.** Not because the protocol is unknown — we
have the `0x11`/`0x13` layout and the addressing convention — but because of
BRAM and timing closure. Written down so it can be revisited with numbers
instead of re-argued.

**A Dante chip has one MAC.** The Dante Pro S1 datasheet's block diagram is
explicit: the die holds a single `Ethernet MAC` behind a 100 Mbps RMII
interface *with MDIO*, feeding an **external** `PHY / Switch`, out of which come
`Primary` and — the label is exact — **`Secondary (Switch only)`**. Brooklyn 3
says the same thing in prose: "Standard RGMII/MII interface for Ethernet PHY
**or switch chip**". Neither datasheet contains the word "redundant" anywhere.
So the second port is not in the chip; it appears when the host board is built
with a switch IC instead of a plain PHY.

**Which means a single MAC behind a plain switch can only do switched mode** —
daisy-chain, both ports bridged, one MAC address, one IP. That is what
"Secondary (Switch only)" describes.

**True redundancy is a different animal.** The two networks must *never* be
bridged, and the device presents two MACs and two IPs. Measured on a
PRODIGY1-32 (Brooklyn 3) over `0x11`:

| | MAC | IP | speed | flags |
|---|---|---|---|---|
| primary | `00:1d:c1:2d:9c:60` | 169.254.143.65 | 1000 | `0x0001` |
| secondary | `00:1d:c1:2d:9c:61` | 172.31.143.66 | 0 | `0x0000` |

MAC + 1, the secondary on 172.31/16, and the secondary reporting **link down
separately from the primary** — a plain unmanaged switch cannot report per-port
state to one MAC, a managed one over MDIO can. That points at a managed switch
on a single MAC, but no datasheet says so and it is inference, not measurement.

**Our board is better placed than the chip.** The i9plus carries **two
independent RGMII PHYs** wired straight to the FPGA (`eth 0` and `eth 1`); we
use `eth 1`, and `eth 0`'s twelve pins are **entirely unclaimed** — `mdio`,
`mdc` and `rst_n` are shared between the two, as the platform intends. No switch
chip needed: two real MACs are possible, which is the clean way to do
redundancy.

**What stops it, with the numbers:**

| | seed 8, the running bitstream |
|---|---|
| RAMB36E1 | **63 / 75 — 84%** |
| RAMB18E1 | 11 / 150 — 7% |
| SLICE_LUTX | 16632 / 65200 — 25% |
| SLICE_FFX | 9434 / 65200 — 14% |

BRAM is the wall, and LiteEth MACs live in BRAM. Worse, this design closes
`usb` timing at 60.64 MHz on **exactly one seed out of eight**, and a second PHY
brings two more 125 MHz clock domains (`eth_clocks 0` tx/rx) into that
floorplan — the seed lottery restarts with no guarantee that any seed closes.
And redundancy duplicates **every audio flow on both networks continuously**,
which doubles packetiser egress and needs a second header-generation path. That
is the part expected to break the design, not the protocol work.

**The `mode` field at `0x11` +2 is still not understood**, and it was briefly
got wrong here: it was set to 1, copied from the PRODIGY because the PRODIGY's
Network Config panel works — and the PRODIGY has two interfaces. Querying every
device on the bench on 2026-11-07:

| device | interfaces | `mode` |
|---|---|---|
| DAC8 (inferno on ESP32-P4) | 1 | **0** |
| PRODIGY1-32 | 2 | **1** |

and inferno's own `send_network_info` hardcodes the same lead bytes
`00 01 00 00 00 00`, i.e. `n=1` with `mode` 0
(`info_mcast_server.rs:355`). It does not name the field either, so what the
value *means* is still unknown — only that both one-interface references send 0
and the only two-interface reference sends 1. We send 0.

### Decoded on this bench — not in inferno

inferno is the best reference available, but it is not complete. These were
worked out here by querying real hardware with `tools/arc_query.py` and
comparing devices that differ in the behaviour of interest — a RedNet A16R
(Brooklyn-3) against a RedNet AM2 (UltimoX2). Probing our own device could not
have found any of them; the comparison had to be against hardware that works.

- **Heartbeat TLV `0x8002` — the Signal column in Controller's Transmit tab.**
  One byte per channel, TX first then RX, zero-padded to a multiple of four,
  and the byte is `round(-40*log10(peak/full_scale))` — **half-decibels BELOW
  full scale, so 0 is 0 dBFS and silence is 253..255.** We used to send all
  zeros with a comment calling them "a truthful no meter"; that claimed 48
  channels pinned at full scale, and Controller answered with a red speaker and
  an exclamation mark. inferno documents the encoding in
  `device_server/peaks.rs`, and a real device confirms it:
  `captures/dante_control.pcap` holds 169.254.61.114's heartbeat, whose
  `0x8002` decodes as `tx_count 0, rx_count 2, data_off 24, peaks [253, 253,
  0, 0]` — two silent RX channels reporting 253, its own noise floor. A quiet
  channel reports a HIGH number.

  Firmware cannot measure this: audio goes USB → gateware ring → packetizer and
  never passes the CPU. `dante_packetizer.py` therefore meters at the ring write
  point, the one place that sees every sample with its channel number already
  decoded, and keeps `max|sample|` per channel in a 48x8-bit array read over
  `peak_select`/`peak_value` and cleared once per heartbeat. **The meter is
  blind while the talker is off**, because the ring write is gated on the
  gateware enable — a heartbeat captured before PTP lock shows 48x 255 with USB
  ingress perfect, and that is not a fault.

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

- **A SMALL NETLIST CHANGE CAN COST YOU THE WHOLE SEED SET.** Measured
  2026-11-06: adding a 48x8-bit peak register file plus three CSRs was enough
  that **seed 1 — the seed behind the two bitstreams before it — stalled in
  HeAP** with no output for 904 s. Sweeping 1..8 on the new netlist gave two
  HeAP stalls, one build error, four FAIL and **one PASS**. So budget a seed
  hunt for any gateware change here, and do not conclude anything from the
  seed number: `tools/seed_sweep.sh` already warns that `sys` Fmax is not
  reproducible run to run, so the BITSTREAM is the validated artifact.

  Two of the four FAILs missed on `usb2` alone — the Backup PHY's clock, which
  `seed_sweep.sh` never checked until it was fixed that day. The same fix
  covers two more traps the tool had: its clock-name patterns had gone stale
  (the names gained a `main_` prefix, so `sys` and `usb` parsed as EMPTY,
  defaulted to 0 and reported **every** seed FAIL), and its BITSTREAM column
  called a leftover from a failed build `yes` — two seeds advertised August
  bitstreams as present. Freshness is now "the .bit is newer than its own
  csr.h", which doubles as the header-pairing check.
- **Inject clock constraints yourself, and know what binds.** openXC7 emits no
  `create_clock` for PLL-derived clocks, so without help every CDC is timed as a
  single-cycle path. `avb_soc.py` injects `create_clock` onto the PLL output
  *nets* and that DOES bind. It also emits `set_clock_groups -asynchronous`, and
  nextpnr-xilinx **ignores it** (`Info: ignoring unsupported XDC command`); so do
  `set_false_path` and `set_multicycle_path`. Only `create_clock` and
  `set_property` are supported. Consequence to plan around: **every
  cross-domain path in this design is permanently timed and must close**, which
  is why failing seeds fail on `usb`, `usb2` or `eth_rx` rather than on `sys`.
  You cannot tell this tool about a CDC, so put a real synchroniser on it.
- **Determinism needs both knobs AND a timestamp-free netlist.**
  `PYTHONHASHSEED=0` *and* ASLR off *and* `ident_version=False`. The first two
  were in place for months while the third silently re-rolled the netlist on
  every build; see the seed entry above.
- **Fmax does not predict USB quality.** A firmware change alters ROM contents
  and genuinely does re-roll placement (that part was never the bug). A seed
  must be validated on two independent axes: it places without a HeAP stall,
  *and* USB works on real hardware. Phase 0.5 exists to remove this tax.
- **Region constraints have a density ceiling, and HeAP will hang rather than
  say so.** Measured 2026-11-03 on one netlist with 1386 constrained USB cells:
  0.18 cells/tile places, 0.55 places, 1.02 and 1.23 both **stall at
  `Running main analytical placer` and never complete an iteration**. So size a
  box by cells-per-tile, not by eye, and treat a hang as "no legal solution"
  (`PLACER=sa` names the cell, but on this design SA needs well over 7 minutes
  to get there, so the density table is the faster instrument).
  A region also **only pulls its own cells in; it never evicts others**. When a
  boxed block starves a neighbour it is doing so by CONGESTION, which is why
  capping the USB box to Y0..50 dropped `usb2` from 64 to 47 MHz: Main's cells
  crowded Backup's own pad band (Y1-32), they were not forbidden from it.
- **Two latent bugs in `floorplan_usb.py`, both found 2026-11-03.** The
  pad-only mode (`NEXTPNR_USB_FP_CTRL=""`) did the OPPOSITE of what it
  documented: `"" in s` is always True in Python, so the empty match inverted
  the net filter and pulled every net (~3000 cells) instead of ~690 — a
  guaranteed stall. And the eth-TX branch calls `constrainCellToRegion`
  directly, bypassing `_constrainable()`, so it dragged the `storage_2` LUTRAM
  into a box with too few SLICEM columns — that is the documented 90-minute
  hang, and it was never the box geometry. Anything that constrains cells must
  go through `_pull()`.
- **`overrun_count` used to lie, and two counters must never be summed.** It
  added a BENIGN event (`send_req` landing mid-traversal, which `pend[]`
  re-serves, so nothing is lost) to an AUDIBLE one (a USB sample dropped on a
  full ring = one stale channel slot = a click). On a healthy 96 kHz stream the
  sum ran at 11k-28k/s and read as catastrophic sample loss. What proved it
  benign: `packets` sat at exactly 24000/s (= 96000/4), `underrun` was 0, the
  ring never came within 140 samples of its rail, and the rate DOUBLED under
  unrelated CPU load while `packets` did not move. Since 2026-11-03 they are
  separate CSRs -- `overrun_count` (benign, load-dependent) and
  `write_drop_count` (**must be 0**, reported by `tools/usbsrc.py`). The general
  rule: a counter that sums two causes cannot be read, only misread.
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
