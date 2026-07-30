# USB → native Dante conversion log

Fork of `avb-aes3` @ `466df96` (branch `48ch-6x8-aaf`), on branch `dante-native`.

Goal: 48-channel USB → native Dante transmitter. The 48ch USB ingress, media
clock NCO, LiteEth gigabit path and TSU are inherited working; what changes is
the network transport (AVB/AVTP L2 → Dante UDP/IP) and the clock protocol
(802.1AS gPTP → PTPv1).

Plan: `/home/lisp/.claude/plans/make-a-plan-how-atomic-melody.md`

**Legal position.** Dante is Audinate's trademark and the protocol is
undocumented and unlicensed. `/home/lisp/DANTE/inferno` (GPL-3/AGPL-3) is used
as a **specification only** — read, not copied. `/home/lisp/DANTE/statime`
(Apache-2.0/MIT) is safe to reference or vendor for PTPv1 field layouts.
`rtl/ulpi_wrapper.v` is GPL, so the bitstream is GPL-encumbered regardless.
Bench-only, no binary distribution, no affiliation with Audinate.

---

## Phase 0 — strip the AVB stack (subtractive only)

### Removed

Parked under `_avb_reference/` (readable, out of the build):

| Item | Size | Why |
|---|---|---|
| `firmware/avdecc.[ch]` | 23,795 B `.text` | IEEE 1722.1 control plane. Dante uses ARC/CMC/mDNS instead. |
| `firmware/srp.[ch]` | 5,682 B `.text` | IEEE 802.1Qat MSRP bandwidth reservation. No Dante equivalent. |
| `firmware/aaf.[ch]` | 2,459 B `.text` + **32,852 B `.bss`** | AVB audio format, software TX/RX. Dante's 9-byte header replaces it; the software audio path is gone for good. |
| `avtp_extractor.py` | ~4 BRAM sites | Gateware AAF RX observer. Milestone 1 is TX-only. |
| `crf_extractor.py` | — | Gateware CRF media-clock snoop. Dante has no CRF. |
| `avtp_stream_filter.py`, `i2s_clean.py`, `rtl/i2s_tx.v` | — | Already dead upstream. |

Also removed: the AVTP/MSRP branches of `dispatch_rx()`, the AVTP fast-path
(unneeded now that no audio is received), all five AVDECC callbacks, the
FAST_CONNECT pending-bind machinery, `usb_aaf_drain()` (replaced by a pure
`usb_fifo_drain()`), and the CRF hardware-feed paths in `mcr.c`.

### Kept as load-bearing

- `gptp.c` — TSU accessors (`:116-198`), median filter (`:456-470`), and
  `gptp_servo_update()` (`:600-797`). Phase 4 refactors the servo into
  `ptp_servo.c` and reuses it verbatim from `ptpv1.c`; gPTP stays working
  during bring-up so the shared servo can be A/B'd against a known-good
  802.1AS master.
- `mcr.c` — NCO servo. `cs=0` (discipline from the PTP addend ratio) is now
  the only mode; `cs=1` (CRF) is forced off even if an inherited NV blob asks
  for it, since CRF timestamps will never arrive.
- `cap.c` — packet recorder. Now a **primary tool**: it shows what the FPGA
  actually received, which beats host-side tcpdump when a switch is filtering.
- `aaf_packetizer.py` — template for Phase 5's `dante_packetizer.py`.
- `cfgflash.c` / `config.c` — NV config; `config.h`'s `reserved[72]` absorbs
  the Dante settings.

### New

- `firmware/pkt_geom.h` — packetizer stream geometry, extracted from the parked
  `aaf.h` so `main.c` no longer includes the AVB audio stack to learn how wide
  a stream is.

### Fixed in passing

`firmware/Makefile` pinned `BUILD_DIR = /home/lisp/FPGA/avb-aes3/build/...` as an
**absolute path**, so a fork silently compiled against the parent repo's
generated `csr.h` / `regions.ld`. That is the documented "stale CSR addr trap"
with a second repo attached. Now derived from the Makefile's own location.

### Memory result

The headline Phase 0 outcome. Deleting the AVB stack freed more than the whole
projected Dante firmware needs, in exactly the two places that were tight:

| | Before | After | |
|---|---|---|---|
| `.text` | 56,308 B | **25,512 B** | −54.7% |
| `.rodata` | 10,604 B | 6,560 B | −38% |
| `.bss` | 57,040 B | **22,400 B** | −60.7% |
| ROM used | 67,044 / 98,304 (68%) | **31.35 KiB (32.7%)** | ~65 KB free |
| SRAM used | 57,080 / 65,536 (87%) | **21.88 KiB (34.2%)** | stack headroom 8.4 KB → ~43 KB |

`main.o .bss` alone dropped 46,860 → 12,328 B (the 32,852-byte `aaf` block).
`cap.o` still holds 9,614 B for the capture ring — a candidate to move to SDRAM
in Phase 1.

Estimated new Dante/PTPv1/net code is 25–35 KB `.text`, which now fits the
existing 96 KB ROM with room to spare. **This is why SDRAM was re-scoped**: it
cannot help `.text` or stack anyway, because `cpu_variant="minimal"` has no
I-cache and no D-cache (`litex/soc/cores/cpu/vexriscv/core.py:360-365`), so
code or stack in SDRAM would route every fetch through Wishbone→L2→SDRAM on a
loop already at 245 µs/iteration. Phase 1 keeps LiteDRAM but for the capture
ring and cold heap only, gated behind `--with-sdram`.

### Change surface

314 insertions, 1111 deletions across 5 files plus 11 renames — purely
subtractive, so any USB or timing regression is attributable to the strip
rather than to new logic.

### Verification status

- [x] SoC elaborates with both extractors removed (`--soft-only`)
- [x] Firmware compiles and links; no undefined references
- [ ] Gateware places and routes at seed 7; Fmax recorded
- [ ] Hardware: boots, gPTP locks, USB enumerates 48 ch

Note `discard_in` on the LiteEth SRAM writer is now left undriven (Migen
defaults it to 0), restoring the unfiltered RX path to the CPU. If Dante
multicast flood ever swamps the dispatcher, reintroduce filtering as
`rx_gate.py` — a MAC allow-list — never as a protocol parser.

---

## Phase 0.5 — decouple firmware from the bitstream

**The problem.** Every firmware change alters the BRAM ROM contents, which are
part of the bitstream, which re-rolls nextpnr's placement. Cost per firmware
edit: a 6–10 min P&R, a seed sweep, and a USB hardware re-test. The remaining
work is thousands of lines of new C against an undocumented protocol — 100+
iterations, not 5 — so iteration cost would have dominated the whole schedule.
Phase 0 demonstrated this immediately: shrinking `.text` by 31 KB invalidated
the pinned seed and dropped `eth_rx_clk` from 133.28 to 117.90 MHz.

**The fix.** Split the image.

| Region | Size | Contents | Changes when |
|---|---|---|---|
| `rom` | 12 KB | frozen loader | ~never |
| `sram` | 64 KB | `.data`/`.bss`/stack | — |
| `coderam` | 96 KB | firmware `.text`/`.rodata` | every firmware build, **no P&R** |

`coderam` is added in `AVBSoC.__init__` via `add_ram(..., mode="rwx")` at
`0x20000000`. Deliberately *not* named `main_ram` — LiteDRAM claims that name in
Phase 1.

**Loader** (`firmware/loader/loader.c`, 7068 bytes = 57% of 12 KB). Tries two
sources in order:

1. **Netload** — a ~400 ms window at each reset listening for raw Ethernet
   frames (ethertype `0x88B5`, IEEE 802 local experimental 1). Stop-and-wait
   with an ACK carrying the next expected offset, so a dropped or duplicated
   chunk self-corrects. This is the development path: edit C → `make` →
   `tools/netload.py` → running, in seconds, with no P&R, no seed roulette and
   no USB risk. It also avoids `/dev/ttyACM0` entirely, which matters because
   that port belongs to the interactive console.
2. **SPI flash** — image at `0x300000` behind a 12-byte header
   (magic `INFN`, length, crc32), so the board boots standalone. Written with
   the existing openocd/bscan path via `tools/mkimage.py`; no new flashing
   infrastructure.

If both fail the loader prints and halts rather than jumping into garbage.

**Reuse over reimplementation.** The loader links `firmware/cfgflash.c`
unchanged rather than re-deriving SPI access. That driver encodes two things
that are easy to get subtly wrong: the STARTUPE2 warm-up (7-series masks the
first `USRCCLKO` edges, so the very first transaction is swallowed and JEDEC
reads `0xFFFFFF`) and the 40-bit top-aligned MOSI convention with one
self-contained auto-CS transfer per byte (manual-CS multi-transfer is flaky on
this SPIMaster). The loader carries no `printf` — libbase's `vfprintf` alone is
~1.7 KB, a seventh of the ROM — using a hand-rolled `puts`/hex instead.

**Escape hatch retained.** `avb_soc.py --bake-firmware` restores the legacy
single-image map (96 KB ROM, firmware baked, no coderam) for a standalone
shippable bitstream, or for recovery if the loader boot path ever breaks.

### Problems found and fixed during implementation

- **`VPATH` resolves targets, not just prerequisites.** The loader Makefile
  reuses `crt0.o` and `cfgflash.o` from `../`, and make found the *already-built*
  copies there, declared the targets satisfied, and passed bare filenames to the
  linker — which then couldn't find them. Fixed by linking with `$^` (VPATH-
  resolved paths) instead of `$(OBJECTS)`.
- **The 12 KB ROM cannot hold the LiteX BIOS.** `builder.py:396-401` skips BIOS
  compilation when the ROM is already initialized, which happens automatically
  once `--firmware` supplies an image — but not for `--soft-only`, which supplies
  none. Now asserts `soc.integrated_rom_initialized = True` explicitly in split
  mode.
- **LiteX `memusage` reported "ROM usage 261%".** It assumes `.text` lives in
  `rom`; the firmware now links into `coderam`. Replaced with a check that reads
  both region sizes out of the generated `regions.ld`, so the budget cannot drift
  from the SoC definition. Same approach for the loader's ROM budget.
- **8 KB was too tight.** The loader measured 7068 bytes = 86% of 8 KB, leaving
  no room to fix a bug in an artifact that is supposed to be frozen. Raised to
  12 KB (~1 extra RAMB36) → 57%.
- **`mem.h` already defines `CODERAM_BASE`/`CODERAM_SIZE`.** Removed the
  hardcoded duplicates in `loader.c` so the two cannot drift.
- **The loader never programmed the PHY.** The B50612D's RGMII internal-delay
  shadow registers must be written or our frames are mis-clocked and never reach
  the wire, and they do **not** survive a power cycle. The loader was relying on
  `main.c` having done it — but on a cold boot `main.c` hasn't run yet. Netload
  would have received DATA frames and then hung forever, because its ACKs never
  left the FPGA. `phy_init()` now runs before netload, mirroring `main.c:709-716`.
- **Ethernet minimum-frame padding would have corrupted the image tail.** DATA
  payload length was derived from the MAC's reported frame length. LiteEth's CRC
  checker does strip the FCS (`mac/crc.py:364-382`: *"Packet data without CRC"*),
  so that part was fine — but Ethernet pads every frame to 60 bytes, so any final
  chunk shorter than 32 bytes arrives with an inflated length. The loader would
  write up to 24 bytes of padding into the image and advance the offset past the
  end. DATA now carries its payload length **explicitly** in `arg1`, with the
  frame length kept only as an upper bound. (`firmware.bin`'s 32136 bytes gives a
  392-byte final chunk, so this would not have bitten today — it was waiting for
  a different image size.)
- **ACK frames leaked stale bytes.** `eth_send` pads to 60 bytes; the loader now
  zeroes the frame first rather than transmitting whatever the previous frame left
  in the TX slot.

### Verified

- Loader builds and fits: 7068 / 12288 bytes (57%).
- Firmware links into `coderam` at the right addresses: `_ftext = 0x20000000`,
  `_fdata_rom = 0x20007d64` (in coderam, so `crt0` copies `.data` initialisers
  out of the loaded image and not out of the loader's ROM), `_fdata = 0x10000000`,
  `_fstack = 0x10010000`.
- Firmware budget: coderam 32100 / 98304 (32%), sram 22432 / 65536 (34%),
  43104 bytes of stack.
- **The loader's hand-rolled CRC-32 matches `binascii.crc32`** on `b"a"`,
  `b"123456789"` (canonical check value `0xcbf43926`) and the real
  `firmware.bin` (`0x2144df1c`). A mismatch here would have NAKed every load.
- `tools/mkimage.py` header round-trips: magic, length and CRC all verify.
- **`tools/test_netload_protocol.py` — 17/17 pass.** The loader's receive path is
  RISC-V code talking to LiteEth CSRs and cannot be unit-tested on the host, so
  this is a faithful Python model of that state machine driven by frames built by
  the *real* sender. It covers 13 image sizes (including the 8- and 20-byte
  remainders that trigger the padding trap, and the real `firmware.bin`), plus
  dropped-chunk, duplicated-chunk and combined perturbations to exercise the
  ACK-based resync.

  The test was **mutation-checked**: reintroducing the frame-length-derived
  payload length makes it fail immediately with "transfer failed to converge" —
  which is exactly the hardware symptom (a hang, with no corruption to hint at
  the cause). A test that cannot fail is worthless; this one can.

### Seed selection — and the `eth_rx` failure from Phase 0, resolved

Swept with the frozen loader (`md5 ea4daf5e977dc14548a0a497eb4471bb`), 3 builds
in parallel on 4 cores. Chosen on **worst-case margin across all four clocks**,
not on any single headline figure:

| Seed | sys (≥55) | usb (≥60) | eth_tx (≥125) | eth_rx (≥125) | worst | verdict |
|---|---|---|---|---|---|---|
| 1 | 61.36 | 91.68 | 135.32 | 125.20 | +0.2% | thin on eth_rx |
| 2 | 53.40 | 93.01 | 153.89 | 141.96 | — | **reject**, sys < 55 |
| 3 | — | — | — | — | — | **HeAP stall** |
| 4 | 57.66 | 96.04 | 167.39 | 128.02 | +2.4% | runner-up |
| **5** | **60.90** | **82.45** | **135.32** | **172.18** | **+8.3%** | **PINNED** |

Seed 5 is now the `--seed` default in `avb_soc.py`. This closes the Phase 0
`eth_rx_clk` failure (117.90 MHz against a 125 MHz requirement): 172.18 MHz here,
+38%.

Because Phase 0.5 took firmware out of the bitstream, this pin is **stable** — it
needs re-sweeping only when the gateware or the loader changes, not on every
firmware edit. Global Fmax still does not predict USB quality, so it must be
confirmed on hardware.

**Seed 3 HeAP-stalls**, and this cost 14 minutes of wall clock before being
noticed: nextpnr printed `Running main analytical placer`, never printed
`HeAP Placer Time`, and sat at 95% CPU producing no output. `seed_sweep.sh` now
carries a watchdog — if a build's log stops growing for `STALL_SECS` (default
240) while HeAP has not completed, it kills that build, records `HEAP-STALL` and
moves on. The failure mode is a property of the seed, not the machine.

### Hardware verification — and three more bugs only silicon could find

The host-side suite passed 17/17 and the first hardware run looked like a clean
pass. It was not: netload succeeded **for the wrong reason**, and that masked
three separate defects. The real verification is a *pair* of tests —

- **A:** with no host present, the window must expire and the loader must fall
  through to flash.
- **B:** with a host present, netload must still land.

Passing B alone proves nothing, because a loader that never stops listening
passes B every time.

| # | Bug | Why the host suite could not catch it |
|---|---|---|
| 1 | `tsu_addend` reset to 0, so the TSU never ticked before firmware ran | gateware reset value, no host equivalent |
| 2 | `tsu_nanoseconds_read()` returns a **latched snapshot**, not a live counter — the latch fires on reading `_seconds_hi` | CSR semantics; `gptp_read_time()` gets this right, the loader did not |
| 3 | Even latched, `tsu_nanoseconds` is a PTP nanoseconds field wrapping at **1e9**, not 2^32, so `(int32_t)(now - t0)` is invalid | arithmetic that is only wrong against real hardware semantics |
| 4 | The window opened **before the link was up** | timing, invisible off-board |

Bugs 1–3 all presented as "netload doesn't work". Fixing 2 turned *never
expires* into *expires at a random point*, which looked like a different failure
entirely.

**Bug 4 is the instructive one.** Measured on hardware: **3.85 s** from
end-of-reconfiguration to the loader being able to answer, almost all of it
gigabit auto-negotiation after the PHY soft reset. The window was ~0.3 s, so it
listened into a dead link.

I first tested the "PHY reset drops the link" hypothesis by watching
`/sys/class/net/ens5/carrier` and saw it stay up throughout — and wrongly
discarded the idea. There is a switch on that segment (an external grandmaster
is on it), so the host-side carrier never drops while the *FPGA's* PHY
renegotiates. The test was measuring the wrong end of the link. Measuring the
actual latency settled in one shot what three rounds of reasoning had not.

The fix waits for link rather than guessing a larger constant:
`phy_wait_link()` polls BMSR (twice — link status is latching-low) with a 10 s
cap, and the netload window starts after it. That adapts to however long
negotiation takes on any given switch; a hardcoded "10 s" would have worked on
this bench and been fragile elsewhere.

**Confirmed working on silicon** (`bitstreams/phase0.5_linkwait.bit`):

- Test A: no host → window expires, loader falls through. ✅
- Test B: host present → **32,136 bytes in 0.02 s**, CRC verified on-device,
  jump taken. ✅
- Firmware genuinely runs from `coderam`: the board answers ARP
  (`169.254.9.200 → 02:00:00:00:00:42`, `REACHABLE`). That responder is in
  `osc.c`, i.e. application firmware delivered at runtime; the loader has no
  ARP. ICMP going unanswered is correct — `osc.c` has no ICMP until Phase 2. ✅

### Still open

- **The firmware transmits no gPTP.** Zero unsolicited frames in 15 s while ARP
  works, so the main loop and firmware TX are fine. Not caused by bugs 1–3:
  `gptp_read_time()` does the latch read correctly. Leading hypothesis is that
  `gptp_uptime_ms()` is stuck in `[0,999]` because the TSU *seconds* field is not
  incrementing, which would make the 1000 ms Pdelay interval unreachable.
  Needs the UART (`s` command) to confirm. Does not block Phase 0.5; does matter
  for Phase 5, whose talker gate is conditioned on `gptp.servo_locked`.
- **USB is untestable from this host** — `lsusb` shows only the root hubs and the
  CH347, so the FPGA's USB port is not cabled here.
### Final seed pin

Swept against the frozen link-wait loader (`loader.bin` 8268 B):

| Seed | sys | usb | eth_tx | eth_rx | worst | |
|---|---|---|---|---|---|---|
| **5** | **59.21** | **89.86** | **143.10** | **163.48** | **+14.5%** | **PINNED** |
| 7 | 64.75 | 80.13 | 140.86 | 134.57 | +7.7% | |
| 6 | 56.93 | 82.64 | 151.81 | 129.89 | +3.9% | |
| 4 | 66.12 | 89.07 | 146.22 | **122.06** | −2.4% | best sys, **fails eth_rx** |
| 8 | 61.39 | 98.94 | 149.70 | **109.90** | −12.1% | 2nd-best sys, **fails eth_rx** |
| 2, 3 | 48.17, 50.24 | | | | | below the sys floor |

Seeds 4 and 8 are the argument for ranking on worst-case margin rather than any
single clock: both have excellent `sys` and would fail gigabit RX on the wire.

Re-verified on hardware with the pinned build
(`bitstreams/phase0.5_FINAL_seed5.bit`): Test A expires, Test B transfers
32,136 B in 0.02 s and the firmware boots and answers ARP.

---

## Side finding: plan risk #2 (mDNS Dante quirks) is smaller than feared

`inferno`'s submodules are now checked out, so `searchfire` — the
`searchlight` fork the plan flagged as un-inspectable — can be diffed. It is
only two commits of substance (`d87e842` "forked as Searchfire",
`318bfd7` "more changes"), ~170 lines total. Most of it is API surface, not
protocol quirks:

- `discovery.rs` adds a one-shot `single_query(types, fqdn)` with a 3 s
  deadline — needed because inferno resolves a channel on demand rather than
  browsing continuously. Not a Dante quirk; we need the equivalent only if we
  ever act as a Dante *receiver*.
- **`broadcast/service.rs` adds a `port == 0` service form**, and this one *is*
  a real non-standard record shape worth knowing. With `port == 0` the
  responder emits **only `A`/`AAAA` records named by the service id, plus one
  optional additional `TXT`** — no `PTR`, no `SRV`. That is exactly how
  `mdns_server.rs:226` advertises `<d>.<c>.<b>.<a>.in-addr.local` to reserve a
  multicast IP (Dante's collision-avoidance mechanism).

Implication for Phase 3: the `_netaudio-*` services are ordinary
PTR/SRV/TXT/A records, and the only unusual shape is the `in-addr.local`
A-record reservation. That is cheap to implement and cheap to omit initially.
The wire is still the authoritative reference — capture a real device — but the
risk is downgraded from "unknown unknowns in a forked mDNS stack" to one
documented record form.
