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
