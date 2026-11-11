# dante-usb — read this before building or debugging

FPGA Dante **transmitter**: 48ch USB audio in → Dante out. Colorlight i9plus /
XC7A50T, openXC7 (yosys + nextpnr-xilinx), LiteX SoC + VexRiscv.

This file exists because the findings below were each already written down
somewhere in the repo and were each re-derived from scratch anyway, at a cost of
hours. They are the ones that are **expensive to rediscover**, not a summary of
the project. Long-form reasoning stays where it is; the pointers are at the
bottom.

---

## Building

```sh
./build.sh                      # builds the validated dual-USB 96 kHz design
tools/seed_sweep.sh 1 8         # seeds 1..8, 3 at a time
```

**The recorded recipe is SIX items plus the watchdog, and `build.sh` now
defaults to all of them.** Do not reassemble it from memory — it was got wrong
twice in one afternoon. It is written out in full in `README.md` under *"Fixed
(2026-11-04): Main's ULPI input eye sat beyond the delay line"*:

```
BOARD=breakout USB2=1 USB_DUAL_RATE=1 NEXTPNR_USB_FP_LIGHT=1 \
USB_PERIOD_NS=14 USB_PHASE_DEG=333 STALL_SECS=900 \
./build.sh --seed 1 --output-dir build_phase333_s1
```

- `USB_PERIOD_NS=14` over-constrains `cd_usb` to 71.43 MHz against a real
  60 so the placer grinds it harder. **The `FAIL at 71.43 MHz` lines are
  expected — score the MHz, not PASS/FAIL.** Do *not* do this to
  `SYS_PERIOD_NS`: 20.0 is the requirement and 16.0 / 14.0 hang HeAP.
- `USB_PHASE_DEG=333` (−27°) applies to **`cd_usb` only**. `cd_usb2` stays at
  phase 0; Backup was already well placed and measures −0.0002% there.
- `STALL_SECS=900`, not the 240/300 defaults: HeAP takes 49–71 s and the log
  does not grow while it solves, so a short quiet window kills good seeds.
- Read the **last** timing report per clock. nextpnr prints one after placement
  and another after routing and they differ by more than 10 MHz.

The design reads **23** environment variables (`avb_soc.py`, `floorplan_usb.py`).
Every build writes all 23 to `$OUT_DIR/build_config.txt`, derived from the
sources, so a build directory can always be asked what it is. Ask it rather than
assuming.

On 2026-11-07 two consecutive 8-seed sweeps were wasted on this, three hours in
total: the first built the **single-USB 48 kHz chip** (`USB2`/`USB_DUAL_RATE`
unset — every log healthy, zero `ulpi2` lines in the XDC), the second hung all
eight seeds (`NEXTPNR_USB_FP_LIGHT` unset). Both times the configuration, not
the design, was the bug.

## The ULPI eye: phase 333, tap 31

Both are settled and neither is a tuning knob any more.

`firmware/main.c:1084` and `:1118` pin **IDELAY tap 31** (the hardware maximum)
on Main and Backup at boot. The tap is only live after the VAR_LOAD load
register is pulsed — a CSR read-back of the tap is not the hardware.

**Measured 2026-11-04, 48 ch @ 96 kHz, with random data** (never `/dev/zero`:
the same bitstream and tap read 0.0000% with zeros and +1.28% with random):

| tap | phase 0 | phase 333 |
|---|---|---|
| 31 (hardware max) | +1.2841% | **+0.0002%** |
| 30 / 29 / 28 | +5.00% / +14.21% / +29.95% | — |
| 24 / 20 / 12 / 8 / 0 | — | **0.0000%** |

At phase 0 no tap could fix it: 31 taps × ~78 ps ≈ 2.4 ns and the eye lay past
the end of the delay line. Moving the *sampling edge* fixed it, and at phase 333
the eye is wider than the whole delay line. So if ULPI sampling regresses, the
cause is placement or a re-emitted descriptor ROM, **not** the tap.

Re-emitting the USB descriptors moves both eye centres: regenerate → rebuild →
`tools/tapsweep.py --phy main` and `--phy backup` → pin the winners → rebuild.

## When the placer hangs

`Running main analytical placer` with no `HeAP Placer Time` afterwards means
**there is no legal placement**. HeAP never reports infeasibility — it spins.
Do not treat it as a slow build, a seed, a memory problem or a machine problem.

**First thing to read, before any hypothesis** — the floorplan's own cell count,
printed early in the nextpnr log:

```sh
grep -a "floorplan_usb] USB:" <log>
```

| cells in `usb_fp` | meaning |
|---|---|
| ~700–1450 | the light box — **this places** (HeAP ~49 s) |
| ~3000+ | the full box — **infeasible by construction**; 11 seeds stalled, 8 more later |

Measured 2026-11-07, two logs that differed in nothing else: 1429 cells placed,
3109 cells hung eight times out of eight. `PLACER=sa` is the escape hatch that
terminates and names the real error (`failed to place chain … PACKED_CARRY4`).

Second thing: a timing constraint far outside what the design can reach also
hangs HeAP — it removes the gradient the analytic solve needs. `_P_SYS` is
20.0 ns (the real requirement); 16.0 and 14.0 stall. See `avb_soc.py` near
`_P_SYS`.

`NOBUILD` in the sweep results means the build produced no bitstream;
`HEAP-STALL` means the configuration is unplaceable. They are different facts
and the second one is the useful one.

**Third thing, and it is free: count the HeAP iterations.** A stall is not one
thing. `grep -c "type ALL: wirelen solved" <log>`:

| iterations before the watchdog | meaning |
|---|---|
| 0 | HeAP never completed a solve — **no legal placement** |
| 1–4 then silence | it started and got stuck — same conclusion in practice |
| ~10 within 4 s, then `HeAP Placer Time` | **this is what success looks like** (38–59 s total) |

Measured 2026-11-11 on the 0-of-8 KVM sweep: seeds 1 and 6 had **zero**, seeds
2/4/5/7/8 had 1–4, seed 3 reached #10 at t=3.3 s and finished at 48 s. Reading
this would have cost one grep and saved most of a day of floorplan knobs.

**A P&R-only probe is five times cheaper than a build and it is a valid
predictor.** `ident_version=False` makes the netlist reproducible — verified
2026-11-11 by diffing the eight sweep `.json` files byte for byte: they differ
in 15,388 bytes and **every one is inside a `"src"` path string**
(`build_seed1` vs `build_seed3`, equal length, hence equal file size).
So run nextpnr directly on one fixed `.json` with `--no-route` and vary only the
seed or the floorplan env: a placeable config answers in ~2 minutes (json load
~60 s + HeAP), anything past ~4 minutes is a stall. Harness:
`$SCRATCH/pnr/probe.sh <tag> <seed> [--route]`, `PLACER=sa` honoured.
It reproduced the real build (seed 3: 1502 cells, HeAP 59 s probe vs 48 s build).
**Pass `NEXTPNR_USB_FP_LIGHT=1` yourself** — nextpnr does not get it from
`build.sh`, and without it the floorplan silently builds the FULL 3326-cell box.
That mistake cost three probes on 2026-11-11, the same configuration trap as the
two wasted sweeps above.

## Widening the USB box does not work, and the box is not the KVM problem

Two separate results from 2026-11-11, both measured, so neither needs repeating.

**X cannot grow.** `NEXTPNR_USB_REGION=0,0,55,156` with `NEXTPNR_CFG_X0=58`
(cfgflash moved out of the way — that is what `NEXTPNR_CFG_X0` was added for)
gave **zero** HeAP iterations on seeds 1 and 5. That matches attempt 7 in
`floorplan_usb.py`'s own log (`0,0..58,156`, no overlap, still stalls). Y is
already the full column. So the box's AREA is not a lever in either direction.

**And the cell count is not the cause.** `NEXTPNR_USB_FP_CTRL_NOT=rom_read_port`
drops the descriptor-ROM read port out of the box (the measured critical path is
`get_descriptor`'s byte counter and its CARRY4, not the ROM data), which took
the box from 1502 to **1429** — the exact size this file records as placing on
2026-11-07. It stalled on seeds 1, 5 and 7.

**What the numbers actually say**, all at `--seed 5` or better:

| netlist | total cells | LUTs | FFs | PADs | box | result |
|---|---|---|---|---|---|---|
| 7 Nov known-good | 17,234 | 16,787 | 9,549 | 49 | 1385 | **places, HeAP 38.17 s** |
| today without HID | 17,764 | 17,239 | 9,931 | 51 | 1390 | stalls |
| today with HID | 18,255 | — | — | — | 1463 | stalls (1 of 8 seeds) |

The 7 November netlist **still places on this machine today**, so the toolchain,
the floorplan and the box are all fine and the delta is entirely ours.

**Retraction.** `KVM_HANDOFF.md` claimed "roughly half the growth predates HID
and is accumulated upstream drift". It does not: regenerating from the current
source **without** HID gives 1390 cells against the old 1385, so HID accounts
for essentially all of the box growth (+73) — and HID is still not what breaks
placement, because the no-HID netlist stalls too. The two extra PADs in that row
are `kvm_serial`: the second UART was in that build.

**Bisect switches**, so the three additions can be built apart (all default 1,
so the shipping build is unchanged):

| env | what it removes |
|---|---|
| `KVM_HID=0` | the HID interface **and** its 72-bit endpoint. It used to drop only the descriptor bytes while `usb_avb_subsystem.py` instantiated the endpoint unconditionally, which made the A/B meaningless |
| `KVM_HID_CSR=0` | the two `CSRStorage(72)` report registers and the `i_hid_report` connections |
| `KVM_UART=0` | the second UART (`RS232PHY` + 64-deep RX FIFO + CSR bank 14) |
| `KVM_UART_PINS="<tx>,<rx>"` | moves `kvm_serial` (default `AB22,W20`) |
| `KVM_UART_FIFO="<tx>,<rx>"` | the UART FIFO depths (default `16,64`) |

### The bisect, and what it found

All four at `--seed 5`, same `rtl/*.v` (md5 checked identical), only the env
differing:

| build | total cells | box | result |
|---|---|---|---|
| no UART, no report CSRs | 17,230 | 1399 | **places, HeAP 30.5 s** |
| report CSRs only | 17,568 | 1431 | **places, HeAP 58.4 s** |
| UART only | 17,493 | 1429 | **stalls, 0 iterations** |
| UART only, pads moved to F13/F14 | 1429 | | **stalls, 0 iterations** |

So: the regenerated `.v` is innocent (`requesthandlers.py` and the generator fix
place fine), the two 72-bit report CSRs are innocent — and note the CSR build has
MORE cells than the UART build and places, so **it is not the cell count** — and
the second UART is what breaks it.

**It is NOT the pin position, though that was the obvious answer and it was
wrong.** `AB22`/`W20` are `IOB_X0Y29`/`IOB_X0Y25`, which in nextpnr's grid is
Y25-29, straight through the ULPI pad band (Main X1 Y23-49, Backup X1 Y1-32) and
inside the USB box — and an IOB is already located, so `floorplan_usb.py` cannot
constrain it away. Moving the pads to `F13`/`F14` (`IOB_X0Y148`/`Y147`, bank 16,
far above both the ULPI band and the eth pads at X1 Y51-72) stalls **identically**.
The ethernet pads sit inside the same box and are fine, so a pinned IOB in the
box is not the mechanism.

> **nextpnr's grid XY for an IOB is the prjxray IOB SITE XY, not the tile XY.**
> `T4` (Main's ULPI clock) is `IOB_X1Y23` in `package_pins.csv` and the floorplan
> measured Main's pads at X1 Y23-49 from a routed design. The tile column
> (`LIOB33_X0` vs `RIOB33_X43`) is a different coordinate system — do not mix
> them. Look pins up with:
> `grep -E "^(PIN|...)," <prjxray-db>/artix7/xc7a50tfgg484-1/package_pins.csv`

**But the UART does not make it infeasible, only tight.** A P&R-only seed sweep
on the UART-only netlist: seeds 1, 2, 3, 5 stall with zero iterations, **seed 7
places** (HeAP 83.6 s). So this is the seed lottery at a worse ratio, not a wall.
Remaining suspect for the tightness, not yet measured: LiteX's `RS232ClkPhaseAccum`
is a **33-bit adder per direction** (nine CARRY4 each) where a divide-by-50
counter would do at 50 MHz / 1 Mbaud; the UART build carries +47 CARRY4 over the
CSR build. `KVM_UART_FIFO=1,1` is in place to split the FIFOs from the PHY.

**And the seed does not transfer between netlists.** Seed 3 placed the HID
netlist with the 16-bit comparator and stalled on the 3-bit one; seed 7 places
the UART-only netlist. After any gateware change, sweep — but sweep with
P&R-only probes on one synthesised netlist, not with eight full builds.

### Result: the KVM build closes on seed 4 (2026-11-11)

The second UART was **deleted**, not fixed. The operator had already decided
this question -- the one UART was kept, when the rest of the console was
stripped, *for* this link -- and it is already 1 Mbaud with a 64-deep FIFO, so
the second one was a duplicate. The whole UART now sits on the SODIMM at the
RP2350: **`AB22` = SODIMM 159 = FPGA TX**, **`W20` = SODIMM 161 = FPGA RX**,
bank 14 (already LVCMOS33 in the shipping build, so no level shifter), with
`KVM_UART_PINS="R3,M3"` to put the console back on the CH347. Nothing read the
console RX -- `loader.c` has one `uart_rxtx_write()` and zero reads, `log.h`
only writes -- so firmware reads reports with `uart_read_nonblock()` /
`uart_read()`, **never the FIFO**: `uart_init()` attaches `uart_isr()`, which
drains the hardware FIFO into a 128-byte software ring, and polling the FIFO
would race it.

Final sweep on that netlist (HID + both report CSRs + the one UART), box 1460:
**seeds 4 and 8 place, the other six stall.** Routed:

| clock | required | seed 4 | seed 8 |
|---|---|---|---|
| sys | 50 | **54.79** | 53.16 |
| usb (Main) | 60 | **71.68** | 59.78 ✗ |
| usb2 (Backup) | 60 | **67.65** | 64.60 |
| eth_rx | 125 | **156.30** | 124.84 ✗ |
| eth_tx | 125 | **159.01** | 129.65 |

**Seed 4 passes all five with margin**, and its 71.68 MHz on Main is the widest
USB margin this project has measured (previous best 69.98). Note seed 4 was the
SLOWER placement of the two -- HeAP 119.75 s against seed 8's 69.51 s -- so
placement time predicts timing no better than `sys` Fmax does.

**The box cell count is not a lever, measured twice more.** 1429 cells stalled
on three seeds and 1393 on two, while 1399 placed in 30 s on the netlist without
HID. Same rectangle, same occupancy, different outcome: it is the structure of
the netlist, not how many cells fit. `NEXTPNR_USB_FP_CTRL_NOT=rom_read_port`
does shrink the box exactly as predicted (1460 -> 1393) and does **not** rescue
a stalling seed. Stop pulling that knob.

### Hardware result, seed 4 bitstream (2026-11-11)

Flashed volatile, taps left at 31/31. **Both HID devices enumerate on both
PHYs** and the kernel binds `usbhid` and builds four input devices:

```
1-10 1209:eab1 Main    if00/if01 class=01 snd-usb-audio   if02 class=03 usbhid
1-8  1209:eab2 Backup  if00/if01 class=01 snd-usb-audio   if02 class=03 usbhid
Main Keyboard/Mouse, Backup Keyboard/Mouse -- report descriptor 107 B, identical
```

Audio untouched: 48 ch / S32_LE / 96000 on both cards. **Four OSC switches left
the USB device numbers unchanged (115/116)** with all four input devices still
present, and the `'W'` opcode counted them (`switches 4`, `dest` following) --
so switching costs no disconnect, which was the requirement.

**Tap sweep, leak criterion, RANDOM data, 10 s per tap** — and it says nothing
needs re-pinning:

| tap | 0 | 8 | 16 | 24 | 28 | 31 |
|---|---|---|---|---|---|---|
| Main | +0.0000% | −0.0004% | −0.0000% | +0.0004% | −0.0004% | +0.0000% |
| Backup | +0.0000% | −0.0004% | −0.0004% | −0.0004% | +0.0005% | +0.0000% |

Every tap at the measurement noise floor on both buses, `overflow` 0, ingress
exactly nominal (18.432 MB/s). The eye is again wider than the whole delay
line, so **31/31 is an arbitrary point inside an open eye, not a cliff edge** —
31 taps of margin against the ~8 the acceptance bar asks for. Dual stream, both
PHYs at 48 ch / 96 kHz with random data, three 15 s windows: both exactly
18.432 MB/s, leak −0.0003%..+0.0000%, `overflow` 0, `write_drops` 0.

**`tools/stats.py` falls over on one lost datagram**; `usbsrc.query()` retries
four times. Measured here: every opcode answers ~6/6 under a live stream, with
ordinary link-local loss around 1 in 12 hitting all of them equally. So
"stats.py times out while the other tools work" is the tool's retry policy, not
the board and not the reply size (`'f'` is 316 bytes and fine).

## Generated files are what the tools consumed

- **`csr.h` must belong to the `.bit` you are flashing.** Optional features are
  guarded by `#ifdef` on generated CSR names, and a missing name takes the
  `#else` **with no warning**. A board once ran 18 hours reporting all 48
  channels silent while music played, because the firmware was built against a
  three-day-old header. `flash.sh` now refuses on a mismatch; a feature report
  describes the *header*, not the *binary*, so only a forced clean makes them
  agree (the `.build_dir` stamp in `firmware/Makefile`).
- **The seed lottery is a netlist lottery.** Diff the generated `.init` files
  before blaming the placer. The loader binary is baked into the BRAM ROM, so
  touching `firmware/loader/` re-rolls placement.
- **Never pick a seed on `sys` Fmax.** Measured twice: the highest clean sys
  Fmax gave the worst USB datapath once (13% ingress leak) and a dead one the
  other time (no outputs at all). Prefer the higher `USB_MHz` on a tie, then
  confirm on hardware: leak < 1%, `ep_out` within ~0.1% of
  channels×rate×4 B/s.
- `set_clock_groups`, `set_false_path` and `set_multicycle_path` are **ignored**
  by nextpnr-xilinx. Every cross-domain path is permanently timed. Any new CDC
  needs a real synchroniser *and* must close timing.
- **LiteX allocates CSR banks by sorting bank NAMES**, not declaration order.
  Pin new ones with `self.add_csr("<name>", csr_id=N)` — adding `icap` unpinned
  moved 77 registers.

## Hardware rules

- **Never flash while audio is streaming.** It can take out the xHCI host
  controller, including the CH347.
- `./flash.sh` is **volatile by default** (`pld load`) on purpose, so a power
  cycle rolls back to the image in SPI flash. `--permanent` writes SPI.
- **ICAPE2 self-reset CANNOT WORK on the openXC7 flow. Do not spend another
  build on it.** Measured 2026-11-07, end to end: Controller's Reboot (`0x90`)
  reached the firmware, the SPI image passed its CRC, IPROG went to the CMD
  register, and the FPGA did not reconfigure -- USB stayed enumerated on the
  same device numbers, so the fabric never reset. `tools/icap.py` then read
  every ICAP register as `0x00000000`, IDCODE and STAT included, **with
  `icap_done` arriving every time** -- so the divided clock runs and LiteX's FSM
  completes; only ICAPE2's `O` bus gives nothing back. The reason is in
  prjxray: the whole `CFG_CENTER_MID` tile has **nine** documented features
  (a CLB has 680), and while `DCIRESET.ENABLED` and
  `STARTUP.USRCCLKO_CONNECTED` are among them, there is **no activation bit for
  ICAP at all** -- only `ICAP_WIDTH_X8`/`X16`. The site is routed (the FASM
  wires all 32 I and O pins, CLK, CSIB, RDWRB) but never switched on, and no
  openXC7 bitstream can switch it on. This is an inference from the sibling
  sites in the same tile, not a proof, but it fits every measurement.
  `STARTUP.USRCCLKO_CONNECTED` being present is why the S7SPIFlash core *does*
  work here.
- **`ctrl_reset` does not produce a bootable board on this design** — the
  firmware lives in coderam, not flash, so a reset lands in the loader and the
  board does not come back, not even with netload. Measured; recorded in
  `firmware/dstats.c`. **ICAPE2 + IPROG is the only self-reset that works** on a
  7-series part. Verify the SPI image's CRC before triggering it: a
  magic+length-only check passed once and bricked the board.
- **After a `--permanent` flash plus power cycle the board looks dead, and it
  is the host's ARP cache.** `jtagspi_program` loads a proxy bitstream, so the
  design stops; on the way back the cached entry for the board goes stale while
  still reading `REACHABLE`, and ping returns 100% loss with `pipe 2`. Loader
  and application firmware share one MAC (`02:00:00:00:00:42`), so the entry
  looks right. Clear it before concluding anything:
  `sudo ip neigh del 169.254.9.200 dev ens5`. On 2026-11-07 this produced a
  confident "the board is sitting in the loader" that was simply wrong --
  USB had re-enumerated and the firmware was running the whole time.
- The console is `/dev/ttyACM0` (CH347, `1a86:55dd`) and **this account is not
  in `dialout`**, so the loader's own diagnostics cannot be read without
  `sudo`. That, not the wiring, is why a `'r'` over the UART never arrived.
- Board default address `169.254.9.200` (`DANTE_BOARD` overrides it;
  `tools/dante_board.py` is the single source). The host side is
  `avahi-autoipd` on 169.254.9.111 — **leave avahi running.**
- Kill build processes **by PID**, never with an unscoped `pkill -f`.

## Dante protocol

- **Capabilities bit 14 (`0x4000`)** of the u32 at `content[0x14]` of info
  message `0x60` is what ungreys Dante Controller's network config
  (`IVersions.hasStaticIp()`). One byte: `c[0x16] |= 0x40`. Everything tried in
  mDNS was a wrong turn — no proxy service is needed, and `arcp_vers=2.8.15` is
  a measured regression.
- Controller's **Reboot button is info opcode `0x90`**, sent once, no retry.
  **It does not reboot anything, by design: it applies the stored address
  live.** A reconfiguration would drop the USB device mid-transfer, and 48
  channels of USB audio are the product -- so the Reboot button re-points the
  network stack instead and leaves USB alone. This costs nothing because
  nothing caches our IP: `mdns.c`, `dante_cmc.c`, `dante_tx.c`,
  `dante_info.c` and `net.c` all read `g_net_ip` as they build each packet. The
  only cached copy is the AAF packetizer's CSR (it emits audio with no CPU in
  the loop), re-latched by `dante_tx_set_src_ip()`. The mDNS A record carries
  the cache-flush class, so one `mdns_announce()` replaces the old address
  everywhere on the link. Applied from `dante_info_poll()`, never in the packet
  path.

  Measured 2026-11-07, .200 -> .201 -> .200 with both PHYs streaming 48ch at
  96 kHz throughout: USB device numbers **unchanged** (110/111, no
  re-enumeration), both streams still `Running`, `packets` continued through
  (1,042,129 -> 2,355,767) instead of resetting, and `underrun` **stayed at 126
  across both changes** -- not one new underrun. mDNS followed to the new
  address. `flows_active` dips 2 -> 1 for a few seconds because a receiver's
  flow was bound to the old source IP, then returns to 2 on its own with
  `flows_rejected 0`.
- Static addressing is `0x13` write → stored config → reboot records in `0x11`
  → applied by `net_init()` at the next boot. Proven .200 → .201 → .200.
- **Primary interface only, by decision.** A Dante chip has one MAC; the
  secondary port is "Switch only". See the README section for the evidence.
- The capture ring (`firmware/cap.c`) excluded all multicast until 2026-11-07,
  so older "saw nothing" results cannot be distinguished from "could not see".

## Where things actually live

- **The USB core's source is NOT in this repo.** It is a sibling project:
  `/home/lisp/FPGA/avb-usb-host/gateware/` (LUNA + amaranth 0.4.5, own venv at
  `/home/lisp/FPGA/avb-usb-host/.venv/bin/python`). The two 22k-line `.v` files
  here are machine output from `tools/gen_usb_subsystem.py`, run by hand, not
  wired into `build.sh`. Re-emitting the descriptors **moves both ULPI eye
  centres**: regenerate → rebuild → `tools/tapsweep.py --phy main` and
  `--phy backup` → pin the winners → rebuild.
- There is **one** UART, on R3/M3 via the CH347 ext board (`avb_soc.py:549`),
  *not* the unused `pmod_uart()` helper. The platform's pin list has a missing
  space between two string literals, so every `dimm:N` above 20 is one position
  off from the comments.
- Long-form reasoning, with the measurements: `DUAL_USB_96K_HANDOFF.md`,
  `STATIC_ADDRESSING_HANDOFF.md`, `README.md` ("Open bugs" included), and the
  comment blocks in `floorplan_usb.py` and `avb_soc.py`. **Read the relevant one
  before forming a hypothesis** — today's three hours were spent re-deriving
  what two of them already said.

## Conventions

- Commit and push only when asked.
- Commit messages end with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`;
  PR bodies end with `🤖 Generated with [Claude Code](https://claude.com/claude-code)`.
