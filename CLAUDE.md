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
