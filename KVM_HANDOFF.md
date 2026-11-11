# KVM / HID per PHY — handoff, 2026-11-07

Branch `kvm`, worktree `/home/lisp/DANTE/FPGA/dante-usb-kvm`, merged up to
`47733e8` (so it has today's `CLAUDE.md`, build profile and live-address work).

Goal: a HID device on **both** USB PHYs, always enumerated, with the report
*routing* switched by OSC (`kvm_main` / `kvm_backup`), fed from an RP2350 over a
serial link. **Stopped mid-way at the user's request. Nothing is committed, in
either repo.**

---

## State: the gateware is written and verified as far as it can be without a
## bitstream. It does not place yet.

### Done, and checked rather than assumed

**In `/home/lisp/FPGA/avb-usb-host/` (the amaranth source — a *different* repo,
own venv, now has its own `CLAUDE.md`):**

- `requesthandlers.py` — the class-request claim is narrowed by interface
  number. It used to claim **all** class requests and stall what it did not
  recognise, so HID requests never reached a HID handler. The request code
  cannot discriminate (HID `GET_REPORT` is `0x01`, so is Audio `CUR`);
  `setup.index` can, which is what the HID spec uses.
- `requesthandlers.py` — `SET_INTERFACE` no longer zeroes **both** altsetting
  registers before its `Switch`. It did, so a `SET_INTERFACE` to any interface
  other than 1 or 2 cleared both and restored neither.
  **Correction to an earlier claim: this was never an audio-killer.** Nothing in
  `usb_avb_subsystem.py` reads those registers — the only reader in the tree is
  `usb_utmi_top.py:309`, a bench top. Latent, not live.
- `usb_descriptors.py` — HID interface, EP `0x83` interrupt IN (1 ms), HID class
  descriptor `0x21` and a 107-byte report descriptor with keyboard (report ID 1)
  and mouse (ID 2). Both reports are **9 bytes** including the ID; the mouse is
  padded to match, because the endpoint relays a fixed width.
  `n_audio_interfaces()` is the one place the interface count is decided — it
  follows `PLAYBACK_ONLY`, so HID is IF**2** in the shipping build, not IF3.
- `usb_avb_subsystem.py` — `USBSignalInEndpoint(width=72, endpoint_number=3,
  signal_domain="sys")`, and `hid_report` added to `ports()`.

**In this repo:**

- `avb_soc.py` — `kvm_main_report` and `kvm_backup_report`, `CSRStorage(72)`
  each, wired to `i_hid_report` on both subsystem instances. **No CSR bank
  moved** (bank bases diffed identical against `build_seed5`).
- `rtl/usb_avb_subsystem{,2}.v` regenerated with `--dual-rate`. The two differ
  on 87 lines, all in the descriptor ROM (PID, product string, serial) — no
  drift.

### Verified by decoding, not by "it built"

```
INTERFACE num=2 alt=0 neps=1 class=0x03 sub=0x00 proto=0x00 iIface=4
HID bcdHID=0x0111 nDesc=1 type=0x22 len=107
ENDPOINT addr=0x83 attr=0x03 mps=16 interval=4
```

### Two bugs found on the way that were not the task

- **The generator was broken at HEAD.** `usb_avb_subsystem.py` read
  `self.current_rate_48k`, which only the request handler has, on an
  unconditional line in `elaborate()`. `gen_usb_subsystem.py` therefore failed
  with `AttributeError` **both with and without `--dual-rate`**. Fixed.
  **Consequence worth keeping in mind: the committed `.v` pair could not have
  come from the committed source.** Any "before" measurement taken from those
  files is a stale baseline.
- **Declaring a Signal is not exporting it.** `hid_report` was missing from
  `ports()`, so Amaranth emitted
  `wire [71:0] hid_report; assign hid_report = 72'h0;` — a port that does not
  exist, with no error anywhere. Found by reading the generated Verilog.

---

## The open problem: it does not place. 0 of 4 seeds.

| seed | result |
|---|---|
| 5 (single build) | HeAP stall, one iteration then 912 s silent |
| 1, 2, 3 | HeAP stall, killed at 900 s |
| 4–8 | not finished — the sweep was stopped |

`[floorplan_usb] USB: 1449 cells` (against 1385 for the old `.v`), which is at
the very top of the band `CLAUDE.md` records as placeable.

**Do not conclude from this that HID broke placement** -- and the reason is
NOT the one written here first. The original text claimed "roughly half the
growth predates HID and is accumulated upstream drift that was never
regenerated", reasoning from the descriptor ROM going 67 -> 139 entries against
HID's own 156 bytes.

> **MEASURED AND RETRACTED 2026-11-11.** Regenerating from the current source
> with `KVM_HID=0` gives **1390** cells against the old **1385**. There is no
> drift worth the name: HID accounts for essentially the whole +73. The real
> finding is stronger, though: **the no-HID netlist does not place either**
> (zero HeAP iterations, seed 1), while the 7 November netlist still places on
> this machine today (1385 cells, HeAP 38.17 s). So HID is not the cause and
> neither is the box -- see `CLAUDE.md`, "Widening the USB box does not work".
> Note also that `KVM_HID=0` only dropped descriptor BYTES until 2026-11-11:
> `usb_avb_subsystem.py` instantiated the 72-bit endpoint unconditionally, so
> the A/B this file asks for could not actually be taken. It can now. The light floorplan box deliberately pulls in the
`get_descriptor` nets — a measured fix for a `cd_usb` critical path — so the box
grows with the ROM whatever the cause.

### Next step, and it is one experiment

Regenerate **without HID from the current source** and build that.

- It also stalls → HID is innocent; the problem is the upstream drift, and the
  question becomes whether those upstream changes are wanted at all.
- It places → HID is the delta, and the targeted options are widening the USB
  region, or shrinking the report descriptor (drop the mouse collection).

Done on 2026-11-11, and it answered: the no-HID build stalls too. The search
moved to the two additions that live OUTSIDE the USB box -- the second UART and
the two 72-bit report CSRs -- via `KVM_UART=0` and `KVM_HID_CSR=0`. See
`CLAUDE.md` for the switch table and the cell/LUT/FF comparison against the
7 November netlist.

### After a seed passes, the sequence is not optional

Re-emitting the descriptors moves **both** ULPI eye centres:
rebuild → `tools/tapsweep.py --phy main` **and** `--phy backup` → pin the
winners → rebuild. See `CLAUDE.md`, "The ULPI eye: phase 333, tap 31".

---

## Not started: the firmware side

- RP2350 link over UART; write 9-byte reports into the two CSRs.
- `kvm_main` / `kvm_backup` in `osc_rx()` (UDP 9000, flat `strcmp` chain, and
  note the five-parameter shape — a four-parameter version crashed the CPU).
- **Release reports on switch.** The routing is deliberately in firmware and not
  a gateware mux for exactly this: whatever is held down when you switch away
  stays held on the host you left unless something sends it an all-released
  keyboard report. A mux can only pass or zero; firmware can do it properly.

## The RP2350 board, read off the schematic (not the wiki)

Waveshare RP2350-USB-A, castellated, header P1 = `GPIO0..GPIO10`,
`GPIO26..GPIO29`, `3V3`, `GND`, `VSYS` (18 pads). **All 15 exposed GPIOs are
free**: the USB-A port uses GPIO12 (D+) and GPIO13 (D−) through 27 Ω, the RGB
LED uses GPIO16, and none of those reach the header. BOOT is on `QSPI_SS`,
RESET on `RUN`.

- **R13 is a 1.5 kΩ pull-up to 3V3 on a USB-A data line** — device-mode
  signalling. Remove it for a keyboard/mouse; the port is then host-only. A
  separate **Type-C** exists for programming, so that still works.
- `IOVDD` is tied to 3V3 → 3.3 V IO, direct to the FPGA's LVCMOS33 banks.
- `VSYS` feeds regulator U1 (RT9013-33GB) **and goes straight to the USB-A
  VBUS pin, with no load switch**. So feed 5 V into `VSYS` (pad 18).
- **Type-C VBUS sits on the same `VSYS` net with no visible diode.** Do not feed
  VSYS from the FPGA board and plug in the Type-C at the same time.

### Proposed wiring (all four pins verified free in `build_seed5`'s XDC)

| RP2350 | dir | SODIMM | FPGA |
|---|---|---|---|
| pad 18 `VSYS` | ← | 2 (5 V) | — |
| pad 17 `GND` | ← | 157 or 158 | — |
| pad 2 `GPIO1` (UART0 RX) | ← | 159 | `AB22` |
| pad 1 `GPIO0` (UART0 TX) | → | 161 | `W20` |

**Use the FPGA pin name, never `dimm:N`.** The `litex-boards` connector string
is missing a space between rows 11–20 and 21–30, so `"NC"` and `"ETH1_2N"` merge
into one token: **200 tokens where 201 belong**, and every `dimm:N` from 20
upward returns the pin the comments call **N+1**. Measured, not assumed.
Unknown and only answerable at the bench: whether the Ext-Board actually routes
those SODIMM positions anywhere.

---

## Uncommitted work, both repos

```
/home/lisp/DANTE/FPGA/dante-usb-kvm   avb_soc.py, rtl/usb_avb_subsystem{,2}.v,
                                      KVM_HANDOFF.md, firmware/loader/loader.bin
/home/lisp/FPGA/avb-usb-host          CLAUDE.md, gateware/requesthandlers.py,
                                      gateware/usb_avb_subsystem.py,
                                      gateware/usb_descriptors.py
```

`firmware/loader/loader.bin` was copied from the main checkout and then rebuilt
byte-identical (`b8ecbf094efff0b11de7d226edab24ad`), so the ROM is unchanged and
the only placement delta is the descriptors — which is what makes the experiment
above clean.

The sibling repo also has pre-existing uncommitted deletions under
`bitstreams/` that are **not** mine. Leave them alone.
