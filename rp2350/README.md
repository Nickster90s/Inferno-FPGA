# rp2350 — the keyboard/mouse side of the KVM link

Firmware for the **Waveshare RP2350-USB-A**. A USB keyboard and mouse plug into
its USB-A port; it sends their reports to the FPGA as the 12-byte frames
`firmware/kvm.c` parses. Which host gets them (Main / Backup) is the FPGA's
business (`/kvm/main`, `/kvm/backup` over OSC) and is invisible from here.

**Status 2026-11-11: compiles, and the two hardware-free parts pass their test
on the build machine. It has never run on the board.** Everything under
"Unverified" below is exactly that.

```
USB-A  (PIO-USB host, GPIO12/13)   keyboard, mouse, hub or combo dongle
UART0  GPIO0 TX -> SODIMM 161 -> FPGA W20    A5 5A id p0..p7 xor, 1 Mbaud 8N1
       GPIO1 RX <- SODIMM 159 <- FPGA AB22   the FPGA's log text
Type-C (native USB, device)        CDC console: status lines + the FPGA's log
```

## Build

Nothing for this was installed before 2026-11-11; it now lives in `~/pico`
(pico-sdk 2.2.0 with its TinyUSB 0.18, Pico-PIO-USB 0.7.3, xPack
arm-none-eabi-gcc 13.3.1).

```sh
export PATH=$HOME/pico/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:$PATH
export PICO_SDK_PATH=$HOME/pico/pico-sdk        # PICO_PIO_USB_PATH defaults to ~/pico/Pico-PIO-USB
cmake -S . -B build -G Ninja && ninja -C build  # -> build/kvm_rp2350.uf2

cc -Wall -Wextra -fsanitize=address,undefined -Isrc \
   test/test_host.c src/kvm_sched.c src/hid_parse.c -o /tmp/kvm_test && /tmp/kvm_test
```

Flash: hold BOOT, plug the Type-C, copy `build/kvm_rp2350.uf2` to the drive.
**Not while VSYS is fed from the FPGA board** — the Type-C's VBUS is on the same
net with no diode (`KVM_HANDOFF.md`). The same applies to reading the console.

## Why the reports are paced and not just forwarded

The FPGA's HID endpoint relays **state**: the report register goes to the host
on every 1 ms poll until something overwrites it, and one register serves both
report IDs. Forward a mouse report with `dx = 5`, leave it there for 20 ms, and
the pointer moves 100. Overwrite a key press 300 µs later with a mouse report
and the key was never pressed. So `src/kvm_sched.c` sends at most one frame per
millisecond, from core 1 so the tick is not at the mercy of the bit-banged USB
host on core 0:

| frame | stands for | then |
|---|---|---|
| keyboard report | 2 ms (held) | anything |
| mouse button change, no motion | 2 ms (held) | anything |
| mouse motion, clamped to ±127 | exactly 1 ms | **must** be replaced: a queued key, else a zero-motion mouse frame |

**Motion is therefore approximate and nothing here can make it exact.** One
poll period of exposure is one delivery only on average: the host's poll clock
is not ours, and the FPGA drains its UART ring from a main loop shared with the
PTP servo. A motion frame will now and then be seen twice or not at all. Keys
and buttons are held and do not have that problem. The exact fix is in gateware
— send each written report once and NAK until the next write — and that means a
regenerate, a seed sweep and two tap sweeps, so it was not started from here.
`KVM_HOLD_TICKS` is the knob if the FPGA's loop turns out to stall longer than
2 ms.

## What is forwarded

- **Keyboard:** boot protocol, whose 8 bytes are the payload of the FPGA's
  report ID 1. Key codes above `0x65` (outside the FPGA's descriptor) are
  zeroed. Media keys have no report to go in.
- **Mouse:** report protocol, layout read from the device's own report
  descriptor (`src/hid_parse.c`), so the wheel works and 12/16-bit axes are
  handled. Three buttons. Falls back to boot protocol (no wheel) if the
  descriptor cannot be parsed or the mouse refuses.
- **Unplugging** a keyboard or mouse sends a release, for the same reason
  `kvm.c` sends one on a switch.

## Unverified — check these at the bench, in this order

1. **R13** (1.5 kΩ pull-up on a USB-A data line) — `KVM_HANDOFF.md` says remove
   it for host use. Not confirmed on a board.
2. Whether the Ext-Board routes SODIMM 159/161 anywhere reachable.
3. Enumeration through PIO-USB at all: the console prints one `[rp] d/i: ...`
   line per HID interface.
4. Frames arriving: `'W'` opcode on the FPGA (`tools/stats.py`) — `frames_ok`
   climbing, `frames_bad` and `resyncs` flat.
5. Mouse feel on the host. If it stutters or overshoots, that is the pacing
   limit above, not a bug to chase here.

Not done: the FPGA does not tell the RP which host is selected, so a key held
*across* a switch is not re-sent to the new host until it changes; and nothing
is sent from the FPGA to the RP except log text.
