# rp2350 — read this before building or debugging

Firmware for the **Waveshare RP2350-USB-A**: USB keyboard + mouse in (USB-A,
PIO-USB host), 12-byte frames out over UART0 to the FPGA, which relays them to
the Main or Backup host. The FPGA side is `../firmware/kvm.c`; the gateware and
its placement history are in `../CLAUDE.md`. `README.md` here has the long form.

**Status 2026-11-12: runs on the board.** Keyboard and mouse reach both hosts,
`/kvm/main` and `/kvm/backup` route them, hot-plug works. Wired low-speed
keyboard + mouse behind a hub work **on both builds with the right hub** -- see
"Hubs" below before touching anything USB. The default (USB-A) build is the one
in use: it needs no adapter and keeps the console.

## Two builds, two host ports

| build dir | cmake | host port | console |
|---|---|---|---|
| `build/` | (default) | USB-A, bit-banged (Pico-PIO-USB) | CDC on the Type-C |
| `build_typec/` | `-DKVM_NATIVE_HOST=1` | **Type-C**, the RP2350's own controller, through a C-to-A OTG adapter | none |

`./flash.sh [FILE.uf2]` copies a build to the board (this box does not
auto-mount the boot drive). The default build accepts `B` on its console to
enter the bootloader; the Type-C build needs BOOT + RESET.

Bench-only switches: `-DKVM_USB_DEBUG=1` (TinyUSB's log; on the Type-C build it
is saved to flash and read with `./readlog.sh`), `-DKVM_LINK_BEACON=1`
(progress as counts in the FPGA's `resyncs`), `-DKVM_PRE_ACK_QBITS` /
`-DKVM_PRE_GAP_QBITS` (PIO-USB timing).

## Hubs: the chip decides, and it cost a day

**A hub with chip `214b:7250` does not pass low-speed devices when the host is
full-speed -- and the RP2350 can only be full-speed.** Two of them were tried.
Measured 2026-11-12 with a Cherry mouse (`046a:b091`) and a keyboard
(`1c4f:0043`), both low-speed:

| host | hub `214b:7250` | hub `0a05:7211` |
|---|---|---|
| Type-C, native controller | fails: data-sequence errors, RX timeouts, 0 reports | **works at once** |
| USB-A, Pico-PIO-USB | fails: 15-30% of transfers lost, 1 run in ~13 enumerates | **works** (mouse, keyboard, 1985 frames) |

The failure is the same on two unrelated host implementations, which is what
pointed at the hub. On a PC both hubs work, because a PC talks high-speed to
`214b:7250` and never uses its full-speed low-speed path. **The tell is in
`dmesg` on a PC: the working hub enumerates as `new full-speed USB device`, the
broken one as `new high-speed`.** Full-speed devices (a wireless combo dongle)
work through either hub.

So when "the hub does not work": identify the hub chip and the device speed
FIRST. Do not tune timing. A day went into `KVM_PRE_*` sweeps and four library
versions before a different hub answered it in a minute.

## Patches to the libraries in ~/pico (`patches/`)

The libraries live outside the repo, so the changes are kept as patch files and
are currently APPLIED in `~/pico`. A fresh clone needs them re-applied.

- `tinyusb-0.18-hcd-rp2040-data-seq.patch` -- the native host driver called
  `panic()` on a data-sequence error and never completed a transfer that timed
  out. Both happen routinely with a marginal hub; the first halted the firmware
  four boots out of four. Now counted and survived. **Needed by the Type-C
  build.** The stop-and-fail path still hit `panic: ep 80 was already
  available` twice on the bad hub; the watchdog covers it.
- `pico-pio-usb-0.7.3-pre-turnaround.patch` -- waits 4 instead of 1.5 low-speed
  bit times before the handshake behind a hub. It moved the bad hub from "never
  past SET_ADDRESS" to "sometimes". The good hub was only ever run WITH it, so
  whether it is needed there is not known -- do not drop it untested.

## Power: do not hot-plug a bus-powered hub on pad 18

Pad 18 (VSYS) is the FPGA board's 5 V. Plugging the bus-powered `214b:7250`
hub into the RP2350 made **Main and Backup disappear from both hosts at the
same instant** while the FPGA's CPU carried on (four times, once with the exact
moment observed). They stayed away 17 minutes, until the FPGA was reloaded.
With a self-powered hub it did not happen. `tools`-side opcode `'H'` (dstats.c)
pulses both PHY resets and re-enumerates a healthy board in half a second;
whether it recovers a board after such a dip is NOT yet tested.

Type-C plus pad 18 at once is fine when both come from the same PC (they did
here all day); it is two different supplies that must not meet.

## No console on the Type-C build: how to see anything

- It sends one all-released keyboard frame at boot, so `frames_ok` on the FPGA
  ticks once per (re)start -- a watchdog reset shows up as an extra frame.
- A watchdog (2 s) restarts it after a panic or hang.
- RAM does not survive the bootloader (the boot ROM wipes it; measured), so the
  debug build writes its log to flash 3 s after the log goes quiet.

## Building

Nothing is installed system-wide. The toolchain lives in `~/pico` and nothing
sets the environment for you:

```sh
export PATH=$HOME/pico/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:$PATH
export PICO_SDK_PATH=$HOME/pico/pico-sdk          # 2.2.0, TinyUSB 0.18
cmake -S . -B build -G Ninja && ninja -C build    # -> build/kvm_rp2350.uf2
```

`PICO_PIO_USB_PATH` defaults to `~/pico/Pico-PIO-USB` (0.7.3); it is not part of
the SDK. Ninja's output is 120 lines of SDK noise — grep it for
`warning|error|FAILED`, our own sources build with zero warnings.

**Run the test after touching `kvm_sched.c` or `hid_parse.c`.** Those two files
are deliberately free of SDK includes so they run on the build machine:

```sh
cc -Wall -Wextra -fsanitize=address,undefined -Isrc \
   test/test_host.c src/kvm_sched.c src/hid_parse.c src/kvm_down.c \
   -o /tmp/kvm_test && /tmp/kvm_test
```

Keep them (and `kvm_down.c`) SDK-free. Glue belongs in `main.c`.

## The wire contract is owned by the FPGA

`A5 5A | id | 8 payload | xor(id..payload)`, 1 Mbaud 8N1, id 1 = keyboard,
2 = mouse. Defined in `../firmware/kvm.c` and by the HID report descriptor in
`/home/lisp/FPGA/avb-usb-host/gateware/usb_descriptors.py` (3 mouse buttons,
8-bit X/Y/wheel with minimum −127, key codes 0..0x65). Changing either side
means changing both, and changing the descriptor re-emits the ROM — see
`../CLAUDE.md` for what that costs (seed sweep, two tap sweeps).

**And back, FPGA → RP2350, same frame, ids 0x80 and up.** `0x80` STATUS: p0 =
target (0 Main, 1 Backup), p1 = version 1, p2..5 = the FPGA's switch count.
Sent on every `/kvm/main` / `/kvm/backup` and once a second, so either end can
restart alone; `kvm_target()` in `main.c` returns it, or UNKNOWN after 3.5 s of
silence. Nothing acts on it yet. The same wire carries the FPGA's log text, and
`kvm_down.c` separates the two -- sender is `kvm_send_status()` in
`../firmware/kvm.c`. Measured 2026-11-12: four OSC switches, four target lines
on the console with the switch count following.

Pins: **GPIO0 = TX → FPGA `W20`**, **GPIO1 = RX ← FPGA `AB22`**. USB-A is
GPIO12 (D+) / GPIO13 (D−).

## The FPGA endpoint relays STATE — this is why reports are paced

The report register is sent to the host on **every 1 ms poll** until it is
overwritten, and one register serves both report IDs. So:

- a motion frame left standing **repeats** (dx=5 for 20 ms = 100 counts);
- a report overwritten before the host polled **never happened**.

`kvm_sched.c` therefore emits at most one frame per ms: keyboard reports and
button changes are held 2 ms (`KVM_HOLD_TICKS`), a motion frame stands exactly
1 ms and is always followed by something motionless. **Do not "simplify" this
into forwarding reports as they arrive** — that is the obvious design and it
is wrong on this hardware.

Mouse motion is approximate by construction (occasionally delivered 0 or 2
times; the host's poll clock is not ours). That is a known limit, not a bug to
chase here. The exact fix is a gateware endpoint that sends each write once and
NAKs otherwise.

## Things that cost time once already

- **`CFG_TUSB_OS` comes from the SDK's command line (`OPT_OS_PICO`).** Do not
  define it in `tusb_config.h`.
- **`tusb_time_millis_api()` must be provided by us** (`main.c`). TinyUSB's
  examples get it from their BSP; without it the link fails, not the compile.
- **`pico_stdio_usb` cannot coexist with a TinyUSB host**, and stdio on UART0
  would put text on the FPGA link. Both are disabled; the console is our own
  CDC interface (`con_printf`). Never `printf`.
- **No control requests from `tuh_hid_mount_cb`.** It runs mid-enumeration and
  the stack's next step is a control request for the device's next interface;
  ours in flight makes that fail and a combo dongle loses an interface. The
  mouse's SET_PROTOCOL is sent from `protocol_task()` once `tuh_mounted()`.
- **`CFG_TUH_ENUMERATION_BUFSIZE` is 512 on purpose.** The report descriptor is
  fetched into that buffer; one that does not fit is silently not delivered and
  the mouse drops to boot protocol with no wheel.
- **System clock is 120 MHz**, not the RP2350's default 150: Pico-PIO-USB needs
  a multiple of 12 MHz.
- **Core 1 does only the 1 ms frame tick.** Pico-PIO-USB runs its bus
  transactions inside a timer interrupt on core 0; putting the tick there makes
  the 1 ms motion window as wide as the last transaction was long.
- **Never forward CDC input to the UART.** The FPGA reads its RX as HID frames
  only; typed characters are just resync noise there.

## Hardware rules

- **Do not plug in the Type-C while VSYS is fed from the FPGA board.** Type-C
  VBUS is on the VSYS net with no diode. That covers flashing *and* reading the
  console.
- **R13** (1.5 kΩ pull-up on a USB-A data line) is device-mode signalling;
  `../KVM_HANDOFF.md` says remove it for host use. Not yet confirmed on a board.
- Flash by BOOT + Type-C + copy the `.uf2`. Account is not in `dialout`, so
  reading the CDC console (`/dev/ttyACM*`) needs `sudo`.

## Bench checks, in order

1. Console prints one `[rp] d/i: ...` line per HID interface → enumeration works.
2. FPGA opcode `'W'` (`../tools/stats.py`): `frames_ok` climbing, `frames_bad`
   and `resyncs` flat → the link works.
3. Only then judge mouse feel.

## Conventions

- Commit and push only when asked.
- Comments say **why**, with the measurement or the file that decided it, in
  the style of the rest of this repo.
