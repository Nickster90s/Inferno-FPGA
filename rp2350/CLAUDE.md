# rp2350 — read this before building or debugging

Firmware for the **Waveshare RP2350-USB-A**: USB keyboard + mouse in (USB-A,
PIO-USB host), 12-byte frames out over UART0 to the FPGA, which relays them to
the Main or Backup host. The FPGA side is `../firmware/kvm.c`; the gateware and
its placement history are in `../CLAUDE.md`. `README.md` here has the long form.

**Status 2026-11-11: compiles and the host-side test passes. It has never run
on the board.** Do not describe anything USB-, UART- or timing-related as
working until it has been measured there.

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
   test/test_host.c src/kvm_sched.c src/hid_parse.c -o /tmp/kvm_test && /tmp/kvm_test
```

Keep them SDK-free. Glue belongs in `main.c`.

## The wire contract is owned by the FPGA

`A5 5A | id | 8 payload | xor(id..payload)`, 1 Mbaud 8N1, id 1 = keyboard,
2 = mouse. Defined in `../firmware/kvm.c` and by the HID report descriptor in
`/home/lisp/FPGA/avb-usb-host/gateware/usb_descriptors.py` (3 mouse buttons,
8-bit X/Y/wheel with minimum −127, key codes 0..0x65). Changing either side
means changing both, and changing the descriptor re-emits the ROM — see
`../CLAUDE.md` for what that costs (seed sweep, two tap sweeps).

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

- Commit and push only when asked. `rp2350/` is untracked on branch `kvm` as of
  2026-11-11.
- Comments say **why**, with the measurement or the file that decided it, in
  the style of the rest of this repo.
