# Inferno-FPGA — 48-channel USB → Dante interface

A standalone hardware audio interface: a host streams 48 channels over USB, and
an FPGA puts them on the network as a native Dante transmitter. No PC-side
driver stack, no Dante Virtual Soundcard, no licensed Audinate module.

**Board:** Colorlight i9plus v6.1 (Xilinx Artix-7 XC7A50T-fgg484-1)
**Toolchain:** openXC7 (yosys + nextpnr-xilinx) — fully FOSS, no Vivado
**SoC:** LiteX + VexRiscv, bare-metal C firmware
**USB:** LUNA/Amaranth UAC2 high-speed device on a USB3300 ULPI PHY

> ### Status: audio works. Phases 0-5 and 7 complete on the bench.
>
> The device appears in Dante Controller, locks to PTPv1, and streams audio to
> real Dante hardware over **unicast flows** negotiated on port 4455 — verified
> against a Focusrite RedNet A16R and a RedNet AM2 simultaneously, at different
> `fpp` and channel counts, sustained at exactly 9001 pps with zero underruns or
> overruns. Multicast flows can be created and deleted from Dante Controller.
>
> Two receivers stream simultaneously at different `fpp` and channel counts.
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
                           │              │                │   6 × Dante
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
  the host paces itself to the media clock. There is no sample-rate converter
  and nothing chases the FIFO, so nothing can run away.

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
| 6 | Hardening / long-run | 24 h, no dropouts | not started |
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

**Media clock is UNDISCIPLINED under PTPv1 — drifts ~15 ms/hour.** Still open,
and the most serious issue. Left running the stream dies silently: every counter
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
collapse mode (risk 8); PTP stays locked despite it, but it is real frame loss
and wants the `rx_gate` MAC allow-list.

**Clipping at full source volume** is unconfirmed as ours. The digital path is
a bit-exact MSB-justified truncation with no gain stage, so it cannot create
clipping that is not already in the source — but this has not been verified at
high level, because it needs a multicast flow to make the payload visible.

**USB is only partly retested** on the current gateware. The device enumerates
as `N-Series USB 48CH` and 2 channels were verified end to end after the unicast
gateware work. All 48 have NOT been exercised on this bitstream — the historical
failure mode in this lineage was 48-channel-specific (channel rotation after a
dropped sample, fixed by channel-addressed ingress), so a 2-channel pass does
not clear it.

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
| `v` | toggle verbose prints (default off) |
| `r` | reboot |
| `h` / `?` | help |

`b` is the early-warning instrument. Targets are **>10,000 iterations/s and 0
writer errors**. If iterations/s falls below ~2000 or writer errors climb once
real Dante multicast is on the wire, the CPU is being swamped by flooded audio
and needs a MAC-level RX allow-list — not a protocol parser.

---

## Repository layout

```
avb_soc.py              LiteX SoC top: CRG, LiteEth+TSU, MCR NCO, USB instance,
                        packetizer wiring, clock constraints, floorplan hooks
aaf_packetizer.py       gateware packetizer: 6×8ch rings → frame builder →
                        TXFrameArbiter.  Phase 5 → dante_packetizer.py
floorplan_usb.py        --pre-place region constraint for the USB block
build.sh                deterministic build wrapper (PYTHONHASHSEED + setarch -R)

tools/
  netload.py            push firmware into coderam over raw Ethernet (dev loop)
  mkimage.py            wrap firmware in the loader's header for SPI flash
  seed_sweep.sh         build N seeds, tabulate Fmax per clock
  test_netload_protocol.py
                        host-side conformance test: a model of the loader's
                        receive state machine driven by the real sender

firmware/
  loader/               FROZEN boot loader baked into the 12 KB ROM. Loads an
                        image into coderam from Ethernet or SPI flash and jumps.
                        Changing it re-rolls placement -- avoid.
  main.c                boot, RX dispatcher, UART CLI, main polling loop
  gptp.c                802.1AS gPTP slave + PI servo on the 52-bit TSU addend.
                        Phase 4 factors the servo out for reuse by ptpv1.c
  mcr.c                 media-clock NCO servo
  osc.c                 minimal ARP/IPv4/UDP shim.  Phase 2 → net.c
  cap.c                 on-FPGA packet capture ring
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

- [inferno](https://gitlab.com/lumifaza/inferno) — Dante for Linux, in Rust
- [statime](https://github.com/pendulum-project/statime) — `no_std` PTP in Rust
- [network-audio-controller](https://github.com/chris-ritsen/network-audio-controller) —
  open Dante device/routing control (`netaudio` CLI), useful for testing
- [LiteX](https://github.com/enjoy-digital/litex) / [LiteEth](https://github.com/enjoy-digital/liteeth)
- [LUNA](https://github.com/greatscottgadgets/luna) — USB gateware in Amaranth
- [openXC7](https://github.com/openXC7) — FOSS Xilinx 7-series flow
- [aes67-linux-daemon](https://github.com/bondagit/aes67-linux-daemon) — the
  AES67 alternative, if standards-based interop suits you better
