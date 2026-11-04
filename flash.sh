#!/bin/bash
# Flash the current build to the board.
#
#   ./flash.sh                 volatile: pld load + firmware over Ethernet
#   ./flash.sh --permanent     SPI: bitstream at 0x0 AND firmware at 3 MB
#   ./flash.sh --unlock        one-time flash unlock (MX25L128 ships protected)
#   ./flash.sh --fw-only       firmware only, over Ethernet (no bitstream)
#
#   --bit PATH                 use a specific bitstream
#
# DEFAULT IS VOLATILE, DELIBERATELY. `pld load` writes SRAM configuration, so a
# power cycle falls back to whatever is in SPI flash -- the cheapest possible
# rollback for a design that can in principle take the board off the network.
# Use --permanent only for a build you have listened to.
set -e
cd "$(dirname "$0")"

OPENOCD=/home/lisp/openocd/src/openocd
OCD_TCL=/home/lisp/openocd/tcl
TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
CFG=$TOOLS/ch347.cfg
IFACE=${IFACE:-ens5}
BOARD=${BOARD:-169.254.9.200}

# WHICH BITSTREAM. bitstreams/dual-usb-96k_2026-11-03_seed8_VERIFIED.bit is the
# one that was VALIDATED on hardware WITH BOTH PHYS: 48 ch @ 96 kHz on each
# simultaneously, PHY-hold, OSC handover, forced handover to a dead source,
# PTP locked, 0 failing clocks. build/colorlight_i9plus/ is whatever was built
# last, which is not the same thing and may be a diagnostic.
#
# IT USED TO POINT AT build_seed8/, AND THAT WAS WRONG SINCE DUAL USB LANDED.
# build_seed8/ has NO usb2 in its XDC -- it is the SINGLE-USB build (validated
# in its day: USB ingress exact, audio to three receivers). Anyone running
# ./flash.sh without --bit was therefore quietly putting one PHY on the board.
# Found 2026-11-04; the dual-USB bitstream existed only in a session temp dir
# at that point, so it was also one cleanup away from being lost.
#
# ---- AND THE FIRMWARE MUST MATCH THE BITSTREAM'S CSR MAP ----
# This script runs `make -C firmware`, which compiles against
# build/colorlight_i9plus/software/include/generated/csr.h -- the headers of
# whatever was generated LAST, not of the bitstream being flashed. On
# 2026-11-04 those were 20 minutes NEWER than the .bit and carried one extra
# CSR (aaf_pkt write_drop_count at 0x68), which pushes fifo_level, dbg_last_ts,
# dbg_last_sec, holdover_count and ts_subsec_max up by 4 bytes. CSR banks sit
# on fixed 0x800 boundaries so nothing outside aaf_pkt moves, but that is no
# comfort: fifo_level feeds the USB isochronous feedback accumulator
# (firmware/main.c) and dbg_last_sec/dbg_last_ts compute the Dante emit
# timestamp (firmware/dante_tx.c), so a 4-byte shift buys you host sample-rate
# drift and a missed playout deadline -- "no audio" with perfect packets.
# BEFORE --permanent OR --fw-only, check that csr.h is not newer than the .bit:
#     ls -l --time-style=+%H:%M:%S build/colorlight_i9plus/gateware/*.bit \
#        build/colorlight_i9plus/software/include/generated/csr.h
# If it is, regenerate the matching header from the commit that built the .bit:
#     git worktree add /tmp/pre <that-commit>
#     cd /tmp/pre && BOARD=breakout USB2=1 USB_DUAL_RATE=1 \
#        python3 ./avb_soc.py --soft-only --firmware firmware/loader/loader.bin \
#        --output-dir /tmp/pre-out
# and copy ONLY csr.h over (soc.h differs just in the ident string,
# variables.mak carries absolute paths and must NOT be copied), then
# `make -C firmware clean && make -C firmware`.
#
# Until 2026-11-03 "rebuild seed 8" did NOT reproduce the validated bitstream:
# LiteX baked a second-resolution build timestamp into the ident ROM, so every
# build handed nextpnr a different netlist (README, seed entry). Fixed with
# ident_version=False, so a rebuild of the same seed on the same source now
# does reproduce. The preference below stays anyway -- the VALIDATED artifact is
# the one that was listened to on hardware, and timing closure is not the same
# claim as working audio. Prefer the validated artifact, say so out loud, and
# warn if a newer one exists rather than silently choosing.
BIT_VALIDATED=bitstreams/dual-usb-96k_2026-11-03_seed8_VERIFIED.bit
BIT_SINGLE_USB=build_seed8/gateware/colorlight_i9plus.bit   # historisch, 1 PHY
BIT_LATEST=build/colorlight_i9plus/gateware/colorlight_i9plus.bit
BIT=""
MODE=volatile

while [[ $# -gt 0 ]]; do
    case "$1" in
        --permanent|-p) MODE=permanent ;;
        --unlock)       MODE=unlock ;;
        --fw-only)      MODE=fwonly ;;
        --bit)          BIT="$2"; shift ;;
        -h|--help)      sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

if [[ -z "$BIT" ]]; then
    if [[ -f "$BIT_VALIDATED" ]]; then
        BIT="$BIT_VALIDATED"
    elif [[ -f "$BIT_LATEST" ]]; then
        BIT="$BIT_LATEST"
    fi
fi

if [[ "$MODE" != "fwonly" && "$MODE" != "unlock" ]]; then
    [[ -f "$BIT" ]] || { echo "no bitstream found (looked for $BIT_VALIDATED)" >&2; exit 1; }
    echo "bitstream: $BIT"
    if [[ "$BIT" == "$BIT_VALIDATED" && -f "$BIT_LATEST" && "$BIT_LATEST" -nt "$BIT_VALIDATED" ]]; then
        echo "  NOTE: $BIT_LATEST is NEWER. Using the validated one anyway."
        echo "        Pass --bit $BIT_LATEST if you meant the fresh build."
    fi
fi

# openocd and picocom cannot both hold the CH347 -- it carries JTAG *and* the
# UART on one interface.
if fuser -s /dev/ttyACM0 2>/dev/null; then
    echo "*** /dev/ttyACM0 is held (picocom?). Close it first." >&2
    exit 1
fi

# NEVER FLASH WHILE AUDIO IS STREAMING. `pld load` reconfigures the FPGA, so the
# USB device disappears MID-TRANSFER. The host sees an isochronous TD that stops
# responding and xHCI can take the whole controller down with it:
#     xhci_hcd: ERROR Transfer event TRB DMA ptr not part of current TD
#     xhci_hcd: xHCI host controller not responding, assume dead
#     xhci_hcd: HC died; cleaning up
# That happened on 2026-11-04 and it took out BOTH PHYs and the CH347 -- i.e.
# the JTAG bridge you would need to undo it. Recovery is possible without a
# reboot:
#     echo 0000:00:14.0 | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind
#     sleep 3
#     echo 0000:00:14.0 | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind
# but the cheaper thing is to not do it. Stop the player first.
_busy=""
for _st in /proc/asound/card*/pcm0p/sub0/status; do
    [[ -r "$_st" ]] || continue
    if grep -q "state: RUNNING" "$_st" 2>/dev/null; then
        _card="${_st#/proc/asound/card}"; _card="${_card%%/*}"
        _name=$(sed -n "/^ $_card \[/s/.*: USB-Audio - //p" /proc/asound/cards 2>/dev/null)
        _busy+=" card$_card${_name:+ ($_name)}"
    fi
done
if [[ -n "$_busy" ]] && [[ "$FLASH_ALLOW_STREAMING" != "1" ]]; then
    echo "*** AUDIO IS STREAMING on:$_busy" >&2
    echo "*** Flashing now pulls the USB device out mid-transfer and can kill" >&2
    echo "*** the xHCI host controller (all USB, including the CH347)." >&2
    echo "*** Stop the player, then re-run. FLASH_ALLOW_STREAMING=1 overrides." >&2
    exit 1
fi

# Always build the firmware: "flash the latest version" must not ship a stale
# binary just because someone forgot to run make.
if [[ "$MODE" != "unlock" ]]; then
    echo "=== building firmware ==="
    make -C firmware 2>&1 | tail -3
fi

case "$MODE" in

unlock)
    echo "=== unlocking SPI flash (once per board) ==="
    sudo "$OPENOCD" -s "$OCD_TCL" -f "$CFG" \
        -c "init; pld load 0 $TOOLS/unlock_flash_xc7a50t.bit; exit"
    echo "=== done -- now run: ./flash.sh --permanent ==="
    ;;

fwonly)
    echo "=== firmware over Ethernet ($IFACE) ==="
    echo "    resetting the board to open the loader window..."
    sudo ./tools/netload.py "$IFACE" firmware/firmware.bin &
    NL=$!
    sleep 2
    sudo stty -F /dev/ttyACM0 1000000 raw -echo 2>/dev/null || true
    printf 'r' | sudo tee /dev/ttyACM0 >/dev/null
    wait $NL
    ;;

volatile)
    # netload FIRST: it must already be listening when the loader's boot window
    # opens, which is only a few hundred ms after configuration.
    echo "=== volatile load (power cycle reverts to SPI flash) ==="
    sudo ./tools/netload.py "$IFACE" firmware/firmware.bin &
    NL=$!
    sleep 1
    sudo "$OPENOCD" -s "$OCD_TCL" -f "$CFG" -c "init; pld load 0 $BIT; exit"
    wait $NL || echo "netload returned $?"
    ;;

permanent)
    echo "=== SPI: bitstream at 0x0 ==="
    sudo "$OPENOCD" -s "$OCD_TCL" -f "$CFG" -c "
        set XC7_JSHUTDOWN 0x0d; set XC7_JPROGRAM 0x0b; set XC7_BYPASS 0x3f
        init; pld load 0 $TOOLS/bscan_spi_xc7a50t.bit; reset halt
        flash probe 0; flash protect 0 0 50 off
        flash write_image erase $BIT 0x0 bin
        irscan xc7.tap \$XC7_JSHUTDOWN; irscan xc7.tap \$XC7_JPROGRAM
        runtest 60000; runtest 2000; irscan xc7.tap \$XC7_BYPASS; runtest 2000; reset
    " -c exit

    # The bitstream alone boots into the LOADER, which then waits for netload and
    # falls through to SPI. Without the firmware image at 3 MB the board comes up
    # with no application after a power cycle -- flashed, and mute.
    echo "=== SPI: firmware image at 0x300000 ==="
    ./tools/mkimage.py firmware/firmware.bin firmware/firmware.img
    sudo "$OPENOCD" -s "$OCD_TCL" -f "$CFG" -c "
        init; pld load 0 $TOOLS/bscan_spi_xc7a50t.bit; reset halt
        flash probe 0; flash protect 0 0 50 off
        flash write_image erase firmware/firmware.img 0x300000 bin
    " -c exit

    echo
    echo "*** SPI is read at POWER-UP only -- a warm reset does not reload it."
    echo "*** Power cycle the board to run what was just written."
    ;;
esac

if [[ "$MODE" == "volatile" || "$MODE" == "fwonly" ]]; then
    echo "=== checking the board answers ==="
    sleep 8
    if timeout 6 python3 tools/stats.py "$BOARD" 2>/dev/null \
         | grep -E "ptp_locked|flows_active|fifo_level"; then
        echo "=== done ==="
    else
        echo "*** no reply from $BOARD:7779 -- it may still be booting." >&2
        echo "*** if it stays silent, re-run; netload retries the image 3x." >&2
        exit 1
    fi
fi
