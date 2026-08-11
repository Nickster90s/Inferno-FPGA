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

# WHICH BITSTREAM. build_seed8/ is the bitstream that was VALIDATED on hardware
# (USB ingress exact, audio to three receivers). build/colorlight_i9plus/ is
# whatever was built last, which is not the same thing and may be a diagnostic.
#
# This matters more than it looks: sys Fmax scatters 52-65 MHz for one seed on
# this flow, so "rebuild seed 8" does NOT reproduce the validated bitstream --
# see the seed entry in README Open bugs. Prefer the validated artifact, say so
# out loud, and warn if a newer one exists rather than silently choosing.
BIT_VALIDATED=build_seed8/gateware/colorlight_i9plus.bit
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
