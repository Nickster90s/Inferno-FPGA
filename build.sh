#!/bin/bash
# Deterministic openXC7 build: ASLR off (setarch -R) so nextpnr's pointer-hashed
# placement is reproducible run-to-run, + PYTHONHASHSEED=0 for migen/floorplan.
# Without ASLR off, the SAME seed thrashes one run and places fine the next.
#
# DANTE PHASE 0.5: the ROM image is the frozen LOADER, not the application
# firmware. Firmware now lives in coderam and is delivered at runtime:
#
#   firmware/loader/  -> baked into the 12 KB BRAM ROM (this build)
#   firmware/         -> pushed over Ethernet with tools/netload.py, or from
#                        SPI flash, WITHOUT rebuilding the bitstream
#
# So a firmware change no longer re-rolls P&R placement. Rebuild the bitstream
# only when the gateware or the loader changes.
#
# For a standalone single-image bitstream (no loader, no runtime load):
#   ./build.sh --bake-firmware --firmware firmware/firmware.bin
set -e

export CHIPDB=/home/lisp/FPGA/demo-projects/chipdb
export PRJXRAY_DB_DIR=/home/lisp/openxc7/openxc7/opt/nextpnr-xilinx/external/prjxray-db
export PYTHONHASHSEED=0

HERE="$(cd "$(dirname "$0")" && pwd)"

# Default ROM image = the loader, unless the caller overrides --firmware.
ROM_IMAGE_ARGS=()
if [[ ! " $* " =~ " --firmware " ]]; then
    LOADER="$HERE/firmware/loader/loader.bin"
    if [[ ! -f "$LOADER" ]]; then
        echo "error: $LOADER not found. Build it first:" >&2
        echo "         make -C firmware/loader" >&2
        exit 1
    fi
    ROM_IMAGE_ARGS=(--firmware "$LOADER")
fi

exec setarch -R python3 "$HERE/avb_soc.py" --build "${ROM_IMAGE_ARGS[@]}" "$@"
