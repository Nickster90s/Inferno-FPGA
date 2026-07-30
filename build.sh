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

# HeAP stall watchdog.
#
# Some seeds make nextpnr's analytical placer hang: it prints "Running main
# analytical placer", never prints "HeAP Placer Time", and sits at 100% CPU
# forever. Observed on seed 3, and again on seed 5 after a gateware change --
# that one burned 43 minutes before anyone looked. It is a property of the
# (seed, netlist) pair, so it reappears whenever the gateware or loader changes.
#
# Fail fast instead of hanging. Set STALL_SECS=0 to disable.
#   NOTE: after ANY gateware or loader change, re-sweep with tools/seed_sweep.sh
#   rather than trusting the pinned --seed. The pin is only valid for the
#   netlist it was measured on.
STALL_SECS="${STALL_SECS:-300}"

if [[ "$STALL_SECS" == "0" ]]; then
    exec setarch -R python3 "$HERE/avb_soc.py" --build "${ROM_IMAGE_ARGS[@]}" "$@"
fi

BUILD_LOG="$(mktemp -t infernobuild.XXXXXX.log)"
echo "build log: $BUILD_LOG  (stall watchdog ${STALL_SECS}s)"
setarch -R python3 "$HERE/avb_soc.py" --build "${ROM_IMAGE_ARGS[@]}" "$@" \
    2>&1 | tee "$BUILD_LOG" &
BPID=$!

( while kill -0 "$BPID" 2>/dev/null; do
      sleep 15
      [[ -f "$BUILD_LOG" ]] || continue
      grep -aq "HeAP Placer Time" "$BUILD_LOG" && exit 0   # past the risky phase
      idle=$(( $(date +%s) - $(stat -c %Y "$BUILD_LOG") ))
      if (( idle > STALL_SECS )); then
          echo ""
          echo "*** HeAP STALL: no build output for ${idle}s and HeAP never finished."
          echo "*** This seed does not place on this netlist. Killing."
          echo "*** Try another seed, or run: tools/seed_sweep.sh"
          pkill -9 -f nextpnr-xilinx 2>/dev/null
          kill -9 "$BPID" 2>/dev/null
          exit 1
      fi
  done ) &
WDOG=$!

wait "$BPID" 2>/dev/null
RC=$?
kill "$WDOG" 2>/dev/null
exit $RC
