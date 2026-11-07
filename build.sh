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

# THE DESIGN'S CONFIGURATION LIVES IN ENVIRONMENT VARIABLES, SO SAY WHICH ONE
# THIS BUILD IS. BOARD, USB2 and USB_DUAL_RATE are read by avb_soc.py and
# nothing enforces them: a script that forgets to export USB2=1 builds the
# SINGLE-USB chip and says so nowhere.
#
# Measured 2026-11-07: a seed sweep was started without them and all eight
# builds were the single-USB, 48 kHz design -- zero `ulpi2` lines in the XDC --
# while the question being asked was about the dual-USB 96 kHz build. The logs
# looked perfectly healthy. Nothing was wrong except that it was the wrong chip.
#
# So every build now prints its configuration and leaves it in the output
# directory, which means a build dir can be asked afterwards what it is.
# PRINTING IT WAS NOT ENOUGH -- SO THE VALIDATED PROFILE IS NOW THE DEFAULT.
# Two 8-seed sweeps were lost on 2026-11-07, three hours, because a caller has
# to assemble four variables correctly and a warning in a build log is read
# after the build, not before it. Reporting a mistake is weaker than making it
# impossible: there is one configuration this project ships, so building it
# must be what `./build.sh` with no arguments does.
#
# Every one of these is overridable -- set it explicitly and it is kept, which
# is what the `${VAR:=default}` form means. To build something else on purpose,
# name the variable (USB2=0 ./build.sh) or opt out of the profile entirely with
# DANTE_PROFILE=none. The echo below always states what is actually in effect,
# so an override is visible rather than silent.
# THE PROFILE IS THE RECORDED RECIPE, COPIED FROM README.md -- SIX ITEMS.
# Taking "the four variables" from memory was wrong twice in one afternoon. The
# build that produced the measured-good result is written out in full in the
# README under "Fixed (2026-11-04): Main's ULPI input eye sat beyond the delay
# line", and it names two more:
#
#   USB_PERIOD_NS=14   an over-constraint (71.43 MHz against a real 60) that
#                      makes the placer grind cd_usb harder. The resulting
#                      `FAIL at 71.43 MHz` lines are EXPECTED -- score the MHz,
#                      not PASS/FAIL. Unset, the target is 60 and the usb
#                      domain is optimised less; that is a different build.
#                      (Do NOT do the same to SYS_PERIOD_NS: 20.0 is the
#                      requirement and 16.0/14.0 hang HeAP outright.)
#   USB_PHASE_DEG=333  -27 deg on cd_usb ONLY. Named explicitly even though
#                      avb_soc.py already defaults to it, because it is the
#                      whole reason Main's eye is clean: at phase 0 even IDELAY
#                      tap 31, the hardware maximum, still leaked 1.28% of real
#                      audio. At 333 every tap measures clean. cd_usb2 stays at
#                      phase 0 -- Backup was already well placed.
#
# STALL_SECS is the watchdog, not the design, and the recipe uses 900. The 300
# default below killed two seeds at ~245 s on 2026-11-07 while HeAP was still
# working; a false stall throws away a good seed, which costs more than waiting.
if [[ "${DANTE_PROFILE:-dual96}" != "none" ]]; then
    : "${BOARD:=breakout}"
    : "${USB2:=1}"
    : "${USB_DUAL_RATE:=1}"
    : "${NEXTPNR_USB_FP_LIGHT:=1}"
    : "${USB_PERIOD_NS:=14}"
    : "${USB_PHASE_DEG:=333}"
    : "${STALL_SECS:=900}"
    export BOARD USB2 USB_DUAL_RATE NEXTPNR_USB_FP_LIGHT USB_PERIOD_NS USB_PHASE_DEG
fi

# AND THE VALIDATED CONFIGURATION IS FOUR VARIABLES, NOT THREE.
# Measured 2026-11-07, the same day and the same mistake one variable over:
# the fix above was applied with NEXTPNR_USB_FP_LIGHT left out, so a second
# sweep built the FULL USB floorplan box. That box is infeasible by
# construction -- DUAL_USB_96K_HANDOFF.md:81 records 11 seeds stalling with no
# error, and floorplan_usb.py:394 puts the numbers at ~697 cells light versus
# ~3000 full. The sweep logs proved it exactly: the build that places has 1429
# cells in usb_fp, the eight that hung had 3109 in the same region. Nothing
# else differed. So name all four here, and see the derived dump below for the
# other nineteen.
echo "=== build config: BOARD=${BOARD:-<unset>} USB2=${USB2:-<unset>} USB_DUAL_RATE=${USB_DUAL_RATE:-<unset>} NEXTPNR_USB_FP_LIGHT=${NEXTPNR_USB_FP_LIGHT:-<unset>} USB_PERIOD_NS=${USB_PERIOD_NS:-<unset>} USB_PHASE_DEG=${USB_PHASE_DEG:-<unset>} ==="
if [[ -z "${USB2:-}" || -z "${USB_DUAL_RATE:-}" || "${NEXTPNR_USB_FP_LIGHT:-}" != "1" ]]; then
    echo "    NOTE: this is NOT the validated dual-USB 96 kHz configuration."
    echo "    That one is: BOARD=breakout USB2=1 USB_DUAL_RATE=1 NEXTPNR_USB_FP_LIGHT=1 \\"
    echo "                 USB_PERIOD_NS=14 USB_PHASE_DEG=333 STALL_SECS=900"
fi

HERE="$(cd "$(dirname "$0")" && pwd)"

# Welke build-directory deze run gebruikt. Stond eerst BINNEN het loader-blok,
# maar de stall-watchdog heeft het ook nodig om zijn kill te scopen.
OUT_DIR="$HERE/build/colorlight_i9plus"
_prev=""
for a in "$@"; do
    [[ "$_prev" == "--output-dir" ]] && OUT_DIR="$a"
    _prev="$a"
done
[[ "$OUT_DIR" != /* ]] && OUT_DIR="$HERE/$OUT_DIR"

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

    # The loader is COMPILED AGAINST generated/csr.h and BAKED INTO the ROM, so
    # any gateware change that moves a CSR address silently invalidates it: the
    # bitstream builds clean, flashes fine, and the loader then pokes registers
    # that have moved. This has cost a debugging session FOUR times now --
    # adding LiteScope shifted the addresses once, removing it shifted them back.
    #
    # Rebuild it here so it always tracks the LAST generated csr.h, and record
    # that header's checksum. The post-build check below compares it against the
    # header this run generates; they differ exactly when the loader needs
    # another pass, which is the case that used to pass unnoticed.
    # Honour --output-dir. tools/seed_sweep.sh gives every seed its own build
    # directory, so a hardcoded path here would compare a csr.h that the sweep
    # build never writes -- the guard would sit silently inert on exactly the
    # builds that run unattended.
    CSR_H="$OUT_DIR/software/include/generated/csr.h"

    # Serialise the loader rebuild. The sweep runs JOBS builds concurrently and
    # they all share firmware/loader/, so three make's racing in one directory
    # can interleave object writes and bake a torn loader into a bitstream whose
    # Fmax then gets recorded as if it meant something.
    ( flock 9 || exit 1
      make -C "$HERE/firmware/loader" >/dev/null 2>&1 ) 9>"$HERE/firmware/loader/.build.lock" || {
        echo "error: loader rebuild failed" >&2; exit 1; }
    # Hash the CSR CONTENT, not the file: LiteX stamps every generated header
    # with "Auto-generated by LiteX (hash) on <date>", so a plain md5sum differs
    # on every single build and the check below would cry wolf every time --
    # which is worse than no check, because a warning that is always wrong gets
    # ignored on the run where it is right.
    csr_fingerprint() { grep -v "Auto-generated by LiteX" "$1" | md5sum | cut -d' ' -f1; }
    CSR_BEFORE=""
    [[ -f "$CSR_H" ]] && CSR_BEFORE=$(csr_fingerprint "$CSR_H")
    trap '
        if [[ -n "$CSR_BEFORE" && -f "$CSR_H" ]]; then
            CSR_AFTER=$(csr_fingerprint "$CSR_H")
            if [[ "$CSR_BEFORE" != "$CSR_AFTER" ]]; then
                echo ""
                echo "*** STALE LOADER *** csr.h CHANGED during this build, so the"
                echo "    loader baked into this bitstream was compiled against the"
                echo "    OLD CSR addresses. DO NOT FLASH IT. Re-run this build:"
                echo "      make -C firmware/loader && ./build.sh $*"
            fi
        fi' EXIT
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
mkdir -p "$OUT_DIR"
# EVERY SWITCH, DERIVED FROM THE SOURCES -- NOT A HAND-KEPT LIST.
# The hand-kept list had three names while the design reads 23, and the one it
# omitted (NEXTPNR_USB_FP_LIGHT) is the one that decides whether the netlist can
# be placed at all. A derived list cannot fall behind: add an os.environ.get to
# avb_soc.py or floorplan_usb.py and it shows up here by itself.
{ echo "built=$(date -Is)"
  echo "args=$*"
  for _v in $(grep -ohE 'environ(\.get\(|\[)"[A-Z0-9_]+"' \
                   "$HERE/avb_soc.py" "$HERE/floorplan_usb.py" \
              | grep -oE '"[A-Z0-9_]+"' | tr -d '"' | sort -u); do
      printf '%s=%s\n' "$_v" "${!_v-}"
  done; } > "$OUT_DIR/build_config.txt"

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
          # SCOPE DE KILL. Dit was `pkill -9 -f nextpnr-xilinx`, wat ELKE
          # nextpnr op de machine omlegt -- op 2026-10-30 schoot een seed-sweep
          # daarmee een parallelle SA-run van 20 minuten dood, en de log zei
          # alleen "Killed", wat niet van een plaatsingsfout te onderscheiden
          # is. Twee builds naast elkaar is normaal op een 4-core machine, dus
          # de watchdog mag alleen zijn EIGEN nextpnr raken.
          #
          # Scopen op de command line kan niet: elke build geeft nextpnr
          # dezelfde argumenten (zelfde json-naam, zelfde chipdb, zelfde
          # --pre-place). Maar elke build draait zijn nextpnr met cwd =
          # <output-dir>/gateware, en dat is wel uniek.
          _mine="$(readlink -f "$OUT_DIR/gateware" 2>/dev/null)"
          for _p in $(pgrep -f nextpnr-xilinx 2>/dev/null); do
              [[ -n "$_mine" && "$(readlink -f /proc/$_p/cwd 2>/dev/null)" == "$_mine" ]] \
                  && kill -9 "$_p" 2>/dev/null
          done
          kill -9 "$BPID" 2>/dev/null
          exit 1
      fi
  done ) &
WDOG=$!

wait "$BPID" 2>/dev/null
RC=$?
kill "$WDOG" 2>/dev/null
exit $RC
