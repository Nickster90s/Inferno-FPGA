#!/bin/bash
# Read the firmware's log out of FLASH. For debug builds where the Type-C is
# the host port (KVM_NATIVE_HOST + KVM_USB_DEBUG) and there is no console.
# The firmware saves its log ring to flash 3 s after it last grew.
#
#   1. put the board in the bootloader (BOOT held while plugging the Type-C in,
#      or BOOT + RESET). Power may drop on the way; the log is in flash.
#   2. ./readlog.sh
#   3. ./flash.sh ... or tap RESET to run the firmware again
#
# RAM was tried first and does not work: the RP2350 boot ROM wipes it.
set -e
cd "$(dirname "$0")"
ADDR=10100000
SIZE=800c
OUT=${TMPDIR:-/tmp}/kvm_conlog.$$.bin
sudo -n "${PICOTOOL:-$HOME/pico/picotool/build/picotool}" save -r 0x$ADDR $(printf '0x%x' $((0x$ADDR + 0x$SIZE))) "$OUT" >/dev/null
sudo -n chmod a+r "$OUT"
python3 - "$OUT" <<'PY'
import struct, sys
d = open(sys.argv[1], "rb").read()
magic, head, tail = struct.unpack_from("<III", d)
ring = d[12:]
if magic != 0x4B564D4C:
    sys.exit(f"no log in flash (magic {magic:#x}) -- not a KVM_USB_DEBUG build, or it never went 3 s quiet")
n = (head - tail) & 0xFFFFFFFF
out = bytes(ring[(tail + i) % len(ring)] for i in range(min(n, len(ring))))
sys.stdout.write(out.decode("latin1").replace("\r", ""))
PY
sudo -n rm -f "$OUT"
