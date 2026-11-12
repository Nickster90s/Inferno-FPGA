#!/bin/bash
# Copy a .uf2 to the RP2350. This box does not auto-mount the boot drive, so:
#   ./flash.sh            build/kvm_rp2350.uf2
#   ./flash.sh --debug    build_dbg/kvm_rp2350.uf2 (TinyUSB host log on the console)
#   ./flash.sh FILE.uf2
#
# If our firmware is running and its console (1209:eab3) is on this machine it
# is asked to enter the bootloader ('B'). Otherwise put the board there by
# hand: hold BOOT, tap RESET (or plug the Type-C in), release BOOT.
#
# TYPE-C AND VSYS: never both. Disconnect VSYS from the FPGA board first.
set -e
cd "$(dirname "$0")"
UF2=build/kvm_rp2350.uf2
case "$1" in
    --debug) UF2=build_dbg/kvm_rp2350.uf2 ;;
    "") ;;
    *) UF2="$1" ;;
esac
[[ -f "$UF2" ]] || { echo "no $UF2 -- build first (README.md)" >&2; exit 1; }

bootdev() { lsblk -rno NAME,LABEL | awk '$2=="RP2350"{print "/dev/"$1; exit}'; }

if [[ -z "$(bootdev)" ]]; then
    for t in /dev/ttyACM*; do
        if udevadm info -q property "$t" 2>/dev/null | grep -q 'ID_MODEL_ID=eab3'; then
            echo "asking the running firmware on $t to enter the bootloader..."
            sudo sh -c "stty -F $t raw -echo; printf B > $t"
        fi
    done
    for _ in $(seq 20); do [[ -n "$(bootdev)" ]] && break; sleep 0.5; done
fi
DEV=$(bootdev)
[[ -n "$DEV" ]] || { echo "no RP2350 boot drive: hold BOOT, tap RESET, release BOOT" >&2; exit 1; }

MNT=$(mktemp -d)
sudo mount "$DEV" "$MNT"
sudo cp "$UF2" "$MNT/"
sync
sudo umount "$MNT" 2>/dev/null || true      # the drive vanishes when it reboots
rmdir "$MNT" 2>/dev/null || true
echo "wrote $UF2"
for _ in $(seq 20); do
    lsusb -d 1209:eab3 >/dev/null 2>&1 && { echo "firmware is running (1209:eab3)"; exit 0; }
    sleep 0.5
done
echo "*** copied, but 1209:eab3 did not appear" >&2; exit 1
