#!/usr/bin/env python3
"""Regenerate rtl/usb_avb_subsystem{,2}.v with DANTE descriptor identities.

    tools/gen_usb_subsystem.py            # both, into rtl/
    tools/gen_usb_subsystem.py --main     # only Main
    tools/gen_usb_subsystem.py --backup   # only Backup

WHY THIS EXISTS. The two .v files are LUNA/Amaranth output, and their USB
descriptors sit in the module as PACKED 32-bit ROM constants:

    rom_read_port[8] = 32'd3801312;

You cannot edit a product string, a PID or a serial number with search and
replace -- the only honest way to change them is to re-emit the module from the
Amaranth source, which is what this does.

WHAT WAS WRONG (found 2026-10-30). dante-usb's two files were IDENTICAL bar the
module name: `usb_avb_subsystem2.v` was a mechanical rename of
`usb_avb_subsystem.v` (2 differing lines; avb-aes3's genuinely regenerated pair
differs in 695). So BOTH USB devices presented the same VID:PID AND the same
serial number, and both claimed to be the "Backup":

    1209:eab1  "N-Series AVB Switchover Backup"  serial 1   <- on BOTH buses

eab1 is Main's PID, the string is Backup's, and the serial is Backup's -- a mix
of the two identities that matches neither. Consequences:

  * The host cannot tell Main from Backup. udev rules, ALSA card order and the
    whole A/B switchover UX key on exactly these fields, so the switchover
    cannot be driven from the host at all.
  * `usb 1-10: 1:1: cannot get freq (v2/v3): err -71` appeared under this
    bitstream and NOT under avb-aes3's, whose descriptors are distinct.

SEQUENCING -- READ BEFORE RUNNING. Re-emitting changes the netlist, which
changes placement, which MOVES BOTH ULPI EYE CENTRES. The IDELAY taps pinned in
avb_soc.py are properties of a placement, not of the hardware (proven the hard
way: pinning Main's tap 30 reshuffled placement and knocked the Backup PHY off
the bus entirely). So always:

    1. regenerate descriptors (this script)
    2. rebuild the gateware
    3. THEN sweep both taps  (tools/tapsweep.py --phy main / --phy backup)
    4. pin the winners and rebuild once more

Doing step 4 before step 1 means measuring a placement you are about to throw
away.
"""
import argparse
import os
import subprocess
import sys

# The Amaranth source lives in the sibling project that did the USB bring-up.
GEN_DIR = "/home/lisp/FPGA/avb-usb-host/gateware"
GEN = os.path.join(GEN_DIR, "usb_avb_subsystem.py")
# ...and it needs THAT project's venv: amlib, luna and amaranth are not in the
# system interpreter, so running this with plain python3 fails at
# `from amlib.utils import EdgeToPulse` -- harmlessly (it dies before writing),
# but it looks like a generator bug rather than a missing environment.
VENV_PY = "/home/lisp/FPGA/avb-usb-host/.venv/bin/python"

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RTL = os.path.join(HERE, "rtl")

# Dante identities. Distinct PID *and* distinct serial: a host that sees two
# devices with the same pair cannot address them separately, and some stacks
# cache by (VID,PID,serial) and will hand you the wrong one.
VARIANTS = {
    "main":   dict(module="usb_avb_subsystem",  out="usb_avb_subsystem.v",
                   product_name="N-Series Dante Main",
                   serial="0", product_id=0xEAB1),
    "backup": dict(module="usb_avb_subsystem2", out="usb_avb_subsystem2.v",
                   product_name="N-Series Dante Backup",
                   serial="1", product_id=0xEAB2),
}

EMIT = r'''
import sys
sys.path.insert(0, {gen_dir!r})
from amaranth.back import verilog
from usb_avb_subsystem import USBAVBSubsystem
sub = USBAVBSubsystem(standalone=True,
                      product_name={product_name!r},
                      serial={serial!r},
                      product_id={product_id!r},
                      dual_rate={dual_rate!r})
with open({out!r}, "w") as f:
    f.write(verilog.convert(sub, name={module!r}, ports=sub.ports()))
print("wrote", {out!r}, "module", {module!r})
'''


def emit(which, dual_rate=False):
    v = VARIANTS[which]
    out = os.path.join(RTL, v["out"])
    code = EMIT.format(gen_dir=GEN_DIR, out=out, dual_rate=dual_rate, **
                       {k: v[k] for k in ("product_name", "serial",
                                          "product_id", "module")})
    print("=== %s -> %s  (%s, PID %#06x, serial %s%s)"
          % (which, v["out"], v["product_name"], v["product_id"], v["serial"],
             ", DUAL-RATE 48/96k" if dual_rate else ", 48k"))
    py = VENV_PY if os.path.exists(VENV_PY) else sys.executable
    r = subprocess.run([py, "-c", code], cwd=GEN_DIR)
    return r.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--main", action="store_true")
    ap.add_argument("--backup", action="store_true")
    ap.add_argument("--dual-rate", action="store_true",
                    help="dimensioneer het ISO-endpoint op 96 kHz en leg "
                         "rate_48k_i naar buiten, zodat een firmware-CSR de "
                         "door Dante gekozen rate kan melden. LET OP: dit "
                         "reserveert 2880 B/uframe = 23 MB/s per PHY ook op "
                         "48 kHz, dus twee PHYs op EEN host controller passen "
                         "dan niet meer (in productie zit elke PHY aan zijn "
                         "eigen host).")
    a = ap.parse_args()
    if not os.path.exists(GEN):
        sys.exit("generator not found: %s" % GEN)
    if not os.path.exists(VENV_PY):
        print("warning: %s missing; falling back to %s, which probably lacks "
              "amlib/luna" % (VENV_PY, sys.executable), file=sys.stderr)
    which = [w for w, f in (("main", a.main), ("backup", a.backup)) if f] \
        or ["main", "backup"]
    rc = 0
    for w in which:
        rc |= emit(w, a.dual_rate)
    if rc == 0:
        print("\nNow rebuild, THEN re-sweep both taps -- placement changed, so")
        print("the pinned IDELAY taps in avb_soc.py are no longer measured.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
