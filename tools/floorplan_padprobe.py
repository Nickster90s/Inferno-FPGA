# nextpnr-xilinx --pre-place probe: WAAR liggen de ULPI-pads werkelijk?
#
# floorplan_usb.py documenteert de Main ULPI op X=1, Y=23-49, maar van de Backup
# ULPI is dat nooit gemeten -- en zes floorplan-varianten faalden omdat ik die
# positie uit pin-letters heb GERADEN (en fout had). Dit meet het.
#
# Run (vanuit de gateware-dir van een DUAL-USB build):
#   nextpnr-xilinx --json colorlight_i9plus.json --xdc colorlight_i9plus.xdc \
#     --chipdb <chipdb>/xc7a50tfgg484.bin --pre-place ../../../tools/floorplan_padprobe.py
#
# Stopt na het dumpen; doet geen P&R.
import sys

def loc_of(cell):
    """Probeer meerdere API-paden; nextpnr's Python-oppervlak varieert per build."""
    for attr in ("bel",):
        b = getattr(cell, attr, None)
        if b is None:
            continue
        for fn in ("getBelLocation",):
            f = getattr(ctx, fn, None)
            if f is None:
                continue
            try:
                L = f(b)
                return (getattr(L, "x", "?"), getattr(L, "y", "?")), str(b)
            except Exception:
                pass
        return None, str(b)
    return None, None

rows = []
api_dumped = False
for cname, cell in ctx.cells:
    low = cname.lower()
    if "ulpi" not in low:
        continue
    ctype = getattr(cell, "type", "?")
    xy, bel = loc_of(cell)
    if not api_dumped and xy is None:
        # Laat zien WAT er beschikbaar is, zodat een verkeerde API-gok zichtbaar is
        print("[padprobe] geen locatie via bel/getBelLocation; cell-attributen:")
        print("   ", sorted(a for a in dir(cell) if not a.startswith("_"))[:24])
        print("    ctx heeft:", sorted(a for a in dir(ctx) if "Bel" in a or "bel" in a)[:14])
        api_dumped = True
    rows.append((cname, str(ctype), xy, bel))

main = [r for r in rows if "ulpi2" not in r[0].lower()]
bkp  = [r for r in rows if "ulpi2" in r[0].lower()]

def report(title, rs):
    print("\n[padprobe] %s -- %d cellen" % (title, len(rs)))
    xs, ys = [], []
    for cname, ctype, xy, bel in sorted(rs):
        if xy and xy[0] != "?":
            xs.append(xy[0]); ys.append(xy[1])
        print("   %-58s %-14s %s  bel=%s" % (cname[:58], ctype, xy, bel))
    if xs:
        print("   => BOUNDING BOX: X %d..%d  Y %d..%d" % (min(xs), max(xs), min(ys), max(ys)))
        print("   => voorstel regio: \"%d,%d,%d,%d\" (met marge)" %
              (max(0, min(xs) - 2), max(0, min(ys) - 6), max(xs) + 46, max(ys) + 6))

report("MAIN ULPI", main)
report("BACKUP ULPI (ulpi2)", bkp)
print("\n[padprobe] totaal %d ulpi-cellen van %d" % (len(rows), len(list(ctx.cells))))
sys.exit(0)
