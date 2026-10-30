#!/bin/bash
# Seed sweep voor de DUAL-USB build (USB2=1), met de USB-floorplan UIT.
#
# WAAROM EEN EIGEN SWEEP EN NIET tools/seed_sweep.sh:
# die schrijft hardcoded naar build_seed$N/ -- daar staat het single-USB
# materiaal, inclusief de gevalideerde bitstreams. Deze houdt alles in EEN boom
# en raakt build/, build_seed*/ en de .validated/.singleusb kopieen niet aan.
#
# WAAROM NEXTPNR_USB_FP=0: gemeten 2026-10-30, de dual-USB netlist stalt ZEVEN
# keer in HeAP met de USB-floorplan en plaatst compleet zonder hem. Niet de
# BRAM (59/75 na ringhalvering) en niet de regio-geometrie (6 varianten) -- het
# is de floorplan zelf: USB1's box werkt voor EEN subsysteem en wordt
# onplaatsbaar zodra de cellen van het tweede bestaan.
#
# ZONDER FLOORPLAN SCATTERT sys 39-57 MHz (de comment in floorplan_usb.py), dus
# we hebben een seed nodig die >= 50 haalt. Daarom deze sweep.
#
#   tools/usb2_sweep.sh [eerste] [laatste]      # default 1..8
#   JOBS=2  parallelle builds (default 2)
#
# MAPSTRUCTUUR (alles onder build_usb2_sweep/):
#   README.txt        deze uitleg + criteria
#   results.txt       tabel, beste eerst
#   logs/seedN.log    volledige buildlog
#   seedN/            BEWAARDE set: .bit + generated headers + csr.csv/json
#   work/seedN/       volledige builddir, WORDT GEWIST na pruning
#
# Per seed blijft ~2.2 MB staan i.p.v. ~200 MB: routed.json (81M), json (34M),
# fasm (17M) en rpt (9M) zijn intermediates die je niet nodig hebt om te flashen.
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"
FIRST="${1:-1}"; LAST="${2:-8}"; JOBS="${JOBS:-2}"
ROOT="$HERE/build_usb2_sweep"
STALL="${STALL_SECS:-900}"

# Veiligheidsnet: nooit in de single-USB bomen schrijven.
case "$ROOT" in *build_usb2_sweep) : ;; *) echo "onveilige ROOT: $ROOT" >&2; exit 1;; esac

mkdir -p "$ROOT/logs" "$ROOT/work"
cat > "$ROOT/README.txt" <<TXT
DUAL-USB (USB2=1) seed sweep, USB-floorplan UIT (NEXTPNR_USB_FP=0).

  seedN/      bewaarde set per seed: gateware/*.bit, software/include/generated,
              csr.csv, csr.json  (~2.2 MB) -- genoeg om te flashen en om
              firmware tegen te bouwen:
                  make -C firmware BUILD_DIR=\$PWD/build_usb2_sweep/seedN
              LET OP: variables.mak bakt BUILDINC_DIRECTORY als ABSOLUUT pad in.
              Verplaats deze map dus niet; kopieer hem niet om firmware tegen te
              bouwen. Dat heeft eerder een firmware tegen de verkeerde CSR-map
              opgeleverd.
  logs/       volledige buildlogs
  work/       volledige builddirs; worden na pruning gewist
  results.txt tabel met Fmax per klok + verdict

CRITERIA (alle vier moeten halen):
  sys     >= 50 MHz   (ontwerp loopt op 50; liefst >= 55 marge)
  eth_rx  >= 125 MHz  gigabit RGMII RX
  eth_tx  >= 125 MHz
  usb,usb2>= 60 MHz   ULPI

Fmax voorspelt GEEN USB-kwaliteit (seed_sweep.sh: "Fmax does not predict USB").
Na het flashen apart meten: HS-enumeratie in dmesg, leak < 1% via
tools/usbdiag.py, en de Backup via tools/usbsrc.py.
TXT

fmax () { grep -iE "Max frequency for clock +'$2'" "$1" | tail -1 | grep -oE "[0-9]+\.[0-9]+" | head -1; }

one () {
  local s=$1 log="$ROOT/logs/seed$s.log" w="$ROOT/work/seed$s"
  rm -rf "$w"; mkdir -p "$w"
  USB2=1 NEXTPNR_USB_FP=0 STALL_SECS="$STALL" \
    ./build.sh --seed "$s" --output-dir "$w" > "$log" 2>&1
  local bit="$w/gateware/colorlight_i9plus.bit"
  if [[ -f "$bit" ]]; then
    mkdir -p "$ROOT/seed$s/gateware" "$ROOT/seed$s/software/include"
    cp "$bit" "$ROOT/seed$s/gateware/"
    cp -a "$w/software/include/generated" "$ROOT/seed$s/software/include/" 2>/dev/null
    cp "$w/csr.csv" "$w/csr.json" "$ROOT/seed$s/" 2>/dev/null
    echo "seed $s: OK"
  else
    echo "seed $s: GEEN BITSTREAM"
  fi
  rm -rf "$w"            # intermediates weg: 146 MB per seed
}

echo "sweep seeds $FIRST..$LAST, $JOBS parallel, output in $ROOT"
for s in $(seq "$FIRST" "$LAST"); do
  while (( $(jobs -rp | wc -l) >= JOBS )); do sleep 10; done
  one "$s" &
done
wait

{ printf "%-5s %8s %8s %8s %8s %8s  %s\n" SEED sys usb usb2 eth_rx eth_tx VERDICT
  for s in $(seq "$FIRST" "$LAST"); do
    L="$ROOT/logs/seed$s.log"
    [[ -f "$ROOT/seed$s/gateware/colorlight_i9plus.bit" ]] || { printf "%-5s %8s %8s %8s %8s %8s  %s\n" "$s" - - - - - "GEEN BITSTREAM"; continue; }
    sy=$(fmax "$L" crg_s7pll0_clkout_buf0); u1=$(fmax "$L" crg_s7pll1_clkout_buf)
    u2=$(fmax "$L" crg_s7pll2_clkout_buf);  rx=$(fmax "$L" eth_rx_clk); tx=$(fmax "$L" eth_tx_clk)
    v=$(python3 -c "
sy,u1,u2,rx,tx=[float(x or 0) for x in ('${sy:-0}','${u1:-0}','${u2:-0}','${rx:-0}','${tx:-0}')]
ok = sy>=50 and rx>=125 and tx>=125 and u1>=60 and u2>=60
print(('PASS' if ok else 'FAIL') + ('' if ok else ' <-' + ' '.join(
  n for n,c in (('sys',sy>=50),('eth_rx',rx>=125),('eth_tx',tx>=125),('usb',u1>=60),('usb2',u2>=60)) if not c)))")
    printf "%-5s %8s %8s %8s %8s %8s  %s\n" "$s" "${sy:--}" "${u1:--}" "${u2:--}" "${rx:--}" "${tx:--}" "$v"
  done
} | tee "$ROOT/results.txt"
echo
echo "resultaten: $ROOT/results.txt"
du -sh "$ROOT" 2>/dev/null
