#!/bin/bash
# nextpnr-xilinx seed sweep — Dante Phase 0.5 support tool.
#
# WHY: placement on this flow is chaotic. A given seed may (a) stall HeAP, (b)
# place but miss a clock target, or (c) meet every global Fmax and still produce
# a broken USB datapath, because the seed shuffles the intra-region USB
# placement and global Fmax does NOT predict USB quality. So a seed must be
# chosen on measured numbers and then confirmed on hardware.
#
# Usage:
#   tools/seed_sweep.sh                 # seeds 1..8, 3 at a time
#   tools/seed_sweep.sh 1 16            # seeds 1..16
#   JOBS=2 tools/seed_sweep.sh 1 8      # limit parallelism
#
# Each seed builds into its own output dir so builds don't collide. Results land
# in tools/seed_sweep_results.txt, sorted best-first.
#
# REQUIREMENTS TO PASS (all must hold):
#   sys_clk    >= 55 MHz   (target 50; below 55 is a build-reject -- sub-50
#                           setup violations have silently corrupted audio)
#   usb_clk    >= 60 MHz
#   eth_tx_clk >= 125 MHz  (gigabit RGMII)
#   eth_rx_clk >= 125 MHz  (gigabit RGMII)
#
# The "FAIL at 125.00 MHz" lines nextpnr prints for sys_clk and usb_clk are
# EXPECTED: sys is deliberately over-constrained to 8 ns so the placer grinds
# harder. Judge against the requirements above, not against nextpnr's verdict.
set -u

HERE="$(cd "$(dirname "$0")/.." && pwd)"
FIRST="${1:-1}"
LAST="${2:-8}"
JOBS="${JOBS:-3}"
OUT="$HERE/tools/seed_sweep_results.txt"
LOGDIR="$HERE/tools/seed_sweep_logs"

mkdir -p "$LOGDIR"

if [[ ! -f "$HERE/firmware/loader/loader.bin" ]]; then
    echo "error: build the loader first: make -C firmware/loader" >&2
    exit 1
fi

echo "sweeping seeds $FIRST..$LAST, $JOBS at a time"
echo "logs: $LOGDIR"

# Some seeds make nextpnr's analytical placer hang: it prints "Running main
# analytical placer", never prints "HeAP Placer Time", and sits at ~100% CPU
# indefinitely. Observed live on seed 3 (14 minutes, no progress). That is a
# property of the seed, not a machine problem, so the sweep must detect it and
# move on instead of burning a job slot forever.
#
# Watchdog: if a build's log has not grown for STALL_SECS while HeAP has not yet
# completed, kill that build and record it as stalled.
STALL_SECS="${STALL_SECS:-240}"

build_one() {
    local seed="$1"
    local log="$LOGDIR/seed$seed.log"
    local dir="$HERE/build_seed$seed"

    ( cd "$HERE" && ./build.sh --seed "$seed" --output-dir "$dir" ) > "$log" 2>&1 &
    local bpid=$!

    while kill -0 "$bpid" 2>/dev/null; do
        sleep 15
        [[ -f "$log" ]] || continue
        # Only police the pre-HeAP window; routing legitimately runs quiet.
        grep -aq "HeAP Placer Time" "$log" && continue
        local idle=$(( $(date +%s) - $(stat -c %Y "$log") ))
        if (( idle > STALL_SECS )); then
            echo "  seed $seed STALLED (no log output for ${idle}s, HeAP never finished) - killing"
            # Kill the nextpnr working in this seed's directory, then the wrapper.
            for p in $(pgrep -f nextpnr-xilinx); do
                if [[ "$(readlink -f /proc/$p/cwd 2>/dev/null)" == "$dir/gateware" ]]; then
                    kill -9 "$p" 2>/dev/null
                fi
            done
            kill -9 "$bpid" 2>/dev/null
            echo "STALLED" >> "$log"
            return
        fi
    done
    wait "$bpid" 2>/dev/null
    echo "  seed $seed done (exit $?)"
}

pids=()
for s in $(seq "$FIRST" "$LAST"); do
    build_one "$s" &
    pids+=("$!")
    while [[ "$(jobs -rp | wc -l)" -ge "$JOBS" ]]; do sleep 5; done
done
wait

# ---- collect -----------------------------------------------------------------
# Take the LAST reported Fmax per clock: nextpnr prints an estimate mid-route and
# the real figure after routing completes.
{
    printf "%-6s %-9s %-9s %-11s %-11s %-8s %s\n" \
           SEED SYS_MHz USB_MHz ETH_TX_MHz ETH_RX_MHz VERDICT BITSTREAM
    for s in $(seq "$FIRST" "$LAST"); do
        log="$LOGDIR/seed$s.log"
        [[ -f "$log" ]] || continue
        get() { grep -a "Max frequency for clock .*'$1'" "$log" | tail -1 \
                | sed -n 's/.*: \([0-9.]*\) MHz.*/\1/p'; }
        sys=$(get 'crg_s7pll0_clkout_buf0')
        usb=$(get 'crg_s7pll1_clkout_buf')
        etx=$(get 'eth_tx_clk')
        erx=$(get 'eth_rx_clk')
        bit="$HERE/build_seed$s/gateware/colorlight_i9plus.bit"
        [[ -f "$bit" ]] && have=yes || have=NO-BIT
        verdict=PASS
        if grep -aq "^STALLED$" "$log"; then
            printf "%-6s %-9s %-9s %-11s %-11s %-8s %s\n" \
                   "$s" - - - - HEAP-STALL "do not use"
            continue
        fi
        for chk in "${sys:-0} 55" "${usb:-0} 60" "${etx:-0} 125" "${erx:-0} 125"; do
            set -- $chk
            awk -v v="$1" -v t="$2" 'BEGIN{exit !(v+0 < t+0)}' && verdict=FAIL
        done
        [[ "$have" == "NO-BIT" ]] && verdict=NOBUILD
        printf "%-6s %-9s %-9s %-11s %-11s %-8s %s\n" \
               "$s" "${sys:--}" "${usb:--}" "${etx:--}" "${erx:--}" "$verdict" "$have"
    done
} | tee "$OUT"

echo
echo "results also in $OUT"
echo "NEXT: pick a PASS seed, pin it as the --seed default in avb_soc.py,"
echo "      then flash it and TEST USB ON HARDWARE -- Fmax does not predict USB."
