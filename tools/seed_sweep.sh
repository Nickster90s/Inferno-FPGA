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
# FMAX IS NOT ENOUGH -- CONFIRM USB ON HARDWARE. Measured 2026-08-07, same
# netlist and firmware, three seeds:
#
#     seed 4   USB leak +0.04%   shortfall -0.00%   underrun    0/s
#     seed 6   USB leak +0.01%   shortfall -0.00%   underrun    0/s
#     seed 5   USB leak +13.06%  shortfall +10.84%  underrun 5200/s
#
# Seed 5 passed every criterion below AND had the highest sys Fmax (65.68), so
# picking on Fmax chose the WORST seed. The ingress loss shows as a starved ring
# and gritty audio, and looks exactly like a firmware or design regression.
#
# 2026-08-07, arbitrary-fpp netlist (after the 512-byte frame-ceiling fix).
# THE SAME TRAP FIRED AGAIN, so it is worth restating with fresh numbers:
#
#     seed 1   sys 63.54 (HIGHEST clean)  USB 79.62   -> MacBook enumerated the
#                                                        device and showed NO
#                                                        OUTPUTS AT ALL. Dead.
#     seed 8   sys 63.26                  USB 85.55   -> ep_out 9,216,211/s vs
#                                                        9,216,000 baseline
#                                                        (+0.002%), leak +0.00%,
#                                                        underrun 0/s, ring 65,
#                                                        unicast audio confirmed
#                                                        to AM2 + A16R + DVS.
#
# Seed 1 was picked on sys Fmax, exactly what the paragraph above says not to do,
# and it broke USB outright. USB_MHz was the column that predicted it (79.62 vs
# 85.55) -- when two seeds are within a MHz on sys, PREFER THE HIGHER USB_MHz.
#
# AND: sys Fmax here is not reproducible. Seed 8 measured 54.97 in a serial
# build and 63.26 in this sweep, same source, same loader, same parsing. That is
# the run-to-run scatter avb_soc.py:37 documents (52-65 MHz on ONE seed); the
# PYTHONHASHSEED=0 re-exec reduced it but has NOT eliminated it. So treat the
# BITSTREAM as the validated artifact, not the seed number -- rebuilding "seed 8"
# may not give you the bitstream that was measured. Keep build_seed8/.
#
# After choosing a PASS seed, flash it and read the USB counters over UDP:
#     tools/stats.py ... opcode 'u'  -> rx_beats, ep_out
# leak = (rx_beats - ep_out)/rx_beats must be < 1%, and ep_out must be within
# ~0.1% of channels*rate*4 bytes/s (9,216,000 for 48ch @ 48k). If it is not,
# try the next PASS seed -- it is placement, not code.
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

# SWEEP THE VALIDATED CONFIGURATION BY DEFAULT.
#
# avb_soc.py reads BOARD, USB2 and USB_DUAL_RATE from the environment and
# build.sh does not set them, so a sweep launched from a plain shell builds the
# SINGLE-USB 48 kHz chip. That happened on 2026-11-07: eight seeds, every log
# healthy, zero `ulpi2` lines in any XDC, and the question being asked was about
# the dual-USB 96 kHz design. A whole sweep answered about the wrong chip.
#
# These are defaults, not overrides -- export them yourself to sweep something
# else, and build.sh prints and records whatever it ends up with.
export BOARD="${BOARD:-breakout}"
export USB2="${USB2:-1}"
export USB_DUAL_RATE="${USB_DUAL_RATE:-1}"
echo "config: BOARD=$BOARD USB2=$USB2 USB_DUAL_RATE=$USB_DUAL_RATE"
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
    printf "%-6s %-9s %-9s %-9s %-11s %-11s %-8s %s\n" \
           SEED SYS_MHz USB_MHz USB2_MHz ETH_TX_MHz ETH_RX_MHz VERDICT BITSTREAM
    for s in $(seq "$FIRST" "$LAST"); do
        log="$LOGDIR/seed$s.log"
        [[ -f "$log" ]] || continue
        get() { grep -a "Max frequency for clock .*'$1'" "$log" | tail -1 \
                | sed -n 's/.*: \([0-9.]*\) MHz.*/\1/p'; }
        # THE NAMES GAINED A `main_` PREFIX AND THIS SILENTLY BROKE.
        # The patterns used to be 'crg_s7pll0_clkout_buf0', but nextpnr now
        # prints 'main_crg_s7pll0_clkout_buf0' -- with the quote before `main`,
        # so the old pattern matched nothing, sys and usb came back EMPTY, the
        # ${sys:-0} default made them 0, and every seed was reported FAIL.
        # On 2026-11-06 that declared a sweep of seven seeds a total loss while
        # seed 8 had in fact passed every requirement.
        sys=$(get 'main_crg_s7pll0_clkout_buf0')
        usb=$(get 'main_crg_s7pll1_clkout_buf')
        # usb2 WAS NEVER CHECKED AT ALL, and it is not cosmetic: in that same
        # sweep seeds 3 and 4 met every other requirement and failed ONLY on
        # usb2 (56.36 and 59.15 against 60), so with the parser fixed and this
        # line missing they would both have been reported PASS with a Backup PHY
        # below spec.
        usb2=$(get 'main_crg_s7pll2_clkout_buf')
        etx=$(get 'eth_tx_clk')
        erx=$(get 'eth_rx_clk')
        bit="$HERE/build_seed$s/gateware/colorlight_i9plus.bit"
        # A BITSTREAM THAT EXISTS IS NOT A BITSTREAM FROM THIS SWEEP. A failed
        # build leaves the PREVIOUS one in place, and this column used to report
        # it as `yes`: on 2026-11-06 seeds 2 and 5 died and still advertised
        # bitstreams from 08-14 and 08-13. Flashing one of those against today's
        # csr.h is the hazard flash.sh devotes a paragraph to.
        #
        # THE TEST IS AGAINST ITS OWN csr.h, NOT AGAINST THE LOG. Comparing to
        # the log reports everything stale, because the log keeps growing after
        # the bitstream is written (the fasm parser warnings come last) -- the
        # first version of this check did exactly that. csr.h is generated
        # BEFORE place-and-route in the same run, so "bit newer than csr.h" is
        # both a freshness test and the header-pairing check this project
        # already lives by.
        csr="$HERE/build_seed$s/software/include/generated/csr.h"
        if [[ ! -f "$bit" ]]; then
            have=NO-BIT
        elif [[ -f "$csr" && "$bit" -ot "$csr" ]]; then
            have=STALE-$(date -r "$bit" +%m-%d)
        else
            have=yes
        fi
        verdict=PASS
        if grep -aq "^STALLED$" "$log"; then
            printf "%-6s %-9s %-9s %-11s %-11s %-8s %s\n" \
                   "$s" - - - - HEAP-STALL "do not use"
            continue
        fi
        for chk in "${sys:-0} 55" "${usb:-0} 60" "${usb2:-0} 60" \
                   "${etx:-0} 125" "${erx:-0} 125"; do
            set -- $chk
            awk -v v="$1" -v t="$2" 'BEGIN{exit !(v+0 < t+0)}' && verdict=FAIL
        done
        [[ "$have" == NO-BIT || "$have" == STALE-* ]] && verdict=NOBUILD
        printf "%-6s %-9s %-9s %-9s %-11s %-11s %-8s %s\n" \
               "$s" "${sys:--}" "${usb:--}" "${usb2:--}" "${etx:--}" "${erx:--}" \
               "$verdict" "$have"
    done
} | tee "$OUT"

echo
echo "results also in $OUT"
echo "NEXT: pick a PASS seed, pin it as the --seed default in avb_soc.py,"
echo "      then flash it and TEST USB ON HARDWARE -- Fmax does not predict USB."
