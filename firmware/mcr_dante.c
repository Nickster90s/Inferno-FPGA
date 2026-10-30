// mcr_dante.c — the Dante media clock. See mcr_dante.h for why.

#include "mcr_dante.h"
#include "ptpv1.h"
#include "gptp.h"
#include "telem.h"
#include "config.h"
#include "dante_tx.h"

#include <generated/csr.h>
#include <stdio.h>

// ---- Tuning ----------------------------------------------------------------

// One update per second. The PTPv1 Sync rate is 1-2 Hz, so there is no new
// information faster than this, and a slower NCO is a kinder disturbance to the
// USB feedback servo.
#define UPDATE_MS        1000u

// Maximum rate change per update. 100 ppb/s means the whole 4.7 ppm crystal
// error is corrected in ~47 s -- fast enough that drift never matters, slow
// enough that the USB PI servo (which corrects at SOF rate against a ring of
// 2048 samples) absorbs it without the level moving perceptibly.
//
// THIS IS THE PARAMETER THE PREVIOUS TWO ATTEMPTS DID NOT HAVE. Both wrote the
// NCO at main-loop rate with no limit at all.
#define SLEW_PPB         100

// Sanity clamp. The measured crystal error is ~4.7 ppm; anything beyond 100 ppm
// is a bug in the servo or a bad rate estimate, not a real oscillator.
#define MAX_PPB          100000

// Auto-disable guard. underrun_count ticks at the 48 kHz media strobe while the
// ring is un-primed, so this threshold is a DUTY CYCLE: 4800 ticks/s = 10% of
// the time emitting silence. Both previous attempts ended with audible damage
// left running on the bench; this one reverts itself instead.
#define UNDERRUN_TRIP_PER_S  4800u

// ...but only once the ring has actually been carrying audio. With no USB host
// attached the ring sits empty and underrun pins at 48000/s legitimately (this
// is the state the bench is in with the MacBook off), which must not look like
// a fault. A level that has reached a quarter scale means real content.
#define RING_ACTIVE_LEVEL    32

// Warm-start persistence. Flash is a sector erase per save, so save rarely and
// only once the slew has converged on the target.
#define NV_SAVE_MS           600000u     // at most once per 10 minutes
#define NV_SAVE_DELTA_PPB    200         // and only if it moved meaningfully

// How close the applied rate must be to the PTP estimate before the talker is
// allowed to start. One slew step (100 ppb) plus margin.
#define RATE_READY_PPB       250

// At or above this the ring is simply empty (48 kHz strobe = 100% duty), which
// means no USB source -- never a reason to blame the media clock.
#define UNDERRUN_NO_SOURCE_PER_S  24000u

// Consecutive one-second polls the trip condition must hold before it is
// believed.
//
// 45, not 6. This guard has now fired spuriously FOUR times in this project:
// lvl_max matching the talker-off constant, then a duty-cycle misread, then the
// USB host starting playback, and now the firmware feedback loop's convergence.
// That last one is instructive: the loop starts at nominal feedback with about
// 40000 underruns/s and converges over roughly a minute, so the underrun rate
// SWEEPS DOWN THROUGH the 4800..24000 trip band and sits inside it for tens of
// consecutive seconds. Six was nowhere near enough.
//
// The cost of tripping late is bounded -- a genuine clock fault persists, so it
// will still be caught, just 45 s later. The cost of tripping early is that the
// media clock free-runs at the crystal's ~4.5 ppm and the timestamp walks out of
// every receiver's window, which is exactly what happened: drift climbed +4 ->
// +48 samples with the NCO pinned at nominal, and re-arming by hand brought it
// straight back to -4.
//
// If this fires a fifth time, the answer is not another constant. The guard
// should key on something that cannot be produced by a transient -- a ring that
// is simultaneously un-primed AND not refilling, for instance.
#define TRIP_STREAK_S        45u

// ---- PHASE: a real PLL on PTP, deliberately feeble --------------------------
//
// DISABLED BY DEFAULT. Two earlier attempts at this broke audio; read why
// before enabling it.
//
// THE PROBLEM. Rate discipline alone is a FREQUENCY-locked loop: it stops the
// error growing but never returns what has accumulated, and the NCO increment
// is a 32-bit integer so ~0.12 ppm of rate error is not even expressible.
// Measured: +0.26 ppm residual, 0.9 ms/hour of phase walk with nothing pulling
// it back. A receiver at 1 ms latency crosses its whole buffer in about an
// hour. That is what "the media clock feels like it is free running" was.
//
// WHY A TEXTBOOK PLL FAILS HERE. A normal PLL assumes the VCO can be moved
// freely. This NCO feeds TWO loops: the Dante timestamp (wants phase lock to
// PTP) and the USB async feedback (the host rate-matches to this same NCO). The
// actuator is also the reference for a faster loop, so moving it quickly
// destabilises the ring. Attempt 1 was proportional-only with a +/-2 ppm clamp
// and a deadband: it limit-cycled and was audible. Attempt 2 avoided the servo
// entirely and added a fractional NCO in gateware -- correct arithmetic (sim:
// 0.0 ppb error) but it dithers STROBE TIMING at 50 MHz, and the USB feedback
// measures strobes-per-SOF, so the ring starved (avg level 36, ~9700 underrun/s).
//
// THE CONSTRAINT THAT FOLLOWS. Loop bandwidth must sit far below the USB loop's.
// What the host demonstrably tolerates: the rate slew of 0 -> -4300 ppb at
// 100 ppb/s ran with ZERO underruns, validated with audio. And the arithmetic is
// kind -- nulling 1 ms (48 samples) of phase over an hour needs only 0.28 ppm.
// So the loop wants a time constant of ~hours, i.e. millihertz bandwidth. That
// is not a compromise: path delay spread here is 4-6 us, so a wide loop would
// chase network jitter straight into the audio.
//
// QUANTISATION SOLVES ITSELF. An integral this slow naturally dithers the
// increment between N and N+1 at ~1 Hz. 0.24 ppm x 48000 = 0.0115 samples/s, so
// a 1 s dither moves the ring by 0.01 samples against a 2048-sample ring --
// nothing. Dither was never the problem; dither at 50 MHz was.
//
// SETPOINT: A FIXED TARGET, not a latched one.
//
// The first version latched the setpoint from whatever drift read after the
// anchor. That holds phase, but it holds it WHEREVER THAT BOOT HAPPENED TO
// LAND -- measured at -37, -26 and -53 samples on successive boots, i.e. the
// absolute offset varied by +/-0.5 ms and was never corrected, only frozen.
// Against a receiver at 1 ms latency that is exactly "the audio is out of
// sync", and it explains why it differed every time the board came up.
//
// So target a defined position instead. inferno's flows_tx.rs uses
// CLOCK_OFFSET_NS = -500_000, i.e. 24 samples IN THE PAST, and is explicit that
// the clock must not be in the future ("otherwise Dante devices receiving from
// us go mad and fart"). Driving to that makes the offset deterministic across
// boots and puts us on the documented value rather than an accident of when the
// anchor fired.
//
// Note this finally makes DANTE_TX_TS_OFFSET's mislabelling harmless: that
// constant says +74 while the wire measures ~-37 because of emit-path latency
// nobody budgeted. The loop closes on the MEASURED position, so it does not
// matter what the open-loop constant claims.
// -24 WAS TWICE A 0.25 ms RECEIVER'S ENTIRE BUDGET, and it was never caught
// because the phase term has never run against a real receiver.
//
// What a receiver measures is (measured latency = const - drift), verified on
// the bench 2026-08-12 across three devices at once:
//
//   A16R fpp=4    reads  3 samples at drift +3   -> const  6
//   AM2  fpp=16   reads 16 samples at drift +3   -> const 19
//   DVS  fpp=60   reads 81 at drift +3, 65 at +17 -> const ~83, tracking 1:1
//
// (const = (fpp-1) + path delay, which is why it differs per receiver.) Driving
// drift to -24 therefore ADDS 24 samples to every receiver's measured latency.
// For the A16R that is 6 + 24 = 30 samples = 0.63 ms against a 12-sample,
// 0.25 ms budget: the loop would have taken it from "too early" straight past
// "too late" into discarding audio.
//
// -24 came from inferno's flows_tx.rs CLOCK_OFFSET_NS = -500 us. That is a
// sane figure for a device at 1 ms or more, and it is 2x the whole budget at
// 0.25 ms. THIS IS THE SAME MISTAKE, WITH THE SAME NUMBER, that was already
// found and fixed in DANTE_TX_REANCHOR_SAMPLES (24 -> 8, "24 samples is 0.5 ms
// -- double a 0.25 ms receiver's entire budget"). It survived here because
// nothing had ever exercised it.
//
// -4, not 0, because THE LOOP DOES NOT PARK ON ITS TARGET. The NCO renders
// whole 242 ppb LSBs and the integral freezes whenever err reads 0, so with
// Kp=30 the steady state sits 2..5 samples ABOVE target -- the quantisation
// floor of (LSB/2)/Kp already described at the gain constants. At target 0 that
// leaves the A16R at 1..4 samples of measured latency out of 12: back against
// the zero rail this whole exercise is about getting away from.
//
// Offsetting the target by the loop's own bias puts the settled band at
// -2..+1, i.e. A16R 5..8 of 12 -- dead centre, and as far from "grey at zero"
// as from "red at the top". sims/sim_mclk_phase_trim.py check 1 asserts this in
// RECEIVER terms rather than as a distance from target, because a distance from
// target is not what breaks audio.
//
// THE TARGET MUST BE JUDGED AGAINST THE TIGHTEST SUBSCRIBED RECEIVER, never
// against a constant borrowed from another implementation whose minimum latency
// is four times ours. If a receiver tighter than 0.25 ms ever binds, this and
// DANTE_TX_REANCHOR_SAMPLES both have to be revisited together.
// UPDATED TO -1 once nco_render() started dithering the increment. The -4 above
// compensated for the loop parking 2..5 samples above target, and that offset
// was the 242 ppb ACTUATOR QUANTISATION -- (LSB/2)/Kp. The dither removes the
// quantisation, so it removes the offset that -4 existed to cancel: at -4 the
// loop now parks at exactly -4 and leaves the A16R at 10 of 12, near the top of
// its budget. -1 parks drift at 0, i.e. A16R at 6 of 12, dead centre, with zero
// steady-state swing. sims/sim_mclk_phase_trim.py, check 1.
#define PHASE_TARGET_SAMPLES     (-1)
// Hard clamp on the loop's contribution. MUST SPAN SEVERAL NCO LSBs.
//
// One increment LSB is 1/4123169 = 242 ppb. The first version clamped at 300 --
// barely ONE LSB -- and the integral saturated there within 10 minutes and sat
// pinned: it could not wind past a grid point to dither between two adjacent
// increments, which is how a slow integral expresses a value BETWEEN them.
// Measured: requested -300 ppb but only ~150 ppb of drift correction appeared,
// because the actuator could only render -242 or -485.
//
// 2000 ppb is ~8 LSB, enough headroom for the integral to settle onto a dither
// point and to absorb PTP feedforward error without hitting the rail. Still far
// below anything the ring notices: the output goes through the 100 ppb/s slew
// limiter, and 2 ppm delivered at that rate takes 20 s -- gentler than the 43 s
// startup slew that ran with zero underruns.
//
// This is the same quantisation the gateware fractional NCO was meant to fix.
// A slow integral dithering between LSBs at ~1 Hz achieves it in firmware and
// avoids the 50 MHz strobe jitter that starved the ring when it was done in
// hardware (0.24 ppm x 48000 = 0.0115 samples/s -- 0.01 samples per second
// against a 2048-sample ring).
#define PHASE_MAX_PPB            2000

// Integral gain, in 1/PHASE_I_SCALE ppb per sample of error per 1 Hz update.
// At 48 samples of error the integral grows 48*6/1024 = 0.28 ppb per second, so
// it reaches the 280 ppb needed to null 1 ms in ~1000 s. Hours, as intended.
// Raised 6 -> 40 after the loop was shown to run against a LIVE ring with no
// effect on it (level 58..67, underrun 0/s, trips 0). At 6 it needed ~1 hour to
// acquire, which is academic when the thing being corrected moves in minutes.
// At 40 the same 48-sample error is nulled in ~10 minutes, still two orders of
// magnitude slower than the USB feedback loop it must not disturb.
// 10, with PHASE_KP_NUM/DEN raised to 30/1 -- see sims/sim_mclk_phase_trim.py,
// which models this loop against the residual actually measured (+0.35 ppm).
//
// THE SHIPPED GAINS CANNOT HOLD PHASE. Kp was 2/5 = 0.4 ppb per sample. The
// plant is an INTEGRATOR (1 ppb -> 4.8e-5 samples/s), so Kp=0.4 is a 14.5 HOUR
// time constant: the proportional term does nothing on any useful timescale,
// the integral runs the loop alone, and it limit-cycles over 20 samples. A
// receiver at 0.25 ms has a budget of 12. Enabling the phase term as shipped
// would have been worse than the drift it removes, which is presumably why it
// defaults off -- but the reason was never written down.
//
// Kp=30 is tau ~= 12 min, settles within 1 sample, 1-sample steady-state swing,
// peaks at 734 ppb against the 2000 clamp, and never exceeds the 100 ppb/s slew
// limiter -- the constraint whose ABSENCE broke the audio twice before. 0.73 ppm
// against the USB feedback servo's 20800 ppm of authority.
#define PHASE_KI                 10
#define PHASE_I_SCALE            1024

// Proportional gain (ppb per sample), as a fraction. Damping only -- kept small
// so a transient cannot jerk the rate. 48 samples -> ~19 ppb.
#define PHASE_KP_NUM             30
#define PHASE_KP_DEN             1

// How long after an anchor to wait before latching the setpoint, so the drift
// measurement has settled.
#define PHASE_SETPOINT_SETTLE_MS 15000

// ---- State -----------------------------------------------------------------

static uint32_t base_inc;
static uint32_t applied_inc;
static uint32_t last_written_inc;
static int32_t  applied_ppb;
static int32_t  target_ppb;
static uint8_t  enabled;
static uint32_t last_update_ms;
static uint32_t nco_writes;
static uint32_t trips;
static uint32_t last_underrun;
static uint32_t underrun_per_s;
static int32_t  drift_samples;
static uint8_t  drift_valid;      // 0 = the last poll had no trustworthy sample
static uint32_t drift_rejects;    // polls that produced no sample at all
static uint32_t phase_rejects;    // samples the magnitude guard threw out
static uint32_t trip_streak;
static int32_t  phase_ppb;
static uint8_t  warm_started;
static int32_t  phase_setpoint;      // drift value the PLL holds
static uint8_t  setpoint_valid;
static uint32_t setpoint_arm_ms;
static uint32_t seen_anchors;
static int64_t  phase_integ;         // units of 1/PHASE_I_SCALE ppb
static uint32_t last_nv_ms;
// Phase term DEFAULTS OFF. See the block comment at PHASE_MAX_PPB.
static uint8_t  phase_enabled;

// 1 s level window, fed at 1 kHz.
static uint32_t win_start_ms;
static uint32_t lvl_sum, lvl_n;
static uint16_t lvl_min_cur = 0xffff, lvl_max_cur;
static uint16_t lvl_min, lvl_max, lvl_avg;

// ---- The NCO increment, and why it drifted -------------------------------
//
// THE +0.13 ppm RESIDUAL WAS THIS ARITHMETIC, not the servo. Measured
// 2026-08-12 at 0.13 ppm; predicted from the constants at 0.133 ppm.
//
// The increment is a 32-bit integer of about 4.12e6, so ONE LSB IS 242 ppb.
// Two roundings live inside it, both in the same direction:
//
//   base_inc     (fs << 32) / sys_clk = 4123168.604, ROUNDED UP to 4123169
//                                                          -> +96 ppb, forever
//   inc_for_ppb  truncates the ppb correction toward zero   -> +37 ppb
//                                                          --------------
//                                                              +133 ppb
//
// AND THE PTP SERVO CANNOT SEE ANY OF IT. g_ptpv1.rate_ppb measures this
// board's CRYSTAL against the Leader; it knows nothing about how coarsely we
// then render that estimate into an integer. So the rate loop reports itself
// converged, with 0 trips and a healthy ring, while the media clock runs
// 0.13 ppm fast and every receiver's measured latency walks down.
//
// HOW THE AVB mcr AVOIDS IT (firmware/mcr.c:219, the cs=0 pure-gPTP path):
//
//     d    = g->current_addend_full - g->base_addend_full;
//     corr = (base_increment * d) / g->base_addend_full;
//
// It takes the ratio from the 52-bit gPTP ADDEND (32-bit integer + 20-bit
// fraction) and recomputes it in mcr_watchdog_tick() at main-loop rate, ~4 kHz,
// behind a 2-unit deadband. The addend moves continuously, so the computed
// increment DITHERS ACROSS THE GRID POINT and the quantisation averages to
// zero. mcr_dante gave that up when it went to one slew-limited write per
// second: a single integer, written once, parked 0.55 LSB off for good.
//
// So do the averaging deliberately instead of hoping a servo supplies it.
// Carry the increment in 16.16 fixed point and first-order sigma-delta the
// fractional part: emit N or N+1 at a duty cycle equal to the fraction, so the
// MEAN increment is exact even though every written value is an integer.
//
// WHY THIS IS SAFE, when the gateware fractional NCO was not. Attempt 2 put the
// fraction in hardware and dithered STROBE TIMING at 50 MHz; the USB feedback
// servo measures strobes-per-SOF, so it saw the jitter and the ring starved
// (level 36, ~9700 underrun/s). Dithering here is a 1 Hz alternation between
// two adjacent rates 242 ppb apart. The phase excursion that can build inside
// one second is 242e-9 * 48000 = 0.012 SAMPLES against a 2048-sample ring, and
// the startup slew already moves the rate 100 ppb every second with zero
// underruns. The block comment at PHASE_MAX_PPB reached this same conclusion
// and called it out as the right way to do it.
//
// Cost: the NCO is now written about once a second instead of ~17 times total.
// Three orders of magnitude below the thousands-per-second path this module
// replaced, and it is the writes that make the average exact.
#define NCO_FRAC_BITS  16
#define NCO_FRAC_ONE   (1u << NCO_FRAC_BITS)

static int64_t  base_inc_q;        // exact (fs << 32) / sys_clk, 16.16, UNROUNDED
static uint32_t nco_dither_acc;    // sigma-delta accumulator, 0 .. NCO_FRAC_ONE-1

static int64_t inc_q_for_ppb(int32_t ppb)
{
    return base_inc_q + (base_inc_q * ppb) / 1000000000LL;
}

// Render a 16.16 increment to the integer the CSR takes, spreading the fraction
// over successive calls. MUST be called exactly once per applied update, or the
// accumulator advances out of step with what the NCO is actually running.
static uint32_t nco_render(int64_t inc_q)
{
    int64_t whole = inc_q >> NCO_FRAC_BITS;
    uint32_t frac = (uint32_t)(inc_q & (NCO_FRAC_ONE - 1));
    nco_dither_acc += frac;
    if (nco_dither_acc >= NCO_FRAC_ONE) {
        nco_dither_acc -= NCO_FRAC_ONE;
        whole++;
    }
    if (whole < 1) whole = 1;
    if (whole > 0xFFFFFFFFLL) whole = 0xFFFFFFFFLL;
    return (uint32_t)whole;
}

static uint32_t inc_for_ppb(int32_t ppb)
{
    return nco_render(inc_q_for_ppb(ppb));
}

void mcr_dante_init(uint32_t sys_clk_freq, uint32_t fs)
{
    // Same arithmetic as mcr_init(), deliberately duplicated rather than
    // reaching into mcr_state_t: this module owns the NCO now, and sharing
    // state with the thing it replaces is how multi-writer bugs come back.
    uint64_t inc = ((uint64_t)fs << 32) / sys_clk_freq;
    uint64_t rem = ((uint64_t)fs << 32) % sys_clk_freq;
    // KEEP THE REMAINDER. Rounding it away is +96 ppb of permanent rate error
    // that nothing downstream can observe or correct -- see the block comment at
    // inc_for_ppb(). Split rather than shifting the whole numerator by 48 bits,
    // which would come within 1.4x of overflowing uint64 at these constants.
    base_inc_q = ((int64_t)inc << NCO_FRAC_BITS)
               + (int64_t)((rem << NCO_FRAC_BITS) / sys_clk_freq);
    if (rem > (sys_clk_freq / 2))
        inc++;
    // base_inc stays ROUNDED: it is the "return to nominal" escape hatch and the
    // status readout, both of which want one stable integer, not a dithered one.
    base_inc        = (uint32_t)inc;
    applied_ppb     = 0;
    applied_inc     = base_inc;
    last_written_inc = base_inc;
    enabled         = 0;
    nco_writes      = 0;
    trips           = 0;
    last_underrun   = aaf_pkt_underrun_count_read();
    last_update_ms  = gptp_uptime_ms();
    win_start_ms    = last_update_ms;

    // WARM START. Without this every boot begins at nominal and slews to the
    // crystal correction at SLEW_PPB/s -- ~43 s at the measured -4300 ppb --
    // with the talker already running on a sample rate that is both wrong and
    // moving. That is the "bad audio for a while after boot" symptom. PTP has
    // warm-started its addend from NV since v3; this does the same for the
    // media clock so the first packet is emitted at within ppb of the right
    // rate.
    //
    // Sanity-clamped: a corrupt or wildly stale value must not start the clock
    // somewhere absurd. Anything beyond MAX_PPB is ignored and we cold-start.
    if (g_cfg.mclk_ppb_valid &&
        g_cfg.mclk_ppb < MAX_PPB && g_cfg.mclk_ppb > -MAX_PPB) {
        applied_ppb      = g_cfg.mclk_ppb;
        applied_inc      = inc_for_ppb(applied_ppb);
        last_written_inc = applied_inc;
        mcr_increment_write(applied_inc);
        warm_started = 1;
        // AUTO-ARM on a warm start. The saved rate is one this board already
        // converged on and ran audio with, so there is nothing to prove by
        // waiting: arming here means the very first packet leaves at the right
        // rate and the talker never has to be held off or dropped for a slew.
        // A cold boot deliberately does NOT auto-arm -- there is no trusted
        // rate to start from, so it stays manual (tools/mclk.py on).
        enabled = 1;
        printf("[mclk] WARM START %ld ppb from NV, ARMED (inc %lu)\n",
               (long)applied_ppb, (unsigned long)applied_inc);
    } else {
        mcr_increment_write(base_inc);
        printf("[mclk] cold start at nominal (no NV rate)\n");
    }

    printf("[mclk] one owner, rate-only, slew %d ppb/s. base_inc=%lu "
           "DISABLED (dry run)\n", SLEW_PPB, (unsigned long)base_inc);
}

void mcr_dante_level_sample(uint32_t level)
{
    if (level < lvl_min_cur) lvl_min_cur = (uint16_t)level;
    if (level > lvl_max_cur) lvl_max_cur = (uint16_t)level;
    lvl_sum += level;
    lvl_n++;
}

void mcr_dante_set_enabled(int on)
{
    enabled = on ? 1 : 0;
    if (!enabled) {
        // Return to nominal. Deliberately a STEP back rather than a slew: this
        // is the escape hatch, and it must be immediate and unconditional.
        applied_ppb      = 0;
        applied_inc      = base_inc;
        last_written_inc = base_inc;
        mcr_increment_write(base_inc);
        nco_writes++;
        telem_event(TELEM_E_MCLK_DISARM, (int32_t)base_inc, 0);
        printf("[mclk] DISABLED, NCO restored to nominal %lu\n",
               (unsigned long)base_inc);
    } else {
        // Start the slew from where the NCO ACTUALLY is. After a warm start
        // that is the persisted rate, not nominal -- resetting to 0 here would
        // throw the warm start away and re-introduce the 43 s ramp.
        applied_inc      = inc_for_ppb(applied_ppb);
        last_written_inc = applied_inc;
        mcr_increment_write(applied_inc);
        telem_event(TELEM_E_MCLK_ARM, target_ppb, SLEW_PPB);
        printf("[mclk] ENABLED, slewing 0 -> %ld ppb at %d ppb/s (~%lu s)\n",
               (long)target_ppb, SLEW_PPB,
               (unsigned long)((target_ppb < 0 ? -target_ppb : target_ppb)
                               / SLEW_PPB));
    }
}

void mcr_dante_poll(void)
{
    uint32_t now = gptp_uptime_ms();
    if ((uint32_t)(now - last_update_ms) < UPDATE_MS)
        return;
    uint32_t elapsed = now - last_update_ms;
    last_update_ms = now;

    // ---- Close the level window ----
    if (lvl_n) {
        lvl_min = lvl_min_cur;
        lvl_max = lvl_max_cur;
        lvl_avg = (uint16_t)(lvl_sum / lvl_n);
    }
    lvl_min_cur = 0xffff; lvl_max_cur = 0; lvl_sum = 0; lvl_n = 0;
    (void)win_start_ms;

    // ---- Underrun rate ----
    uint32_t u = aaf_pkt_underrun_count_read();
    uint32_t du = u - last_underrun;              // wraps correctly
    last_underrun = u;
    underrun_per_s = elapsed ? (uint32_t)(((uint64_t)du * 1000u) / elapsed) : 0;

    // ---- Drift, for the dry-run cross-check AND the phase loop ----
    //
    // Works with no USB attached: the packetizer keeps emitting (silence) at
    // the media rate, so the media clock still tracks the NCO. It does NOT work
    // with the talker off, and that distinction is the whole point of
    // dante_tx_media_phase() returning a validity flag rather than a number --
    // read its comment before touching this. The previous version computed
    // (emitted timestamp - PTP) from a register that freezes when the talker
    // stops, so a talker-off stretch was reported as a drift walking at
    // -48000 samples/s, which is what railed the phase integral on 2026-08-11.
    //
    // NOTE THE CHANGE OF REFERENCE: this is now the media clock COUNTER against
    // PTP, not the emitted timestamp. They differ by a constant (fpp-1) --
    // 3 samples at the A16R's fpp=4, 59 at DVS's fpp=60 -- so absolute readings
    // shift slightly against anything recorded before this commit. The rate
    // (`mclk.py watch`) is unaffected, and the counter is the right control
    // variable: it does not step when a receiver rebinds at a different fpp.
    {
        int32_t d;
        if (dante_tx_media_phase(&d)) {
            drift_samples = d;
            drift_valid   = 1;
        } else {
            drift_valid = 0;
            drift_rejects++;
        }
    }

    // ---- Auto-disable ----
    // A 100%-duty underrun is NOT a clock fault -- it is an empty ring, i.e.
    // no USB source. A clock that is disturbing the ring causes PARTIAL
    // underrun: brief repeated dips as the level crosses the prime floor.
    //
    // The old test used lvl_max >= 32, which fired spuriously on 2026-08-04
    // when the USB host stopped: block_level reports a constant 64 while the
    // talker is disabled (dante_packetizer.py "report centre while talker
    // off"), so the talker-off window looked like an active ring, and the
    // guard reverted a perfectly good warm-started rate. Duty cycle is the
    // discriminator that actually separates the two cases.
    // REQUIRE PERSISTENCE. The window between "ring empty" (48000/s) and "ring
    // healthy" (0/s) is traversed EVERY TIME the USB host starts streaming, and
    // on the way through it passes exactly through this band with the level
    // already climbing past RING_ACTIVE_LEVEL. Starting playback on the MacBook
    // therefore tripped the guard and reverted the rate correction, after which
    // the media clock free-ran at the crystal's ~4.4 ppm -- about 13 samples per
    // minute, crossing a receiver's entire 1 ms budget in under four minutes.
    //
    // Observed 2026-08-05: trips=1, applied +0 ppb against a target of
    // -4408 ppb, and the emitted timestamp walking out of every latency window.
    // This is the THIRD spurious trip of this guard (previously: lvl_max >= 32
    // matching the talker-off constant 64), and each time the cause was a
    // transient state rather than a clock fault.
    //
    // A real clock fault is persistent by nature; a startup transient lasts a
    // second or two. Requiring several consecutive seconds separates them
    // without weakening the guard against what it is actually for.
    if (enabled &&
        underrun_per_s > UNDERRUN_TRIP_PER_S &&
        underrun_per_s < UNDERRUN_NO_SOURCE_PER_S &&
        lvl_max >= RING_ACTIVE_LEVEL)
        trip_streak++;
    else
        trip_streak = 0;

    if (trip_streak >= TRIP_STREAK_S) {
        trip_streak = 0;
        trips++;
        telem_event(TELEM_E_MCLK_TRIP, (int32_t)underrun_per_s, lvl_min);
        printf("[mclk] TRIP: underrun %lu/s with ring active (level %u..%u) "
               "-- reverting to nominal\n",
               (unsigned long)underrun_per_s, lvl_min, lvl_max);
        mcr_dante_set_enabled(0);
        return;
    }

    // ---- Rate estimate ----
    // rate_ppb is the PTPv1 servo INTEGRAL alone (ptpv1.h:35-39): the smooth
    // estimate of how fast this board's crystal runs against the Leader. The
    // proportional term is phase correction and must never reach an audio
    // sample rate. Both clocks derive from sys_clk, so the same ppb that
    // corrects the TSU addend corrects the media NCO, with the same sign.
    // ---- Phase term ----
    // drift_samples was computed above: emitted media timestamp minus PTP.
    // Positive means we are running AHEAD, so the clock must be slowed.
    // ---- PLL ----
    // Re-latch the setpoint after any anchor: the anchor moves absolute phase,
    // so the old setpoint no longer describes where we want to sit.
    if (g_tx_stats.anchors != seen_anchors) {
        seen_anchors    = g_tx_stats.anchors;
        setpoint_valid  = 0;
        setpoint_arm_ms = now + PHASE_SETPOINT_SETTLE_MS;
        phase_integ     = 0;
    }
    if (!setpoint_valid && dante_tx_enabled() && g_ptpv1.locked && drift_valid &&
        (int32_t)(now - setpoint_arm_ms) >= 0) {
        phase_setpoint = PHASE_TARGET_SAMPLES;
        setpoint_valid = 1;
        printf("[mclk] phase target %ld samples (currently %ld, err %ld)\n",
               (long)phase_setpoint, (long)drift_samples,
               (long)(drift_samples - phase_setpoint));
    }

    if (!phase_enabled || !setpoint_valid || !g_ptpv1.locked) {
        phase_ppb = 0;
    } else if (!drift_valid) {
        // NO SAMPLE THIS POLL. Hold the whole term where it is -- do not wind
        // the integral on a stale value, and do not zero the output either.
        // Zeroing would step the applied rate by however much the integral had
        // accumulated, and a rate step underneath the USB feedback servo is the
        // failure mode this module exists to avoid (see mcr_dante.h).
        //
        // Holding is safe for as long as this lasts, because the only way to
        // lose the sample is the talker being off or the media strobe being
        // stopped -- in both cases nothing is being emitted, so the phase we
        // are holding a correction for is not reaching any receiver. When the
        // talker comes back it anchors, anchors != seen_anchors fires above,
        // and the setpoint and integral are reset from scratch.
        goto phase_done;
    } else {
        // err > 0 means we are AHEAD of where we should be, so slow down.
        int32_t err = drift_samples - phase_setpoint;

        // MAGNITUDE GUARD, kept as a second net.
        //
        // This was the emergency stop after the 2026-08-11 runaway, when
        // drift_samples could be whole seconds out (-47959, then -335977).
        // dante_tx_media_phase() now makes a whole-second error structurally
        // impossible -- it never reads a seconds field on either side, and it
        // refuses to report at all when the media counter is not ticking -- so
        // this should no longer have anything to reject. It stays because it
        // costs one comparison and because the thing it protects against was
        // expensive: an integral railed at 2000 ppb, ten re-anchors, and audio
        // interrupted. If it ever fires now, the measurement is wrong again and
        // the phase term must NOT be trusted until that is understood.
        //
        // 2400 samples is 50 ms, far beyond anything the re-anchor backstop
        // (8 samples) would ever allow to accumulate, so a reading past it is
        // a sampling artefact and not a real phase error. Skip the update
        // entirely rather than clamping: a clamped outlier still winds the
        // integral in the wrong direction.
        //
        // NOTE THIS ZEROES WHERE THE no-sample PATH ABOVE HOLDS, and the
        // difference is deliberate. "No sample" means the talker is off and
        // nothing is being emitted, so the correction is not reaching anyone
        // and holding it costs nothing. Getting here instead means a sample
        // ARRIVED AND WAS NONSENSE -- the measurement is suspect, so fail back
        // toward rate-only, which is the known-good configuration. If it keeps
        // firing the term slews out over 20 s and the clock is left exactly as
        // it behaves with the phase term switched off. The integral is left
        // standing so a single bad reading costs one poll, not the acquisition.
        //
        // sims/sim_mclk_phase_trim.py check 6 models this: the guard alone is
        // NOT sufficient (it passes a stale reading for the first 50 ms of a
        // freeze, 6 landings in 100, worth 687 ppb of invented correction),
        // which is why dante_tx_media_phase() exists rather than a bigger guard.
        if (err > 2400 || err < -2400) {
            phase_rejects++;
            phase_ppb = 0;
            goto phase_done;
        }

        phase_integ += -(int64_t)err * PHASE_KI;
        int64_t ilim = (int64_t)PHASE_MAX_PPB * PHASE_I_SCALE;
        if (phase_integ >  ilim) phase_integ =  ilim;
        if (phase_integ < -ilim) phase_integ = -ilim;

        int32_t p = -(err * PHASE_KP_NUM) / PHASE_KP_DEN;
        int32_t i = (int32_t)(phase_integ / PHASE_I_SCALE);
        phase_ppb = p + i;
        if (phase_ppb >  PHASE_MAX_PPB) phase_ppb =  PHASE_MAX_PPB;
        if (phase_ppb < -PHASE_MAX_PPB) phase_ppb = -PHASE_MAX_PPB;
    }
phase_done:

    target_ppb = g_ptpv1.rate_ppb + phase_ppb;
    if (target_ppb >  MAX_PPB) target_ppb =  MAX_PPB;
    if (target_ppb < -MAX_PPB) target_ppb = -MAX_PPB;

    // Hold the last applied rate when PTP is not locked. Do NOT snap back to
    // nominal: a momentary unlock is not evidence the crystal changed, and
    // snapping is exactly what the AVB watchdog did.
    if (!g_ptpv1.locked)
        return;

    // applied_ppb must always describe WHAT THE NCO IS ACTUALLY SET TO, not
    // where we would like it to be. While disabled the NCO is at nominal, so
    // applied_ppb stays 0 and the slew starts from the hardware's real state
    // when armed.
    //
    // Getting this wrong is subtle and was caught on the bench: an earlier
    // version slewed during the dry run too, so by the time the operator armed
    // it, applied_ppb had already reached the target and the first write was
    // the WHOLE 4.3 ppm correction in one step -- precisely the rate step the
    // slew limiter exists to prevent, delivered by the safety mechanism itself.
    if (!enabled) {
        applied_ppb = 0;
        applied_inc = base_inc;
        return;
    }

    int32_t d = target_ppb - applied_ppb;
    if (d >  SLEW_PPB) d =  SLEW_PPB;
    if (d < -SLEW_PPB) d = -SLEW_PPB;
    applied_ppb += d;

    applied_inc = inc_for_ppb(applied_ppb);

    if (enabled && applied_inc != last_written_inc) {
        last_written_inc = applied_inc;
        mcr_increment_write(applied_inc);
        nco_writes++;
    }

    // Persist the converged rate for the next boot, but rarely: this is a
    // flash sector erase/write, and doing it on every small change would wear
    // the part and stall the main loop. Only once converged (slew has caught
    // up with the target) and only when it has actually moved.
    // The FIRST save is not time-gated: a cold-started board must persist its
    // rate as soon as it converges, or the very next boot is cold again and
    // the whole warm-start mechanism never engages. Later saves are rate
    // limited, because each one is a flash sector erase.
    if (enabled && applied_ppb == target_ppb &&
        (!g_cfg.mclk_ppb_valid || (uint32_t)(now - last_nv_ms) >= NV_SAVE_MS)) {
        last_nv_ms = now;
        int32_t d = g_cfg.mclk_ppb - applied_ppb;
        if (!g_cfg.mclk_ppb_valid || d > NV_SAVE_DELTA_PPB || d < -NV_SAVE_DELTA_PPB) {
            g_cfg.mclk_ppb_valid = 1;
            g_cfg.mclk_ppb       = applied_ppb;
            cfg_save();
            printf("[mclk] rate %ld ppb saved for warm start\n", (long)applied_ppb);
        }
    }
}

void mcr_dante_set_phase_enabled(int on)
{
    phase_enabled = on ? 1 : 0;
    if (!phase_enabled) phase_ppb = 0;
    printf("[mclk] phase term %s\n", phase_enabled ? "ENABLED (experimental)"
                                                    : "disabled");
}

int mcr_dante_rate_ready(void)
{
    // Not disciplining at all: the clock is whatever the crystal does, which is
    // steady even if it is wrong. Nothing to wait for.
    if (!enabled)
        return 1;
    if (!g_ptpv1.locked)
        return 0;
    // Converged when the slew limiter has caught the target. A warm start
    // arrives here on the first poll instead of ~43 s later.
    int32_t d = target_ppb - applied_ppb;
    return (d < RATE_READY_PPB && d > -RATE_READY_PPB);
}

void mcr_dante_get_status(mcr_dante_status_t *out)
{
    out->phase_enabled  = phase_enabled;
    out->phase_ppb      = phase_ppb;
    out->enabled        = enabled;
    out->ptp_locked     = g_ptpv1.locked;
    out->target_ppb     = target_ppb;
    out->applied_ppb    = applied_ppb;
    out->base_inc       = base_inc;
    out->applied_inc    = applied_inc;
    out->nco_writes     = nco_writes;
    out->trips          = trips;
    out->lvl_min        = lvl_min;
    out->lvl_max        = lvl_max;
    out->lvl_avg        = lvl_avg;
    out->underrun_per_s = underrun_per_s;
    out->drift_samples  = drift_samples;
    out->drift_valid    = drift_valid;
    out->drift_rejects  = drift_rejects;
    out->phase_rejects  = phase_rejects;
}
