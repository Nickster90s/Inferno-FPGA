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
#define PHASE_TARGET_SAMPLES     (-24)   // inferno CLOCK_OFFSET_NS = -500 us
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
#define PHASE_KI                 40
#define PHASE_I_SCALE            1024

// Proportional gain (ppb per sample), as a fraction. Damping only -- kept small
// so a transient cannot jerk the rate. 48 samples -> ~19 ppb.
#define PHASE_KP_NUM             2
#define PHASE_KP_DEN             5

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

static uint32_t inc_for_ppb(int32_t ppb)
{
    int64_t inc = (int64_t)base_inc + ((int64_t)base_inc * ppb) / 1000000000LL;
    if (inc < 1) inc = 1;
    if (inc > 0xFFFFFFFFLL) inc = 0xFFFFFFFFLL;
    return (uint32_t)inc;
}

void mcr_dante_init(uint32_t sys_clk_freq, uint32_t fs)
{
    // Same arithmetic as mcr_init(), deliberately duplicated rather than
    // reaching into mcr_state_t: this module owns the NCO now, and sharing
    // state with the thing it replaces is how multi-writer bugs come back.
    uint64_t inc = ((uint64_t)fs << 32) / sys_clk_freq;
    if ((((uint64_t)fs << 32) % sys_clk_freq) > (sys_clk_freq / 2))
        inc++;
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

    // ---- Drift, for the dry-run cross-check ----
    // Works with no USB attached: the packetizer keeps emitting (silence) at
    // the media rate, so the emitted timestamp still tracks the NCO.
    {
        ptp_timestamp_t t = gptp_read_time();
        int64_t ptp_smp = (int64_t)t.seconds * 48000
                        + (int64_t)((t.nanoseconds * 3u) / 62500u);
        uint32_t esec, esub;
        dante_tx_read_emitted(&esec, &esub);   // atomic pair; see dante_tx.h
        int64_t emit    = (int64_t)esec * 48000 + (int64_t)esub;
        drift_samples = (int32_t)(emit - ptp_smp);
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
    if (enabled &&
        underrun_per_s > UNDERRUN_TRIP_PER_S &&
        underrun_per_s < UNDERRUN_NO_SOURCE_PER_S &&
        lvl_max >= RING_ACTIVE_LEVEL) {
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
    if (!setpoint_valid && dante_tx_enabled() && g_ptpv1.locked &&
        (int32_t)(now - setpoint_arm_ms) >= 0) {
        phase_setpoint = PHASE_TARGET_SAMPLES;
        setpoint_valid = 1;
        printf("[mclk] phase target %ld samples (currently %ld, err %ld)\n",
               (long)phase_setpoint, (long)drift_samples,
               (long)(drift_samples - phase_setpoint));
    }

    if (!phase_enabled || !setpoint_valid || !g_ptpv1.locked) {
        phase_ppb = 0;
    } else {
        // err > 0 means we are AHEAD of where we should be, so slow down.
        int32_t err = drift_samples - phase_setpoint;

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
}
