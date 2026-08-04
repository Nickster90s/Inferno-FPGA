// mcr_dante.c — the Dante media clock. See mcr_dante.h for why.

#include "mcr_dante.h"
#include "ptpv1.h"
#include "gptp.h"

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

// ---- Phase ----
//
// DISABLED BY DEFAULT, because the first version of it broke audio.
//
// It was pure-proportional with a deadband, and its output fed the rate slew
// limiter -- i.e. the lag was INSIDE the loop. That is a textbook limit cycle:
// on the bench the phase hunted -27 -> +216 samples (a 5 ms excursion, target
// +74) while modulating the media clock rate by roughly +/-0.5 ppm on a slow
// period. Receivers locking to those timestamps reported clock problems, and
// the operator heard them. Bisected 2026-08-04 by disabling the discipline:
// audio went clean immediately.
//
// Rate discipline alone is validated and safe (see the audio test above), so
// the two are now separable: `mclk on` gives rate only. The phase term stays
// off until it is redesigned with damping -- a PI whose integrator is not
// behind the slew limiter, much lower gain, and telemetry that can actually
// show it oscillating. Do not re-enable it on a hunch.
//
// RATE AND PHASE ARE SEPARATE STATE. Correcting the rate stops the error
// growing; it never brings back what has already accumulated. Missing this is
// what produced silence on 2026-08-04: rate discipline was working (+0.53 ppm)
// while the emitted timestamp sat 227 ms in the future from two days of
// free-running drift, and every receiver discarded the audio.
//
// So a small phase term is pulled in alongside the rate. It is deliberately
// FEEBLE: clamped to +/-2000 ppb, which is under half the crystal error the
// rate term already handles, so it can never fight the ring or the USB servo.
// At the clamp it closes 1 ms of phase in ~8 minutes -- far too slow to rescue
// a large accumulated error (that is the re-anchor backstop's job in
// dante_tx.c) but ample to stop one ever forming.
#define PHASE_MAX_PPB        2000

// Proportional gain: ppb per sample of phase error. 4 ppb/sample reaches the
// clamp at 500 samples (~10 ms) of error and is proportional below that, so
// small errors get gentle correction and large ones saturate rather than
// producing a step.
#define PHASE_KP_PPB_PER_SAMPLE  4

// Deadband, in samples. Below this the phase is left alone: PTP's own offset
// noise is hundreds of ns and chasing it would modulate the audio rate for no
// benefit. 48 samples = 1 ms.
#define PHASE_DEADBAND_SAMPLES   48

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

    mcr_increment_write(base_inc);

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
        printf("[mclk] DISABLED, NCO restored to nominal %lu\n",
               (unsigned long)base_inc);
    } else {
        // Start the slew from nominal: that is where the NCO actually is.
        applied_ppb      = 0;
        applied_inc      = base_inc;
        last_written_inc = base_inc;
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
        int64_t emit    = (int64_t)aaf_pkt_dbg_last_sec_read() * 48000
                        + (int64_t)aaf_pkt_dbg_last_ts_read();
        drift_samples = (int32_t)(emit - ptp_smp);
    }

    // ---- Auto-disable ----
    if (enabled && lvl_max >= RING_ACTIVE_LEVEL &&
        underrun_per_s > UNDERRUN_TRIP_PER_S) {
        trips++;
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
    if (!phase_enabled) {
        phase_ppb = 0;
    } else {
        int32_t perr = drift_samples - 74;      // DANTE_TX_TS_OFFSET
        if (perr > PHASE_DEADBAND_SAMPLES || perr < -PHASE_DEADBAND_SAMPLES) {
            phase_ppb = -perr * PHASE_KP_PPB_PER_SAMPLE;
            if (phase_ppb >  PHASE_MAX_PPB) phase_ppb =  PHASE_MAX_PPB;
            if (phase_ppb < -PHASE_MAX_PPB) phase_ppb = -PHASE_MAX_PPB;
        } else {
            phase_ppb = 0;
        }
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
}

void mcr_dante_set_phase_enabled(int on)
{
    phase_enabled = on ? 1 : 0;
    if (!phase_enabled) phase_ppb = 0;
    printf("[mclk] phase term %s\n", phase_enabled ? "ENABLED (experimental)"
                                                    : "disabled");
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
