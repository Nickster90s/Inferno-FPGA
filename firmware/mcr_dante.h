// mcr_dante.h — the Dante media clock. One owner, rate only, slew-limited.
//
// Replaces mcr.c's ownership of the NCO. mcr.c stays linked for the NCO
// arithmetic and the USB feedback scaling constants, but it must NOT write the
// increment any more: main.c stops calling mcr_watchdog_tick().
//
// See MCR_REPLACEMENT.md. The short version of why this exists:
//
//   * mcr is AVB machinery. Under Dante there is no CRF, so its PI servo never
//     receives data and its only LIVE effect was the CRF-loss watchdog's
//     fallback branch (mcr.c:267) rewriting the NCO on EVERY main-loop
//     iteration (~4 kHz) with a 2-unit (~0.5 ppm) deadband. An AVB stale-stream
//     watchdog was setting the Dante sample rate.
//   * The ring already has a controller. The USB wrapper's async feedback is
//     not a pure rate report -- it is a PI servo on ring level with setpoint 64
//     (traced in rtl/usb_avb_subsystem.v: err = 64 - block_level, integ += err,
//     P gain err<<6, updated every SOF). Disciplining the NCO therefore adds a
//     SECOND controller to one buffer.
//   * That is why both previous attempts failed. Following the full addend
//     (rate + phase) and following the integral alone BOTH produced underrun
//     storms, because in both cases the NCO was being stepped at main-loop rate
//     underneath a servo correcting at SOF rate. The missing constraint was
//     never WHICH rate estimate -- it was HOW FAST it may change.
//
// So: follow g_ptpv1.rate_ppb (the servo integral, i.e. rate without phase),
// update at 1 Hz, and slew-limit it so the USB servo sees a quasi-static
// change it can absorb as an ordinary disturbance.

#ifndef MCR_DANTE_H
#define MCR_DANTE_H

#include <stdint.h>

// Latch the nominal increment and write it once. Leaves discipline DISABLED:
// the module computes and reports what it WOULD write, so the sign and
// magnitude can be checked against independently measured drift before
// anything touches the clock.
void mcr_dante_init(uint32_t sys_clk_freq, uint32_t fs);

// Main loop. Internally rate-limited to one update per second.
void mcr_dante_poll(void);

// Call from the existing 1 kHz tick in main.c with aaf_pkt_fifo_level_read().
// Tracks min/max/mean over a 1 s window -- the instrument that was missing.
// A 1 Hz SNAPSHOT of the ring level is what made the last two attempts
// unreadable: underrun_count ticks at 48 kHz while un-primed, so 0.35% duty
// (40547 ticks in 4 min) is a real, repeated dip that a slow snapshot lands in
// almost never. Min is the number that matters, not mean.
void mcr_dante_level_sample(uint32_t level);

// Arm or disarm the discipline. Disarming restores the nominal increment.
void mcr_dante_set_enabled(int on);

void mcr_dante_set_phase_enabled(int on);

// 1 when the media clock is fit to start a stream on: the rate has converged on
// the PTP estimate (or was warm-started onto it) rather than still slewing.
// dante_tx gates the talker on this so a stream never begins on a sample rate
// that is still moving -- that ramp is what made the first ~45 s sound wrong.
int mcr_dante_rate_ready(void);

typedef struct {
    uint8_t  enabled;
    uint8_t  phase_enabled;
    int32_t  phase_ppb;
    uint8_t  ptp_locked;
    int32_t  target_ppb;        // g_ptpv1.rate_ppb -- where we want to be
    int32_t  applied_ppb;       // where the slew limiter has actually got to
    uint32_t base_inc;          // nominal NCO increment
    uint32_t applied_inc;       // increment implied by applied_ppb
    uint32_t nco_writes;        // times the CSR was actually written
    uint32_t trips;             // auto-disables on underrun
    uint16_t lvl_min;           // ring level over the last 1 s window
    uint16_t lvl_max;
    uint16_t lvl_avg;
    uint32_t underrun_per_s;    // media ticks spent un-primed, per second
    int32_t  drift_samples;     // emitted media timestamp - PTP, in samples
} mcr_dante_status_t;

void mcr_dante_get_status(mcr_dante_status_t *out);

#endif // MCR_DANTE_H
