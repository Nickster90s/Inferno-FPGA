// PTPv1 (IEEE 1588-2002) slave — Dante Phase 4. See ptpv1.h.
//
// Wire format CONFIRMED byte for byte against a Focusrite RedNet A16R on the
// bench (captures/README.md), not merely taken from a reference implementation:
//
//   transport   UDP multicast 224.0.1.129, event 319, general 320
//   IP          TOS 0xE0 (DSCP CS7), TTL 1
//   header      40 bytes, big-endian
//     [0..2]    version_ptp = 1
//     [2..4]    version_network = 1
//     [4..20]   subdomain[16] = "_DFLT"
//     [20]      port_type (1 = Event for Sync/DelayReq, else General)
//     [21]      source_communication_technology = 1
//     [22..28]  source_uuid = sender MAC
//     [28..30]  source_port_id
//     [30..32]  sequence_id
//     [32]      control
//     [35]      flags; bit3 = ptp_assist (two-step -> a FollowUp will follow)
//   Sync body   originTimestamp at [40..48] = u32 seconds + u32 nanoseconds
//               (NOT PTPv2's 6+4 layout)
//
// The A16R's Sync carries flags 0x0c: boundary clock + two-step, so the precise
// origin time arrives in a separate FollowUp.
//
// Servo: reuses the PI loop shape and gains that got gPTP to +-30 ns on this
// hardware, acting on the same 52-bit TSU addend. Two things are deliberately
// different from gPTP, both forced by the protocol rather than by choice:
//
//  1. PTPv1 has NO correctionField. 802.1AS transparent clocks add switch
//     residence time; PTPv1 has no equivalent, so queueing delay lands directly
//     in the offset measurement. Expect hundreds of ns to low microseconds
//     through a switch, not tens of ns.
//  2. Sync arrives at ~4 Hz here rather than 8 Hz, so the integral gain is
//     scaled accordingly and the median filter is widened.

#include "ptpv1.h"
#include "telem.h"
#include "config.h"
#include "net.h"
#include "dante_dev.h"
#include <generated/soc.h>
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

#define PTP_EVENT_PORT      319
#define PTP_GENERAL_PORT    320

#define CTRL_SYNC           0
#define CTRL_DELAY_REQ      1
#define CTRL_FOLLOWUP       2
#define CTRL_DELAY_RESP     3

#define PORT_TYPE_EVENT     1
#define PORT_TYPE_GENERAL   2

#define HDR_LEN             40
#define SYNC_BODY_LEN       84
#define FOLLOWUP_BODY_LEN   12
#define DELAY_RESP_BODY_LEN 20

#define FLAG_PTP_ASSIST     (1u << 3)

// DelayReq cadence. IEEE 1588 randomises this to avoid synchronised bursts from
// many slaves; with one device on the bench a fixed period is fine and easier to
// reason about. Slower than Sync on purpose -- path delay changes far more
// slowly than offset.
#define DELAY_REQ_MS        4000
#define DELAY_REQ_FAST_MS   1000         // until the path delay is measured

// Servo. Same shape as gptp.c's, retuned for the lower Sync rate.
#define SERVO_KP_NUM        72
#define SERVO_KP_DEN        1000
#define SERVO_KP_FAST_NUM   200          // when |offset| > 1 us
#define SERVO_KI_NUM        900          // gPTP used 3600 at 8 Hz; /4 for ~2 Hz
#define SERVO_KI_DEN        1000000

// Frequency ACQUISITION by direct measurement, not by winding up the integral.
//
// Lock used to take 3-5 minutes. The cause was structural: proportional action
// alone settles at a standing error of offset = crystal_offset / KP. With our
// crystal at ~-4640 ppb and KP = 0.2 that is 23 us -- exactly the 17..35 us
// plateau we sat on while the frequency looked perfectly stable. Only the
// integral could remove it, and at 900/1e6 with ~4 Hz Sync that needs ~64 s
// just to wind up to -4640.
//
// Raising the gains was tried and MADE IT WORSE: KP 0.5 / KI 0.02 rang badly
// (+705668, -190902, -440991 ppb) and had not settled after 97 s. The 7-deep
// median delays the measurement by ~1.75 s, and no gain choice both survives
// that delay and converges in seconds.
//
// So measure the crystal offset instead of servoing to it. Over a window, the
// master's elapsed time and ours differ by exactly our frequency error:
//
//     err_ppb = ((local_elapsed - master_elapsed) * 1e9) / master_elapsed
//
// That is a measurement, not a control action -- no loop dynamics, nothing to
// destabilise. An 8 s window with ~1 us timestamps resolves ~125 ppb, which
// leaves the servo a residual it can close with the gentle tracking gains that
// were already proven on this hardware.
//
// The addend is deliberately left UNTOUCHED during the window: the estimate is
// only valid if our rate is constant across it.
#define ACQ_WINDOW_NS       8000000000LL

// How long to wait for a first path-delay measurement before locking without
// one. Four DelayReq cadences: long enough that a healthy link always measures
// it first, short enough that a device whose DelayResps never match still
// becomes usable.
#define PATH_DELAY_GRACE_MS 20000

// Accepted path-delay measurements required before the estimate is trusted
// enough to lock against. One is not enough -- see the note in the DelayResp
// handler.
#define PD_MIN_SAMPLES      4

// Re-step rather than steer while unlocked and further out than this. Chosen
// just above the 2 us lock threshold so it closes the gap to lock in one go but
// never fires against normal in-lock jitter.
#define SERVO_INTEGRAL_MAX  100000000LL  // +-100 ms
#define SERVO_STEP_NS       500000000LL  // step rather than slew beyond 500 ms
// Lock thresholds, sized for PTPv1 on this path rather than inherited.
//
// These were 500/2000, carried over from gPTP -- which had 802.1AS transparent
// clocks contributing a correctionField for switch residence time. PTPv1 has NO
// correctionField, so switch queueing and path asymmetry land directly in the
// offset. On this bench the path measures ~20.9 us with real asymmetry, and the
// servo settles at a rock-steady 785 ns: 56 consecutive samples spanning
// 754..802 ns, i.e. +-18 ns peak to peak.
//
// That is a good clock refusing to admit it. 785 ns of standing offset with
// +-18 ns of noise is far better than the "hundreds of ns to low microseconds"
// the plan predicted for PTPv1 through a switch, and the STABILITY is what
// audio depends on -- a constant offset shifts every sample equally, while
// jitter is what actually corrupts a stream. 785 ns is 0.04 samples at 48 kHz.
//
// Raising the threshold is the honest fix here, not more servo tuning: no gain
// choice removes a term the protocol cannot measure.
// Thresholds are 2000/5000 and stay there. A 2026-08-04 attempt to widen them
// to 6000/15000 was based on a misdiagnosis: the offset noise had risen to
// 3-7 us and looked like an irreducible floor, but the actual cause was rx_gate
// sitting DISABLED after a reboot, putting 21% control-frame loss back on the
// PTP path. With rx_gate armed the offset converges below 1 us and these
// thresholds are comfortable. Do not widen them to chase noise -- check that
// rx_gate is armed first.
#define LOCK_THRESHOLD_NS   2000
#define UNLOCK_THRESHOLD_NS 5000
#define LOCK_STREAK         8
// Consecutive out-of-band samples required before declaring loss of lock.
#define UNLOCK_STREAK       4

// Median filter. Widened from gPTP's 5 because each sample is ~4x more
// expensive to acquire at this Sync rate.
#define MEDIAN_N            7

static const uint8_t ptp_group[4] = {224, 0, 1, 129};

ptpv1_state_t g_ptpv1;

static uint8_t  our_uuid[6];
static uint16_t our_port_id = 1;
static uint16_t delay_req_seq;
static uint32_t next_delay_req_ms;

// t1 = master Sync origin, t2 = our RX of Sync,
// t3 = our DelayReq TX,     t4 = master RX of DelayReq.
static ptp_timestamp_t t1, t2, t3, t4;
static uint8_t         have_t1, have_t2, have_t3;
static uint16_t        pending_sync_seq;
static uint8_t         awaiting_followup;

static int64_t  median_buf[MEDIAN_N];
static uint8_t  median_count, median_pos;

// RUNTIME-TUNABLE SERVO PARAMETERS.
//
// Exists so a variant can be A/B'd without a reflash. Reflashing costs a
// reboot, which re-locks PTP, re-anchors the media clock and restarts the
// talker -- all of which inject exactly the transients a servo comparison is
// trying to measure. It also makes INTERLEAVED runs practical, which is what
// distinguishes a real ranking from run-to-run network noise: a single capture
// per variant was measured to vary ~3x on identical code.
static uint8_t  rt_median_n     = MEDIAN_N;
static int32_t  rt_ki_num       = SERVO_KI_NUM;
// EXACT INTEGRAL IS NOW THE DEFAULT. Established by a 5-run interleaved matrix
// (5 variants x 5 rounds, runtime-switched, no reboots), 2026-08-04:
//
//   variant             off sd            |mean| standing     rate sd
//   A med7 ki900   108 [46..203]     187 [120..535] ns   0.0 [0.0..0.0]
//   B med1 ki900    53 [39..124]       318 [9..764]      0.0 [0.0..0.0]
//   C med1 ki3600   59 [46..157]        36 [8..182]      0.0 [0.0..21.7]
//   D med7 ki3600   92 [38..173]       77 [22..143]      5.0 [0.0..14.0]
//   E med7 ki900 EX 75 [48..139]         27 [4..90]      6.4 [5.2..14.6]
//
// Offset NOISE is indistinguishable across all five -- every range overlaps
// every other, so neither the median width nor KI measurably affects jitter.
// An earlier single-capture-per-variant table appeared to show 5x differences;
// that was entirely run-to-run variance, and it is why this was redone with
// interleaved repeats.
//
// The STANDING OFFSET does separate, and A vs E do not overlap at all
// ([120..535] vs [4..90]). E is 7x better on the median with a tighter spread,
// at the SAME low gain -- it simply stops discarding the remainder. rate sd
// going 0.0 -> 6.4 ppb is the loop actually closing: under the old form the
// integral was frozen for any offset below 1111 ns and could not track real
// drift at all.
//
// 0 unlocks and 0 underruns across all 25 runs.
static uint8_t  rt_exact_integ = 1;
static int64_t  freq_integral_num;       // exact accumulator, when enabled



static int64_t  freq_integral;

void ptpv1_set_tuning(uint8_t median_n, int32_t ki_num, uint8_t exact)
{
    if (median_n < 1) median_n = 1;
    if (median_n > MEDIAN_N) median_n = MEDIAN_N;
    rt_median_n = median_n;
    rt_ki_num   = ki_num;
    rt_exact_integ = exact ? 1 : 0;
    // Reset the filter so the new width does not inherit a half-full buffer of
    // the old one, and re-seed the exact accumulator from the live integral.
    median_count = 0; median_pos = 0;
    freq_integral_num = freq_integral * SERVO_KI_DEN;
}

void ptpv1_get_tuning(uint8_t *median_n, int32_t *ki_num, uint8_t *exact)
{
    *median_n = rt_median_n; *ki_num = rt_ki_num; *exact = rt_exact_integ;
}

static int       unlock_streak;
static uint32_t lock_streak;

// Frequency-acquisition state. While acq_active the servo is held off and the
// addend is left alone, so the rate stays constant across the measurement.
static uint8_t         acq_active, acq_have_base;
static ptp_timestamp_t acq_t1_0, acq_t2_0;

// Raw (t2-t1) from the most recent resolved Sync pair, kept for the path-delay
// calculation. Must stay RAW -- see the note in the DelayResp handler.
static int64_t  last_t2_t1;
static uint8_t  have_t2_t1;

// TEMPORARY: why is mean_path_delay stuck at 0? Count each rejection path in
// the DelayResp handler separately, so the answer is a number rather than a
// theory. It measured 21.4-21.7 us earlier today, so something in the
// acquisition rework regressed it.
static uint32_t first_sync_ms;      // when we first heard the master
static uint32_t pd_samples;         // accepted path-delay measurements
static uint8_t  pd_step_done;       // residual phase removed once delay is known

static uint32_t dbg_dr_uuid, dbg_dr_port, dbg_dr_not3, dbg_dr_seq;
static uint32_t dbg_dr_neg, dbg_dr_big, dbg_dr_ok;
static int64_t  dbg_last_d, dbg_last_t4t3;

// ---- TEMPORARY DIAGNOSTIC -- remove once RX timestamping is characterised ---
//
// The heartbeat showed the frequency servo converging cleanly (52813 -> -4340
// ppb) while the raw offset refused to settle, bouncing 9..191 us at random.
// That is not a servo failing to converge, it is noise on the measurement, and
// its spread is suspiciously close to one main-loop period.
//
// (t1, t2) is dumped for every offset computation to 224.0.0.233:9999. The
// master's Sync period is exactly 250 ms, so successive t1 deltas are a fixed
// ruler; any jitter in the matching t2 deltas is OUR receive timestamping
// error, measured without needing the UART or a reference clock on the host.
static const uint8_t dbg_group[4] = {224, 0, 0, 233};
#define DBG_N 8
static struct { uint32_t t1s, t1n, t2s, t2n; } dbg_ring[DBG_N];
static uint8_t  dbg_w;
static uint32_t next_dbg_ms;

// Second diagnostic: a controlled-stimulus probe of the RX timestamp path.
//
// Aligning our t2 against the host capture showed our Sync arrival deltas
// snapping to multiples of 250.3333 ms with only +/-3.5 us spread, while the
// master's real inter-Sync jitter is +/-100 us and the host sees it plainly.
// An arrival timestamp cannot smooth away jitter that is really on the wire,
// so the capture is not tracking arrival -- but PTP is a poor instrument for
// proving that, because we do not control when the master transmits.
//
// So: the host sends UDP to port 7777 at deliberately IRREGULAR intervals with
// a sequence number in the payload, and we report the RX timestamp we captured
// for each. Even spacing under uneven stimulus is proof the capture path is
// broken, independent of anything PTP does.
#define PROBE_PORT 7777
#define PROBE_N 16
static struct { uint32_t seq, ts_s, ts_n; } probe_ring[PROBE_N];
static uint8_t probe_w;

static inline uint32_t rd32(const uint8_t *p);

static void probe_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *payload, uint32_t len)
{
    (void)src_ip; (void)dst_ip; (void)src_port;
    if (len < 4) return;
    ptp_timestamp_t ts = gptp_read_rx_timestamp();
    probe_ring[probe_w].seq  = rd32(payload);
    probe_ring[probe_w].ts_s = (uint32_t)ts.seconds;
    probe_ring[probe_w].ts_n = ts.nanoseconds;
    probe_w = (uint8_t)((probe_w + 1) % PROBE_N);
}

// ---------------------------------------------------------------------------

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t rd32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

// This libc has no memcmp (osc.c hit the same thing). Six bytes, so a loop is
// both sufficient and clearer than pulling in a dependency.
static inline int uuid_eq(const uint8_t *a, const uint8_t *b)
{
    int diff = 0;
    for (int i = 0; i < 6; i++) diff |= a[i] ^ b[i];
    return !diff;
}

static int64_t median_of(int64_t v)
{
    median_buf[median_pos] = v;
    median_pos = (uint8_t)((median_pos + 1) % rt_median_n);
    if (median_count < rt_median_n) median_count++;

    int64_t tmp[MEDIAN_N];
    for (uint8_t i = 0; i < median_count; i++) tmp[i] = median_buf[i];
    for (uint8_t i = 1; i < median_count; i++) {      // insertion sort, n<=7
        int64_t k = tmp[i];
        int8_t  j = (int8_t)(i - 1);
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    return tmp[median_count / 2];
}

// ---------------------------------------------------------------------------
// Servo — PI on the 52-bit TSU addend, same as gPTP's.
// ---------------------------------------------------------------------------

static void servo_update(int64_t offset_ns)
{
    g_ptpv1.offset_ns = offset_ns;
    g_ptpv1.servo_updates++;

    // A large offset is a time jump, not a frequency error.
    //
    // Use an ABSOLUTE step, not a relative correction. At cold boot our TSU
    // counts from 0 while the Dante Leader is at ~6171 s, so the first offset is
    // ~-6.1e12 ns. Feeding that to gptp_adjust_offset() as a relative nudge does
    // not converge, and the servo then re-steps on every Sync forever without
    // ever reaching the frequency-control path -- observed as a frequency offset
    // pinned at exactly 0 ppb in the heartbeat across 24 consecutive reports.
    //
    // Setting the clock directly to master time + path delay is what gptp.c does
    // for the same situation, and it converges in one Sync.
    if (offset_ns > SERVO_STEP_NS || offset_ns < -SERVO_STEP_NS) {
        ptp_timestamp_t target = t1;
        int64_t add_ns = g_ptpv1.mean_path_delay_ns;
        int64_t ns = (int64_t)target.nanoseconds + add_ns;
        while (ns >= 1000000000LL) { ns -= 1000000000LL; target.seconds++; }
        target.nanoseconds = (uint32_t)ns;
        gptp_step_time(target);
        g_ptpv1.step_count++;          // anything anchored to PTP is now stale
        g_ptpv1.phase_settled = 0;

        // Deliberately KEEP freq_integral across a step.
        //
        // A step is a PHASE correction; the integral holds our crystal's
        // frequency offset, which is a property of our own oscillator and does
        // not change because the master's time jumped. Zeroing it here threw
        // away every bit of frequency learning on each step and forced a full
        // re-acquisition from scratch -- which is what happened when the leader
        // jumped 1.87 s ("[ptpv1] step to 56947.4... was off -1869571162 ns")
        // and re-locking then took minutes all over again.
        //
        // Phase state IS reset below, because it is now meaningless.
        //
        // Re-measure the frequency after a step as well: it costs one window
        // and removes any doubt about whether the step also changed our rate.
        acq_active = 1; acq_have_base = 0;
        lock_streak   = 0;
        g_ptpv1.locked = 0;
        median_count = 0; median_pos = 0;
        printf("[ptpv1] step to %llu.%09lu (was off %lld ns)\n",
               (unsigned long long)target.seconds,
               (unsigned long)target.nanoseconds, (long long)offset_ns);
        return;
    }

    // ---- Frequency acquisition: measure our crystal offset directly --------
    if (acq_active) {
        if (!acq_have_base) {
            acq_t1_0 = t1; acq_t2_0 = t2; acq_have_base = 1;
            return;                       // servo held off; addend untouched
        }
        int64_t dm = gptp_ts_diff_ns(t1, acq_t1_0);   // master elapsed
        int64_t dl = gptp_ts_diff_ns(t2, acq_t2_0);   // our elapsed
        if (dm < ACQ_WINDOW_NS) return;               // keep accumulating

        // Our rate error against the master, in ppb, over the whole window.
        int64_t err_ppb = ((dl - dm) * 1000000000LL) / dm;

        // What the addend is currently worth, so the correction is applied to
        // the rate actually in force rather than to nominal.
        int64_t applied = 0;
        if (g_ptpv1.base_addend_full) {
            int64_t base = (int64_t)g_ptpv1.base_addend_full;
            int64_t cur  = (int64_t)g_ptpv1.current_addend_full;
            applied = ((cur - base) * 1000000000LL) / base;
        }
        freq_integral = applied - err_ppb;
        if (freq_integral >  SERVO_INTEGRAL_MAX) freq_integral =  SERVO_INTEGRAL_MAX;
        if (freq_integral < -SERVO_INTEGRAL_MAX) freq_integral = -SERVO_INTEGRAL_MAX;

        int64_t na = (int64_t)g_ptpv1.base_addend_full
                   + (freq_integral * (int64_t)g_ptpv1.base_addend_full) / 1000000000LL;
        if (na < 1) na = 1;
        g_ptpv1.current_addend_full = (uint64_t)na;
        gptp_set_addend_full(g_ptpv1.current_addend_full);

        // Frequency is now right; remove the standing phase error in one go.
        //
        // RELATIVE, not an absolute step to t1 + path_delay. offset_ns is the
        // difference between our clock and the master's AT THE SAME INSTANT, so
        // subtracting it is correct whenever it is applied. An absolute step is
        // not: it lands at main-loop processing time rather than at packet
        // receipt, and is late by exactly that lag.
        //
        // That is what left ~13 us standing here -- (t2-t1) measured ~35 us
        // against a 21.4 us path delay -- which the integral then removed at
        // ~30 ns/s, i.e. minutes. The absolute step is still right for the cold
        // -6.1e12 ns case, where a relative one cannot land at all: the TSU's
        // offset correction only handles +-1 s (liteeth core/ptp.py).
        gptp_adjust_offset(-offset_ns);

        acq_active = 0;
        // Discard anything measured during the window and start clean.
        g_ptpv1.mean_path_delay_ns = 0;
        pd_samples = 0; pd_step_done = 0;
        median_count = 0; median_pos = 0;
        lock_streak = 0;
        printf("[ptpv1] freq acquired: %lld ppb (window %lld ms)\n",
               (long long)freq_integral, (long long)(dm / 1000000));
        return;
    }

    // Once the path delay is known, remove the residual phase in one step
    // instead of servoing it down. The initial acquisition step necessarily
    // ran without a trustworthy delay, so it lands one path delay out; with a
    // real measurement in hand that error is known exactly and there is no
    // reason to spend two minutes of integral action on it.
    if (!pd_step_done && pd_samples >= PD_MIN_SAMPLES) {
        pd_step_done = 1;
        gptp_adjust_offset(-offset_ns);
        // THIS is the step that used to land AFTER the talker had already
        // anchored: mean_path_delay_ns goes non-zero on the FIRST DelayResp,
        // which satisfied dante_tx's gate, but this residual correction needs
        // PD_MIN_SAMPLES of them. The media clock was anchored to the pre-step
        // timeline and stayed there.
        g_ptpv1.step_count++;
        g_ptpv1.phase_settled = 1;     // no further phase steps expected
        median_count = 0; median_pos = 0;
        lock_streak = 0;
        printf("[ptpv1] path delay %lld ns, phase corrected %lld ns\n",
               (long long)g_ptpv1.mean_path_delay_ns, (long long)-offset_ns);
        return;
    }

    int64_t filtered = median_of(offset_ns);

    int64_t kp_num = (filtered > 1000 || filtered < -1000)
                     ? SERVO_KP_FAST_NUM : SERVO_KP_NUM;
    int64_t ki_num = rt_ki_num;
    int64_t p = (-filtered * kp_num) / SERVO_KP_DEN;

    // Anti-windup: only integrate inside a band, so a transient does not leave
    // a persistent frequency bias behind.
    if (filtered < 1000000 && filtered > -1000000) {
        if (rt_exact_integ) {
            // Exact accumulation: keeps the remainder the truncating form
            // throws away every update (offsets below SERVO_KI_DEN/ki_num ns
            // otherwise contribute nothing at all).
            freq_integral_num += (int64_t)(-filtered) * ki_num;
            if (freq_integral_num >  SERVO_INTEGRAL_MAX * SERVO_KI_DEN)
                freq_integral_num =  SERVO_INTEGRAL_MAX * SERVO_KI_DEN;
            if (freq_integral_num < -SERVO_INTEGRAL_MAX * SERVO_KI_DEN)
                freq_integral_num = -SERVO_INTEGRAL_MAX * SERVO_KI_DEN;
            freq_integral = freq_integral_num / SERVO_KI_DEN;
        } else {
            freq_integral += (-filtered * ki_num) / SERVO_KI_DEN;
            freq_integral_num = freq_integral * SERVO_KI_DEN;
        }
        if (freq_integral >  SERVO_INTEGRAL_MAX) freq_integral =  SERVO_INTEGRAL_MAX;
        if (freq_integral < -SERVO_INTEGRAL_MAX) freq_integral = -SERVO_INTEGRAL_MAX;
    }

    // Do not let a wild sample steer the rate. Once locked, steady state is
    // sub-microsecond; anything an order of magnitude beyond the unlock
    // threshold is a corrupt pairing, and integrating it corrupts the clock
    // that every downstream receiver follows.
    if (g_ptpv1.locked) {
        int64_t mag = filtered < 0 ? -filtered : filtered;
        if (mag > (int64_t)UNLOCK_THRESHOLD_NS * 4) return;
    }

    int64_t adj = p + freq_integral;
    // Scale ns-of-error into addend LSBs: the addend is ~2^52/clk, so a 1 ppb
    // change is base/1e9. Keep the arithmetic in 64-bit throughout.
    int64_t addend = (int64_t)g_ptpv1.base_addend_full
                   + (int64_t)((adj * (int64_t)g_ptpv1.base_addend_full) / 1000000000LL);
    if (addend < 1) addend = 1;
    g_ptpv1.current_addend_full = (uint64_t)addend;
    // Publish the INTEGRAL alone as the rate estimate. mcr follows this.
    g_ptpv1.rate_ppb = (int32_t)freq_integral;
    gptp_set_addend_full(g_ptpv1.current_addend_full);

    // One telemetry record per servo update. Raw AND filtered offset together:
    // the difference between them is what the median filter is actually doing,
    // and no snapshot could ever show it.
    telem_push(TELEM_T_PTP, g_ptpv1.locked ? TELEM_F_LOCKED : 0,
               (uint16_t)g_ptpv1.servo_updates,
               (int32_t)g_ptpv1.offset_ns,
               (int32_t)filtered,
               (int32_t)g_ptpv1.mean_path_delay_ns,
               g_ptpv1.rate_ppb);

    // Hold lock off until the path delay is known -- but never indefinitely.
    //
    // mean_path_delay starts at 0, so the offset is computed without it and can
    // look excellent until the first DelayResp lands and shifts it by the whole
    // path delay at once. That is the single unlock/relock seen right after
    // boot, and the size matches exactly:
    //
    //   [ptpv1] LOCKED, offset -60 ns
    //   [ptpv1] unlocked, offset -13468 ns    <-- mean_path_delay 0 -> 13.6 us
    //   [ptpv1] LOCKED, offset 413 ns
    //
    // A bare `mean_path_delay != 0` gate was tried first and was WRONG: when
    // the delay estimate stalls at 0 the gate never opens and the device never
    // locks at all. A clock that will not lock is far worse than one that
    // flaps once, so the wait is now bounded -- past PATH_DELAY_GRACE_MS we
    // lock anyway and accept the offset being short by the path delay, which
    // is the behaviour we had before this gate existed.
    int64_t a = filtered < 0 ? -filtered : filtered;
    uint32_t waited = gptp_uptime_ms() - first_sync_ms;
    int delay_known = (pd_samples >= PD_MIN_SAMPLES) ||
                      (first_sync_ms && waited > PATH_DELAY_GRACE_MS);
    if (a < LOCK_THRESHOLD_NS && delay_known) {
        unlock_streak = 0;
        if (++lock_streak >= LOCK_STREAK && !g_ptpv1.locked) {
            g_ptpv1.locked = 1;
            telem_event(TELEM_E_PTP_LOCK, (int32_t)filtered,
                        (int32_t)g_ptpv1.mean_path_delay_ns);
            // WARM START: remember the converged addend. The crystal error is a
            // property of this board, not of this boot, so re-deriving it from
            // scratch every time costs ~10-20 s of the ~30 s lock for no new
            // information. Saved once per lock edge, not per update, to avoid
            // writing flash continuously.
            if (!g_cfg.ptp_addend_valid ||
                g_cfg.ptp_addend_full != g_ptpv1.current_addend_full) {
                g_cfg.ptp_addend_valid = 1;
                g_cfg.ptp_addend_full  = g_ptpv1.current_addend_full;
                cfg_save();
            }
            printf("[ptpv1] LOCKED, offset %lld ns\n", (long long)filtered);
        }
    } else if (a > UNLOCK_THRESHOLD_NS) {
        lock_streak = 0;
        // UNLOCK HYSTERESIS. A single bad sample must not drop lock.
        //
        // The console showed this cycling continuously: lock at sub-microsecond,
        // then one sample at +5 to +10 us trips the 5 us threshold, unlock,
        // relock at sub-microsecond, repeat. Every unlock turns the talker OFF
        // and every relock RE-ANCHORS the media clock, so the stream was being
        // stopped and restarted every few tens of seconds. That -- not drift,
        // not underruns -- is what was killing the audio.
        //
        // Steady state is sub-microsecond, so an excursion 10x larger is a bad
        // measurement, not a real clock movement: our own crystal cannot move
        // 10 us between two Syncs. The most likely source is a lost FollowUp or
        // DelayResp pairing with the wrong Sync, which is plausible given
        // mac_writer_err climbing ~25/s under multicast flood.
        //
        // Requiring several consecutive bad samples keeps a genuine loss of
        // sync detected (4 samples is a few seconds at PTPv1 rates) while
        // ignoring isolated outliers.
        if (++unlock_streak >= UNLOCK_STREAK) {
            if (g_ptpv1.locked) {
                g_ptpv1.locked = 0;
                telem_event(TELEM_E_PTP_UNLOCK, (int32_t)filtered, unlock_streak);
                printf("[ptpv1] unlocked, offset %lld ns (%d consecutive)\n",
                       (long long)filtered, unlock_streak);
            }
        } else if (g_ptpv1.locked) {
            telem_push(TELEM_T_PTP, TELEM_F_LOCKED | TELEM_F_OUTLIER,
                       (uint16_t)unlock_streak, (int32_t)g_ptpv1.offset_ns,
                       (int32_t)filtered, 0, g_ptpv1.rate_ppb);
            printf("[ptpv1] outlier %lld ns ignored (%d/%d)\n",
                   (long long)filtered, unlock_streak, UNLOCK_STREAK);
        }
    } else {
        unlock_streak = 0;      // inside the band again
    }
}

// Offset = (t2 - t1) - mean_path_delay
static void try_compute_offset(void)
{
    if (!have_t1 || !have_t2) return;

    dbg_ring[dbg_w].t1s = (uint32_t)t1.seconds; dbg_ring[dbg_w].t1n = t1.nanoseconds;
    dbg_ring[dbg_w].t2s = (uint32_t)t2.seconds; dbg_ring[dbg_w].t2n = t2.nanoseconds;
    dbg_w = (uint8_t)((dbg_w + 1) % DBG_N);

    int64_t t2_t1 = gptp_ts_diff_ns(t2, t1);
    last_t2_t1 = t2_t1;
    have_t2_t1 = 1;
    servo_update(t2_t1 - g_ptpv1.mean_path_delay_ns);
    have_t1 = have_t2 = 0;
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

static uint32_t put_header(uint8_t *p, uint8_t control, uint8_t port_type,
                           uint16_t seq)
{
    memset(p, 0, HDR_LEN);
    wr16(p + 0, 1);                      // version_ptp
    wr16(p + 2, 1);                      // version_network
    p[4] = '_'; p[5] = 'D'; p[6] = 'F'; p[7] = 'L'; p[8] = 'T';
    p[20] = port_type;
    p[21] = 1;                           // source_communication_technology
    memcpy(p + 22, our_uuid, 6);
    wr16(p + 28, our_port_id);
    wr16(p + 30, seq);
    p[32] = control;
    return HDR_LEN;
}

static void send_delay_req(void)
{
    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_header(p, CTRL_DELAY_REQ, PORT_TYPE_EVENT, delay_req_seq);
    memset(p + n, 0, SYNC_BODY_LEN);     // DelayReq shares Sync's body layout
    n += SYNC_BODY_LEN;

    if (net_udp_commit(ptp_group, PTP_EVENT_PORT, PTP_EVENT_PORT, n,
                       NET_TOS_PTP) == 0) {
        // TX timestamp is latched by the TSU on the first beat of the frame.
        t3 = gptp_read_tx_timestamp();
        have_t3 = 1;
        g_ptpv1.tx_delay_req++;
    }
    delay_req_seq++;
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

static int header_ok(const uint8_t *p, uint32_t len)
{
    if (len < HDR_LEN) return 0;
    if (rd16(p) != 1) return 0;                        // PTPv1 only
    if (p[4] != '_' || p[5] != 'D' || p[6] != 'F' ||
        p[7] != 'L' || p[8] != 'T') return 0;          // subdomain _DFLT
    return 1;
}

static void ptpv1_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *p, uint32_t len)
{
    (void)src_ip; (void)dst_ip; (void)src_port;
    if (!header_ok(p, len)) return;

    const uint8_t *uuid = p + 22;
    uint16_t seq   = rd16(p + 30);
    uint8_t  ctrl  = p[32];
    uint8_t  flags = p[35];
    const uint8_t *body = p + HDR_LEN;
    uint32_t blen = len - HDR_LEN;

    // Ignore our own multicast echo.
    if (uuid_eq(uuid, our_uuid)) return;

    switch (ctrl) {

    case CTRL_SYNC: {
        if (blen < 8) return;
        g_ptpv1.rx_sync++;

        // Whoever is sending Sync is the Leader. No BMCA: we are slave-only, so
        // we simply follow the source of Sync.
        memcpy(g_ptpv1.master_uuid, uuid, 6);
        g_ptpv1.master_port_id = rd16(p + 28);
        g_ptpv1.have_master    = 1;
        if (!first_sync_ms) first_sync_ms = gptp_uptime_ms();

        // t2: our hardware RX timestamp for this frame, latched by the TSU and
        // popped by the dispatcher for this slot.
        t2 = gptp_read_rx_timestamp();
        have_t2 = 1;

        if (flags & FLAG_PTP_ASSIST) {
            // Two-step: the precise origin time comes in a FollowUp.
            pending_sync_seq  = seq;
            awaiting_followup = 1;
            have_t1 = 0;
        } else {
            t1.seconds     = rd32(body);
            t1.nanoseconds = rd32(body + 4);
            have_t1 = 1;
            awaiting_followup = 0;
            try_compute_offset();
        }
        break;
    }

    case CTRL_FOLLOWUP: {
        if (blen < FOLLOWUP_BODY_LEN) return;
        g_ptpv1.rx_followup++;
        if (!awaiting_followup) return;
        // associated_sequence_id at body[2..4] must match the Sync we are
        // holding a t2 for; otherwise we would pair mismatched timestamps.
        if (rd16(body + 2) != pending_sync_seq) return;
        t1.seconds     = rd32(body + 4);
        t1.nanoseconds = rd32(body + 8);
        have_t1 = 1;
        awaiting_followup = 0;
        try_compute_offset();
        break;
    }

    case CTRL_DELAY_RESP: {
        if (blen < DELAY_RESP_BODY_LEN) return;
        g_ptpv1.rx_delay_resp++;
        // Match on the requester fields, or we would consume another slave's
        // DelayResp -- they all arrive on the same multicast group.
        // Ignore path-delay measurements taken DURING frequency acquisition.
        //
        // ((t2-t1) + (t4-t3)) / 2 cancels our clock error only if that error is
        // the same at t2 and t3. While the acquisition window is open the clock
        // is free-running and drifting by design, so the two halves do not
        // cancel and the estimate comes out inflated -- measured ~41 us against
        // a true 20.5 us. The acquisition phase-step then used that inflated
        // value and left a standing offset equal to the difference, which only
        // the integral could remove, at ~150 ns/s.
        if (acq_active) return;

        if (!uuid_eq(body + 10, our_uuid)) { dbg_dr_uuid++; return; }
        if (rd16(body + 16) != our_port_id)  { dbg_dr_port++; return; }
        if (!have_t3)                        { dbg_dr_not3++; return; }
        if (rd16(body + 18) != (uint16_t)(delay_req_seq - 1)) { dbg_dr_seq++; return; }

        t4.seconds     = rd32(body);
        t4.nanoseconds = rd32(body + 4);

        // mean_path_delay = ((t2-t1) + (t4-t3)) / 2, using the most recent
        // Sync pair we resolved.
        //
        // Use the RAW (t2-t1), not g_ptpv1.offset_ns. offset_ns is already
        // (t2-t1) - mean_path_delay, so feeding it back here made the estimate
        // recursive: d_new = d_true - d_old/2, whose fixed point is (2/3)d_true.
        // That left a permanent ~1/3 error in the path delay, and since the
        // offset is corrected by exactly this term, an equal permanent error in
        // the offset -- microseconds of it, far above the 500 ns lock threshold,
        // so the servo could converge beautifully and still never lock.
        int64_t t4_t3 = gptp_ts_diff_ns(t4, t3);
        if (!have_t2_t1) return;              // no Sync pair resolved yet
        int64_t d = (last_t2_t1 + t4_t3) / 2;
        dbg_last_d = d; dbg_last_t4t3 = t4_t3;
        if (d < 0) { dbg_dr_neg++; d = 0; }   // negative delay is nonsense
        if (d >= 10000000LL) dbg_dr_big++;
        if (d < 10000000LL) {                 // ignore absurd (>10 ms) outliers
            dbg_dr_ok++;
            // SMOOTH, do not jump. The formula cancels our clock offset only if
            // that offset is the same at t2 and at t3, which holds once locked
            // but not while the servo is still moving -- so each raw estimate
            // carries whatever phase correction happened in between.
            //
            // Applying it directly made the offset ping-pong with the servo:
            // the servo drives (t2-t1) toward mpd, which recomputes mpd lower,
            // which shifts the offset again -- 35 -> 21.4 -> 14.8 -> 11.4 us,
            // each step throwing us out of lock. Exactly the -13612 ns and
            // 8547 ns unlock excursions seen on the console right after a clean
            // "LOCKED, offset -44 ns".
            //
            // The real path delay is constant on a fixed link, so a slow filter
            // is not just a damper, it is the better estimator. alpha = 1/8
            // over a 4 s DelayReq cadence settles in ~30 s and keeps each step
            // well inside the 2 us unlock threshold.
            // Running MEAN for the first few samples, then the slow filter.
            //
            // Seeding from a single measurement and smoothing at alpha = 1/8
            // from there converges far too slowly to lock against: the first
            // accepted value can be well off (it is taken while the servo is
            // still moving), and the estimate then crawls toward the truth,
            // dragging the offset with it. That is the -13295 ns unlock seen
            // right after a clean "LOCKED, offset -285 ns" -- the delay was
            // non-zero, so the lock gate opened, but it had not SETTLED.
            //
            // A running average over the first PD_MIN_SAMPLES converges in
            // those samples instead of asymptotically, and costs nothing.
            if (pd_samples < PD_MIN_SAMPLES) {
                pd_samples++;
                g_ptpv1.mean_path_delay_ns +=
                    (d - g_ptpv1.mean_path_delay_ns) / (int64_t)pd_samples;
            } else {
                g_ptpv1.mean_path_delay_ns += (d - g_ptpv1.mean_path_delay_ns) / 8;
            }
        }
        have_t3 = 0;
        break;
    }

    default:
        g_ptpv1.rx_other++;
        break;
    }
}

// ---------------------------------------------------------------------------

void ptpv1_poll(void)
{
    uint32_t now = gptp_uptime_ms();
    if (!next_delay_req_ms || (int32_t)(now - next_delay_req_ms) >= 0) {
        if (g_ptpv1.have_master) send_delay_req();
        // Ask faster until the path delay is known. Lock now waits for that
        // measurement (see servo_update), so at the steady 4 s cadence it
        // gated lock at ~24 s; at 1 s it arrives during the frequency
        // acquisition window and costs nothing. Path delay changes far more
        // slowly than offset, so the fast rate is only for acquiring it.
        next_delay_req_ms = now + (pd_samples >= PD_MIN_SAMPLES ? DELAY_REQ_MS
                                                                : DELAY_REQ_FAST_MS);
    }

    // TEMPORARY: dump the (t1,t2) ring. See the note at dbg_ring.
    if (!next_dbg_ms || (int32_t)(now - next_dbg_ms) >= 0) {
        next_dbg_ms = now + 1000;
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = 0;
        for (int i = 0; i < DBG_N; i++) {
            int k = (dbg_w + i) % DBG_N;   // oldest first
            const uint32_t v[4] = { dbg_ring[k].t1s, dbg_ring[k].t1n,
                                    dbg_ring[k].t2s, dbg_ring[k].t2n };
            for (int j = 0; j < 4; j++) {
                p[n++] = (uint8_t)(v[j] >> 24); p[n++] = (uint8_t)(v[j] >> 16);
                p[n++] = (uint8_t)(v[j] >> 8);  p[n++] = (uint8_t)v[j];
            }
        }
        // Ring health, appended to the (t1,t2) dump. If commits and pops ever
        // diverge, the ring is misaligned and every popped timestamp belongs to
        // some OTHER frame -- on this network almost always a flooded audio
        // frame, which arrives on the sender's 3 kHz media-clock grid. That is
        // the shape of the 333.35 us quantisation we measured.
        {
            const uint32_t v[8] = { g_ptpv1.rx_delay_resp,
                                    dbg_dr_uuid | (dbg_dr_port << 16),
                                    dbg_dr_not3,
                                    dbg_dr_seq,
                                    dbg_dr_neg | (dbg_dr_big << 16),
                                    dbg_dr_ok,
                                    (uint32_t)(int32_t)(dbg_last_d / 1000),
                                    (uint32_t)(int32_t)(dbg_last_t4t3 / 1000) };
            for (int j = 0; j < 8; j++) {
                p[n++] = (uint8_t)(v[j] >> 24); p[n++] = (uint8_t)(v[j] >> 16);
                p[n++] = (uint8_t)(v[j] >> 8);  p[n++] = (uint8_t)v[j];
            }
        }
        net_udp_commit(dbg_group, 9999, 9999, n, NET_TOS_BEST_EFFORT);

        // Probe ring -> :9998, oldest first.
        p = net_udp_payload_buf();
        n = 0;
        for (int i = 0; i < PROBE_N; i++) {
            int k = (probe_w + i) % PROBE_N;
            const uint32_t v[3] = { probe_ring[k].seq, probe_ring[k].ts_s,
                                    probe_ring[k].ts_n };
            for (int j = 0; j < 3; j++) {
                p[n++] = (uint8_t)(v[j] >> 24); p[n++] = (uint8_t)(v[j] >> 16);
                p[n++] = (uint8_t)(v[j] >> 8);  p[n++] = (uint8_t)v[j];
            }
        }
        net_udp_commit(dbg_group, 9998, 9998, n, NET_TOS_BEST_EFFORT);
    }
}

void ptpv1_init(const uint8_t mac[6])
{
    memcpy(our_uuid, mac, 6);
    memset(&g_ptpv1, 0, sizeof(g_ptpv1));

    // Nominal 52-bit addend, same formula as gptp.c:918 and the gateware reset.
    // (base is the nominal addend; the warm-start value below overrides the
    //  STARTING point of the servo, not this reference.)
    g_ptpv1.base_addend_full = (((uint64_t)1 << 52) + (CONFIG_CLOCK_FREQUENCY / 2))
                             / CONFIG_CLOCK_FREQUENCY;
    g_ptpv1.current_addend_full = g_ptpv1.base_addend_full;
    acq_active = 1; acq_have_base = 0;    // measure the crystal before servoing

    // WARM START. If a converged addend survived from the last run, start the
    // servo there and SKIP frequency acquisition entirely -- the 8 s window
    // exists only to learn this board's crystal error, which has not changed
    // since the last boot. Lock then costs only the phase pull-in and the
    // LOCK_STREAK confirmation instead of 8 s of measurement first.
    //
    // Sanity-bounded: a stored value more than 200 ppm from nominal is treated
    // as corrupt rather than trusted, since accepting a wild addend would send
    // the clock somewhere it may never recover from.
    if (g_cfg.ptp_addend_valid && g_cfg.ptp_addend_full) {
        int64_t base = (int64_t)g_ptpv1.base_addend_full;
        int64_t warm = (int64_t)g_cfg.ptp_addend_full;
        int64_t lim  = base / 5000;                  // 200 ppm
        if (warm > base - lim && warm < base + lim) {
            // Seed the INTEGRATOR, not current_addend_full. The servo
            // recomputes the addend every update as
            //     base + (p + freq_integral) * base / 1e9
            // so a warm value written straight into current_addend_full is
            // discarded on the first Sync. The integral is what carries the
            // standing rate correction, so that is what must be restored.
            freq_integral = ((warm - base) * 1000000000LL) / base;
            g_ptpv1.rate_ppb = (int32_t)freq_integral;
            g_ptpv1.current_addend_full = (uint64_t)warm;
            gptp_set_addend_full(g_ptpv1.current_addend_full);
            acq_active = 0;                          // skip acquisition
            printf("[ptpv1] warm start: addend %llu (%+lld ppb from nominal)\n",
                   (unsigned long long)warm,
                   (long long)((warm - base) * 1000000000LL / base));
        } else {
            printf("[ptpv1] stored addend out of range, ignoring\n");
        }
    }

    if (net_udp_bind(PTP_EVENT_PORT,   ptpv1_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", PTP_EVENT_PORT);
    if (net_udp_bind(PTP_GENERAL_PORT, ptpv1_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", PTP_GENERAL_PORT);
    if (net_udp_bind(PROBE_PORT,       probe_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", PROBE_PORT);   // TEMPORARY diagnostic
    net_igmp_join(ptp_group);

    printf("[ptpv1] slave on 224.0.1.129:%u/%u, uuid %02x%02x%02x%02x%02x%02x\n",
           PTP_EVENT_PORT, PTP_GENERAL_PORT,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
