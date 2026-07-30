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

// Servo. Same shape as gptp.c's, retuned for the lower Sync rate.
#define SERVO_KP_NUM        72
#define SERVO_KP_DEN        1000
#define SERVO_KP_FAST_NUM   200          // when |offset| > 1 us
#define SERVO_KI_NUM        900          // gPTP used 3600 at 8 Hz; /4 for ~2 Hz
#define SERVO_KI_DEN        1000000
#define SERVO_INTEGRAL_MAX  100000000LL  // +-100 ms
#define SERVO_STEP_NS       500000000LL  // step rather than slew beyond 500 ms
#define LOCK_THRESHOLD_NS   500
#define UNLOCK_THRESHOLD_NS 2000
#define LOCK_STREAK         8

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

static int64_t  freq_integral;
static uint32_t lock_streak;

// Raw (t2-t1) from the most recent resolved Sync pair, kept for the path-delay
// calculation. Must stay RAW -- see the note in the DelayResp handler.
static int64_t  last_t2_t1;
static uint8_t  have_t2_t1;

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
    median_pos = (uint8_t)((median_pos + 1) % MEDIAN_N);
    if (median_count < MEDIAN_N) median_count++;

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

        freq_integral = 0;
        lock_streak   = 0;
        g_ptpv1.locked = 0;
        median_count = 0; median_pos = 0;
        printf("[ptpv1] step to %llu.%09lu (was off %lld ns)\n",
               (unsigned long long)target.seconds,
               (unsigned long)target.nanoseconds, (long long)offset_ns);
        return;
    }

    int64_t filtered = median_of(offset_ns);

    int64_t kp_num = (filtered > 1000 || filtered < -1000)
                     ? SERVO_KP_FAST_NUM : SERVO_KP_NUM;
    int64_t p = (-filtered * kp_num) / SERVO_KP_DEN;

    // Anti-windup: only integrate inside a band, so a transient does not leave
    // a persistent frequency bias behind.
    if (filtered < 1000000 && filtered > -1000000) {
        freq_integral += (-filtered * SERVO_KI_NUM) / SERVO_KI_DEN;
        if (freq_integral >  SERVO_INTEGRAL_MAX) freq_integral =  SERVO_INTEGRAL_MAX;
        if (freq_integral < -SERVO_INTEGRAL_MAX) freq_integral = -SERVO_INTEGRAL_MAX;
    }

    int64_t adj = p + freq_integral;
    // Scale ns-of-error into addend LSBs: the addend is ~2^52/clk, so a 1 ppb
    // change is base/1e9. Keep the arithmetic in 64-bit throughout.
    int64_t addend = (int64_t)g_ptpv1.base_addend_full
                   + (int64_t)((adj * (int64_t)g_ptpv1.base_addend_full) / 1000000000LL);
    if (addend < 1) addend = 1;
    g_ptpv1.current_addend_full = (uint64_t)addend;
    gptp_set_addend_full(g_ptpv1.current_addend_full);

    int64_t a = filtered < 0 ? -filtered : filtered;
    if (a < LOCK_THRESHOLD_NS) {
        if (++lock_streak >= LOCK_STREAK && !g_ptpv1.locked) {
            g_ptpv1.locked = 1;
            printf("[ptpv1] LOCKED, offset %lld ns\n", (long long)filtered);
        }
    } else if (a > UNLOCK_THRESHOLD_NS) {
        lock_streak = 0;
        if (g_ptpv1.locked) {
            g_ptpv1.locked = 0;
            printf("[ptpv1] unlocked, offset %lld ns\n", (long long)filtered);
        }
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
        if (!uuid_eq(body + 10, our_uuid)) return;
        if (rd16(body + 16) != our_port_id) return;
        if (!have_t3) return;
        if (rd16(body + 18) != (uint16_t)(delay_req_seq - 1)) return;

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
        if (d < 0) d = 0;                     // negative delay is nonsense
        if (d < 10000000LL)                   // ignore absurd (>10 ms) outliers
            g_ptpv1.mean_path_delay_ns = d;
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
        next_delay_req_ms = now + DELAY_REQ_MS;
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
            extern uint32_t rx_ts_resyncs, g_lvl_pre, g_lvl_post;
            const uint32_t v[4] = { g_lvl_pre,
                                    g_lvl_post,
                                    main_rx_ts_commit_count_read(),
                                    rx_ts_resyncs };
            for (int j = 0; j < 4; j++) {
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
    g_ptpv1.base_addend_full = (((uint64_t)1 << 52) + (CONFIG_CLOCK_FREQUENCY / 2))
                             / CONFIG_CLOCK_FREQUENCY;
    g_ptpv1.current_addend_full = g_ptpv1.base_addend_full;

    net_udp_bind(PTP_EVENT_PORT,   ptpv1_rx);
    net_udp_bind(PTP_GENERAL_PORT, ptpv1_rx);
    net_udp_bind(PROBE_PORT,       probe_rx);   // TEMPORARY diagnostic
    net_igmp_join(ptp_group);

    printf("[ptpv1] slave on 224.0.1.129:%u/%u, uuid %02x%02x%02x%02x%02x%02x\n",
           PTP_EVENT_PORT, PTP_GENERAL_PORT,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
