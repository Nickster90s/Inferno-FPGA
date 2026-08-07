// Dante multicast audio talker — Phase 5. See dante_tx.c.

#ifndef DANTE_TX_H
#define DANTE_TX_H

#include <stdint.h>

// IP TOS 0xB8 = DSCP EF (expedited forwarding), which is what real Dante
// devices mark audio with. TTL 1 keeps multicast on the local segment.
#define DANTE_TX_IP_TOS      0xB8
#define DANTE_TX_IP_TTL      1

// Timestamp offset in SAMPLES, applied to the emitted sample index.
//
// SIGN. The gateware stamps `counter - (fpp-1)` and ts_anchor() loads the
// counter at PTP + this value, so with a counter that tracks PTP exactly:
//
//     lag = PTP_now - emitted_timestamp = (fpp-1) - DANTE_TX_TS_OFFSET = 15 - X
//
// POSITIVE lag means stamped in the past, which is the safe side. flows_tx.rs:44
// puts it plainly: "it's better to have the clock in the past than in the future
// - otherwise Dante devices receiving from us go mad and fart." So a LARGER X
// pushes us toward the future, and this constant wants to stay SMALL.
//
// MEASURED 2026-08-05 with tools/ts_lag.py, against the PTP timeline with a
// RedNet A16R in the same capture as a control:
//
//     A16R (real Dante transmitter)   +10.8 samples   (in the past -- correct)
//     us, with X = 74                 -151.1 samples  (3.15 ms in the FUTURE)
//     difference                      +161.9 samples  (exact; method error
//                                                      cancels between streams)
//
// X = 74 predicts lag = -59. The other 92 samples are accumulated phase drift:
// anchors == 1, so the media clock is anchored once at boot and its phase then
// free-runs. Fixing X does NOT fix that -- see docs/LATENCY.md, this is step 1
// of 3 and only makes the offset correct AT BOOT.
//
// X = 6, CALIBRATED ON HARDWARE 2026-08-05 with tools/ts_offset.py.
//
// Three points on the live board, discipline armed, nothing else changing:
//
//     offset    0  ->  lag  17.0
//     offset  -60  ->  lag  75.4
//     offset -120  ->  lag 136.6      slope -1.00, lag = 17 - X
//
// so X = 6 gives lag 11, which is where a RedNet A16R sits. Verified by the
// receivers themselves via tools/dante_latency.py (the 0x8003 block devices
// publish in the 8708 heartbeat): A16R 10-12 samples, AM2 14, against 1 ms
// (48 sample) settings.
//
// DO NOT re-derive this from a model. Two attempts did, and both missed --
// see below. Re-calibrate on hardware if the packetizer pipeline changes.
//
// DERIVE THIS FROM THE RELATIVE FIGURE ONLY. The first attempt used the
// ABSOLUTE lag (-151.1) with a model that predicted -59, called the 92-sample
// gap "accumulated drift", and then computed X = 15 - 11 = 4 from the model
// rather than the measurement. That left us 92 samples (1.9 ms) in the future
// and the receiver still silent. The absolute is soft -- ts_lag.py's host->PTP
// fit reported an implausible +29.4 ppm slope, and the PTPv1 Sync field offsets
// it parses are inferred, not verified.
//
// The relative needs no model: our timestamps measured +161.9 samples HIGHER
// than the A16R's for the same arrival instant, and this constant adds 1:1 to
// the emitted timestamp, so it must drop by 162.
//
//     74 - 162 = -88
//
// VERIFY BY RE-MEASURING THE RELATIVE, not the absolute: after this change
// tools/ts_lag.py should report "A16R - us" near 0, not near +162.
//
// MULTIPLE-OF-fpp APPLIES TO THE CSR, NOT TO THIS. The two offset paths differ:
//   * ts_offset CSR is added AFTER the pacing tick, so it only preserves
//     `emitted % fpp == 0` in multiples of 16, and it has no carry into
//     seconds. That is why it is pinned at 0 (see ts_anchor() in dante_tx.c).
//   * THIS shifts the counter itself. The tick fires on ts_sub[0:4] == 15 and
//     subtracts 15, so emitted timestamps are multiples of fpp whatever the
//     counter was loaded with. Any integer is safe here.
// The previous comment stated the rule without distinguishing them, which is
// why X ended up at 74 rather than a small number: it reads as though only
// multiples of 16 were available.
//
// Verify with tools/ts_lag.py, NOT with the `drift` statistic -- drift is
// `emit - ptp_now` sampled in the main loop and includes scheduling latency.
// It read -39 while the true on-wire figure was -151.
// Compile-time SEED for the runtime value; sweep it with tools/ts_offset.py
// and only fold a verified number back into this constant.
#define DANTE_TX_TS_OFFSET   (6)        // samples; applied in ts_anchor(), NOT the CSR

// Re-anchor threshold, in samples of phase error against PTP.
//
// WAS 240 (5 ms), which is LARGER THAN THE THING IT PROTECTS. A Dante receiver
// at 1 ms discards anything outside 48 samples, so a guard that tolerates 240
// permits a violation five times over and reports nothing. Measured 2026-08-05:
// the boot anchor landed 116 samples (2.4 ms) out, the guard watched it sit
// there, and both a RedNet A16R and an AM2 reported 127-130 samples of actual
// latency against 1 ms settings -- audible failure, every counter green.
//
// The old value was chosen as a backstop BEHIND mcr_dante.c's slow phase term
// ("this only catches a PTP step or phase accumulated while the talker was
// idle"). That phase term is DISABLED, so nothing was holding phase at all and
// the backstop was the only mechanism -- set 5x too wide to be one.
//
// 24 samples = 0.5 ms: half a 1 ms budget, and 4x the measured noise floor of
// the error signal (10 consecutive reads spanned 6 samples: +1..+7). Drift with
// rate discipline armed is ~0.5 samples/min, so this fires roughly every 45
// minutes -- rare enough that the step is a non-event, which is exactly how
// inferno handles it (flows_tx.rs:138, `if lag > max_lag_samples ->
// bootstrap_next_ts`).
//
// Do not widen this to silence re-anchors. If it fires often, the phase is
// genuinely moving and the cause is upstream -- most likely mcr_dante's rate
// discipline having tripped, which lets the clock free-run at ~4.4 ppm.
#define DANTE_TX_REANCHOR_SAMPLES  (24)

typedef struct {
    uint32_t enables;
    uint32_t disables;
    uint32_t anchors;     // media-clock loads from PTP
    int32_t  trim_ppb;    // media clock rate trim, ppb
    int32_t  drift;       // last measured emitted-vs-PTP error, samples
} dante_tx_stats_t;
extern dante_tx_stats_t g_tx_stats;

void    dante_tx_init(void);     // bind flows, publish bundles, hold talker off
void    dante_tx_poll(void);     // enable/disable the talker on PTPv1 lock
uint8_t dante_tx_enabled(void);
void    dante_tx_report(void);   // console diagnostics

// Flow -> multicast group, for the mDNS bundle records.
const uint8_t *dante_tx_flow_ip(unsigned f);
unsigned       dante_tx_flows(void);


// Bind a unicast flow from a receiver's request. Returns the context index, or
// -1 if none is free. `chans` are 1-based tx channel numbers, 0 meaning an
// unused slot that must carry silence.
int  dante_tx_bind_unicast(const uint8_t peer_ip[4], const uint8_t dst_ip[4],
                           uint16_t dst_port, const uint16_t *chans,
                           uint8_t nslots, uint8_t fpp);

unsigned dante_tx_active(void);
int  dante_tx_flow_desc(unsigned f, uint8_t ip[4], uint16_t *port,
                        uint8_t *nslots, uint8_t *fpp, uint8_t *mcast,
                        uint16_t *id);
int  dante_tx_bind_multicast(uint16_t ext_id, const uint16_t *chans, uint8_t n);
int  dante_tx_ctx_for_id(uint16_t ext_id);
void dante_tx_flow_mac(unsigned f, uint8_t mac[6]);

// Read the last EMITTED media timestamp atomically.
//
// dbg_last_sec and dbg_last_ts are two separate CSR reads, and the gateware
// re-latches the pair on every emitted packet (3000-6000/s). A read that
// straddles a seconds boundary returns sec_old with sub_new -- a clean ONE
// SECOND error. Observed on the bench 2026-08-04:
//     [dtx] media clock phase -48079 samples (-1001 ms) off -- re-anchoring
// which then STEPPED the media clock, and a step is audible. Rate discipline
// holds drift to ~1.9 ms/hour, so a 1000 ms jump is never real.
void dante_tx_read_emitted(uint32_t *sec, uint32_t *sub);

// Full per-flow detail for the UDP readout: what we ACTUALLY bound, so the
// binding can be compared against what the receiver asked for without needing
// the console. Returns 0 if the slot is unused.
typedef struct {
    uint8_t  in_use;
    uint8_t  peer[4];
    uint8_t  dst[4];
    uint16_t dport;
    uint8_t  nslots;
    uint8_t  fpp;
    uint8_t  mcast;
    uint32_t age_ms;
    uint16_t chans[8];
} dante_tx_flow_detail_t;
int dante_tx_flow_detail(unsigned f, dante_tx_flow_detail_t *out);
uint16_t dante_tx_flow_chan(unsigned f, unsigned slot);
int  dante_tx_chan_bundle(uint16_t ch1, uint16_t *id, uint8_t *pos);
int  dante_tx_mcast_by_id(uint16_t id, uint8_t ip[4], uint8_t *nslots);
int  dante_tx_mcast_enum(unsigned n, uint16_t *id);
int  dante_tx_unbind(unsigned f);
void dante_tx_expire(void);
void dante_tx_drop_all(void);   // force receivers to renegotiate now
void dante_tx_flow_info(unsigned f, uint8_t *in_use, uint32_t *age_ms,
                        uint32_t *rebinds);      // drop flows whose keepalives stopped

// NOTE: this #endif used to sit at line 61, in the MIDDLE of the file, so every
// declaration below it was OUTSIDE the include guard. It went unnoticed because
// everything down there was function prototypes, and re-declaring a prototype is
// legal C -- dstats.c includes this header twice (lines 12 and 16) and got away
// with it. The first typedef added below the old #endif broke the build with
// "conflicting types". Keep this at the end of the file.
#endif // DANTE_TX_H

int32_t dante_tx_get_ts_offset(void);
void    dante_tx_set_ts_offset(int32_t samples);

int      dante_tx_fpp_supported(uint16_t fpp);
uint32_t dante_tx_fpp_index(uint16_t fpp);
