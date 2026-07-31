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
// Deliberately negative. flows_tx.rs:44 puts it plainly: "it's better to have
// the clock in the past than in the future - otherwise Dante devices receiving
// from us go mad and fart." A receiver can buffer a packet that arrives early
// for its timestamp; one that arrives late is already useless.
//
// MUST BE A MULTIPLE OF fpp (16). Every emitted timestamp is a multiple of fpp
// by construction -- the packetizer paces off the low bits of the sample
// counter precisely so that holds, and flows_tx.rs:58-61 bootstraps the same
// property by hand. An offset that is not a multiple of fpp destroys it.
//
// This was -24, taken from inferno's CLOCK_OFFSET_NS of -500 us. That is the
// right MAGNITUDE but the wrong place to apply it: inferno offsets the CLOCK
// and bootstraps next_ts separately, so its timestamps stay aligned. Ours went
// straight into the emitted index and left every timestamp at ts % 16 == 8,
// caught by decoding our own packets off the wire.
//
// -32 samples = -667 us, the nearest multiple of fpp to inferno's -500 us that
// errs on the side of "further in the past".
#define DANTE_TX_TS_OFFSET   (74)       // samples; applied in ts_anchor(), NOT the CSR

typedef struct {
    uint32_t enables;
    uint32_t disables;
    uint32_t anchors;     // media-clock loads from PTP
} dante_tx_stats_t;
extern dante_tx_stats_t g_tx_stats;

void    dante_tx_init(void);     // bind flows, publish bundles, hold talker off
void    dante_tx_poll(void);     // enable/disable the talker on PTPv1 lock
uint8_t dante_tx_enabled(void);
void    dante_tx_report(void);   // console diagnostics

// Flow -> multicast group, for the mDNS bundle records.
const uint8_t *dante_tx_flow_ip(unsigned f);
unsigned       dante_tx_flows(void);

#endif // DANTE_TX_H

// Bind a unicast flow from a receiver's request. Returns the context index, or
// -1 if none is free. `chans` are 1-based tx channel numbers, 0 meaning an
// unused slot that must carry silence.
int  dante_tx_bind_unicast(const uint8_t peer_ip[4], const uint8_t dst_ip[4],
                           uint16_t dst_port, const uint16_t *chans,
                           uint8_t nslots, uint8_t fpp);
int  dante_tx_bind_multicast(unsigned f);   // 8 consecutive ch, fpp 16
void dante_tx_expire(void);      // drop flows whose keepalives stopped
