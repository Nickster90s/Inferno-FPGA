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
#define DANTE_TX_TS_OFFSET   (-24)      // half a packet at fpp=16

typedef struct {
    uint32_t enables;
    uint32_t disables;
} dante_tx_stats_t;
extern dante_tx_stats_t g_tx_stats;

void    dante_tx_init(void);     // bind flows, publish bundles, hold talker off
void    dante_tx_poll(void);     // enable/disable the talker on PTPv1 lock
uint8_t dante_tx_enabled(void);
void    dante_tx_report(void);   // console diagnostics

#endif // DANTE_TX_H
