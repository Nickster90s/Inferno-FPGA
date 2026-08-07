// Dante ARC server (port 4440) — Phase 3. See dante_arc.c.
//
// The Routing-tab enabler: Dante Controller asks here for channel counts and
// the transmit-channel list. Without it the device shows in the Device tab with
// no channels and cannot be routed.

#ifndef DANTE_ARC_H
#define DANTE_ARC_H

#include <stdint.h>

void dante_arc_init(void);

typedef struct {
    uint32_t rx;        // requests received
    uint32_t tx;        // responses sent
    uint32_t unknown;   // opcodes we do not implement
} dante_arc_stats_t;
extern dante_arc_stats_t g_arc_stats;

#endif // DANTE_ARC_H

// Patch an inline key in the 0x1100 property table (latency-capability probing).
int dante_arc_patch_1100(uint16_t key, uint16_t val);

// 0x1000 capability bytes (latency-list probing).
extern uint8_t g_dev_flags0;
extern uint8_t g_dev_flags2;
extern uint16_t g_router_vers;
extern uint16_t g_arcp_vers;
int dante_arc_patch_1100_u32(uint16_t key, uint32_t val);
