// Dante flow-control server, port 4455. See dante_flows.c.
#ifndef DANTE_FLOWS_H
#define DANTE_FLOWS_H
#include <stdint.h>

typedef struct {
    uint32_t rx, tx, requests, accepted, rejected, unknown;
    uint32_t stopped;     // 0x0101 teardowns honoured -- see dante_flows.c
    uint32_t active;      // flows actually BUILT, not merely answered
} dante_flows_stats_t;
extern dante_flows_stats_t g_flows_stats;

void dante_flows_init(void);

#endif
