// Dante CMC server (port 8800) — Phase 3. See dante_cmc.c.

#ifndef DANTE_CMC_H
#define DANTE_CMC_H

#include <stdint.h>

void dante_cmc_init(void);

typedef struct { uint32_t rx, tx; } dante_cmc_stats_t;
extern dante_cmc_stats_t g_cmc_stats;

#endif // DANTE_CMC_H
