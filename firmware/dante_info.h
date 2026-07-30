// Dante device-info / heartbeat multicast — Phase 3. See dante_info.c.
//
// This is what makes Dante Controller actually engage with the device: DC does
// not poll something it has only seen in mDNS, it listens for these
// announcements.

#ifndef DANTE_INFO_H
#define DANTE_INFO_H

#include <stdint.h>
#include "gptp.h"

void dante_info_init(void);

// Supply the PTP state so the heartbeat can report frequency offset in ppb --
// the number Dante Controller plots in its clock histogram.
void dante_info_set_gptp(const gptp_t *g);

// Call from the main loop; sends the 1 Hz heartbeat.
void dante_info_poll(void);

typedef struct {
    uint32_t rx, rx_unknown;
    uint32_t tx_heartbeat, tx_info;
} dante_info_stats_t;
extern dante_info_stats_t g_info_stats;

#endif // DANTE_INFO_H
