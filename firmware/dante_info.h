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
    /* Dante Controller's reboot button (info opcode 0x90). Counted because
     * printf is compiled out of the shipping build, so without these the
     * difference between "DC never asked", "we ignored it" and "we refused
     * because there is no SPI image to come back to" is invisible -- and a
     * board that simply stays up looks identical in all three cases. Read with
     * `tools/stats.py --info` ('I' over UDP 7779). */
    uint32_t reboot_req;        /* 0x90 arrived with action 0x0001          */
    uint32_t reboot_odd;        /* 0x90 arrived with some other action      */
    uint32_t reboot_refused;    /* refused: no usable image in SPI flash    */
} dante_info_stats_t;
extern dante_info_stats_t g_info_stats;

/* Runtime probe of the q=0x13 network-info payload. 'N' over UDP 7779.
 * See the long note at the definition in dante_info.c. */
int  dante_info_netinfo_patch(uint16_t off, const uint8_t *val, uint8_t n);
void dante_info_netinfo_clear(void);

// Zet de sample rate LIVE: g_fs, de subsecond-wrap, de NCO en de
// USB-clock-source, en laat de flows opnieuw onderhandelen. Geen reset -- de
// firmware staat in coderam, dus een reset landt in de loader.
void dante_info_rate_apply(uint32_t hz);

#endif // DANTE_INFO_H
