// Minimal mDNS / DNS-SD responder — Dante Phase 3.
//
// Enough to be discovered, not a general-purpose responder. Serves:
//   <host>.local                              A
//   _netaudio-arc._udp.local                  PTR -> <name>._netaudio-arc...
//   <name>._netaudio-arc._udp.local           SRV + TXT   (port 4440)
//   _netaudio-cmc._udp.local                  PTR -> <name>._netaudio-cmc...
//   <name>._netaudio-cmc._udp.local           SRV + TXT   (port 8800)
//   _services._dns-sd._udp.local              PTR (service enumeration)
//
// Deliberately omitted for now, and each is a real gap rather than an oversight:
//   * probing / conflict resolution -- we assume our name is unique, which the
//     MAC suffix makes near-certain on a small network.
//   * known-answer suppression -- we answer even if the querier already knows.
//   * the 20-120 ms shared-record response delay -- we answer immediately.
//   * name compression on OUTPUT -- legal to omit, and it removes a whole class
//     of pointer bugs. Compression on INPUT is supported, because queriers use
//     it and we must parse what they send.
//
// The 48 _netaudio-chan records and the 6 _netaudio-bund records come with the
// audio path; they are what Dante receivers actually subscribe to.

#ifndef MDNS_H
#define MDNS_H

#include <stdint.h>

void mdns_init(void);

// Call from the main loop. Drives the boot announcement burst.
void mdns_poll(void);

// Re-announce (e.g. after an IP change).
void mdns_announce(void);

typedef struct {
    uint32_t rx_queries, rx_ignored;
    uint32_t tx_responses, tx_announce;
} mdns_stats_t;
extern mdns_stats_t g_mdns_stats;

#endif // MDNS_H
