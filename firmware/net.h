// Minimal IPv4/UDP/ICMP/IGMP/ARP stack — Dante Phase 2.
//
// Grown from osc.c, which could only reply to ARP and parse inbound UDP on one
// hardcoded port. Dante needs the other direction: every part of its control
// plane (mDNS, ARC, CMC, info/heartbeat multicast) and PTPv1 are UDP, and we
// have to originate them.
//
// Scope is deliberately small. This is not a general-purpose stack:
//   * IPv4 only, no fragmentation (in or out), no options on TX.
//   * No TCP. Dante's control plane here is entirely UDP.
//   * No DHCP. Link-local static addressing, as Dante devices use by default.
//   * No routing. Everything is on-link; there is no gateway concept beyond
//     recognising that off-link traffic is not ours.
//
// Runs from the single-threaded main loop, so nothing here is reentrant and no
// locking is needed. TX borrows the LiteEth reader slots exactly as osc.c did.

#ifndef NET_H
#define NET_H

#include <stdint.h>

#define ARP_ETHERTYPE   0x0806u
#define IPV4_ETHERTYPE  0x0800u

// DSCP values observed on real Dante hardware (captures/README.md):
//   audio uses TOS 0xB8 = DSCP 46 (EF, expedited forwarding)
//   PTPv1 uses TOS 0xE0 = DSCP 56 (CS7, network control)
// Control-plane chatter goes best-effort.
#define NET_TOS_BEST_EFFORT  0x00u
#define NET_TOS_AUDIO        0xB8u
#define NET_TOS_PTP          0xE0u

// A bound UDP port handler. `src_ip` is the sender, `dst_ip` lets a handler
// distinguish unicast from the multicast group it arrived on (mDNS and the
// Dante info sockets both care). Payload excludes the UDP header.
typedef void (*net_udp_handler_t)(const uint8_t src_ip[4],
                                  const uint8_t dst_ip[4],
                                  uint16_t src_port,
                                  const uint8_t *payload,
                                  uint32_t len);

// ---- lifecycle -------------------------------------------------------------

void net_init(const uint8_t mac[6], const uint8_t ip[4], uint8_t prefix);
void net_set_ip(const uint8_t ip[4], uint8_t prefix);

// Call from the main loop. Drives IGMP membership reports and ARP cache aging.
void net_poll(void);

// Entry from the RX dispatcher for ethertypes 0x0806 and 0x0800.
// Returns 1 if the frame was consumed.
int  net_rx_frame(const uint8_t *frame, uint32_t len);

// ---- UDP -------------------------------------------------------------------

// Bind a handler to a local port. Returns 0 on success, -1 if the table is
// full. Binding the same port twice replaces the handler.
int  net_udp_bind(uint16_t port, net_udp_handler_t handler);

// Send a UDP datagram. Returns 0 on success, -1 if the destination MAC is
// unknown (an ARP request is issued; retry shortly) or the payload is too big.
//
// Multicast destinations never fail for want of ARP: the MAC is computed.
int  net_udp_send(const uint8_t dst_ip[4], uint16_t dst_port,
                  uint16_t src_port, const uint8_t *payload, uint32_t len,
                  uint8_t tos);

// Borrow the TX buffer to build a large payload in place, avoiding a second
// copy for things like mDNS responses and ARC replies. Returns the payload
// area (write up to NET_MAX_PAYLOAD bytes), then call net_udp_commit().
#define NET_MAX_PAYLOAD  1400u
uint8_t *net_udp_payload_buf(void);
int      net_udp_commit(const uint8_t dst_ip[4], uint16_t dst_port,
                        uint16_t src_port, uint32_t len, uint8_t tos);

// ---- IGMP ------------------------------------------------------------------

// Join an IPv4 multicast group. Sends a v2 membership report immediately and
// answers subsequent queries. Groups in 224.0.0.0/24 are link-local scope and
// are never pruned by snooping switches, so joining them is optional -- but
// harmless, and it keeps the accept filter honest.
int  net_igmp_join(const uint8_t group[4]);

// ---- ARP -------------------------------------------------------------------

// Look up an on-link IPv4 address. Returns 1 and fills mac_out on a hit;
// returns 0 and issues a request on a miss.
int  net_arp_lookup(const uint8_t ip[4], uint8_t mac_out[6]);

// ---- helpers / diagnostics -------------------------------------------------

uint16_t net_checksum(const uint8_t *data, uint32_t len, uint32_t initial);
void     net_mcast_mac(const uint8_t ip[4], uint8_t mac_out[6]);

extern uint8_t g_net_ip[4];
extern uint8_t g_net_prefix;

typedef struct {
    uint32_t rx_arp, rx_ip, rx_udp, rx_icmp, rx_igmp, rx_dropped;
    uint32_t tx_udp, tx_arp, tx_icmp, tx_igmp, tx_dropped;
    uint32_t arp_misses;
} net_stats_t;
extern net_stats_t g_net_stats;

// Textual IP config, retained from osc.c for the console and NV config.
int  net_ip_str(char *buf, int maxlen);
int  net_parse_ipstr(const char *s);

#endif // NET_H
