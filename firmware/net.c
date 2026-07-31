// Minimal IPv4/UDP/ICMP/IGMP/ARP stack — Dante Phase 2. See net.h.
//
// Grown from osc.c. What that had: an ARP responder, an inbound-only IPv4/UDP
// parser on one hardcoded port, and a raw-frame TX helper. What Dante needs on
// top: an IPv4/UDP TRANSMIT path (its whole control plane is UDP and we
// originate most of it), ARP requests and a cache, IGMP membership so switches
// forward our groups, and ICMP echo -- which is worth the ~30 lines purely
// because `ping` is the fastest possible proof the stack works at all.

#include "net.h"
#include "cap.h"
#include <string.h>
#include <stdio.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#define IPPROTO_ICMP  1u
#define IPPROTO_IGMP  2u
#define IPPROTO_UDP   17u

#define ETH_HDR_LEN   14u
#define IP_HDR_LEN    20u
#define UDP_HDR_LEN   8u

uint8_t     g_net_ip[4]   = {169, 254, 9, 200};   // link-local static default
uint8_t     g_net_prefix  = 16;
net_stats_t g_net_stats;

static uint8_t  net_mac[6];
static uint16_t ip_ident;          // IP identification counter

// ---------------------------------------------------------------------------
// byte helpers (network order is big-endian)
// ---------------------------------------------------------------------------

static inline uint16_t rd16(const uint8_t *p){ return ((uint16_t)p[0] << 8) | p[1]; }
static inline void     wr16(uint8_t *p, uint16_t v){ p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void     wr32(uint8_t *p, uint32_t v){
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}
static inline int eq4(const uint8_t *a, const uint8_t *b){
    return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3];
}
static inline int is_mcast(const uint8_t ip[4]){ return ip[0] >= 224 && ip[0] <= 239; }
static inline int is_bcast(const uint8_t ip[4]){
    return ip[0]==255 && ip[1]==255 && ip[2]==255 && ip[3]==255;
}

// ---------------------------------------------------------------------------
// Checksum
//
// Standard ones-complement sum with end-around carry. `initial` lets callers
// seed a UDP pseudo-header sum; pass 0 for IP headers and ICMP/IGMP.
// ---------------------------------------------------------------------------

uint16_t net_checksum(const uint8_t *data, uint32_t len, uint32_t initial)
{
    uint32_t sum = initial;
    while (len > 1) { sum += rd16(data); data += 2; len -= 2; }
    if (len)        sum += (uint32_t)data[0] << 8;      // odd tail byte
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)~sum;
}

// IPv4 multicast MAC: 01:00:5e + low 23 bits of the group address.
void net_mcast_mac(const uint8_t ip[4], uint8_t mac_out[6])
{
    mac_out[0] = 0x01; mac_out[1] = 0x00; mac_out[2] = 0x5e;
    mac_out[3] = ip[1] & 0x7f; mac_out[4] = ip[2]; mac_out[5] = ip[3];
}

// ---------------------------------------------------------------------------
// Raw Ethernet TX
//
// Own slot index, round-robined across the reader slots, exactly as osc.c did.
// Sends only happen from the single-threaded main loop, so no locking.
// ---------------------------------------------------------------------------

static uint32_t txslot;

static uint8_t *tx_buf(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + txslot));
}

static void eth_send(uint32_t len)
{
    if (len < 60) len = 60;                       // pad to minimum frame

    // Record our own control-plane transmissions. Both directions have to come
    // from the board: the host cannot see either side of a unicast exchange
    // with the controller (unmanaged switch forwards unicast only to the
    // destination port), so a capture holding requests without our replies
    // would not show whether we answered, or what we answered with.
    cap_record(1, (const uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE *
                                    (ETHMAC_RX_SLOTS + txslot)), len);
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    txslot = (txslot + 1) % ETHMAC_TX_SLOTS;

    // Wait for the queue to drain before returning.
    //
    // `ready` only means the command FIFO can accept another entry -- it does
    // NOT mean the previous frame has been read out of its slot. With only
    // ETHMAC_TX_SLOTS=2 buffers, a caller issuing several sends in a row wraps
    // around and overwrites a slot whose frame is still in flight.
    //
    // MEASURED: dante_info_init() issues five sends back-to-back (two IGMP
    // reports plus three boot announcements) and NONE of the announcements
    // reached the wire, while the isolated 1 Hz heartbeat and the
    // request/response ARC replies were always fine. That asymmetry -- bursts
    // lost, isolated sends fine -- is the signature.
    //
    // Serialising TX is acceptable here: this path carries only control plane
    // traffic at a few hundred pps worst case. The 48-channel audio stream is
    // emitted by the gateware packetizer and never touches this function.
    //
    // Bounded so a wedged MAC cannot hang the main loop.
    for (uint32_t guard = 0; guard < 100000; guard++) {
        if (ethmac_sram_reader_level_read() == 0) break;
    }
}

// ---------------------------------------------------------------------------
// ARP
// ---------------------------------------------------------------------------

#define ARP_CACHE_N  6

typedef struct {
    uint8_t  ip[4];
    uint8_t  mac[6];
    uint8_t  valid;
} arp_entry_t;

static arp_entry_t arp_cache[ARP_CACHE_N];
static uint8_t     arp_next;                     // round-robin replacement

// Learn from any frame that carries a sender IP/MAC pair. Cheap, and it means
// a peer that has already spoken to us never needs an ARP request -- which for
// milestone 1 covers essentially every unicast peer.
static void arp_learn(const uint8_t ip[4], const uint8_t mac[6])
{
    if (is_mcast(ip) || is_bcast(ip)) return;
    if (!ip[0] && !ip[1] && !ip[2] && !ip[3]) return;        // 0.0.0.0
    for (int i = 0; i < ARP_CACHE_N; i++) {
        if (arp_cache[i].valid && eq4(arp_cache[i].ip, ip)) {
            memcpy(arp_cache[i].mac, mac, 6);
            return;
        }
    }
    arp_entry_t *e = &arp_cache[arp_next];
    arp_next = (uint8_t)((arp_next + 1) % ARP_CACHE_N);
    memcpy(e->ip, ip, 4);
    memcpy(e->mac, mac, 6);
    e->valid = 1;
}

static void arp_send(uint16_t op, const uint8_t target_ip[4], const uint8_t target_mac[6])
{
    uint8_t *tx = tx_buf();
    memset(tx, 0, 60);
    if (op == 1) memset(tx, 0xFF, 6);                     // request -> broadcast
    else         memcpy(tx, target_mac, 6);
    memcpy(tx + 6, net_mac, 6);
    wr16(tx + 12, ARP_ETHERTYPE);

    uint8_t *a = tx + ETH_HDR_LEN;
    wr16(a + 0, 1);                 // hw type = Ethernet
    wr16(a + 2, IPV4_ETHERTYPE);    // proto type
    a[4] = 6; a[5] = 4;             // hw/proto address lengths
    wr16(a + 6, op);
    memcpy(a + 8,  net_mac,  6);
    memcpy(a + 14, g_net_ip, 4);
    if (op == 2) memcpy(a + 18, target_mac, 6);           // reply: requester HW
    memcpy(a + 24, target_ip, 4);

    eth_send(60);
    g_net_stats.tx_arp++;
}

int net_arp_lookup(const uint8_t ip[4], uint8_t mac_out[6])
{
    if (is_mcast(ip)) { net_mcast_mac(ip, mac_out); return 1; }
    if (is_bcast(ip)) { memset(mac_out, 0xFF, 6);   return 1; }

    for (int i = 0; i < ARP_CACHE_N; i++)
        if (arp_cache[i].valid && eq4(arp_cache[i].ip, ip)) {
            memcpy(mac_out, arp_cache[i].mac, 6);
            return 1;
        }

    g_net_stats.arp_misses++;
    arp_send(1, ip, NULL);                                // who-has
    return 0;
}

static void arp_rx(const uint8_t *f, uint32_t len)
{
    if (len < ETH_HDR_LEN + 28) return;
    const uint8_t *a = f + ETH_HDR_LEN;
    if (rd16(a) != 1 || rd16(a + 2) != IPV4_ETHERTYPE || a[4] != 6 || a[5] != 4)
        return;

    uint16_t op = rd16(a + 6);
    arp_learn(a + 14, a + 8);                             // sender IP/MAC
    g_net_stats.rx_arp++;

    if (op == 1 && eq4(a + 24, g_net_ip))                 // who-has us
        arp_send(2, a + 14, a + 8);
}

// ---------------------------------------------------------------------------
// IGMPv2
//
// Only what is needed for a switch to forward our groups: send a membership
// report on join, and answer queries. No leave (harmless -- the group ages out),
// no v3, no querier election.
// ---------------------------------------------------------------------------

#define IGMP_N              6
#define IGMP_QUERY          0x11
#define IGMP_REPORT_V2      0x16

static uint8_t  igmp_groups[IGMP_N][4];
static uint8_t  igmp_count;

static void igmp_report(const uint8_t group[4])
{
    uint8_t *tx = tx_buf();
    memset(tx, 0, 60);

    uint8_t gmac[6];
    net_mcast_mac(group, gmac);
    memcpy(tx, gmac, 6);
    memcpy(tx + 6, net_mac, 6);
    wr16(tx + 12, IPV4_ETHERTYPE);

    uint8_t *ip = tx + ETH_HDR_LEN;
    // IHL 6: IGMP carries the Router Alert option, which is what makes
    // snooping switches actually look at the packet.
    ip[0] = 0x46; ip[1] = 0xC0;                           // v4, IHL 6, TOS 0xC0
    wr16(ip + 2, 24 + 8);                                 // total length
    wr16(ip + 4, ip_ident++); wr16(ip + 6, 0);
    ip[8] = 1;                                            // TTL 1
    ip[9] = IPPROTO_IGMP;
    wr16(ip + 10, 0);
    memcpy(ip + 12, g_net_ip, 4);
    memcpy(ip + 16, group, 4);
    wr32(ip + 20, 0x94040000u);                           // Router Alert option
    wr16(ip + 10, net_checksum(ip, 24, 0));

    uint8_t *g = ip + 24;
    g[0] = IGMP_REPORT_V2; g[1] = 0;
    wr16(g + 2, 0);
    memcpy(g + 4, group, 4);
    wr16(g + 2, net_checksum(g, 8, 0));

    eth_send(ETH_HDR_LEN + 24 + 8);
    g_net_stats.tx_igmp++;
}

int net_igmp_join(const uint8_t group[4])
{
    for (int i = 0; i < igmp_count; i++)
        if (eq4(igmp_groups[i], group)) return 0;         // already joined
    if (igmp_count >= IGMP_N) return -1;
    memcpy(igmp_groups[igmp_count++], group, 4);
    igmp_report(group);
    return 0;
}

static void igmp_rx(const uint8_t *p, uint32_t len)
{
    if (len < 8) return;
    g_net_stats.rx_igmp++;
    if (p[0] != IGMP_QUERY) return;

    // General query (group 0.0.0.0) -> report everything; group-specific ->
    // report just that one. Real implementations randomise the response over
    // the max-response-time to avoid a storm; with <=6 groups on a small bench
    // network an immediate report is fine and much easier to reason about.
    for (int i = 0; i < igmp_count; i++)
        if (!p[4] && !p[5] && !p[6] && !p[7])
            igmp_report(igmp_groups[i]);
        else if (eq4(igmp_groups[i], p + 4))
            igmp_report(igmp_groups[i]);
}

// ---------------------------------------------------------------------------
// UDP
// ---------------------------------------------------------------------------

#define UDP_BINDINGS  8

typedef struct { uint16_t port; net_udp_handler_t fn; } udp_binding_t;
static udp_binding_t udp_bindings[UDP_BINDINGS];

int net_udp_bind(uint16_t port, net_udp_handler_t handler)
{
    for (int i = 0; i < UDP_BINDINGS; i++)
        if (udp_bindings[i].port == port) { udp_bindings[i].fn = handler; return 0; }
    for (int i = 0; i < UDP_BINDINGS; i++)
        if (!udp_bindings[i].port) {
            udp_bindings[i].port = port;
            udp_bindings[i].fn   = handler;
            return 0;
        }
    return -1;
}

uint8_t *net_udp_payload_buf(void)
{
    return tx_buf() + ETH_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN;
}

// Build headers around a payload already sitting in the TX buffer.
int net_udp_commit(const uint8_t dst_ip[4], uint16_t dst_port,
                   uint16_t src_port, uint32_t len, uint8_t tos)
{
    if (len > NET_MAX_PAYLOAD) { g_net_stats.tx_dropped++; return -1; }

    uint8_t dmac[6];
    if (!net_arp_lookup(dst_ip, dmac)) { g_net_stats.tx_dropped++; return -1; }

    uint8_t *tx = tx_buf();
    memcpy(tx, dmac, 6);
    memcpy(tx + 6, net_mac, 6);
    wr16(tx + 12, IPV4_ETHERTYPE);

    uint8_t *ip = tx + ETH_HDR_LEN;
    ip[0] = 0x45; ip[1] = tos;
    wr16(ip + 2, (uint16_t)(IP_HDR_LEN + UDP_HDR_LEN + len));
    // Real Dante audio uses IP ID 0 (captures/README.md). Non-audio traffic
    // gets an incrementing ID; neither matters while we never fragment.
    wr16(ip + 4, (tos == NET_TOS_AUDIO) ? 0 : ip_ident++);
    wr16(ip + 6, 0);                                      // no flags/fragment
    ip[8] = is_mcast(dst_ip) ? 32 : 64;                   // TTL
    ip[9] = IPPROTO_UDP;
    wr16(ip + 10, 0);
    memcpy(ip + 12, g_net_ip, 4);
    memcpy(ip + 16, dst_ip,   4);
    wr16(ip + 10, net_checksum(ip, IP_HDR_LEN, 0));

    uint8_t *udp = ip + IP_HDR_LEN;
    wr16(udp + 0, src_port);
    wr16(udp + 2, dst_port);
    wr16(udp + 4, (uint16_t)(UDP_HDR_LEN + len));
    // UDP checksum is OPTIONAL over IPv4 and real Dante transmits 0x0000 on its
    // audio flows (confirmed in captures/README.md), so we skip it everywhere.
    // If a peer ever rejects that, compute it here over the pseudo-header.
    wr16(udp + 6, 0);

    eth_send(ETH_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN + len);
    g_net_stats.tx_udp++;
    return 0;
}

int net_udp_send(const uint8_t dst_ip[4], uint16_t dst_port, uint16_t src_port,
                 const uint8_t *payload, uint32_t len, uint8_t tos)
{
    if (len > NET_MAX_PAYLOAD) { g_net_stats.tx_dropped++; return -1; }
    // Resolve BEFORE copying: on an ARP miss this avoids filling the slot with
    // a payload we are about to throw away.
    uint8_t dmac[6];
    if (!net_arp_lookup(dst_ip, dmac)) { g_net_stats.tx_dropped++; return -1; }
    memcpy(net_udp_payload_buf(), payload, len);
    return net_udp_commit(dst_ip, dst_port, src_port, len, tos);
}

// ---------------------------------------------------------------------------
// ICMP — echo reply only
//
// Not needed by Dante at all. It is here because `ping` answering is the
// cheapest end-to-end proof that RX classification, checksums and the TX path
// all work, and it costs ~30 lines.
// ---------------------------------------------------------------------------

static void icmp_rx(const uint8_t *src_ip, const uint8_t *p, uint32_t len)
{
    if (len < 8 || p[0] != 8) return;                     // echo request only
    if (len > NET_MAX_PAYLOAD) return;
    g_net_stats.rx_icmp++;

    uint8_t *out = net_udp_payload_buf();                 // scratch, reused
    memcpy(out, p, len);
    out[0] = 0;                                           // type 0 = echo reply
    out[2] = 0; out[3] = 0;
    uint16_t ck = net_checksum(out, len, 0);
    wr16(out + 2, ck);

    uint8_t dmac[6];
    if (!net_arp_lookup(src_ip, dmac)) return;

    uint8_t *tx = tx_buf();
    // out points into this same buffer past the headers, so build around it.
    memmove(tx + ETH_HDR_LEN + IP_HDR_LEN, out, len);
    memcpy(tx, dmac, 6);
    memcpy(tx + 6, net_mac, 6);
    wr16(tx + 12, IPV4_ETHERTYPE);

    uint8_t *ip = tx + ETH_HDR_LEN;
    ip[0] = 0x45; ip[1] = 0;
    wr16(ip + 2, (uint16_t)(IP_HDR_LEN + len));
    wr16(ip + 4, ip_ident++); wr16(ip + 6, 0);
    ip[8] = 64; ip[9] = IPPROTO_ICMP;
    wr16(ip + 10, 0);
    memcpy(ip + 12, g_net_ip, 4);
    memcpy(ip + 16, src_ip,   4);
    wr16(ip + 10, net_checksum(ip, IP_HDR_LEN, 0));

    eth_send(ETH_HDR_LEN + IP_HDR_LEN + len);
    g_net_stats.tx_icmp++;
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

// Is this IPv4 destination addressed to us?
static int ip_for_us(const uint8_t d[4])
{
    if (eq4(d, g_net_ip)) return 1;
    if (is_bcast(d))      return 1;
    if (is_mcast(d))      return 1;      // filtered per-port by the bindings
    if (g_net_prefix == 24) {
        if (d[0]==g_net_ip[0] && d[1]==g_net_ip[1] && d[2]==g_net_ip[2] && d[3]==255) return 1;
    } else {
        if (d[0]==g_net_ip[0] && d[1]==g_net_ip[1] && d[2]==255 && d[3]==255) return 1;
    }
    return 0;
}

int net_rx_frame(const uint8_t *frame, uint32_t len)
{
    if (len < ETH_HDR_LEN) return 0;
    uint16_t et = rd16(frame + 12);

    if (et == ARP_ETHERTYPE) { arp_rx(frame, len); return 1; }
    if (et != IPV4_ETHERTYPE) return 0;
    if (len < ETH_HDR_LEN + IP_HDR_LEN) return 0;

    const uint8_t *ip = frame + ETH_HDR_LEN;
    if ((ip[0] >> 4) != 4) return 0;
    uint32_t ihl = (uint32_t)(ip[0] & 0x0F) * 4u;
    if (ihl < IP_HDR_LEN || len < ETH_HDR_LEN + ihl) return 0;

    uint32_t total = rd16(ip + 2);
    if (total < ihl || len < ETH_HDR_LEN + total) {
        // Padded runt frames are normal (Ethernet minimum is 60), so trust the
        // IP total_length when it fits, and drop only when it overruns.
        if (len < ETH_HDR_LEN + total) { g_net_stats.rx_dropped++; return 1; }
    }

    const uint8_t *src_ip = ip + 12;
    const uint8_t *dst_ip = ip + 16;
    if (!ip_for_us(dst_ip)) return 1;                     // ours to ignore

    g_net_stats.rx_ip++;
    arp_learn(src_ip, frame + 6);                         // free ARP learning

    const uint8_t *pl  = ip + ihl;
    uint32_t       plen = total - ihl;

    switch (ip[9]) {
    case IPPROTO_ICMP:
        icmp_rx(src_ip, pl, plen);
        return 1;
    case IPPROTO_IGMP:
        igmp_rx(pl, plen);
        return 1;
    case IPPROTO_UDP: {
        if (plen < UDP_HDR_LEN) return 1;
        uint16_t sport = rd16(pl + 0);
        uint16_t dport = rd16(pl + 2);
        uint16_t ulen  = rd16(pl + 4);
        if (ulen < UDP_HDR_LEN || ulen > plen) { g_net_stats.rx_dropped++; return 1; }
        g_net_stats.rx_udp++;
        for (int i = 0; i < UDP_BINDINGS; i++)
            if (udp_bindings[i].port == dport && udp_bindings[i].fn) {
                udp_bindings[i].fn(src_ip, dst_ip, sport,
                                   pl + UDP_HDR_LEN, ulen - UDP_HDR_LEN);
                return 1;
            }
        return 1;
    }
    default:
        return 1;
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void net_init(const uint8_t mac[6], const uint8_t ip[4], uint8_t prefix)
{
    memcpy(net_mac, mac, 6);
    if (ip) memcpy(g_net_ip, ip, 4);
    if (prefix) g_net_prefix = prefix;
    memset(arp_cache, 0, sizeof(arp_cache));
    memset(&g_net_stats, 0, sizeof(g_net_stats));
    igmp_count = 0;
    txslot = 0;
    printf("[net] %u.%u.%u.%u/%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
           g_net_ip[0], g_net_ip[1], g_net_ip[2], g_net_ip[3], g_net_prefix,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void net_set_ip(const uint8_t ip[4], uint8_t prefix)
{
    memcpy(g_net_ip, ip, 4);
    if (prefix) g_net_prefix = prefix;
}

void net_poll(void)
{
    // Nothing periodic yet: IGMP reports are sent on join and on query, and the
    // ARP cache is refreshed by learning rather than aged out. Kept as a call
    // site so Phase 3 can hang mDNS re-announce and heartbeat timers here.
}

// ---------------------------------------------------------------------------
// "a.b.c.d/prefix" <-> g_net_ip, retained from osc.c for console + NV config
// ---------------------------------------------------------------------------

static int u8str(char *b, unsigned v)
{
    if (v >= 100){ b[0]='0'+v/100; b[1]='0'+(v/10)%10; b[2]='0'+v%10; return 3; }
    if (v >= 10) { b[0]='0'+v/10;  b[1]='0'+v%10; return 2; }
    b[0]='0'+v; return 1;
}

int net_ip_str(char *buf, int maxlen)
{
    (void)maxlen;
    int n = 0;
    for (int k = 0; k < 4; k++){ n += u8str(buf+n, g_net_ip[k]); if (k<3) buf[n++]='.'; }
    buf[n++] = '/'; n += u8str(buf+n, g_net_prefix); buf[n] = 0;
    return n;
}

int net_parse_ipstr(const char *s)
{
    unsigned oct[4], val = 0, pfx = 0; int oi = 0, dig = 0, havep = 0;
    const char *p = s;
    for (;;) {
        if (*p>='0' && *p<='9'){ val = val*10 + (unsigned)(*p-'0'); if (val>255) return 0; dig=1; p++; }
        else if (*p=='.'){ if (!dig || oi>=3) return 0; oct[oi++]=val; val=0; dig=0; p++; }
        else break;
    }
    if (!dig || oi != 3) return 0;
    oct[oi++] = val;
    if (*p=='/'){ p++; val=0; dig=0;
        while (*p>='0' && *p<='9'){ val=val*10+(unsigned)(*p-'0'); dig=1; p++; }
        if (dig){ pfx=val; havep=1; } }
    if (*p!=0 && *p!='\n' && *p!='\r' && *p!=' ') return 0;
    for (int k=0;k<4;k++) g_net_ip[k] = (uint8_t)oct[k];
    if (havep) g_net_prefix = (pfx >= 20) ? 24 : 16;
    return 1;
}
