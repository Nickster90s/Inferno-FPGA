// On-device capture of the Dante control plane — Phase 3/4 debug tool.
//
// WHY THIS EXISTS, and why host-side tcpdump is not a substitute.
//
// Dante Controller talks to this board over UNICAST. The bench switch is
// unmanaged: it floods multicast and broadcast, but forwards unicast only to
// the destination port. So a tcpdump on the build host sees our multicast
// (mDNS, heartbeat, device info) and NONE of the controller's requests to us,
// nor our replies. Every "DC sent us nothing" conclusion drawn from a host
// capture during this bring-up was measuring the switch, not DC -- including a
// 45 s capture on port 4440 that caught zero packets while the board's console
// was logging ARC requests the whole time.
//
// The board is the only place both directions are visible. This records them
// and ships the ring to the host over UDP, so the exchange can be decoded
// properly instead of being read as console hex.
//
// It was previously an AVB tool, filtering MSRP (0x22ea) and ADP/ACMP (0x22f0).
// Under Dante none of those ethertypes occur, so it had been silently recording
// nothing since Phase 0.

#include "cap.h"
#include "net.h"
#include "gptp.h"      // gptp_uptime_ms()
#include <stdio.h>
#include <string.h>

// 64 x 256 B ~= 16.6 KB. 256 bytes holds a full ARC response header plus the
// start of its payload, which is what identifies an exchange; the recorded
// length field still reports the true on-wire size when truncated.
#define CAP_N      128
#define CAP_BYTES  256

// Trigger and delivery ports. The host sends anything to CAP_REQ_PORT and the
// ring comes back on CAP_OUT_PORT, so a dump needs no console access -- which
// matters because the UART belongs to the user's picocom.
#define CAP_REQ_PORT  7778
#define CAP_OUT_PORT  9997

typedef struct {
    uint32_t t_ms;
    uint8_t  dir;              // 0 = RX, 1 = TX
    uint16_t len;              // true on-wire length
    uint16_t caplen;           // bytes actually stored
    uint8_t  data[CAP_BYTES];
} cap_entry_t;

static cap_entry_t cap_ring[CAP_N];
static uint16_t    cap_head;        // next slot to write
static uint32_t    cap_total;       // frames recorded since reset
static uint8_t     cap_wrapped;

void cap_set_eid(const uint8_t *eid) { (void)eid; }   // AVB leftover; unused

// Control plane only. Deliberately EXCLUDES 4321 (multicast audio): that is
// 99.2% of frames on this network and would evict the exchange we care about
// within milliseconds. Also excludes our own mDNS and heartbeat multicast,
// which the host can already see.
static int cap_is_control(const uint8_t *f, uint32_t len)
{
    if (len < 42) return 0;
    if (f[12] != 0x08 || f[13] != 0x00) return 0;      // IPv4
    uint32_t ihl = (uint32_t)(f[14] & 0x0F) * 4;
    if (f[23] != 17) return 0;                         // UDP
    uint32_t u = 14 + ihl;
    if (len < u + 8) return 0;

    uint16_t sp = (uint16_t)((f[u] << 8) | f[u + 1]);
    uint16_t dp = (uint16_t)((f[u + 2] << 8) | f[u + 3]);

    static const uint16_t ports[] = {
        4440,   // ARC   — routing/control, where DC asks the questions
        8800,   // CMC
        8700,   // info request / clock stats
        4455,   // flow control -- the unicast setup lands here
        319, 320 // PTPv1 event/general
    };

    int hit = 0;
    for (unsigned i = 0; i < sizeof(ports) / sizeof(ports[0]); i++)
        if (sp == ports[i] || dp == ports[i]) { hit = 1; break; }
    if (!hit) return 0;

    // Drop ARC opcode 0x4100 from the capture.
    //
    // Refreshing the Clock Status tab makes Dante Controller emit THOUSANDS of
    // these -- 7328 frames in one refresh, ~1 ms apart, with INCREMENTING
    // sequence numbers, so they are distinct requests rather than retries of
    // one. They filled the entire ring and hid every other exchange.
    //
    // Not a fault of ours: replaying DC's exact request bytes at both RedNets
    // returns the same 0x0030 we return (2809000a112241000030, byte for byte),
    // so real hardware rejects it too and DC evidently storms everyone with it.
    // Excluding it here is a scope decision about the capture, not a change to
    // what we answer on the wire.
    if ((sp == 4440 || dp == 4440) && len >= u + 8 + 8) {
        const uint8_t *pl = f + u + 8;
        uint16_t op1 = (uint16_t)((pl[6] << 8) | pl[7]);
        if (op1 == 0x4100) return 0;
    }
    return 1;
}

void cap_record(uint8_t dir, const uint8_t *frame, uint32_t len)
{
    // Anything INBOUND UNICAST is worth recording regardless of port: a
    // transmitter setting up a unicast flow may use a port we do not bind and
    // therefore would not think to look for, and missing the one packet that
    // explains the negotiation defeats the point of capturing. Multicast is
    // excluded below, so this cannot flood the ring; outbound still goes
    // through the control-plane port filter.
    // Drop the 0x4100 storm FIRST, before any catch-all. Dante Controller emits
    // thousands of these per refresh; the exclusion in cap_is_control was
    // already there for exactly that reason, and the inbound-unicast catch-all
    // below bypassed it and re-flooded the ring -- 7705 frames recorded in one
    // patch attempt, burying the exchange we were trying to observe.
    if (len >= 42 && frame[12] == 0x08 && frame[13] == 0x00 && frame[23] == 17) {
        uint32_t ihl = (uint32_t)(frame[14] & 0x0F) * 4;
        uint32_t u = 14 + ihl;
        if (len >= u + 16) {
            uint16_t sp = (uint16_t)((frame[u] << 8) | frame[u + 1]);
            uint16_t dp = (uint16_t)((frame[u + 2] << 8) | frame[u + 3]);
            if (sp == 4440 || dp == 4440) {
                const uint8_t *pl = frame + u + 8;
                if (((pl[6] << 8) | pl[7]) == 0x4100) return;
            }
        }
    }

    // IPv4 ONLY, and this check must come FIRST.
    //
    // The two tests below used to read frame[30] -- an IP destination byte --
    // without ever checking the ethertype. On a non-IP frame that byte is
    // arbitrary payload, so it was almost always < 224: every L2 frame passed
    // the inbound-unicast catch-all AND missed the multicast exclusion. An AVB
    // device on the bench emits gPTP (0x88f7) and MVRP/MMRP (0x88f5/0x88f6)
    // faster than 1 Hz, so the ring held 62 of those out of 64 entries and
    // wrapped every ~20 s. Every "the receiver sent us nothing" reading taken
    // through this ring was measuring an empty ring, not an idle network.
    //
    // The whole Dante control plane is IPv4/UDP, so nothing of interest is lost.
    if (len < 34 || frame[12] != 0x08 || frame[13] != 0x00) return;

    // L2 group bit, not a guess at the IP address. frame[0] bit 0 is set for
    // every multicast and broadcast destination MAC, which is the actual
    // question being asked and cannot be fooled by payload bytes.
    int unicast = (frame[0] & 0x01) == 0;

    if (!(!dir && unicast) && !cap_is_control(frame, len)) return;

    // UNICAST ONLY, in both directions.
    //
    // Multicast is flooded by the switch, so the build host can already see all
    // of it -- our announcements AND other devices' heartbeats. Recording it
    // here buys nothing and actively destroys the ring's value: our own 8
    // announcements every 3 s filled all 64 entries in ~24 s, and once those
    // were excluded the AM2's 1 Hz heartbeats did the same.
    //
    // What the host CANNOT see is unicast between the controller and this
    // board, because the switch forwards it only to the destination port. That
    // is the entire reason this ring exists, so that is all it now keeps.
    if (!unicast) return;

    // RING, not stop-when-full. The old first-N behaviour was right for a boot
    // handshake; here the interesting traffic happens when someone clicks in
    // Dante Controller, long after boot, so keep the most RECENT frames.
    cap_entry_t *e = &cap_ring[cap_head];
    cap_head = (uint16_t)((cap_head + 1) % CAP_N);
    if (cap_head == 0) cap_wrapped = 1;
    cap_total++;

    e->t_ms   = gptp_uptime_ms();
    e->dir    = dir;
    e->len    = (uint16_t)len;
    uint32_t n = len < CAP_BYTES ? len : CAP_BYTES;
    e->caplen = (uint16_t)n;
    memcpy(e->data, frame, n);
}

void cap_reset(void)
{
    cap_head = 0; cap_total = 0; cap_wrapped = 0;
    memset(cap_ring, 0, sizeof(cap_ring));
}

// Ship the ring to the requester, oldest first, two entries per datagram.
static void cap_send_ring(const uint8_t dst_ip[4], uint16_t dst_port)
{
    uint16_t start = cap_wrapped ? cap_head : 0;
    uint16_t count = cap_wrapped ? CAP_N : cap_head;

    for (uint16_t i = 0; i < count; i += 2) {
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = 0;

        p[n++] = (uint8_t)(cap_total >> 24); p[n++] = (uint8_t)(cap_total >> 16);
        p[n++] = (uint8_t)(cap_total >> 8);  p[n++] = (uint8_t)cap_total;
        p[n++] = (uint8_t)(i >> 8);          p[n++] = (uint8_t)i;
        p[n++] = (uint8_t)(count >> 8);      p[n++] = (uint8_t)count;

        for (uint16_t k = 0; k < 2 && (i + k) < count; k++) {
            const cap_entry_t *e = &cap_ring[(start + i + k) % CAP_N];
            p[n++] = (uint8_t)(e->t_ms >> 24); p[n++] = (uint8_t)(e->t_ms >> 16);
            p[n++] = (uint8_t)(e->t_ms >> 8);  p[n++] = (uint8_t)e->t_ms;
            p[n++] = e->dir;
            p[n++] = (uint8_t)(e->len >> 8);    p[n++] = (uint8_t)e->len;
            p[n++] = (uint8_t)(e->caplen >> 8); p[n++] = (uint8_t)e->caplen;
            memcpy(p + n, e->data, e->caplen);
            n += e->caplen;
        }
        net_udp_commit(dst_ip, dst_port, CAP_OUT_PORT, n, NET_TOS_BEST_EFFORT);
    }
}

static void cap_req_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                       uint16_t src_port, const uint8_t *payload, uint32_t len)
{
    (void)dst_ip; (void)src_port;
    // Payload byte 0: 'r' also resets the ring after dumping.
    cap_send_ring(src_ip, CAP_OUT_PORT);
    if (len >= 1 && payload[0] == 'r') cap_reset();
}

void cap_init(void)
{
    net_udp_bind(CAP_REQ_PORT, cap_req_rx);
    printf("[cap] control-plane capture: %u frames, request on :%u -> :%u\n",
           CAP_N, CAP_REQ_PORT, CAP_OUT_PORT);
}

// Console dump kept for the 'R' command, but deliberately terse: the UDP path
// is the one that carries full frames.
void cap_dump(void)
{
    uint16_t start = cap_wrapped ? cap_head : 0;
    uint16_t count = cap_wrapped ? CAP_N : cap_head;
    printf("[cap] %lu frames recorded, %u in ring\n",
           (unsigned long)cap_total, count);
    for (uint16_t i = 0; i < count; i++) {
        const cap_entry_t *e = &cap_ring[(start + i) % CAP_N];
        uint32_t ihl = (uint32_t)(e->data[14] & 0x0F) * 4;
        uint32_t u = 14 + ihl;
        uint16_t sp = (uint16_t)((e->data[u] << 8) | e->data[u + 1]);
        uint16_t dp = (uint16_t)((e->data[u + 2] << 8) | e->data[u + 3]);
        printf("  %8lu ms %s %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u len=%u\n",
               (unsigned long)e->t_ms, e->dir ? "TX" : "RX",
               e->data[26], e->data[27], e->data[28], e->data[29], sp,
               e->data[30], e->data[31], e->data[32], e->data[33], dp,
               e->len);
    }
}
