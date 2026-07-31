// Dante multicast audio talker — Phase 5.
//
// Binds the gateware packetizer to 6 multicast bundles of 8 channels and turns
// it on once the PTPv1 clock is locked. Replaces the AVB talker setup, which
// bound AVTP stream identities learned from SRP/ACMP.
//
// WHY MULTICAST BUNDLES, and why this file is short:
//
// A Dante receiver resolving one of our channels queries
// <chan>@<host>._netaudio-chan._udp, sees a "b.<bundle>=<pos+1>" TXT key,
// queries <bundle>@<host>._netaudio-bund._udp for a.0/p.0, and joins that
// group. IT NEVER CONTACTS US. So there is no dynamic flow-control server, no
// per-flow channel negotiation, no unicast keepalive, no receiver ARP and no
// per-flow header recomputation -- the whole subscribe path is mDNS.
//
// CAVEAT: mdns.c does NOT yet serve the records that path needs (see
// dante_tx_init). Audio is emitted correctly; nothing can subscribe to it yet.
//
// The header is therefore CONSTANT for the life of a flow, which is what lets
// the IPv4 checksum be computed here once instead of in gateware.

#include "dante_tx.h"
#include "dante_dev.h"
#include "ptpv1.h"
#include "gptp.h"
#include "dante_flows.h"
#include "net.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

dante_tx_stats_t g_tx_stats;

#define N_FLOWS         (DANTE_TX_CHANNELS / 8)     // 6
#define FPP             16
#define BYTES_PER_SAMP  3

// Multicast group base. Dante audio lives in 239.255.0.0/16; real devices on the
// bench use 239.255.x.y with the low bytes derived per flow. We take a block
// keyed off our own MAC so two boards on one network do not collide, then one
// consecutive address per bundle.
#define MCAST_A         239
#define MCAST_B         255

static uint8_t  flow_ip[N_FLOWS][4];
static uint8_t  talker_on;

// ---------------------------------------------------------------------------
// IPv4 header checksum, computed once per flow.
//
// The gateware emits the header verbatim from CSRs, so this must match it byte
// for byte: version/IHL, TOS, total length, ID 0, no fragmentation, TTL, proto
// 17, checksum field itself zero, then src and dst addresses.
// ---------------------------------------------------------------------------

static uint16_t ip_header_checksum(const uint8_t src[4], const uint8_t dst[4],
                                   uint16_t total_len, uint8_t tos, uint8_t ttl)
{
    uint32_t sum = 0;

    sum += (0x4500 | tos) & 0xFFFF;      // version 4, IHL 5, TOS
    sum += total_len;
    sum += 0x0000;                       // identification
    sum += 0x0000;                       // flags / fragment offset
    sum += ((uint32_t)ttl << 8) | 17;    // TTL, protocol UDP
    // checksum field contributes zero while computing
    sum += ((uint32_t)src[0] << 8) | src[1];
    sum += ((uint32_t)src[2] << 8) | src[3];
    sum += ((uint32_t)dst[0] << 8) | dst[1];
    sum += ((uint32_t)dst[2] << 8) | dst[3];

    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum & 0xFFFF);
}

// ---------------------------------------------------------------------------
// Flow binding
// ---------------------------------------------------------------------------

static void bind_flow(unsigned f)
{
    const uint16_t ip_total = 20 + 8 + 9 + (FPP * 8 * BYTES_PER_SAMP);   // 421

    uint8_t *dip = flow_ip[f];

    // IPv4 multicast -> Ethernet multicast: 01:00:5e plus the LOW 23 BITS of
    // the group address. Bit 23 is deliberately discarded -- 32 IPv4 groups
    // share each MAC, which is exactly why receivers must still filter on the
    // IP address and why we must not invent a mapping of our own.
    uint8_t dmac[6] = {
        0x01, 0x00, 0x5E,
        (uint8_t)(dip[1] & 0x7F), dip[2], dip[3]
    };

    uint16_t csum = ip_header_checksum(g_net_ip, dip, ip_total,
                                       DANTE_TX_IP_TOS, DANTE_TX_IP_TTL);

    // Select the context FIRST; every per-flow CSR below is indirect through it.
    aaf_pkt_ctx_select_write(f);

    aaf_pkt_dst_ip_write((uint32_t)dip[0] << 24 | (uint32_t)dip[1] << 16 |
                              (uint32_t)dip[2] << 8  | dip[3]);
    aaf_pkt_ip_csum_write(csum);
    aaf_pkt_udp_sport_write(DANTE_PORT_MEDIA);

    aaf_pkt_dst_mac_hi_write(((uint32_t)dmac[0] << 8) | dmac[1]);
    // dst_mac_lo LAST of the MAC pair -- it is the latch trigger, matching the
    // AVB binding order.
    aaf_pkt_dst_mac_lo_write(((uint32_t)dmac[2] << 24) | ((uint32_t)dmac[3] << 16) |
                                  ((uint32_t)dmac[4] << 8)  |  dmac[5]);

    // udp_dport LAST of all: writing it latches dst_ip/ip_csum/udp_sport into
    // the context, the same way stream_id_lo latched the AVTP pair.
    aaf_pkt_udp_dport_write(DANTE_PORT_MEDIA);

    printf("[dtx] flow %u -> %u.%u.%u.%u:%u  mac %02x:%02x:%02x:%02x:%02x:%02x  csum %04x\n",
           f, dip[0], dip[1], dip[2], dip[3], DANTE_PORT_MEDIA,
           dmac[0], dmac[1], dmac[2], dmac[3], dmac[4], dmac[5], csum);
}

void dante_tx_init(void)
{
    // One group per bundle, keyed off our MAC so two boards do not collide.
    for (unsigned f = 0; f < N_FLOWS; f++) {
        flow_ip[f][0] = MCAST_A;
        flow_ip[f][1] = MCAST_B;
        flow_ip[f][2] = g_dante.mac[4];
        flow_ip[f][3] = (uint8_t)(g_dante.mac[5] + f);
    }

    // Shared (non-indirect) fields.
    aaf_pkt_src_ip_write((uint32_t)g_net_ip[0] << 24 | (uint32_t)g_net_ip[1] << 16 |
                              (uint32_t)g_net_ip[2] << 8  | g_net_ip[3]);
    aaf_pkt_src_mac_hi_write(((uint32_t)g_dante.mac[0] << 8) | g_dante.mac[1]);
    aaf_pkt_src_mac_lo_write(((uint32_t)g_dante.mac[2] << 24) | ((uint32_t)g_dante.mac[3] << 16) |
                                  ((uint32_t)g_dante.mac[4] << 8)  |  g_dante.mac[5]);
    aaf_pkt_ip_tos_write(DANTE_TX_IP_TOS);
    aaf_pkt_ip_ttl_write(DANTE_TX_IP_TTL);

    // CLOCK IN THE PAST, NOT THE FUTURE. flows_tx.rs:44 is unusually direct
    // about this: "it's better to have the clock in the past than in the future
    // - otherwise Dante devices receiving from us go mad and fart." The offset
    // is in samples; -24 is half a packet at fpp=16.
    // ZERO, deliberately -- see ts_anchor(). This CSR adds into an unsigned
    // subsecond field with no carry into seconds, so any nonzero value wraps
    // at one end of the second or the other. The offset lives in the anchor.
    aaf_pkt_ts_offset_write(0);

    for (unsigned f = 0; f < N_FLOWS; f++) bind_flow(f);

    // NOT YET PUBLISHED: the _netaudio-bund records that carry a.0/p.0 for
    // these groups, and the b.<bundle>= keys on the _netaudio-chan records.
    //
    // mdns.c currently serves only _netaudio-arc and _netaudio-cmc, which is
    // what Dante Controller needs to SHOW the device -- and is why Phase 3
    // looked complete. It is not what a RECEIVER needs to subscribe: a receiver
    // resolves <chan>@<host>._netaudio-chan._udp, reads b.<bundle>=<pos+1>,
    // resolves <bundle>@<host>._netaudio-bund._udp for a.0/p.0, and joins that
    // group. With neither record served, these flows are emitted correctly and
    // nothing can find them.
    //
    // The audio path below is complete and testable without it -- the packets
    // are on the wire and tools/dante_decode.py can verify them -- but end-to-end
    // subscription needs those records. Tracked as its own task.

    talker_on = 0;
    printf("[dtx] %u flows x 8 ch, fpp=%u, 24-bit, %u pps/flow (talker held off)\n",
           N_FLOWS, FPP, 48000u / FPP);
}

// ---------------------------------------------------------------------------
// Talker gate
// ---------------------------------------------------------------------------

// Load the media-clock timestamp counter from the PTP clock.
//
// The counters free-run from reset, so without this the header carries seconds
// since BOOT while every other device carries seconds since the grandmaster's
// epoch -- measured 426 against the A16R's 84625. A receiver subscribes fine
// and then discards every packet as hours stale: green patch, no audio.
//
// sub = ns * 48000 / 1e9 = ns * 3 / 62500, exact and small enough for 32 bits
// (ns < 1e9, so ns*3 < 3e9 -- unsigned, and it must stay unsigned).
static void ts_anchor(void)
{
    ptp_timestamp_t t = gptp_read_time();
    int32_t  sub = (int32_t)((t.nanoseconds * 3u) / 62500u);
    uint32_t sec = (uint32_t)t.seconds;

    // THE OFFSET IS APPLIED HERE, NOT IN THE ts_offset CSR.
    //
    // The gateware computes the emitted timestamp as ts_sub - (fpp-1) and the
    // CSR offset was added to that, in a 32-bit UNSIGNED field with no carry
    // into seconds. With the old -32 the sum went negative twice a second (at
    // ts_sub 15 and 31) and wrapped:
    //
    //     sec=85673 subsec=4294967264      <- -32, unsigned
    //
    // Measured 6 of these in a 3-second capture of one flow: 2 per second per
    // flow, 12 per second across six flows, each ~4.29e9 samples in the
    // future. A positive CSR offset has the mirror bug at the top of the
    // second (subsec >= 48000). Only zero is safe there.
    //
    // Shifting the ANCHOR instead is exact: the counter is a real (sec, sub)
    // pair, so the carry is done here in C where seconds exist, and the
    // emitted value is always ts_sub - 15 with ts_sub congruent to 15 mod 16 --
    // in [0, 47984], a multiple of fpp, and never wrapping either way.
    sub += DANTE_TX_TS_OFFSET;
    if (sub < 0)            { sub += 48000; sec -= 1; }
    else if (sub >= 48000)  { sub -= 48000; sec += 1; }

    aaf_pkt_ts_load_sec_write(sec);
    aaf_pkt_ts_load_sub_write((uint32_t)sub);
    aaf_pkt_ts_load_write(1);
    g_tx_stats.anchors++;
    printf("[dtx] media clock anchored to PTP %lu.%09lu -> %lu.%lu (offset %d)\n",
           (unsigned long)t.seconds, (unsigned long)t.nanoseconds,
           (unsigned long)sec, (unsigned long)sub, DANTE_TX_TS_OFFSET);
}

void dante_tx_poll(void)
{
    // TRANSMIT ONLY WHEN SOMETHING HAS ASKED FOR A FLOW.
    //
    // We used to source all six multicast bundles the moment PTP locked,
    // regardless of whether anyone had subscribed. Measured on the segment
    // that was 65.5 Mbit/s of 69.6 Mbit/s total -- 94% of all traffic, 65% of
    // a 100 Mbit link -- and an unmanaged switch floods every group to every
    // port, so the A16R was filtering 65 Mbit/s in hardware while playing two
    // channels. Sending audio nobody has requested is not free; it is most of
    // the network.
    // `active`, not `accepted`. Answering a request is not serving it: we reply
    // OK to keep the receiver's state machine moving, but until a flow is
    // actually built there is nothing to send, and enabling the talker would
    // put all six multicast bundles back on the wire for nobody.
    uint8_t want = g_ptpv1.locked && (g_flows_stats.active > 0);

    // Re-anchor when the emitted seconds drifts from PTP by more than a second.
    // The media clock is rate-disciplined by mcr, so this should never fire in
    // steady state; it exists to recover from a PTP step, which moves absolute
    // time out from under a counter that only ever increments.
    if (talker_on && want) {
        ptp_timestamp_t t = gptp_read_time();
        uint32_t emitted = aaf_pkt_dbg_last_sec_read();
        uint32_t now_s   = (uint32_t)t.seconds;
        uint32_t diff    = (emitted > now_s) ? (emitted - now_s) : (now_s - emitted);
        if (diff > 1) {
            printf("[dtx] media clock %lu vs PTP %lu -- re-anchoring\n",
                   (unsigned long)emitted, (unsigned long)now_s);
            ts_anchor();
        }
    }

    if (want == talker_on) return;

    if (want) {
        // Anchor BEFORE enabling. Enabling first would put a burst of
        // wrong-epoch packets on the wire, and a receiver that has already
        // decided our timestamps are nonsense may not re-evaluate.
        ts_anchor();
        aaf_pkt_enable_write(1);
        talker_on = 1;
        g_tx_stats.enables++;
        printf("[dtx] clock locked -- talker ENABLED\n");
    } else {
        aaf_pkt_enable_write(0);
        talker_on = 0;
        g_tx_stats.disables++;
        printf("[dtx] clock unlocked -- talker OFF\n");
    }
}

uint8_t dante_tx_enabled(void) { return talker_on; }

// mdns.c needs these to build the _netaudio-bund a.0= records: a receiver reads
// b.<bundle>= off a channel record, resolves the bundle record, and joins the
// group named there. The group must be the one we actually transmit to, so it
// comes from here rather than being recomputed.
const uint8_t *dante_tx_flow_ip(unsigned f)
{
    return flow_ip[f < N_FLOWS ? f : 0];
}
unsigned dante_tx_flows(void) { return N_FLOWS; }

void dante_tx_report(void)
{
    printf("[dtx] talker=%u packets=%lu underrun=%lu overrun=%lu level=%lu\n",
           talker_on,
           (unsigned long)aaf_pkt_packet_count_read(),
           (unsigned long)aaf_pkt_underrun_count_read(),
           (unsigned long)aaf_pkt_overrun_count_read(),
           (unsigned long)aaf_pkt_fifo_level_read());
    printf("[dtx] last ts = %lu.%lu (sec.subsec_samples)\n",
           (unsigned long)aaf_pkt_dbg_last_sec_read(),
           (unsigned long)aaf_pkt_dbg_last_ts_read());
}
