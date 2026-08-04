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
#include "mcr.h"
#include "dante_flows.h"
#include "net.h"
#include "telem.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

dante_tx_stats_t g_tx_stats;

// How long PTP must stay locked before the media clock is anchored to it.
// The first lock edge is not trustworthy -- see dante_tx_poll.
#define PTP_SETTLE_MS    4000

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

    // nslots = 0: bound but NOT transmitting. The header fields are ready so a
    // bundle can be switched on later (0x2201, DC's "add a flow") without
    // re-deriving anything, but nothing goes on the wire until something asks.
    // This is what took us from 65.5 Mbit/s to 0.03 -- see dante_tx_poll.
    aaf_pkt_flow_cfg_write(0);

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
    telem_event(TELEM_E_ANCHOR, (int32_t)sec, (int32_t)sub);
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
    // WAIT FOR THE CLOCK TO SETTLE, not just to report locked once.
    //
    // The console shows why: the first lock is not stable. PTP locks, drops out
    // again (offset -21406 ns), relocks, and only afterwards applies its path
    // delay and a phase correction:
    //
    //   [ptpv1] LOCKED, offset -29 ns
    //   [dtx] media clock anchored ... / talker ENABLED     <- audio starts BAD
    //   [ptpv1] unlocked, offset -21406 ns
    //   [ptpv1] LOCKED, offset 1138 ns
    //   [ptpv1] path delay 21250 ns, phase corrected -1175 ns
    //                                                       <- ~1 s later, clean
    //
    // Anchoring on that first edge samples a clock that then moves underneath
    // it, and the media clock free-runs from the anchor, so it keeps whatever
    // error was present at that instant. Requiring the lock to HOLD, and the
    // path delay to have been measured, costs a few seconds of silence at
    // startup and starts clean instead of starting wrong and recovering.
    static uint32_t lock_since;
    static uint8_t  was_locked;
    if (g_ptpv1.locked && !was_locked) lock_since = gptp_uptime_ms();
    was_locked = g_ptpv1.locked;

    uint8_t settled = g_ptpv1.locked
                   && g_ptpv1.mean_path_delay_ns != 0
                   && (gptp_uptime_ms() - lock_since) >= PTP_SETTLE_MS;

    uint8_t want = settled && (dante_tx_active() > 0);

    // ---- MEDIA CLOCK RATE SERVO -------------------------------------------
    //
    // Measure the drift between what we STAMP and what PTP says, and trim the
    // media clock rate to null it. Measured residual after the gPTP addend
    // ratio was +4.34 ppm = +15.6 ms/hour: the stream accumulated 19.6 ms in
    // 1.4 hours and ~100 ms overnight, which is 20x to 100x a receiver's ~1 ms
    // buffer. Every counter stayed healthy while the audio was long dead.
    //
    // RATE, not re-anchoring. A constant offset is absorbed by the receiver as
    // latency; only accumulation is fatal. Re-anchoring would step the
    // timestamp every few seconds and every step is an audible click.
    //
    // PI on the drift, run once a second. Gains are deliberately small: the
    // error is parts per million and there is no hurry, so slow correction
    // beats overshoot that would itself be heard.
    if (talker_on && g_ptpv1.locked) {
        static uint32_t last_servo_ms;
        static int32_t  trim_ppb;
        uint32_t now = gptp_uptime_ms();
        if ((uint32_t)(now - last_servo_ms) >= 1000u &&
            (uint32_t)(now - last_servo_ms) <  60000u) {
            last_servo_ms = now;
            ptp_timestamp_t t = gptp_read_time();
            int64_t ptp_smp = (int64_t)t.seconds * 48000
                            + (int64_t)((t.nanoseconds * 3u) / 62500u);
            int64_t emit    = (int64_t)aaf_pkt_dbg_last_sec_read() * 48000
                            + (int64_t)aaf_pkt_dbg_last_ts_read();
            int32_t err = (int32_t)(emit - ptp_smp);     // + = we are ahead

            // ~20 ppb per sample of error, integrated. At the observed 4.3 ppm
            // this converges in a couple of minutes without overshooting.
            trim_ppb -= err * 20;
            if (trim_ppb >  50000) trim_ppb =  50000;
            if (trim_ppb < -50000) trim_ppb = -50000;
            // DISABLED. Enabling this made things far worse than the drift it
            // was meant to fix: the emitted timestamp fell 256617 samples
            // (5.3 s) behind PTP in 115 s -- a thousand times more than the
            // +/-50 ppm this trim can even produce, so the media clock or the
            // talker stops rather than merely running slow. Cause not found.
            // Reverted to the known-good behaviour (steady +4.3 ppm) rather
            // than left in a state that breaks audio outright.
            //
            // The measurement below still runs, so drift stays observable.
            // mcr_set_trim_ppb(trim_ppb);
            g_tx_stats.trim_ppb = trim_ppb;
            g_tx_stats.drift    = err;
        } else if ((uint32_t)(now - last_servo_ms) >= 60000u) {
            last_servo_ms = now;         // clock jumped; skip this round
        }
    }

    // Re-anchor on PHASE error measured in SAMPLES.
    //
    // This used to compare only the SECONDS field and fire on diff > 1, which
    // made it structurally blind to exactly the error that matters. Found
    // 2026-08-04 with no audio and a fully green patch in Dante Controller:
    //
    //     emitted 402927.04519   PTP 402926.41611   =  +10908 samples
    //                                               =  +227 ms INTO THE FUTURE
    //
    // 147x DANTE_TX_TS_OFFSET, and ~227x a receiver's buffer. Receivers accept
    // the subscription -- the control plane is unaffected -- and then discard
    // every audio packet as far-future. Silence, with every counter healthy:
    // ring centred, zero underruns, 9001 pps on the wire.
    //
    // A one-second threshold is ~1000x a receiver's latency setting. The
    // seconds field only differed by 1, and `diff > 1` needs 2, so it never
    // fired even at a quarter second out.
    //
    // This is a STEP and it is audible, so it is a coarse backstop only -- for
    // a PTP step, or for phase accumulated while the talker was idle. Steady
    // state is held by the slow phase term in mcr_dante.c, which corrects
    // without stepping.
    if (talker_on && want) {
        ptp_timestamp_t t = gptp_read_time();
        int64_t ptp_smp = (int64_t)t.seconds * 48000
                        + (int64_t)((t.nanoseconds * 3u) / 62500u);
        int64_t emit    = (int64_t)aaf_pkt_dbg_last_sec_read() * 48000
                        + (int64_t)aaf_pkt_dbg_last_ts_read();
        int64_t err = emit - ptp_smp - DANTE_TX_TS_OFFSET;
        if (err > DANTE_TX_REANCHOR_SAMPLES || err < -DANTE_TX_REANCHOR_SAMPLES) {
            printf("[dtx] media clock phase %ld samples (%ld ms) off "
                   "-- re-anchoring\n",
                   (long)err, (long)(err / 48));
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
        telem_event(TELEM_E_TALKER_ON, (int32_t)dante_tx_active(), 0);
        printf("[dtx] clock locked -- talker ENABLED\n");
    } else {
        aaf_pkt_enable_write(0);
        talker_on = 0;
        telem_event(TELEM_E_TALKER_OFF, 0, 0);
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

// ---------------------------------------------------------------------------
// Unicast flows
//
// A unicast flow differs from a multicast bundle in three ways, all of which
// come from the receiver's request rather than from us: the destination is the
// receiver's own address, the slot map is the channel list it named, and fpp is
// its choice. Measured from two real receivers asking for the same two
// channels -- A16R 4 slots [1,2,0,0] fpp 8, AM2 2 slots [1,2] fpp 16 -- which
// is why slot count, map and fpp are all per-context.
//
// The gateware side is unchanged in kind: same ctx_select indirection, same
// firmware-computed IP checksum. Multicast still works through exactly this
// path; a bundle is just a flow whose destination is a group address.
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  in_use;
    uint8_t  peer[4];
    uint32_t last_ms;
    uint32_t rebinds;      // times this context was re-bound from scratch
    uint8_t  dst[4];
    uint16_t dport;
    uint8_t  nslots;
    uint8_t  fpp;
    uint8_t  mcast;        // 1 = multicast bundle, 0 = unicast to a receiver
    uint16_t ext_id;       // flow id Dante Controller uses for this flow
    uint16_t chans[8];     // slot -> tx channel, 1-based, 0 = silent slot
    uint8_t  dmac[6];      // what we actually bound as the destination MAC
} flow_slot_t;
static flow_slot_t flows[N_FLOWS];

// Keepalives arrive about every 5 s; flows_control.rs calls a lapsed flow
// "stream expired (i.e. no keepalives)".
//
// 5 MINUTES. Once flows stopped being rejected, the receivers slowed their
// keepalives dramatically -- measured ages of 16-38 s where they had been 5 s
// while we were answering 0x0315 -- so 45 s still expired them mid-stream. Each
// teardown writes nslots = 0 and silences the flow until the next refresh, and
// it showed in the packet rate: 6944 pps against an expected 9000, with flow 1
// dead for most of a 20 s window.
//
// The only thing this timeout buys is releasing one of six contexts when a
// receiver leaves for good, and re-binding is keyed on peer IP so a returning
// receiver reclaims its own slot anyway. Erring long is nearly free.
//
// Earlier note, kept because the reasoning still applies:
// 45 s, not 16. At 16 s the expiry fired BETWEEN keepalives on real hardware --
// every flow was torn down and rebuilt on each refresh, which disables the
// context (nslots = 0) for the gap and would be audible. The only thing this
// timeout does is release a context when a receiver goes away for good, so
// erring long costs nothing and erring short costs audio.
#define FLOW_TIMEOUT_MS  300000



static void write_ctx(unsigned f, const uint8_t dst_ip[4], const uint8_t dmac[6],
                      uint16_t dport, const uint16_t *chans, uint8_t nslots,
                      uint8_t fpp)
{
    uint16_t ip_total = (uint16_t)(20 + 8 + 9 + nslots * fpp * BYTES_PER_SAMP);
    uint16_t csum = ip_header_checksum(g_net_ip, dst_ip, ip_total,
                                       DANTE_TX_IP_TOS, DANTE_TX_IP_TTL);

    aaf_pkt_ctx_select_write(f);

    uint32_t lo = 0, hi = 0;
    for (unsigned i = 0; i < 8; i++) {
        uint32_t ent = 0;
        if (i < nslots && chans[i] != 0 && chans[i] <= DANTE_TX_CHANNELS)
            ent = (uint32_t)((chans[i] - 1) & 0x3F) | 0x80u;   // valid bit
        if (i < 4) lo |= ent << (i * 8);
        else       hi |= ent << ((i - 4) * 8);
    }
    aaf_pkt_chmap_lo_write(lo);
    aaf_pkt_chmap_hi_write(hi);

    aaf_pkt_dst_ip_write((uint32_t)dst_ip[0] << 24 | (uint32_t)dst_ip[1] << 16 |
                         (uint32_t)dst_ip[2] << 8  | dst_ip[3]);
    aaf_pkt_ip_csum_write(csum);
    aaf_pkt_udp_sport_write(DANTE_PORT_MEDIA);
    aaf_pkt_dst_mac_hi_write(((uint32_t)dmac[0] << 8) | dmac[1]);
    aaf_pkt_dst_mac_lo_write(((uint32_t)dmac[2] << 24) | ((uint32_t)dmac[3] << 16) |
                             ((uint32_t)dmac[4] << 8)  |  dmac[5]);
    aaf_pkt_udp_dport_write(dport);

    // flow_cfg LAST: it latches the channel map with it, so the builder never
    // sees a half-written map.
    aaf_pkt_flow_cfg_write((uint32_t)(nslots & 0x0F) | ((fpp == 16) ? 0x10u : 0u));
}

// 4-byte IP compare. Not memcmp(): this picolibc build does not link one, which
// is why the surrounding code compares bytes by hand.
static inline int ip4_eq(const uint8_t a[4], const uint8_t b[4])
{
    return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3];
}

int dante_tx_bind_unicast(const uint8_t peer_ip[4], const uint8_t dst_ip[4],
                          uint16_t dst_port, const uint16_t *chans,
                          uint8_t nslots, uint8_t fpp)
{
    uint8_t dmac[6];
    // net_arp_lookup returns 1 on SUCCESS, 0 on miss -- not the 0-is-success
    // convention the rest of net.h uses (net_udp_bind, net_udp_commit). This
    // was inverted here and rejected every flow whose MAC we already knew, with
    // 0x0315 "too many TX flows" while all six contexts sat free.
    if (!net_arp_lookup(dst_ip, dmac)) {
        // The request itself came from this peer, so its MAC is in the cache
        // from that frame. If it somehow is not, refuse rather than transmit to
        // a broadcast MAC.
        printf("[dtx] no ARP entry for %u.%u.%u.%u\n",
               dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3]);
        return -1;
    }

    // Re-bind an existing flow with the SAME DESTINATION SOCKET rather than
    // allocating a new context: the ~5 s keepalive is the SAME request
    // repeated, and treating each one as a new flow would exhaust all six
    // contexts in half a minute.
    //
    // THE KEY IS (peer, dst_ip, dst_port), NOT THE PEER ALONE.
    //
    // Matching on the peer capped every receiver at ONE flow: a second flow
    // request from the same device found the first context and overwrote it,
    // so a 16-channel receiver received 8 channels and nothing was ever
    // rejected. Found 2026-08-04 with a RedNet A16R (16 ch) and an AM2 (2 ch)
    // subscribed: 2 contexts bound, 205 requests, 0 rejected, both devices
    // asking for more than they got.
    //
    // A receiver demultiplexes its flows by DESTINATION UDP PORT -- each one
    // arrives in its own 0x0802 socket descriptor (dante_flows.c:88) -- so the
    // port is what distinguishes two flows from one device. Keying on the full
    // socket keeps the keepalive behaviour the old comment wanted while
    // letting one device hold as many flows as it asks for.
    uint32_t now_ms = gptp_uptime_ms();
    int f = -1;
    for (unsigned i = 0; i < N_FLOWS; i++)
        if (flows[i].in_use &&
            flows[i].dport == dst_port &&
            ip4_eq(flows[i].peer, peer_ip) &&
            ip4_eq(flows[i].dst,  dst_ip)) { f = (int)i; break; }
    if (f < 0)
        for (unsigned i = 0; i < N_FLOWS; i++)
            if (!flows[i].in_use) { f = (int)i; break; }
    if (f < 0) {
        // All six taken by other peers: evict the least-recently-bound rather
        // than refusing. This replaces timer expiry -- it reclaims a context
        // only when one is actually needed, so it cannot fire spuriously the
        // way a PTP-derived timeout did.
        uint32_t oldest = 0; f = 0;
        for (unsigned i = 0; i < N_FLOWS; i++) {
            uint32_t age = now_ms - flows[i].last_ms;
            if (age >= oldest) { oldest = age; f = (int)i; }
        }
        printf("[dtx] all contexts busy -- evicting %d\n", f);
        flows[f].in_use = 0;
    }

    write_ctx((unsigned)f, dst_ip, dmac, dst_port, chans, nslots, fpp);

    if (!flows[f].in_use) {
        flows[f].in_use = 1;
        flows[f].rebinds++;
        for (int i = 0; i < 4; i++) flows[f].peer[i] = peer_ip[i];
        telem_event(TELEM_E_FLOW_BIND, (int32_t)((f << 24) | (nslots << 8) | fpp),
                    (int32_t)dst_port);
        printf("[dtx] flow %d -> %u.%u.%u.%u:%u, %u slots, fpp %u\n",
               f, dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3], dst_port, nslots, fpp);
    }
    for (int i = 0; i < 4; i++) flows[f].dst[i] = dst_ip[i];
    for (int i = 0; i < 6; i++) flows[f].dmac[i] = dmac[i];
    flows[f].dport  = dst_port;
    flows[f].nslots = nslots;
    flows[f].fpp    = fpp;
    flows[f].mcast  = 0;
    for (unsigned i = 0; i < 8; i++) flows[f].chans[i] = (i < nslots) ? chans[i] : 0;
    flows[f].last_ms = now_ms;
    return f;
}

void dante_tx_expire(void)
{
    // DELIBERATELY DOES NOTHING. Flows are never expired on a timer.
    //
    // Every timer-based version of this was wrong, because the only clock we
    // have is derived from PTP and PTP steps. The console showed the cost
    // plainly: at lock, both flows expired in the same instant, active dropped
    // to zero, and the talker went ENABLED -> OFF -> ENABLED within seconds.
    // Each toggle re-anchors the media clock and re-primes the ring, and THAT
    // is the mangled audio at stream start -- not the ring level, which now
    // sits at its centre through the whole hold-off. Multicast never showed it
    // because multicast never gated on flows.
    //
    // Raising the timeout (16 s -> 45 s -> 5 min) never addressed it: the
    // apparent gap after a step is tens of thousands of seconds, not seconds.
    // Nor did the discontinuity guard, which compared consecutive polls and
    // still missed it.
    //
    // Expiry is not needed for correctness. A context is reclaimed by peer IP
    // when the same receiver refreshes, and when all six are taken a NEW peer
    // evicts the least-recently-bound one (see dante_tx_bind_unicast). A
    // departed receiver therefore costs one idle context and nothing else --
    // and an idle context transmits nothing, because nslots stays set but the
    // talker only ever sends to bound destinations.
}

// Turn a multicast bundle on: 8 consecutive channels at fpp 16, to the group
// bound by bind_flow(). Multicast is NOT a separate datapath -- it is a flow
// whose destination happens to be a group address, so it runs through exactly
// the same per-context map the unicast path uses. Kept available for 0x2201.
int dante_tx_bind_multicast(uint16_t ext_id, const uint16_t *chans, uint8_t n)
{
    if (n == 0 || n > 8) return -1;

    // Reuse the context already serving this Dante Controller flow id, else a
    // free one. DC's flow id is a HANDLE, not an index into our contexts -- it
    // picks from the max-flows value we advertise in 0x1000 (32), so the two
    // numbering spaces have to be kept apart.
    int f = -1;
    for (unsigned i = 0; i < N_FLOWS; i++)
        if (flows[i].in_use && flows[i].mcast && flows[i].ext_id == ext_id) { f = (int)i; break; }
    if (f < 0)
        for (unsigned i = 0; i < N_FLOWS; i++)
            if (!flows[i].in_use) { f = (int)i; break; }
    if (f < 0) return -1;

    const uint8_t *dip = flow_ip[f];
    uint8_t dmac[6] = { 0x01, 0x00, 0x5E, (uint8_t)(dip[1] & 0x7F), dip[2], dip[3] };
    write_ctx((unsigned)f, dip, dmac, DANTE_PORT_MEDIA, chans, n, 16);

    flows[f].in_use = 1;
    for (int i = 0; i < 4; i++) flows[f].dst[i] = dip[i];
    flows[f].dport = DANTE_PORT_MEDIA;
    flows[f].nslots = n; flows[f].fpp = 16; flows[f].mcast = 1;
    flows[f].ext_id = ext_id;
    for (unsigned i = 0; i < 8; i++) flows[f].chans[i] = (i < n) ? chans[i] : 0;
    flows[f].last_ms = gptp_uptime_ms();
    printf("[dtx] multicast flow %u (ctx %d) -> %u.%u.%u.%u, %u ch\n",
           ext_id, f, dip[0], dip[1], dip[2], dip[3], n);
    return f;
}

// Find the context serving a Dante Controller flow id, for 0x2202.
int dante_tx_ctx_for_id(uint16_t ext_id)
{
    for (unsigned i = 0; i < N_FLOWS; i++)
        if (flows[i].in_use && flows[i].ext_id == ext_id) return (int)i;
    return -1;
}

// Per-flow state for the UDP stats endpoint: is the context bound, and how long
// since its last keepalive. Exposed because the console is not readable from
// the build host, and "is context 1 cycling?" cannot be answered any other way.
// active is DERIVED, not counted. It was incremented on bind and decremented on
// expire, and the two got out of step -- reported 4 with two flows in use.
unsigned dante_tx_active(void)
{
    unsigned n = 0;
    for (unsigned i = 0; i < N_FLOWS; i++) if (flows[i].in_use) n++;
    return n;
}

void dante_tx_flow_info(unsigned f, uint8_t *in_use, uint32_t *age_ms,
                        uint32_t *rebinds)
{
    if (f >= N_FLOWS) { *in_use = 0; *age_ms = 0; *rebinds = 0; return; }
    *in_use  = flows[f].in_use;
    *age_ms  = flows[f].in_use ? (gptp_uptime_ms() - flows[f].last_ms) : 0;
    *rebinds = flows[f].rebinds;
}

// Describe a bound flow for ARC 0x2200. Returns 0 if the context is idle, so
// the reply lists what we ACTUALLY transmit rather than six bundles that exist
// only as pre-computed headers.
int dante_tx_flow_desc(unsigned f, uint8_t ip[4], uint16_t *port,
                       uint8_t *nslots, uint8_t *fpp, uint8_t *mcast, uint16_t *id)
{
    if (f >= N_FLOWS || !flows[f].in_use) return 0;
    for (int i = 0; i < 4; i++) ip[i] = flows[f].dst[i];
    *port = flows[f].dport; *nslots = flows[f].nslots;
    *fpp = flows[f].fpp;    *mcast = flows[f].mcast;
    if (id) *id = flows[f].ext_id ? flows[f].ext_id : (uint16_t)(f + 1);
    return 1;
}

// Tear a flow down on request (ARC 0x2202). nslots = 0 makes the builder skip
// the context entirely.
int dante_tx_unbind(unsigned f)
{
    if (f >= N_FLOWS || !flows[f].in_use) return -1;
    flows[f].in_use = 0;
    aaf_pkt_ctx_select_write(f);
    aaf_pkt_flow_cfg_write(0);
    telem_event(TELEM_E_FLOW_UNBIND, (int32_t)f, 0);
    printf("[dtx] flow %u deleted\n", f);
    return 0;
}

// The actual slot map, so 0x2200 reports what a flow really carries. It used to
// report (ctx * 8 + slot + 1) -- derived from the CONTEXT INDEX -- so a flow
// created for channels 1 and 2 in context 1 was reported as channels 9 and 10.
uint16_t dante_tx_flow_chan(unsigned f, unsigned slot)
{
    if (f >= N_FLOWS || slot >= 8) return 0;
    return flows[f].chans[slot];
}

// Is this tx channel carried by an active MULTICAST flow, and at which slot?
// mdns.c needs it to advertise b.<flow>=<pos+1>, which is what makes a receiver
// join the group instead of asking us for a unicast flow.
int dante_tx_chan_bundle(uint16_t ch1, uint16_t *id, uint8_t *pos)
{
    for (unsigned i = 0; i < N_FLOWS; i++) {
        if (!flows[i].in_use || !flows[i].mcast) continue;
        for (unsigned c = 0; c < flows[i].nslots; c++)
            if (flows[i].chans[c] == ch1) {
                *id = flows[i].ext_id; *pos = (uint8_t)(c + 1); return 1;
            }
    }
    return 0;
}

// Look up an active multicast flow by the id Dante Controller gave it, for the
// _netaudio-bund record a receiver resolves after reading b.<flow>=.
int dante_tx_mcast_by_id(uint16_t id, uint8_t ip[4], uint8_t *nslots)
{
    for (unsigned i = 0; i < N_FLOWS; i++) {
        if (!flows[i].in_use || !flows[i].mcast || flows[i].ext_id != id) continue;
        for (int k = 0; k < 4; k++) ip[k] = flows[i].dst[k];
        *nslots = flows[i].nslots;
        return 1;
    }
    return 0;
}

// Enumerate active multicast flow ids (for the bundle PTR sweep).
int dante_tx_mcast_enum(unsigned n, uint16_t *id)
{
    unsigned k = 0;
    for (unsigned i = 0; i < N_FLOWS; i++) {
        if (!flows[i].in_use || !flows[i].mcast) continue;
        if (k++ == n) { *id = flows[i].ext_id; return 1; }
    }
    return 0;
}

// Report the destination MAC a context was bound with. Two flows emitting at
// exactly the right combined rate but only one receiver hearing audio points at
// per-context header state, and this is the field that cannot be checked from
// the build host: unicast is forwarded only to its destination port.
void dante_tx_flow_mac(unsigned f, uint8_t mac[6])
{
    for (int i = 0; i < 6; i++) mac[i] = (f < N_FLOWS) ? flows[f].dmac[i] : 0;
}
