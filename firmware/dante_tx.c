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
#include "mdns.h"
#include "dante_dev.h"
#include "ptpv1.h"
#include "gptp.h"
#include "mcr.h"
#include "dante_flows.h"
#include "net.h"
#include "telem.h"
#include "mcr_dante.h"
#include "config.h"   // g_fs: de Dante-timestamp is seconden*fs + subsec
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

dante_tx_stats_t g_tx_stats;

// How long PTP must stay locked before the media clock is anchored to it.
// The first lock edge is not trustworthy -- see dante_tx_poll.
#define PTP_SETTLE_MS    4000
#define PHASE_SETTLE_MAX_MS  20000   // fallback: never block audio indefinitely

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
// fpp for ARC-created multicast bundles. MUST MATCH the fpp advertised in the
// _netaudio-bund TXT record (mdns.c, "fpp=16") -- a receiver subscribes on the
// advertised value and cannot decode packets in another format. Left at 60
// after a debug sweep once, which showed as a GREEN subscription with no audio:
// the flow resolves, the payload is unreadable. If this is ever made variable
// for real, the mDNS record has to be generated from the same variable.
uint16_t g_mcast_fpp = 16;

// RUNTIME-TUNABLE TIMESTAMP OFFSET.
//
// This was a compile-time constant, and calibrating it cost a rebuild, a flash,
// a reboot, and the operator re-creating the multicast flow by hand -- about
// five minutes per data point, with the board's uptime (and therefore any
// accumulated phase drift) different every time. Two attempts at predicting the
// right value from a model both missed, and the second overshot by more than
// the first undershot, because the model `lag = 15 - offset` does not describe
// the measured system: a -162 change in the offset moved the on-wire lag by
// +366, a ratio of 2.26 rather than 1.
//
// A knob that can be swept in seconds, without a reboot and without losing the
// flow table, turns that into a two-point calibration. Measure, adjust, measure
// -- no model required.
static int32_t ts_offset_samples = DANTE_TX_TS_OFFSET;

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

// PIPELINE ADJUSTMENT for the pacing phase seed.
//
// MEASURED, not derived. With the seed written as (ts_sub % fpp) the emitted
// subsecond came out congruent to 2 mod fpp on every packet -- 2000 out of 2000
// in a capture, dead constant:
//
//     subsec mod 16 histogram: {2: 2000}
//     first subsec: 43714, 43730, 43746   (spacing 16, all == 2 mod 16)
//
// so the tick fires at ts_sub == 1 mod fpp instead of fpp-1. The value firmware
// reads back over the CSR is two media samples behind what the datapath uses
// when it evaluates the due comparison. Seeding two higher makes the counter
// reach fpp-1 two samples earlier, which lands the tick where it belongs.
//
// If the packetizer's due/latch pipeline is ever restructured this constant has
// to be re-measured -- decode a capture with tools/dante_decode.py and read the
// "ts multiple of fpp" line, which is exactly what found it.
uint32_t g_phase_adj = 2;   /* was #define PHASE_PIPELINE_ADJ; see below */

// Mirrors FPP_TABLE in dante_packetizer.py. Keep the two in step: a mismatch
// makes the gateware pace at a different rate than the header advertises, which
// is silent on every counter we have.
static const uint16_t k_fpp_table[8] = { 8, 16, 24, 32, 60, 4, 2, 48 };

int dante_tx_fpp_supported(uint16_t fpp)
{
    // Enforce the advertised maximum. mDNS says "fpp=8,2"; accepting 16 anyway
    // was a promise we did not keep, and it is why an AM2 sits on a 333 us
    // window that no 250 us latency can accommodate.
    if (fpp > g_fpp_max_accept) return 0;
    for (unsigned i = 0; i < 8; i++)
        if (k_fpp_table[i] == fpp) {
            // pay_len = nslots*3*fpp must stay a multiple of 4 (f_last_idx is
            // pay_len/4). Only fpp=2 with an ODD slot count violates that.
            return 1;
        }
    return 0;
}

uint32_t dante_tx_fpp_index(uint16_t fpp)
{
    for (unsigned i = 0; i < 8; i++)
        if (k_fpp_table[i] == fpp) return i;
    return 0;                                   /* unreachable: gated above */
}

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

// Defined after flows[]; bound in dante_tx_init so the observation starts at
// boot rather than at the first subscription.
static void media_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *p, uint32_t len);

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

    // LISTEN ON OUR OWN MEDIA PORT. We are a transmitter, so nothing sends us
    // audio -- but a receiver is reported to send a keepalive to 4321, and with
    // no binding here those datagrams were discarded by the UDP demux without
    // even a counter. Binding it is how the claim gets measured.
    //
    // Cost is bounded by main.c's MAC-level early drop: flooded multicast audio
    // for other devices' groups never reaches IP parsing, and the handler drops
    // any multicast destination on its first line, so this only ever sees
    // unicast addressed to us.
    if (net_udp_bind(DANTE_PORT_MEDIA, media_rx) != 0)
        printf("[dtx] BIND FAILED on %u -- udp table full, keepalives unobservable\n",
               DANTE_PORT_MEDIA);

    talker_on = 0;
    printf("[dtx] %u flows x 8 ch, fpp=%u, 24-bit, %u pps/flow (talker held off)\n",
           N_FLOWS, FPP, g_fs / FPP);
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
// sub = ns * fs / 1e9. Uitgevoerd als ns * 3 / (3e9/fs): 62500 bij 48 kHz,
// 31250 bij 96 kHz, beide exact. De *3 blijft STAAN en wordt niet in de deler
// verrekend, want die factor is precies wat de som binnen 32 bits houdt:
// ns < 1e9 dus ns*3 < 3e9 < 2^32 -- unsigned, en het moet unsigned blijven.
// Zou je in plaats daarvan ns * fs doen, dan is dat bij 96 kHz 9.6e13 en moet
// alles naar 64 bits.
// Defined below, once flows[] is in scope.
static void dante_tx_reseed_phases(uint32_t sub);

static void ts_anchor(void)
{
    ptp_timestamp_t t = gptp_read_time();
    int32_t  sub = (int32_t)((t.nanoseconds * 3u) / (3000000000u / g_fs));
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
    sub += ts_offset_samples;
    // De subsecond-teller loopt 0..fs-1, dus de carry MOET met fs gebeuren en
    // niet met een literal: bij 96 kHz zou 48000 hier elke timestamp een halve
    // seconde verkeerd zetten en de ontvanger alles als te laat verwerpen.
    if (sub < 0)                 { sub += (int32_t)g_fs; sec -= 1; }
    else if (sub >= (int32_t)g_fs) { sub -= (int32_t)g_fs; sec += 1; }

    aaf_pkt_ts_load_sec_write(sec);
    aaf_pkt_ts_load_sub_write((uint32_t)sub);
    aaf_pkt_ts_load_write(1);

    // RESEED EVERY CONTEXT'S PACING PHASE.
    //
    // The per-context phase counter must track ts_sub % fpp, or the emitted
    // timestamp -- ts_sub - (fpp-1), taken on the strobe where the counter
    // reaches fpp-1 -- stops being a multiple of fpp. Seeding it once at bind
    // is not enough: THIS FUNCTION JUST MOVED ts_sub, and nothing reloaded the
    // counters, so every bound context is left offset by however far the anchor
    // jumped. Permanently, since the counter free-runs from there.
    //
    // Measured on the wire after the borrow fix removed the 2^32 wrap:
    //   step histogram: 16x1999   (spacing correct, no gaps)
    //   ts multiple of fpp: NO    (alignment still wrong)
    // which is exactly "phase is a fixed offset from where it should be".
    //
    // The load is adopted at the next media strobe, so the value to seed from
    // is the sub we just wrote, not the live counter.
    dante_tx_reseed_phases((uint32_t)sub);
    g_tx_stats.anchors++;
    telem_event(TELEM_E_ANCHOR, (int32_t)sec, (int32_t)sub);
    printf("[dtx] media clock anchored to PTP %lu.%09lu -> %lu.%lu (offset %d)\n",
           (unsigned long)t.seconds, (unsigned long)t.nanoseconds,
           (unsigned long)sec, (unsigned long)sub, (int)ts_offset_samples);
}

// Defined below, once flows[] is in scope. Returns the smallest and largest
// fpp among bound flows, or 16/16 if none are bound.
static void dante_tx_reseed_phases(uint32_t sub);

void dante_tx_poll(void)
{
    // A bind or unbind can change what the bound flows need, so the advertised
    // value has to follow. Without this the auto-raise would only take effect
    // at the next unrelated announcement, i.e. a receiver could subscribe at
    // fpp=16 and sit red until something else happened to republish.
    {
        static uint32_t last_eff;
        uint32_t eff = dante_tx_latency_effective();
        if (eff != last_eff) {
            if (last_eff)                       // not the first call at boot
                printf("[dtx] advertised latency %lu -> %lu ns\n",
                       (unsigned long)last_eff, (unsigned long)eff);
            last_eff = eff;
            mdns_announce();
        }
    }
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

    // WAIT FOR PTP'S PHASE TO BE FINAL, not merely for lock.
    //
    // mean_path_delay_ns goes non-zero on the FIRST DelayResp, but ptpv1 then
    // applies a residual phase STEP once it has PD_MIN_SAMPLES of them
    // (ptpv1.c:446). The old gate was satisfied by the first condition, so the
    // talker anchored the media clock and THEN PTP stepped absolute time out
    // from under it. The media clock is a free-running counter anchored once --
    // it stayed on the pre-step timeline permanently. That is audio which is
    // out of sync from the instant the stream starts, with every counter
    // reading healthy.
    // phase_settled is a GATE, not a hard dependency: if it somehow never
    // arrives, audio must not be blocked forever. After PHASE_SETTLE_MAX_MS of
    // lock we start anyway -- a late step will now re-anchor us (below) rather
    // than leaving the media clock stranded, so starting is recoverable and
    // never starting is not.
    uint8_t phase_ok = g_ptpv1.phase_settled ||
                       (gptp_uptime_ms() - lock_since) >= PHASE_SETTLE_MAX_MS;
    uint8_t settled = g_ptpv1.locked
                   && phase_ok
                   && g_ptpv1.mean_path_delay_ns != 0
                   && (gptp_uptime_ms() - lock_since) >= PTP_SETTLE_MS;

    // SELF-HEALING: any later PTP step invalidates the anchor, so follow it.
    // This is the general form of the bug above -- it does not matter WHY the
    // clock stepped (cold-boot acquisition, a master change, a leap), only that
    // anything anchored to it is now on the wrong timeline.
    {
        static uint32_t seen_steps;
        static uint8_t  seen_init;
        if (!seen_init) { seen_steps = g_ptpv1.step_count; seen_init = 1; }
        if (g_ptpv1.step_count != seen_steps) {
            seen_steps = g_ptpv1.step_count;
            if (talker_on) {
                printf("[dtx] PTP stepped -- re-anchoring media clock\n");
                ts_anchor();
            }
        }
    }

    // Also wait for the MEDIA CLOCK to be ready, not just PTP.
    //
    // PTP lock says the TIME reference is good; it says nothing about the
    // sample rate the packetizer is emitting at. Starting a stream while
    // mcr_dante is still slewing means the first ~43 s go out on a rate that is
    // both wrong and moving -- the "bad audio for a while after boot" symptom.
    // With the NV warm start this is normally satisfied immediately.
    // *** ZODRA DE TALKER LOOPT, STOPT HIJ NIET MEER OP EEN PTP-UNLOCK. ***
    //
    // Gemeten 2026-11-05: een excursie van 16 us, zes samples lang, zette de
    // talker uit en kostte 13 SECONDEN stilte bij ELKE ontvanger -- 8,8 s
    // unlock plus exact 4,000 s PTP_SETTLE_MS na de relock. De flows van
    // beide ontvangers vielen 5 s NA het uitzetten weg: gevolg, geen oorzaak.
    //
    // Uitzetten beschermt niets. De mediaklok is een vrijlopende teller die
    // EEN keer aan PTP wordt geankerd, dus een offset-excursie in PTP
    // verschuift onze timestamps niet. Wat wel telt is de fase van die
    // mediaklok, en die wordt hierboven al gemeten en bewaakt: fijn op
    // DANTE_TX_REANCHOR_SAMPLES (16 samples = 167 us bij 96 kHz) en grof op
    // +-24000 samples (0,5 s). Die twee vangen een echte klokbreuk; de
    // lock-vlag voegde daar alleen een mute van 13 s aan toe.
    //
    // REFERENTIE, niet zelf verzonnen: Inferno-ESP32-P4 (dezelfde auteur, de
    // N-Series DAC8 op dit netwerk) gate de audio NERGENS op lock. In
    // media_clock.c doet een unlock precies een ding:
    //
    //     if (!g_ptpv1.locked) {
    //         // PTP is the timeline. Without it the error term is
    //         // meaningless, so hold the last rate rather than servoing
    //         // on noise.
    //         return;
    //     }
    //
    // De servo bevriest, de audio loopt door. Daar is ook geen wachttijd na
    // een relock -- niet na een relock en niet bij koud opstarten.
    //
    // Wat BLIJFT: de eerste start wacht nog wel op settled + rate_ready. Daar
    // is de hele reden van PTP_SETTLE_MS en phase_settled dat je niet verkeerd
    // geankerd begint (zie de commentaren hierboven: 227 ms in de toekomst,
    // elke pakket weggegooid, elke teller gezond). Fout beginnen is niet
    // herstelbaar voor een ontvanger die ons al heeft afgeschreven; doorgaan
    // door een unlock heen is dat wel.
    uint8_t have_flow = (dante_tx_active() > 0);
    uint8_t want = talker_on ? have_flow
                             : (settled && have_flow && mcr_dante_rate_ready());

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
            int64_t ptp_smp = (int64_t)t.seconds * (int64_t)g_fs
                            + (int64_t)((t.nanoseconds * 3u) / (3000000000u / g_fs));
            int64_t emit    = (int64_t)aaf_pkt_dbg_last_sec_read() * (int64_t)g_fs
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
    // RATE-LIMIT THIS BLOCK. It used to run at main-loop rate (~4 kHz) doing
    // about ten CSR reads, which was free. It is not free any more:
    // dante_tx_media_phase() BUSY-WAITS up to one sample period (20.8 us) for
    // the media counter to tick, so running it every iteration stalls a 250 us
    // main loop by ~10 us on average -- and this loop is what drains the two
    // MAC RX slots.
    //
    // MEASURED CONSEQUENCE, 2026-08-14: PTP offset noise went from a -26 ns
    // steady state to +-3.5 us rms 1340 ns, with 2% of Syncs losing their
    // FollowUp, and the talker cycling on every unlock. None of that existed
    // before this block started blocking. It was a self-inflicted regression
    // introduced with the atomic-drift-sample work, and it took the per-Sync
    // telemetry to see it.
    //
    // 10 Hz is ample for a coarse backstop: with bad_streak >= 2 a genuine
    // phase error is caught in 200 ms, against a re-anchor threshold that takes
    // minutes to reach. Cost falls from ~8% of the main loop to 0.02%.
    static uint32_t last_reanchor_ms;
    uint32_t now_ms = gptp_uptime_ms();
    if (talker_on && want && (uint32_t)(now_ms - last_reanchor_ms) >= 100u) {
        last_reanchor_ms = now_ms;
        // MEASURE THE COUNTER, NOT THE LAST EMITTED TIMESTAMP.
        //
        // This guard used to read the emitted pair, which is the media counter
        // minus (fpp-1) for whichever flow happened to emit last. Firmware
        // cannot tell which flow that was, so the reading carried (fmax-fmin)
        // of genuine ambiguity, and the band was widened by exactly that to
        // stop it firing continuously when DVS (fpp=60) subscribed alongside an
        // A16R (fpp=4).
        //
        // THAT WIDENING SILENTLY DISABLED THE BACKSTOP FOR EVERY LOW-LATENCY
        // RECEIVER. Measured 2026-08-12 with an A16R (fpp=4), an AM2 (fpp=16)
        // and DVS (fpp=60) all bound:
        //
        //     band = 8 + (60 - 4) = 64 samples = 1.33 ms
        //
        // which is 5.3x the A16R's ENTIRE 12-sample budget at 0.25 ms and 1.3x
        // the AM2's at 1 ms. The drift walked to +17 samples over 55 minutes
        // with nothing firing; both receivers' reported latency reached zero
        // and stayed there, while the guard sat waiting for 64. Subscribing a
        // DVS turned off the backstop for the other two, and nothing said so.
        //
        // dante_tx_media_phase() reads the LIVE counter, which has no fpp term
        // in it at all, so the ambiguity that forced the widening is gone and
        // the band can be a flat DANTE_TX_REANCHOR_SAMPLES again -- the value
        // that was deliberately chosen (24 -> 8) to fit inside a 0.25 ms
        // budget. What the band must be sized against is the TIGHTEST
        // receiver's tolerance, never the spread of fpps that happen to be
        // subscribed.
        int32_t phase;
        if (dante_tx_media_phase(&phase)) {
            // The anchor puts the counter at PTP + ts_offset_samples, so that
            // is where it should sit.
            int32_t err  = phase - ts_offset_samples;
            // *** DE BAND IS EEN TIJD, NIET EEN AANTAL SAMPLES. ***
            // DANTE_TX_REANCHOR_SAMPLES is 8 en komt uit "24 samples is 0,5 ms"
            // (mcr_dante.c:155) -- dus uit 48 kHz. Bij 96 kHz is 8 samples maar
            // 83 us, de helft van het bedoelde budget, en dan valt een
            // systematische afwijking van 12 samples (125 us) er net buiten en
            // ankert dit eeuwig door op 10 Hz. Gemeten 2026-11-04: 144
            // re-anchors in 12 s met "phase -12 samples (-125 us)", terwijl de
            // werkelijke emit-tegen-PTP fout -0,2 ms was en rotsvast stond.
            // Schaal mee met de rate zodat het budget 167 us blijft: 8 bij
            // 48 kHz, 16 bij 96 kHz. Dat blijft binnen de 0,25 ms van de
            // strengste ontvanger (A16R), die bij 96 kHz 24 samples is.
            int32_t band = (int32_t)((uint32_t)DANTE_TX_REANCHOR_SAMPLES
                                     * g_fs / 48000u);
            if (band < DANTE_TX_REANCHOR_SAMPLES)
                band = DANTE_TX_REANCHOR_SAMPLES;   // nooit strakker dan ontworpen
            // RESTEERT: die 12 samples zijn systematisch, niet ruis -- de
            // re-anchor zet de teller op PTP+offset en meteen daarna meet hij
            // weer -12. Dat is vermoedelijk de firmware-latency tussen
            // gptp_read_time() en de load-puls (~125 us = ~6000 cycles op 50
            // MHz). De band verdraagt het nu; hem WEGHALEN vraagt die latency
            // meten en compenseren in ts_anchor(). Dat staat nog open.
            // CONFIRM BEFORE STEPPING. A re-anchor is audible, so one bad
            // reading must never cause one. Require the error to persist across
            // two polls; real phase error is monotonic and easily survives
            // that, while a measurement artefact does not.
            static uint8_t bad_streak;
            if (err > band || err < -band)
                bad_streak++;
            else
                bad_streak = 0;
            if (bad_streak >= 2) {
                bad_streak = 0;
                // us-conversie OOK op g_fs: err*125/6 is 20,833 us/sample
                // = 48 kHz. Op 96 kHz meldde dit 2,96 s voor een afwijking van
                // 1,48 s, wat het probleem twee keer zo groot liet lijken als
                // het was. 1e6/fs us per sample.
                printf("[dtx] media clock phase %ld samples (%ld us) off "
                       "[band %ld fs %lu] -- re-anchoring\n",
                       (long)err, (long)((int64_t)err * 1000000 / (int64_t)g_fs),
                       (long)band, (unsigned long)g_fs);
                ts_anchor();
            }
        }

        // COARSE BACKSTOP, kept separately. dante_tx_media_phase() folds into
        // +-24000 samples, so it cannot see an epoch-scale error -- and that is
        // the error this guard was originally written for: 2026-08-04, emitted
        // 402927.04519 against PTP 402926.41611, +10908 samples = +227 ms into
        // the future, every packet discarded as far-future while the control
        // plane stayed green and every counter read healthy. Keep a full-range
        // check so a PTP step or a stranded anchor is still caught.
        {
            ptp_timestamp_t t = gptp_read_time();
            uint32_t esec, esub;
            dante_tx_read_emitted(&esec, &esub);      // atomic: see the header
            int64_t ptp_smp = (int64_t)t.seconds * (int64_t)g_fs
                            + (int64_t)((t.nanoseconds * 3u) / (3000000000u / g_fs));
            int64_t emit    = (int64_t)esec * (int64_t)g_fs + (int64_t)esub;
            int64_t err     = emit - ptp_smp;
            // Half a second. Far beyond any fpp bias (59 samples at most) and
            // any staleness the emitted register can carry while the talker is
            // ON, so this cannot fire on the artefacts that plague the fine
            // measurement -- it only sees errors the fine one cannot represent.
            static uint8_t coarse_streak;
            if (err > 24000 || err < -24000)
                coarse_streak++;
            else
                coarse_streak = 0;
            if (coarse_streak >= 2) {
                coarse_streak = 0;
                printf("[dtx] media clock %ld samples (%ld ms) off "
                       "-- COARSE re-anchor\n",
                       (long)err, (long)(err / 48));
                ts_anchor();
            }
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
        // WELKE van de drie termen in `want` viel weg? "clock unlocked" stond
        // hier als vaste tekst, maar `want` heeft drie oorzaken en twee ervan
        // hebben niets met de klok te maken. Bij een dode console is dit de
        // enige manier om een dropout van 10 s toe te wijzen.
        // EEN TELLER, EEN OORZAAK. Onder het beleid hierboven kan een
        // lopende talker alleen nog uitgaan doordat de laatste flow weg is,
        // dus dat wordt eerst getest. off_unlock en off_rate zijn daarmee
        // tripwires: als die oplopen is het beleid ergens omzeild, en dat is
        // iets anders dan "er was toevallig ook geen lock".
        uint8_t why = 0;
        if (!have_flow) { why |= 2; g_tx_stats.off_noflow++; }
        else {
            if (!settled)                { why |= 1; g_tx_stats.off_unlock++; }
            if (!mcr_dante_rate_ready()) { why |= 4; g_tx_stats.off_rate++;   }
        }
        aaf_pkt_enable_write(0);
        talker_on = 0;
        telem_event(TELEM_E_TALKER_OFF, (int32_t)why, (int32_t)dante_tx_active());
        g_tx_stats.disables++;
        printf("[dtx] talker OFF (why=%u: %s%s%s)\n", why,
               (why & 1) ? "ptp-unsettled " : "",
               (why & 2) ? "no-flow " : "",
               (why & 4) ? "rate-not-ready" : "");
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
    // THE HANDLE WE GAVE THE RECEIVER. dante_flows.c answers a 0x0100 with
    // (flow index u32, cookie u16) because inferno does (flows_tx.rs:631-633).
    // The cookie was a local `static` that incremented per reply and was never
    // stored, so a 0x0101 stop could only be matched on the index -- and the
    // index is REUSED, so a stale handle from a previous binding of the same
    // slot would have torn down whatever is in it now. Keeping it per flow is
    // what makes the handle mean one specific binding.
    //
    // Assigned on a FRESH bind only, and deliberately kept across a refresh:
    // the receiver holds the handle it was first given, and a refresh must not
    // invalidate it.
    uint16_t cookie;
    // OBSERVATION ONLY -- nothing acts on these. The question they exist to
    // answer is whether a receiver sends anything to our media port (4321), as
    // a DAC8 is reported to do, because that would be the keepalive signal the
    // 4455 path never gave us: measured here, our one flow's age_ms climbed
    // monotonically through 19 minutes with zero refreshes and `tcpdump -i ens5
    // 'udp port 4455'` saw 0 packets in 20 s.
    //
    // They are NOT wired to expiry. Every timer-based expiry in this file has
    // silenced a live flow (see dante_tx_expire), so the number comes first and
    // the policy only after it is known which devices actually send this.
    uint32_t ka_rx;        // datagrams seen from this peer on port 4321
    uint32_t ka_ms;        // uptime of the most recent one, 0 = never
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
// 600 s, not 300. On the bench an AM2's live flow was measured at 229 s and
// still climbing between refreshes, so a 300 s timer was close enough to its
// keepalive interval to silence a flow that was still in use -- the exact
// "erring short costs audio" failure recorded above. Slot pressure is handled
// by eviction-on-demand in the bind paths, which reclaims only when a slot is
// genuinely needed, so the timer can afford to be purely a backstop.
// NO LONGER USED, and deliberately left here unreferenced so the next person
// to reach for a 4455-age timeout reads the comment above first. dante_tx_expire
// now keys on the keepalives the receivers actually send, to our MEDIA port --
// see the measurements in that function. Do not resurrect this.
#define FLOW_TIMEOUT_MS_UNUSED  600000



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

    // fpp is now an INDEX into the packetizer's FPP_TABLE, not a 0/1 bit.
    // Index 0 and 1 are still 8 and 16, so the encoding is unchanged for the
    // two values we used to support.
    uint32_t fpp_idx = dante_tx_fpp_index(fpp);

    // PACING PHASE, seeded from the LIVE sample counter.
    //
    // The packetizer counts each context's samples 0..fpp-1 and emits on the
    // last, so the counter must start at (ts_sub % fpp) or emitted timestamps
    // stop being multiples of fpp. Firmware does the modulo because fpp is no
    // longer a power of two.
    //
    // THE FIRST ATTEMPT SEEDED FROM dante_tx_read_emitted(), which returns the
    // last EMITTED timestamp -- already (counter - (fpp-1)) for whichever flow
    // emitted, and stale by however long ago that was. Every context therefore
    // started at an arbitrary phase, and the measured packet rate came out at
    // about a fifth of nominal. ts_now_sub exists so there is a correct value
    // to read.
    //
    // SEQLOCK ACROSS THE LATCH. The phase must equal ts_sub at the instant
    // flow_cfg is written, because that write is what loads the counter. A
    // sample is 20.8 us and this sequence is a few us, so it usually holds --
    // but "usually" leaves a 1-sample misalignment that would be invisible
    // until someone decoded the stream. Re-read after the write and retry if
    // the counter moved; re-writing flow_cfg just re-latches, which is
    // harmless.
    {
        uint32_t cfg = (uint32_t)(nslots & 0x0F) | ((fpp_idx & 0x7u) << 4);
        for (unsigned tries = 0; tries < 8; tries++) {
            uint32_t s0 = aaf_pkt_ts_now_sub_read();
            aaf_pkt_flow_phase_write((s0 + g_phase_adj) % (uint32_t)fpp);
            // flow_cfg LAST: it latches the channel map and the phase with it,
            // so the builder never sees a half-written context.
            aaf_pkt_flow_cfg_write(cfg);
            if (aaf_pkt_ts_now_sub_read() == s0) break;
        }
    }
}

static void dante_tx_reseed_phases(uint32_t sub)
{
    for (unsigned f = 0; f < N_FLOWS; f++) {
        if (!flows[f].in_use || !flows[f].fpp) continue;
        aaf_pkt_ctx_select_write(f);
        // MINUS ONE, unlike the bind path. The two seeding paths sample the
        // counter at different instants:
        //
        //   bind   seeds from ts_now_sub -- the counter's CURRENT value
        //   anchor seeds from `sub`      -- the value the counter will ADOPT
        //                                  at the next media strobe
        //
        // The phase register loads immediately on flow_cfg.re, but ts_sub does
        // not take `sub` until that next strobe, by which point phase has
        // already incremented once. Seeding (sub + adj) therefore lands one
        // sample high.
        //
        // Measured: a freshly bound fpp=60 flow emitted subsec congruent to 0,
        // and the same flow after a re-anchor emitted 59 -- exactly -1. It went
        // unnoticed at fpp=16 for a while because the misalignment only appears
        // once an anchor has fired.
        //
        // + fpp before the modulo so the subtraction cannot go negative.
        aaf_pkt_flow_phase_write((sub + g_phase_adj + flows[f].fpp - 1u)
                                 % (uint32_t)flows[f].fpp);
        // flow_cfg.re is what latches the phase; re-write the same config.
        aaf_pkt_flow_cfg_write((uint32_t)(flows[f].nslots & 0x0F) |
                               ((dante_tx_fpp_index(flows[f].fpp) & 0x7u) << 4));
    }
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
        // PREFER A UNICAST VICTIM. A multicast flow is never refreshed (ARC
        // 2201 creates it, nothing sends keepalives for it), so its last_ms is
        // frozen at creation and its age grows without bound -- which makes it
        // the oldest context on the box within minutes and therefore ALWAYS the
        // first thing an age-ordered scan evicts. Observed on the bench: an
        // A16R renegotiating four flows evicted the operator's multicast
        // immediately, every time, while genuinely stale unicast contexts
        // survived. Age is simply not a meaningful staleness signal for
        // multicast, so rank it last instead of first.
        uint32_t oldest = 0; f = -1;
        for (unsigned i = 0; i < N_FLOWS; i++) {
            if (flows[i].mcast) continue;
            uint32_t age = now_ms - flows[i].last_ms;
            if (f < 0 || age >= oldest) { oldest = age; f = (int)i; }
        }
        if (f < 0) {
            // Every context is multicast -- fall back to age order among them
            // rather than refusing the bind.
            oldest = 0; f = 0;
            for (unsigned i = 0; i < N_FLOWS; i++) {
                uint32_t age = now_ms - flows[i].last_ms;
                if (age >= oldest) { oldest = age; f = (int)i; }
            }
        }
        printf("[dtx] all contexts busy -- evicting %d (%s, idle %lu s)\n",
               f, flows[f].mcast ? "multicast" : "unicast",
               (unsigned long)((now_ms - flows[f].last_ms) / 1000));
        flows[f].in_use = 0;
    }

    write_ctx((unsigned)f, dst_ip, dmac, dst_port, chans, nslots, fpp);

    if (!flows[f].in_use) {
        flows[f].in_use = 1;
        flows[f].rebinds++;
        // Never 0: a receiver that sends a zeroed handle is sending nothing
        // meaningful, and that must not match a live flow.
        static uint16_t cookie_seq;
        if (++cookie_seq == 0) cookie_seq = 1;
        flows[f].cookie = cookie_seq;
        flows[f].ka_rx = 0;
        flows[f].ka_ms = 0;
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

int32_t dante_tx_get_ts_offset(void) { return ts_offset_samples; }

void dante_tx_set_ts_offset(int32_t v)
{
    // Re-anchoring mid-stream is a timestamp DISCONTINUITY: one packet advances
    // by other than fpp, and a receiver may click or briefly mute. That is
    // acceptable for calibration and is not something to do while anyone is
    // listening for pleasure -- which is precisely why this is a diagnostic
    // knob and not part of any control loop.
    ts_offset_samples = v;
    printf("[dtx] ts offset -> %d samples, re-anchoring\n", (int)v);
    ts_anchor();
}

void dante_tx_expire(void)
{
    // RELEASE A FLOW WHOSE RECEIVER HAS STOPPED SENDING KEEPALIVES -- where
    // "keepalive" is what was MEASURED on the wire, not what the flow-control
    // port was assumed to do.
    //
    // THE HISTORY THIS REPLACES. Two earlier versions keyed on the age of the
    // last 4455 flow request and both silenced a LIVE flow, at 300 s and then
    // at 600 s:
    //
    //   [dtx] flow 4 stale (600 s, 169.254.61.114:14405) -- releasing
    //
    // an AM2 whose subscription was live and whose audio stopped when we
    // released its context. The mistake was not the number. It was that 4455
    // requests are not a heartbeat at all: UNICAST_FLOWS.md measured receivers
    // "repeating every ~5 s" while they were being REJECTED, and once we
    // started accepting, refreshes stopped entirely. Measured 2026-11-05 on
    // this bench, with both flows playing audio:
    //
    //   flow 0 -> 169.254.127.2:14337   age 105.5 -> 165.5 s, +10 s per 10 s
    //   flow 1 -> 169.254.145.57:14337  age  86.4 s and climbing
    //
    // monotonic, zero refreshes, and `tcpdump 'udp port 4455'` saw 0 packets in
    // 20 s. No timeout on that signal can be both safe and useful.
    //
    // WHAT THE RECEIVERS ACTUALLY SEND. They talk back on OUR media port 4321,
    // from the same socket they receive the flow on. Captured off the board's
    // own ring (the host cannot see it -- unicast, unmanaged switch):
    //
    //   169.254.145.57  Leine Prodigy, server_vers 4.2.0, a REAL Dante device:
    //                   02 00 00 00 <u32 LE counter>, every 250 ms exactly,
    //                   counter incrementing once per second
    //   169.254.127.2   N-Series DAC8: 13 37 00.., constant, every ~104 ms
    //
    // So this is read off the wire from a shipping Dante receiver, not invented
    // and not a private contract with one device. media_rx() counts them.
    //
    // NO TIMESTAMP ARITHMETIC. The decision is "did the counter move since the
    // previous poll", never "how long ago was the last one". gptp_uptime_ms()
    // is PTP-derived and a clock STEP is exactly what defeated every previous
    // attempt here -- it makes every age enormous at once and expires the lot.
    // A step can now only delay a poll; it cannot produce a teardown.
    //
    // A FLOW IS ONLY EVER EXPIRED IF IT HAS SENT AT LEAST ONE. A RedNet A16R
    // and an AM2 were on this bench and are not now, so whether they use this
    // port is UNMEASURED. Until it is, they behave exactly as before: their
    // contexts are reclaimed by an explicit 0x0101 stop or by eviction on
    // demand, never by this.
    //
    // 5 consecutive silent seconds. The Prodigy sends 4/s and the DAC8 ~10/s,
    // so one silent poll already means 4 to 10 missed in a row; five of them is
    // a margin of more than an order of magnitude over the observed cadence.
    enum { KA_MISS_LIMIT = 5 };

    static uint32_t next_ms;
    static uint32_t seen[N_FLOWS];
    static uint8_t  miss[N_FLOWS];

    uint32_t now = gptp_uptime_ms();
    if (next_ms == 0) { next_ms = now + 1000u; return; }      // arm on first call
    int32_t due = (int32_t)(now - next_ms);
    if (due < 0) {
        // Clock stepped BACKWARDS far enough that we would stall. Re-arm rather
        // than wait it out, but decide nothing this time round.
        if (due < -10000) next_ms = now + 1000u;
        return;
    }
    if (due > 10000) {                  // stepped forward: re-arm, decide nothing
        next_ms = now + 1000u;
        return;
    }
    next_ms = now + 1000u;

    for (unsigned f = 0; f < N_FLOWS; f++) {
        // MULTICAST HAS NO SUBSCRIBER. A multicast flow is created once by ARC
        // 0x2201 and nothing refreshes it or talks back to us about it, so
        // there is no keepalive to miss. Ageing one out does not reclaim an
        // abandoned context, it kills a stream that works -- measured here at
        // 391 s and still carrying real audio at -6 dBFS.
        if (!flows[f].in_use || flows[f].mcast) { miss[f] = 0; continue; }

        uint32_t ka = flows[f].ka_rx;
        if (ka == 0) { miss[f] = 0; continue; }   // never spoke to 4321: not ours to judge
        if (ka != seen[f]) { seen[f] = ka; miss[f] = 0; continue; }

        if (++miss[f] < KA_MISS_LIMIT) continue;

        printf("[dtx] flow %u keepalives stopped (%u s silent after %lu, "
               "%u.%u.%u.%u:%u) -- releasing\n",
               f, miss[f], (unsigned long)ka,
               flows[f].dst[0], flows[f].dst[1], flows[f].dst[2],
               flows[f].dst[3], flows[f].dport);
        miss[f] = 0;
        seen[f] = 0;
        dante_tx_unbind_why(f, DTX_UB_EXPIRE);
    }
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
    if (f < 0) {
        // EVICT THE LEAST-RECENTLY-BOUND rather than refusing, which is what
        // dante_tx_bind_unicast already does. Refusing produced
        // "[arc] 2201: no free context for flow 32" on the bench with all six
        // slots held -- four by an A16R taking only FOUR channels per flow, one
        // by an AM2, and one by an A16R context it had abandoned when it moved
        // ports. The abandoned one was the obvious candidate and nothing would
        // take it.
        //
        // Evicting on demand is strictly better than waiting for the timer:
        // it reclaims only when a slot is actually needed and always picks the
        // stalest, so it cannot silence a flow that is still being refreshed.
        uint32_t now = gptp_uptime_ms(), oldest = 0;
        f = 0;
        for (unsigned i = 0; i < N_FLOWS; i++) {
            uint32_t age = now - flows[i].last_ms;
            if (age > 3600000u) age = 3600000u;      // clock moved; do not trust
            if (age >= oldest) { oldest = age; f = (int)i; }
        }
        printf("[dtx] multicast needs a context -- evicting %d (stale %lu s)\n",
               f, (unsigned long)(oldest / 1000));
        dante_tx_unbind((unsigned)f);
    }

    const uint8_t *dip = flow_ip[f];
    uint8_t dmac[6] = { 0x01, 0x00, 0x5E, (uint8_t)(dip[1] & 0x7F), dip[2], dip[3] };
    // fpp is settable so a multicast stream -- the only stream visible on the
    // wire from the build host -- can be put at the SAME fpp a unicast receiver
    // negotiated. DVS asks for 60 and stays red while the fpp=16 multicast it
    // also receives is green, so the fault is fpp=60-specific and could not be
    // decoded until now: unicast is never flooded to the capture port.
    write_ctx((unsigned)f, dip, dmac, DANTE_PORT_MEDIA, chans, n, g_mcast_fpp);

    flows[f].in_use = 1;
    for (int i = 0; i < 4; i++) flows[f].dst[i] = dip[i];
    flows[f].dport = DANTE_PORT_MEDIA;
    // RE-ANNOUNCE. mdns_announce() ran three times at BOOT, when there were no
    // multicast flows -- bundles are created later, by ARC 0x2201 -- so the
    // _netaudio-bund PTR set was empty every time we announced and nothing ever
    // told the network a bundle had appeared.
    //
    // The responder has always been able to ANSWER a bundle query
    // (match_bund_inst, W_BUND_PTR); the gap was purely that nobody knows to
    // ask about a bundle whose existence was never advertised. Measured on the
    // bench: a 70 s mDNS capture showed _netaudio-arc/-chan/-cmc from us and no
    // -bund at all, while the A16R -- which does advertise its bundle -- is the
    // one device Dante Controller attributes transmit bandwidth to.
    mdns_announce();
    flows[f].nslots = n; flows[f].fpp = (uint8_t)g_mcast_fpp; flows[f].mcast = 1;
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

// Drop every bound flow so receivers renegotiate NOW.
//
// A receiver reads our advertised latency when it SETS UP a flow, not while one
// is running, so changing latency_ns had no effect on live audio until each
// receiver happened to re-subscribe -- which is the "it changes but it takes a
// while" this exists to fix. Releasing the contexts makes every receiver
// re-request on its next keepalive (~5 s observed) and re-read the new value.
//
// This DOES interrupt audio briefly. That is inherent: changing latency is a
// re-buffering event, and a receiver cannot move its playout point without a
// discontinuity. Real Dante hardware glitches on a latency change too. The
// caller must therefore only invoke this when the value ACTUALLY changed --
// Controller re-sends the same latency on every view refresh, and dropping
// flows for a no-op change would be an unexplained dropout every time an
// operator opened a tab.
// EFFECTIVE ADVERTISED LATENCY = max(what was selected, what the bound flows
// can actually meet).
//
// Receivers use our advertised latency_ns AS the flow's playout latency -- not
// max() with their own setting. Proven on the bench: a RedNet AM2's flow
// request is BYTE-IDENTICAL at a 0.25 ms and a 0.5 ms advertisement (same fpp,
// same channels, only the sequence number differs), yet it reports 0.27 ms of
// latency at 0.5 and 1.9 ms at 0.25. It is buffering exactly what we tell it.
//
// A packet cannot exist until fpp samples after its own timestamp, so a flow at
// fpp=16 needs 333 us and CANNOT be served inside a 250 us budget however good
// the transmit side is. Advertising 0.25 with that flow bound guarantees late
// packets and a red Latency Status.
//
// So do what Audinate documents a transmitter doing: "if a transmitter cannot
// support the configured latency, Dante increases it to the lowest supported
// value." Select 0.25 and an AM2 subscribes -> we publish 0.5 and nothing goes
// red. The AM2 leaves -> we drop back to 0.25 on its own.
//
// STRICTLY greater than the window, not >=: fpp=48 is exactly 1.000 ms, and a
// 1 ms budget for a packet that takes 1 ms to assemble leaves nothing for
// transit. Only the five standard Dante values are used -- 0.34 ms and 0.40 ms
// were tried on the bench and a receiver treats them as it treats 0.25.
uint32_t dante_tx_latency_effective(void)
{
    // OFF BY DEFAULT once measured: with the advertised fpp maximum capped, a
    // receiver takes max(our floor, its own setting) and sorts itself out --
    // A16R 0.25 ms and AM2 1.0 ms concurrently against a 0.25 ms floor. The
    // auto-raise then does nothing but drag every fast receiver up to whatever
    // the slowest one needs, which is the opposite of what it was added for.
    if (!g_latency_autoraise) return g_latency_ns;

    static const uint32_t std_lat[5] = {250000u, 500000u, 1000000u,
                                        2000000u, 5000000u};
    uint32_t worst = 0;
    for (unsigned i = 0; i < N_FLOWS; i++) {
        if (!flows[i].in_use || !flows[i].fpp) continue;
        // fpp samples at the active rate, in nanoseconds. 1e9/48000 = 20833.33
        // (10416.67 at 96k); use the
        // 64-bit form so fpp=60 does not round down into a too-small window.
        uint32_t win = (uint32_t)(((uint64_t)flows[i].fpp * 1000000000ull) / (uint64_t)g_fs);
        if (win > worst) worst = win;
    }
    uint32_t need = 5000000u;
    for (unsigned i = 0; i < 5; i++)
        if (std_lat[i] > worst) { need = std_lat[i]; break; }
    return (need > g_latency_ns) ? need : g_latency_ns;
}

void dante_tx_drop_all(void)
{
    unsigned n = 0;
    for (unsigned f = 0; f < N_FLOWS; f++)
        if (flows[f].in_use && dante_tx_unbind_why(f, DTX_UB_DROP) == 0) n++;
    printf("[dtx] dropped %u flow(s) to force renegotiation\n", n);
}

// Tear a flow down on request (ARC 0x2202). nslots = 0 makes the builder skip
// the context entirely.
int dante_tx_unbind(unsigned f)
{
    return dante_tx_unbind_why(f, DTX_UB_ARC);
}

int dante_tx_unbind_why(unsigned f, uint8_t why)
{
    if (f >= N_FLOWS || !flows[f].in_use) return -1;
    flows[f].in_use = 0;
    aaf_pkt_ctx_select_write(f);
    aaf_pkt_flow_cfg_write(0);
    switch (why) {
    case DTX_UB_STOP:   g_tx_stats.ub_stop++;   break;
    case DTX_UB_EXPIRE: g_tx_stats.ub_expire++; break;
    case DTX_UB_DROP:   g_tx_stats.ub_drop++;   break;
    default:            g_tx_stats.ub_arc++;    break;
    }
    // De reden gaat MEE in het event, niet alleen in een teller: de tellers
    // zeggen hoeveel, de events zeggen in welke volgorde -- en de vraag hier
    // was juist of de unbind voor of na het uitzetten van de talker kwam.
    telem_event(TELEM_E_FLOW_UNBIND, (int32_t)f, (int32_t)why);
    printf("[dtx] flow %u deleted (why=%u)\n", f, why);
    return 0;
}

uint16_t dante_tx_flow_cookie(unsigned f)
{
    return (f < N_FLOWS) ? flows[f].cookie : 0;
}

// Release a flow the receiver asked us to stop (flow control 0x0101).
//
// WHY THIS IS NOT THE EXPIRY TIMER AGAIN. Every timeout tried in this file
// guessed that silence meant death and was wrong -- 300 s and 600 s each
// silenced a flow that was still playing. This is the opposite kind of signal:
// the receiver states that it is finished with a flow it can name. There is no
// interval to tune and nothing to be wrong about, which is why it is safe where
// a timer was not.
//
// Both halves of the handle must match. The index alone is not enough: indices
// are reused, so a stale handle naming slot 2 would otherwise tear down
// whatever was bound into slot 2 afterwards.
int dante_tx_release_handle(uint32_t idx, uint16_t cookie)
{
    if (idx >= N_FLOWS) return -1;
    if (!flows[idx].in_use) return -1;
    if (flows[idx].cookie != cookie) return -1;
    if (flows[idx].mcast) return -1;      // multicast is torn down by ARC 0x2202
    return dante_tx_unbind_why((unsigned)idx, DTX_UB_STOP);
}

// Count datagrams a receiver sends to our media port. OBSERVATION ONLY.
static void media_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *p, uint32_t len)
{
    // Multicast on 4321 is audio between other devices, flooded to us by the
    // switch. Drop it on the first line: this handler sits in the path of the
    // highest-rate traffic on the segment.
    if (dst_ip[0] >= 224) return;

    uint32_t now = gptp_uptime_ms();
    int hit = -1;
    for (unsigned f = 0; f < N_FLOWS; f++)
        if (flows[f].in_use && !flows[f].mcast && ip4_eq(flows[f].peer, src_ip)) {
            hit = (int)f; break;
        }
    if (hit >= 0) { flows[hit].ka_rx++; flows[hit].ka_ms = now; }

    // Print the first few in full and then at most one every 10 s. The payload
    // bytes matter: a keepalive reported as "13 37" needs to be read, not
    // assumed, before anything keys off it.
    static uint32_t shown, last_ms;
    if (shown < 8u || (uint32_t)(now - last_ms) > 10000u) {
        shown++; last_ms = now;
        printf("[dtx] 4321 rx %u.%u.%u.%u:%u len %lu flow %d:",
               src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port,
               (unsigned long)len, hit);
        for (uint32_t i = 0; i < len && i < 16u; i++) printf(" %02x", p[i]);
        printf("\n");
    }
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
void dante_tx_read_emitted(uint32_t *sec, uint32_t *sub)
{
    // Seqlock on the seconds field: read sec, sub, sec again. If seconds moved
    // between the two reads the pair straddled a boundary, so retry. Bounded --
    // at 3000-6000 packets/s a boundary is crossed once per second, so a second
    // consecutive straddle is not physically reachable, but the loop is capped
    // anyway rather than trusted.
    for (int i = 0; i < 4; i++) {
        uint32_t s1 = aaf_pkt_dbg_last_sec_read();
        uint32_t sb = aaf_pkt_dbg_last_ts_read();
        uint32_t s2 = aaf_pkt_dbg_last_sec_read();
        if (s1 == s2) { *sec = s1; *sub = sb; return; }
    }
    *sec = aaf_pkt_dbg_last_sec_read();
    *sub = aaf_pkt_dbg_last_ts_read();
}

// ---- The media clock's phase against PTP, measured so it cannot lie ---------
//
// This is the signal the phase trim in mcr_dante.c closes on, and getting the
// MEASUREMENT wrong is what broke that loop on 2026-08-11: it was fed values
// whole seconds out (-47959 = -48000 + 41, then -335977 = -7 x 48000 + 23),
// drove its integral to the +2000 ppb rail inside a minute, and cost ten
// re-anchors and an audio interruption. See DRIFT_HANDOFF.md.
//
// The measurement it replaces was
//
//     emit_sec*48000 + emit_sub  -  (ptp_sec*48000 + ptp_ns_in_samples)
//
// and DRIFT_HANDOFF.md blamed a one-second read straddle. That was only half
// right, and the half it named was already fixed:
//
//   * The STRADDLE is real but handled. dbg_last_sec and dbg_last_ts are two
//     CPU reads and the gateware re-latches the pair between them at the packet
//     rate, so a read across the seconds boundary returns sec_old with sub_new.
//     dante_tx_read_emitted() above seqlocks exactly that away, and
//     gptp_read_time() is latched in gateware (avb_soc.py: reading seconds_hi
//     snapshots seconds AND nanoseconds together). Both pairs were already
//     atomic when the loop ran away.
//
//   * STALENESS is what actually broke it, and nothing guarded it.
//     dbg_last_{sec,ts} hold the LAST EMITTED packet's timestamp, and the media
//     counter behind them is gated on `en` (dante_packetizer.py:
//     strobe = mcr.sample_strobe & en). Drop the talker -- a flow expiring,
//     dante_tx_drop_all(), the re-anchor path -- and the register FREEZES while
//     PTP keeps running, so the difference walks at exactly -48000 samples per
//     second of silence. -335977 is seven seconds of that. A straddle can only
//     ever be +-1 second, so the -7 s reading is proof on its own that
//     staleness, not straddle, was the mechanism.
//
// This version removes both, by construction rather than by retry:
//
//   * NO SECONDS FIELD ON EITHER SIDE. The media side reads the LIVE subsecond
//     counter (ts_now_sub, 0..47999); the PTP side uses only nanoseconds. A
//     quantity with no seconds field cannot be a second out, whatever the two
//     reads do relative to each other. The true phase is held far inside +-0.5 s
//     by the re-anchor backstop (8 samples), and even a wholly undisciplined
//     crystal at 5 ppm needs 28 hours to accumulate half a second, so folding
//     into +-24000 discards nothing that was ever real.
//
//   * LIVENESS IS PROVEN, NOT ASSUMED. A frozen counter does not read as a slow
//     ramp: against a running PTP it sweeps the whole +-24000 range once per
//     second, so a magnitude guard alone still passes roughly a tenth of the
//     samples -- with a random sign. Wait for the counter to actually tick and
//     report NO sample rather than a plausible-looking one if it never does.
//
//   * The counter is read either side of the PTP read and a disagreement
//     retries, so the residual error is under one sample.
//
// The cost is a spin of up to one sample period (20.8 us at 48 kHz, ~1040 sys
// clocks at 50 MHz) once per caller. mcr_dante_poll() calls this at 1 Hz.
// Was een #define op 48000. Nu de actieve rate, want deze constante vouwt het
// fase-verschil naar +-fs/2; met de verkeerde waarde valt elke meting buiten de
// band en wordt de re-anchor-guard permanent getriggerd.
#define MEDIA_SUBSEC        (g_fs)
// Bound the liveness spin. Each CSR read is a handful of cycles through the
// registered bridge, so one sample period is on the order of a hundred reads;
// 2000 is ~20x that and still well under a millisecond, which is what a main
// loop running at ~4 kHz can absorb once a second. Exceeding it means the media
// strobe has stopped, which is a fault to report, not to block on.
#define MEDIA_TICK_SPIN_MAX 2000u

int dante_tx_media_phase(int32_t *out_samples)
{
    // The counter only advances while the packetizer is enabled. With the
    // talker off there is no media clock phase to measure -- not a small error,
    // no quantity at all -- and this is the case the old code silently reported
    // a number for.
    if (!talker_on)
        return 0;

    uint32_t s = aaf_pkt_ts_now_sub_read();
    unsigned spin = 0;
    while (aaf_pkt_ts_now_sub_read() == s) {
        if (++spin > MEDIA_TICK_SPIN_MAX)
            return 0;              // media strobe stopped: no sample
    }

    for (unsigned tries = 0; tries < 4; tries++) {
        uint32_t a = aaf_pkt_ts_now_sub_read();
        ptp_timestamp_t t = gptp_read_time();
        uint32_t b = aaf_pkt_ts_now_sub_read();
        if (a != b)
            continue;              // a media tick landed inside the window
        // Neither of these can happen with healthy hardware. Report no sample
        // rather than fold an out-of-range value into a plausible one.
        if (a >= MEDIA_SUBSEC || t.nanoseconds >= 1000000000u)
            return 0;

        int32_t ptp_sub = (int32_t)((t.nanoseconds * 3u) / (3000000000u / g_fs));
        int32_t d = (int32_t)a - ptp_sub;
        // *** SIGNED FOLD. MEDIA_SUBSEC IS g_fs EN DIE IS uint32_t. ***
        // Zonder deze casts promoveert C de negatieve `d` naar unsigned in de
        // vergelijking, wordt elke negatieve fase een gigantisch positief
        // getal, is de test ALTIJD waar, en wordt er nog een hele seconde
        // afgetrokken van iets dat al negatief was:
        //     d = -11  ->  (uint32_t)-11 = 4294967285 >= 48000  ->  waar
        //              ->  d = -11 - 96000 = -96011
        // Dat is exact wat de console meldde: "media clock phase -96011 samples
        // (-1000114 us) off -- re-anchoring", 140 keer in 12 seconden, terwijl
        // de werkelijke fase -11 tot -17 samples was (los nagemeten uit
        // last_sec/last_ts tegen ptp_now: -0,2 ms, rotsvast over 88 s).
        //
        // GEVOLGSCHADE, en dat is waarom dit zo lang onzichtbaar bleef: de
        // re-anchor zelf zet de teller op de JUISTE waarde ("offset 0"), dus
        // geen enkele teller werd ongezond. Wat het kostte was de printf-vloed
        // op 10 Hz, die de main loop zo lang ophield dat RX-frames wegvielen --
        // 3,6 van 44 frames/s, 8% -- en daar zaten PTP-berichten tussen. Vandaar
        // offset-uitschieters van -17 tot -50 us en af en toe een UDP-timeout.
        //
        // DEZE BUG BESTOND NIET TOEN MEDIA_SUBSEC EEN LITERAL 48000 WAS: int
        // tegen int gaat goed. Hij is geintroduceerd toen de constante
        // rate-instelbaar werd. Let hierop bij elke andere plek waar een
        // int-expressie tegen g_fs wordt vergeleken.
        const int32_t _half = (int32_t)(MEDIA_SUBSEC / 2u);
        const int32_t _full = (int32_t)MEDIA_SUBSEC;
        if (d >= _half)       d -= _full;
        else if (d < -_half)  d += _full;
        *out_samples = d;
        return 1;
    }
    return 0;
}

int dante_tx_flow_detail(unsigned f, dante_tx_flow_detail_t *out)
{
    if (f >= N_FLOWS) return 0;
    const flow_slot_t *s = &flows[f];
    out->in_use = s->in_use;
    for (int i = 0; i < 4; i++) { out->peer[i] = s->peer[i]; out->dst[i] = s->dst[i]; }
    out->dport  = s->dport;
    out->nslots = s->nslots;
    out->fpp    = s->fpp;
    out->mcast  = s->mcast;
    out->age_ms = gptp_uptime_ms() - s->last_ms;
    out->cookie = s->cookie;
    out->ka_rx  = s->ka_rx;
    out->ka_age_ms = s->ka_ms ? (gptp_uptime_ms() - s->ka_ms) : 0xFFFFFFFFu;
    for (int i = 0; i < 8; i++) out->chans[i] = s->chans[i];
    return s->in_use;
}

void dante_tx_flow_mac(unsigned f, uint8_t mac[6])
{
    for (int i = 0; i < 6; i++) mac[i] = (f < N_FLOWS) ? flows[f].dmac[i] : 0;
}
