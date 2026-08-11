// Dante flow-control server, port 4455 — the missing piece for a stream.
//
// Our channel records advertise SRV <host>:4455. A receiver that resolves a
// channel connects HERE to ask for a flow; with nothing listening, resolution
// can succeed and the subscription still fails, which is exactly what we were
// seeing. inferno has this as flows_control_server.rs; we had no equivalent.
//
// Request format from flows_control_server.rs, opcode1 = 0x0100 "request flow":
//
//    0  2  hostname_offset        absolute from packet start
//    2  4  sample_rate            u32
//    6  4  bits_per_sample        u32
//   10  2  1
//   12  2  num_channels
//   14  2  remote_descriptor_offset
//   16  .. channel indices, u16 each, 1-based (0 = unused slot)
//
// SCOPE: this accepts the request and answers it. It does NOT yet build a
// per-receiver unicast flow -- that needs a packetizer context bound to the
// requester's IP and MAC with a recomputed IP checksum, which the gateware
// already supports (the per-flow CSRs are written through ctx_select) but the
// firmware side does not yet drive. Answering at all is the step that lets a
// receiver get past this point, and the request it sends is the reference the
// unicast work needs.

#include "dante_flows.h"
#include "dante_dev.h"
#include "dante_msg.h"
#include "net.h"
#include "dante_tx.h"
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

dante_flows_stats_t g_flows_stats;

#define OP_REQUEST_FLOW   0x0100

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

static void flows_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip;
    if (len < DANTE_HDR_LEN) return;

    g_flows_stats.rx++;

    uint16_t op1 = dante_req_opcode1(req);
    uint16_t op2 = dante_req_opcode2(req);
    if (op2 != 0) return;                       // not a request

    const uint8_t *c = req + DANTE_HDR_LEN;
    uint32_t clen = len - DANTE_HDR_LEN;

    // Stash for the mirror BEFORE building into the shared TX buffer. The
    // question this exists to answer: does a receiver ask for something
    // DIFFERENT when we advertise 0.25 ms versus 0.5 ms? A RedNet AM2 reports
    // 0.27 ms of latency at a 0.5 ms floor and 1.9 ms at 0.25 ms, while its fpp
    // stays 16 and our transmit timing does not change at all -- measured 13
    // samples EARLY at the moment it claimed to be 1.9 ms late. Either the
    // request differs, or the difference is inside the receiver where we cannot
    // reach it. A byte-level diff of the two requests decides which.
    //
    // The console cannot do this job: it drops output under load, and the
    // [flow] lines are the first thing lost when 48 channel TXTs are sweeping.
    uint8_t  fmir[160];
    uint32_t fmirn = len > sizeof(fmir) ? sizeof(fmir) : len;
    int      do_fmirror = (g_arc_mirror_ip[0] | g_arc_mirror_ip[1] |
                           g_arc_mirror_ip[2] | g_arc_mirror_ip[3]) != 0;
    if (do_fmirror) memcpy(fmir, req, fmirn);

    uint8_t *buf = net_udp_payload_buf();
    dante_msg_t m;
    dante_msg_begin(&m, buf, req);
    uint16_t code = DANTE_CODE_OK;

    if (op1 == OP_REQUEST_FLOW && clen > 16) {
        uint32_t rate    = rd32(c + 2);
        uint32_t bits    = rd32(c + 6);
        uint16_t nch     = rd16(c + 12);
        uint16_t rdo     = rd16(c + 14);        // remote socket descriptor

        // fpp and the receiver's flow name sit AFTER the channel list, at
        // 16 + nch*2 + 6 -- the six skipped bytes are unidentified.
        uint16_t fpp = 0;
        uint32_t fo  = 16u + (uint32_t)nch * 2u + 6u;
        if (fo + 2 <= clen) fpp = rd16(c + fo);

        // The receiver tells us WHERE to send, in a 0x0802 socket descriptor
        // at an absolute offset. This is the piece a unicast flow needs and
        // multicast never did: destination IP and port come from the request,
        // not from a group we picked.
        uint8_t  dst[4] = {0,0,0,0};
        uint16_t dport  = 0;
        if (rdo >= DANTE_HDR_LEN && rdo + 8 <= len) {
            const uint8_t *rd = req + rdo;
            if (rd[0] != 0x08 || rd[1] != 0x02)
                printf("[flow] warn: expected 0x0802, got 0x%02x%02x\n", rd[0], rd[1]);
            dport = (uint16_t)((rd[2] << 8) | rd[3]);
            dst[0]=rd[4]; dst[1]=rd[5]; dst[2]=rd[6]; dst[3]=rd[7];
        }

        g_flows_stats.requests++;
        printf("[flow] %u.%u.%u.%u wants %u ch, %lu Hz %lu bit, fpp=%u -> %u.%u.%u.%u:%u\n",
               src_ip[0], src_ip[1], src_ip[2], src_ip[3], nch,
               (unsigned long)rate, (unsigned long)bits, fpp,
               dst[0], dst[1], dst[2], dst[3], dport);
        for (unsigned i = 0; i < nch && 16u + i*2u + 2u <= clen; i++)
            printf("[flow]   slot %u -> tx channel %u\n", i, rd16(c + 16 + i*2));

        // Error codes are the real ones from flows_control.rs, not invented:
        //   0x0301 sample rate mismatch, 0x0315 too many TX flows,
        //   0x0103 flow not found / expired. This previously answered 0x0016,
        //   which means nothing to a Dante receiver.
        if (rate != g_dante.sample_rate || bits != g_dante.bits_per_sample) {
            printf("[flow] rejected: we are %lu Hz / %u bit\n",
                   (unsigned long)g_dante.sample_rate, g_dante.bits_per_sample);
            code = 0x0301;
            g_flows_stats.rejected++;
        } else if (!dante_tx_fpp_supported(fpp) ||
                   (fpp == 2 && (nch & 1))) {
            // fpp must be in FPP_TABLE (every entry divides 48000, which is
            // what keeps the pacing phase aligned across the subsec wrap), and
            // fpp=2 with an odd slot count breaks pay_len % 4 == 0.
            printf("[flow] rejected: fpp %u unsupported (8/16/24/32/48/60/4/2)\n", fpp);
            code = 0x0301;
            g_flows_stats.rejected++;
        } else if (nch == 0 || nch > 8) {
            printf("[flow] rejected: %u slots, max 8\n", nch);
            code = 0x0315;
            g_flows_stats.rejected++;
        } else {
            // BUILD THE FLOW. Everything needed comes from the request: the
            // destination is the receiver's own socket descriptor, the slot map
            // is the channel list it named, and fpp is its choice, not ours.
            uint16_t chans[8];
            for (unsigned i = 0; i < nch; i++)
                chans[i] = (16u + i*2u + 2u <= clen) ? rd16(c + 16 + i*2) : 0;

            int f = dante_tx_bind_unicast(src_ip, dst, dport, chans, (uint8_t)nch,
                                          (uint8_t)fpp);
            if (f < 0) {
                printf("[flow] no free context\n");
                code = 0x0315;                  // too many TX flows
                g_flows_stats.rejected++;
            } else {
                g_flows_stats.accepted++;
                printf("[flow] bound context %d\n", f);

                // RETURN A FLOW HANDLE. We answered OK with EMPTY content;
                // inferno returns a 6-byte handle -- flow index as u32 BE then
                // a u16 cookie (flows_tx.rs:631-633). A receiver that cannot
                // identify the flow it was just given has nothing to reference
                // in a refresh, which is the best available explanation for
                // keepalives arriving every 20-30 MINUTES here against ~5 s
                // while we were still rejecting requests -- and for
                // subscriptions going stale until manually re-made.
                static uint16_t cookie;
                cookie++;
                dante_msg_u32(&m, (uint32_t)f);
                dante_msg_u16(&m, cookie);
            }
        }
    } else {
        printf("[flow] unhandled opcode1 %#06x len=%lu from %u.%u.%u.%u\n",
               op1, (unsigned long)clen,
               src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        code = 0x0103;                          // not supported, but ANSWERED
        g_flows_stats.unknown++;
    }

    uint32_t total = dante_msg_finish(&m, code);
    if (net_udp_commit(src_ip, src_port, DANTE_PORT_FLOWS, total,
                       NET_TOS_BEST_EFFORT) == 0)
        g_flows_stats.tx++;

    // Mirror AFTER the reply: both share net_udp_payload_buf(), and answering
    // the receiver matters more than the diagnostic.
    if (do_fmirror) {
        uint8_t *mp = net_udp_payload_buf();
        uint32_t n2 = 0;
        mp[n2++] = 'F'; mp[n2++] = 'L'; mp[n2++] = 'C'; mp[n2++] = '1';
        mp[n2++] = (uint8_t)(code >> 8); mp[n2++] = (uint8_t)code;
        mp[n2++] = src_ip[0]; mp[n2++] = src_ip[1];
        mp[n2++] = src_ip[2]; mp[n2++] = src_ip[3];
        for (uint32_t i = 0; i < fmirn; i++) mp[n2++] = fmir[i];
        net_udp_commit(g_arc_mirror_ip, 7780, DANTE_PORT_FLOWS, n2,
                       NET_TOS_BEST_EFFORT);
    }
}

void dante_flows_init(void)
{
    if (net_udp_bind(DANTE_PORT_FLOWS, flows_rx) != 0) {
        printf("[flow] BIND FAILED on %u -- udp table full, port will not answer\n",
               DANTE_PORT_FLOWS);
        return;
    }
    printf("[flow] flow control listening on %u\n", DANTE_PORT_FLOWS);
}
