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

    uint8_t *buf = net_udp_payload_buf();
    dante_msg_t m;
    dante_msg_begin(&m, buf, req);
    uint16_t code = DANTE_CODE_OK;

    if (op1 == OP_REQUEST_FLOW && clen >= 16) {
        uint32_t rate    = rd32(c + 2);
        uint32_t bits    = rd32(c + 6);
        uint16_t nch     = rd16(c + 12);

        g_flows_stats.requests++;
        printf("[flow] request from %u.%u.%u.%u: rate=%lu bits=%lu nch=%u\n",
               src_ip[0], src_ip[1], src_ip[2], src_ip[3],
               (unsigned long)rate, (unsigned long)bits, nch);

        // Reject what we cannot actually carry, rather than accepting and then
        // sending nothing. inferno answers a rate mismatch with a specific
        // error code; the same idea applies here.
        if (rate != g_dante.sample_rate || bits != g_dante.bits_per_sample) {
            printf("[flow] rejected: we are %lu Hz / %u bit\n",
                   (unsigned long)g_dante.sample_rate, g_dante.bits_per_sample);
            code = 0x0016;                      // sample-rate / format mismatch
            g_flows_stats.rejected++;
        } else {
            // Accepted, but no unicast flow is built yet -- see the scope note
            // at the top. The reply carries no flow descriptor, so a receiver
            // will not get audio from this alone; what it does is complete the
            // exchange instead of leaving it hanging.
            g_flows_stats.accepted++;
        }
    } else {
        printf("[flow] unhandled opcode1 %#06x len=%lu from %u.%u.%u.%u\n",
               op1, (unsigned long)clen,
               src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        code = 0x0022;                          // not supported, but ANSWERED
        g_flows_stats.unknown++;
    }

    uint32_t total = dante_msg_finish(&m, code);
    if (net_udp_commit(src_ip, src_port, DANTE_PORT_FLOWS, total,
                       NET_TOS_BEST_EFFORT) == 0)
        g_flows_stats.tx++;
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
