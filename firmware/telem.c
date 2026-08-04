// telem.c — streaming telemetry ring. See telem.h for why.

#include "telem.h"
#include "net.h"
#include "gptp.h"

#include <stdio.h>

#define TELEM_PORT   7779          // shares the stats port, opcode 't'

// Max records per drain reply. 20-byte header + 16 * 28 = 468 bytes, well
// under NET_MAX_PAYLOAD (1400) and well under the size at which this port has
// historically misbehaved. Deliberately conservative: the host polls, so a
// small cap costs nothing but poll frequency.
#define TELEM_MAX_PER_REPLY  16

typedef struct {
    uint32_t seq;
    uint32_t t_ms;
    uint8_t  type;
    uint8_t  flags;
    uint16_t aux;
    int32_t  v[4];
} telem_rec_t;

static telem_rec_t ring[TELEM_RING_RECS];
static uint32_t    next_seq = 1;      // seq 0 is never used, so 0 means "from oldest"
static uint32_t    dropped;           // records overwritten before the host read them

static void put32(uint8_t *p, uint32_t n, uint32_t v)
{
    p[n] = (uint8_t)(v >> 24); p[n+1] = (uint8_t)(v >> 16);
    p[n+2] = (uint8_t)(v >> 8); p[n+3] = (uint8_t)v;
}

void telem_push(uint8_t type, uint8_t flags, uint16_t aux,
                int32_t v0, int32_t v1, int32_t v2, int32_t v3)
{
    telem_rec_t *r = &ring[next_seq % TELEM_RING_RECS];
    // If the slot we are about to reuse still holds a record the host has not
    // drained, that is a real loss and the host must be told. It shows up as a
    // sequence gap, which the host tool reports rather than papering over --
    // silent truncation reads as "nothing happened", which is exactly the
    // failure mode this whole file exists to prevent.
    if (r->seq != 0 && (next_seq - r->seq) >= TELEM_RING_RECS)
        dropped++;

    r->seq   = next_seq++;
    r->t_ms  = gptp_uptime_ms();
    r->type  = type;
    r->flags = flags;
    r->aux   = aux;
    r->v[0]  = v0; r->v[1] = v1; r->v[2] = v2; r->v[3] = v3;
}

void telem_event(uint16_t event_id, int32_t a, int32_t b)
{
    telem_push(TELEM_T_EVENT, 0, event_id, a, b, 0, 0);
}

// Drain: request is 't' followed by a 4-byte big-endian sequence number the
// host wants next. 0 means "whatever you have oldest".
//
// Reply header (all big-endian):
//   [0..4)   'TLM1'  version tag -- the host refuses to parse anything else
//   [4..8)   record size in bytes
//   [8..12)  oldest seq still in the ring
//   [12..16) next seq the device will allocate
//   [16..20) records dropped since boot (ring overwritten before a drain)
//   [20..)   records
static void telem_rx(const uint8_t src_ip[4], uint16_t src_port,
                     const uint8_t *req, uint32_t len)
{
    uint32_t want = 0;
    if (len >= 5)
        want = ((uint32_t)req[1] << 24) | ((uint32_t)req[2] << 16) |
               ((uint32_t)req[3] << 8)  |  (uint32_t)req[4];

    uint32_t oldest = (next_seq > TELEM_RING_RECS)
                    ? (next_seq - TELEM_RING_RECS) : 1;
    if (want < oldest) want = oldest;

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    put32(p, n, 0x544C4D31u);   n += 4;      // 'TLM1'
    put32(p, n, TELEM_REC_BYTES); n += 4;
    put32(p, n, oldest);        n += 4;
    put32(p, n, next_seq);      n += 4;
    put32(p, n, dropped);       n += 4;

    unsigned emitted = 0;
    while (want < next_seq && emitted < TELEM_MAX_PER_REPLY) {
        const telem_rec_t *r = &ring[want % TELEM_RING_RECS];
        if (r->seq != want) { want++; continue; }   // overwritten under us
        put32(p, n, r->seq);          n += 4;
        put32(p, n, r->t_ms);         n += 4;
        put32(p, n, ((uint32_t)r->type  << 24) |
                    ((uint32_t)r->flags << 16) |
                     (uint32_t)r->aux);        n += 4;
        put32(p, n, (uint32_t)r->v[0]); n += 4;
        put32(p, n, (uint32_t)r->v[1]); n += 4;
        put32(p, n, (uint32_t)r->v[2]); n += 4;
        put32(p, n, (uint32_t)r->v[3]); n += 4;
        want++; emitted++;
    }

    net_udp_commit(src_ip, src_port, TELEM_PORT, n, NET_TOS_BEST_EFFORT);
}

void telem_init(void)
{
    // The stats port is already bound by dante_stats_init(); this hooks the
    // same port's 't' opcode through that dispatcher, so nothing extra to bind.
    (void)telem_rx;
    printf("[telem] ring %u recs x %u B, drain via udp %u opcode 't'\n",
           TELEM_RING_RECS, TELEM_REC_BYTES, TELEM_PORT);
}

// Called from dstats.c's dispatcher for opcode 't'.
void telem_drain(const uint8_t src_ip[4], uint16_t src_port,
                 const uint8_t *req, uint32_t len)
{
    telem_rx(src_ip, src_port, req, len);
}
