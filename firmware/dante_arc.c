// Dante ARC (Audinate Router Control) server, port 4440 — Phase 3.
//
// This is what makes the device appear in Dante Controller's Routing tab: DC
// asks for channel counts and then enumerates the transmit channels. Without
// it the device shows in the Device tab with no channels and cannot be routed.
//
// Structures follow inferno's protocol/proto_arc.rs + device_server/arc_server.rs,
// which is the reverse-engineering reference. Every offset written into a
// descriptor is ABSOLUTE FROM THE PACKET START (i.e. includes the 10-byte
// header) -- see dante_msg.h.

#include "dante_arc.h"
#include "dante_msg.h"
#include "dante_tx.h"
#include "dante_dev.h"
#include "mdns.h"
#include "net.h"
#include <string.h>
#include <stdio.h>

dante_arc_stats_t g_arc_stats;

// Per-RX-channel subscription, set by 0x3010 and reported back by 0x3000.
//
// 0x3010 is a SET, not a query. Dante Controller sends "subscribe RxN to
// <channel>@<host>" and expects the device to REMEMBER it -- then reads 0x3000
// back to confirm. Answering OK and discarding it makes a patch fail silently:
// we claim success and then contradict ourselves.
#define DANTE_MAX_SUBNAME 32
static char sub_tx_name[DANTE_RX_CHANNELS ? DANTE_RX_CHANNELS : 1][DANTE_MAX_SUBNAME];
static char sub_tx_host[DANTE_RX_CHANNELS ? DANTE_RX_CHANNELS : 1][DANTE_MAX_SUBNAME];

// inferno documents 0x01010009 for an active subscription and 0x00000001 for
// "remembers the subscription but has not resolved it". We use the latter: the
// subscription is stored and advertised, but we do not receive audio, so
// claiming an active flow would be a lie DC would act on.
#define SUB_STATUS_PENDING 0x00000001u

static void sub_copy(char *dst, const uint8_t *req, uint32_t len, uint16_t off)
{
    dst[0] = 0;
    if (!off || off >= len) return;
    uint32_t n = 0;
    while (off + n < len && req[off + n] && n < DANTE_MAX_SUBNAME - 1) {
        dst[n] = (char)req[off + n];
        n++;
    }
    dst[n] = 0;
}

// Opcodes. Names from inferno; the numeric values are what matters.
#define OP_CHANNELS_AND_FLOWS_COUNT   0x1000
#define OP_GET_DEVICE_NAME            0x1002
#define OP_GET_DEVICE_NAMES           0x1003
#define OP_UNKNOWN_1100               0x1100
#define OP_UNKNOWN_1102               0x1102
#define OP_GET_TX_CHANNELS            0x2000
#define OP_GET_TX_FRIENDLY_NAMES      0x2010
#define OP_QUERY_TX_FLOWS             0x2200
#define OP_UNKNOWN_2320               0x2320
#define OP_GET_RX_CHANNELS            0x3000
#define OP_QUERY_RX_FLOWS             0x3200
#define OP_UNKNOWN_3300               0x3300

// ---------------------------------------------------------------------------
// 0x1100 / 0x1102 -- the device property tables.
//
// These used to be 110 and 94 zero bytes (as inferno returns them). That is
// what made Dante Controller misclassify us: DC showed InfernoFPGA with PTPv2
// Domain 0 and Priority 0/0 and "N/A" under Primary v1 Multicast, while the
// two RedNets on the same network showed Leader/Follower under v1, "Disabled"
// under v2, and N/A for the PTPv2 fields. Exactly inverted -- DC was looking
// for us in the v2 clock domain, where we do not participate, which is why
// getting the PTPv1 offset down to 290 ns (parity with the A16R's 218 ns)
// changed nothing on screen.
//
// Queried from real hardware with tools/arc_query.py, both are structured:
//
//   0x1102   u16 count, then count x (u16 key, u16 type)
//   0x1100   same key set, u16 (flags<<8 | count), then count x
//            (u16 key, u16 value); keys with bit 15 set hold an OFFSET into a
//            data blob that follows the table rather than an inline value
//
// Verified on the AM2: count 0x1f = 31, and 2 + 31*4 = 126 = the exact 0x1102
// length. The 0x1100 blob holds 0x0000bb80 = 48000 among other things.
//
// This is a byte-exact replay of the RF04-RedNetAM2 (169.254.61.114), captured
// from hardware on this bench -- deliberately the device Dante Controller shows
// as *Follower*, which is the role we want. Replaying it verbatim is safe with
// respect to the offset fields: those offsets are absolute from the start of
// the packet (see dante_msg.h), and our header is the same 10 bytes, so every
// offset stays valid as long as the body is emitted unchanged.
//
// This is an EXPERIMENT, not a finished answer. It asserts another device's
// property values. If it flips DC to v1/Follower it confirms where the clock
// columns come from, and the next step is to decode the individual keys and
// report our own values rather than the AM2's.
// NOT const: patchable at runtime ('K' inline key, 'B' u32 in the data blob).
//
// THE LATENCY CAPABILITY LIVES IN THE DATA BLOB, addressed by the offset keys:
//
//     key      A16R        AM2        meaning
//     0x8204   1,000,000   1,000,000
//     0x8205     500,000   1,000,000
//     0x8301     500,000   1,000,000
//     0x8306     250,000   1,000,000  MINIMUM supported latency
//     0x8321   2,000,000   2,000,000
//
// A16R minimum 0.25 ms -> Dante Controller offers 0.25/0.5/1/2/5.
// AM2  minimum 1 ms    -> Controller offers 1/2/5.
//
// This table is an AM2 replay, so we inherited the AM2's 1 ms minimum and were
// offered 1/2/5 no matter what else we matched -- the capability word, the
// 0x1003 layout, the inline keys, the model ID, even the arcp version. Those
// were all dead ends. 0x8205/0x8301/0x8306 below are now OURS, not the AM2's.
//
// Found by sweeping opcodes against BOTH RedNets and searching their replies
// for the latency values as nanoseconds -- the encoding 0x1101 already uses.
// 0x1100 was the only opcode whose values tracked what each device offers.
//
// This table is a byte-exact replay of a RedNet AM2, and Dante Controller
// offers us exactly the AM2's latency choices -- 1/2/5 ms -- because that is
// literally the capability we claim. A RedNet A16R (Brooklyn-3), which DOES
// offer 0.25/0.5, differs on these inline keys and nowhere else that is not a
// message offset:
//
//     key      A16R    AM2/ours
//     0x0210     16       0
//     0x0211     16       0
//     0x0213      2       0
//     0x0214    128       0
//     0x0303      4       2
//     0x0311      2      16
//
// Which of those gates the latency list is not documented anywhere we have, and
// each guess otherwise costs a firmware push and an audio dropout. Patching at
// runtime turns that into seconds. Keys with bit 15 set are OFFSETS into the
// trailing blob and must never be patched -- their values are positions in a
// message whose length differs between devices, so copying the A16R's would
// point into the wrong bytes. arc_1100_patch() refuses them.
static uint8_t arc_1100_body[202] = {
    0x24, 0x1f, 0x80, 0x20, 0x00, 0x9c, 0x80, 0x21, 0x00, 0xa0, 0x00, 0x22,
    0x00, 0x01, 0x00, 0x23, 0x00, 0x18, 0x00, 0x24, 0x00, 0x01, 0x80, 0x60,
    0x00, 0xb0, 0x00, 0x62, 0x00, 0x01, 0x00, 0x63, 0x00, 0x01, 0x02, 0x01,
    0x00, 0x01, 0x82, 0x04, 0x00, 0xb4, 0x82, 0x05, 0x00, 0xb8, 0x02, 0x0a,
    0x00, 0x00, 0x02, 0x0b, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x02, 0x11,
    0x00, 0x00, 0x02, 0x12, 0x00, 0x30, 0x02, 0x13, 0x00, 0x00, 0x02, 0x14,
    0x00, 0x00, 0x02, 0x22, 0x13, 0x8c, 0x83, 0x01, 0x00, 0xbc, 0x83, 0x06,
    0x00, 0xc0, 0x83, 0x02, 0x00, 0xc4, 0x83, 0x21, 0x00, 0xc8, 0x03, 0x10,
    0x00, 0x10, 0x03, 0x11, 0x00, 0x10, 0x03, 0x12, 0x00, 0x30, 0x03, 0x03,
    0x00, 0x02, 0x83, 0xf0, 0x00, 0xcc, 0x06, 0x01, 0x00, 0x00, 0x03, 0x09,
    0x00, 0x03, 0x02, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xbb, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x45,
    0x00, 0x00, 0x00, 0x0f, 0x42, 0x40, 0x00, 0x07, 0xa1, 0x20, 0x00, 0x07,
    0xa1, 0x20, 0x00, 0x03, 0xd0, 0x90, 0x01, 0x35, 0xf1, 0xb4, 0x00, 0x1e,
    0x84, 0x80, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t arc_1102_body[126] = {
    0x00, 0x1f, 0x80, 0x20, 0x00, 0x01, 0x80, 0x21, 0x00, 0x03, 0x00, 0x22,
    0x00, 0x03, 0x00, 0x23, 0x00, 0x03, 0x00, 0x24, 0x00, 0x01, 0x80, 0x60,
    0x00, 0x03, 0x00, 0x62, 0x00, 0x03, 0x00, 0x63, 0x00, 0x01, 0x02, 0x01,
    0x00, 0x03, 0x82, 0x04, 0x00, 0x03, 0x82, 0x05, 0x00, 0x03, 0x02, 0x0a,
    0x00, 0x01, 0x02, 0x0b, 0x00, 0x01, 0x02, 0x10, 0x00, 0x03, 0x02, 0x11,
    0x00, 0x03, 0x02, 0x12, 0x00, 0x03, 0x02, 0x13, 0x00, 0x01, 0x02, 0x14,
    0x00, 0x01, 0x02, 0x22, 0x00, 0x03, 0x83, 0x01, 0x00, 0x03, 0x83, 0x06,
    0x00, 0x01, 0x83, 0x02, 0x00, 0x01, 0x83, 0x21, 0x00, 0x01, 0x03, 0x10,
    0x00, 0x01, 0x03, 0x11, 0x00, 0x01, 0x03, 0x12, 0x00, 0x01, 0x03, 0x03,
    0x00, 0x03, 0x83, 0xf0, 0x00, 0x01, 0x06, 0x01, 0x00, 0x01, 0x03, 0x09,
    0x00, 0x01, 0x02, 0x09, 0x00, 0x01,
};

// Soft cap on response size, so a paginated reply stays inside one datagram.
#define PACKET_SIZE_SOFT_LIMIT        800

#define MAX_CHANNELS_IN_FLOW          8
#define MAX_TX_FLOWS                  32
#define MAX_RX_FLOWS                  32

// ---------------------------------------------------------------------------
// CommonChannelsDescriptor -- 16 bytes, shared by every TX channel descriptor
// so the format is stated once per response rather than per channel.
// ---------------------------------------------------------------------------

static uint16_t put_common_descriptor(dante_msg_t *m)
{
    uint16_t off = (uint16_t)m->len;
    dante_msg_u32(m, g_dante.sample_rate);
    dante_msg_u8 (m, 1);                        // unknown1_1
    dante_msg_u8 (m, 1);                        // unknown2_1
    dante_msg_u16(m, g_dante.bits_per_sample);  // bits_per_sample_1
    dante_msg_u16(m, 0x400);                    // unknown3_400
    dante_msg_u16(m, g_dante.bits_per_sample);  // bits_per_sample_2
    dante_msg_u16(m, g_dante.bits_per_sample);  // bits_per_sample_3
    dante_msg_u16(m, 0xE);                      // pcm_type; real HW: "pcm=3 4"
    return off;
}

// ---------------------------------------------------------------------------
// Pagination
//
// Wire layout of a paginated response body:
//   [0] space_items  u8    how many slots we reserved
//   [1] actual_items u8    how many we filled
//   [2..] the item array (space_items * item_size, zero-filled)
//   [..]  strings, referenced by absolute offset from the items
//
// The request carries a 1-based start index at content[2..4]. When more items
// remain we answer with opcode2 = 0x8112 and DC asks again from where it left
// off; 48 channels take 3-4 round trips.
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t items_at;      // absolute offset of the item array
    uint32_t item_size;
    uint32_t space;         // slots reserved
    uint32_t actual;        // slots filled
} page_t;

static void page_begin(dante_msg_t *m, page_t *pg, uint32_t item_size, uint32_t space)
{
    if (space > 255) space = 255;
    pg->item_size = item_size;
    pg->space     = space;
    pg->actual    = 0;
    dante_msg_u8(m, (uint8_t)space);
    dante_msg_u8(m, 0);                    // actual_items, patched at end
    pg->items_at  = m->len;
    dante_msg_zeros(m, item_size * space);
}

static uint8_t *page_slot(dante_msg_t *m, page_t *pg)
{
    return m->buf + pg->items_at + pg->actual * pg->item_size;
}

static void page_end(dante_msg_t *m, page_t *pg)
{
    m->buf[pg->items_at - 1] = (uint8_t)pg->actual;
}

static inline void put_u16_at(uint8_t *p, uint32_t off, uint16_t v)
{
    p[off] = (uint8_t)(v >> 8); p[off + 1] = (uint8_t)v;
}

// ---------------------------------------------------------------------------
// Opcode handlers
// ---------------------------------------------------------------------------

// ARC REQUEST MIRROR -- forward every request we receive to a collector over
// UDP, so what Dante Controller sends can be read off a machine that is not in
// the unicast path.
//
// The console CANNOT do this job. Its output is dropped under load: a printf in
// the 'L' handler never appeared even though the opcode demonstrably ran and
// returned the right value, while [flow] lines from the same network path came
// through. A diagnostic that silently loses the one line you need is worse than
// none. The unhandled-opcode logger also fires once per boot, so anything DC
// sent before the capture started is invisible.
//
// Off unless a collector is set (tools/stats.py opcode 'A'), so it costs one
// compare per request in normal operation.
uint8_t g_arc_mirror_ip[4];

// Patch one inline (key, value) in the 0x1100 property table.
// Layout: u16 header (flags<<8 | count), then count x (u16 key, u16 value).
// 0x1000 capability bytes; see the OP_CHANNELS_AND_FLOWS_COUNT case.
uint16_t g_router_vers = 0x0404;      // 4.4.0, matching our mDNS router_vers
uint16_t g_arcp_vers   = 0x280c;      // 2.8.12 -- the A16R's; ours advertises 2.8.9
uint8_t g_dev_flags0 = 0x00;
uint8_t g_dev_flags2 = (1u << 4) | (1u << 5);

static int arc_1100_patch(uint16_t key, uint16_t val)
{
    if (key & 0x8000) return -1;                  // offset key -- never patch
    uint32_t count = arc_1100_body[1];
    for (uint32_t i = 0; i < count; i++) {
        uint32_t o = 2 + i * 4;
        if (o + 4 > sizeof(arc_1100_body)) break;
        uint16_t k = (uint16_t)((arc_1100_body[o] << 8) | arc_1100_body[o + 1]);
        if (k == key) {
            arc_1100_body[o + 2] = (uint8_t)(val >> 8);
            arc_1100_body[o + 3] = (uint8_t)val;
            return 0;
        }
    }
    return -1;
}

// Write a u32 into the 0x1100 DATA BLOB, addressed by the offset key that
// points at it.
//
// THIS is where the latency capability lives, and it took far too long to find
// because arc_1100_patch() deliberately refused keys with bit 15 set as "just
// offsets". They are offsets -- to the u32s that decide what Dante Controller
// offers. Read off the bench:
//
//     key      A16R        AM2/ours     offers
//     0x8204   1,000,000   1,000,000
//     0x8205     500,000   1,000,000
//     0x8301     500,000   1,000,000
//     0x8306     250,000   1,000,000    <- minimum supported latency
//     0x8321   2,000,000   2,000,000
//
// A16R minimum 0.25 ms -> Controller offers 0.25/0.5/1/2/5.
// AM2  minimum 1 ms    -> Controller offers 1/2/5. We replay the AM2's table,
// which is exactly why we were offered 1/2/5 no matter what else we matched.
//
// Offsets in this protocol are ABSOLUTE from the start of the packet and our
// header is 10 bytes, so the blob index is (offset - DANTE_HDR_LEN).
int dante_arc_patch_1100_u32(uint16_t key, uint32_t val)
{
    uint32_t count = arc_1100_body[1];
    for (uint32_t i = 0; i < count; i++) {
        uint32_t o = 2 + i * 4;
        if (o + 4 > sizeof(arc_1100_body)) break;
        uint16_t k = (uint16_t)((arc_1100_body[o] << 8) | arc_1100_body[o + 1]);
        if (k != key) continue;
        uint32_t off = (uint32_t)((arc_1100_body[o + 2] << 8) | arc_1100_body[o + 3]);
        if (off < DANTE_HDR_LEN) return -1;
        uint32_t bi = off - DANTE_HDR_LEN;
        if (bi + 4 > sizeof(arc_1100_body)) return -1;
        arc_1100_body[bi + 0] = (uint8_t)(val >> 24);
        arc_1100_body[bi + 1] = (uint8_t)(val >> 16);
        arc_1100_body[bi + 2] = (uint8_t)(val >> 8);
        arc_1100_body[bi + 3] = (uint8_t)val;
        printf("[arc] 1100 key 0x%04x @%lu = %lu\n", key,
               (unsigned long)off, (unsigned long)val);
        return 0;
    }
    return -1;
}

int dante_arc_patch_1100(uint16_t key, uint16_t val)
{
    int r = arc_1100_patch(key, val);
    printf("[arc] 1100 key 0x%04x %s %u\n", key, r ? "NOT FOUND" : "->", val);
    return r;
}

static void arc_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                   uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip;
    if (len < DANTE_HDR_LEN) return;
    if (dante_req_opcode2(req) != 0) return;       // not a request

    uint16_t opcode = dante_req_opcode1(req);
    const uint8_t *content = req + DANTE_HDR_LEN;
    uint32_t clen = len - DANTE_HDR_LEN;

    g_arc_stats.rx++;

    // The request must be copied out before we build into the shared TX buffer:
    // net_udp_payload_buf() and the RX scratch are different buffers today, but
    // the header echo reads from `req` while we write into `buf`, so keep the
    // few bytes we need on the stack.
    uint8_t hdr[DANTE_HDR_LEN];
    memcpy(hdr, req, DANTE_HDR_LEN);

    // Stash for the mirror BEFORE building into net_udp_payload_buf(). `req`
    // must not be read after the reply is committed.
    uint8_t  mir[96];
    uint32_t mirn = len > sizeof(mir) ? sizeof(mir) : len;
    int      do_mirror = (g_arc_mirror_ip[0] | g_arc_mirror_ip[1] |
                          g_arc_mirror_ip[2] | g_arc_mirror_ip[3]) != 0;
    if (do_mirror) memcpy(mir, req, mirn);

    uint8_t   *buf = net_udp_payload_buf();
    dante_msg_t m;
    dante_msg_begin(&m, buf, hdr);
    uint16_t   code = DANTE_CODE_OK;

    switch (opcode) {

    case OP_CHANNELS_AND_FLOWS_COUNT: {
        // The response DC uses to size everything else.
        // flags2 is an LSB-first bitfield: bit4 = supports_tx_channel_rename,
        // bit5 = supports_tx_multicast. We claim both.
        // THE FIRST TWO BYTES ARE A CAPABILITY WORD, and ours claimed almost
        // nothing. Read off the bench with tools/arc_query.py:
        //
        //     A16R (offers 0.25/0.5)   0x0F 0xF9
        //     AM2  (offers 1/2/5)      0x0D 0xF9
        //     ours                     0x00 0x30
        //
        // Both real devices set flags2 = 0xF9; we set only bits 4 and 5. And
        // the two differ from each other in byte0 by exactly ONE bit -- 0x02 --
        // present on the device that offers the low latencies. That is the best
        // candidate for the capability gating Controller's latency list.
        //
        // Runtime-settable (opcode 'G') so the hypothesis can be tested against
        // Controller in seconds rather than one firmware push per guess.
        dante_msg_u8 (&m, g_dev_flags0);                   // unknown1_0
        dante_msg_u8 (&m, g_dev_flags2);                   // flags2
        dante_msg_u16(&m, DANTE_TX_CHANNELS);
        dante_msg_u16(&m, DANTE_RX_CHANNELS);
        dante_msg_u16(&m, 4);                              // unknown2_4
        dante_msg_u16(&m, MAX_CHANNELS_IN_FLOW);
        dante_msg_u16(&m, 8);                              // unknown4_8
        dante_msg_u16(&m, MAX_TX_FLOWS);
        dante_msg_u16(&m, MAX_RX_FLOWS);
        dante_msg_u16(&m, DANTE_TX_CHANNELS + DANTE_RX_CHANNELS);
        dante_msg_u16(&m, 1);                              // unknown6_1
        dante_msg_u16(&m, 1);                              // unknown7_1
        for (int i = 0; i < 6; i++) dante_msg_u16(&m, 0);  // unknown8_0[6]
        break;
    }

    case OP_GET_DEVICE_NAME:
        // Plain 0-terminated string. network-audio-controller uses this one.
        dante_msg_str(&m, g_dante.name);
        break;

    case OP_GET_DEVICE_NAMES: {
        // REWRITTEN to the layout real Dante devices actually emit.
        //
        // The previous version followed inferno's proto_arc.rs
        // get_device_names::ResponseHeader: a 38-byte header with the name
        // offsets at +12/+14/+16. Neither RedNet on this bench does that. Both
        // an A16R (Brooklyn-3) and an AM2 (UltimoX2) return a byte-identical
        // structure, differing only in their strings and version numbers, and
        // it does not match inferno's:
        //
        //   +0   0x001c        +18  0x0500        +34  router_vers (0x0404)
        //   +2   0x001c        +20  friendly      +38  arcp_vers   (0x280c)
        //   +4   0x0028        +22  factory       +40  0x0204
        //   +6   board str     +24  friendly      +42  0x1200
        //   +8   revision str  +30  0x0a0a        +44  0x1004
        //
        //   48-byte header, 2 pad bytes, then FIXED 32-byte name fields:
        //     friendly @ +50, factory @ +82, then board and revision inline.
        //   A16R total = 48 + 2 + 32 + 32 + 13 + 5 = 132 bytes. Exact.
        //
        // The version fields decode against those devices' own mDNS records,
        // which is what confirms the mapping rather than assuming it:
        //   AM2  0x0403 / 0x2809 = router 4.3.0, arcp 2.8.9   (mDNS agrees)
        //   A16R 0x0404 / 0x280c = router 4.4.0, arcp 2.8.12  (mDNS agrees)
        //
        // WHY THIS MATTERS: with the 0x1000 capability word claiming low-latency
        // support, Dante Controller starts asking for latency detail and parses
        // THIS reply. Against our old 95-byte format it failed with "Cannot
        // retrieve Device Latency", which is how the format mismatch was found.
        uint32_t head = m.len;
        dante_msg_zeros(&m, 48);
        dante_msg_zeros(&m, 2);                       // pad, as both devices send

        uint16_t friendly = (uint16_t)m.len;          // fixed 32-byte field
        dante_msg_bytes(&m, g_dante.name, strnlen(g_dante.name, 31));
        dante_msg_zeros(&m, 32 - strnlen(g_dante.name, 31));

        uint16_t factory = (uint16_t)m.len;           // fixed 32-byte field
        dante_msg_bytes(&m, g_dante.hostname, strnlen(g_dante.hostname, 31));
        dante_msg_zeros(&m, 32 - strnlen(g_dante.hostname, 31));

        uint16_t board    = dante_msg_str(&m, "N-Series AOIP");
        uint16_t revision = dante_msg_str(&m, ":705");

        put_u16_at(m.buf, head +  0, 0x001c);
        put_u16_at(m.buf, head +  2, 0x001c);
        put_u16_at(m.buf, head +  4, 0x0028);
        put_u16_at(m.buf, head +  6, board);
        put_u16_at(m.buf, head +  8, revision);
        put_u16_at(m.buf, head + 18, 0x0500);
        put_u16_at(m.buf, head + 20, friendly);
        put_u16_at(m.buf, head + 22, factory);
        put_u16_at(m.buf, head + 24, friendly);
        put_u16_at(m.buf, head + 30, 0x0a0a);
        // Runtime-settable (opcode 'V'): the two devices differ here as well as
        // in the capability word, and 2.8.12-vs-2.8.9 is a live candidate for
        // what actually gates the latency list. Being able to try it without a
        // firmware push is the difference between minutes and an hour.
        put_u16_at(m.buf, head + 34, g_router_vers);
        put_u16_at(m.buf, head + 38, g_arcp_vers);
        put_u16_at(m.buf, head + 40, 0x0204);
        put_u16_at(m.buf, head + 42, 0x1200);
        put_u16_at(m.buf, head + 44, 0x1004);
        break;
    }

    case OP_GET_TX_CHANNELS: {
        // 1-based start index, or 1 when absent.
        uint16_t start = (clen >= 4) ? dante_req_u16(content, 2) : 1;
        if (start == 0) start = 1;

        page_t pg;
        page_begin(&m, &pg, 8, 32);

        uint16_t common = 0;
        uint16_t idx    = start;
        for (; idx <= DANTE_TX_CHANNELS && pg.actual < pg.space; idx++) {
            if (!common) common = put_common_descriptor(&m);
            char nm[8];
            dante_tx_channel_name(idx, nm, sizeof(nm));
            uint16_t name_off = dante_msg_str(&m, nm);

            uint8_t *slot = page_slot(&m, &pg);
            put_u16_at(slot, 0, idx);            // channel_id, 1-based
            put_u16_at(slot, 2, 7);              // unknown1_7
            put_u16_at(slot, 4, common);
            put_u16_at(slot, 6, name_off);
            pg.actual++;
            if (m.len >= PACKET_SIZE_SOFT_LIMIT) { idx++; break; }
        }
        page_end(&m, &pg);
        if (idx <= DANTE_TX_CHANNELS) code = DANTE_CODE_MORE;
        break;
    }

    case OP_GET_TX_FRIENDLY_NAMES: {
        uint16_t start = (clen >= 4) ? dante_req_u16(content, 2) : 1;
        if (start == 0) start = 1;

        page_t pg;
        page_begin(&m, &pg, 6, 32);

        int wrote_pad = 0;
        uint16_t idx = start;
        for (; idx <= DANTE_TX_CHANNELS && pg.actual < pg.space; idx++) {
            if (!wrote_pad) { dante_msg_u32(&m, 0); wrote_pad = 1; }
            char nm[8];
            dante_tx_channel_name(idx, nm, sizeof(nm));
            uint16_t name_off = dante_msg_str(&m, nm);

            uint8_t *slot = page_slot(&m, &pg);
            put_u16_at(slot, 0, idx);
            put_u16_at(slot, 2, idx);
            put_u16_at(slot, 4, name_off);
            pg.actual++;
            if (m.len >= PACKET_SIZE_SOFT_LIMIT) { idx++; break; }
        }
        page_end(&m, &pg);
        if (idx <= DANTE_TX_CHANNELS) code = DANTE_CODE_MORE;
        break;
    }

    case OP_GET_RX_CHANNELS: {
        // 20-BYTE items, per proto_arc.rs get_receive_channels. The transmit
        // descriptor is 8 bytes and mirroring it here crashed DC -- it read 8
        // where it expected 20 and ran off the end of the array.
        //
        //   0  channel_id
        //   2  unknown1_6                (6 here; the transmit side uses 7)
        //   4  common_descriptor_offset
        //   6  tx_channel_name_offset    0 -- nothing subscribed yet
        //   8  tx_hostname_offset        0
        //  10  friendly_name_offset      our own label for the receive slot
        //  12  subscription_status u32   0 = not subscribed. inferno notes
        //                                0x01010009 subscribed, 0x00000001
        //                                remembered/in progress
        //  16  unknown2_0 u32
        uint16_t start = (clen >= 4) ? dante_req_u16(content, 2) : 1;
        if (start == 0) start = 1;

        page_t pg;
        page_begin(&m, &pg, 20, DANTE_RX_CHANNELS);

        uint16_t common = 0;
        uint16_t idx    = start;
        for (; idx <= DANTE_RX_CHANNELS && pg.actual < pg.space; idx++) {
            if (!common) common = put_common_descriptor(&m);
            char nm[8];
            nm[0] = 'R'; nm[1] = 'x';
            nm[2] = (char)('0' + idx); nm[3] = 0;
            uint16_t name_off = dante_msg_str(&m, nm);

            uint8_t *slot = page_slot(&m, &pg);
            put_u16_at(slot,  0, idx);
            put_u16_at(slot,  2, 6);
            put_u16_at(slot,  4, common);
            // Report the subscription 0x3010 stored, or zeros if none. DC
            // reads this back to confirm the patch took.
            uint16_t tx_off = 0, host_off = 0;
            uint32_t status = 0;
            if (sub_tx_name[idx - 1][0]) {
                tx_off   = dante_msg_str(&m, sub_tx_name[idx - 1]);
                host_off = sub_tx_host[idx - 1][0]
                         ? dante_msg_str(&m, sub_tx_host[idx - 1]) : 0;
                status   = SUB_STATUS_PENDING;
            }
            put_u16_at(slot,  6, tx_off);
            put_u16_at(slot,  8, host_off);
            put_u16_at(slot, 10, name_off);
            put_u16_at(slot, 12, (uint16_t)(status >> 16));
            put_u16_at(slot, 14, (uint16_t)status);
            put_u16_at(slot, 16, 0);
            put_u16_at(slot, 18, 0);
            pg.actual++;
            if (m.len >= PACKET_SIZE_SOFT_LIMIT) { idx++; break; }
        }
        page_end(&m, &pg);
        if (idx <= DANTE_RX_CHANNELS) code = DANTE_CODE_MORE;
        break;
    }

    case OP_QUERY_TX_FLOWS: {
        // Report the 6 multicast bundles we actually transmit.
        //
        // Dante Controller's Device View "Transmit" tab is built from this, and
        // with it stubbed at zero flows a subscription could go green while the
        // device still showed no flow at all -- which is exactly what we saw.
        //
        // THE ITEM IS A u16 OFFSET, not an inline struct. Unlike 0x2000, where
        // each slot holds the descriptor itself, here each slot holds an
        // absolute offset to a FlowDescriptorHeader written in the trailing
        // area (inferno proto_arc.rs query_tx_flows, arc_server.rs:311-394).
        // Each flow contributes four separate blobs, and the order they are
        // written in matters only because later ones reference earlier offsets.
        uint16_t start = (clen >= 4) ? dante_req_u16(content, 2) : 1;
        if (start == 0) start = 1;

        page_t pg;
        page_begin(&m, &pg, 2, dante_tx_flows());

        uint16_t f = start;
        for (; f <= dante_tx_flows() && pg.actual < pg.space; f++) {
            // Only report contexts that are actually BOUND. We used to list all
            // six multicast bundles unconditionally, which after the unicast
            // rework meant Dante Controller showed six flows that put nothing on
            // the wire -- and then tried to delete them.
            uint8_t  dip[4]; uint16_t dport8, ext; uint8_t ns, fp, mc;
            if (!dante_tx_flow_desc(f - 1, dip, &dport8, &ns, &fp, &mc, &ext)) continue;

            // "<flow_id>_<process_id>", the local flow name real devices use.
            char fname[24];
            {
                unsigned n2 = 0;
                if (f >= 10) fname[n2++] = (char)('0' + f / 10);
                fname[n2++] = (char)('0' + f % 10);
                fname[n2++] = '_';
                unsigned pid = g_dante.process_id, div = 10000, lead = 0;
                while (div) {
                    unsigned d = (pid / div) % 10;
                    if (d || lead || div == 1) { fname[n2++] = (char)('0' + d); lead = 1; }
                    div /= 10;
                }
                fname[n2] = 0;
            }
            uint16_t local_flow_name_off = dante_msg_str(&m, fname);

            // MULTICAST is signalled by both remote offsets being zero -- there
            // is no remote host, because nobody is told where to send. That is
            // the same test inferno makes to pick flow_type.
            const uint16_t remote_host_off    = 0;
            const uint16_t remote_rx_flow_off = 0;

            while (m.len & 3) dante_msg_u8(&m, 0);      // align 4
            uint16_t sock_off = (uint16_t)m.len;
            dante_msg_u16(&m, 0x8002);
            dante_msg_u16(&m, dport8);
            dante_msg_bytes(&m, dip, 4);

            uint16_t names_off = (uint16_t)m.len;
            dante_msg_u16(&m, 0x0a00);
            dante_msg_u16(&m, 1);
            dante_msg_u16(&m, remote_host_off);
            dante_msg_u16(&m, remote_rx_flow_off);
            dante_msg_u16(&m, 0x0010);
            dante_msg_u16(&m, local_flow_name_off);
            // For a multicast flow the first 4 bytes of this trailing field are
            // the latency in ns; the bundle record advertises 1 ms, so match it.
            dante_msg_u32(&m, g_latency_ns);
            dante_msg_u32(&m, 0);

            uint16_t descr_off = (uint16_t)m.len;
            dante_msg_u16(&m, ext);                     // the id DC knows it by
            dante_msg_u16(&m, mc ? 2 : 0x11);           // 2 = multicast, 0x11 unicast
            dante_msg_u32(&m, g_dante.sample_rate);
            dante_msg_u16(&m, 0);
            dante_msg_u16(&m, g_dante.bits_per_sample);
            dante_msg_u16(&m, 1);
            dante_msg_u16(&m, ns);                      // channels in this flow
            dante_msg_u16(&m, sock_off);
            // The REAL map, not (ctx*8 + slot + 1). Deriving it from the
            // context index reported a flow created for channels 1 and 2 as
            // channels 9 and 10, because it landed in context 1.
            for (unsigned c = 0; c < ns; c++)
                dante_msg_u16(&m, dante_tx_flow_chan(f - 1, c));
            dante_msg_u16(&m, names_off);               // footer

            put_u16_at(page_slot(&m, &pg), 0, descr_off);
            pg.actual++;
            if (m.len >= PACKET_SIZE_SOFT_LIMIT) { f++; break; }
        }
        page_end(&m, &pg);
        if (f <= dante_tx_flows()) code = DANTE_CODE_MORE;
        break;
    }

    case 0x2201: {
        // Create multicast TX flow.
        //
        // Content is a PAGE, not a bare array: space_items u8, actual_items u8,
        // then that many u16 offsets to flow descriptors. Scanning every u16 in
        // the content as an offset is what produced "refusing flow id=2560" --
        // 0x0A00 is the magic word inside MostlyZeros, i.e. we were reading the
        // descriptor bodies as if they were the index.
        //
        // Each descriptor (inferno create_multicast_tx_flow::FlowDescriptorHeader):
        //   0  2  flow_id     -- Dante Controller's HANDLE, up to the max-flows
        //                        value we advertise in 0x1000 (32). NOT an index
        //                        into our six contexts; that mapping is kept in
        //                        dante_tx_bind_multicast.
        //   2  2  flow_type   -- 2 = multicast
        //   4 10  unknown
        //  14  2  channels_count
        //  16 .. channel indices, u16 each, 1-based
        uint16_t accepted[8]; uint32_t nacc = 0;
        uint32_t nitems = (clen >= 2) ? content[1] : 0;
        for (uint32_t i = 0; i < nitems && nacc < 8; i++) {
            uint32_t io = 2 + i * 2;
            if (io + 2 > clen) break;
            uint32_t off = dante_req_u16(content, io);
            if (off < DANTE_HDR_LEN) continue;
            uint32_t d = off - DANTE_HDR_LEN;
            if (d + 16 > clen) continue;

            uint16_t fid  = dante_req_u16(content, d);
            uint16_t ftyp = dante_req_u16(content, d + 2);
            uint16_t nch  = dante_req_u16(content, d + 14);
            if (ftyp != 2 || fid == 0) {
                printf("[arc] 2201: refusing id=%u type=%u\n", fid, ftyp);
                continue;
            }
            // ANY 1..8 channels. This used to demand exactly 8 in the fixed
            // bundle order, which is why adding channels 1 and 2 was refused --
            // a restriction left over from before the gateware gained per-slot
            // channel maps. It has them now, so honour what was asked for.
            if (nch == 0 || nch > 8 || d + 16 + 2 * nch > clen) {
                printf("[arc] 2201: refusing flow %u, %u channels\n", fid, nch);
                continue;
            }
            uint16_t chans[8];
            for (unsigned c = 0; c < nch; c++)
                chans[c] = dante_req_u16(content, d + 16 + 2 * c);

            if (dante_tx_bind_multicast(fid, chans, (uint8_t)nch) < 0) {
                printf("[arc] 2201: no free context for flow %u\n", fid);
                code = 0x0315;
                continue;
            }
            accepted[nacc++] = fid;
        }
        if (nacc == 0) { if (code == DANTE_CODE_OK) code = 0x0022; break; }
        dante_msg_u16(&m, (uint16_t)nacc);
        dante_msg_u16(&m, 0);
        for (uint32_t i = 0; i < nacc; i++) dante_msg_u16(&m, accepted[i]);
        printf("[arc] 2201: created %lu multicast flow(s)\n", (unsigned long)nacc);
        break;
    }

    case 0x2202: {
        // Delete multicast TX flow. This USED to refuse, on the grounds that the
        // six bundles were permanent. That stopped being true when the unicast
        // rework made bundles bind with nslots = 0 and transmit nothing until
        // asked -- so the refusal was a stale comment refusing a request we can
        // now honour trivially, and Dante Controller simply retried forever.
        //
        // Content is a list of u16 flow ids, 1-based.
        uint32_t deleted = 0;
        for (uint32_t i = 0; i + 2 <= clen; i += 2) {
            uint16_t fid = dante_req_u16(content, i);
            if (fid == 0) continue;
            int ctx = dante_tx_ctx_for_id(fid);      // DC's id -> our context
            if (ctx >= 0 && dante_tx_unbind((unsigned)ctx) == 0) deleted++;
        }
        printf("[arc] 2202: deleted %lu flow(s)\n", (unsigned long)deleted);
        break;
    }

    case OP_QUERY_RX_FLOWS: {
        // We source no RX flows; DANTE_RX_CHANNELS exists only as an
        // observation aid, and nothing subscribes on our behalf.
        page_t pg;
        page_begin(&m, &pg, 8, 0);
        page_end(&m, &pg);
        break;
    }

    case OP_UNKNOWN_3300:
        // arc_server.rs:652 is blunt about this one: "WTF: this is necessary to
        // avoid 'clock domain mismatch' error in DC". Reproduced verbatim.
        dante_msg_u16(&m, 0x3800); dante_msg_u16(&m, 0x38fd);
        dante_msg_u16(&m, 0x38fe); dante_msg_u16(&m, 0x38ff);
        break;

    case OP_UNKNOWN_1100:
        dante_msg_bytes(&m, arc_1100_body, sizeof(arc_1100_body));
        break;

    case OP_UNKNOWN_1102:
        dante_msg_bytes(&m, arc_1102_body, sizeof(arc_1102_body));
        break;

    // 0x3010 / 0x3014 -- RX channel subscription state. These only appear once
    // a device advertises RX channels, which is why a transmit-only build never
    // saw them and the plan said not to implement them.
    //
    // DC shows "Null@" and refuses to patch when they error. The correct answer
    // is OK with NO content, which is exactly what DVS returns (measured with
    // tools/arc_query.py: 0x3010 -> OK len=0, 0x3014 -> OK len=0). An empty
    // subscription list is a valid answer; an error is not.
    case 0x3010: {
        // Subscription SET. Captured from DC patching the A16R to us:
        //
        //   0201 0001 0034 0037 00.. "01" 00 "RedNetA16R"
        //    |    |    |    |
        //    |    |    |    +-- tx hostname offset   (absolute from packet start)
        //    |    |    +------- tx channel name offset
        //    |    +------------ our RX channel id, 1-based
        //    +----------------- 0x0201, fixed in every request seen
        //
        // An empty name is an UNSUBSCRIBE, which is how DC clears a patch.
        if (clen >= 8) {
            uint16_t ch  = dante_req_u16(content, 2);
            uint16_t noff = dante_req_u16(content, 4);
            uint16_t hoff = dante_req_u16(content, 6);
            if (ch >= 1 && ch <= DANTE_RX_CHANNELS) {
                sub_copy(sub_tx_name[ch - 1], req, len, noff);
                sub_copy(sub_tx_host[ch - 1], req, len, hoff);
                printf("[arc] subscribe Rx%u <- '%s'@'%s'\n",
                       ch, sub_tx_name[ch - 1], sub_tx_host[ch - 1]);
            }
        }
        break;
    }

    case 0x3014:
        break;                      // code stays DANTE_CODE_OK, zero content

    case OP_UNKNOWN_2320:
        code = 0x30;
        break;

    // 0x2204 -- tx flow detail. MUST answer OK with an empty list, not an error.
    //
    // This is what kept Dante Controller's Sync red. The on-device capture
    // (tools/cap_fetch.py) shows DC looping at ~1 kHz on a refresh:
    //
    //   RX op=0x2200 tx flows        -> TX OK, zero flows
    //   RX op=0x2204 tx flow detail  -> TX code=0x0022   <-- error
    //   RX op=0x2200 ... and round again, thousands of times
    //
    // DC never completes the refresh, so it never populates the clock columns
    // and falls back to defaults -- which is exactly the "PTPv2 Domain 0 /
    // Priority 0/0, Primary v1 Multicast N/A" row we kept seeing.
    //
    // Replaying DC's exact request bytes at real hardware settles the format.
    // The AM2 has no flows either and still answers OK with an empty list:
    //
    //   AM2   2801000c 3344 2204 0001 0000        OK, empty
    //   A16R  28010014 3344 2204 0001 0400 ...    OK, with flow data
    //   ours  2809000a 3344 2204 0022             ERROR
    //
    // An earlier commit made this return 0x22 rather than dropping the packet,
    // on the strength of arc_query showing real devices returning 0x0022. That
    // probe sent 0x2204 with EMPTY content; DC sends it with six bytes. Same
    // opcode, different question, different answer -- and only the on-device
    // capture could show which one DC actually asks.
    case 0x2204: {
        page_t pg;
        page_begin(&m, &pg, 8, 0);
        page_end(&m, &pg);
        break;
    }

    case 0x4100:
        code = 0x30;
        break;

    // Dante Controller asks this once per Device View open. Both a RedNet A16R
    // and an AM2 answer OK with two zero bytes; we answered 0x22 unsupported.
    // Identical on both devices, so it does NOT gate the latency list -- but a
    // rejection Controller did not expect is worth not sending.
    case 0x2032:
        dante_msg_u16(&m, 0);
        break;

    // DANTE CONTROLLER'S LATENCY SET. Found by mirroring ARC requests to a
    // collector while an operator used the Latency tab -- it is not in inferno,
    // which never implemented it, so there was nothing to read it off.
    //
    // Two captures, the two values clicked, nothing else in the message
    // changing:
    //   ...0000 0000 0000 0000 004c4b40 004c4b40   5 ms  (0x4C4B40 = 5000000)
    //   ...0000 0000 0000 0000 001e8480 001e8480   2 ms  (0x1E8480 = 2000000)
    //
    // Content layout (30 bytes): seven u16 whose meaning we do not know, then
    // two zero u32, then the latency in NANOSECONDS twice. Both copies have
    // always been equal; require that rather than guessing which one leads, so
    // a message we have misread is rejected instead of silently applied.
    //
    // Answering 0x22 here is why the setting would not stick: Controller sent
    // the value, we said "unsupported", and it fell back to showing 1.0 ms.
    case 0x1101: {
        if (clen < 30) { code = 0x22; break; }
        uint32_t v1 = ((uint32_t)content[22] << 24) | ((uint32_t)content[23] << 16) |
                      ((uint32_t)content[24] << 8)  |  (uint32_t)content[25];
        uint32_t v2 = ((uint32_t)content[26] << 24) | ((uint32_t)content[27] << 16) |
                      ((uint32_t)content[28] << 8)  |  (uint32_t)content[29];
        if (v1 != v2 || v1 < 100000u || v1 > 40000000u) {
            printf("[arc] 1101 latency rejected: %lu / %lu\n",
                   (unsigned long)v1, (unsigned long)v2);
            code = 0x22;
            break;
        }
        int changed = (v1 != g_latency_ns);
        g_latency_ns = v1;
        mdns_announce();          // republish so receivers see the new value
        // Only renegotiate on a REAL change. Controller re-sends the current
        // latency whenever the Latency tab is opened, and tearing flows down
        // for that would be an unexplained dropout every time someone looked.
        if (changed) dante_tx_drop_all();
        printf("[arc] latency set to %lu ns (%lu.%02lu ms)\n",
               (unsigned long)v1, (unsigned long)(v1 / 1000000u),
               (unsigned long)((v1 / 10000u) % 100u));
        // Echo the content back. We do not know what Controller expects, but
        // echoing the request is the common pattern in this protocol and is
        // strictly more informative than an empty OK.
        dante_msg_bytes(&m, content, clen);
        break;
    }

    default:
        // Answer anyway. The previous "silence beats a wrong answer" here was
        // the wrong instinct for this protocol: every real device replies to
        // every request, and a client cannot distinguish a dropped packet from
        // a deliberate silence. 0x22 is what real hardware returns for an
        // unsupported opcode.
        g_arc_stats.unknown++;
        // LOG EACH DISTINCT OPCODE, not the first N requests.
        //
        // The cap was `unknown <= 12`, which is a budget on REQUESTS. Dante
        // Controller polls, so twelve arrive within seconds of boot and every
        // opcode after that is dropped in silence -- including any asked later
        // when an operator opens Device View. We have been unable to answer
        // "what is Controller asking us that we ignore?" for that reason alone.
        //
        // A 64-bit set keyed on the low 6 bits of the opcode is enough: the
        // opcodes seen on this bus are sparse (0x1100, 0x1102, 0x2320, 0x3300
        // in inferno; 0x2201/2/4, 0x3010/4, 0x4100 here), so collisions are
        // unlikely and the cost of one is a missed log line, not a wrong reply.
        static uint64_t seen_opcodes;
        uint64_t bit = 1ULL << (opcode & 63);
        int first_time = !(seen_opcodes & bit);
        seen_opcodes |= bit;
        if (first_time) {
            printf("[arc] unhandled opcode %#06x from %u.%u.%u.%u (answering 0x22)\n",
                   opcode, src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
            // DUMP THE BODY, not just the opcode number. Knowing that Dante
            // Controller asked something is useless for implementing it; the
            // payload is where the requested value lives -- e.g. a latency
            // selection. Capped at 48 bytes so a chatty poller cannot flood the
            // console (the UART is 1 Mbaud and the main loop shares it).
            uint32_t dn = len > 48 ? 48 : len;
            printf("[arc]   body[%lu]:", (unsigned long)len);
            for (uint32_t i = 0; i < dn; i++) printf(" %02x", req[i]);
            printf("\n");
        }
        code = 0x22;
        break;
    }

    uint32_t total = dante_msg_finish(&m, code);
    if (net_udp_commit(src_ip, src_port, DANTE_PORT_ARC, total,
                       NET_TOS_BEST_EFFORT) == 0)
        g_arc_stats.tx++;

    // Mirror AFTER the reply: both share net_udp_payload_buf(), and answering
    // Dante Controller matters more than the diagnostic.
    if (do_mirror) {
        uint8_t *mp = net_udp_payload_buf();
        uint32_t n  = 0;
        mp[n++] = 'A'; mp[n++] = 'R'; mp[n++] = 'C'; mp[n++] = '1';
        mp[n++] = (uint8_t)(code >> 8); mp[n++] = (uint8_t)code;
        mp[n++] = src_ip[0]; mp[n++] = src_ip[1];
        mp[n++] = src_ip[2]; mp[n++] = src_ip[3];
        for (uint32_t i = 0; i < mirn; i++) mp[n++] = mir[i];
        net_udp_commit(g_arc_mirror_ip, 7780, DANTE_PORT_ARC, n,
                       NET_TOS_BEST_EFFORT);
    }
}

void dante_arc_init(void)
{
    if (net_udp_bind(DANTE_PORT_ARC, arc_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", DANTE_PORT_ARC);
    printf("[arc] listening on %u, %u tx / %u rx channels\n",
           DANTE_PORT_ARC, DANTE_TX_CHANNELS, DANTE_RX_CHANNELS);
}
