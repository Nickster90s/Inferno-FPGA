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
#include "dante_dev.h"
#include "net.h"
#include <string.h>
#include <stdio.h>

dante_arc_stats_t g_arc_stats;

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
// Both tables are now DVS's, and they have to come from the SAME device.
//
// 0x1102 declares which properties EXIST; 0x1100 supplies their values. Taking
// them from different devices makes the pair self-contradictory, which is
// exactly what happened: 0x1100 was switched to DVS (declaring key 0x0064 = 0,
// no external clock) while 0x1102 was still the AM2's, which does not list
// 0x0064 at all. We were simultaneously telling DC the property does not exist
// and giving it a value.
//
//   DVS   28 keys, 0x0064 = 0x0001 (exists)    -> N/A
//   A16R  33 keys, 0x0064 = 0x0001 (exists)    -> checkbox, value 1 in 0x1100
//   AM2   31 keys, 0x0064 ABSENT               -> N/A
//
// DVS is the correct source for both: a device with no clock hardware to offer,
// which is this board. It declares the property exists and reports it as zero,
// and that pairing is what yields N/A on a model DC does not recognise. Replaying it verbatim is safe with
// respect to the offset fields: those offsets are absolute from the start of
// the packet (see dante_msg.h), and our header is the same 10 bytes, so every
// offset stays valid as long as the body is emitted unchanged.
//
// This is an EXPERIMENT, not a finished answer. It asserts another device's
// property values. If it flips DC to v1/Follower it confirms where the clock
// columns come from, and the next step is to decode the individual keys and
// report our own values rather than the AM2's.
static const uint8_t arc_1100_body[202] = {
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
    0x00, 0x00, 0x00, 0x0f, 0x42, 0x40, 0x00, 0x0f, 0x42, 0x40, 0x00, 0x0f,
    0x42, 0x40, 0x00, 0x0f, 0x42, 0x40, 0x01, 0x35, 0xf1, 0xb4, 0x00, 0x1e,
    0x84, 0x80, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t arc_1100_keyed[142] = {
    0x19, 0x19, 0x02, 0x01, 0x00, 0x01, 0x82, 0x04, 0x00, 0x70, 0x82, 0x05,
    0x00, 0x74, 0x02, 0x10, 0x00, 0x20, 0x02, 0x11, 0x00, 0x20, 0x00, 0x00,
    0x82, 0x18, 0x00, 0x00, 0x82, 0x19, 0x83, 0x01, 0x00, 0x78, 0x83, 0x02,
    0x00, 0x7c, 0x83, 0x06, 0x00, 0x80, 0x03, 0x10, 0x00, 0x3c, 0x03, 0x11,
    0x00, 0x10, 0x03, 0x03, 0x00, 0x04, 0x80, 0x21, 0x00, 0x84, 0x00, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x00, 0x22, 0x00, 0x01, 0x00, 0x63,
    0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x02, 0x22,
    0x00, 0x00, 0x02, 0x12, 0x00, 0x30, 0x83, 0x21, 0x00, 0x94, 0x00, 0x00,
    0x00, 0x66, 0x02, 0x14, 0x00, 0x80, 0x00, 0x98, 0x96, 0x80, 0x00, 0x98,
    0x96, 0x80, 0x00, 0x98, 0x96, 0x80, 0x1d, 0xcd, 0x65, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x84, 0x80,
};

// 0x1100 answered for a KEYED query.
//
// Dante Controller does not send 0x1100 empty -- it sends a list of 25 property
// keys (0201 8204 8205 0210 0211 8218 8219 8301 8302 8306 0310 0311 0303 8021
// 00f0 8060 0022 0063 0064 0065 0222 0212 8321 0066 0214), and the reply is
// (key, value) pairs answering exactly those, 146 bytes.
//
// We were returning arc_1100_body -- the AM2's answer to an EMPTY probe, a
// different shape at 202 bytes with a different 31-key set -- for every
// request. DC cannot map that onto the keys it asked for, so it falls back to
// defaults, which is how "Enable Sync To External" ends up offering an enabled
// checkbox for us while both DVS and the AM2 show N/A.
//
// The keyed reply is DVS's, captured by replaying DC's exact request at it.
//
// KEY 0x0064 IS THE EXTERNAL-CLOCK CAPABILITY, and the three devices differ in
// a way that matters more than "capable or not":
//
//   A16R (checkbox)  ... 0063 0001  0064 0001 ...   present, value 1
//   DVS  (N/A)       ... 0063 0000  0064 0000 ...   present, value 0
//   AM2  (N/A)       ... 0063 0001  0000 0064 ...   ABSENT
//
// Both DVS and the AM2 display N/A, so copying either "should" have worked --
// but it did not. We copied the AM2, which OMITS the key, and DC fell back to
// its default and offered the checkbox anyway. DVS states the key explicitly as
// zero, and gets N/A.
//
// That is the same rule that has governed every one of these fixes: DC treats
// absent data as "assume the default", and only an explicit value overrides it.
// The AM2 presumably gets away with omitting it because DC recognises its model
// and knows an Ultimo has no word-clock input; our model means nothing to DC,
// so we have to say so ourselves.
//
// DVS is the right template regardless: a device with no clock hardware to
// offer, which is exactly this board.
//
// CAVEAT: it also carries DVS's latency figures (0x00989680 = 10 ms). Those are
// not ours. Answering the query in the right shape is what this fixes; deriving
// each value is separate work, and matters before anything depends on them.
static const uint8_t arc_1102_body[114] = {
    0x00, 0x1c, 0x80, 0x20, 0x00, 0x01, 0x80, 0x21, 0x00, 0x03, 0x00, 0x22,
    0x00, 0x03, 0x00, 0x24, 0x00, 0x01, 0x00, 0x63, 0x00, 0x01, 0x00, 0x64,
    0x00, 0x01, 0x00, 0xf0, 0x00, 0x03, 0x02, 0x01, 0x00, 0x03, 0x82, 0x04,
    0x00, 0x01, 0x82, 0x05, 0x00, 0x01, 0x02, 0x0a, 0x00, 0x01, 0x02, 0x0b,
    0x00, 0x01, 0x02, 0x10, 0x00, 0x01, 0x02, 0x11, 0x00, 0x01, 0x02, 0x12,
    0x00, 0x01, 0x02, 0x13, 0x00, 0x01, 0x02, 0x14, 0x00, 0x01, 0x83, 0x01,
    0x00, 0x01, 0x83, 0x06, 0x00, 0x01, 0x83, 0x02, 0x00, 0x01, 0x03, 0x10,
    0x00, 0x01, 0x03, 0x11, 0x00, 0x01, 0x03, 0x03, 0x00, 0x03, 0x83, 0xf0,
    0x00, 0x01, 0x06, 0x01, 0x00, 0x01, 0x03, 0x09, 0x00, 0x01, 0x02, 0x09,
    0x00, 0x01, 0x07, 0x01, 0x00, 0x03,
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

    uint8_t   *buf = net_udp_payload_buf();
    dante_msg_t m;
    dante_msg_begin(&m, buf, hdr);
    uint16_t   code = DANTE_CODE_OK;

    switch (opcode) {

    case OP_CHANNELS_AND_FLOWS_COUNT: {
        // The response DC uses to size everything else.
        //
        // We used to send unknown1_0 = 0x00 and flags2 = 0x30, setting only the
        // two bits inferno names (bit4 supports_tx_channel_rename, bit5
        // supports_tx_multicast) and leaving every unnamed bit clear. Real
        // hardware does not:
        //
        //   A16R  1f f9      DVS  1f f9      AM2  1d f9      ours  00 30
        //
        // flags2 = 0xf9 on ALL THREE -- so beyond the two named bits they also
        // set the low nibble to 9 and the top two bits to 3. Whatever those
        // encode, every shipping device asserts them, and we were the only
        // device on the network declaring otherwise. Under-declaring is exactly
        // how this bring-up kept ending up with DC's defaults instead of our
        // values.
        //
        // Matching DVS (1f f9): it is the device modelled everywhere else here
        // -- clock stats, 0x1100 and 0x1102 all come from it -- and consistency
        // across those matters more than any single bit, as mixing the AM2's
        // and DVS's property tables already proved.
        dante_msg_u8 (&m, 0x1f);                           // unknown1_0
        dante_msg_u8 (&m, 0xf9);                           // flags2
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
        // 38-byte header of offsets, then the strings it points at.
        uint32_t head = m.len;
        dante_msg_zeros(&m, 38);
        uint16_t friendly = dante_msg_str(&m, g_dante.name);
        uint16_t factory  = dante_msg_str(&m, g_dante.hostname);
        uint16_t board    = dante_msg_str(&m, "InfernoFPGA");
        uint16_t revision = dante_msg_str(&m, ":705");
        // Field order per proto_arc.rs get_device_names::ResponseHeader.
        put_u16_at(m.buf, head +  6, board);
        put_u16_at(m.buf, head +  8, revision);
        put_u16_at(m.buf, head + 12, friendly);
        put_u16_at(m.buf, head + 14, factory);
        put_u16_at(m.buf, head + 16, friendly);
        put_u16_at(m.buf, head + 30, 0x2729);              // start_code
        put_u16_at(m.buf, head + 34, 0x1102);              // unknown_opcode_1102
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
        // Transmit-only device: an empty page, not an error.
        page_t pg;
        page_begin(&m, &pg, 8, 0);
        page_end(&m, &pg);
        break;
    }

    case OP_QUERY_TX_FLOWS:
    case OP_QUERY_RX_FLOWS: {
        // No flows yet. Phase 5 fills TX in with the 6 multicast bundles.
        page_t pg;
        page_begin(&m, &pg, 8, 0);
        page_end(&m, &pg);
        break;
    }

    case OP_UNKNOWN_3300:
        // The clock-domain response. arc_server.rs:652 is blunt about it:
        // "WTF: this is necessary to avoid 'clock domain mismatch' error in
        // DC", and hardcodes 38 00 38 fd 38 fe 38 ff -- which is the AM2's.
        //
        // It is NOT one constant. Queried from hardware, all three differ:
        //
        //   DVS  (N/A)      380038fb38fc38ff 00000000   12 bytes
        //   A16R (checkbox) 3800397f398039ff 7900efef   12 bytes
        //   AM2  (N/A)      380038fd38fe38ff             8 bytes
        //
        // We were sending inferno's constant, i.e. the AM2's 8-byte form.
        // Switched to DVS's, for the same reason as clock stats, the keyed
        // 0x1100, 0x1102 and the 0x1000 flags: DVS is the device this design
        // actually resembles, and these descriptions have to agree with each
        // other -- mixing sources already produced a device that contradicted
        // itself once.
        dante_msg_u16(&m, 0x3800); dante_msg_u16(&m, 0x38fb);
        dante_msg_u16(&m, 0x38fc); dante_msg_u16(&m, 0x38ff);
        dante_msg_u16(&m, 0x0000); dante_msg_u16(&m, 0x0000);
        break;

    case OP_UNKNOWN_1100: {
        // A keyed query carries a u16 count at content[0]. An empty probe
        // (count 0 or no content) still gets the full advertisement.
        uint16_t nkeys = (len >= DANTE_HDR_LEN + 2)
                       ? (uint16_t)((req[DANTE_HDR_LEN] << 8) | req[DANTE_HDR_LEN + 1])
                       : 0;
        if (nkeys)
            dante_msg_bytes(&m, arc_1100_keyed, sizeof(arc_1100_keyed));
        else
            dante_msg_bytes(&m, arc_1100_body, sizeof(arc_1100_body));
        break;
    }

    case OP_UNKNOWN_1102:
        dante_msg_bytes(&m, arc_1102_body, sizeof(arc_1102_body));
        break;

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

    default:
        // Answer anyway. The previous "silence beats a wrong answer" here was
        // the wrong instinct for this protocol: every real device replies to
        // every request, and a client cannot distinguish a dropped packet from
        // a deliberate silence. 0x22 is what real hardware returns for an
        // unsupported opcode.
        g_arc_stats.unknown++;
        if (g_arc_stats.unknown <= 12)
            printf("[arc] unhandled opcode %#06x from %u.%u.%u.%u (answering 0x22)\n",
                   opcode, src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        code = 0x22;
        break;
    }

    uint32_t total = dante_msg_finish(&m, code);
    if (net_udp_commit(src_ip, src_port, DANTE_PORT_ARC, total,
                       NET_TOS_BEST_EFFORT) == 0)
        g_arc_stats.tx++;
}

void dante_arc_init(void)
{
    net_udp_bind(DANTE_PORT_ARC, arc_rx);
    printf("[arc] listening on %u, %u tx / %u rx channels\n",
           DANTE_PORT_ARC, DANTE_TX_CHANNELS, DANTE_RX_CHANNELS);
}
