// Dante device-info / heartbeat multicast — Phase 3.
//
// WHY THIS MATTERS MORE THAN IT LOOKS: Dante Controller does not poll a device
// it has merely seen in mDNS. Real devices ANNOUNCE themselves here, and DC
// populates its device list from that. Measured on the bench: with mDNS + ARC +
// CMC all working, DC sent our device exactly ZERO packets in 20 s -- it had a
// name and nothing else, which is precisely the "shows in Device tab without
// network and device info" symptom.
//
// Framing confirmed byte for byte against a Focusrite RedNet AM2
// (captures/README.md): a 32-byte multicast header, then TLV-ish sub-records.
//
//   0  2  start_code        0xfffe for the heartbeat
//   2  2  total_length      whole datagram
//   4  2  seqnum
//   6  2  process
//   8  8  factory_device_id EUI-64 of the MAC
//  16  8  vendor            "Audinate"
//  24  8  opcode
//  32  .. content
//
// Sent FROM port 8700 TO 224.0.0.233:8708, once a second, exactly as the AM2
// does.

#include "dante_info.h"
#include "dante_dev.h"
#include "gptp.h"
#include "ptpv1.h"
#include "net.h"
#include <string.h>
#include <stdio.h>

#define MCAST_HDR_LEN     32
#define HEARTBEAT_MS      1000

static const uint8_t grp_heartbeat[4] = {224, 0, 0, 233};
static const uint8_t grp_devinfo[4]   = {224, 0, 0, 231};

// The vendor field really is the literal ASCII "Audinate" on the wire, even
// from a Focusrite device -- it identifies the protocol, not the manufacturer.
static const char vendor_str[8] = {'A','u','d','i','n','a','t','e'};

dante_info_stats_t g_info_stats;

static uint16_t seqnum;
static uint32_t next_heartbeat_ms;

// Device info is announced CONTINUOUSLY, not as a burst at boot.
//
// Two separate findings forced this shape:
//
// 1. Init-time transmissions are lost. main() does a PHY soft reset
//    (mdio 0x00 <- 0x9140) at startup, which the loader measured as ~3.85 s of
//    link-down while gigabit auto-negotiation completes. Anything sent from
//    *_init() goes into a dead link. The tell: the 1 Hz heartbeat, sent later
//    from the main loop, always reached the wire while the three init-time
//    announcements never did.
//
// 2. A burst is not enough. MEASURED on the bench: both RedNets transmit to
//    224.0.0.231 continuously, ~22 and ~20 packets per 45 s, i.e. roughly every
//    2 s, forever. Dante Controller builds its device list from these
//    announcements rather than by polling -- during a full Refresh it sent no
//    unicast to ANY device, only mDNS queries. So a device that stops
//    announcing simply ceases to exist the next time DC rebuilds its list.
//    That is exactly what happened: the device appeared with full Routing and
//    Device Info, then vanished on Refresh.
#define INFO_ANNOUNCE_MS      3000
static uint32_t info_announce_next_ms;
static const gptp_t *s_gptp;      // for the clock sub-record; may be NULL

void dante_info_set_gptp(const gptp_t *g) { s_gptp = g; }

// ---------------------------------------------------------------------------

static uint32_t put_hdr(uint8_t *p, uint16_t start_code, const uint8_t opcode[8])
{
    uint32_t n = 0;
    p[n++] = (uint8_t)(start_code >> 8); p[n++] = (uint8_t)start_code;
    p[n++] = 0; p[n++] = 0;                                   // total_length
    p[n++] = (uint8_t)(seqnum >> 8);     p[n++] = (uint8_t)seqnum;
    p[n++] = (uint8_t)(g_dante.process_id >> 8);
    p[n++] = (uint8_t)g_dante.process_id;
    memcpy(p + n, g_dante.device_id, 8); n += 8;
    memcpy(p + n, vendor_str, 8);        n += 8;
    memcpy(p + n, opcode, 8);            n += 8;
    return n;                                                 // == 32
}

static inline void put_u16(uint8_t *p, uint32_t at, uint16_t v)
{
    p[at] = (uint8_t)(v >> 8); p[at + 1] = (uint8_t)v;
}
static inline void put_u32(uint8_t *p, uint32_t at, uint32_t v)
{
    p[at] = (uint8_t)(v >> 24); p[at+1] = (uint8_t)(v >> 16);
    p[at+2] = (uint8_t)(v >> 8); p[at+3] = (uint8_t)v;
}

// ---------------------------------------------------------------------------
// Heartbeat
//
// Content is a sequence of sub-records, each:
//
//    0  2  record length (including this 12-byte header)
//    2  2  type
//    4  2  0x0004 (constant on every record from every device seen)
//    6  2  content length
//    8  2  seqnum / uptime
//   10  2  0
//   12  .. content
//
// We send 0x8001 (frequency offset) and 0x8000 (clock sync quality). Real
// devices also emit 0x8002 (per-channel signal peaks), 0x8003 (sample rate +
// per-flow words) and 0x8004; those are reporting rather than clock state.
//
// 0x8002/0x8003/0x8004 are deliberately NOT sent. Their leading u16 is a count
// that scales the record (2 on the A16R, 32 on the other device on the bench),
// and nothing observed so far pins down what it counts -- tx channels, rx
// channels and flows are all consistent with the two samples we have. Emitting
// a guessed count is worse than emitting nothing: a receiver that trusts it
// would mis-parse every following record in the same datagram, since these are
// length-delimited and parsed in sequence.
// ---------------------------------------------------------------------------

static void send_heartbeat(void)
{
    static const uint8_t op[8] = {0x00, 0x08, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00};

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr(p, 0xFFFE, op);

    // 0x8001: frequency offset in ppb. Derived from how far the PTP servo has
    // pulled the TSU addend away from nominal -- the same number DC plots in
    // its clock histogram.
    // Report the PTPv1 servo's pull, not gPTP's -- PTPv1 owns the clock now,
    // and reporting a servo that no longer steers anything would put a
    // permanently flat line in Dante Controller's clock histogram.
    int32_t ppb = 0;
    if (g_ptpv1.base_addend_full) {
        int64_t base = (int64_t)g_ptpv1.base_addend_full;
        int64_t cur  = (int64_t)g_ptpv1.current_addend_full;
        ppb = (int32_t)(((cur - base) * 1000000000LL) / base);
    }

    put_u16(p, n, 16);      n += 2;      // length of this sub-record
    put_u16(p, n, 0x8001);  n += 2;      // type
    put_u16(p, n, 4);       n += 2;
    put_u16(p, n, 4);       n += 2;      // content length
    put_u16(p, n, seqnum);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u32(p, n, (uint32_t)ppb); n += 4;

    // 0x8000: clock sync quality -- one 16-byte item holding the two live PTP
    // measurements. This is what Dante Controller's Sync indicator reads; with
    // the record absent it has nothing to judge us on and shows red, which is
    // exactly what we were seeing while the servo itself was healthy.
    //
    // The four leading u16s are byte-identical on both Dante devices on the
    // bench (count=1, item size=0x10), so they are structure, not device state.
    // The two words that follow jitter per-second on real hardware -- 218..998
    // and 1232..3072 on the A16R -- which is offset-from-master and mean path
    // delay in nanoseconds, not counters.
    //
    // Reported honestly: while the servo is far out these are large, and DC
    // should show red. Green has to be earned by the servo, not by the report.
    int64_t off = g_ptpv1.offset_ns;
    if (off < 0) off = -off;
    if (off > 0xFFFFFFFFLL) off = 0xFFFFFFFFLL;
    int64_t pd = g_ptpv1.mean_path_delay_ns;
    if (pd < 0) pd = 0;
    if (pd > 0xFFFFFFFFLL) pd = 0xFFFFFFFFLL;

    put_u16(p, n, 36);      n += 2;      // length of this sub-record
    put_u16(p, n, 0x8000);  n += 2;      // type
    put_u16(p, n, 4);       n += 2;
    put_u16(p, n, 4);       n += 2;      // content length (4 even though 24
                                         // bytes follow -- both real devices
                                         // send exactly this)
    put_u16(p, n, seqnum);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, 0x0010);  n += 2;      // item size
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, 1);       n += 2;      // item count
    put_u16(p, n, 0x0010);  n += 2;
    put_u32(p, n, (uint32_t)off); n += 4;
    put_u32(p, n, (uint32_t)pd);  n += 4;
    put_u32(p, n, 0);       n += 4;
    put_u32(p, n, 0);       n += 4;

    // 0x8003 / 0x8004: emitted LAST, deliberately.
    //
    // Both begin with a count that scales the record. That count was why these
    // were left out earlier -- guessing it wrong makes a receiver mis-parse
    // every record AFTER it, because records are length-delimited and read in
    // sequence. Putting them at the end bounds that risk to themselves.
    //
    // The count now has evidence behind it rather than being a guess: the A16R
    // is a 16-channel device and sends 2, and 16/8 = 2 flows (Dante's
    // MAX_CHANNELS_IN_FLOW is 8). The other device on the bench sends 32, which
    // is consistent with the same rule for a larger box. Our 48 channels give 6.
    //
    // 0x8003 carries the sample rate (the A16R's 0x0000bb80 = 48000) followed
    // by one word per flow; 0x8004 is one word per flow with no preamble. Both
    // are all-zero past those on real hardware.
    const uint16_t nflows = DANTE_TX_CHANNELS / 8;

    put_u16(p, n, (uint16_t)(12 + 8 + 4 + 4 * nflows)); n += 2;
    put_u16(p, n, 0x8003);  n += 2;
    put_u16(p, n, 4);       n += 2;
    put_u16(p, n, (uint16_t)(8 + 4 + 4 * nflows)); n += 2;
    put_u16(p, n, seqnum);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, nflows);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, 0x0018);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u32(p, n, 48000);   n += 4;                  // sample rate
    for (uint16_t i = 0; i < nflows; i++) { put_u32(p, n, 0); n += 4; }

    put_u16(p, n, (uint16_t)(12 + 8 + 4 * nflows)); n += 2;
    put_u16(p, n, 0x8004);  n += 2;
    put_u16(p, n, 4);       n += 2;
    put_u16(p, n, (uint16_t)(8 + 4 * nflows)); n += 2;
    put_u16(p, n, seqnum);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, nflows);  n += 2;
    put_u16(p, n, 0);       n += 2;
    put_u16(p, n, 0x0014);  n += 2;
    put_u16(p, n, 0);       n += 2;
    for (uint16_t i = 0; i < nflows; i++) { put_u32(p, n, 0); n += 4; }

    put_u16(p, 2, (uint16_t)n);          // total_length

    if (net_udp_commit(grp_heartbeat, DANTE_PORT_HEARTBEAT,
                       DANTE_PORT_INFO_REQ, n, NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_heartbeat++;
    seqnum++;
}

// ---------------------------------------------------------------------------
// Device info
//
// Byte layout from inferno's send_board_info (info_mcast_server.rs:103-145),
// which annotates the capability flags. Sent to 224.0.0.231:8702 at boot and on
// request.
//
// content[0xbb] = 0x1f matters: inferno notes that leaving it 0 makes the device
// "flooded with info multicast requests around 1 per second".
// ---------------------------------------------------------------------------

static void send_device_info(const uint8_t *dst_ip, uint16_t dst_port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00};

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr(p, 0xFFFF, op);
    uint8_t *c = p + n;
    memset(c, 0, 200);

    c[0] = 4; c[1] = 1; c[2] = 0; c[3] = 6;      // firmware version
    c[0x23] = 2;
    c[4] = 4; c[5] = 1; c[6] = 0; c[7] = 3;      // hardware version
    c[0x27] = 1;
    c[0x28] = 1;                                  // boot version 1.0.0.0

    // Capability flags, per inferno's annotation:
    //   [0x14] 0x04 supports AES67, 0x08 lockable  -> neither
    //   [0x16] 0x10 has manufacturer name, 0x40 network configurable
    //   [0x17] identify / rate config / reboot / factory reset
    c[0x14] = 0;
    c[0x15] = 0;
    c[0x16] = 0x10;
    c[0x17] = 0;
    c[0xbb] = 0x1f;

    // Board name at 12 (8 bytes) and again at 0x38 (16 bytes).
    memcpy(c + 12,    "Inferno", 7);
    memcpy(c + 0x38,  "InfernoFPGA", 11);

    n += 200;
    put_u16(p, 2, (uint16_t)n);

    if (net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_info++;
    seqnum++;
}

// ---------------------------------------------------------------------------
// Product info -- Model Name and Product Version in Dante Controller.
//
// Layout from inferno send_product_info (info_mcast_server.rs:139-156).
// Strings are fixed-width fields, NOT NUL-terminated-and-packed:
//   0x00  8   manufacturer
//   0x08  8   board name
//   0x1c  4   firmware version
//   0x2c  16  manufacturer (again, longer field)
//   0xac  16  model name
// ---------------------------------------------------------------------------

static void put_fixed(uint8_t *c, uint32_t at, uint32_t width, const char *s)
{
    uint32_t i = 0;
    for (; s[i] && i < width; i++) c[at + i] = (uint8_t)s[i];
    for (; i < width; i++)         c[at + i] = 0;
}

static void send_product_info(const uint8_t *dst_ip, uint16_t dst_port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00};

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr(p, 0xFFFF, op);
    uint8_t *c = p + n;
    memset(c, 0, 336);

    put_fixed(c, 0x00, 8,  "Inferno");
    put_fixed(c, 0x08, 8,  "InfrnFPG");
    c[0x1c] = 0; c[0x1d] = 0; c[0x1e] = 0; c[0x1f] = 1;      // firmware 0.0.0.1
    put_fixed(c, 0x2c, 16, "Inferno");
    put_fixed(c, 0xac, 16, "InfernoFPGA 48ch");

    n += 336;
    put_u16(p, 2, (uint16_t)n);

    if (net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_info++;
    seqnum++;
}

// ---------------------------------------------------------------------------
// Network info -- Primary Address and Link Speed.
//
// THIS IS THE ONE THAT GATES ROUTING. Dante Controller showed our device with a
// blank Primary Address, and without an address it cannot send us ARC requests,
// so the Routing tab stayed empty no matter how correct the ARC server was.
//
// Layout from inferno send_network_info (info_mcast_server.rs:352-376).
// Note the reply opcode is 0x0011 while the REQUEST is 0x0013 -- they are not
// the same value, which is easy to miss.
// ---------------------------------------------------------------------------

static void send_network_info(const uint8_t *dst_ip, uint16_t dst_port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00};

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr(p, 0xFFFF, op);
    uint8_t *c = p + n;
    uint32_t o = 0;

    static const uint8_t lead[6] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    memcpy(c + o, lead, 6); o += 6;

    put_u16(c, o, 1000); o += 2;              // link speed, Mbps -> "1Gbps"
    put_u16(c, o, 1);    o += 2;

    memcpy(c + o, g_dante.mac, 6); o += 6;
    memcpy(c + o, g_net_ip,    4); o += 4;

    // Netmask from the prefix, then gateway (none) and DNS (same).
    uint8_t mask[4] = {255, 255, 0, 0};
    if (g_net_prefix == 24) { mask[2] = 255; }
    memcpy(c + o, mask, 4); o += 4;
    memset(c + o, 0, 4);    o += 4;           // gateway
    memset(c + o, 0, 4);    o += 4;           // DNS

    static const uint8_t tail[32] = {
        0x00, 0x18, 0x00, 0x30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    memcpy(c + o, tail, 32); o += 32;

    n += o;
    put_u16(p, 2, (uint16_t)n);

    if (net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_info++;
    seqnum++;
}

// ---------------------------------------------------------------------------
// Requests arriving on 8700
// ---------------------------------------------------------------------------

static void info_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                    uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip;
    if (len < MCAST_HDR_LEN) return;
    g_info_stats.rx++;

    // opcode[8] sits at offset 24; byte 3 selects the query. Dispatch matches
    // inferno's run_server (info_mcast_server.rs:405-425).
    uint8_t q = req[24 + 3];
    switch (q) {
    case 0x60: case 0x61: send_device_info (src_ip, src_port); break;
    case 0xc0: case 0xc1: send_product_info(src_ip, src_port); break;
    case 0x13:            send_network_info(src_ip, src_port); break;
    default:              g_info_stats.rx_unknown++;           break;
    }
}

// ---------------------------------------------------------------------------

void dante_info_poll(void)
{
    uint32_t now = gptp_uptime_ms();

    // Periodic device-info announcement, forever. Stopping makes us disappear
    // from Dante Controller on its next refresh -- see INFO_ANNOUNCE_MS.
    if (!info_announce_next_ms || (int32_t)(now - info_announce_next_ms) >= 0) {
        send_device_info (grp_devinfo, DANTE_PORT_INFO);
        send_product_info(grp_devinfo, DANTE_PORT_INFO);
        send_network_info(grp_devinfo, DANTE_PORT_INFO);
        info_announce_next_ms = now + INFO_ANNOUNCE_MS;
        return;                          // don't also heartbeat this pass
    }

    if (next_heartbeat_ms && (int32_t)(now - next_heartbeat_ms) < 0) return;
    next_heartbeat_ms = now + HEARTBEAT_MS;
    send_heartbeat();
}

void dante_info_init(void)
{
    net_udp_bind(DANTE_PORT_INFO_REQ, info_rx);
    net_igmp_join(grp_heartbeat);
    net_igmp_join(grp_devinfo);
    next_heartbeat_ms = 0;                       // fire on the next poll

    // Announcements are driven from dante_info_poll(), not sent here -- the link
    // is still down at this point. See the note on INFO_ANNOUNCE_MS.
    info_announce_next_ms = 0;
    printf("[info] heartbeat -> 224.0.0.233:%u, info -> 224.0.0.231:%u\n",
           DANTE_PORT_HEARTBEAT, DANTE_PORT_INFO);
}
