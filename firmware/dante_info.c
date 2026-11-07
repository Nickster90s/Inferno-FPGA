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
#include "config.h"   // g_fs, cfg_save(): de rate wordt hier gezet
#include "cfgflash.h" // de SPI-interlock onder handle_sys_reset()
#include <generated/csr.h>   // ts_subsec_max / usb_rate_48k bij een rate-wijziging
#include <string.h>
#include <stdio.h>

/* printf in this file is a NARRATIVE, not an interface. The format strings
 * were 77% of .rodata; see log.h for the measurement and the rebuild flag. */
#include "log.h"
#define printf(...) LOGF(__VA_ARGS__)


static void put_fixed(uint8_t *c, uint32_t at, uint32_t width, const char *s);

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

// Variant that echoes a specific seqnum instead of using our own counter, for
// replies that a controller has to correlate with its request.
static uint32_t put_hdr_seq(uint8_t *p, uint16_t start_code,
                            const uint8_t opcode[8], uint16_t seq)
{
    uint32_t n = 0;
    p[n++] = (uint8_t)(start_code >> 8); p[n++] = (uint8_t)start_code;
    p[n++] = 0; p[n++] = 0;                                   // total_length
    p[n++] = (uint8_t)(seq >> 8);        p[n++] = (uint8_t)seq;
    p[n++] = (uint8_t)(g_dante.process_id >> 8);
    p[n++] = (uint8_t)g_dante.process_id;
    memcpy(p + n, g_dante.device_id, 8); n += 8;
    memcpy(p + n, vendor_str, 8);        n += 8;
    memcpy(p + n, opcode, 8);            n += 8;
    return n;                                                 // == 32
}

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

    // 0x8002: per-channel signal peaks. EVERY real device on the bench sends
    // this and we were the only one that did not -- compared structurally with
    // a heartbeat capture on 2026-08-05:
    //
    //   MacBook  8000 8001 8002 8003 8004 8006 8008 8009 800a
    //   A16R     8000 8001 8002 8003 8004
    //   AM2      8000 8001 8002 8003 8004
    //   us       8000 8001 ---- 8003 8004
    //
    // It was skipped because the leading count "could not be pinned down".
    // inferno documents it exactly (info_mcast_server.rs:201-225): u16 tx
    // count, 0, u16 rx count, 0, 24, 0, then ONE BYTE per channel, tx first
    // then rx, zero-padded to a multiple of four. The observed lengths agree:
    // the A16R's 60 = 24 + 36 (16+16 channels padded), the AM2's 28 = 24 + 4.
    //
    // PEAKS ARE REAL NOW, and the ZEROS WE USED TO SEND WERE NOT "no meter".
    //
    // The old comment here said firmware cannot measure this -- true, audio goes
    // USB -> gateware ring -> packetizer and never passes the CPU -- and then
    // called the zeros a truthful absence. THAT WAS WRONG ABOUT THE ENCODING.
    // The receiver project's device_server/peaks.rs sends
    //
    //     peak_lin = peak / full_scale
    //     byte     = round(-40 * log10(peak_lin))   clamped 0..255
    //
    // so the byte counts HALF-DECIBELS BELOW FULL SCALE: 0 is 0 dBFS, 12 is
    // -6 dBFS, 96 is -48 dBFS and silence is ~253..255. Sending zeros claimed
    // all 48 channels were pinned at full scale, and Dante Controller answered
    // with a red speaker and an exclamation mark in the Signal column of its
    // Transmit tab -- reported from the bench 2026-11-06.
    //
    // CONFIRMED AGAINST A REAL DEVICE, not just against the receiver project.
    // captures/dante_control.pcap holds the heartbeat of 169.254.61.114, a
    // receive-only device, and its 0x8002 decodes field for field as this code
    // writes it:
    //
    //     len 28  type 0x8002  4  content_len 16  ctr 780  0
    //     tx_count 0  0  rx_count 2  0  data_off 24  0
    //     peaks [253, 253, 0, 0]
    //
    // Two silent RX channels reporting 253 -- about -126 dBFS, its own noise
    // floor -- and the two pad bytes zero. A quiet channel reports a HIGH
    // number on real hardware. That settles the direction without having to
    // trust a single source for it.
    //
    // dante_packetizer.py now meters at the ring write point, where every sample
    // arrives with its channel number already decoded, and keeps max|sample| per
    // channel in a 48 x 8-bit array. We read all of them and then clear, so the
    // window is exactly one heartbeat.
    //
    // THE TABLE IS EXACT AND COSTS NO ARITHMETIC. 256 entries of -40*log10(m/256)
    // for the 8-bit magnitude the gateware keeps; no log, no float, no divide on
    // a CPU that has none. Its floor is a cliff and that is worth knowing: m=1 is
    // -48 dBFS (96) and m=0 jumps straight to 255, so anything quieter than
    // -48 dBFS reads as silence. Enough for "is there signal", coarse for a meter
    // bottom; widening the gateware magnitude to 16 bits is the fix if that ever
    // matters, and then this table needs a leading-zero count in front of it.
    static const uint8_t PEAK_DB[256] = {
    255,  96,  84,  77,  72,  68,  65,  63,  60,  58,  56,  55,  53,  52,  50,  49,
     48,  47,  46,  45,  44,  43,  43,  42,  41,  40,  40,  39,  38,  38,  37,  37,
     36,  36,  35,  35,  34,  34,  33,  33,  32,  32,  31,  31,  31,  30,  30,  29,
     29,  29,  28,  28,  28,  27,  27,  27,  26,  26,  26,  25,  25,  25,  25,  24,
     24,  24,  24,  23,  23,  23,  23,  22,  22,  22,  22,  21,  21,  21,  21,  20,
     20,  20,  20,  20,  19,  19,  19,  19,  19,  18,  18,  18,  18,  18,  17,  17,
     17,  17,  17,  17,  16,  16,  16,  16,  16,  15,  15,  15,  15,  15,  15,  15,
     14,  14,  14,  14,  14,  14,  13,  13,  13,  13,  13,  13,  13,  12,  12,  12,
     12,  12,  12,  12,  12,  11,  11,  11,  11,  11,  11,  11,  10,  10,  10,  10,
     10,  10,  10,  10,  10,   9,   9,   9,   9,   9,   9,   9,   9,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,
    };
    {
        const uint16_t ntx = DANTE_TX_CHANNELS, nrx = DANTE_RX_CHANNELS;
        uint16_t npk = ntx + nrx;
        uint16_t pad = (uint16_t)((4u - (npk & 3u)) & 3u);
        put_u16(p, n, (uint16_t)(24 + npk + pad)); n += 2;
        put_u16(p, n, 0x8002);  n += 2;
        put_u16(p, n, 4);       n += 2;
        put_u16(p, n, (uint16_t)(12 + npk)); n += 2;
        put_u16(p, n, seqnum);  n += 2;
        put_u16(p, n, 0);       n += 2;
        put_u16(p, n, ntx);     n += 2;
        put_u16(p, n, 0);       n += 2;
        put_u16(p, n, nrx);     n += 2;
        put_u16(p, n, 0);       n += 2;
        put_u16(p, n, 24);      n += 2;
        put_u16(p, n, 0);       n += 2;
#ifdef CSR_AAF_PKT_PEAK_VALUE_ADDR
        for (uint16_t i = 0; i < ntx; i++) {
            aaf_pkt_peak_select_write(i);
            p[n] = PEAK_DB[aaf_pkt_peak_value_read() & 0xFFu];
            n += 1;
        }
        // Our RX channels exist so Controller can subscribe to us; no audio is
        // ever received, so 255 (silence) is the honest value, not 0 (0 dBFS).
        for (uint16_t i = 0; i < nrx; i++) { p[n] = 255; n += 1; }
        aaf_pkt_peak_clear_write(1);     // window = one heartbeat
#else
        // No detector in this bitstream: say SILENCE, not full scale.
        for (uint16_t i = 0; i < npk; i++) { p[n] = 255; n += 1; }
#endif
        for (uint16_t i = 0; i < pad; i++) { p[n] = 0; n += 1; }
    }

    // 0x8003 / 0x8004: emitted LAST, deliberately.
    //
    // Both begin with a count that scales the record. That count was why these
    // were left out earlier -- guessing it wrong makes a receiver mis-parse
    // every record AFTER it, because records are length-delimited and read in
    // sequence. Putting them at the end bounds that risk to themselves.
    //
    // COUNT IS THE **RX** FLOW CAPACITY, NOT A FUNCTION OF TX CHANNELS.
    //
    // This used to be DANTE_TX_CHANNELS / 8 = 6, justified as "the A16R is a
    // 16-channel device and sends 2, and 16/8 = 2". That rule does not hold:
    // measured directly off the wire with tools/dante_latency.py on 2026-08-05,
    // the A16R sends 32 and the AM2 sends 2. 16/8 is 2 only by coincidence.
    //
    // What the block actually is (inferno info_mcast_server.rs:227-250): the
    // per-flow `actual_latency_samples` of the flows this device RECEIVES,
    // taken from its channels_subscriber. A pure transmitter has none. We were
    // advertising six phantom RX flows, each reporting 0 samples of latency --
    // a value no real flow can have, since it is measured as
    // (now - packet_timestamp) at receipt.
    //
    // We have DANTE_RX_CHANNELS = 2. The AM2, also a two-channel device,
    // reports 2. That is the only direct evidence available, so follow it.
    //
    // NOTE this is unlikely to change Dante Controller's grey "Latency Status"
    // for us: with nothing subscribed to our RX channels there are no received
    // flows to measure, and grey is the correct rendering of that. Fixed
    // because advertising flows we do not have is wrong on its own terms.
    //
    // 0x8003 carries the sample rate (the A16R's 0x0000bb80 = 48000) followed
    // by one word per flow; 0x8004 is one word per flow with no preamble. Both
    // are all-zero past those on real hardware.
    const uint16_t nflows = DANTE_RX_CHANNELS;

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
    put_u32(p, n, g_fs);    n += 4;                  // sample rate
    // Zero because we RECEIVE nothing. This record is the per-flow measured
    // RX latency and a transmit-only device has none.
    //
    // TESTED 2026-08-05: reporting a plausible non-zero value here (14 samples,
    // matching what the AM2 reports) did NOT change Dante Controller's grey
    // Latency Status for us. So the colour is NOT driven by this record, and
    // the long-held assumption that it was is wrong. Controller appears to show
    // latency only for flows IT knows about -- i.e. subscriptions -- and
    // ignores a latency claim for a flow that does not exist in its model.
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
    //
    // 0x17 = 0xdb is de waarde van een RedNet AM2, van de bench gelezen (zie
    // Inferno-ESP32-P4 aoip_info.c, waar 0 de controller liet zeggen "This
    // device does not support sample rate configuration"). WELKE BIT WAT DOET
    // IS NIET GEISOLEERD, dus dit adverteert mogelijk ook identify / reboot /
    // factory reset. Dat is bewust: een echt device nabootsen is hier de
    // projectregel, tegenover een bit verzinnen die plausibel lijkt. Stuurt DC
    // zo'n commando, dan komt het in de unhandled-query log van info_rx() --
    // daar zie je het, in plaats van dat het stil verdwijnt.
    // BIT 14 OF THIS u32 IS WHAT UNGREYS DANTE CONTROLLER'S NETWORK CONFIG.
    // Decompiled from DanteController.jar: StaticAddressesPane prints "This
    // device does not support static addressing" unless IVersions.hasStaticIp(),
    // which is `inferredCapabilities & 0x4000` of the capabilities u32 at
    // content[0x14] of THIS message -- so 0x40 in c[0x16]. The PRODIGY1-32
    // reference sends 0x0e7cd65b (bit set); we sent 0x000010db (clear). Every
    // earlier attempt -- four extra 0x1100 keys, a unicast mDNS reply,
    // arcp_vers, the dbc and chan services, a second interface in 0x11 -- was
    // looking in the wrong message. The same bit in the READ-ONLY capabilities
    // means hasReadOnlyStaticIp(), which draws the form but keeps it locked, so
    // it belongs here and nowhere else.
    c[0x14] = 0;
    c[0x15] = 0;
    c[0x16] = 0x10 | 0x40;
    c[0x17] = 0xdb;
    c[0xbb] = 0x1f;

    // Board name at 12 (8 bytes) and again at 0x38 (16 bytes).
    //
    // THIS IS "Dante Model" IN CONTROLLER -- a DIFFERENT field from the "Model
    // Name" that send_product_info writes at 0xac. An older comment here said
    // the two were "kept in step so DC never sees two different models for one
    // device"; that was wrong, and it hid the distinction because both records
    // happened to carry the same string. Setting them to one value made
    // Controller show Model Name and Dante Model identically, which is what
    // exposed it.
    //
    // Lengths are the FIELD widths (8 and 16), and put_fixed zero-pads without
    // reserving a terminator -- a string of exactly `width` characters runs
    // into the next field, so the 8-byte code stays at 7.
    put_fixed(c, 12,   8,  "NSerAoI");
    put_fixed(c, 0x38, 16, "N-Series AoIP");

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

    // 8-BYTE FIELDS, and put_fixed() does NOT reserve a terminator: a string of
    // exactly `width` chars fills the field with no NUL and runs into the next
    // one. "N-Series" is exactly 8, so the short codes stay <= 7 chars and the
    // full names go in the 16-byte fields below.
    put_fixed(c, 0x00, 8,  "NSeries");
    put_fixed(c, 0x08, 8,  "USBSwtc");   // 8-byte field, short model code
    // Product Version, the column Dante Controller shows next to Model Name.
    // 4 bytes. The render is INFERRED, not confirmed: DVS shows 4.5.1.1 (four
    // components) and the AM2 shows 0.0.1 (three), which fits DC dropping a
    // leading zero byte -- so 00 00 01 00 should read "0.1.0". Neither device
    // answers a synthetic 0x00c1 query, so this could not be checked against
    // real hardware the way the rest of the layout was.
    //
    // It previously held 00 00 00 01 and DC rendered the column BLANK, so
    // something beyond the raw bytes gates it; if this still shows blank, that
    // is the thing to chase rather than the value.
    c[0x1c] = 0; c[0x1d] = 0; c[0x1e] = 1; c[0x1f] = 0;      // product 0.1.0
    put_fixed(c, 0x2c, 16, "N-Series");        // Manufacturer, as DC shows it
    // Model Name as Dante Controller displays it. 16-byte FIXED field, so the
    // string must be <= 16 characters -- "N-Series USB 48" is 15 and fits;
    // "N-Series USB 48CH" would be 17 and would silently truncate.
    put_fixed(c, 0xac, 16, "USB Switchover");  // 14 chars, fits the 16-byte field

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

// ---------------------------------------------------------------------------
// Clock stats (request opcode byte 0x21 -> reply 0x0020)
//
// THE MISSING PIECE behind Dante Controller showing us as a PTPv2 device.
//
// DC's Clock Status tab showed InfernoFPGA with "PTPv2 Domain 0 / Priority 0/0"
// and N/A under Primary v1 Multicast, while the two RedNets showed v1
// Leader/Follower with the PTPv2 columns N/A. Exactly inverted. DC was looking
// for us in the v2 clock domain, where we do not participate -- which is why
// getting the PTPv1 offset down to 290 ns (parity with the A16R's 218 ns)
// changed nothing on screen, and why replaying a real device's ARC property
// tables byte for byte changed nothing either.
//
// This reply is what tells DC we are a v1 FOLLOWER and *whose* follower: it
// carries the master clock id. With no reply DC has no v1 clock state for us at
// all and falls back to defaults -- which read as PTPv2, domain 0, priority 0/0.
//
// It is REQUEST-DRIVEN, never unsolicited: 45 s of multicast capture shows the
// RedNets sending heartbeats and nothing else on 8702. DC's request is unicast
// to us on 8700, which is precisely the traffic this host cannot sniff (the
// switch forwards unicast only to the destination port), so the request never
// appeared in any capture -- only in the board's own console.
//
// Layout from inferno info_mcast_server.rs:327-341.
// ---------------------------------------------------------------------------

// Clock-stats payload, modelled byte for byte on the Dante Virtual Soundcard --
// the device Dante Controller labels "Follower Only", which is what we actually
// are: we never transmit Sync and never join BMCA, so we CANNOT be elected
// leader. Modelling on the AM2 got us "Follower", a device that could be
// elected and merely is not.
//
// DVS is also the closest analogue to this design: a device with no local clock
// hardware to offer. Its payload is 148 bytes against the AM2's 188 and the
// A16R's 208, and carries 0x000000ff at [4:8] where the AM2 has 0x0000009b --
// inferno's source has that field as 0x9f with the comment /* was 0xff */, so
// their original capture was of a DVS too.
//
// Captured from real hardware once clock stats became multicast: their replies
// go to 224.0.0.231 too, so making our own reply multicast is what finally made
// theirs visible to the build host. Before that they were unicast to the
// controller and unreachable from here.
//
// The two fields that matter, both wrong in the previous hand-built payload:
//
//  1. CLOCK IDs ARE MAC + 0x0000, NOT EUI-64. Real devices send
//     001dc12d4a180000 at offsets 12/20/28; we were inserting FF FE to make
//     001dc1fffe2d4a18. DC could not match our reported master against the
//     actual leader. (The EUI-64 form IS used, but further in, at 152/160/168 --
//     the payload carries both.)
//
//  2. OFFSET 40 IS THE PTP PORT STATE. The Follower sends 0x0009, the Leader
//     0x0006 -- IEEE 1588 SLAVE and MASTER. That is the field behind DC's
//     "Primary v1 Multicast" column, and we were sending zeros there.
//
// Everything else is carried over from the AM2 verbatim. Fields whose meaning
// is not established are left exactly as a working Follower sends them rather
// than guessed at.
static const uint8_t clock_stats_tmpl[148] = {
    0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xf1, 0x6a,
    0xc8, 0xa3, 0x62, 0xeb, 0xf8, 0xc8, 0x00, 0x00, 0x00, 0x1d, 0xc1, 0x2d,
    0x4a, 0x18, 0x00, 0x00, 0x00, 0x1d, 0xc1, 0x2d, 0x4a, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x34, 0x00, 0x09, 0x00, 0x00, 0x02, 0x34, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0c,
    0x00, 0x78, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x68, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x09, 0x00, 0x07, 0x00, 0x01, 0x00, 0x07, 0x00, 0x98, 0x00, 0x04,
    0xc8, 0xa3, 0x62, 0xff, 0xfe, 0xeb, 0xf8, 0xc8, 0x00, 0x1d, 0xc1, 0xff,
    0xfe, 0x2d, 0x4a, 0x18, 0x00, 0x1d, 0xc1, 0xff, 0xfe, 0x2d, 0x4a, 0x18,
    0x00, 0x01, 0x00, 0x00,
};

static void send_clock_stats(const uint8_t *dst_ip, uint16_t dst_port,
                             const uint8_t *req)
{
    if (!g_ptpv1.have_master) return;        // nothing meaningful to report yet

    // ECHO the request's seqnum and opcode instead of using our own.
    //
    // Dante Controller asks with seq=0x03de and opcode 073e 0021 0000 0064; we
    // were answering with our own counter and a hardcoded 072a 0020 0000 0000
    // (inferno's constant). The exchange completed and the content was correct
    // -- LOCKED, right ppb, right master clock id -- and DC still showed us as
    // PTPv2 Domain 0 / Priority 0/0 with Primary v1 Multicast N/A, i.e. it was
    // not ingesting the reply at all. A controller that cannot correlate a
    // response to its request has no reason to.
    //
    // Only byte 3 changes, 0x21 (request) -> 0x20 (reply), which is the one
    // part of the opcode inferno's own dispatch treats as fixed; every other
    // byte, including the 0x3e family and the trailing 0x64, is carried back.
    // req == NULL for the periodic announcement, which has nothing to echo.
    // The 0x2a family byte is per-device (the RedNets use 0x38 and 0x32); ours
    // is the one we already advertise on every other info message.
    static const uint8_t op_default[8] = {0x07, 0x2a, 0x00, 0x20, 0, 0, 0, 0};
    uint8_t op[8];
    uint16_t rseq;
    if (req) {
        memcpy(op, req + 24, 8);
        op[3] = 0x20;
        rseq = (uint16_t)((req[4] << 8) | req[5]);
    } else {
        memcpy(op, op_default, 8);
        rseq = seqnum;
    }

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr_seq(p, 0xFFFF, op, rseq);
    uint8_t *c = p + n;

    memcpy(c, clock_stats_tmpl, sizeof(clock_stats_tmpl));

    // Status word: 0x0003 locked, 0x0001 not (inferno: 0x01 = PLL not locked).
    c[2] = 0x00; c[3] = g_ptpv1.locked ? 0x03 : 0x01;

    int32_t ppb = 0;
    if (g_ptpv1.base_addend_full) {
        int64_t base = (int64_t)g_ptpv1.base_addend_full;
        int64_t cur  = (int64_t)g_ptpv1.current_addend_full;
        ppb = (int32_t)(((cur - base) * 1000000000LL) / base);
    }
    c[8]  = (uint8_t)(ppb >> 24); c[9]  = (uint8_t)(ppb >> 16);
    c[10] = (uint8_t)(ppb >> 8);  c[11] = (uint8_t)ppb;

    // MAC + 0x0000 form, at 12 (us), 20 (grandmaster), 28 (parent).
    memcpy(c + 12, g_dante.mac, 6);          c[18] = 0; c[19] = 0;
    memcpy(c + 20, g_ptpv1.master_uuid, 6);  c[26] = 0; c[27] = 0;
    memcpy(c + 28, g_ptpv1.master_uuid, 6);  c[34] = 0; c[35] = 0;

    // EUI-64 form, at 120 (us), 128 and 136 (the leader).
    uint8_t eui[8];
    eui[0] = g_ptpv1.master_uuid[0]; eui[1] = g_ptpv1.master_uuid[1];
    eui[2] = g_ptpv1.master_uuid[2]; eui[3] = 0xFF; eui[4] = 0xFE;
    eui[5] = g_ptpv1.master_uuid[3]; eui[6] = g_ptpv1.master_uuid[4];
    eui[7] = g_ptpv1.master_uuid[5];
    memcpy(c + 120, g_dante.device_id, 8);
    memcpy(c + 128, eui, 8);
    memcpy(c + 136, eui, 8);

    n += sizeof(clock_stats_tmpl);
    put_u16(p, 2, (uint16_t)n);

    if (net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_info++;
    seqnum++;
}

// ---------------------------------------------------------------------------
// Capability messages 0x0080 / 0x0082 / 0x0084 / 0x1009
//
// Real devices announce these; we announced none of them. The pattern
// throughout this bring-up has been that DC falls back to a DEFAULT when a
// device says nothing -- that is how we ended up displayed as PTPv2 Domain 0 --
// and "Enable Sync To External" showing an enabled checkbox for us, while both
// DVS and the AM2 show N/A, has the same shape.
//
// HONEST LIMIT: I could not isolate the field that controls that column. It is
// not any of these on the evidence available: [12:14] of 0x0080 is 0x0002 on
// BOTH the A16R (checkbox) and the AM2 (N/A), and 0x0084 is byte-identical on
// all three devices. So this is declaring our real capabilities rather than a
// targeted fix, and it may not move that column.
//
// 0x0080 is the supported-sample-rate table -- the tail is a list of rates
// (0xac44 = 44100, 0xbb80 = 48000, 0x15888 = 88200, 0x17700 = 96000, and the
// A16R adds 176400/192000). We declare 48000 and 96000: both are really
// produced (de USB-descriptors zijn op 96k gedimensioneerd, de NCO en de
// timestamp-rekenkunde volgen g_fs). Nog steeds NIET DVS's zes rates
// overnemen -- adverteer niets wat dit ontwerp niet kan leveren.
//
// DE HEADER-LAYOUT IS GECORRIGEERD. Hij stond zo:
//     0x0018, count, current, 0, 0, 0, rates...
// met het commentaar "DVS has 0 here, the A16R 2". Een AM2 -- en de
// implementatie die het rate-menu in DC daadwerkelijk krijgt -- stuurt:
//     0x0018, count, current, CURRENT (nogmaals), 2, 0, rates...
// Offset 8 droeg dus een 0 waar de rate hoort en offset 12 een 0 waar een 2
// hoort. Ons eigen 0x0082 zet daar al wel een 2, dus de inconsistentie zat
// binnen dit bestand.
// ---------------------------------------------------------------------------

// De rates die dit ontwerp echt kan. Volgorde zoals een AM2: oplopend.
static const uint32_t RATES[] = { 48000, 96000 };

// req != NULL -> antwoord op een concreet verzoek: echo zijn opcode en seqnum,
// met byte 3 op 0x80 geforceerd (de SET komt binnen als 0x81 maar het antwoord
// is altijd de 0x80-tabel).
static void send_rate_table(const uint8_t *dst_ip, uint16_t dst_port,
                            const uint8_t *req, uint32_t hz)
{
    static const uint8_t op_default[8] = {0x07, 0x2a, 0x00, 0x80, 0, 0, 0, 0};
    uint8_t op[8];
    uint16_t seq = seqnum;
    memcpy(op, op_default, 8);
    if (req) {
        memcpy(op, req + 24, 8);
        op[3] = 0x80;
        seq = (uint16_t)((req[4] << 8) | req[5]);
    }
    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr_seq(p, 0xFFFF, op, seq);
    uint8_t *c = p + n;
    const uint16_t nr = (uint16_t)(sizeof(RATES) / sizeof(RATES[0]));
    put_u16(c,  0, 0x0018);
    put_u16(c,  2, nr);
    put_u32(c,  4, hz);
    put_u32(c,  8, hz);
    put_u16(c, 12, 2);
    put_u16(c, 14, 0);
    for (uint16_t i = 0; i < nr; i++) put_u32(c, 16 + 4 * i, RATES[i]);
    n += 16 + 4 * nr;
    put_u16(p, 2, (uint16_t)n);
    net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n, NET_TOS_BEST_EFFORT);
    seqnum++;
}

static int rate_supported(uint32_t hz)
{
    for (unsigned i = 0; i < sizeof(RATES) / sizeof(RATES[0]); i++)
        if (RATES[i] == hz) return 1;
    return 0;
}

static void send_caps(const uint8_t *dst_ip, uint16_t dst_port)
{
    send_rate_table(dst_ip, dst_port, NULL, g_fs);
    // 0x0082: supported sample FORMATS (bit depths). DVS lists 0x10/0x18/0x20
    // = 16/24/32-bit; both RedNets list 0x18 only. We are 24-bit only
    // (dante_dev.c bits_per_sample, "pcm=3" on the wire), so we declare one.
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x82, 0, 0, 0, 0};
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = put_hdr(p, 0xFFFF, op);
        uint8_t *c = p + n; uint32_t o = 0;
        put_u16(c, o, 0x0018); o += 2;
        put_u16(c, o, 1);      o += 2;        // one supported format
        put_u32(c, o, 24);     o += 4;        // current: 24-bit
        put_u32(c, o, 0);      o += 4;
        put_u16(c, o, 2);      o += 2;
        put_u16(c, o, 0);      o += 2;
        put_u32(c, o, 24);     o += 4;        // the one format we support
        n += o; put_u16(p, 2, (uint16_t)n);
        net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n, NET_TOS_BEST_EFFORT);
        seqnum++;
    }
    // 0x1009: DVS sends 16 zero bytes. Reproduced as-is -- the AM2's variant
    // carries device ids and is clearly not device-independent, so the
    // all-zero form is the safer of the two observed.
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x10, 0x09, 0, 0, 0, 0};
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = put_hdr(p, 0xFFFF, op);
        memset(p + n, 0, 16); n += 16;
        put_u16(p, 2, (uint16_t)n);
        net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n, NET_TOS_BEST_EFFORT);
        seqnum++;
    }
    // 0x0084 is byte-identical on the A16R, the AM2 and DVS, so it carries no
    // per-device state and is reproduced as-is.
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x84, 0, 0, 0, 0};
        static const uint8_t body[60] = {
            0x00,0x30,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
            0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x04,
        };
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = put_hdr(p, 0xFFFF, op);
        memcpy(p + n, body, sizeof(body)); n += sizeof(body);
        put_u16(p, 2, (uint16_t)n);
        net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n, NET_TOS_BEST_EFFORT);
        seqnum++;
    }
}

// RUNTIME OVERRIDE OF THE NETWORK-INFO PAYLOAD.
//
// Dante Controller now ASKS us for this message -- it never did before today.
// Advertising a DirectOut PRODIGY's model id (mdns.c) is what flipped that:
// with model=_00000000000000ff DC never touched the info port, with
// _0000000000000020 it immediately queries q=0x13 (this reply), 0x83, 0x61 and
// 0xc1. Found with a proxy on the build host that borrowed the PRODIGY's TXT
// records: DC asked IT for model information and never asked our board.
//
// The field is still grey, so whatever decides it is in these 64 bytes -- and
// we do not know what any of them mean. Gateway and DNS are zero, the netmask
// comes from a 16-or-24 prefix, the leading six bytes are 00 01 00 00 00 00 and
// the 32-byte tail starts 00 18 00 30 and is otherwise empty.
//
// Sweeping that by rebuilding is one firmware push and one audio dropout per
// guess. The 0x1100 table has had runtime patching for exactly this reason
// ("Patching at runtime turns that into seconds"), so this message gets it too:
// UDP opcode 'N' writes bytes into an override array with a per-byte mask, and
// the builder applies them last. One build, then as many experiments as it
// takes.
static uint8_t g_ni_ovr[64];
static uint8_t g_ni_msk[64];

int dante_info_netinfo_patch(uint16_t off, const uint8_t *val, uint8_t n)
{
    if (off >= sizeof(g_ni_ovr)) return -1;
    if (off + n > sizeof(g_ni_ovr)) n = (uint8_t)(sizeof(g_ni_ovr) - off);
    for (uint8_t i = 0; i < n; i++) {
        g_ni_ovr[off + i] = val[i];
        g_ni_msk[off + i] = 1;
    }
    printf("[info] netinfo patch @%u x%u\n", off, n);
    return 0;
}

void dante_info_netinfo_clear(void)
{
    for (unsigned i = 0; i < sizeof(g_ni_msk); i++) g_ni_msk[i] = 0;
    printf("[info] netinfo overrides cleared\n");
}

// STATIC ADDRESSING STATE.
//
// THIS DEVICE IS ALWAYS STATICALLY CONFIGURED, so the 0x0002 interface flag is
// unconditional. net_init() takes its address from g_cfg.osc_ip at every boot
// (main.c); there is no DHCP client and no link-local negotiation anywhere in
// this firmware. 169.254.9.200 sitting in the link-local RANGE does not make it
// link-local ASSIGNED -- the PRODIGY reports 0x0001 on its own 169.254 address
// because it auto-assigned that address, which is a different thing.
//
// This started out as a g_if_static flag set by a controller's write, and that
// was wrong in a way the reboot exposed: the flag lived in RAM, so the board
// would come up from a write it had just honoured reporting the address as NOT
// static. The device's actual behaviour does not depend on being told.
//
// g_if_pend* is a REQUESTED address that is not live yet, reported in the
// reboot records so Controller shows "must be rebooted" -- which is how real
// Dante hardware behaves and, here, the only honest option: changing g_net_ip
// under a running stack would strand every bound flow, every ARP cache entry on
// the segment and our own reply in flight.
static uint8_t g_if_pend;
static uint8_t g_if_pend_ip[4];
static uint8_t g_if_pend_mask[4];

static void send_network_info(const uint8_t *dst_ip, uint16_t dst_port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00};

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = put_hdr(p, 0xFFFF, op);
    uint8_t *c = p + n;
    uint32_t o = 0;

    // Layout from the conmon_audinate_interface_* accessors in
    // libDanteController.dylib, checked byte for byte against a PRODIGY1-32's
    // reply (STATIC_ADDRESSING_HANDOFF.md):
    //
    //   +0   u16 interface count n
    //   +2   u16 mode
    //   +4   n records of 28 bytes:
    //          u32 link speed Mbps, u16 flags, mac[6], ip, netmask, dns, gateway
    //   then u16 0x0018  reboot record size
    //        u16 offset  of the reboot records, 28*n + 0x14, counted from the
    //                    OPCODE, which is 8 bytes before this content
    //        u16 message flags
    //        u16 0
    //   then n reboot records of 0x18 bytes
    //
    // This was already the right shape -- the bytes used to be a `lead[6]` and
    // a `tail[32]` blob with the fields unnamed, which is why patching it at
    // runtime produced a message that no longer parsed (the handoff's "FPGA
    // interface 2 shifted by 2 bytes" is one of those probes, not this code).
    //
    // ONE interface, because the gateware instantiates one RGMII PHY
    // (avb_soc.py: platform.request("eth", 1)). The PRODIGY reports two because
    // it has a redundant port; claiming a second one here would be advertising
    // hardware that does not exist. It is also not what unlocks the form:
    // patching this message to n=2 made Controller draw primary AND secondary,
    // both still grey. The capability bit in 0x60 is what does it.
    const uint32_t n_if = 1;

    put_u16(c, o, n_if); o += 2;

    // `mode` AT +2: STILL UNKNOWN, BUT 0 IS THE VALUE A ONE-INTERFACE DEVICE
    // SENDS. This was briefly 1, copied from the PRODIGY because the PRODIGY's
    // panel works -- and the PRODIGY has TWO interfaces. Measured 2026-11-07 by
    // querying every Dante device on the bench:
    //
    //   DAC8 (inferno on an ESP32-P4), 1 interface : mode 0
    //   PRODIGY1-32,                   2 interfaces: mode 1
    //
    // and inferno's own send_network_info hardcodes the same six lead bytes
    // 00 01 00 00 00 00, i.e. n=1 with mode 0 (info_mcast_server.rs:355). It
    // does not name the field either, so what the value MEANS is still not
    // known -- only that both one-interface references say 0 and the only
    // two-interface reference says 1. Sending 1 from a single-PHY device was
    // advertising something we cannot back.
    put_u16(c, o, 0);    o += 2;

    put_u32(c, o, 1000);   o += 4;            // link speed, Mbps -> "1Gbps"
    // bit 0 = link up, 0x0002 = statically configured -- see the note above
    // on why the second bit is not conditional on anything.
    put_u16(c, o, 0x0003); o += 2;

    memcpy(c + o, g_dante.mac, 6); o += 6;
    memcpy(c + o, g_net_ip,    4); o += 4;

    // Netmask from the prefix, then DNS and gateway -- we have neither. The
    // order is the decompiled one; it used to be commented the other way round,
    // which was invisible because both are zero.
    uint8_t mask[4] = {255, 255, 0, 0};
    if (g_net_prefix == 24) { mask[2] = 255; }
    memcpy(c + o, mask, 4); o += 4;
    memset(c + o, 0, 4);    o += 4;           // DNS
    memset(c + o, 0, 4);    o += 4;           // gateway

    put_u16(c, o, 0x0018);           o += 2;  // reboot record size
    put_u16(c, o, 28 * n_if + 0x14); o += 2;  // reboot record offset
    put_u16(c, o, 0x0003);           o += 2;  // message flags, as the PRODIGY
    put_u16(c, o, 0x0000);           o += 2;

    // Reboot config. The PRODIGY sends these ALL ZERO -- no pending change --
    // and Controller still offers the form, so zero is the resting state. A
    // value that DIFFERS from the live config is what makes Controller say the
    // device must be rebooted, which is also the only way this device can apply
    // a new address (see handle_netinfo_write).
    memset(c + o, 0, 0x18 * n_if);
    if (g_if_pend) {
        // 0x0004 = valid, 0x0002 = static -- so 0x0006 for a statically
        // configured pending address. INFERENCE, NOT MEASUREMENT: the
        // decompile gives 0x0004 as "valid" and
        // STATIC_ADDRESSING_HANDOFF.md reasons that a static address wants
        // 0x0006. There is no reference capture to check it against, because
        // the PRODIGY sends its reboot records ALL ZERO -- it has nothing
        // pending, so real hardware never showed us a filled-in one. This sent
        // 0x0004 first; if Controller acts on 0x0006 where it ignored 0x0004,
        // that is the confirmation.
        put_u16(c, o + 0, 0x0006);              // valid | static
        memcpy(c + o + 4,  g_if_pend_ip,   4);
        memcpy(c + o + 8,  g_if_pend_mask, 4);
        // DNS at +12 and gateway at +16 stay zero: we have neither, and the
        // write that set this could not give us one (see below).
    }
    o += 0x18 * n_if;

    // Overrides last, so a probe can reach any byte including the ones computed
    // above. `o` is 64 here; the arrays are sized to match.
    for (uint32_t i = 0; i < o && i < sizeof(g_ni_ovr); i++)
        if (g_ni_msk[i]) c[i] = g_ni_ovr[i];

    n += o;
    put_u16(p, 2, (uint16_t)n);

    if (net_udp_commit(dst_ip, dst_port, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT) == 0)
        g_info_stats.tx_info++;
    seqnum++;
}

// EXECUTING DANTE CONTROLLER'S 0x13 WRITE.
//
// Message layout from the conmon_audinate_interface_* setters in
// libDanteController.dylib, and every field below was then read off the wire:
// DC's own Apply, captured by the on-device ring (tools/cap_fetch.py), is the
// only place this message is visible -- it is unicast, and an unmanaged switch
// forwards it to the board's port alone.
//
//   content +00 u8  interface count          DC writes 1
//           +01 u8  record size              0x1c
//           +02 u8  field mask               0x0f on a static write
//           +03 u8  record offset FROM THE OPCODE, i.e. content + (off - 8)
//           +04 u16 mask      } only for message version >= 0x0602
//           +06 u16 flags     }
//   record  +00 u16 interface index
//           +02 u16 flag values      0x0002 = static
//           +04 ip   +08 netmask   +12 dns   +16 gateway
//           +20 u16 which of those flags are valid
//
// MEASURED 2026-11-07, three things the decompile left open:
//   - the interface index COUNTS FROM 0 (DC addressed our only interface as 0),
//   - the static setter really does send flag values 0x0002 AND valid 0x0002,
//   - DC retries the write every ~2 s while we answer with an unchanged 0x11.
//
// What DC asked for was "make the current address static": index 0, static,
// 169.254.9.200/255.255.0.0, no DNS, no gateway.
//
// WE DO NOT SUPPORT DNS OR A GATEWAY, and a request for either is refused
// rather than accepted and dropped. There is nothing in this firmware that
// would use them, and reporting zeros back while having said "fine" is the kind
// of quiet lie that costs a session to find.
static int mask_prefix(const uint8_t m[4])
{
    if (m[0] == 255 && m[1] == 255 && m[2] == 0   && m[3] == 0) return 16;
    if (m[0] == 255 && m[1] == 255 && m[2] == 255 && m[3] == 0) return 24;
    return 0;   // net.c carries a /16 or /24 and nothing else
}

static int ip_usable(const uint8_t a[4], int prefix)
{
    if (a[0] == 0 || a[0] == 127 || a[0] >= 224) return 0;  // zero, loopback, multicast+
    if (prefix == 16) {
        if (a[2] == 0   && a[3] == 0)   return 0;           // network address
        if (a[2] == 255 && a[3] == 255) return 0;           // broadcast
    } else {
        if (a[3] == 0 || a[3] == 255)   return 0;
    }
    return 1;
}

static void handle_netinfo_write(const uint8_t *req, uint32_t len,
                                 const uint8_t src_ip[4], uint16_t src_port)
{
    const uint8_t *c    = req + MCAST_HDR_LEN;
    uint32_t       clen = len - MCAST_HDR_LEN;

    // Every rejection still answers with 0x11. A controller that gets silence
    // retries forever (measured: every ~2 s); one that gets the current state
    // can see its request did not take.
    if (clen < 8) {
        printf("[info] 0x13 write too short (%lu)\n", (unsigned long)clen);
        send_network_info(src_ip, src_port);
        return;
    }

    uint8_t n_rec = c[0], rsize = c[1], fmask = c[2], roff = c[3];

    if (n_rec < 1 || roff < 8 || rsize < 22) {
        printf("[info] 0x13 write odd header n=%u rsize=%u off=%u\n",
               n_rec, rsize, roff);
        send_network_info(src_ip, src_port);
        return;
    }
    uint32_t off = (uint32_t)roff - 8;
    if (off + 22 > clen) {
        printf("[info] 0x13 record runs past the message\n");
        send_network_info(src_ip, src_port);
        return;
    }

    const uint8_t *r    = c + off;
    uint16_t       idx  = (uint16_t)((r[0] << 8) | r[1]);
    uint16_t       fval = (uint16_t)((r[2] << 8) | r[3]);
    uint16_t       fvld = (uint16_t)((r[20] << 8) | r[21]);

    if (idx != 0) {
        printf("[info] 0x13 for interface %u; we have one, index 0\n", idx);
        send_network_info(src_ip, src_port);
        return;
    }

    // ADDRESSING MODES WE DO NOT HAVE. Flag bits from DC's own setters
    // (STATIC_ADDRESSING_HANDOFF.md): 0x0002 static, 0x0020 DHCP, 0x0008
    // link-local, each paired with a "which flags are valid" mask. Asking this
    // firmware to stop being static, or to start doing DHCP or link-local
    // assignment, is asking for code that does not exist -- so it is refused
    // rather than accepted and ignored. Turning OFF something we never had is a
    // no-op and passes.
    if ((fvld & 0x0002) && !(fval & 0x0002)) {
        printf("[info] 0x13 wil niet-statisch; dit apparaat kan alleen static\n");
        send_network_info(src_ip, src_port);
        return;
    }
    if ((fvld & 0x0020) && (fval & 0x0020)) {
        printf("[info] 0x13 wil DHCP; geen DHCP-client aanwezig\n");
        send_network_info(src_ip, src_port);
        return;
    }
    if ((fvld & 0x0008) && (fval & 0x0008)) {
        printf("[info] 0x13 wil link-local toewijzing; niet geimplementeerd\n");
        send_network_info(src_ip, src_port);
        return;
    }

    if ((fmask & 0x04) && (r[12] | r[13] | r[14] | r[15])) {
        printf("[info] 0x13 asks for a DNS server; unsupported\n");
        send_network_info(src_ip, src_port);
        return;
    }
    if ((fmask & 0x08) && (r[16] | r[17] | r[18] | r[19])) {
        printf("[info] 0x13 asks for a gateway; unsupported\n");
        send_network_info(src_ip, src_port);
        return;
    }

    if (fmask & 0x01) {
        const uint8_t *ip   = r + 4;
        const uint8_t *mask = r + 8;
        int prefix = (fmask & 0x02) ? mask_prefix(mask) : g_net_prefix;

        if (prefix == 0) {
            printf("[info] 0x13 netmask %u.%u.%u.%u unsupported (/16 or /24)\n",
                   mask[0], mask[1], mask[2], mask[3]);
            send_network_info(src_ip, src_port);
            return;
        }
        if (!ip_usable(ip, prefix)) {
            printf("[info] 0x13 refuses %u.%u.%u.%u/%d\n",
                   ip[0], ip[1], ip[2], ip[3], prefix);
            send_network_info(src_ip, src_port);
            return;
        }

        // Byte for byte: this libc has memcpy and memset but no memcmp, and
        // the link failure is the only warning you get.
        int same = (ip[0] == g_net_ip[0] && ip[1] == g_net_ip[1] &&
                    ip[2] == g_net_ip[2] && ip[3] == g_net_ip[3]);
        // cfg_save() WRITES SPI FLASH, AND DC RETRIES EVERY ~2 s. So the
        // stored value decides whether to write, not the fact that a request
        // arrived -- otherwise a controller that keeps re-sending the same
        // address grinds the flash for as long as its dialog is open.
        int cfg_differs = !(g_cfg.osc_ip[0] == ip[0] && g_cfg.osc_ip[1] == ip[1] &&
                            g_cfg.osc_ip[2] == ip[2] && g_cfg.osc_ip[3] == ip[3] &&
                            g_cfg.osc_prefix == (uint8_t)prefix);

        if (same && prefix == g_net_prefix) {
            // Exactly what we already are: nothing pending, nothing to reboot
            // for. This is the case DC's first Apply produced. It also CANCELS
            // an earlier pending change -- setting the address back to what it
            // is must undo the stored one, or the next boot would still move
            // the board to an address nobody is asking for any more.
            g_if_pend = 0;
            if (cfg_differs) {
                memcpy(g_cfg.osc_ip, ip, 4);
                g_cfg.osc_prefix = (uint8_t)prefix;
                cfg_save();
                printf("[info] 0x13 cancelt de opgeslagen wijziging\n");
            }
        } else {
            memcpy(g_if_pend_ip, ip, 4);
            g_if_pend_mask[0] = 255; g_if_pend_mask[1] = 255;
            g_if_pend_mask[2] = (prefix == 24) ? 255 : 0;
            g_if_pend_mask[3] = 0;
            g_if_pend = 1;

            if (cfg_differs) {
                memcpy(g_cfg.osc_ip, ip, 4);
                g_cfg.osc_prefix = (uint8_t)prefix;
                cfg_save();
                printf("[info] 0x13 -> %u.%u.%u.%u/%d na reboot (opgeslagen)\n",
                       ip[0], ip[1], ip[2], ip[3], prefix);
            }
        }
    }

    send_network_info(src_ip, src_port);
}

// ---------------------------------------------------------------------------
// Requests arriving on 8700
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SAMPLE RATE SET (info opcode 0x81 met content)
//
// De SET is DEZELFDE opcode als de rate-query -- onderscheiden op content:
//     u32 1          (set)
//     u32 rate
// Zonder content is het een query. Dit is gecaptured van de controller die een
// rate kiest (zie Inferno-ESP32-P4 aoip_info.c handle_rate_set), en DC biedt
// het alleen aan als board-info byte 0x17 configuratie adverteert.
//
// Een AM2 antwoordt met de nieuwe rate-tabel, dan 0x0100 met body
// 01 01 00 00 00 00 00 02, dan 0x0106 leeg; daarna slaat hij op en herstart.
//
// WIJ HERSTARTEN NIET, EN DAT IS EEN CORRECTIE OP EEN EERDERE VERSIE HIERVAN.
// Die deed ctrl_reset_write(1) zoals de ESP-implementatie, maar daar zit de
// firmware in flash en hier in CODERAM: een reset landt in de loader, die op
// een image over het netwerk wacht. Gemeten op 2026-10-30: na zo'n reset
// antwoordde het bord niet meer en kreeg zelfs netload hem niet terug -- alleen
// een bitstream-reload. Op de bench is dat een vastloper, in productie een
// apparaat dat van het netwerk valt zodra iemand een menu aanraakt.
//
// Het hoeft ook niet. Alle vier dingen die van de rate afhangen zijn runtime
// beschrijfbaar: g_fs, de NCO (mcr_dante_init), de subsecond-wrap in de
// packetizer en de USB-clock-source. Dus we zetten ze live en laten de flows
// opnieuw onderhandelen. De USB-host leest zijn rate pas bij de volgende
// enumeratie, dus die heeft een replug of een poort-cycle nodig -- dat is de
// enige handeling die overblijft, en hij is zichtbaar in plaats van stil.
// ---------------------------------------------------------------------------
// Ook aangeroepen door de 'R'-opcode in dstats.c.
void dante_info_rate_apply(uint32_t hz)
{
    g_fs = hz;
    // Volgorde: eerst de wrap, dan de NCO. De packetizer vergelijkt ts_sub
    // tegen deze CSR, dus hem na de NCO zetten laat een paar strobes door met
    // de oude grens en dat is precies een verkeerd gewrapte timestamp.
#ifdef CSR_AAF_PKT_TS_SUBSEC_MAX_ADDR
    aaf_pkt_ts_subsec_max_write(hz);
#endif
    mcr_dante_init(CONFIG_CLOCK_FREQUENCY, hz);
#ifdef CSR_MAIN_USB_RATE_48K_ADDR
    main_usb_rate_48k_write(hz == 48000 ? 1 : 0);
#endif
    g_dante.sample_rate = hz;
    // Ontvangers luisteren nog op de oude rate en hun fpp hoort bij de oude
    // sample-periode, dus laat ze opnieuw onderhandelen in plaats van ze stil
    // verkeerd te laten draaien.
    dante_tx_drop_all();
}

static void handle_rate_set(const uint8_t *req, uint32_t len,
                            const uint8_t src_ip[4], uint16_t src_port)
{
    (void)len;
    uint32_t hz = ((uint32_t)req[36] << 24) | ((uint32_t)req[37] << 16) |
                  ((uint32_t)req[38] << 8)  |  (uint32_t)req[39];

    if (hz == g_fs) {                       // al goed: alleen bevestigen
        send_rate_table(src_ip, src_port, req, hz);
        return;
    }
    if (!rate_supported(hz)) {
        printf("[info] rate %lu Hz geweigerd; blijf op %lu\n",
               (unsigned long)hz, (unsigned long)g_fs);
        send_rate_table(src_ip, src_port, req, g_fs);   // de WARE rate melden
        return;
    }

    g_cfg.sample_rate = hz;
    cfg_save();
    dante_info_rate_apply(hz);
    printf("[info] rate -> %lu Hz door de controller (live; USB-host leest de "
           "nieuwe rate bij zijn volgende enumeratie)\n", (unsigned long)hz);

    send_rate_table(src_ip, src_port, req, hz);
    {   // 0x0100
        static const uint8_t op[8]   = {0x07, 0x2a, 0x01, 0x00, 0, 0, 0, 0};
        static const uint8_t body[8] = {0x01, 0x01, 0, 0, 0, 0, 0, 0x02};
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = put_hdr(p, 0xFFFF, op);
        memcpy(p + n, body, sizeof(body)); n += sizeof(body);
        put_u16(p, 2, (uint16_t)n);
        net_udp_commit(grp_devinfo, DANTE_PORT_INFO, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT);
        seqnum++;
    }
    {   // 0x0106, leeg
        static const uint8_t op[8] = {0x07, 0x2a, 0x01, 0x06, 0, 0, 0, 0};
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = put_hdr(p, 0xFFFF, op);
        put_u16(p, 2, (uint16_t)n);
        net_udp_commit(grp_devinfo, DANTE_PORT_INFO, DANTE_PORT_INFO_REQ, n,
                       NET_TOS_BEST_EFFORT);
        seqnum++;
    }
}

// DANTE CONTROLLER'S REBOOT BUTTON IS INFO OPCODE 0x90.
//
// Captured off the wire on 2026-11-07, which is the only way this was ever
// going to be found: the command is UNICAST to our info port, and it was
// invisible for one round of looking because the capture ring dropped all
// multicast and so could not prove the difference between "DC sent nothing" and
// "we cannot see it" (see firmware/cap.c).
//
//   RX INFO 169.254.228.170:8700 -> :8700  len 78
//   ffff 0024 6a5d 0000 c8a362ebd51a0000 "Audinate" 07 3d 00 90 00 00 00 64
//   00 01 00 00
//
// So: 36 bytes, four of content, version 0x073d (a CONTROLLER asking for
// something, as against 0x072a which is a device talking about itself),
// preceded by a CMC 0x1001 device-advertisement request -- the same two-step DC
// uses before the 0x13 interface write.
//
// MEASURED, AND WORTH RELYING ON: DC sent it EXACTLY ONCE and did not retry,
// unlike the 0x13 write which it repeated every ~2 s until the answer reflected
// it. A reboot command needs no acknowledgement, because the device going away
// IS the acknowledgement. So this sends no reply.
//
// NOT KNOWN: what the content means. `00 01` is honoured here and ANYTHING ELSE
// IS REFUSED AND LOGGED, rather than assumed to also mean reboot. The
// capability byte at content[0x17] of 0x60 is annotated "identify / rate config
// / reboot / factory reset" with the bits never isolated, so 0x90 may well be a
// family of system commands with the u16 selecting which. Resetting the board
// because a command we cannot read arrived on the same opcode is not a risk
// worth taking for a guess. inferno does not handle 0x90 at all, so there is no
// reference to check against.
//
// THE INTERLOCK IS THE POINT OF THIS FUNCTION.
//
// This firmware runs from coderam, not flash. A reset enters the loader, which
// waits for netload and then falls through to the SPI image at 0x300000 -- and
// if there is no image there it HALTS rather than jump into junk
// (firmware/loader/loader.c). So honouring a reboot on a board that has only
// ever had `./flash.sh` (volatile) run against it turns one click in Controller
// into a board that needs JTAG to come back. The image header is checked first
// and the reboot is refused if it is not there.
//
// THE CHECK VERIFIES THE CRC, AND IT HAS TO.
//
// This first checked only the magic and a sane length, with a comment admitting
// it did not cover "flashed but since corrupted" and arguing that case was
// far-fetched. It was not: replaying DC's reboot bytes on 2026-11-07 passed the
// magic check, reset the board, and the loader then REFUSED the image and
// halted -- exactly the brick the interlock exists to prevent, on the very
// first test. Something at 0x300000 carries valid "INFN" magic and a plausible
// length and does not pass its own CRC. An interlock that does not check what
// the loader checks is not an interlock.
//
// COST: the image is streamed out of SPI in 256-byte chunks and CRC'd
// bitwise -- for a 75 KB image that is ~600k iterations plus the SPI reads,
// which stalls the main loop for on the order of 100 ms and will glitch the
// audio flows. That is the right trade: it only runs when a reboot command
// arrives, and on the accept path the board is about to reset anyway. On the
// refuse path it costs one glitch instead of a trip to the JTAG cable.
#define IMG_ADDR   0x00300000u
#define IMG_MAGIC  0x494E464Eu          /* "INFN" */

// IEEE CRC-32, reflected, bitwise -- the same function the loader applies to
// the image it reads (firmware/loader/loader.c), in seedable form so the image
// can be streamed instead of buffered. It must stay byte-for-byte equivalent to
// that one or the interlock would be checking a different question.
static uint32_t crc32_upd(uint32_t crc, const uint8_t *p, uint32_t n)
{
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return crc;
}

static int spi_image_present(void)
{
#ifdef CSR_CFGFLASH_BASE
    uint8_t hdr[12];
    cfgflash_warmup();
    cfgflash_read(IMG_ADDR, hdr, sizeof(hdr));
    uint32_t magic = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
                     ((uint32_t)hdr[2] << 8)  |  (uint32_t)hdr[3];
    uint32_t len   = ((uint32_t)hdr[4] << 24) | ((uint32_t)hdr[5] << 16) |
                     ((uint32_t)hdr[6] << 8)  |  (uint32_t)hdr[7];
    uint32_t want  = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                     ((uint32_t)hdr[10] << 8) |  (uint32_t)hdr[11];
    if (magic != IMG_MAGIC) {
        printf("[info] reboot geweigerd: geen SPI-image (magic %08lx)\n",
               (unsigned long)magic);
        return 0;
    }
    if (len == 0 || len > 0x40000u) {
        printf("[info] reboot geweigerd: SPI-image lengte %lu\n",
               (unsigned long)len);
        return 0;
    }

    uint8_t  buf[256];
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t off = 0; off < len; ) {
        uint32_t n = len - off;
        if (n > sizeof(buf)) n = sizeof(buf);
        cfgflash_read(IMG_ADDR + 12u + off, buf, n);
        crc = crc32_upd(crc, buf, n);
        off += n;
    }
    crc = ~crc;
    if (crc != want) {
        printf("[info] reboot geweigerd: SPI-image CRC %08lx, wil %08lx\n",
               (unsigned long)crc, (unsigned long)want);
        return 0;
    }
    return 1;
#else
    printf("[info] reboot geweigerd: geen config-flash in deze SoC\n");
    return 0;
#endif
}

static void handle_sys_reset(const uint8_t *req, uint32_t len,
                             const uint8_t src_ip[4])
{
    if (len < MCAST_HDR_LEN + 4) {
        printf("[info] 0x90 te kort (%lu)\n", (unsigned long)len);
        return;
    }
    uint16_t what = (uint16_t)((req[32] << 8) | req[33]);
    if (what != 0x0001) {
        g_info_stats.reboot_odd++;
        printf("[info] 0x90 met onbekende actie 0x%04x van %u.%u.%u.%u; "
               "genegeerd\n", what, src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        return;
    }
    g_info_stats.reboot_req++;

    if (!spi_image_present()) {
        g_info_stats.reboot_refused++;
        return;
    }

    printf("[info] reboot op verzoek van %u.%u.%u.%u\n",
           src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
    ctrl_reset_write(1);
    for (;;) { }                        /* reset is onderweg */
}

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
    // Met content is het een WRITE, zonder een query -- hetzelfde patroon als
    // 0x81 hieronder. DC's query is precies de 32-byte header.
    case 0x13:
        if (len > MCAST_HDR_LEN) handle_netinfo_write(req, len, src_ip, src_port);
        else                     send_network_info(src_ip, src_port);
        break;
    // Clock stats goes to the DEVICE-INFO MULTICAST GROUP, not back to the
    // requester. inferno sends it to device_info_destination like every other
    // info message, and this deviated from that by replying unicast.
    //
    // Which matters, because it explains the whole symptom. Our device/product/
    // network info reach DC as the 3 s multicast ANNOUNCEMENTS -- and those DC
    // ingests happily, the Device Info tab is fully populated. Clock stats is
    // the one message we only ever sent unicast on request, and it is the one
    // DC never acted on: the exchange completed, the content was correct
    // (LOCKED, right ppb, right master clock id), and the clock columns stayed
    // at PTPv2 Domain 0 / Priority 0/0 with v1 N/A.
    case 0x21:            send_clock_stats (grp_devinfo, DANTE_PORT_INFO, req); break;
    case 0x81:
        // Met content is het een SET (u32 1, u32 rate); zonder een query.
        if (len >= MCAST_HDR_LEN + 8 && req[32] == 0 && req[33] == 0 &&
            req[34] == 0 && req[35] == 1) {
            handle_rate_set(req, len, src_ip, src_port);
        } else {
            send_caps(src_ip, src_port);
        }
        break;
    case 0x83: case 0x85: send_caps(src_ip, src_port); break;
    case 0x90:            handle_sys_reset(req, len, src_ip); break;
    default:
        // Log unknowns: DC's requests to us are unicast and therefore invisible
        // to host-side capture, so the console is the only place they show up.
        g_info_stats.rx_unknown++;
        printf("[info] unhandled query 0x%02x from %u.%u.%u.%u\n", q,
               src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        break;
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
        // Announce clock state too, not only when asked.
        //
        // Dante Controller requests clock stats (0x21) only during a manual
        // refresh, so at boot it caches our honest "not locked" -- we have not
        // locked yet at that point -- and nothing updates it until the user
        // clicks Refresh. Sync therefore came up red and needed a manual
        // refresh to turn green, even though the clock had locked seconds
        // after boot.
        //
        // Announcing it on the same 3 s cadence as the other info messages
        // matches how DC consumes device state generally: it builds its view
        // from what arrives on the info multicast group rather than by polling.
        send_clock_stats (grp_devinfo, DANTE_PORT_INFO, 0);
        send_caps        (grp_devinfo, DANTE_PORT_INFO);
        info_announce_next_ms = now + INFO_ANNOUNCE_MS;
        return;                          // don't also heartbeat this pass
    }

    if (next_heartbeat_ms && (int32_t)(now - next_heartbeat_ms) < 0) return;
    next_heartbeat_ms = now + HEARTBEAT_MS;
    send_heartbeat();
}

void dante_info_init(void)
{
    if (net_udp_bind(DANTE_PORT_INFO_REQ, info_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", DANTE_PORT_INFO_REQ);
    net_igmp_join(grp_heartbeat);
    net_igmp_join(grp_devinfo);
    next_heartbeat_ms = 0;                       // fire on the next poll

    // Announcements are driven from dante_info_poll(), not sent here -- the link
    // is still down at this point. See the note on INFO_ANNOUNCE_MS.
    info_announce_next_ms = 0;
    printf("[info] heartbeat -> 224.0.0.233:%u, info -> 224.0.0.231:%u\n",
           DANTE_PORT_HEARTBEAT, DANTE_PORT_INFO);
}
