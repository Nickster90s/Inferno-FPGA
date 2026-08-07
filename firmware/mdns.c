// Minimal mDNS / DNS-SD responder — Dante Phase 3. See mdns.h.
//
// Modelled on captured traffic from a Focusrite RedNet AM2 and A16R, which
// answer a service query with SRV + TXT + A as answers plus the PTR, with the
// cache-flush bit set on the unique records. Nothing about Dante's use of mDNS
// turned out to be exotic -- it is ordinary DNS-SD (captures/README.md).

#include "mdns.h"
#include "net.h"
#include "dante_dev.h"
#include "dante_tx.h"
extern uint16_t g_mcast_fpp;
#include "gptp.h"
#include <string.h>
#include <stdio.h>

#define MDNS_PORT       5353
// TTL 10, and NO cache-flush bit. Both read off a channel reply the RedNets
// accept, byte for byte:
//
//   A16R (accepted)  ttl=10    class=0x0001   no cache-flush
//   ours (rejected)  ttl=4500  class=0x8001   cache-flush on every record
//
// The 4500 was a guess that this file recorded as "matches what Dante devices
// use". It does not -- real Dante channel records live for 10 seconds, which
// makes sense for state that changes whenever a patch changes.
//
// The cache-flush bit is the more likely blocker of the two. RFC 6762 sets it
// on unique records, but Dante does not, and a receiver that has cached a
// channel record may treat a flushing answer as an instruction to discard
// rather than as the resolution it asked for.
// TTLs measured off the A16R answering the same queries, not guessed:
//   PTR 4500, TXT 4500, SRV 120, A 120
// Both were 10, which is far shorter than anything real Dante hardware uses.
// At TTL 10 avahi expired our records every ten seconds, so a resolver had to
// re-query constantly and any gap in answering read as the channel vanishing.
// TXT carries b.<flow>=, which changes when a multicast flow is created or
// deleted, so it cannot use the A16R's 4500 s. PTR is static and keeps it.
#define MDNS_TTL        4500    // PTR
#define MDNS_TTL_TXT    120     // TXT -- dynamic, see b.<flow>= in build_txt_chan
#define MDNS_TTL_A      120     // A, and SRV

#define DNS_T_A         1
#define DNS_T_PTR       12
#define DNS_T_TXT       16
#define DNS_T_SRV       33
#define DNS_T_ANY       255

#define DNS_C_IN        1
// CACHE-FLUSH (RFC 6762 s10.2): tells a resolver to REPLACE what it holds for
// this name+type rather than add to it. Legal only on records we are the unique
// authority for -- A, SRV, TXT -- never on shared PTRs.
//
// Without it our channel TXT is immutable for its 4500 s TTL, and that broke
// subscription: a b.<flow>= key advertised while a multicast flow existed kept
// pointing receivers at that group long after the flow was gone. They joined a
// dead group instead of falling through to the unicast flow server, so no flow
// request ever arrived and no audio played. The key is dynamic now -- it appears
// and disappears with the flow -- so the records carrying it must be replaceable.
#define DNS_C_IN_FLUSH  0x8001
#define DNS_CACHE_FLUSH 0x8000

static const uint8_t mdns_group[4] = {224, 0, 0, 251};

mdns_stats_t g_mdns_stats;

// Boot announcement schedule. mDNS asks for 2-8 announcements a second apart;
// we send 3, which is enough for a controller to notice us without a burst.
static uint8_t  announce_left;
static uint32_t announce_next_ms;

// ---------------------------------------------------------------------------
// Name helpers
//
// Names are handled as lowercase dotted strings internally ("x._udp.local")
// and encoded/decoded at the wire boundary. That keeps comparisons trivial at
// the cost of a little copying, which is irrelevant at mDNS rates.
// ---------------------------------------------------------------------------

static char lc(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

// Decode a wire name at `off` into `out`. Follows compression pointers.
// Returns the offset just past the name in the ORIGINAL record, or 0 on error.
static uint32_t name_decode(const uint8_t *msg, uint32_t msglen, uint32_t off,
                            char *out, int outlen)
{
    int      o        = 0;
    uint32_t next     = 0;          // where the caller continues, once set
    int      jumps    = 0;

    while (off < msglen) {
        uint8_t l = msg[off];
        if (l == 0) {
            if (!next) next = off + 1;
            break;
        }
        if ((l & 0xC0) == 0xC0) {                   // compression pointer
            if (off + 1 >= msglen) return 0;
            if (!next) next = off + 2;
            off = (uint32_t)((l & 0x3F) << 8 | msg[off + 1]);
            if (++jumps > 8) return 0;              // pointer loop guard
            continue;
        }
        if ((l & 0xC0) != 0) return 0;              // reserved label type
        off++;
        if (off + l > msglen) return 0;
        if (o && o < outlen - 1) out[o++] = '.';
        for (uint8_t i = 0; i < l; i++)
            if (o < outlen - 1) out[o++] = lc((char)msg[off + i]);
        off += l;
    }
    out[o < outlen ? o : outlen - 1] = 0;
    return next ? next : off + 1;
}

// Encode a dotted name. Uncompressed, deliberately -- see mdns.h.
// NAME COMPRESSION.
//
// A working A16R channel reply -- one DVS accepted -- is 141 bytes with ~8
// compression pointers. Ours was 322 with ~1, because every name was written
// out in full. That was a deliberate simplification in the plan ("emit
// uncompressed names"), legal per RFC 1035, and the strongest remaining
// candidate for why two independent RedNet resolvers re-queried our records
// instead of accepting them.
//
// A small per-message cache of names already written. On a repeat we emit the
// 2-byte 0xC0 pointer instead of the full name, which is where nearly all the
// saving is: the instance name appears as the owner of both the SRV and the
// TXT record, and the SRV target reappears as the A record's owner.
static uint32_t name_encode(uint8_t *p, const char *name);   // fwd

#define NC_MAX   10
#define NC_NAME  96
static struct { char name[NC_NAME]; uint16_t off; } nc[NC_MAX];
static uint8_t nc_n;

static void nc_reset(void) { nc_n = 0; }

static int nc_find(const char *name)
{
    for (uint8_t i = 0; i < nc_n; i++) {
        const char *a = nc[i].name, *b = name;
        while (*a && *a == *b) { a++; b++; }
        if (!*a && !*b) return nc[i].off;
    }
    return -1;
}

static void nc_add(const char *name, uint32_t off)
{
    if (nc_n >= NC_MAX || off > 0x3FFF) return;
    uint32_t i = 0;
    while (name[i] && i < NC_NAME - 1) { nc[nc_n].name[i] = name[i]; i++; }
    nc[nc_n].name[i] = 0;
    nc[nc_n].off = (uint16_t)off;
    nc_n++;
}

// Write `name` at p+n, as a pointer if it has been written already.
static uint32_t name_put(uint8_t *p, uint32_t n, const char *name)
{
    int off = nc_find(name);
    if (off >= 0) {                       // whole name already written
        p[n++] = (uint8_t)(0xC0 | ((off >> 8) & 0x3F));
        p[n++] = (uint8_t)(off & 0xFF);
        return n;
    }

    // SUFFIX compression. Write labels one at a time; at each label boundary
    // check whether the REMAINDER has been written before and, if so, finish
    // with a pointer to it. That is how the A16R fits an SRV target into 18
    // bytes where our full name took 27: its ".local" is a pointer.
    //
    // Every suffix is registered as it is written, so a later name sharing any
    // tail can point into this one.
    nc_add(name, n);
    const char *cur = name;
    while (*cur) {
        int soff = (cur != name) ? nc_find(cur) : -1;
        if (soff >= 0) {
            p[n++] = (uint8_t)(0xC0 | ((soff >> 8) & 0x3F));
            p[n++] = (uint8_t)(soff & 0xFF);
            return n;
        }
        if (cur != name) nc_add(cur, n);
        const char *dot = cur;
        while (*dot && *dot != '.') dot++;
        uint32_t l = (uint32_t)(dot - cur);
        if (l > 63) l = 63;
        p[n++] = (uint8_t)l;
        for (uint32_t i = 0; i < l; i++) p[n++] = (uint8_t)cur[i];
        cur = *dot ? dot + 1 : dot;
    }
    p[n++] = 0;
    return n;
}

static uint32_t name_encode(uint8_t *p, const char *name)
{
    uint32_t n = 0;
    while (*name) {
        const char *dot = name;
        while (*dot && *dot != '.') dot++;
        uint32_t l = (uint32_t)(dot - name);
        if (l > 63) l = 63;
        p[n++] = (uint8_t)l;
        for (uint32_t i = 0; i < l; i++) p[n++] = (uint8_t)name[i];
        name = *dot ? dot + 1 : dot;
    }
    p[n++] = 0;
    return n;
}

// ---------------------------------------------------------------------------
// Our names, built once at init so the hot path does no string formatting.
// ---------------------------------------------------------------------------

// Two copies of each name we own: the original case is what goes ON THE WIRE
// (the instance name is what Dante Controller DISPLAYS, so lowercasing it would
// show "infernofpga-..." instead of "InfernoFPGA-..."), and a lowercased copy
// is what incoming queries are compared against, since name_decode() folds
// case and DNS matching is case-insensitive.
static char n_host[DANTE_MAX_NAME + 8];     // "<host>.local"
static char n_arc_svc[48];                  // "_netaudio-arc._udp.local"
static char n_cmc_svc[48];
static char n_chan_svc[48];                 // "_netaudio-chan._udp.local"
static char n_bund_svc[48];                 // "_netaudio-bund._udp.local"
static char n_arc_inst[DANTE_MAX_NAME + 48];
static char n_cmc_inst[DANTE_MAX_NAME + 48];
static const char n_services[] = "_services._dns-sd._udp.local";

static char lcq_host[DANTE_MAX_NAME + 8];
static char lcq_arc_inst[DANTE_MAX_NAME + 48];
static char lcq_cmc_inst[DANTE_MAX_NAME + 48];

static void cat2(char *dst, int max, const char *a, const char *b)
{
    int n = 0;
    while (*a && n < max - 1) dst[n++] = *a++;
    while (*b && n < max - 1) dst[n++] = *b++;
    dst[n] = 0;
}

// ---------------------------------------------------------------------------
// TXT records
//
// Values follow real hardware. Note "mf" is truncated to 8 bytes there
// ("Focusrite" -> "Focusrit"), so keep ours within that too.
// Each string is length-prefixed in the record.
// ---------------------------------------------------------------------------

static uint32_t txt_put(uint8_t *p, uint32_t n, const char *s)
{
    uint32_t l = 0;
    while (s[l]) l++;
    if (l > 255) l = 255;
    p[n++] = (uint8_t)l;
    for (uint32_t i = 0; i < l; i++) p[n++] = (uint8_t)s[i];
    return n;
}

static uint32_t txt_put_hexid(uint8_t *p, uint32_t n)
{
    static const char hx[] = "0123456789abcdef";
    char buf[8 + 16 + 1];
    int  o = 0;
    const char *pre = "id=";
    while (*pre) buf[o++] = *pre++;
    for (int i = 0; i < 8; i++) {
        buf[o++] = hx[g_dante.device_id[i] >> 4];
        buf[o++] = hx[g_dante.device_id[i] & 0xF];
    }
    buf[o] = 0;
    return txt_put(p, n, buf);
}

static uint32_t build_txt_arc(uint8_t *p)
{
    uint32_t n = 0;
    n = txt_put(p, n, "arcp_vers=2.8.9");
    n = txt_put(p, n, "arcp_min=0.2.4");
    n = txt_put(p, n, "router_vers=4.4.0");
    n = txt_put(p, n, "router_info=N-Series AoIP");   // "Dante Model" in DC
    n = txt_put(p, n, "mf=N-Series");
    n = txt_put(p, n, "model=_00000000000000ff");
    return n;
}

static uint32_t build_txt_cmc(uint8_t *p)
{
    uint32_t n = 0;
    n = txt_put_hexid(p, n);
    n = txt_put(p, n, "process=0");
    n = txt_put(p, n, "cmcp_vers=1.2.0");
    n = txt_put(p, n, "cmcp_min=1.0.0");
    n = txt_put(p, n, "server_vers=4.1.0");
    // channels= : USE THE KNOWN-GOOD CONSTANT. Do not invent one.
    //
    // This was 0x60000130, "chosen to look like a 48-channel transmitter" with
    // the note "refine if Dante Controller objects". It objected -- by never
    // sending us a single packet. Measured 2026-08-05: across 75 s of capture,
    // a click on our device in Device View, and a subscription pushed to our RX
    // channels, Dante Controller sent us NOTHING. No ARC, no CMC, no
    // subscription command. It displays us and never speaks to us, which is why
    // Subscription Status is blank, TX bandwidth reads 0 and Latency Status is
    // grey -- three symptoms, one cause.
    //
    // The bit pattern says why the invented value was a bad idea:
    //
    //     AM2   0x6000004d   low bits 0100 1101   bits 0,2,3,6
    //     A16R  0x6000017f   low bits 1 0111 1111 bits 0-6,8
    //     ours  0x60000130   low bits 1 0011 0000 bits 4,5,8
    //
    // Every real device sets 0x4d; ours shared NOT ONE bit with it. Whatever
    // those bits mean -- and nobody has decoded them -- asserting none of them
    // is not a plausible device description.
    //
    // inferno hardcodes the AM2's exact value with a literal "// ???"
    // (mdns_server.rs:78) rather than compute one, for the same reason. Follow
    // that: a constant copied from working hardware beats a guess that parses
    // into something no device would say.
    n = txt_put(p, n, "channels=0x6000004d");
    n = txt_put(p, n, "mf=N-Series");
    n = txt_put(p, n, "model=_00000000000000ff");
    return n;
}

// ---------------------------------------------------------------------------
// Record emission
// ---------------------------------------------------------------------------

static uint32_t put_rr_head(uint8_t *p, uint32_t n, const char *name,
                            uint16_t type, uint16_t cls, uint32_t ttl)
{
    n = name_put(p, n, name);
    p[n++] = (uint8_t)(type >> 8); p[n++] = (uint8_t)type;
    p[n++] = (uint8_t)(cls  >> 8); p[n++] = (uint8_t)cls;
    p[n++] = (uint8_t)(ttl >> 24); p[n++] = (uint8_t)(ttl >> 16);
    p[n++] = (uint8_t)(ttl >> 8);  p[n++] = (uint8_t)ttl;
    return n;
}

static uint32_t put_a(uint8_t *p, uint32_t n)
{
    n = put_rr_head(p, n, n_host, DNS_T_A, DNS_C_IN_FLUSH, MDNS_TTL_A);
    p[n++] = 0; p[n++] = 4;
    memcpy(p + n, g_net_ip, 4); n += 4;
    return n;
}

static uint32_t put_ptr(uint8_t *p, uint32_t n, const char *svc, const char *inst)
{
    n = put_rr_head(p, n, svc, DNS_T_PTR, DNS_C_IN, MDNS_TTL);
    uint32_t lp = n; n += 2;                       // rdlength placeholder
    uint32_t nn = name_put(p, n, inst);
    uint32_t l  = nn - n;
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n + l;
}

static uint32_t put_srv(uint8_t *p, uint32_t n, const char *inst, uint16_t port)
{
    n = put_rr_head(p, n, inst, DNS_T_SRV, DNS_C_IN_FLUSH, MDNS_TTL_A);  // SRV 120, like A
    uint32_t lp = n; n += 2;
    uint32_t s0 = n;
    p[n++] = 0; p[n++] = 0;                        // priority
    p[n++] = 0; p[n++] = 0;                        // weight
    p[n++] = (uint8_t)(port >> 8); p[n++] = (uint8_t)port;
    n = name_put(p, n, n_host);
    uint32_t l = n - s0;
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n;
}

static uint32_t put_txt(uint8_t *p, uint32_t n, const char *inst, int is_arc)
{
    n = put_rr_head(p, n, inst, DNS_T_TXT, DNS_C_IN_FLUSH, MDNS_TTL_TXT);
    uint32_t lp = n; n += 2;
    uint32_t l  = is_arc ? build_txt_arc(p + n) : build_txt_cmc(p + n);
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n + l;
}


// ---------------------------------------------------------------------------
// Channel and bundle records
//
// THE SUBSCRIBE PATH. A receiver resolving one of our channels does:
//   1. query <chname>@<host>._netaudio-chan._udp for TXT
//   2. find a key starting "b." -- b.<bundle>=<position>
//   3. query <bundle>@<host>._netaudio-bund._udp for TXT
//   4. read a.0 / p.0 and JOIN that multicast group
// It never contacts us. Everything above is mDNS, which is why the transmit
// side needs no flow-control server at all.
//
// Format taken byte for byte from a RedNet A16R on this bench
// (captures/netaudio_chan_A16R.txt, netaudio_bund.txt), not from the plan's
// summary of it -- the two differ, e.g. the chan record advertises fpp=8,2
// (the unicast range) while the BUNDLE advertises the fpp actually used, 16.
//
// Records are SYNTHESISED per query rather than stored: 48 channel records
// differ only in instance name, id= and b.N=, so building them on demand costs
// a scratch buffer instead of ~15 KB of blobs.
// ---------------------------------------------------------------------------

#define N_BUNDLES   (DANTE_TX_CHANNELS / 8)

// Soft cap on one response. 48 channel PTRs would overflow a datagram, and an
// mDNS response must not be fragmented.
#define MDNS_SPLIT_BYTES  900

// Instance-name buffer. This must hold the FULL service instance name --
// "<channel>@<device>._netaudio-chan._udp.local" -- which is up to 32 + 1 + 31
// + 26 + 1 = 91 bytes. The old buffers were DANTE_MAX_NAME + 48 = 80, sized
// when these names were built WITHOUT the service suffix.
#define MDNS_INST_MAX     112

// Rotating cursor for the channel PTR flood; see W_CHAN_PTR below.
static unsigned chan_ptr_start;

// The FULL service instance name: "<label>@<devicename>._netaudio-chan._udp.local".
//
// The service suffix used to be missing here, and that single omission is why
// no receiver could ever subscribe to us.
//
// It was invisible because the two halves of the code disagreed. The query
// MATCHERS (match_chan_inst) appended the suffix before comparing, so an
// incoming query for "1@Dev._netaudio-chan._udp.local" matched and we replied.
// But EMISSION used this bare name for the PTR rdata and for the SRV and TXT
// owner names, so the reply announced records belonging to "1@Dev." -- a
// root-level name that answers nobody's question. A resolver discards it and
// reports "cannot find this channel on the network".
//
// Measured against the A16R for the same browse query:
//     A16R   PTR dlen=16  -> 01@RedNetA16R._netaudio-chan._udp.local
//     ours   PTR dlen=23  -> 1@N-Series-Switchover            <- bare, at root
//
// The suffix now also gives name compression something to bite on: it is
// already in the packet as the question/owner name, so it costs 2 bytes.
static void inst_name(char *dst, int max, const char *label, const char *svc)
{
    int n = 0;
    while (*label && n < max - 1) dst[n++] = *label++;
    if (n < max - 1) dst[n++] = '@';
    const char *d = g_dante.name;
    while (*d && n < max - 1) dst[n++] = *d++;
    if (n < max - 1) dst[n++] = '.';
    while (*svc && n < max - 1) dst[n++] = *svc++;
    dst[n] = 0;
}

static void chan_inst(char *dst, int max, unsigned ch1)   // ch1 = 1..48
{
    char label[DANTE_MAX_NAME];
    dante_tx_channel_name((uint16_t)ch1, label, sizeof(label));
    inst_name(dst, max, label, n_chan_svc);
}

// b1 here is the DANTE CONTROLLER flow id, not a 1..6 bundle index, so the
// record a receiver resolves from b.<flow>= matches what DC created.
static void bund_inst(char *dst, int max, unsigned b1)
{
    char label[8];
    int n = 0;
    if (b1 >= 100) label[n++] = (char)('0' + (b1 / 100) % 10);
    if (b1 >= 10)  label[n++] = (char)('0' + (b1 / 10) % 10);
    label[n++] = (char)('0' + b1 % 10);
    label[n] = 0;
    inst_name(dst, max, label, n_bund_svc);
}

static uint32_t txt_put_kv_u(uint8_t *p, uint32_t n, const char *k, uint32_t v)
{
    char buf[40]; int i = 0;
    while (*k && i < 30) buf[i++] = *k++;
    char num[12]; int j = 0;
    if (!v) num[j++] = '0';
    while (v) { num[j++] = (char)('0' + v % 10); v /= 10; }
    while (j) buf[i++] = num[--j];
    buf[i] = 0;
    return txt_put(p, n, buf);
}

static uint32_t build_txt_chan(uint8_t *p, unsigned ch1)
{
    unsigned b1  = (ch1 - 1) / 8 + 1;          // bundle, 1-based
    unsigned pos = (ch1 - 1) % 8 + 1;          // position within it, 1-based
    uint32_t n = 0;
    n = txt_put(p, n, "txtvers=2");
    n = txt_put(p, n, "dbcp1=0x1200");
    n = txt_put_kv_u(p, n, "id=", ch1);
    n = txt_put(p, n, "dbcp=0x1004");
    n = txt_put(p, n, "rate=48000");
    n = txt_put(p, n, "en=24");
    n = txt_put(p, n, "pcm=3 4");
    n = txt_put(p, n, "enc=24");
    // latency_ns MUST COVER OUR OWN PACKET WINDOW.
    //
    // A receiver takes max(this, its own minimum) as the playout latency --
    // inferno channels_subscriber.rs:807, reading the value back from this TXT
    // key via mdns_client.rs:288. So advertising too little tells a receiver it
    // may buffer less than our packets can possibly arrive in.
    //
    // The emitted timestamp labels the OLDEST sample of the window a packet
    // covers, so the packet cannot exist until fpp samples after its own
    // timestamp. At the largest fpp we now accept (60, which is what Dante
    // Virtual Soundcard asks for) that is 60/48000 = 1.25 ms -- two and a half
    // times the 500 us this used to claim. The old value was written when fpp
    // could only be 8 or 16 (0.17 / 0.33 ms) and was never revisited.
    //
    // 2 ms covers fpp=60 with margin for network transit. inferno flags its own
    // use of this field with "FIXME should be tx latency"; this is that.
    n = txt_put_kv_u(p, n, "latency_ns=", g_latency_ns);
    // fpp=<MAX>,<MIN>, per inferno mdns_server.rs:120
    // (format!("fpp={},{}", FPP_MAX_ADVERTISED, FPP_MIN) = 32,2).
    //
    // REVERTED TO 8 (2026-08-05). Advertising 16 is more truthful --
    // dante_flows.c accepts 8 or 16 -- and it was changed for exactly that
    // reason. But it made a RedNet A16R renegotiate all four of its flows from
    // fpp=8 to fpp=16, and the audio went bad in the room at that moment.
    //
    // Everything measurable stayed identical across the change: rate discipline
    // armed, phase term off, drift -31 samples, ring 61..68, underrun 0/s, PTP
    // locked at -167 ns, one anchor, one enable, 18002 pps for 6 fpp=16 flows,
    // and the multicast stream on the wire byte-perfect (no gaps, no dups,
    // fpp-aligned, -5 dBFS of real audio). The only variable that moved was the
    // receivers' fpp, and the only instrument that detected it was the operator.
    //
    // The change also bought nothing: the A16R still requests 4 channels per
    // flow either way, which was the reason for trying it. So this is a revert
    // of a correctness nicety that cost audio and returned nothing.
    //
    // If it is ever raised again, note that with every flow at fpp=16 they all
    // fall due on tick_hi together -- six packets in one burst every 333 us
    // instead of spread across every tick, which is worth ruling out before
    // blaming the receivers.
    n = txt_put(p, n, "fpp=8,2");
    // nchan is the channels in a FLOW, not the device's channel count.
    // inferno: MAX_CHANNELS_IN_FLOW.min(tx_channels.len()) -> 8. We were
    // advertising 48, which would have a receiver negotiate a 48-channel flow
    // against a device whose flows carry 8.
    n = txt_put_kv_u(p, n, "nchan=", 8);
    // b.<flow>=<pos> ONLY WHEN A MULTICAST FLOW ACTUALLY CARRIES THIS CHANNEL.
    //
    // mdns_client.rs takes the multicast path the moment ANY key starts with
    // "b.", so this key must appear exactly when there is a real group to join
    // and never otherwise. Advertising it unconditionally is what forced every
    // subscription to multicast and put 65.5 Mbit/s of unwanted audio on the
    // segment; omitting it entirely is why patching a channel that IS in a
    // multicast flow still produced a unicast request.
    //
    // The id is the flow id Dante Controller assigned, so the bundle record a
    // receiver resolves next is named the same thing DC shows in its UI.
    {
        uint16_t bid; uint8_t bpos;
        if (dante_tx_chan_bundle((uint16_t)ch1, &bid, &bpos)) {
            char kv[24]; int i = 0;
            kv[i++] = 'b'; kv[i++] = '.';
            if (bid >= 10) kv[i++] = (char)('0' + (bid / 10) % 10);
            kv[i++] = (char)('0' + bid % 10);
            kv[i++] = '=';
            kv[i++] = (char)('0' + bpos);
            kv[i] = 0;
            n = txt_put(p, n, kv);
        }
    }

    // Historical note -- this key used to be emitted unconditionally:
    //
    // mdns_client.rs takes the multicast path the moment ANY TXT key starts
    // with "b.", so advertising it forces every subscription to multicast.
    // That was costing 65.5 Mbit/s of the 69.6 Mbit/s measured on the segment
    // -- 94% of all traffic -- because we transmitted all six bundles
    // unconditionally whether or not anyone had subscribed, and an unmanaged
    // switch floods every one of them to every port. The A16R was receiving
    // 65 Mbit/s it never asked for while playing two channels.
    //
    // Real Dante defaults to unicast; multicast flows are created explicitly
    // (opcode 0x2201, DC's "add a flow"). Without this key a receiver resolves
    // the channel, finds no bundle, and connects to the SRV target -- our flow
    // control server on 4455 -- naming exactly the channels and fpp it wants.
    n = txt_put(p, n, "at2");
    (void)b1; (void)pos;
    return n;
}

// b1 is Dante Controller's flow id. The group address and channel count come
// from the flow that is actually transmitting, not from a fixed bundle table --
// a multicast flow can now carry any 1..8 channels, so nchan varies.
static uint32_t build_txt_bund(uint8_t *p, unsigned b1)
{
    uint8_t ipbuf[4] = {0,0,0,0}; uint8_t ns = 8;
    dante_tx_mcast_by_id((uint16_t)b1, ipbuf, &ns);
    const uint8_t *ip = ipbuf;
    uint32_t n = 0;
    n = txt_put(p, n, "txtvers=1");
    n = txt_put_kv_u(p, n, "id=", b1);
    n = txt_put_kv_u(p, n, "nchan=", ns);
    // SAME variable as the chan record. These two disagreed (2 ms vs 1 ms) and
    // Dante Controller reads THIS one, which is why the offered latency choices
    // started at 1 ms while the chan record claimed 2.
    n = txt_put_kv_u(p, n, "latency_ns=", g_latency_ns);
    // GENERATED FROM THE SAME VARIABLE THE PACKETIZER USES. This was hardcoded
    // "fpp=16" while dante_tx bound the context from g_mcast_fpp, so the two
    // could disagree -- and did: a debug sweep left the transmit fpp at 60 while
    // this still said 16, which shows as a GREEN subscription with NO AUDIO
    // (the flow resolves, the payload is in a format the receiver is not
    // expecting). Two places holding one fact is the bug; this removes one.
    n = txt_put_kv_u(p, n, "fpp=", g_mcast_fpp);
    n = txt_put(p, n, "rate=48000");
    n = txt_put(p, n, "enc=24");
    n = txt_put(p, n, "at2");
    {
        char kv[32]; int i = 0;
        const char *k = "a.0=";
        while (*k) kv[i++] = *k++;
        for (int o = 0; o < 4; o++) {
            uint8_t v = ip[o];
            if (v >= 100) kv[i++] = (char)('0' + v / 100);
            if (v >= 10)  kv[i++] = (char)('0' + (v / 10) % 10);
            kv[i++] = (char)('0' + v % 10);
            if (o < 3) kv[i++] = '.';
        }
        kv[i] = 0;
        n = txt_put(p, n, kv);
    }
    n = txt_put(p, n, "p.0=4321");
    return n;
}

// Query names arrive lowercased by the sender's convention but our device name
// keeps its original case, so compare case-insensitively -- the same reason the
// arc/cmc instances keep separate lcq_* copies.
static int ieq(const char *a, const char *b)
{
    while (*a && lc(*a) == lc(*b)) { a++; b++; }
    return lc(*a) == lc(*b);
}

// Both matchers MUST check the service suffix first. Without it a bundle query
// falls into the channel loop and is answered with a channel record -- observed
// on the wire: a query for 1@<host>._netaudio-bund._udp.local came back as the
// CHAN record for index 2, which is exactly the "cannot find this channel"
// a receiver reports when it tries to resolve b.N= to a group.
static int has_suffix(const char *q, const char *suf)
{
    const char *e = q;
    while (*e) e++;
    const char *t = suf;
    while (*t) t++;
    if ((e - q) < (t - suf)) return 0;
    const char *a = e - (t - suf);
    return ieq(a, suf);
}

static int match_chan_inst(const char *q, unsigned *idx)
{
    if (!has_suffix(q, "._netaudio-chan._udp.local")) return 0;
    // chan_inst now yields the full name, so compare against it directly. The
    // old cat2() here is exactly what hid the emission bug: matching built the
    // full name, emission did not.
    char inst[MDNS_INST_MAX];
    for (unsigned c = 1; c <= DANTE_TX_CHANNELS; c++) {
        chan_inst(inst, sizeof(inst), c);
        if (ieq(q, inst)) { *idx = c; return 1; }
    }
    return 0;
}

static int match_bund_inst(const char *q, unsigned *idx)
{
    if (!has_suffix(q, "._netaudio-bund._udp.local")) return 0;
    // Only ACTIVE multicast flows have bundle records now.
    char inst[MDNS_INST_MAX];
    uint16_t id;
    for (unsigned i = 0; dante_tx_mcast_enum(i, &id); i++) {
        bund_inst(inst, sizeof(inst), id);
        if (ieq(q, inst)) { *idx = id; return 1; }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Response assembly
// ---------------------------------------------------------------------------

// Which of our records a query matched. Bitmask so one response can carry
// everything a browser asked for.
#define W_ARC_PTR   (1u << 0)
#define W_ARC_SRV   (1u << 1)
#define W_ARC_TXT   (1u << 2)
#define W_CMC_PTR   (1u << 3)
#define W_CMC_SRV   (1u << 4)
#define W_CMC_TXT   (1u << 5)
#define W_A         (1u << 6)
#define W_SERVICES  (1u << 7)
#define W_CHAN_PTR  (1u << 8)      // all 48 channel PTRs
#define W_BUND_PTR  (1u << 9)      // all 6 bundle PTRs
// Single-instance answers carry their index alongside the mask.
#define W_CHAN_ONE  (1u << 10)
#define W_BUND_ONE  (1u << 11)
static unsigned want_idx;          // 1-based channel or bundle for W_*_ONE

// The question to echo back, captured from the query. Real devices DO echo it:
// a working A16R channel reply carries qdcount=1, while ours carried 0. mDNS
// responses usually omit the question, but we are matching what actually
// resolves on this network rather than what the RFC permits.
static const uint8_t *echo_q;
static uint32_t       echo_q_len;

static void send_response(uint32_t want)
{
    if (!want) return;

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    uint16_t answers = 0;

    nc_reset();                           // compression offsets are per-message
    p[0] = 0; p[1] = 0;                   // transaction id: 0 for mDNS
    p[2] = 0x84; p[3] = 0x00;             // response + authoritative
    p[4] = 0; p[5] = 0;                   // qdcount, patched below
    p[6] = 0; p[7] = 0;                   // ancount, patched below
    p[8] = 0; p[9] = 0;                   // nscount
    p[10] = 0; p[11] = 0;                 // arcount, patched below
    n = 12;

    // NO question echo. Added on a misread of an earlier capture and REVERTED:
    // a working A16R reply that DVS accepted has qd=0. The qd=1 sample that
    // prompted the change was a different response type.

    if (want & W_SERVICES) { n = put_ptr(p, n, n_services, n_arc_svc); answers++;
                             n = put_ptr(p, n, n_services, n_cmc_svc); answers++; }
    if (want & W_ARC_PTR)  { n = put_ptr(p, n, n_arc_svc, n_arc_inst); answers++; }
    if (want & W_ARC_SRV)  { n = put_srv(p, n, n_arc_inst, DANTE_PORT_ARC); answers++; }
    if (want & W_ARC_TXT)  { n = put_txt(p, n, n_arc_inst, 1); answers++; }
    if (want & W_CMC_PTR)  { n = put_ptr(p, n, n_cmc_svc, n_cmc_inst); answers++; }
    if (want & W_CMC_SRV)  { n = put_srv(p, n, n_cmc_inst, DANTE_PORT_CMC); answers++; }
    if (want & W_CMC_TXT)  { n = put_txt(p, n, n_cmc_inst, 0); answers++; }
    // The A record goes in ADDITIONAL, not ANSWERS. The A16R answers a channel
    // query with 2 answers (SRV + TXT) and 1 additional (A); we were putting
    // all three in ANSWERS, and a resolver that expects the address as
    // supporting data can reject the record it came with -- which reads on the
    // far side as "cannot find this channel on the network".
    uint16_t additional = 0;
    uint16_t chan_txt_additional = 0;

    // Channel / bundle records, synthesised on demand.
    //
    // PTR floods are SPLIT: 48 channel PTRs do not fit one datagram alongside
    // anything else, and mDNS responses must not be fragmented. We answer a
    // bounded batch per response and let the browser re-query; avahi and DC
    // both do, and a truncated-but-valid response beats an oversized one.
    if (want & W_CHAN_PTR) {
        // ROTATE the starting channel. One response holds ~24 PTRs, so a fixed
        // start meant channels 1..24 were announced on every browse and 25..48
        // on none of them -- avahi listed exactly 24 of our 48. The A16R
        // answers a rotating subset for the same reason (09..18, then 03..08,
        // then 01..02) and browsers accumulate across repeated queries.
        char inst[MDNS_INST_MAX];
        for (unsigned i = 0; i < DANTE_TX_CHANNELS && n < MDNS_SPLIT_BYTES; i++) {
            unsigned c = (chan_ptr_start + i) % DANTE_TX_CHANNELS + 1;
            chan_inst(inst, sizeof(inst), c);
            n = put_ptr(p, n, n_chan_svc, inst); answers++;
            chan_ptr_start = c;                  // resume past the last one sent
        }
    }
    if (want & W_BUND_PTR) {
        char inst[MDNS_INST_MAX];
        uint16_t bid;
        for (unsigned i = 0; dante_tx_mcast_enum(i, &bid); i++) {
            bund_inst(inst, sizeof(inst), bid);
            n = put_ptr(p, n, n_bund_svc, inst); answers++;
        }
    }
    if (want & W_CHAN_ONE) {
        // SRV is the ANSWER; TXT is ADDITIONAL. A reply DVS accepted from the
        // A16R carries an=1, ar=3 -- only the SRV answers the question, and the
        // TXT and A records ride along as supporting data. We had both in
        // answers (an=2), and a resolver looking for the TXT among additionals
        // will not find it there.
        char inst[MDNS_INST_MAX];
        chan_inst(inst, sizeof(inst), want_idx);
        n = put_srv(p, n, inst, DANTE_PORT_FLOWS); answers++;   // SRV port 4455
        n = put_rr_head(p, n, inst, DNS_T_TXT, DNS_C_IN_FLUSH, MDNS_TTL_TXT);
        { uint32_t lp = n; n += 2;
          uint32_t l = build_txt_chan(p + n, want_idx);
          p[lp] = (uint8_t)(l >> 8); p[lp+1] = (uint8_t)l; n += l; }
        chan_txt_additional++;
    }
    if (want & W_BUND_ONE) {
        char inst[MDNS_INST_MAX];
        bund_inst(inst, sizeof(inst), want_idx);
        n = put_srv(p, n, inst, DANTE_PORT_MEDIA); answers++;   // SRV port 4321
        n = put_rr_head(p, n, inst, DNS_T_TXT, DNS_C_IN_FLUSH, MDNS_TTL_TXT);
        { uint32_t lp = n; n += 2;
          uint32_t l = build_txt_bund(p + n, want_idx);
          p[lp] = (uint8_t)(l >> 8); p[lp+1] = (uint8_t)l; n += l; }
        chan_txt_additional++;
    }

    if (want & W_A) { n = put_a(p, n); additional++; }

    p[6]  = (uint8_t)(answers >> 8);    p[7]  = (uint8_t)answers;
    additional += chan_txt_additional;
    p[10] = (uint8_t)(additional >> 8); p[11] = (uint8_t)additional;

    if (net_udp_commit(mdns_group, MDNS_PORT, MDNS_PORT, n, NET_TOS_BEST_EFFORT) == 0)
        g_mdns_stats.tx_responses++;
}

// ---------------------------------------------------------------------------
// Query handling
// ---------------------------------------------------------------------------

static int streq(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return *a == *b;
}

static void mdns_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                    uint16_t src_port, const uint8_t *msg, uint32_t len)
{
    (void)src_ip; (void)dst_ip; (void)src_port;
    if (len < 12) return;

    uint16_t flags   = (uint16_t)((msg[2] << 8) | msg[3]);
    uint16_t qdcount = (uint16_t)((msg[4] << 8) | msg[5]);
    if (flags & 0x8000) return;                    // a response, not a query
    if (!qdcount) return;

    g_mdns_stats.rx_queries++;

    uint32_t off  = 12;
    uint32_t want = 0;
    char     qname[128];
    echo_q = 0; echo_q_len = 0;

    for (uint16_t q = 0; q < qdcount && off < len; q++) {
        off = name_decode(msg, len, off, qname, sizeof(qname));
        if (!off || off + 4 > len) return;
        uint16_t qtype = (uint16_t)((msg[off] << 8) | msg[off + 1]);
        uint32_t qstart = 0;
        { /* the whole question record, for the echo */
            uint32_t nlen = off - 12;
            (void)nlen; qstart = 12;
        }
        off += 4;                                   // qtype + qclass
        if (!echo_q_len) { echo_q = msg + qstart; echo_q_len = off - qstart; }

        int any = (qtype == DNS_T_ANY);

        if (streq(qname, n_services)) {
            want |= W_SERVICES;
        } else if (streq(qname, n_arc_svc)) {
            if (any || qtype == DNS_T_PTR)
                want |= W_ARC_PTR | W_ARC_SRV | W_ARC_TXT | W_A;
        } else if (streq(qname, n_cmc_svc)) {
            if (any || qtype == DNS_T_PTR)
                want |= W_CMC_PTR | W_CMC_SRV | W_CMC_TXT | W_A;
        } else if (streq(qname, lcq_arc_inst)) {
            if (any || qtype == DNS_T_SRV) want |= W_ARC_SRV | W_A;
            if (any || qtype == DNS_T_TXT) want |= W_ARC_TXT;
        } else if (streq(qname, lcq_cmc_inst)) {
            if (any || qtype == DNS_T_SRV) want |= W_CMC_SRV | W_A;
            if (any || qtype == DNS_T_TXT) want |= W_CMC_TXT;
        } else if (streq(qname, n_chan_svc)) {
            if (any || qtype == DNS_T_PTR) want |= W_CHAN_PTR | W_A;
        } else if (streq(qname, n_bund_svc)) {
            if (any || qtype == DNS_T_PTR) want |= W_BUND_PTR | W_A;
        } else if (match_chan_inst(qname, &want_idx)) {
            if (any || qtype == DNS_T_SRV || qtype == DNS_T_TXT)
                want |= W_CHAN_ONE | W_A;
        } else if (match_bund_inst(qname, &want_idx)) {
            if (any || qtype == DNS_T_SRV || qtype == DNS_T_TXT)
                want |= W_BUND_ONE | W_A;
        } else if (streq(qname, lcq_host)) {
            if (any || qtype == DNS_T_A) want |= W_A;
        } else {
            g_mdns_stats.rx_ignored++;
        }
    }

    send_response(want);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void mdns_announce(void)
{
    announce_left    = 3;
    announce_next_ms = 0;                          // fire on the next poll
}

void mdns_poll(void)
{
    if (!announce_left) return;

    uint32_t now = gptp_uptime_ms();
    if (announce_next_ms && (int32_t)(now - announce_next_ms) < 0) return;

    // Unsolicited announcement: everything we have, so a controller that is
    // already running notices us without having to query.
    send_response(W_ARC_PTR | W_ARC_SRV | W_ARC_TXT |
                  W_CMC_PTR | W_CMC_SRV | W_CMC_TXT | W_A);
    g_mdns_stats.tx_announce++;

    announce_left--;
    announce_next_ms = now + 1000;
}

void mdns_init(void)
{
    cat2(n_host, sizeof(n_host), g_dante.hostname, ".local");
    cat2(n_arc_svc, sizeof(n_arc_svc), "_netaudio-arc._udp", ".local");
    cat2(n_cmc_svc, sizeof(n_cmc_svc), "_netaudio-cmc._udp", ".local");
    cat2(n_chan_svc, sizeof(n_chan_svc), "_netaudio-chan._udp", ".local");
    cat2(n_bund_svc, sizeof(n_bund_svc), "_netaudio-bund._udp", ".local");
    cat2(n_arc_inst, sizeof(n_arc_inst), g_dante.name, "._netaudio-arc._udp.local");
    cat2(n_cmc_inst, sizeof(n_cmc_inst), g_dante.name, "._netaudio-cmc._udp.local");

    // Lowercased copies FOR COMPARISON ONLY. The originals keep their case
    // because the instance name is the display name in Dante Controller.
    memcpy(lcq_host,     n_host,     sizeof(lcq_host));
    memcpy(lcq_arc_inst, n_arc_inst, sizeof(lcq_arc_inst));
    memcpy(lcq_cmc_inst, n_cmc_inst, sizeof(lcq_cmc_inst));
    for (char *s = lcq_host;     *s; s++) *s = lc(*s);
    for (char *s = lcq_arc_inst; *s; s++) *s = lc(*s);
    for (char *s = lcq_cmc_inst; *s; s++) *s = lc(*s);

    if (net_udp_bind(MDNS_PORT, mdns_rx) != 0)
        printf("[net] BIND FAILED on port %u -- udp table full\n", MDNS_PORT);
    net_igmp_join(mdns_group);
    mdns_announce();

    printf("[mdns] %s -> %u.%u.%u.%u  arc:%u cmc:%u\n",
           n_host, g_net_ip[0], g_net_ip[1], g_net_ip[2], g_net_ip[3],
           DANTE_PORT_ARC, DANTE_PORT_CMC);
}
