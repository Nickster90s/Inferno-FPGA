// Minimal mDNS / DNS-SD responder — Dante Phase 3. See mdns.h.
//
// Modelled on captured traffic from a Focusrite RedNet AM2 and A16R, which
// answer a service query with SRV + TXT + A as answers plus the PTR, with the
// cache-flush bit set on the unique records. Nothing about Dante's use of mDNS
// turned out to be exotic -- it is ordinary DNS-SD (captures/README.md).

#include "mdns.h"
#include "net.h"
#include "dante_dev.h"
#include "gptp.h"
#include <string.h>
#include <stdio.h>

#define MDNS_PORT       5353
#define MDNS_TTL        4500        // seconds; matches what Dante devices use
#define MDNS_TTL_A      120

#define DNS_T_A         1
#define DNS_T_PTR       12
#define DNS_T_TXT       16
#define DNS_T_SRV       33
#define DNS_T_ANY       255

#define DNS_C_IN        1
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
    n = txt_put(p, n, "router_info=InfernoFPGA");
    n = txt_put(p, n, "mf=Inferno");
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
    // channels= is PER-DEVICE (AM2 0x6000004d vs A16R 0x6000017f), not the
    // constant an early capture suggested. Value chosen to look like a
    // 48-channel transmitter; refine if Dante Controller objects.
    n = txt_put(p, n, "channels=0x60000130");
    n = txt_put(p, n, "mf=Inferno");
    n = txt_put(p, n, "model=_00000000000000ff");
    return n;
}

// ---------------------------------------------------------------------------
// Record emission
// ---------------------------------------------------------------------------

static uint32_t put_rr_head(uint8_t *p, uint32_t n, const char *name,
                            uint16_t type, uint16_t cls, uint32_t ttl)
{
    n += name_encode(p + n, name);
    p[n++] = (uint8_t)(type >> 8); p[n++] = (uint8_t)type;
    p[n++] = (uint8_t)(cls  >> 8); p[n++] = (uint8_t)cls;
    p[n++] = (uint8_t)(ttl >> 24); p[n++] = (uint8_t)(ttl >> 16);
    p[n++] = (uint8_t)(ttl >> 8);  p[n++] = (uint8_t)ttl;
    return n;
}

static uint32_t put_a(uint8_t *p, uint32_t n)
{
    n = put_rr_head(p, n, n_host, DNS_T_A, DNS_C_IN | DNS_CACHE_FLUSH, MDNS_TTL_A);
    p[n++] = 0; p[n++] = 4;
    memcpy(p + n, g_net_ip, 4); n += 4;
    return n;
}

static uint32_t put_ptr(uint8_t *p, uint32_t n, const char *svc, const char *inst)
{
    n = put_rr_head(p, n, svc, DNS_T_PTR, DNS_C_IN, MDNS_TTL);
    uint32_t lp = n; n += 2;                       // rdlength placeholder
    uint32_t l  = name_encode(p + n, inst);
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n + l;
}

static uint32_t put_srv(uint8_t *p, uint32_t n, const char *inst, uint16_t port)
{
    n = put_rr_head(p, n, inst, DNS_T_SRV, DNS_C_IN | DNS_CACHE_FLUSH, MDNS_TTL);
    uint32_t lp = n; n += 2;
    uint32_t s0 = n;
    p[n++] = 0; p[n++] = 0;                        // priority
    p[n++] = 0; p[n++] = 0;                        // weight
    p[n++] = (uint8_t)(port >> 8); p[n++] = (uint8_t)port;
    n += name_encode(p + n, n_host);
    uint32_t l = n - s0;
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n;
}

static uint32_t put_txt(uint8_t *p, uint32_t n, const char *inst, int is_arc)
{
    n = put_rr_head(p, n, inst, DNS_T_TXT, DNS_C_IN | DNS_CACHE_FLUSH, MDNS_TTL);
    uint32_t lp = n; n += 2;
    uint32_t l  = is_arc ? build_txt_arc(p + n) : build_txt_cmc(p + n);
    p[lp] = (uint8_t)(l >> 8); p[lp + 1] = (uint8_t)l;
    return n + l;
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

static void send_response(uint32_t want)
{
    if (!want) return;

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    uint16_t answers = 0;

    p[0] = 0; p[1] = 0;                   // transaction id: 0 for mDNS
    p[2] = 0x84; p[3] = 0x00;             // response + authoritative
    p[4] = 0; p[5] = 0;                   // qdcount
    p[6] = 0; p[7] = 0;                   // ancount, patched below
    p[8] = 0; p[9] = 0;                   // nscount
    p[10] = 0; p[11] = 0;                 // arcount
    n = 12;

    if (want & W_SERVICES) { n = put_ptr(p, n, n_services, n_arc_svc); answers++;
                             n = put_ptr(p, n, n_services, n_cmc_svc); answers++; }
    if (want & W_ARC_PTR)  { n = put_ptr(p, n, n_arc_svc, n_arc_inst); answers++; }
    if (want & W_ARC_SRV)  { n = put_srv(p, n, n_arc_inst, DANTE_PORT_ARC); answers++; }
    if (want & W_ARC_TXT)  { n = put_txt(p, n, n_arc_inst, 1); answers++; }
    if (want & W_CMC_PTR)  { n = put_ptr(p, n, n_cmc_svc, n_cmc_inst); answers++; }
    if (want & W_CMC_SRV)  { n = put_srv(p, n, n_cmc_inst, DANTE_PORT_CMC); answers++; }
    if (want & W_CMC_TXT)  { n = put_txt(p, n, n_cmc_inst, 0); answers++; }
    if (want & W_A)        { n = put_a(p, n); answers++; }

    p[6] = (uint8_t)(answers >> 8); p[7] = (uint8_t)answers;

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

    for (uint16_t q = 0; q < qdcount && off < len; q++) {
        off = name_decode(msg, len, off, qname, sizeof(qname));
        if (!off || off + 4 > len) return;
        uint16_t qtype = (uint16_t)((msg[off] << 8) | msg[off + 1]);
        off += 4;                                   // qtype + qclass

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

    net_udp_bind(MDNS_PORT, mdns_rx);
    net_igmp_join(mdns_group);
    mdns_announce();

    printf("[mdns] %s -> %u.%u.%u.%u  arc:%u cmc:%u\n",
           n_host, g_net_ip[0], g_net_ip[1], g_net_ip[2], g_net_ip[3],
           DANTE_PORT_ARC, DANTE_PORT_CMC);
}
