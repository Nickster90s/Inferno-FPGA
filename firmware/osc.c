// OSC control: switch the USB A/B source (and the sample rate) from a show
// controller.
//
// The A/B select was already reachable over UDP 7779, but only through this
// project's own binary opcodes -- fine for a bench script, useless from QLab or
// any OSC-speaking desk. This binds a second port and speaks the subset of OSC
// that a switch needs.
//
// Addresses:
//     /usb/source   i|f   0 = Main, 1 = Backup
//     /usb/main     -     switch to Main   (argument ignored)
//     /usb/backup   -     switch to Backup (argument ignored)
//     /usb/rate     i|f   48000 or 96000
//     /ping         -     replies /pong, so a desk can prove it has the board
//
// WHY BOTH i AND f. Plenty of OSC controllers send every number as a float --
// TouchOSC and a lot of QLab cue setups do. Accepting only ",i" works on the
// bench and then silently ignores the desk, which is the worst way for this to
// fail. Both are parsed, and an unknown type tag is LOGGED rather than dropped.
//
// WHAT THIS DELIBERATELY DOES NOT DO: bundles (`#bundle`). They are detected and
// reported, not parsed. A single message covers a switch, and a half-supported
// bundle parser is a worse place to find a bug than an explicit "not supported"
// line in the console.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <generated/csr.h>

#include "net.h"
#include "config.h"
#include "dante_info.h"
#include "osc.h"

#define OSC_PORT 9000

// OSC pads every string to a 4-byte boundary, INCLUDING its null terminator.
// A 4-character address like "/abc" is therefore 8 bytes on the wire, not 5 --
// getting this wrong reads the type tag from inside the address and every
// message looks malformed.
static uint32_t osc_pad4(uint32_t n) { return (n + 4u) & ~3u; }

static uint32_t osc_str_len(const uint8_t *p, uint32_t avail)
{
    uint32_t i = 0;
    while (i < avail && p[i]) i++;
    if (i >= avail) return 0;              // unterminated: refuse
    return osc_pad4(i);
}

static int32_t osc_be32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8)  |  (uint32_t)p[3]);
}

// IEEE-754 float -> int, without pulling in soft-float: this CPU has no FPU and
// linking libm for one conversion is not worth it. Values a desk sends here are
// small integers expressed as floats (0.0, 1.0, 48000.0), so decode the exponent
// and mantissa directly and truncate.
static int32_t osc_f32_to_int(const uint8_t *p)
{
    uint32_t b = (uint32_t)osc_be32(p);
    int      e = (int)((b >> 23) & 0xFF) - 127;
    uint32_t m = (b & 0x7FFFFF) | 0x800000;        // implicit leading 1
    int32_t  v;
    if (e < 0)        v = 0;                        // |x| < 1 -> 0
    else if (e > 30)  v = 0x7FFFFFFF;               // absurd, clamp
    else if (e >= 23) v = (int32_t)(m << (e - 23));
    else              v = (int32_t)(m >> (23 - e));
    return (b & 0x80000000u) ? -v : v;
}

static void osc_reply(const uint8_t ip[4], uint16_t port, const char *addr)
{
    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    uint32_t l = (uint32_t)strlen(addr);
    memcpy(p, addr, l); n = l;
    while (n & 3u) p[n++] = 0;
    if (n == l) { p[n++] = 0; while (n & 3u) p[n++] = 0; }   // need >=1 null
    p[n++] = ','; while (n & 3u) p[n++] = 0;                 // empty type tag
    net_udp_commit(ip, port, OSC_PORT, n, NET_TOS_BEST_EFFORT);
}

static void osc_set_source(uint8_t which)
{
#ifdef CSR_MAIN_USB_SOURCE_SELECT_ADDR
    main_usb_source_select_write(which ? 1 : 0);
    printf("[osc] source -> %s\n", which ? "Backup(USB2)" : "Main(USB1)");
#else
    (void)which;
    printf("[osc] source ignored: this bitstream has no 2nd USB (USB2=1)\n");
#endif
}

// SIGNATUUR: VIJF PARAMETERS, dst_ip hoort erbij. Een versie hiervan had er
// vier (dst_ip vergeten) en dat compileert met alleen een
// incompatible-pointer-waarschuwing -- die ik wegfilterde uit de build-output.
// Bij de aanroep schuift dan alles op: `msg` krijgt het src_port-GETAL als
// pointer, en de eerste dereference crasht de CPU. Symptoom: het bord
// antwoordt op niets meer zodra er een pakket op poort 9000 binnenkomt.
static void osc_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                   uint16_t src_port, const uint8_t *msg, uint32_t len)
{
    (void)dst_ip;
    if (len < 8) return;

    // memcmp() is not in this bare-metal libc subset (strcmp/memcpy/strlen are),
    // so compare by hand rather than pull in a dependency for seven bytes.
    int is_bundle = 1;
    for (uint32_t i = 0; i < 7; i++)
        if (msg[i] != (uint8_t)"#bundle"[i]) { is_bundle = 0; break; }
    if (is_bundle) {
        // See the header comment: reported, not parsed.
        printf("[osc] #bundle not supported; send a single message\n");
        return;
    }

    uint32_t alen = osc_str_len(msg, len);
    if (!alen || alen >= len) return;
    const char *addr = (const char *)msg;

    // Arguments, if any. A message with no type tag is legal and some desks
    // send exactly that for a trigger, so treat a missing tag as "no argument"
    // rather than as an error.
    int32_t  argv   = 0;
    int      have   = 0;
    uint32_t o      = alen;
    if (o < len && msg[o] == ',') {
        uint32_t tlen = osc_str_len(msg + o, len - o);
        if (tlen) {
            char t = (char)msg[o + 1];
            uint32_t ao = o + tlen;
            if ((t == 'i' || t == 'f') && ao + 4 <= len) {
                argv = (t == 'i') ? osc_be32(msg + ao)
                                  : osc_f32_to_int(msg + ao);
                have = 1;
            } else if (t && t != 'i' && t != 'f') {
                printf("[osc] %s: unhandled type tag '%c'\n", addr, t);
            }
        }
    }

    if (strcmp(addr, "/usb/source") == 0) {
        if (!have) { printf("[osc] /usb/source needs an argument\n"); return; }
        osc_set_source(argv ? 1 : 0);
    } else if (strcmp(addr, "/usb/main") == 0) {
        osc_set_source(0);
    } else if (strcmp(addr, "/usb/backup") == 0) {
        osc_set_source(1);
    } else if (strcmp(addr, "/usb/rate") == 0) {
        if (!have) { printf("[osc] /usb/rate needs 48000 or 96000\n"); return; }
        if (argv != 48000 && argv != 96000) {
            printf("[osc] rate %ld refused; 48000 or 96000\n", (long)argv);
            return;
        }
        if ((uint32_t)argv != g_fs) {
            g_cfg.sample_rate = (uint32_t)argv;
            cfg_save();
            dante_info_rate_apply((uint32_t)argv);
            printf("[osc] rate -> %ld Hz (live; replug USB so the host "
                   "re-reads it)\n", (long)argv);
        }
    } else if (strcmp(addr, "/ping") == 0) {
        osc_reply(src_ip, src_port, "/pong");
    } else {
        // Log unknown addresses. A desk with a typo'd address is otherwise
        // indistinguishable from a dead board, and that is a bad afternoon.
        printf("[osc] unknown address '%s'\n", addr);
    }
}

void osc_init(void)
{
    if (net_udp_bind(OSC_PORT, osc_rx) != 0) {
        printf("[osc] BIND FAILED on %u\n", OSC_PORT);
        return;
    }
    printf("[osc] listening on udp %u  (/usb/source, /usb/main, /usb/backup, "
           "/usb/rate, /ping)\n", OSC_PORT);
}
