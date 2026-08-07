// Dante device identity — Phase 3. See dante_dev.h.

#include "dante_dev.h"
#include <string.h>
#include <stdio.h>

dante_dev_t g_dante;

// Default 2 ms: the value the chan record already advertised, chosen to cover
// fpp=60. Starting from the LOWER (1 ms) bundle value would have quietly told
// every receiver it may buffer less than a DVS flow can possibly arrive in.
uint32_t g_latency_ns = 2000000;

static int append_str(char *dst, int pos, int maxlen, const char *s)
{
    while (*s && pos < maxlen - 1) dst[pos++] = *s++;
    dst[pos] = 0;
    return pos;
}

static int append_hex2(char *dst, int pos, int maxlen, uint8_t v)
{
    static const char hx[] = "0123456789abcdef";
    if (pos < maxlen - 2) {
        dst[pos++] = hx[v >> 4];
        dst[pos++] = hx[v & 0xF];
    }
    dst[pos] = 0;
    return pos;
}

void dante_dev_init(const uint8_t mac[6])
{
    memcpy(g_dante.mac, mac, 6);

    // EUI-64: MAC with ff:fe inserted in the middle. Confirmed against the
    // RedNet AM2, whose mDNS id= and info-multicast factory_device_id are both
    // 001dc1fffea1723c for MAC 00:1d:c1:a1:72:3c.
    g_dante.device_id[0] = mac[0];
    g_dante.device_id[1] = mac[1];
    g_dante.device_id[2] = mac[2];
    g_dante.device_id[3] = 0xFF;
    g_dante.device_id[4] = 0xFE;
    g_dante.device_id[5] = mac[3];
    g_dante.device_id[6] = mac[4];
    g_dante.device_id[7] = mac[5];

    // Name and hostname carry the low 3 MAC bytes so two boards on one network
    // do not collide. Real devices do the same (RN-A16R2-2d4a18).
    //
    // KEEP THE HOSTNAME SHORT. device_info.rs:31 notes Dante Controller ignores
    // devices whose name exceeds 31 characters.
    int p = 0;
    // Dante device names double as the mDNS hostname, so they must be a valid
    // DNS label: letters, digits and hyphens only. The space in "N-Series
    // Switchover" becomes a hyphen -- a hostname cannot contain a space and
    // Dante Controller rejects such names. Every real device on this bench
    // follows the same rule (RF04-RedNetAM2-RFtech).
    //
    // The MAC suffix is deliberately gone, by request. The trade-off: two of
    // these boards on one network would now present the same name and hostname
    // and collide in mDNS. Fine for a single bench unit; add the suffix back if
    // a second board ever appears.
    p = append_str(g_dante.name, p, DANTE_MAX_NAME, "N-Series-Switchover");

    memcpy(g_dante.hostname, g_dante.name, DANTE_MAX_NAME);

    g_dante.process_id     = 0;
    g_dante.sample_rate    = 48000;
    g_dante.bits_per_sample = 24;       // pcm=3 on the wire

    printf("[dante] %s  id=%02x%02x%02x%02x%02x%02x%02x%02x  %luch tx @%lu\n",
           g_dante.name,
           g_dante.device_id[0], g_dante.device_id[1],
           g_dante.device_id[2], g_dante.device_id[3],
           g_dante.device_id[4], g_dante.device_id[5],
           g_dante.device_id[6], g_dante.device_id[7],
           (unsigned long)DANTE_TX_CHANNELS,
           (unsigned long)g_dante.sample_rate);
}

int dante_tx_channel_name(uint16_t index_1based, char *buf, int maxlen)
{
    if (maxlen < 4) { if (maxlen > 0) buf[0] = 0; return 0; }
    int n = 0;
    if (index_1based >= 10) buf[n++] = (char)('0' + (index_1based / 10) % 10);
    buf[n++] = (char)('0' + index_1based % 10);
    buf[n] = 0;
    return n;
}
