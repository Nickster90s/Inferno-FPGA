// FPGA -> RP2350 frames, separated from the FPGA's log text. See kvm_down.h.

#include "kvm_down.h"
#include <string.h>

void kvm_down_init(kvm_down_t *d)
{
    memset(d, 0, sizeof *d);
    d->target = KVM_TARGET_UNKNOWN;
}

int kvm_down_feed(kvm_down_t *d, uint8_t b, uint32_t now_ms,
                  uint8_t text[12], bool *status)
{
    *status = false;

    if (d->len == 0) {
        if (b != 0xA5) { text[0] = b; return 1; }
        d->buf[d->len++] = b;
        return 0;
    }
    if (d->len == 1 && b != 0x5A) {
        // Not a frame start. The A5 we held back was text; this byte may be
        // the real A5, so look at it again rather than dropping both.
        text[0] = 0xA5;
        if (b == 0xA5) return 1;            /* keep len = 1 with the new A5 */
        d->len = 0;
        text[1] = b;
        return 2;
    }
    d->buf[d->len++] = b;
    if (d->len < 12) return 0;

    d->len = 0;
    uint8_t x = 0;
    for (int i = 2; i < 11; i++) x ^= d->buf[i];
    if (x != d->buf[11]) {
        // Twelve bytes that only looked like a frame: give them back as text.
        // A real frame that was hiding inside is lost; the next one is at
        // most a second away.
        d->frames_bad++;
        memcpy(text, d->buf, 12);
        return 12;
    }
    if (d->buf[2] == KVM_DOWN_ID_STATUS && d->buf[4] == 1) {
        d->target   = d->buf[3] ? KVM_TARGET_BACKUP : KVM_TARGET_MAIN;
        d->switches = (uint32_t)d->buf[5] | (uint32_t)d->buf[6] << 8 |
                      (uint32_t)d->buf[7] << 16 | (uint32_t)d->buf[8] << 24;
        d->last_ms  = now_ms;
        d->frames_ok++;
        *status = true;
    } else {
        // A well-formed frame this firmware does not know: a newer FPGA.
        // Swallow it -- it is certainly not text -- and count it.
        d->frames_unknown++;
    }
    return 0;
}

uint8_t kvm_down_target(const kvm_down_t *d, uint32_t now_ms)
{
    if (d->target == KVM_TARGET_UNKNOWN) return KVM_TARGET_UNKNOWN;
    if ((uint32_t)(now_ms - d->last_ms) > KVM_DOWN_STALE_MS) return KVM_TARGET_UNKNOWN;
    return d->target;
}
