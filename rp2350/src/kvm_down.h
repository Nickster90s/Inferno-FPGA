#ifndef KVM_DOWN_H
#define KVM_DOWN_H

#include <stdbool.h>
#include <stdint.h>

/* FPGA -> RP2350: the other direction of the link. Pure C, no SDK:
 * test/test_host.c runs it on the build machine.
 *
 * Same 12-byte frame as the reports going up (firmware/kvm.c):
 *
 *     A5 5A | id | 8 payload | xor(id..payload)
 *
 * IDs 0x80 and up travel FPGA -> RP. The one that exists:
 *
 *     0x80 STATUS   p0 = target: 0 Main, 1 Backup   (KVM_DEST_* in kvm.h)
 *                   p1 = format version, 1
 *                   p2..p5 = switch count, little-endian
 *                   p6, p7 = 0
 *
 * sent on every switch and once a second. Once a second because either end can
 * restart on its own and the other must not need to notice: an RP2350 that has
 * just booted knows the target within a second, and one that hears nothing for
 * KVM_DOWN_STALE_MS knows the FPGA is not there.
 *
 * THE SAME WIRE CARRIES THE FPGA'S LOG TEXT (log.h, DANTE_VERBOSE builds), so
 * the parser hands back every byte that turned out not to be part of a frame,
 * and the caller prints those. That is why the framing has two sync bytes and
 * a checksum in this direction too. */

#define KVM_DOWN_ID_STATUS   0x80
#define KVM_TARGET_MAIN      0
#define KVM_TARGET_BACKUP    1
#define KVM_TARGET_UNKNOWN   0xFF
#define KVM_DOWN_STALE_MS    3500

typedef struct {
    uint8_t  buf[12];
    uint8_t  len;

    uint8_t  target;            /* KVM_TARGET_*; UNKNOWN until a STATUS arrives */
    uint32_t switches;          /* the FPGA's own count, from the last STATUS   */
    uint32_t last_ms;           /* when that STATUS arrived                     */

    uint32_t frames_ok, frames_bad, frames_unknown;
} kvm_down_t;

void kvm_down_init(kvm_down_t *d);

/* Feed one received byte. Returns how many bytes were written to `text` (0..12)
 * -- bytes that are NOT part of a frame, i.e. the FPGA's log. Sets *status when
 * this byte completed a valid STATUS frame. */
int  kvm_down_feed(kvm_down_t *d, uint8_t b, uint32_t now_ms,
                   uint8_t text[12], bool *status);

/* The target, or KVM_TARGET_UNKNOWN if no STATUS arrived within the stale time. */
uint8_t kvm_down_target(const kvm_down_t *d, uint32_t now_ms);

#endif
