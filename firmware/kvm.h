#ifndef DANTE_KVM_H
#define DANTE_KVM_H

#include <stdint.h>

/* KVM: HID reports arrive from an RP2350 over the design's ONE UART (on the
 * SODIMM, AB22/W20) and are written to ONE PHY's report register at a time.
 * Both HID devices stay enumerated always -- switching never touches USB, it
 * only changes which register gets written, so the host sees no disconnect and
 * no re-enumeration. Measured 2026-11-11: four switches, USB device numbers
 * unchanged, all four input devices still present.
 *
 * See firmware/kvm.c for the wire format and for why switching sends release
 * reports to the PHY it leaves. */

#define KVM_DEST_MAIN    0
#define KVM_DEST_BACKUP  1

/* HID report: 1 ID byte + 8 payload, the same length for keyboard (ID 1) and
 * mouse (ID 2) -- the mouse is padded in the report descriptor because the
 * endpoint relays a fixed width. */
#define KVM_REPORT_BYTES 9

typedef struct {
    uint32_t bytes_rx;      /* bytes taken off the UART                      */
    uint32_t frames_ok;     /* frames accepted and written to a PHY          */
    uint32_t frames_bad;    /* checksum failures                             */
    uint32_t resyncs;       /* times the parser dropped a byte to re-find sync*/
    uint32_t switches;      /* kvm_select() calls that changed the target    */
    uint32_t overrun;       /* UART RX FIFO was full when polled             */
    uint32_t status_tx;     /* STATUS frames sent to the RP2350              */
} kvm_stats_t;

extern kvm_stats_t g_kvm_stats;

void    kvm_init(void);
void    kvm_poll(void);
void    kvm_select(uint8_t dest);     /* KVM_DEST_MAIN / KVM_DEST_BACKUP */
uint8_t kvm_dest(void);

#endif
