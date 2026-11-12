#ifndef KVM_SCHED_H
#define KVM_SCHED_H

#include <stdbool.h>
#include <stdint.h>

/* The scheduler between "a HID device said something" and "12 bytes go to the
 * FPGA". Pure C, no SDK: test/test_host.c runs it on the build machine.
 *
 * See kvm_sched.c for why this exists at all -- the FPGA's endpoint relays
 * STATE, so a relative mouse report cannot simply be forwarded. */

#define KVM_FRAME_BYTES   12      /* A5 5A id p0..p7 xor -- firmware/kvm.c   */
#define KVM_ID_KEYBOARD   1
#define KVM_ID_MOUSE      2

/* How many extra 1 ms ticks a report that MUST arrive stays in the FPGA's
 * register before anything may overwrite it. 1 means two ticks = 2 ms, against
 * a 1 ms host poll. Raise it if the FPGA's main loop turns out to stall longer
 * than that (it drains the UART in bursts, see README). */
#ifndef KVM_HOLD_TICKS
#define KVM_HOLD_TICKS    1
#endif

#define KVM_KBD_QUEUE     16      /* keyboard reports waiting for a slot     */

typedef struct {
    /* keyboard: a FIFO of 8-byte boot reports, oldest first */
    uint8_t  kq[KVM_KBD_QUEUE][8];
    uint8_t  kq_head, kq_len;

    /* mouse: buttons are state, motion is accumulated until a slot is free */
    uint8_t  buttons;
    bool     buttons_dirty;
    int32_t  dx, dy, dw;

    uint8_t  hold;                /* ticks left before the register may change */
    bool     need_neutral;        /* last frame carried motion: stop it next   */

    /* counters, for the status line */
    uint32_t frames_kbd, frames_mouse, frames_neutral, kbd_coalesced;
} kvm_sched_t;

void kvm_sched_init(kvm_sched_t *s);

/* Producers (USB side). report = modifiers, reserved, 6 key codes. */
void kvm_sched_keyboard(kvm_sched_t *s, const uint8_t report[8]);
void kvm_sched_mouse(kvm_sched_t *s, uint8_t buttons, bool buttons_valid,
                     int32_t dx, int32_t dy, int32_t dw);

/* Consumer: call exactly once per millisecond. Returns true and fills `frame`
 * when a frame must go out on this tick. */
bool kvm_sched_tick(kvm_sched_t *s, uint8_t frame[KVM_FRAME_BYTES]);

#endif
