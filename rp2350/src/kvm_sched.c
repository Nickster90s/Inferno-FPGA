// What goes to the FPGA, and when.
//
// THE FPGA'S HID ENDPOINT RELAYS STATE, NOT EVENTS. It is a
// USBSignalInEndpoint: whatever is in the 72-bit report register is sent to the
// host on EVERY 1 ms poll until something overwrites it (avb_soc.py, "The
// endpoint relays STATE"; firmware/kvm.c, "the stuck-key problem"). There is
// one register for both report IDs. Three consequences, and this file is the
// answer to all three:
//
//  1. A RELATIVE MOUSE REPORT REPEATS. Leave "dx = +5" in the register for
//     20 ms and the host moves the pointer 100 counts, not 5. So a motion
//     frame is followed, exactly one tick later, by something that carries no
//     motion. One poll period of exposure = on average one delivery.
//
//  2. A REPORT OVERWRITTEN BEFORE THE HOST POLLED NEVER HAPPENED. A key press
//     followed 300 us later by a mouse report would be lost. So anything that
//     must arrive -- every keyboard report, every button change -- is held for
//     KVM_HOLD_TICKS extra ticks before the register may change again.
//
//  3. THE REGISTER RESTS ON SOMETHING HARMLESS. A keyboard report and a
//     zero-motion mouse report are both idempotent: the host can read them a
//     thousand times and nothing changes. Only a motion frame is not, and it
//     is never left standing.
//
// WHAT THIS CANNOT FIX. "One poll period of exposure" is one delivery only on
// average. The host's poll clock and this tick are not synchronised, and the
// FPGA's firmware drains its UART ring from a main loop it shares with the PTP
// servo, so the real exposure jitters: now and then a motion frame is seen
// twice or not at all. Motion is therefore approximate; keys and buttons are
// not, because they are held. The exact fix is in gateware (send each written
// report once, NAK otherwise) and is deliberately not attempted from here.
//
// Priority per tick: stop motion > keyboard > button change > motion.

#include "kvm_sched.h"
#include <string.h>

void kvm_sched_init(kvm_sched_t *s)
{
    memset(s, 0, sizeof *s);
}

void kvm_sched_keyboard(kvm_sched_t *s, const uint8_t report[8])
{
    uint8_t slot;
    if (s->kq_len == KVM_KBD_QUEUE) {
        // Full: overwrite the NEWEST entry. The newest report is the current
        // state of the keyboard, so replacing it keeps the end state right
        // (nothing stays stuck); dropping the oldest would not be wrong
        // either, dropping the incoming one would be.
        slot = (uint8_t)((s->kq_head + s->kq_len - 1) % KVM_KBD_QUEUE);
        s->kbd_coalesced++;
    } else {
        slot = (uint8_t)((s->kq_head + s->kq_len) % KVM_KBD_QUEUE);
        s->kq_len++;
    }
    memcpy(s->kq[slot], report, 8);
    // The FPGA's descriptor declares key codes 0..0x65. A host discards an
    // out-of-range array entry anyway; zeroing it here keeps the wire honest.
    for (int i = 2; i < 8; i++)
        if (s->kq[slot][i] > 0x65) s->kq[slot][i] = 0;
}

void kvm_sched_mouse(kvm_sched_t *s, uint8_t buttons, bool buttons_valid,
                     int32_t dx, int32_t dy, int32_t dw)
{
    if (buttons_valid) {
        buttons &= 0x07;                    /* the descriptor has 3 buttons */
        if (buttons != s->buttons) {
            s->buttons = buttons;
            s->buttons_dirty = true;
        }
    }
    s->dx += dx;
    s->dy += dy;
    s->dw += dw;
}

static void frame_build(uint8_t f[KVM_FRAME_BYTES], uint8_t id, const uint8_t p[8])
{
    f[0] = 0xA5;
    f[1] = 0x5A;
    f[2] = id;
    memcpy(f + 3, p, 8);
    uint8_t x = 0;
    for (int i = 2; i < 11; i++) x ^= f[i];
    f[11] = x;
}

static int8_t take(int32_t *acc)
{
    int32_t v = *acc;
    if (v > 127)  v = 127;
    if (v < -127) v = -127;                 /* descriptor minimum is -127 */
    *acc -= v;
    return (int8_t)v;
}

static void mouse_frame(const kvm_sched_t *s, uint8_t f[KVM_FRAME_BYTES],
                        int8_t x, int8_t y, int8_t w)
{
    // buttons, X, Y, wheel, then the 4 pad bytes the descriptor declares.
    uint8_t p[8] = { s->buttons, (uint8_t)x, (uint8_t)y, (uint8_t)w, 0, 0, 0, 0 };
    frame_build(f, KVM_ID_MOUSE, p);
}

static bool kbd_pop(kvm_sched_t *s, uint8_t f[KVM_FRAME_BYTES])
{
    if (!s->kq_len) return false;
    frame_build(f, KVM_ID_KEYBOARD, s->kq[s->kq_head]);
    s->kq_head = (uint8_t)((s->kq_head + 1) % KVM_KBD_QUEUE);
    s->kq_len--;
    s->hold = KVM_HOLD_TICKS;
    s->frames_kbd++;
    return true;
}

bool kvm_sched_tick(kvm_sched_t *s, uint8_t f[KVM_FRAME_BYTES])
{
    if (s->hold) { s->hold--; return false; }

    if (s->need_neutral) {
        // A motion frame has been standing for one tick. End it NOW, with
        // whatever harmless thing is most useful.
        s->need_neutral = false;
        if (kbd_pop(s, f)) return true;
        mouse_frame(s, f, 0, 0, 0);
        s->frames_neutral++;
        return true;
    }

    if (kbd_pop(s, f)) return true;

    if (s->buttons_dirty) {
        // A click travels alone and is held: a motion frame can be missed by
        // the host, and a button change must not ride on something missable.
        s->buttons_dirty = false;
        mouse_frame(s, f, 0, 0, 0);
        s->hold = KVM_HOLD_TICKS;
        s->frames_mouse++;
        return true;
    }

    if (s->dx || s->dy || s->dw) {
        int8_t x = take(&s->dx), y = take(&s->dy), w = take(&s->dw);
        mouse_frame(s, f, x, y, w);
        s->need_neutral = true;
        s->frames_mouse++;
        return true;
    }
    return false;
}
