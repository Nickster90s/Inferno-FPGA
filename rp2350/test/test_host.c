// Runs on the build machine: cc -Isrc test/test_host.c src/kvm_sched.c src/hid_parse.c
//
// Checks the two pieces with no hardware in them against the two things they
// must agree with: the FPGA's frame parser (the checks in firmware/kvm.c's
// kvm_frame(), restated here) and real mouse report descriptors.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "hid_parse.h"
#include "kvm_sched.h"

// firmware/kvm.c, kvm_frame(): the acceptance test a frame faces on the FPGA.
static int fpga_accepts(const uint8_t *f)
{
    if (f[0] != 0xA5 || f[1] != 0x5A) return 0;
    uint8_t x = 0;
    for (int i = 2; i < 11; i++) x ^= f[i];
    if (x != f[11]) return 0;
    return f[2] == 1 || f[2] == 2;
}

// The FPGA register + a host polling it every tick: what the host ends up with.
typedef struct { int x, y, w; uint8_t buttons; uint8_t kbd[8]; int has; uint8_t reg[9]; } host_t;

static void host_poll(host_t *h)
{
    if (!h->has) return;
    if (h->reg[0] == 1) memcpy(h->kbd, h->reg + 1, 8);
    else { h->buttons = h->reg[1]; h->x += (int8_t)h->reg[2];
           h->y += (int8_t)h->reg[3]; h->w += (int8_t)h->reg[4]; }
}

static int run(kvm_sched_t *s, host_t *h, int ticks)
{
    int frames = 0;
    for (int t = 0; t < ticks; t++) {
        uint8_t f[KVM_FRAME_BYTES];
        if (kvm_sched_tick(s, f)) {
            assert(fpga_accepts(f));
            memcpy(h->reg, f + 2, 9);
            h->has = 1;
            frames++;
        }
        host_poll(h);           /* ideal host: exactly one poll per tick */
    }
    return frames;
}

static void test_sched(void)
{
    kvm_sched_t s; host_t h; memset(&h, 0, sizeof h);
    kvm_sched_init(&s);

    // Motion larger than one report: arrives whole, exactly once, then stops.
    kvm_sched_mouse(&s, 0, true, 300, -5, 1);
    run(&s, &h, 50);
    assert(h.x == 300 && h.y == -5 && h.w == 1);
    run(&s, &h, 500);                       /* register must rest harmless */
    assert(h.x == 300 && h.y == -5 && h.w == 1);

    // Press and release inside one tick: both must reach the host, in order,
    // each standing for more than one poll.
    uint8_t down[8] = { 0x02, 0, 0x04, 0, 0, 0, 0, 0 }, up[8] = { 0 };
    kvm_sched_keyboard(&s, down);
    kvm_sched_keyboard(&s, up);
    uint8_t f[KVM_FRAME_BYTES];
    assert(kvm_sched_tick(&s, f) && f[2] == 1 && f[3] == 0x02 && f[5] == 0x04);
    for (int i = 0; i < KVM_HOLD_TICKS; i++) assert(!kvm_sched_tick(&s, f));
    assert(kvm_sched_tick(&s, f) && f[2] == 1 && f[3] == 0 && f[5] == 0);

    // A key arriving while the mouse is moving is not lost and not delayed
    // behind the motion backlog; the motion still totals correctly.
    memset(&h, 0, sizeof h);
    kvm_sched_mouse(&s, 0, true, 1000, 0, 0);
    run(&s, &h, 3);
    kvm_sched_keyboard(&s, down);
    run(&s, &h, 3);
    assert(h.kbd[0] == 0x02 && h.kbd[2] == 0x04);
    run(&s, &h, 100);
    assert(h.x == 1000);

    // A click is its own held frame and survives.
    kvm_sched_mouse(&s, 1, true, 10, 0, 0);
    run(&s, &h, 2);
    assert(h.buttons == 1);
    kvm_sched_mouse(&s, 0, true, 0, 0, 0);
    run(&s, &h, 20);
    assert(h.buttons == 0 && h.x == 1010);

    // Queue overflow keeps the END state (nothing stuck).
    for (int i = 0; i < 40; i++) kvm_sched_keyboard(&s, (i & 1) ? up : down);
    run(&s, &h, 200);
    assert(h.kbd[0] == 0 && h.kbd[2] == 0 && s.kbd_coalesced > 0);

    // Key codes outside the FPGA descriptor's 0..0x65 are removed.
    uint8_t odd[8] = { 0, 0, 0x66, 0x65, 0xE0, 0, 0, 0 };
    kvm_sched_keyboard(&s, odd);
    run(&s, &h, 5);
    assert(h.kbd[2] == 0 && h.kbd[3] == 0x65 && h.kbd[4] == 0);
    puts("sched   ok");
}

// The FPGA's own mouse collection (usb_descriptors.py): ID 2, 8-bit axes.
static const uint8_t desc_fpga[] = {
    0x05,0x01, 0x09,0x06, 0xA1,0x01, 0x85,0x01, 0x05,0x07, 0x19,0xE0, 0x29,0xE7,
    0x15,0x00, 0x25,0x01, 0x75,0x01, 0x95,0x08, 0x81,0x02, 0x95,0x01, 0x75,0x08,
    0x81,0x01, 0x95,0x06, 0x75,0x08, 0x15,0x00, 0x25,0x65, 0x05,0x07, 0x19,0x00,
    0x29,0x65, 0x81,0x00, 0xC0,
    0x05,0x01, 0x09,0x02, 0xA1,0x01, 0x85,0x02, 0x09,0x01, 0xA1,0x00, 0x05,0x09,
    0x19,0x01, 0x29,0x03, 0x15,0x00, 0x25,0x01, 0x95,0x03, 0x75,0x01, 0x81,0x02,
    0x95,0x01, 0x75,0x05, 0x81,0x01, 0x05,0x01, 0x09,0x30, 0x09,0x31, 0x09,0x38,
    0x15,0x81, 0x25,0x7F, 0x75,0x08, 0x95,0x03, 0x81,0x06, 0x95,0x04, 0x75,0x08,
    0x81,0x01, 0xC0, 0xC0,
};

// A common gaming-mouse shape: no report ID, 5 buttons + 3 pad bits... here 16
// buttons, then 16-bit X/Y, 8-bit wheel, and an AC Pan on the consumer page.
static const uint8_t desc_16bit[] = {
    0x05,0x01, 0x09,0x02, 0xA1,0x01, 0x09,0x01, 0xA1,0x00,
    0x05,0x09, 0x19,0x01, 0x29,0x10, 0x15,0x00, 0x25,0x01, 0x95,0x10, 0x75,0x01,
    0x81,0x02,
    0x05,0x01, 0x16,0x01,0x80, 0x26,0xFF,0x7F, 0x75,0x10, 0x95,0x02,
    0x09,0x30, 0x09,0x31, 0x81,0x06,
    0x15,0x81, 0x25,0x7F, 0x75,0x08, 0x95,0x01, 0x09,0x38, 0x81,0x06,
    0x05,0x0C, 0x0A,0x38,0x02, 0x95,0x01, 0x81,0x06,
    0xC0, 0xC0,
};

// 12-bit packed X/Y (Logitech-style), with a report ID.
static const uint8_t desc_12bit[] = {
    0x05,0x01, 0x09,0x02, 0xA1,0x01, 0x85,0x02, 0x09,0x01, 0xA1,0x00,
    0x05,0x09, 0x19,0x01, 0x29,0x08, 0x15,0x00, 0x25,0x01, 0x95,0x08, 0x75,0x01,
    0x81,0x02,
    0x05,0x01, 0x16,0x01,0xF8, 0x26,0xFF,0x07, 0x75,0x0C, 0x95,0x02,
    0x09,0x30, 0x09,0x31, 0x81,0x06,
    0x15,0x81, 0x25,0x7F, 0x75,0x08, 0x95,0x01, 0x09,0x38, 0x81,0x06,
    0xC0, 0xC0,
};

// A joystick has X and Y too. It must NOT be taken for a mouse.
static const uint8_t desc_joy[] = {
    0x05,0x01, 0x09,0x04, 0xA1,0x01, 0x09,0x30, 0x09,0x31, 0x15,0x81, 0x25,0x7F,
    0x75,0x08, 0x95,0x02, 0x81,0x02, 0xC0,
};

static void test_parse(void)
{
    hid_mouse_layout_t l; hid_mouse_event_t ev;

    assert(hid_parse_mouse(desc_fpga, sizeof desc_fpga, &l));
    assert(l.report_id == 2 && l.buttons.bit == 0 && l.buttons.size == 3);
    assert(l.x.bit == 8 && l.y.bit == 16 && l.wheel.bit == 24 && l.x.size == 8);
    uint8_t r1[] = { 2, 0x05, 0xFB, 0x03, 0xFF, 0, 0, 0, 0 };
    assert(hid_mouse_decode(&l, r1, sizeof r1, &ev));
    assert(ev.buttons == 5 && ev.dx == -5 && ev.dy == 3 && ev.dw == -1);
    uint8_t rk[] = { 1, 0, 0, 4, 0, 0, 0, 0, 0 };       /* the keyboard's ID */
    assert(!hid_mouse_decode(&l, rk, sizeof rk, &ev));

    assert(hid_parse_mouse(desc_16bit, sizeof desc_16bit, &l));
    assert(l.report_id == 0 && l.buttons.size == 3);
    assert(l.x.bit == 16 && l.x.size == 16 && l.y.bit == 32 && l.wheel.bit == 48);
    uint8_t r2[] = { 0x02, 0x00, 0x2C, 0x01, 0x00, 0x80, 0xFE, 0x00 };
    assert(hid_mouse_decode(&l, r2, sizeof r2, &ev));
    assert(ev.buttons == 2 && ev.dx == 300 && ev.dy == -32768 && ev.dw == -2);
    assert(!hid_mouse_decode(&l, r2, 3, &ev));          /* truncated report */

    assert(hid_parse_mouse(desc_12bit, sizeof desc_12bit, &l));
    assert(l.report_id == 2 && l.x.bit == 8 && l.x.size == 12 && l.y.bit == 20);
    // x = -3 (0xFFD), y = +5 (0x005): bytes FD 5F 00
    uint8_t r3[] = { 2, 0x01, 0xFD, 0x5F, 0x00, 0x01 };
    assert(hid_mouse_decode(&l, r3, sizeof r3, &ev));
    assert(ev.buttons == 1 && ev.dx == -3 && ev.dy == 5 && ev.dw == 1);

    assert(!hid_parse_mouse(desc_joy, sizeof desc_joy, &l));

    // Truncated and garbage descriptors must terminate and say no.
    for (unsigned n = 0; n < sizeof desc_16bit; n++)
        hid_parse_mouse(desc_16bit, (uint16_t)n, &l);
    uint8_t junk[64];
    for (unsigned seed = 1; seed < 2000; seed++) {
        unsigned v = seed * 2654435761u;
        for (unsigned i = 0; i < sizeof junk; i++) { v = v * 1103515245u + 12345u; junk[i] = (uint8_t)(v >> 16); }
        if (hid_parse_mouse(junk, sizeof junk, &l)) {
            uint8_t rr[4] = { 1, 2, 3, 4 };
            hid_mouse_decode(&l, rr, sizeof rr, &ev);
        }
    }
    puts("parse   ok");
}

int main(void)
{
    test_sched();
    test_parse();
    return 0;
}
