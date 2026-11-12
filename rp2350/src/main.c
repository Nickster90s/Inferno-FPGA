// RP2350 side of the KVM link: USB keyboard + mouse in, 12-byte frames out.
//
//   USB-A  (PIO-USB host, GPIO12/13)  keyboard, mouse, hub or combo dongle
//   UART0  GPIO0 TX -> FPGA W20       A5 5A id p0..p7 xor, 1 Mbaud 8N1
//          GPIO1 RX <- FPGA AB22      the FPGA's log text, passed to the console
//   Type-C (native USB device)        CDC console: status + the FPGA's log
//
// The wire format and the reason for every byte of it are in the FPGA's
// firmware/kvm.c; the reason frames are PACED rather than forwarded is in
// kvm_sched.c. Which host gets the reports (Main / Backup) is decided on the
// FPGA by OSC and is invisible from here.
//
// TWO CORES, AND THE SPLIT IS ABOUT TIMING. Core 0 runs both USB stacks;
// Pico-PIO-USB does its bus transactions inside a 1 ms timer interrupt there.
// Core 1 does nothing but put a frame on the UART at exact 1 ms boundaries.
// kvm_sched.c's whole scheme rests on a motion frame standing for ONE poll
// period, and a tick that shares a core with a bit-banged USB host would be
// late by however long the last transaction took.

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "pio_usb.h"
#include "tusb.h"

#include "hid_parse.h"
#include "kvm_sched.h"

#define LINK_UART       uart0
#define LINK_TX_PIN     0           /* pad 1 -> SODIMM 161 -> FPGA W20  (its RX) */
#define LINK_RX_PIN     1           /* pad 2 <- SODIMM 159 <- FPGA AB22 (its TX) */
#define LINK_BAUD       1000000     /* avb_soc.py: the one UART, 1 Mbaud */

#define USB_A_DP_PIN    12          /* D+ ; D- is the next pin, 13 */

static kvm_sched_t        g_sched;
static critical_section_t g_lock;   /* core 0 produces, core 1 consumes */

// ---------------------------------------------------------------------------
// Console (CDC on the Type-C)
// ---------------------------------------------------------------------------
// Never blocks and never waits for a reader: with nothing plugged into the
// Type-C -- the normal case -- every write is simply dropped.
static void con_write(const void *p, uint32_t n)
{
    if (!tud_cdc_connected()) return;
    uint32_t room = tud_cdc_write_available();
    if (n > room) n = room;
    if (n) tud_cdc_write(p, n);
}

static void con_printf(const char *fmt, ...)
{
    char b[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof b - 1) n = sizeof b - 1;
    con_write(b, (uint32_t)n);
    tud_cdc_write_flush();
}

// ---------------------------------------------------------------------------
// Core 1: the 1 ms frame clock
// ---------------------------------------------------------------------------
static void core1_main(void)
{
    absolute_time_t next = make_timeout_time_ms(1);
    for (;;) {
        busy_wait_until(next);
        next = delayed_by_us(next, 1000);

        uint8_t f[KVM_FRAME_BYTES];
        critical_section_enter_blocking(&g_lock);
        bool send = kvm_sched_tick(&g_sched, f);
        critical_section_exit(&g_lock);

        // 12 bytes into a 32-byte FIFO that was emptied 880 us ago: this
        // cannot block, so the frame starts within microseconds of the tick.
        if (send) uart_write_blocking(LINK_UART, f, sizeof f);
    }
}

// ---------------------------------------------------------------------------
// USB host: HID interfaces
// ---------------------------------------------------------------------------
enum { ROLE_NONE = 0, ROLE_KEYBOARD, ROLE_MOUSE_BOOT, ROLE_MOUSE_REPORT,
       ROLE_MOUSE_ASK,        /* wants report protocol, not requested yet */
       ROLE_MOUSE_ASKED };    /* requested, waiting for the answer        */

typedef struct {
    uint8_t            dev, idx;
    uint8_t            role;
    hid_mouse_layout_t mouse;
} hid_slot_t;

static hid_slot_t g_slots[CFG_TUH_HID];

static hid_slot_t *slot_find(uint8_t dev, uint8_t idx, bool create)
{
    hid_slot_t *free_slot = NULL;
    for (unsigned i = 0; i < CFG_TUH_HID; i++) {
        hid_slot_t *s = &g_slots[i];
        if (s->role != ROLE_NONE && s->dev == dev && s->idx == idx) return s;
        if (s->role == ROLE_NONE && !free_slot) free_slot = s;
    }
    if (!create || !free_slot) return NULL;
    memset(free_slot, 0, sizeof *free_slot);
    free_slot->dev = dev;
    free_slot->idx = idx;
    return free_slot;
}

static void push_keyboard(const uint8_t r[8])
{
    critical_section_enter_blocking(&g_lock);
    kvm_sched_keyboard(&g_sched, r);
    critical_section_exit(&g_lock);
}

static void push_mouse(uint8_t buttons, bool valid, int32_t dx, int32_t dy, int32_t dw)
{
    critical_section_enter_blocking(&g_lock);
    kvm_sched_mouse(&g_sched, buttons, valid, dx, dy, dw);
    critical_section_exit(&g_lock);
}

static void start_rx(uint8_t dev, uint8_t idx)
{
    if (!tuh_hid_receive_report(dev, idx))
        con_printf("[rp] %u/%u: receive_report failed\r\n", dev, idx);
}

void tuh_hid_mount_cb(uint8_t dev, uint8_t idx, uint8_t const *desc, uint16_t len)
{
    uint8_t     proto = tuh_hid_interface_protocol(dev, idx);
    hid_slot_t *s = slot_find(dev, idx, true);
    if (!s) { con_printf("[rp] %u/%u: no free slot\r\n", dev, idx); return; }

    if (proto == HID_ITF_PROTOCOL_KEYBOARD) {
        // TinyUSB has already put it in BOOT protocol (its default), which is
        // byte for byte the payload the FPGA's report ID 1 declares.
        s->role = ROLE_KEYBOARD;
        con_printf("[rp] %u/%u: keyboard (boot protocol)\r\n", dev, idx);
        start_rx(dev, idx);
        return;
    }

    bool parsed = desc && len && hid_parse_mouse(desc, len, &s->mouse);

    if (proto == HID_ITF_PROTOCOL_MOUSE) {
        // Boot mouse: three bytes, no wheel. If we know the layout, ask for
        // report protocol -- but NOT FROM HERE. This callback runs in the
        // middle of enumeration, and the stack's next step is a control
        // request for the device's NEXT interface; a request of ours in
        // flight on the same pipe would make that one fail, which on a combo
        // dongle (keyboard + mouse in one device) loses an interface.
        // protocol_task() sends it once the whole device is configured.
        if (parsed) { s->role = ROLE_MOUSE_ASK; return; }
        s->role = ROLE_MOUSE_BOOT;
        con_printf("[rp] %u/%u: mouse (boot protocol, no wheel)\r\n", dev, idx);
        start_rx(dev, idx);
        return;
    }

    // Not a boot interface. Combo dongles put a mouse here, behind report IDs,
    // next to consumer keys we have no report for.
    if (parsed) {
        s->role = ROLE_MOUSE_REPORT;
        con_printf("[rp] %u/%u: mouse (report id %u, x%u y%u w%u)\r\n", dev, idx,
                   s->mouse.report_id, s->mouse.x.size, s->mouse.y.size,
                   s->mouse.wheel.size);
        start_rx(dev, idx);
        return;
    }
    s->role = ROLE_NONE;
    con_printf("[rp] %u/%u: HID interface ignored (%u byte descriptor)\r\n",
               dev, idx, len);
}

void tuh_hid_set_protocol_complete_cb(uint8_t dev, uint8_t idx, uint8_t protocol)
{
    hid_slot_t *s = slot_find(dev, idx, false);
    if (!s || s->role != ROLE_MOUSE_ASKED) return;
    // Ask the stack what the device is in NOW rather than trusting `protocol`:
    // a mouse that stalls the request stays in boot protocol.
    (void)protocol;
    if (tuh_hid_get_protocol(dev, idx) == HID_PROTOCOL_REPORT) {
        s->role = ROLE_MOUSE_REPORT;
        con_printf("[rp] %u/%u: mouse (report id %u, x%u y%u w%u)\r\n", dev, idx,
                   s->mouse.report_id, s->mouse.x.size, s->mouse.y.size,
                   s->mouse.wheel.size);
    } else {
        s->role = ROLE_MOUSE_BOOT;
        con_printf("[rp] %u/%u: mouse refused report protocol, no wheel\r\n", dev, idx);
    }
    start_rx(dev, idx);
}

void tuh_hid_umount_cb(uint8_t dev, uint8_t idx)
{
    hid_slot_t *s = slot_find(dev, idx, false);
    if (!s) return;
    // THE STUCK KEY, ONE HOP UPSTREAM. A keyboard pulled out with a key down
    // never sends the release, and the FPGA's register would hold that key on
    // the host for ever. Same for a mouse button.
    if (s->role == ROLE_KEYBOARD) {
        static const uint8_t released[8] = { 0 };
        push_keyboard(released);
    } else if (s->role != ROLE_NONE) {
        push_mouse(0, true, 0, 0, 0);
    }
    s->role = ROLE_NONE;
    con_printf("[rp] %u/%u: gone\r\n", dev, idx);
}

void tuh_hid_report_received_cb(uint8_t dev, uint8_t idx, uint8_t const *r, uint16_t len)
{
    hid_slot_t *s = slot_find(dev, idx, false);
    if (!s) return;

    switch (s->role) {
    case ROLE_KEYBOARD:
        if (len >= 8) push_keyboard(r);
        break;
    case ROLE_MOUSE_BOOT:
        if (len >= 3)
            push_mouse(r[0], true, (int8_t)r[1], (int8_t)r[2],
                       len >= 4 ? (int8_t)r[3] : 0);
        break;
    case ROLE_MOUSE_REPORT: {
        hid_mouse_event_t ev;
        if (hid_mouse_decode(&s->mouse, r, len, &ev))
            push_mouse(ev.buttons, ev.buttons_valid, ev.dx, ev.dy, ev.dw);
        break;
    }
    default:
        break;
    }
    start_rx(dev, idx);
}

static void protocol_task(void)
{
    for (unsigned i = 0; i < CFG_TUH_HID; i++) {
        hid_slot_t *s = &g_slots[i];
        if (s->role != ROLE_MOUSE_ASK || !tuh_mounted(s->dev)) continue;
        // False = the control pipe is busy; try again on the next pass.
        if (tuh_hid_set_protocol(s->dev, s->idx, HID_PROTOCOL_REPORT))
            s->role = ROLE_MOUSE_ASKED;
    }
}

// ---------------------------------------------------------------------------
// Core 0 housekeeping
// ---------------------------------------------------------------------------
// What the FPGA prints (log.h, DANTE_VERBOSE builds) comes in on GPIO1. Since
// the UART left the CH347 this is the only place that text can be read.
static void fpga_log_task(void)
{
    uint8_t b[64];
    uint32_t n = 0;
    while (n < sizeof b && uart_is_readable(LINK_UART))
        b[n++] = (uint8_t)uart_getc(LINK_UART);
    if (n) { con_write(b, n); tud_cdc_write_flush(); }
}

static void status_task(void)
{
    static absolute_time_t next;
    if (!time_reached(next)) return;
    next = make_timeout_time_ms(2000);

    // Swallow whatever was typed: nothing here takes commands, and the FPGA
    // reads its RX as HID frames only, so keystrokes must NOT be forwarded.
    if (tud_cdc_available()) tud_cdc_read_flush();

    critical_section_enter_blocking(&g_lock);
    kvm_sched_t c = g_sched;
    critical_section_exit(&g_lock);
    con_printf("[rp] frames kbd %lu mouse %lu neutral %lu, kbd coalesced %lu\r\n",
               (unsigned long)c.frames_kbd, (unsigned long)c.frames_mouse,
               (unsigned long)c.frames_neutral, (unsigned long)c.kbd_coalesced);
}

// TinyUSB's host stack times its enumeration delays with this. The examples
// get it from their board support package, which this project does not use.
uint32_t tusb_time_millis_api(void)
{
    return to_ms_since_boot(get_absolute_time());
}

int main(void)
{
    // Pico-PIO-USB bit-bangs 12 Mbit/s and needs a system clock that is a
    // multiple of 12 MHz. The RP2350's default 150 MHz is not.
    set_sys_clock_khz(120000, true);

    uart_init(LINK_UART, LINK_BAUD);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    gpio_set_function(LINK_TX_PIN, UART_FUNCSEL_NUM(LINK_UART, LINK_TX_PIN));
    gpio_set_function(LINK_RX_PIN, UART_FUNCSEL_NUM(LINK_UART, LINK_RX_PIN));
    // An unconfigured FPGA floats its TX; without this the RX reads noise.
    gpio_pull_up(LINK_RX_PIN);

    kvm_sched_init(&g_sched);
    critical_section_init(&g_lock);

    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = USB_A_DP_PIN;
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUH_RHPORT, &host_init);

    multicore_launch_core1(core1_main);

    for (;;) {
        tud_task();
        tuh_task();
        protocol_task();
        fpga_log_task();
        status_task();
    }
}
