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
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#if KVM_USB_DEBUG
#include "hardware/flash.h"
#include "pico/flash.h"
#endif
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

#if !KVM_NATIVE_HOST
#include "pio_usb.h"
#endif
#include "tusb.h"

#include "hid_parse.h"
#include "kvm_sched.h"
#include "kvm_down.h"

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
// Never blocks and never waits for a reader. Text goes into a ring and
// con_task() moves it to the CDC endpoint as room appears; with nothing plugged
// into the Type-C -- the normal case -- the ring just keeps its newest
// contents, which is what you want to read the moment you do plug in.
//
// The ring is big in a KVM_USB_DEBUG build because TinyUSB's own log is routed
// here too, and an enumeration prints several kilobytes in a burst.
#if KVM_USB_DEBUG
#define CON_RING 32768u
#else
#define CON_RING 4096u
#endif
// NOT ZEROED AT BOOT, ON PURPOSE: the ring survives a watchdog reset, so the
// log shows what led up to one. con_init() only clears it when the magic is
// wrong (a real power-up).
//
// It does NOT survive the bootloader. Measured 2026-11-12: after BOOT+RESET
// with power held, picotool read this whole region back as zeros -- the
// RP2350's boot ROM wipes RAM. So a debug build also copies the ring to flash
// (conlog_flash_task) and rp2350/readlog.sh reads it from there.
#define CON_MAGIC 0x4B564D4Cu                  /* 'KVML' */
struct conlog {
    uint32_t magic;
    uint32_t head, tail;                       /* free-running; index = % CON_RING */
    uint8_t  buf[CON_RING];
};
static struct conlog __uninitialized_ram(g_conlog);
#define g_con      g_conlog.buf
#define g_con_head g_conlog.head
#define g_con_tail g_conlog.tail

static void con_init(void)
{
    if (g_conlog.magic != CON_MAGIC || g_con_head - g_con_tail > CON_RING) {
        g_conlog.magic = CON_MAGIC;
        g_con_head = g_con_tail = 0;
    }
}

static void con_write(const void *p, uint32_t n)
{
    const uint8_t *b = p;
    // TinyUSB may log from the PIO-USB timer interrupt as well as the task.
    uint32_t irq = save_and_disable_interrupts();
    for (uint32_t i = 0; i < n; i++) {
        g_con[g_con_head++ % CON_RING] = b[i];
        if (g_con_head - g_con_tail > CON_RING) g_con_tail++;   /* drop oldest */
    }
    restore_interrupts(irq);
}

static void con_task(void)
{
#if KVM_NATIVE_HOST
    // The Type-C is the HOST port in this build: there is no CDC to drain to.
    // The ring still fills, for a debugger.
    return;
#else
    if (!tud_cdc_connected()) return;
    uint32_t room = tud_cdc_write_available();
    uint8_t  chunk[64];
    uint32_t n = 0;
    uint32_t irq = save_and_disable_interrupts();
    while (n < sizeof chunk && n < room && g_con_tail != g_con_head)
        chunk[n++] = g_con[g_con_tail++ % CON_RING];
    restore_interrupts(irq);
    if (n) { tud_cdc_write(chunk, n); tud_cdc_write_flush(); }
#endif
}

#if KVM_USB_DEBUG
// tusb_config.h points CFG_TUSB_DEBUG_PRINTF here.
int kvm_dbg_printf(const char *fmt, ...)
{
    char b[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof b - 1) n = sizeof b - 1;
    if (n > 0) con_write(b, (uint32_t)n);
    return n;
}
#endif

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
}

#if KVM_USB_DEBUG
// THE LOG, IN FLASH, FOR WHEN THE TYPE-C IS THE HOST PORT AND THERE IS NO
// CONSOLE. Three seconds after the log last grew, the whole ring is written at
// CONLOG_FLASH_OFF; rp2350/readlog.sh reads it back with picotool once the
// board is in the bootloader, by whatever route -- power may drop, unlike RAM.
// Debug builds only: it stops both cores for about half a second per write.
#define CONLOG_FLASH_OFF  0x100000u            /* 1 MB into a 2 MB flash */
#define CONLOG_FLASH_LEN  ((sizeof g_conlog + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1))

static void conlog_flash_write(void *unused)
{
    (void)unused;
    static uint8_t page[CONLOG_FLASH_LEN];     /* a stable, page-padded copy */
    memset(page, 0xFF, sizeof page);
    memcpy(page, &g_conlog, sizeof g_conlog);
    // A sector erase may take 400 ms and there are nine of them; the watchdog
    // is 2 s. Feed it per sector or the write resets the board half-way.
    for (uint32_t o = 0; o < CONLOG_FLASH_LEN; o += FLASH_SECTOR_SIZE) {
        watchdog_update();
        flash_range_erase(CONLOG_FLASH_OFF + o, FLASH_SECTOR_SIZE);
    }
    watchdog_update();
    flash_range_program(CONLOG_FLASH_OFF, page, CONLOG_FLASH_LEN);
    watchdog_update();
}

static void conlog_flash_task(void)
{
    static uint32_t        seen, saved;
    static absolute_time_t quiet_since;
    if (g_con_head != seen) {
        seen = g_con_head;
        quiet_since = get_absolute_time();
        return;
    }
    if (seen == saved || absolute_time_diff_us(quiet_since, get_absolute_time()) < 3000000)
        return;
    saved = seen;
    watchdog_update();
    flash_safe_execute(conlog_flash_write, NULL, 1000);
    watchdog_update();
}
#endif

// Counted by the patched hcd_rp2040.c (patches/) instead of panicking.
volatile uint32_t kvm_data_seq_errors, kvm_rx_timeouts;

// PICO_PANIC_FUNCTION (CMakeLists.txt): the SDK's panic() prints through stdio,
// which this firmware does not have. Keep the message, then wait for the
// watchdog.
void kvm_panic(const char *fmt, ...)
{
    char b[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    con_write("\r\n*** PANIC: ", 14);
    if (n > 0) con_write(b, (uint32_t)(n > (int)sizeof b - 1 ? (int)sizeof b - 1 : n));
    con_write("\r\n", 2);
    for (;;) tight_loop_contents();
}

// ---------------------------------------------------------------------------
// Core 1: the 1 ms frame clock
// ---------------------------------------------------------------------------
static void core1_main(void)
{
#if KVM_USB_DEBUG
    flash_safe_execute_core_init();            /* let core 0 park us for a flash write */
#endif
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
#if KVM_LINK_BEACON
// BENCH ONLY (-DKVM_LINK_BEACON=1). With no console, progress is signalled by
// sending n bytes of 0x00 to the FPGA: not a frame, so each one bumps the
// 'resyncs' counter of opcode 'W' and nothing else. 3 = a device was
// configured, 20 = a HID interface was mounted, 100 = a device went away.
static void beacon(unsigned n)
{
    static const uint8_t z[100] = { 0 };
    uart_write_blocking(LINK_UART, z, n > sizeof z ? sizeof z : n);
}
void tuh_mount_cb(uint8_t dev)  { (void)dev; beacon(3); }
void tuh_umount_cb(uint8_t dev) { (void)dev; beacon(100); }
#else
#define beacon(n) ((void)0)
#endif
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
    beacon(20);
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
//
// It also carries the FPGA's STATUS frames (kvm_down.h): which host the reports
// are being routed to. kvm_down_feed() keeps those and hands back the rest.
static kvm_down_t g_down;

// WHICH HOST THE FPGA IS FEEDING: KVM_TARGET_MAIN, KVM_TARGET_BACKUP, or
// KVM_TARGET_UNKNOWN when the FPGA has been silent for KVM_DOWN_STALE_MS.
// Nothing acts on it yet -- it is here for what comes next (an indicator, a
// per-host key map, ...). Core 0 only.
uint8_t kvm_target(void)
{
    return kvm_down_target(&g_down, to_ms_since_boot(get_absolute_time()));
}

static const char *target_name(uint8_t t)
{
    return t == KVM_TARGET_MAIN ? "Main" : t == KVM_TARGET_BACKUP ? "Backup" : "unknown";
}

static void fpga_rx_task(void)
{
    static uint8_t shown = KVM_TARGET_UNKNOWN;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    for (int budget = 128; budget > 0 && uart_is_readable(LINK_UART); budget--) {
        uint8_t text[12];
        bool    status;
        int     n = kvm_down_feed(&g_down, (uint8_t)uart_getc(LINK_UART), now,
                                  text, &status);
        if (n) con_write(text, (uint32_t)n);
    }

    // Report the target when it CHANGES, including to and from "unknown":
    // losing the FPGA is as much news as a switch.
    uint8_t t = kvm_down_target(&g_down, now);
    if (t != shown) {
        shown = t;
        con_printf("[rp] target %s (fpga switches %lu)\r\n", target_name(t),
                   (unsigned long)g_down.switches);
    }
}

static void status_task(void)
{
    static absolute_time_t next;
    if (!time_reached(next)) return;
    next = make_timeout_time_ms(2000);

    // Typed input is never forwarded: the FPGA reads its RX as HID frames
    // only. The one command is 'B' = reboot into the UF2 bootloader, so a new
    // firmware can be copied without reaching for the BOOT button.
#if !KVM_NATIVE_HOST
    while (tud_cdc_available()) {
        if (tud_cdc_read_char() == 'B') rom_reset_usb_boot(0, 0);
    }
#endif

    critical_section_enter_blocking(&g_lock);
    kvm_sched_t c = g_sched;
    critical_section_exit(&g_lock);

    // ONLY WHEN SOMETHING MOVED. An unconditional line every 2 s meant the log
    // was never quiet for 3 s, so conlog_flash_task() never wrote it: measured
    // 2026-11-12, the flash region read back as all FF after a full session.
    static uint32_t last;
    uint32_t sum = c.frames_kbd + c.frames_mouse + c.frames_neutral + c.kbd_coalesced;
    if (sum == last) return;
    last = sum;
    con_printf("[rp] frames kbd %lu mouse %lu neutral %lu, kbd coalesced %lu"
               " | target %s, status ok %lu bad %lu\r\n",
               (unsigned long)c.frames_kbd, (unsigned long)c.frames_mouse,
               (unsigned long)c.frames_neutral, (unsigned long)c.kbd_coalesced,
               target_name(kvm_target()), (unsigned long)g_down.frames_ok,
               (unsigned long)g_down.frames_bad);
}

// TinyUSB's host stack times its enumeration delays with this. The examples
// get it from their board support package, which this project does not use.
uint32_t tusb_time_millis_api(void)
{
    return to_ms_since_boot(get_absolute_time());
}

int main(void)
{
#if !KVM_NATIVE_HOST
    // Pico-PIO-USB bit-bangs 12 Mbit/s and needs a system clock that is a
    // multiple of 12 MHz. The RP2350's default 150 MHz is not.
    set_sys_clock_khz(120000, true);
#endif

    uart_init(LINK_UART, LINK_BAUD);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    gpio_set_function(LINK_TX_PIN, UART_FUNCSEL_NUM(LINK_UART, LINK_TX_PIN));
    gpio_set_function(LINK_RX_PIN, UART_FUNCSEL_NUM(LINK_UART, LINK_RX_PIN));
    // An unconfigured FPGA floats its TX; without this the RX reads noise.
    gpio_pull_up(LINK_RX_PIN);

    con_init();
    con_printf("\r\n[rp] boot%s\r\n", watchdog_caused_reboot() ? " (WATCHDOG)" : "");
    // A panic or a hang in the USB stack must not leave a dead keyboard until
    // someone power-cycles the box. panic() ends in a loop; this ends the loop.
    watchdog_enable(2000, true);

    kvm_sched_init(&g_sched);
    kvm_down_init(&g_down);
    critical_section_init(&g_lock);

#if KVM_NATIVE_HOST
    // HOST ON THE RP2350'S OWN USB CONTROLLER (the Type-C, through a C-to-A
    // OTG adapter). Unlike the bit-banged port it has the low-speed preamble
    // in hardware, which is what a wired keyboard behind a hub needs.
    tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUH_RHPORT, &host_init);
#else
    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = USB_A_DP_PIN;
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUH_RHPORT, &host_init);
#endif

    // SAY SOMETHING AT BOOT. One all-released keyboard report: it is what the
    // host should hold after we restart anyway (a key that was down when we
    // went away must not stay down), and it makes frames_ok on the FPGA tick
    // once -- the only sign of life this board can give when the Type-C is
    // not a console.
    static const uint8_t released[8] = { 0 };
    kvm_sched_keyboard(&g_sched, released);

    multicore_launch_core1(core1_main);

    for (;;) {
        watchdog_update();
#if !KVM_NATIVE_HOST
        tud_task();
#endif
        tuh_task();
        protocol_task();
        fpga_rx_task();
        status_task();
        con_task();
#if KVM_USB_DEBUG
        conlog_flash_task();
#endif
    }
}
