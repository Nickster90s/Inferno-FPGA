// KVM: HID reports from the RP2350, routed to one PHY at a time.
//
// BOTH HID DEVICES ARE ALWAYS ENUMERATED. The descriptors carry the HID
// interface unconditionally on both PHYs, so switching does not add, remove or
// re-enumerate anything -- it only changes which report register this file
// writes. The host whose turn it is not simply sees its report stop changing.
// That is the whole reason the routing is here and not a mux in gateware: a mux
// can pass or zero, and zeroing is not the same as releasing (see below).
//
// WIRE FORMAT, 12 bytes per report:
//
//     0  0xA5   sync
//     1  0x5A   sync
//     2  report ID: 1 = keyboard, 2 = mouse
//     3..10     8 payload bytes
//     11        XOR of bytes 2..10
//
// Two sync bytes and a checksum because a UART has no framing of its own and
// either end can be reset mid-frame. On a checksum failure the parser drops ONE
// byte and rescans rather than discarding the buffer: a stream that lost a byte
// resynchronises within 12, and a stream that is merely carrying 0xA5 0x5A in a
// payload resynchronises on the next failure instead of locking up.
//
// 12 bytes at even 1 kHz is 12 kB/s against a 1 Mbaud link -- about 10% -- so
// the link is sized by latency and robustness, not throughput.

#include "kvm.h"
#include "gptp.h"            // gptp_uptime_ms()
#include <generated/csr.h>
#include <stdio.h>
#include <libbase/uart.h>    // uart_read*/uart_write: the ring, never the FIFO

#include "log.h"
#define printf(...) LOGF(__VA_ARGS__)

kvm_stats_t g_kvm_stats;

static uint8_t g_dest = KVM_DEST_MAIN;

// Release sequence for the PHY we just left. See kvm_select().
static uint8_t  g_rel_phy;
static uint8_t  g_rel_stage;
static uint32_t g_rel_at_ms;

#define KVM_RELEASE_SETTLE_MS 4      /* >= 4 polls at the 1 ms endpoint rate */

// ---------------------------------------------------------------------------
// Writing a report
// ---------------------------------------------------------------------------
// The report CSR is 72 bits = three words, and LiteX gives no inline accessor
// past 64 bits, so the words go out by hand. It is declared atomic_write, which
// means the two upper words land in a backstore and the LOWEST word commits all
// three at once -- and the lowest word is the HIGHEST address, so ascending
// address order commits last by construction. Writing them in any other order
// would publish a half-updated report, which the host reads as a key that was
// never pressed.
//
// Bit mapping: the endpoint is little-endian, so report byte 0 is bits [7:0].
//     ADDR+0 -> bits [71:64] -> byte 8
//     ADDR+4 -> bits [63:32] -> bytes 7..4
//     ADDR+8 -> bits [31:0]  -> bytes 3..0   (commits)
static void kvm_write_report(uint8_t phy, const uint8_t *r)
{
    uint32_t hi  = (uint32_t)r[8];
    uint32_t mid = ((uint32_t)r[7] << 24) | ((uint32_t)r[6] << 16) |
                   ((uint32_t)r[5] << 8)  |  (uint32_t)r[4];
    uint32_t lo  = ((uint32_t)r[3] << 24) | ((uint32_t)r[2] << 16) |
                   ((uint32_t)r[1] << 8)  |  (uint32_t)r[0];

    volatile uint32_t *base;
    if (phy == KVM_DEST_BACKUP) {
#ifdef CSR_MAIN_KVM_BACKUP_REPORT_ADDR
        base = (volatile uint32_t *)CSR_MAIN_KVM_BACKUP_REPORT_ADDR;
#else
        return;                      /* single-USB build: no Backup PHY */
#endif
    } else {
        base = (volatile uint32_t *)CSR_MAIN_KVM_MAIN_REPORT_ADDR;
    }
    base[0] = hi;
    base[1] = mid;
    base[2] = lo;                    /* commits */
}

// All-zero payload under a given report ID = "nothing pressed, nothing moved".
static void kvm_write_release(uint8_t phy, uint8_t report_id)
{
    uint8_t r[KVM_REPORT_BYTES] = { report_id, 0, 0, 0, 0, 0, 0, 0, 0 };
    kvm_write_report(phy, r);
}

// ---------------------------------------------------------------------------
// Switching
// ---------------------------------------------------------------------------
// THE STUCK-KEY PROBLEM, AND WHY ZEROING IS NOT ENOUGH.
//
// The endpoint relays whatever is in the register on every poll, so a host
// keeps seeing the LAST report it was given. Stop writing to a PHY while a key
// is down and that key stays down on that host, forever. Writing zeroes is not
// a fix either: byte 0 is the report ID, and report ID 0 is not declared, so an
// all-zero register is a report the host's parser discards -- leaving the last
// real report standing.
//
// So the PHY being left is handed two real release reports: the mouse first,
// then the keyboard, with a settle gap between them because ONE register cannot
// hold both at once and the host must actually poll each. The keyboard one is
// written last and stays, because a held key is the failure people notice.
// ---------------------------------------------------------------------------
// Telling the RP2350 which host it is typing into
// ---------------------------------------------------------------------------
// The same 12-byte frame, the other way: A5 5A | 0x80 | 8 payload | xor.
//
//     p0     target, KVM_DEST_MAIN / KVM_DEST_BACKUP
//     p1     format version, 1
//     p2..5  g_kvm_stats.switches, little-endian
//     p6,7   0
//
// IDs 0x80 and up are FPGA -> RP2350; 1 and 2 are the HID reports coming up.
// rp2350/src/kvm_down.h is the other end and must change with this.
//
// SENT ON EVERY SWITCH AND ONCE A SECOND. Once a second because either end can
// restart alone: an RP2350 that has just booted learns the target within a
// second without asking, and one that hears nothing knows we are gone. 12
// bytes a second on a 1 Mbaud line.
//
// THIS UART'S TX ALSO CARRIES THE LOG (log.h), which is why the frame is
// self-synchronising in this direction too. The two cannot interleave: both
// are written from the main loop, and uart_write() only queues into libbase's
// TX ring, which the ISR drains in order.
#define KVM_STATUS_ID        0x80
#define KVM_STATUS_PERIOD_MS 1000

static uint32_t g_status_at_ms;

static void kvm_send_status(void)
{
    uint32_t sw = g_kvm_stats.switches;
    uint8_t f[12] = { 0xA5, 0x5A, KVM_STATUS_ID, g_dest, 1,
                      (uint8_t)sw, (uint8_t)(sw >> 8), (uint8_t)(sw >> 16),
                      (uint8_t)(sw >> 24), 0, 0, 0 };
    for (int i = 2; i < 11; i++) f[11] ^= f[i];
    for (int i = 0; i < 12; i++) uart_write((char)f[i]);
    g_status_at_ms = gptp_uptime_ms();
    g_kvm_stats.status_tx++;
}

void kvm_select(uint8_t dest)
{
    dest = (dest == KVM_DEST_BACKUP) ? KVM_DEST_BACKUP : KVM_DEST_MAIN;
    if (dest == g_dest) return;

    g_rel_phy   = g_dest;
    g_rel_stage = 2;                       /* 2 = mouse next, 1 = keyboard */
    g_rel_at_ms = gptp_uptime_ms();
    kvm_write_release(g_rel_phy, 2);       /* mouse: buttons up, no movement */
    g_rel_stage = 1;

    g_dest = dest;
    g_kvm_stats.switches++;
    kvm_send_status();                     /* now, not at the next second */
    printf("[kvm] -> %s\n", dest == KVM_DEST_BACKUP ? "Backup" : "Main");
}

uint8_t kvm_dest(void) { return g_dest; }

void kvm_init(void)
{
    // Park BOTH PHYs on a released keyboard report. Not strictly required --
    // the CSR resets to 1, which is already report ID 1 with an empty payload
    // -- but doing it explicitly means the idle state is stated in one place
    // rather than inferred from a reset value in avb_soc.py.
    kvm_write_release(KVM_DEST_MAIN, 1);
    kvm_write_release(KVM_DEST_BACKUP, 1);
    g_dest = KVM_DEST_MAIN;                /* always Main at boot, by decision */
    printf("[kvm] ready, target Main\n");
}

// ---------------------------------------------------------------------------
// The link
// ---------------------------------------------------------------------------
// THE REPORTS COME IN ON THE ONE UART THIS DESIGN HAS -- 1 Mbaud, 64-deep FIFO,
// moved off the CH347 onto the SODIMM at the RP2350 (AB22 = SODIMM 159 = our
// TX, W20 = SODIMM 161 = our RX). There is no second UART.
// There was one for a day (2026-11-11) and it cost that day: with it the
// netlist placed on 1 of 8 seeds, without it 6 of 8. It was also a literal
// duplicate -- same rate, same FIFO depth -- of a peripheral that had been
// kept FOR this link when the rest of the console was stripped.
//
// READ THROUGH LIBBASE, NEVER THE FIFO. LiteX's uart_init() enables
// UART_EV_RX and attaches uart_isr(), which drains the hardware FIFO into a
// 128-byte software ring (litex/soc/software/libbase/uart.c). Polling
// uart_rxtx_read() here would race that handler for the same bytes and lose
// them at random -- so this reads uart_read_nonblock()/uart_read(), which take
// from the ring the ISR fills.
//
// Nothing else reads this UART: loader.c has one uart_rxtx_write() and log.h
// only writes, so the receive side belongs entirely to this file.
// CORRECTED 2026-11-11: that was false when written. main.c's check_uart_cmd()
// read the same ring once per main-loop pass, took the A5 off isolated frames
// and ran it as a console command. Its call is removed; see main.c's loop. TX still
// carries the log, which is why the frames below are self-synchronising --
// whatever the FPGA prints cannot be mistaken for a report.


static uint8_t  g_buf[12];
static uint8_t  g_len;

static void kvm_frame(const uint8_t *f)
{
    uint8_t x = 0;
    for (int i = 2; i < 11; i++) x ^= f[i];
    if (x != f[11]) { g_kvm_stats.frames_bad++; return; }
    if (f[2] != 1 && f[2] != 2) { g_kvm_stats.frames_bad++; return; }
    kvm_write_report(g_dest, f + 2);
    g_kvm_stats.frames_ok++;
}

void kvm_poll(void)
{
    // Finish a release sequence that kvm_select() started.
    if (g_rel_stage == 1 &&
        (uint32_t)(gptp_uptime_ms() - g_rel_at_ms) >= KVM_RELEASE_SETTLE_MS) {
        kvm_write_release(g_rel_phy, 1);   /* keyboard: all keys up */
        g_rel_stage = 0;
    }

    if ((uint32_t)(gptp_uptime_ms() - g_status_at_ms) >= KVM_STATUS_PERIOD_MS)
        kvm_send_status();

    // BOUNDED. The software ring is 128 bytes and this runs in the main loop
    // next to the PTP servo and the Dante stacks; draining "until empty" would
    // let a stuck-high RX line starve everything else. 96 bytes is eight whole
    // frames per pass, which at 1 kHz is eight times the arrival rate.
    for (int budget = 96; budget > 0; budget--) {
        if (!uart_read_nonblock()) break;
        uint8_t b = (uint8_t)uart_read();
        g_kvm_stats.bytes_rx++;

        if (g_len == 0 && b != 0xA5) { g_kvm_stats.resyncs++; continue; }
        if (g_len == 1 && b != 0x5A) {
            // Not a frame start after all. The byte we have may itself be the
            // real 0xA5, so re-examine it rather than dropping both.
            g_len = (b == 0xA5) ? 1 : 0;
            g_kvm_stats.resyncs++;
            continue;
        }
        g_buf[g_len++] = b;
        if (g_len == 12) {
            kvm_frame(g_buf);
            g_len = 0;
        }
    }

    // Hardware FIFO full means the ISR is not keeping up -- a weaker signal
    // than it was when this file owned the FIFO, because libbase's ring can
    // also overflow silently, but it is the only one exposed.
    if (uart_rxfull_read()) g_kvm_stats.overrun++;
}
