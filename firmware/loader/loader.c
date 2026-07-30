// Inferno-FPGA boot loader — Dante Phase 0.5
//
// Lives in the 8 KB BRAM ROM and is PERMANENTLY FROZEN: its contents must not
// change when the application firmware changes, because ROM contents are part
// of the bitstream and altering them re-rolls nextpnr's placement (a 6-10 min
// P&R plus a seed sweep plus a USB hardware re-test, per firmware edit).
//
// Job: get an application image into `coderam` (0x20000000) and jump to it.
// Two sources, tried in this order:
//
//   1. NETLOAD  — a short window at boot listening for raw Ethernet frames
//      (ethertype 0x88B5, IEEE 802 local experimental 1) carrying the image.
//      This is the development path: edit C, make, netload, running. Seconds,
//      no P&R, no seed roulette, no USB risk. Also avoids /dev/ttyACM0
//      entirely, which matters because that port is the interactive console.
//
//   2. SPI FLASH — the persistent path, so the board boots standalone with no
//      host attached. Image lives at FLASH_IMAGE_ADDR behind a small header.
//
// If both fail it prints and halts rather than jumping into garbage.
//
// DELIBERATELY has no dependency on libbase printf: at ~1.7 KB for vfprintf
// alone that is a fifth of the ROM. UART output here is a hand-rolled puts/hex.

#include <stdint.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <system.h>              // busy_wait
#include <libliteeth/mdio.h>     // PHY RGMII-ID delay setup
#include "cfgflash.h"

// ---------------------------------------------------------------------------
// Image layout
// ---------------------------------------------------------------------------

// CODERAM_BASE / CODERAM_SIZE come from generated/mem.h, i.e. straight from the
// SoC memory map -- do not hardcode them here or the two can drift apart.

// Magic marks a valid image header, in flash and on the wire.
#define IMG_MAGIC           0x494E464Eu     // "INFN"

// Flash sector holding the persistent image. Chosen well above the ~2.2 MB
// bitstream and BELOW the config sector that config.c owns at the top.
#define FLASH_IMAGE_ADDR    0x00300000u     // 3 MB

// Netload window is defined in poll iterations, not time -- see the long note
// above NETLOAD_WINDOW_SPINS for why two attempts at a real timebase failed.

// ---------------------------------------------------------------------------
// Netload wire protocol (ethertype 0x88B5)
// ---------------------------------------------------------------------------
//
//   offset  size  field
//   0       6     dst mac (broadcast accepted)
//   6       6     src mac (host; we reply here)
//   12      2     0x88B5
//   14      4     magic "INFN"
//   18      1     opcode
//   19      1     reserved
//   20      4     arg0  (START: total length | DATA: byte offset)
//   24      4     arg1  (START: crc32 of image | DATA: payload length)
//   28      ...   payload (DATA only)
//
// DATA carries its payload length EXPLICITLY rather than deriving it from the
// MAC's frame length. LiteEth's CRC checker strips the FCS, so the reported
// length is clean -- but Ethernet pads every frame to a 60-byte minimum, so a
// final chunk shorter than 32 bytes would arrive with its length inflated by
// padding. Deriving plen from the frame length would then write padding bytes
// into the image and advance the offset too far, silently corrupting it.
//
// Opcodes. The loader ACKs every frame with ACK carrying the next expected
// offset, so the host is stop-and-wait and can retry a lost chunk. At 1024-byte
// chunks a 32 KB image is 32 round trips — milliseconds on a direct link.
#define OP_START    1u
#define OP_DATA     2u
#define OP_EXEC     3u
#define OP_ACK      0x80u
#define OP_NAK      0x81u

#define ETH_HDR_LEN     14u
#define NL_HDR_LEN      (ETH_HDR_LEN + 14u)     // through arg1
#define NL_ETHERTYPE    0x88B5u
#define NL_MAX_PAYLOAD  1024u

#define ETHMAC_EV_SRAM_WRITER 0x1

// ---------------------------------------------------------------------------
// Minimal UART (no libbase)
// ---------------------------------------------------------------------------

static void uputc(char c)
{
    while (uart_txfull_read())
        ;
    uart_rxtx_write((uint8_t)c);
}

static void uputs(const char *s)
{
    while (*s) {
        if (*s == '\n') uputc('\r');
        uputc(*s++);
    }
}

static void uputhex(uint32_t v)
{
    static const char hx[] = "0123456789abcdef";
    uputs("0x");
    for (int i = 28; i >= 0; i -= 4)
        uputc(hx[(v >> i) & 0xf]);
}

static void uputdec(uint32_t v)
{
    char buf[11];
    int n = 0;
    if (v == 0) { uputc('0'); return; }
    while (v && n < 10) { buf[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n--) uputc(buf[n]);
}

// ---------------------------------------------------------------------------
// CRC-32 (IEEE, reflected) — bitwise, no 1 KB table. Runs once over the image;
// at 50 MHz a 96 KB worst case is a few ms, which is irrelevant at boot.
// ---------------------------------------------------------------------------

static uint32_t crc32(const uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return ~crc;
}

// ---------------------------------------------------------------------------
// TSU startup
// ---------------------------------------------------------------------------

// Make the TSU tick.
//
// The addend CSRs now reset to nominal in the gateware, so this is belt and
// braces -- but the loader must not silently depend on that, because if the
// addend is 0 the counter never advances. The netload window no longer depends
// on it, but firmware and the PTP servo do, and a TSU that only starts ticking
// once firmware happens to write an addend is a footgun.
//
// Same nominal value as gptp.c:918; gptp_init() later rewrites it identically.
static void tsu_start(void)
{
    uint64_t base = (((uint64_t)1 << 52) + (CONFIG_CLOCK_FREQUENCY / 2))
                    / CONFIG_CLOCK_FREQUENCY;
    tsu_addend_write     ((uint32_t)(base >> 20));
    tsu_addend_frac_write((uint32_t)(base & 0xFFFFFu));
}

// The netload window is counted in POLL ITERATIONS, not in time.
//
// Two failed attempts at a real timebase, both found on hardware, are why:
//
//   1. tsu_nanoseconds_read() returns a LATCHED snapshot, not a live counter --
//      the latch fires on reading _seconds_hi (avb_soc.py, TSUWithCSRs). Reading
//      nanoseconds alone returns the same stale value forever, so the window
//      never expired and a hostless board would hang instead of booting from
//      flash.
//
//   2. Even latched correctly, tsu_nanoseconds is a PTP nanoseconds field that
//      wraps at 1e9, NOT a counter wrapping at 2^32. The usual
//      (int32_t)(now - t0) delta trick is invalid for a 1e9 wrap, so the window
//      expired at an effectively random point -- sometimes immediately, which
//      made netload miss the window entirely.
//
// A loader needs "wait a moment for a host", not a calibrated interval. A spin
// count is monotonic, cannot wrap within the window, and depends on no
// peripheral that firmware is responsible for initialising. If the poll loop
// ever gains work, retune this constant -- it is deliberately generous.
//
// The window now starts AFTER phy_wait_link(), so it only has to cover the
// host's ~20 ms START cadence plus slack -- not link negotiation. ~4e6
// iterations of a bounded CSR-read loop at 50 MHz is on the order of a second,
// which is many START retries, while keeping the fall-through to SPI flash
// prompt once the link is already up.
#define NETLOAD_WINDOW_SPINS   4000000u

// ---------------------------------------------------------------------------
// PHY setup
//
// The B50612D needs its RGMII internal-delay shadow registers programmed or our
// frames are mis-clocked at the PHY and never reach the wire. These registers
// survive a bitstream reload but NOT a power cycle, so the loader must program
// them itself: without this, netload receives DATA frames but its ACKs never
// leave the FPGA, and the transfer hangs forever on a cold boot.
//
// Same sequence as firmware/main.c -- keep the two in step.
// ---------------------------------------------------------------------------

#define PHY_ADDR        1                // RJ45 jack = U9 = PHY1, MDIO addr 1
#define PHY_BMSR        0x01             // basic mode status register
#define PHY_BMSR_LINK   (1u << 2)        // link status (latching low)

static void phy_init(void)
{
    mdio_write(PHY_ADDR, 0x00, 0x9140);  // soft-reset + autoneg + 1G + FD
    busy_wait(200);
    mdio_write(PHY_ADDR, 0x18, 0xF1E7);  // shadow_07 bit 8 = 1 (RXC delay)
    busy_wait(10);
    mdio_write(PHY_ADDR, 0x1C, 0x8E00);  // shadow_03 bit 9 = 1 (TXC delay)
    busy_wait(10);
}

// Wait for the PHY to report link up, bounded.
//
// MEASURED ON HARDWARE: from end-of-reconfiguration to the loader being able to
// answer a netload frame takes ~3.85 s, almost all of it gigabit
// auto-negotiation after the soft-reset above. Opening the netload window
// immediately therefore listened into a dead link and missed the host entirely.
//
// Waiting for link is much better than guessing a big enough window: it adapts
// to however long negotiation actually takes, and the listening window that
// follows can then be short. Do not skip the reset to avoid the wait -- the
// RGMII delay registers do not survive a power cycle and are what make our TX
// reach the wire at all.
//
// BMSR link status is latching-low: read twice and trust the second read.
static void phy_wait_link(void)
{
    for (uint32_t i = 0; i < 200; i++) {   // ~10 s cap at 50 ms/poll
        (void)mdio_read(PHY_ADDR, PHY_BMSR);
        if (mdio_read(PHY_ADDR, PHY_BMSR) & PHY_BMSR_LINK) {
            uputs("[ldr] link up after ");
            uputdec(i * 50u);
            uputs(" ms\n");
            return;
        }
        busy_wait(50);
    }
    uputs("[ldr] link did not come up within 10 s -- continuing anyway\n");
}

// ---------------------------------------------------------------------------
// Ethernet helpers
// ---------------------------------------------------------------------------

static uint8_t  our_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x42};
static uint32_t tx_slot;

static inline uint8_t *rx_slot_ptr(uint32_t slot)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * slot);
}
static inline uint8_t *tx_slot_ptr(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + tx_slot));
}

static void eth_send(uint32_t len)
{
    if (len < 60) len = 60;                 // pad to minimum frame
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(tx_slot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    tx_slot = (tx_slot + 1) % ETHMAC_TX_SLOTS;
}

static inline uint32_t rd32be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}
static inline void wr32be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >>  8); p[3] = (uint8_t)v;
}

// Reply with ACK/NAK carrying `next_off` in arg0.
static void nl_reply(const uint8_t *req, uint8_t op, uint32_t next_off)
{
    uint8_t *tx = tx_slot_ptr();
    // Zero the whole minimum-size frame first: eth_send pads to 60 bytes, and
    // without this the pad would leak stale bytes from a previous frame.
    for (int i = 0; i < 60; i++) tx[i] = 0;
    for (int i = 0; i < 6; i++) tx[i]     = req[6 + i];   // dst = requester
    for (int i = 0; i < 6; i++) tx[6 + i] = our_mac[i];
    tx[12] = (uint8_t)(NL_ETHERTYPE >> 8);
    tx[13] = (uint8_t)NL_ETHERTYPE;
    wr32be(tx + 14, IMG_MAGIC);
    tx[18] = op;
    tx[19] = 0;
    wr32be(tx + 20, next_off);
    wr32be(tx + 24, 0);
    eth_send(NL_HDR_LEN);
}

// ---------------------------------------------------------------------------
// Netload
//
// Returns 1 if a complete, CRC-verified image is now in coderam.
// ---------------------------------------------------------------------------

static int netload(void)
{
    uint32_t img_len = 0, img_crc = 0, next_off = 0;
    int      started = 0;
    uint32_t spins   = 0;

    while (1) {
        // While idle, honour the window. Once a transfer has started, keep
        // going: the host is actively feeding us and the window no longer
        // applies (a large image can easily outlast it).
        if (!started && ++spins > NETLOAD_WINDOW_SPINS)
            return 0;

        if (!(ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER))
            continue;

        uint32_t slot = ethmac_sram_writer_slot_read();
        uint32_t len  = ethmac_sram_writer_length_read();
        uint8_t *f    = rx_slot_ptr(slot);

        if (len >= NL_HDR_LEN &&
            (((uint16_t)f[12] << 8) | f[13]) == NL_ETHERTYPE &&
            rd32be(f + 14) == IMG_MAGIC) {

            uint8_t  op   = f[18];
            uint32_t arg0 = rd32be(f + 20);
            uint32_t arg1 = rd32be(f + 24);

            if (op == OP_START) {
                if (arg0 == 0 || arg0 > CODERAM_SIZE) {
                    nl_reply(f, OP_NAK, 0);
                } else {
                    img_len  = arg0;
                    img_crc  = arg1;
                    next_off = 0;
                    started  = 1;
                    uputs("[ldr] netload start len=");
                    uputdec(img_len);
                    uputs(" crc=");
                    uputhex(img_crc);
                    uputs("\n");
                    nl_reply(f, OP_ACK, 0);
                }
            } else if (op == OP_DATA && started) {
                uint32_t off   = arg0;
                uint32_t plen  = arg1;                    // explicit, see above
                uint32_t avail = len - NL_HDR_LEN;        // upper bound on trust
                if (plen > NL_MAX_PAYLOAD) plen = NL_MAX_PAYLOAD;
                if (plen > avail)          plen = avail;
                if (plen && off == next_off && off + plen <= img_len) {
                    // Byte copy: the payload is not word-aligned in the slot
                    // (NL_HDR_LEN is 28, but the RX slot base is), and coderam
                    // is byte-addressable, so this is correct if unexciting.
                    volatile uint8_t *dst = (volatile uint8_t *)(CODERAM_BASE + off);
                    const uint8_t    *src = f + NL_HDR_LEN;
                    for (uint32_t i = 0; i < plen; i++)
                        dst[i] = src[i];
                    next_off = off + plen;
                }
                // ACK the offset we now expect, so a lost or duplicated frame
                // self-corrects: the host simply resends from next_off.
                nl_reply(f, OP_ACK, next_off);
            } else if (op == OP_EXEC && started) {
                if (next_off != img_len) {
                    nl_reply(f, OP_NAK, next_off);
                } else {
                    uint32_t got = crc32((const uint8_t *)CODERAM_BASE, img_len);
                    if (got != img_crc) {
                        uputs("[ldr] netload CRC mismatch: got ");
                        uputhex(got);
                        uputs(" want ");
                        uputhex(img_crc);
                        uputs("\n");
                        nl_reply(f, OP_NAK, next_off);
                        started = 0;                // let the host retry cleanly
                    } else {
                        nl_reply(f, OP_ACK, img_len);
                        // Let the ACK actually reach the wire before we jump.
                        for (volatile int d = 0; d < 200000; d++)
                            ;
                        uputs("[ldr] netload OK, ");
                        uputdec(img_len);
                        uputs(" bytes\n");
                        ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
                        return 1;
                    }
                }
            }
        }

        ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
    }
}

// ---------------------------------------------------------------------------
// SPI flash
//
// Reuses firmware/cfgflash.c verbatim rather than reimplementing it. That
// driver is hardware-proven and encodes two things easy to get wrong: the
// STARTUPE2 warm-up (7-series masks the first USRCCLKO edges, so the very first
// transaction is swallowed and JEDEC reads 0xFFFFFF), and the 40-bit
// top-aligned MOSI convention with one self-contained auto-CS transfer per byte
// (manual-CS multi-transfer is flaky on this SPIMaster). It is self-contained:
// it includes only <stdint.h> and generated/csr.h.
//
// Slow -- ~16 us/byte at 2.5 MHz, so ~0.5 s for a 32 KB image -- but this runs
// only at boot, and only when netload didn't supply an image.
//
// Header at FLASH_IMAGE_ADDR, big-endian:
//   0  4  magic "INFN"
//   4  4  length
//   8  4  crc32
// ---------------------------------------------------------------------------

#ifdef CSR_CFGFLASH_BASE

static int flashload(void)
{
    uint8_t hdr[12];

    cfgflash_warmup();
    cfgflash_read(FLASH_IMAGE_ADDR, hdr, sizeof(hdr));

    if (rd32be(hdr) != IMG_MAGIC) {
        uputs("[ldr] no flash image (magic ");
        uputhex(rd32be(hdr));
        uputs(")\n");
        return 0;
    }

    uint32_t len = rd32be(hdr + 4);
    uint32_t crc = rd32be(hdr + 8);
    if (len == 0 || len > CODERAM_SIZE) {
        uputs("[ldr] flash image length bad: ");
        uputdec(len);
        uputs("\n");
        return 0;
    }

    uputs("[ldr] flash image ");
    uputdec(len);
    uputs(" bytes, loading\n");
    cfgflash_read(FLASH_IMAGE_ADDR + sizeof(hdr),
                  (uint8_t *)CODERAM_BASE, len);

    uint32_t got = crc32((const uint8_t *)CODERAM_BASE, len);
    if (got != crc) {
        uputs("[ldr] flash CRC mismatch: got ");
        uputhex(got);
        uputs(" want ");
        uputhex(crc);
        uputs("\n");
        return 0;
    }
    return 1;
}

#else   /* no config flash in this SoC build */
static int flashload(void) { return 0; }
#endif

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

int main(void)
{
    uputs("\n[ldr] Inferno-FPGA loader (coderam ");
    uputhex(CODERAM_BASE);
    uputs(", ");
    uputdec(CODERAM_SIZE / 1024);
    uputs(" KB)\n");

    // Both must precede netload:
    //   tsu_start() -- start the timestamp unit ticking for the firmware.
    //   phy_init()  -- the PHY delay registers do not survive a power cycle,
    //                  and without them our ACKs never reach the wire.
    tsu_start();
    phy_init();
    phy_wait_link();     // else the netload window listens into a dead link

    int ok = netload();
    if (!ok)
        ok = flashload();

    if (!ok) {
        uputs("[ldr] no image. Send one with tools/netload.py and reset.\n");
        while (1)
            ;                                   // halt rather than jump to junk
    }

    uputs("[ldr] jumping to firmware\n");

    // No cache maintenance needed: VexRiscv "minimal" has neither an I-cache
    // nor a D-cache, so the instructions we just wrote are already visible to
    // the fetch unit. (This is one place where the cacheless CPU helps.)
    ((void (*)(void))CODERAM_BASE)();

    while (1)
        ;                                       // firmware never returns
}
