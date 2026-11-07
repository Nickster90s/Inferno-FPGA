#ifndef DANTE_LOG_H
#define DANTE_LOG_H

#include <stdio.h>

// THE CONSOLE NARRATIVE WAS THE BIGGEST SINGLE THING IN CODERAM, and it was the
// format strings, not the driver. Measured 2026-11-06 on the linked image:
//
//     .text    78808        coderam total 92108 / 98304 = 93%
//     .rodata  13120
//
//     203 printf format strings     10084 bytes  = 77% of ALL .rodata
//     vfprintf                       1712 bytes
//     uart_* driver + printf           676 bytes
//
// So the UART costs ~2.4 KB and the STORIES cost 10 KB. The UART stays -- it is
// wanted for an RP2350 link, and the console commands with side effects still
// work: 'r' reboot, 't'/'T' gateware talker, 'x' rx_gate, 'd' mcr_dante,
// 'z' cap_reset. What goes is the prose.
//
// CORRECTED 2026-11-07: this used to add "firmware/loader/loader.c reads it
// (that is what tools/netload.py talks to)" as a reason. It does not.
// loader.c contains exactly one UART reference, uart_rxtx_write() -- a write --
// and no reads at all; netload.py speaks raw Ethernet, not serial. The UART's
// read side is used only by check_uart_cmd() in the application firmware. The
// conclusion stands, the reason was wrong, and it was wrong in a comment that
// exists to be trusted.
//
// HOW. Each .c that only tells stories does, AFTER its includes:
//
//     #include "log.h"
//     #define printf(...) LOGF(__VA_ARGS__)
//
// Quiet, LOGF does not evaluate its arguments, so the literals are never
// referenced and never reach .rodata. The text stays in the source, which is
// the point: in this project the messages ARE the record of what was measured,
// and deleting them would throw that away to save bytes that are already saved.
// Rebuild with -DDANTE_VERBOSE=1 and every line comes back.
//
// CHECKED BEFORE DOING IT: all 203 sites were parsed and the only function
// calls inside printf arguments are CSR reads and pure getters
// (aaf_pkt_*_read, ethmac_*_read, mcr_*_read, main_rx_ts_*_read, rd16,
// dante_fpp_adv_max, rx_gate_sw_filtered). Nothing with a side effect is lost
// by not evaluating them. There is no puts/putchar/vprintf anywhere, so printf
// is the only output path.
//
// ONE CONSEQUENCE, STATED SO IT IS NOT A SURPRISE: g_verbose ('v' on the
// console, read in gptp.c) now gates lines that no longer exist. A runtime flag
// cannot save a format string -- only compile-time removal can -- so the flag
// is kept but its remaining sites are inert unless DANTE_VERBOSE is set.

#ifndef DANTE_VERBOSE
#define DANTE_VERBOSE 0
#endif

#if DANTE_VERBOSE
#define LOGF(...) (printf)(__VA_ARGS__)
#else
#define LOGF(...) do { } while (0)
#endif

// Printed whatever DANTE_VERBOSE says. The parenthesised (printf) reaches the
// real function even in a file that has redirected printf: a function-like
// macro is not expanded when its name is not followed by '('.
#define UARTF(...) (printf)(__VA_ARGS__)

#endif /* DANTE_LOG_H */
