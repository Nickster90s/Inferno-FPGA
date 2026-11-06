// Runtime counters over UDP, because the UART is not ours to read.
//
// /dev/ttyACM0 belongs to the operator's picocom, so the build host cannot see
// console output and every "is it underrunning?" question has had to be
// answered by asking for a pasted log. These are the counters that actually
// distinguish a transmit-side glitch from a receive-side one, so they are worth
// a bound port: send any byte to 7779, get a fixed array of
// big-endian u32 counters back. Binary, not text: this libc has no snprintf.

#include "net.h"
#include "dante_dev.h"
#include "dante_arc.h"
#include "mdns.h"
extern uint32_t usb_fb_manual;
// RX TIMESTAMP RING HEALTH. main.c keeps these as globals marked TEMPORARY and
// nothing ever read them. They are the direct measure of the fault the +-3.5 us
// PTP offset noise points at: if the ts ring falls out of lock-step with slot
// consumption, frames are paired with the PREVIOUS frame's timestamp, which
// injects microseconds into every offset without losing a single packet.
extern uint32_t rx_ts_resyncs, rx_pops, g_lvl_pre, g_lvl_post;   // main.c
extern uint16_t g_mcast_fpp;
extern uint32_t g_phase_adj;
extern uint8_t  usb_fb_sweep_hold;
#include "dante_tx.h"
#include "gptp.h"
#include "ptpv1.h"
#include "dante_flows.h"
#include "dante_tx.h"
#include "gptp.h"
#include "rx_gate.h"
#include "mcr_dante.h"
#include "telem.h"
#include <generated/csr.h>
#include "config.h"   // g_fs, g_cfg, cfg_save(): de 'R'-opcode
// dante_info_rate_apply() werd hier IMPLICIET gedeclareerd. Op RISC-V kwam dat
// er door toeval mee weg (een uint32_t-arg in a0, return genegeerd), maar een
// impliciete declaratie is precies wat op 2026-11-03 de OSC-handler met vier in
// plaats van vijf parameters liet compileren en de CPU liet crashen. Includeer
// de header.
#include "dante_info.h"
#include <stdio.h>

/* printf in this file is a NARRATIVE, not an interface. The format strings
 * were 77% of .rodata; see log.h for the measurement and the rebuild flag. */
#include "log.h"
#define printf(...) LOGF(__VA_ARGS__)


#define STATS_PORT 7779

static void put32(uint8_t *p, uint32_t n, uint32_t v)
{
    p[n] = (uint8_t)(v >> 24); p[n+1] = (uint8_t)(v >> 16);
    p[n+2] = (uint8_t)(v >> 8); p[n+3] = (uint8_t)v;
}

// rx_gate control + readout, on a SEPARATE request opcode.
//
// Deliberately NOT extra fields on the main reply. Growing that reply from 200
// to 208 bytes once killed the port outright -- no response at all, while ARC
// and flow control kept answering -- and the cause was never found (see the
// comment further down). The main reply therefore stays byte-identical and
// rx_gate gets its own, smaller record. A different reply is not a bigger one.
//
// Request: 'g'                -> status only
//          'g' '1'            -> arm, provisionally (auto-reverts, see below)
//          'g' '0'            -> disarm
//          'g' 'c'            -> commit (cancel the auto-revert)
//
// Arming over the network is provisional on purpose: if the allow-list is
// wrong, the packet carrying "turn it off" is exactly the one that gets
// dropped. The board undoes it by itself unless told the network survived.
#define RXGATE_REVERT_MS 30000u

static void rxgate_rx(const uint8_t src_ip[4], uint16_t src_port,
                      const uint8_t *req, uint32_t len)
{
    if (len >= 2) {
        switch (req[1]) {
            case '1': rx_gate_set(1, RXGATE_REVERT_MS); break;
            case '0': rx_gate_set(0, 0);                break;
            case 'c': rx_gate_commit();                 break;
            default:  break;                            // status-only
        }
    }

    rx_gate_status_t g;
    rx_gate_get_status(&g);

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    put32(p, n, 0x52584731u);          n += 4;   // 'RXG1' — version tag, so the
                                                 // host parses by name and never
                                                 // by a hand-counted offset.
    put32(p, n, g.enabled);            n += 4;
    put32(p, n, g.pending_revert);     n += 4;
    put32(p, n, g.revert_in_ms);       n += 4;
    put32(p, n, g.match);              n += 4;
    put32(p, n, g.nomatch);            n += 4;
    put32(p, n, g.discarded);          n += 4;
    put32(p, n, ((uint32_t)g.last_drop_mac[0] << 24) |
                ((uint32_t)g.last_drop_mac[1] << 16) |
                ((uint32_t)g.last_drop_mac[2] << 8)  |
                 (uint32_t)g.last_drop_mac[3]);  n += 4;
    put32(p, n, ((uint32_t)g.last_drop_mac[4] << 8) |
                 (uint32_t)g.last_drop_mac[5]);  n += 4;
    // The two counters that say whether this is working, alongside the gate's
    // own: writer_errors is the thing being fixed, and rx_filtered is the
    // SOFTWARE filter's drop count -- it should go flat as the gate takes over.
    put32(p, n, ethmac_sram_writer_errors_read()); n += 4;
    put32(p, n, rx_gate_sw_filtered());            n += 4;

    net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
}

// Media-clock control + readout, opcode 'm'. Same reasoning as 'g': a separate,
// smaller record rather than extra fields on the 200-byte main reply, which
// died once when it grew to 208.
//
//   'm'        -> status only
//   'm' '1'    -> arm the discipline
//   'm' '0'    -> disarm (NCO back to nominal, immediately)
static void mclk_rx(const uint8_t src_ip[4], uint16_t src_port,
                    const uint8_t *req, uint32_t len)
{
    if (len >= 2) {
        if (req[1] == '1') mcr_dante_set_enabled(1);
        else if (req[1] == '0') mcr_dante_set_enabled(0);
        else if (req[1] == 'P') mcr_dante_set_phase_enabled(1);
        else if (req[1] == 'p') mcr_dante_set_phase_enabled(0);
    }

    mcr_dante_status_t m;
    mcr_dante_get_status(&m);

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    put32(p, n, 0x4D434C4Bu);                    n += 4;   // 'MCLK' version tag
    put32(p, n, m.enabled);                      n += 4;
    put32(p, n, m.ptp_locked);                   n += 4;
    put32(p, n, (uint32_t)m.target_ppb);         n += 4;
    put32(p, n, (uint32_t)m.applied_ppb);        n += 4;
    put32(p, n, m.base_inc);                     n += 4;
    put32(p, n, m.applied_inc);                  n += 4;
    put32(p, n, m.nco_writes);                   n += 4;
    put32(p, n, m.trips);                        n += 4;
    put32(p, n, m.lvl_min);                      n += 4;
    put32(p, n, m.lvl_avg);                      n += 4;
    put32(p, n, m.lvl_max);                      n += 4;
    put32(p, n, m.underrun_per_s);               n += 4;
    put32(p, n, (uint32_t)m.drift_samples);      n += 4;
    put32(p, n, aaf_pkt_underrun_count_read());  n += 4;
    put32(p, n, aaf_pkt_overrun_count_read());   n += 4;
    put32(p, n, m.phase_enabled);                n += 4;
    put32(p, n, (uint32_t)m.phase_ppb);          n += 4;
    // Appended, not inserted: tools/mclk.py zips FIELDS against however many
    // words arrive, so a tool older than the firmware simply drops the tail.
    // drift_valid says whether drift_samples is this second's measurement or
    // the last good one held over -- differentiating a held value is what
    // makes `watch` report a drift rate that never happened.
    put32(p, n, m.drift_valid);                  n += 4;
    put32(p, n, m.drift_rejects);                n += 4;
    put32(p, n, m.phase_rejects);                n += 4;

    net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
}

static void stats_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip;

    if (len >= 1 && req[0] == 'g') {
        rxgate_rx(src_ip, src_port, req, len);
        return;
    }
    // Servo tuning: 's' <median_n> <ki_num BE32> <exact>  -> current tuning back.
    if (len >= 1 && req[0] == 's') {
        if (len >= 7)
            ptpv1_set_tuning(req[1],
                             (int32_t)(((uint32_t)req[2] << 24) |
                                       ((uint32_t)req[3] << 16) |
                                       ((uint32_t)req[4] << 8)  | req[5]),
                             req[6]);
        uint8_t mn, ex; int32_t ki;
        ptpv1_get_tuning(&mn, &ki, &ex);
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = 0;
        put32(p, n, 0x53525630u); n += 4;   // 'SRV0'
        put32(p, n, mn);          n += 4;
        put32(p, n, (uint32_t)ki); n += 4;
        put32(p, n, ex);          n += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
        return;
    }
    // Per-flow detail, opcode 'f'. What we ACTUALLY bound: destination socket,
    // slot count, fpp and the slot->channel map. Exists because diagnosing "all
    // green in Dante Controller but no audio" repeatedly came down to guessing
    // at this from packet rates when the console had it written down.
    // 'p' -- pacing diagnostics: per-context DUE vs EMIT counts.
    //
    // "packets per second is a fraction of nominal" does not localise the
    // fault, and three attempts at inferring it from the aggregate were wrong.
    // due tells us what the pacing decided, emit what reached the wire.
    if (len >= 1 && req[0] == 'p') {
        uint8_t *p3 = net_udp_payload_buf();
        uint32_t n3 = 0;
        put32(p3, n3, 0x50414331u); n3 += 4;                 // 'PAC1'
        put32(p3, n3, DANTE_TX_CHANNELS / 8); n3 += 4;       // contexts
        put32(p3, n3, aaf_pkt_ts_now_sub_read()); n3 += 4;   // live sample counter
        net_udp_commit(src_ip, src_port, STATS_PORT, n3, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'f') {
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = 0;
        // 'FLW3', not 'FLW2'. A seven-word trailer was appended after the
        // flow array (why the talker went off, why flows were unbound), which
        // an FLW2 reader would read as a seventh flow, so the tag moves with
        // the layout rather than letting an old reader silently misalign.
        put32(p, n, 0x464C5733u); n += 4;          // 'FLW3'
        put32(p, n, 6);           n += 4;          // N_FLOWS
        for (unsigned f = 0; f < 6; f++) {
            dante_tx_flow_detail_t d;
            dante_tx_flow_detail(f, &d);
            put32(p, n, ((uint32_t)d.in_use << 24) | ((uint32_t)d.nslots << 16) |
                        ((uint32_t)d.fpp << 8) | d.mcast);            n += 4;
            put32(p, n, ((uint32_t)d.dst[0] << 24) | ((uint32_t)d.dst[1] << 16) |
                        ((uint32_t)d.dst[2] << 8) | d.dst[3]);        n += 4;
            put32(p, n, d.dport);                                     n += 4;
            put32(p, n, d.age_ms);                                    n += 4;
            put32(p, n, d.cookie);                                    n += 4;
            put32(p, n, d.ka_rx);                                     n += 4;
            put32(p, n, d.ka_age_ms);                                 n += 4;
            for (unsigned i = 0; i < 8; i += 2)
                { put32(p, n, ((uint32_t)d.chans[i] << 16) | d.chans[i+1]); n += 4; }
        }
        // TRAILER: waarom de talker uitging en waarom flows verdwenen.
        // `disables` en `rebinds` zeggen hoeveel, niet waardoor; met een dode
        // console is dit het enige onderscheid tussen "PTP viel weg" (beide
        // ontvangers stil, want de talker gaat uit) en "een ontvanger stopte"
        // (alleen die flow weg). Zie dante_tx.h.
        put32(p, n, g_tx_stats.off_unlock); n += 4;
        put32(p, n, g_tx_stats.off_noflow); n += 4;
        put32(p, n, g_tx_stats.off_rate);   n += 4;
        put32(p, n, g_tx_stats.ub_arc);     n += 4;
        put32(p, n, g_tx_stats.ub_stop);    n += 4;
        put32(p, n, g_tx_stats.ub_expire);  n += 4;
        put32(p, n, g_tx_stats.ub_drop);    n += 4;
        // En de twee PTP-tellers die hier de oorzaak van off_unlock zijn.
        // step_count stond nergens in een antwoord, dus een re-anchor door een
        // klokstap was alleen uit `anchors` te raden.
        put32(p, n, g_ptpv1.unlocks);       n += 4;
        put32(p, n, g_ptpv1.step_count);    n += 4;
        // En de RX-timestamp-ring. Hij staat al in het 'u'-antwoord, maar dat
        // heeft voorwaardelijke velden en dus geen vaste offset -- precies de
        // fout waar never-sum/stale-offset-notities over gaan. Hier staat hij
        // naast de unlock-teller, want een resync verschuift de ring een
        // entry en dan hoort een timestamp bij een ANDER frame: de fout is dan
        // het gat tussen twee frames, en Sync->FollowUp is ~17 us.
        put32(p, n, rx_ts_resyncs);         n += 4;
        put32(p, n, rx_pops);               n += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
        return;
    }

    // 'r' -- RING HEALTH. Its own opcode, not four more words on the main
    // reply: growing that one from 200 to 208 bytes once killed the port
    // outright (see the top of this file). A different reply is not a bigger
    // one.
    //
    // WHY THIS EXISTS. fifo_level reads block_level, which CLAMPS a negative
    // ring level to 0. The 2026-08 wedges each presented as fifo_level 0 with
    // every ingress counter perfect, and the single number that would have
    // named the bug on its first occurrence -- a level of -4,294,967,232 --
    // was the one the clamp threw away. Three investigations concluded "the
    // decoder is producing nothing" from that healthy-looking 0. These four
    // words are the unclamped truth.
    //
    // READ ring_underflow FIRST. It is an EPISODE count, not a tick count, and
    // it must be 0. The hysteresis recovers an inverted ring in ~11 ms (level
    // < FRAME_SAMPLES un-primes, rd holds, wr catches up), so a nonzero value
    // is a glitch nobody heard rather than a silence somebody did -- but it
    // means the route exists, which 48-bit pointers are supposed to make
    // impossible.
#ifdef CSR_AAF_PKT_RING_LEVEL_RAW_ADDR
    if (len >= 1 && req[0] == 'r') {
        uint8_t *p = net_udp_payload_buf();
        uint32_t n = 0;
        put32(p, n, 0x524E4731u); n += 4;          // 'RNG1'
        put32(p, n, aaf_pkt_ring_level_raw_read()); n += 4;   // signed, 2's compl
        put32(p, n, aaf_pkt_ring_level_min_read()); n += 4;   // signed, sticky
        put32(p, n, aaf_pkt_ring_underflow_read()); n += 4;   // MUST be 0
        put32(p, n, aaf_pkt_ring_unprime_read());   n += 4;
        // Context, so a reader never has to correlate two replies by hand:
        // the clamped level it would otherwise have believed, and the tick
        // count that says how long the un-primed episodes lasted in total.
        put32(p, n, aaf_pkt_fifo_level_read());     n += 4;
        put32(p, n, aaf_pkt_underrun_count_read()); n += 4;
        // AND WHETHER THE TALKER HAS EVER RUN. Without it a reader cannot tell
        // "the ring inverted" from "the ring was never driven": with en low, rd
        // is parked and the modular level sweeps its whole range, which read as
        // an inversion on the first boot of this firmware.
        put32(p, n, g_tx_stats.enables);            n += 4;
        // RAW PEAK READBACK, for one question only: is the gateware array zero,
        // or is the readout path broken? The heartbeat reported 255 (silence)
        // on all 48 channels while USB ingress was a perfect 18,431,934 B/s, and
        // PEAK_DB[0] is 255, so the firmware read 0 -- but that does not say
        // whether the detector never fired or peak_select never took effect.
        // Four channels plus a select read-back answers it.
        for (unsigned ch = 0; ch < 4; ch++) {
            aaf_pkt_peak_select_write(ch);
            put32(p, n, aaf_pkt_peak_value_read()); n += 4;
        }
        aaf_pkt_peak_select_write(3);
        put32(p, n, aaf_pkt_peak_select_read());    n += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
        return;
    }
#endif
    // 'o' [ASCII signed decimal] -- set the TX timestamp offset in samples and
    // re-anchor. Replies with the value now in force so a sweep can confirm the
    // write landed rather than assuming it did.
    // 'F' [ASCII decimal] -- hold the USB async-feedback value (0 = auto loop).
    //
    // The wrapper exposes this precisely to sweep hardcoded feedback live and
    // characterise the host's response without a rebuild. There is a console
    // 'F' command already, but driving the UART perturbs the system under
    // measurement, so this is the same knob over UDP.
    //
    // Value is Q16.16 samples per microframe; nominal 6.0 = 393216.
    // 'u' -- USB ingress localisation. rx_beats is what the core hands the
    // endpoint; ep_out is what the endpoint hands the decoder. A gap between
    // them is audio arriving over USB and never reaching the ring, which is a
    // LEAK, not a rate error -- and a leak is what the feedback loop has been
    // silently compensating for.
    // 'M' [ASCII fpp] -- set the fpp used for ARC-created multicast bundles,
    // and (with no digits) create one on channels 1-2 so a stream at any fpp can
    // be decoded from the build host without Dante Controller.
    // 'P' [ASCII] -- pacing phase seed adjustment, swept live. It was a
    // #define tuned at fpp=16; at fpp=60 the emitted subsec comes out congruent
    // to 59 (i.e. -1) with the same value, so the correction is not the simple
    // fixed pipeline delay it was assumed to be.
    if (len >= 1 && req[0] == 'P') {
        if (len >= 2 && req[1] >= '0' && req[1] <= '9') {
            uint32_t v = 0; uint32_t i = 1;
            for (; i < len && req[i] >= '0' && req[i] <= '9'; i++)
                v = v * 10u + (uint32_t)(req[i] - '0');
            g_phase_adj = v;
        }
        uint8_t *p7 = net_udp_payload_buf();
        uint32_t n7 = 0;
        put32(p7, n7, 0x50485631u); n7 += 4;              // 'PHV1'
        put32(p7, n7, g_phase_adj); n7 += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n7, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'M') {
        if (len >= 2 && req[1] >= '0' && req[1] <= '9') {
            uint16_t v = 0; uint32_t i = 1;
            for (; i < len && req[i] >= '0' && req[i] <= '9'; i++)
                v = (uint16_t)(v * 10 + (req[i] - '0'));
            if (dante_tx_fpp_supported(v)) g_mcast_fpp = v;
        } else {
            // FOUR channels, matching what DVS negotiates on unicast. The
            // remaining failure is DVS-unicast-only, and fpp=60 is proven good
            // (DVS has audio on a fpp=60 multicast), so slot count is the next
            // variable: 2 slots is 360 B of payload, 4 slots is 720 B.
            uint16_t ch[4] = { 1, 2, 3, 4 };
            dante_tx_bind_multicast(0x40, ch, 4);
        }
        uint8_t *p6 = net_udp_payload_buf();
        uint32_t n6 = 0;
        put32(p6, n6, 0x4D434131u); n6 += 4;              // 'MCA1'
        put32(p6, n6, g_mcast_fpp); n6 += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n6, NET_TOS_BEST_EFFORT);
        return;
    }

    // 'L'            -> report advertised latency_ns
    // 'L' + u32 BE   -> set it and re-announce mDNS
    //
    // Runtime-settable so a latency target can be tried and REVERTED in seconds
    // without a firmware push, which matters because advertising too little is
    // a SILENT failure: receivers drop packets that miss the playout deadline
    // and every transmit-side counter stays green.
    // 'A' + 4 bytes IP -> mirror every ARC request to that host on port 7780.
    // 'A' alone         -> report the current collector.
    // 'K' + u16 key + u16 value -> patch the 0x1100 property table in RAM.
    // Finds which capability key gates Dante Controller's latency list.
    // 'G' + u8 flags0 + u8 flags2 -> patch the 0x1000 capability word.
    // 'V' + u16 router_vers + u16 arcp_vers -> patch the 0x1003 version fields.
    if (len >= 5 && req[0] == 'V') {
        g_router_vers = (uint16_t)((req[1] << 8) | req[2]);
        g_arcp_vers   = (uint16_t)((req[3] << 8) | req[4]);
        printf("[arc] vers router=%04x arcp=%04x\n", g_router_vers, g_arcp_vers);
        uint8_t *pv = net_udp_payload_buf();
        uint32_t nv = 0;
        put32(pv, nv, 0x56455231u); nv += 4;               // 'VER1'
        net_udp_commit(src_ip, src_port, STATS_PORT, nv, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 3 && req[0] == 'G') {
        g_dev_flags0 = req[1];
        g_dev_flags2 = req[2];
        printf("[arc] 1000 caps = %02x %02x\n", g_dev_flags0, g_dev_flags2);
        uint8_t *pg = net_udp_payload_buf();
        uint32_t ng = 0;
        put32(pg, ng, 0x43415031u); ng += 4;               // 'CAP1'
        pg[ng++] = g_dev_flags0; pg[ng++] = g_dev_flags2;
        net_udp_commit(src_ip, src_port, STATS_PORT, ng, NET_TOS_BEST_EFFORT);
        return;
    }

    // 'B' + u16 key + u32 value -> patch a u32 in the 0x1100 data blob.
    if (len >= 7 && req[0] == 'B') {
        uint16_t k = (uint16_t)((req[1] << 8) | req[2]);
        uint32_t v = ((uint32_t)req[3] << 24) | ((uint32_t)req[4] << 16) |
                     ((uint32_t)req[5] << 8)  |  (uint32_t)req[6];
        int r = dante_arc_patch_1100_u32(k, v);
        uint8_t *pb = net_udp_payload_buf();
        uint32_t nb = 0;
        put32(pb, nb, r ? 0x424c4242u : 0x424c424fu);      // 'BLBB' / 'BLBO'
        nb += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, nb, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 5 && req[0] == 'K') {
        uint16_t k = (uint16_t)((req[1] << 8) | req[2]);
        uint16_t v = (uint16_t)((req[3] << 8) | req[4]);
        int r = dante_arc_patch_1100(k, v);
        uint8_t *pk = net_udp_payload_buf();
        uint32_t nk = 0;
        put32(pk, nk, r ? 0x4b455242u : 0x4b45594fu);   // 'KERB' bad / 'KEYO' ok
        nk += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, nk, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'A') {
        if (len >= 5) {
            for (int i = 0; i < 4; i++) g_arc_mirror_ip[i] = req[1 + i];
            printf("[arc] mirror -> %u.%u.%u.%u:7780\n",
                   g_arc_mirror_ip[0], g_arc_mirror_ip[1],
                   g_arc_mirror_ip[2], g_arc_mirror_ip[3]);
        }
        uint8_t *pa = net_udp_payload_buf();
        uint32_t na = 0;
        put32(pa, na, 0x4d495231u); na += 4;            // 'MIR1'
        for (int i = 0; i < 4; i++) pa[na++] = g_arc_mirror_ip[i];
        net_udp_commit(src_ip, src_port, STATS_PORT, na, NET_TOS_BEST_EFFORT);
        return;
    }

    // 'X' + u8 -> largest fpp we accept in a flow request (60 = anything).
    // 'Y' + u8 -> fpp maximum advertised in the channel record.
    // 'Z' + 0|1 -> auto-raise the advertised latency to cover bound flows.
    // 'C' + 0|1 -> clamp an oversized fpp request instead of rejecting it.
    if (len >= 2 && req[0] == 'C') {
        g_fpp_clamp = req[1] ? 1 : 0;
        printf("[flow] oversized fpp: %s\n", g_fpp_clamp ? "CLAMP" : "reject");
        dante_tx_drop_all();
        uint8_t *pc = net_udp_payload_buf();
        uint32_t nc = 0;
        put32(pc, nc, 0x434c4d31u); nc += 4;               // 'CLM1'
        pc[nc++] = g_fpp_clamp;
        net_udp_commit(src_ip, src_port, STATS_PORT, nc, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 2 && req[0] == 'Z') {
        g_latency_autoraise = req[1] ? 1 : 0;
        mdns_announce();
        printf("[lat] auto-raise = %u\n", g_latency_autoraise);
        uint8_t *pz = net_udp_payload_buf();
        uint32_t nz = 0;
        put32(pz, nz, 0x4c415a31u); nz += 4;               // 'LAZ1'
        pz[nz++] = g_latency_autoraise;
        net_udp_commit(src_ip, src_port, STATS_PORT, nz, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 2 && req[0] == 'Y') {
        // 0 now means AUTO (scale with the sample rate), not 8. Advertising a
        // rate-independent constant is the bug this replaced.
        g_fpp_adv_max = req[1];
        mdns_announce();
        printf("[flow] advertised fpp max = %u%s\n", dante_fpp_adv_max(),
               g_fpp_adv_max ? " (override)" : " (auto, scaled to rate)");
        uint8_t *py = net_udp_payload_buf();
        uint32_t ny = 0;
        put32(py, ny, 0x46505941u); ny += 4;               // 'FPYA'
        py[ny++] = dante_fpp_adv_max();                    // effective, not raw
        net_udp_commit(src_ip, src_port, STATS_PORT, ny, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 2 && req[0] == 'X') {
        g_fpp_max_accept = req[1] ? req[1] : 60;
        printf("[flow] max accepted fpp = %u\n", g_fpp_max_accept);
        dante_tx_drop_all();          // force receivers to renegotiate now
        uint8_t *px = net_udp_payload_buf();
        uint32_t nx = 0;
        put32(px, nx, 0x46505831u); nx += 4;               // 'FPX1'
        px[nx++] = g_fpp_max_accept;
        net_udp_commit(src_ip, src_port, STATS_PORT, nx, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'L') {
        if (len >= 5) {
            uint32_t v = ((uint32_t)req[1] << 24) | ((uint32_t)req[2] << 16) |
                         ((uint32_t)req[3] << 8)  |  (uint32_t)req[4];
            // Floor at one packet window for the largest fpp we ACCEPT (60 =
            // 1.25 ms), not the largest we advertise (8). Those differ, and the
            // accepted one is what can actually be on the wire.
            if (v >= 100000u && v <= 40000000u) {
                g_latency_ns = v;
                mdns_announce();
                printf("[lat] latency_ns = %lu\n", (unsigned long)v);
            } else {
                printf("[lat] rejected %lu (range 100000..40000000)\n",
                       (unsigned long)v);
            }
        }
        uint8_t *pl = net_udp_payload_buf();
        uint32_t nl = 0;
        put32(pl, nl, 0x4c415431u); nl += 4;            // 'LAT1'
        put32(pl, nl, g_latency_ns); nl += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, nl, NET_TOS_BEST_EFFORT);
        return;
    }

    // 'U' -- DUAL USB A/B source: select and report.
    //
    //   'U'        -> status only
    //   'U' '0'    -> request Main (USB1)
    //   'U' '1'    -> request Backup (USB2)
    //
    // THE SWITCH IS FRAME-ALIGNED IN GATEWARE, so a write is a REQUEST, not an
    // immediate change: usb_source_live follows usb_source_select only once the
    // candidate presents a channel-0-of-frame marker. A blind mid-frame switch
    // would hand the ring a partial 48-channel frame and rotate every channel
    // permanently -- see the handover comment in avb_soc.py. So `select` and
    // `live` differing is NORMAL for a few microseconds, and differing
    // PERSISTENTLY means the candidate is not delivering frames at all.
    //
    // The Backup's own ingress counters are reported alongside, because a hot
    // standby you cannot measure is a standby on trust: switching to a dead
    // Backup trades working audio for silence. Check rx_beats/ep_out are live
    // BEFORE requesting the switch.
    //
    // Separate opcode with its own bounded reply, per the rule at the top of
    // this file: growing the main reply 200 -> 208 bytes killed that port once.
    if (len >= 1 && req[0] == 'U') {
#ifdef CSR_MAIN_USB_SOURCE_LIVE_ADDR
        if (len >= 2) {
            if (req[1] == '0' || req[1] == '1') {
                main_usb_source_select_write(req[1] - '0');
                printf("[usb] source REQUEST -> %s (aligned if the candidate "
                       "delivers a frame, FORCED after the timeout)\n",
                       req[1] == '1' ? "Backup(USB2)" : "Main(USB1)");
            }
        }
        uint8_t *pu = net_udp_payload_buf();
        uint32_t nu = 0;
        put32(pu, nu, 0x55534232u);                        nu += 4;  // 'USB2'
        put32(pu, nu, main_usb_source_select_read());      nu += 4;
        put32(pu, nu, main_usb_source_live_read());        nu += 4;
        put32(pu, nu, main_usb_switch_count_read());       nu += 4;
        put32(pu, nu, main_usb_dbg_rx_beats_read());       nu += 4;  // Main
        put32(pu, nu, main_usb_dbg_ep_out_read());         nu += 4;
        put32(pu, nu, main_usb_sample_overflow_read());    nu += 4;
        put32(pu, nu, main_usb2_dbg_rx_beats_read());      nu += 4;  // Backup
        put32(pu, nu, main_usb2_dbg_ep_out_read());        nu += 4;
        put32(pu, nu, main_usb2_sample_overflow_read());   nu += 4;
        // APPENDED: geforceerde switches. Het VERSCHIL met switch_count is de
        // diagnose -- een geforceerde switch betekent dat de kandidaat geen
        // frame-marker aanbood, dus dat de standby niets leverde. Dat wil je
        // achteraf kunnen zien in plaats van vermoeden.
#ifdef CSR_MAIN_USB_SWITCH_FORCED_ADDR
        put32(pu, nu, main_usb_switch_forced_read());      nu += 4;
#else
        put32(pu, nu, 0);                                  nu += 4;
#endif
        // APPENDED: ring-vol write-drops, apart van overrun_count.
        //
        // Dit is het ENIGE getal in dit antwoord dat "er gaat audio verloren"
        // betekent. overrun_count (het '?'-antwoord, woord 3) telde tot
        // 2026-11-03 ook de benigne tak mee -- send_req die midden in een
        // traversal landt, wat pend[] alsnog serveert -- en liep daardoor op een
        // gezonde 96 kHz-stroom met 11k-28k/s terwijl er niets misging. Hier
        // staat alleen de hoorbare tak: een USB-sample dat op een volle ring
        // viel, dus een kanaalslot dat een oude waarde hield = een klik.
        // MOET 0 blijven.
#ifdef CSR_AAF_PKT_WRITE_DROP_COUNT_ADDR
        put32(pu, nu, aaf_pkt_write_drop_count_read());    nu += 4;
#else
        put32(pu, nu, 0);                                  nu += 4;
#endif
        net_udp_commit(src_ip, src_port, STATS_PORT, nu, NET_TOS_BEST_EFFORT);
#else
        // Single-USB build (USB2 not set at build time). Answer rather than
        // time out, so the tool can say "not built with dual USB" instead of
        // looking like a dead board.
        uint8_t *pu = net_udp_payload_buf();
        uint32_t nu = 0;
        put32(pu, nu, 0x55534231u); nu += 4;               // 'USB1' = single
        net_udp_commit(src_ip, src_port, STATS_PORT, nu, NET_TOS_BEST_EFFORT);
#endif
        return;
    }

    // 'T' -- ULPI input IDELAY taps over UDP.
    //
    //   "T"            -> report both taps
    //   "T" <m>        -> set Main's tap to m   (0..31)
    //   "T" <m> <b>    -> set Main's and Backup's taps
    //
    // The console 'u' key already steps the Main tap, but it only increments,
    // it needs a physical re-plug to re-test, and on 2026-10-30 the serial
    // console went completely silent (both 1 Mbaud and 115200, no unsolicited
    // output) while UDP kept answering -- so the one documented tuning knob for
    // the 60 MHz ULPI eye was unreachable. Over UDP it is scriptable, and the
    // hub port can be power-cycled from sysfs (usb1-port8/disable) instead of
    // by hand, which makes a full 32-tap sweep unattended.
    //
    // WHY PER-BUS TAPS: the eye centre is a property of each PHY's own wiring
    // and of ITS paths' placement, not of the chip type. avb-usb-host measured
    // tap 8 working on the Backup PHY while the SAME tap left Main decoding SOFs
    // cleanly but never asserting its D+ pull-up -- reads tolerate ~0.6 ns of
    // extra nxt/data delay, the write handshake (our driven data against the
    // PHY's NXT) does not. So never assume one tap fits both.
    // 'R' -- sample rate lezen/zetten zonder Dante Controller.
    //
    //   "R"        -> rapporteer de actieve rate
    //   "R" <code> -> zet 48 kHz (code 0) of 96 kHz (code 1), bewaar, herstart
    //
    // Bestaat omdat de rate normaal uit DC komt (info 0x81) en dat pad niet op
    // de bench te scripten is. Doet exact wat handle_rate_set() doet: opslaan en
    // herstarten, want de NCO, de USB-clock-source en de subsecond-wrap worden
    // bij boot uit g_fs opgezet.
    if (len >= 1 && req[0] == 'R') {
        uint8_t *pr = net_udp_payload_buf();
        uint32_t nr = 0;
        put32(pr, nr, 0x52415445u); nr += 4;               // 'RATE'
        put32(pr, nr, g_fs);        nr += 4;
        if (len >= 2) {
            uint32_t want = (req[1] == 1) ? 96000u : 48000u;
            put32(pr, nr, want);    nr += 4;
            net_udp_commit(src_ip, src_port, STATS_PORT, nr, NET_TOS_BEST_EFFORT);
            if (want != g_fs) {
                g_cfg.sample_rate = want;
                cfg_save();
                // LIVE, geen reset. Een reset landt in de loader omdat de
                // firmware in coderam staat en niet in flash -- gemeten: het
                // bord kwam niet terug, ook niet met netload. Zie
                // rate_apply_live() in dante_info.c.
                dante_info_rate_apply(want);
                printf("[rate] -> %lu Hz via UDP (live; replug de USB om de "
                       "host de nieuwe rate te laten lezen)\n",
                       (unsigned long)want);
            }
            return;
        }
        put32(pr, nr, g_fs);        nr += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, nr, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'T') {
        if (len >= 2) {
            main_ulpi_idelay_tap_write(req[1] & 0x1F);
            main_ulpi_idelay_load_write(1);
        }
#ifdef CSR_MAIN_ULPI2_IDELAY_TAP_ADDR
        if (len >= 3) {
            main_ulpi2_idelay_tap_write(req[2] & 0x1F);
            main_ulpi2_idelay_load_write(1);
        }
#endif
        uint8_t *pt = net_udp_payload_buf();
        uint32_t nt = 0;
        put32(pt, nt, 0x54415031u); nt += 4;               // 'TAP1'
        pt[nt++] = (uint8_t)main_ulpi_idelay_tap_read();
#ifdef CSR_MAIN_ULPI2_IDELAY_TAP_ADDR
        pt[nt++] = (uint8_t)main_ulpi2_idelay_tap_read();
        pt[nt++] = 1;                                      // dual USB built
#else
        pt[nt++] = 0;
        pt[nt++] = 0;
#endif
        net_udp_commit(src_ip, src_port, STATS_PORT, nt, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'u') {
        uint8_t *p5 = net_udp_payload_buf();
        uint32_t n5 = 0;
        put32(p5, n5, 0x55534231u); n5 += 4;                      // 'USB1'
        put32(p5, n5, main_usb_dbg_rx_beats_read()); n5 += 4;
        put32(p5, n5, main_usb_dbg_ep_out_read());   n5 += 4;
        put32(p5, n5, aaf_pkt_overrun_count_read()); n5 += 4;
        put32(p5, n5, aaf_pkt_underrun_count_read());n5 += 4;
        // APPENDED so the ring-loses-prime bug is diagnosable WITHOUT a console.
        //
        // It has now been caught live twice (2026-08-12, 2026-08-13) and both
        // times the decisive counters were only on the UART, which cannot be
        // read while openocd holds the CH347. So both occurrences ended in
        // "signature recorded, cause not established" for want of three
        // registers that were already in the CSR map.
        //
        //   sample_overflow  the FIFO between the USB endpoint and the ring.
        //                    Climbing = data reaching the endpoint but NOT the
        //                    ring, which is exactly the 2026-08-12 signature
        //                    (rx_beats = ep_out = 9.3 M/s, fifo_level 0).
        //   fb_ovr           the async feedback value handed to the host. If
        //                    this is out of the host's accept window the host
        //                    can stop streaming, which is a candidate for the
        //                    2026-08-13 signature (rx_beats 0).
        //   block_level      the feedback servo's INPUT, un-decimated. fifo_level
        //                    is the ring; this is what the servo actually sees.
        put32(p5, n5, main_usb_sample_overflow_read()); n5 += 4;
        put32(p5, n5, main_usb_fb_ovr_read());          n5 += 4;
        put32(p5, n5, aaf_pkt_fifo_level_read());       n5 += 4;

        // IS THE DECODER PRODUCING SAMPLES AT ALL?
        //
        // This is the one measurement that separates the two remaining
        // explanations for the ring-loses-prime wedge, and it needs no gateware
        // change -- usb_sample_data already exposes the USB->AAF FIFO head, with
        // bit 7 = valid ("FIFO non-empty, head is real", avb_soc.py:823).
        //
        //   valid never set, while rx_beats/ep_out stream at 9.3 MB/s
        //       -> the decoder is eating bytes and emitting no samples, so the
        //          ring write pointer never advances. Frame-alignment loss.
        //   valid set, with fifo_level stuck at 0
        //       -> the decoder IS producing and the packetizer is not draining
        //          it into the ring. A different fault, in the other module.
        //
        // Sampled 256 times rather than once: in normal operation the head is
        // drained continuously so `valid` flickers, and a single snapshot would
        // be a coin toss. The COUNT is the signal -- near 0 or near 256 both
        // mean something, 128 means it is running.
        //
        // READ-ONLY, DELIBERATELY. usb_sample_pop is a CSRStorage that advances
        // the FIFO read pointer; writing it here would steal samples from the
        // packetizer and corrupt audio to take a measurement. Never pop.
        {
            uint32_t vcount = 0, head = 0;
            for (unsigned i = 0; i < 256u; i++) {
                head = main_usb_sample_data_read();
                if (head & 0x80u) vcount++;
            }
            put32(p5, n5, vcount); n5 += 4;   // 0..256 with bit7 (valid) set
            put32(p5, n5, head);   n5 += 4;   // last raw head word
        }
        // THE ONE THAT ACTUALLY NAMES THE FAULT. Frames whose channel-0 marker
        // the packetizer had to synthesise because the USB wrapper stopped
        // sending one. Nonzero = the condition that killed the ring three times
        // (2026-08-12, twice 2026-08-13) has occurred; the flywheel in
        // dante_packetizer.py now rides through it instead of wedging, so this
        // climbing WITH audio still flowing is the expected new behaviour.
        // Climbing fast = the marker is gone, not glitching.
        // GUARDED so one firmware source builds against BOTH CSR maps. Adding
        // this CSR shifted 48 addresses, so a firmware built for the flywheel
        // bitstream pokes moved registers on build_seed8 and vice versa. The
        // guard lets a revert to seed8 be a rebuild rather than a source edit.
#ifdef CSR_AAF_PKT_HOLDOVER_COUNT_ADDR
        put32(p5, n5, aaf_pkt_holdover_count_read()); n5 += 4;
        put32(p5, n5, rx_ts_resyncs);  n5 += 4;   // stale ts entries discarded
        put32(p5, n5, rx_pops);        n5 += 4;   // pops issued, for the ratio
        put32(p5, n5, g_lvl_pre);      n5 += 4;   // ring level seen at drain
#endif
        // THE AUDIBLE ONE, SEPARATED FROM THE BENIGN ONE.
        //
        // overrun_count (word 3 above) used to be overruns + write_drops summed.
        // On 2026-11-03 that sum read 11k-28k/s on a HEALTHY 96 kHz stream --
        // `packets` exactly 24000/s, underrun 0, ring never within 140 samples of
        // its rail -- and it cost a debug session to establish that the whole
        // rate was the benign branch (send_req landing mid-traversal, which
        // pend[] re-serves). write_drop_count is the branch that is actually
        // audible: a USB sample dropped on a full ring leaves one channel slot
        // stale = a per-channel click. It MUST read 0. Own guard so one firmware
        // source still builds against a bitstream without the CSR.
#ifdef CSR_AAF_PKT_WRITE_DROP_COUNT_ADDR
        put32(p5, n5, aaf_pkt_write_drop_count_read()); n5 += 4;
#endif
        net_udp_commit(src_ip, src_port, STATS_PORT, n5, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'F') {
        if (len >= 2) {
            uint32_t v = 0; uint32_t i = 1; int digits = 0;
            for (; i < len && req[i] >= '0' && req[i] <= '9'; i++) {
                v = v * 10u + (uint32_t)(req[i] - '0'); digits++;
            }
            // Set the FIRMWARE variable, not the CSR: the main loop rewrites
            // the CSR from usb_fb_manual every iteration.
            // A sweep takes manual control; writing 0 hands it back to the
            // firmware outer loop.
            if (digits) { usb_fb_manual = v; usb_fb_sweep_hold = (v != 0); }
        }
        uint8_t *p4 = net_udp_payload_buf();
        uint32_t n4 = 0;
        put32(p4, n4, 0x46425631u); n4 += 4;                 // 'FBV1'
        put32(p4, n4, usb_fb_manual); n4 += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n4, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 'o') {
        if (len >= 2) {
            int32_t v = 0, sign = 1; uint32_t i = 1;
            if (req[i] == '-') { sign = -1; i++; }
            else if (req[i] == '+') { i++; }
            int digits = 0;
            for (; i < len && req[i] >= '0' && req[i] <= '9'; i++) {
                v = v * 10 + (req[i] - '0'); digits++;
            }
            if (digits) dante_tx_set_ts_offset(sign * v);
        }
        uint8_t *p2 = net_udp_payload_buf();
        uint32_t n2 = 0;
        put32(p2, n2, 0x54534F31u); n2 += 4;                     // 'TSO1'
        put32(p2, n2, (uint32_t)dante_tx_get_ts_offset()); n2 += 4;
        net_udp_commit(src_ip, src_port, STATS_PORT, n2, NET_TOS_BEST_EFFORT);
        return;
    }

    if (len >= 1 && req[0] == 't') {
        void telem_drain(const uint8_t *, uint16_t, const uint8_t *, uint32_t);
        telem_drain(src_ip, src_port, req, len);
        return;
    }
    if (len >= 1 && req[0] == 'm') {
        mclk_rx(src_ip, src_port, req, len);
        return;
    }
    (void)req; (void)len;

    uint8_t *p = net_udp_payload_buf();
    uint32_t n = 0;
    put32(p, n, dante_tx_enabled());                   n += 4;   // 0
    put32(p, n, aaf_pkt_packet_count_read());          n += 4;   // 1
    put32(p, n, aaf_pkt_underrun_count_read());        n += 4;   // 2
    put32(p, n, aaf_pkt_overrun_count_read());         n += 4;   // 3
    put32(p, n, aaf_pkt_fifo_level_read());            n += 4;   // 4
    put32(p, n, aaf_pkt_dbg_last_sec_read());          n += 4;   // 5
    put32(p, n, aaf_pkt_dbg_last_ts_read());           n += 4;   // 6
    put32(p, n, g_tx_stats.anchors);                   n += 4;   // 7
    put32(p, n, g_tx_stats.enables);                   n += 4;   // 8
    put32(p, n, g_tx_stats.disables);                  n += 4;   // 9
    put32(p, n, g_ptpv1.locked);                       n += 4;   // 10
    put32(p, n, (uint32_t)(int32_t)g_ptpv1.offset_ns); n += 4;   // 11
    put32(p, n, (uint32_t)(int32_t)g_ptpv1.mean_path_delay_ns); n += 4; // 12
    put32(p, n, ethmac_sram_writer_errors_read());     n += 4;   // 13
    put32(p, n, ethmac_rx_datapath_crc_errors_read()); n += 4;   // 14

    // PTP time alongside the last EMITTED timestamp, so drift between the media
    // clock and PTP can be measured directly. The media clock free-runs from a
    // single anchor; nothing here reported how far it had wandered.
    {
        ptp_timestamp_t t = gptp_read_time();
        put32(p, n, (uint32_t)t.seconds);                      n += 4;
        // ns -> samples op g_fs; 62500 was de 48 kHz-waarde en gaf op 96 kHz
        // de helft. Zie dante_tx.c regel ~267.
        put32(p, n, (uint32_t)((t.nanoseconds * 3u) / (3000000000u / g_fs))); n += 4;
    }
    // trim_ppb/drift removed from this reply: adding them killed the port
    // outright (no response at all, while ARC 4440 and flow control 4455 kept
    // answering and ping was clean). Cause not identified. The servo still runs
    // and still applies the trim -- only the reporting is backed out, so the
    // endpoint stays usable for measuring drift, which is the thing that
    // actually matters here.
    put32(p, n, dante_tx_active());                 n += 4;   // 15
    put32(p, n, g_flows_stats.requests);               n += 4;   // 16
    put32(p, n, g_flows_stats.rejected);               n += 4;   // 17
    // Then 3 words per flow: in_use, age since last keepalive, rebind count.
    for (unsigned f = 0; f < 6; f++) {
        uint8_t iu; uint32_t age, rb;
        dante_tx_flow_info(f, &iu, &age, &rb);
        put32(p, n, iu);  n += 4;
        put32(p, n, age); n += 4;
        put32(p, n, rb);  n += 4;
        uint8_t mac[6]; dante_tx_flow_mac(f, mac);
        put32(p, n, ((uint32_t)mac[0]<<24)|((uint32_t)mac[1]<<16)|
                    ((uint32_t)mac[2]<<8)|mac[3]);          n += 4;
        put32(p, n, ((uint32_t)mac[4]<<8)|mac[5]);          n += 4;
        uint8_t iu2; uint32_t a2, r2;
        (void)iu2; (void)a2; (void)r2;
    }

    net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
}

void dante_stats_init(void)
{
    if (net_udp_bind(STATS_PORT, stats_rx) != 0) {
        printf("[stats] BIND FAILED on %u\n", STATS_PORT);
        return;
    }
    printf("[stats] counters on udp %u\n", STATS_PORT);
}
