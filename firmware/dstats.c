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
extern uint32_t usb_fb_manual;
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
#include <stdio.h>

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
        put32(p, n, 0x464C5731u); n += 4;          // 'FLW1'
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
            for (unsigned i = 0; i < 8; i += 2)
                { put32(p, n, ((uint32_t)d.chans[i] << 16) | d.chans[i+1]); n += 4; }
        }
        net_udp_commit(src_ip, src_port, STATS_PORT, n, NET_TOS_BEST_EFFORT);
        return;
    }
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
    if (len >= 1 && req[0] == 'u') {
        uint8_t *p5 = net_udp_payload_buf();
        uint32_t n5 = 0;
        put32(p5, n5, 0x55534231u); n5 += 4;                      // 'USB1'
        put32(p5, n5, main_usb_dbg_rx_beats_read()); n5 += 4;
        put32(p5, n5, main_usb_dbg_ep_out_read());   n5 += 4;
        put32(p5, n5, aaf_pkt_overrun_count_read()); n5 += 4;
        put32(p5, n5, aaf_pkt_underrun_count_read());n5 += 4;
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
        put32(p, n, (uint32_t)((t.nanoseconds * 3u) / 62500u)); n += 4;
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
