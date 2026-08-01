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
#include "dante_tx.h"
#include "gptp.h"
#include "ptpv1.h"
#include "dante_flows.h"
#include "dante_tx.h"
#include "gptp.h"
#include <generated/csr.h>
#include <stdio.h>

#define STATS_PORT 7779

static void put32(uint8_t *p, uint32_t n, uint32_t v)
{
    p[n] = (uint8_t)(v >> 24); p[n+1] = (uint8_t)(v >> 16);
    p[n+2] = (uint8_t)(v >> 8); p[n+3] = (uint8_t)v;
}

static void stats_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                     uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip; (void)req; (void)len;

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
