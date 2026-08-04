// rx_gate.c — control and readout for the gateware RX MAC allow-list.
//
// The gateware classifies every RX frame's destination MAC whether or not the
// filter is armed, so the honest sequence here is:
//
//   1. the classifier runs whether or not the filter is armed, so nomatch_count
//      is always a live account of what is being dropped (or would be);
//   2. arm it and watch discard_count move -- that is the proof the enable
//      actually reached the gateware, rather than inferring it from a counter
//      that would have moved anyway;
//   3. watch ethmac_sram_writer_errors stop climbing.
//
// AUTO-REVERT. A wrong allow-list drops all RX, including the packet that would
// carry "turn it off". So a network-requested arm is provisional: it undoes
// itself after a timeout unless explicitly committed. The console path does not
// need this -- the UART cannot be locked out by a MAC filter -- so it arms
// permanently.

#include "rx_gate.h"
#include "gptp.h"          // gptp_uptime_ms()

#include <generated/csr.h>
#include <stdio.h>

static uint8_t  s_enabled;
static uint8_t  s_pending;
static uint32_t s_revert_at_ms;

static void write_local_mac(const uint8_t mac[6])
{
    rx_gate_local_mac_hi_write(((uint32_t)mac[0] << 8) | mac[1]);
    rx_gate_local_mac_lo_write(((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                               ((uint32_t)mac[4] << 8)  |  (uint32_t)mac[5]);
}

void rx_gate_init(const uint8_t mac[6])
{
    // Program the unicast address FIRST and unconditionally. If this is ever
    // skipped, arming the filter drops every unicast frame -- ARC, the stats
    // port, netload -- which is the one failure mode worth engineering against.
    write_local_mac(mac);

    // ARM AT BOOT.
    //
    // This used to leave the filter off, so it had to be re-armed by hand after
    // every reboot. On 2026-08-04 that cost hours: several reflashes in a row
    // came up disarmed, which put 21% control-plane frame loss back on the PTP
    // path. PTP then took minutes to lock or never locked at all, Dante
    // Controller stayed amber, and the talker never started -- and the symptom
    // was misdiagnosed twice as a servo noise floor before someone checked
    // whether the gate was actually on.
    //
    // The safety case for defaulting off has been met by other means: the
    // gateware interlock keeps the filter inert until this function has
    // programmed local_mac (which happens immediately above), the allow-list is
    // a proven superset of the software filter, and 0/400 control round-trips
    // were lost across an extended armed run. The escape hatches remain: the
    // 'x' console command, which a MAC filter cannot lock out, and
    // tools/rx_gate.py off.
    rx_gate_enable_write(1);
    s_enabled = 1;
    s_pending = 0;

    printf("[rxgate] armed=1 (auto) local=%02x:%02x:%02x:%02x:%02x:%02x "
           "(allow: bcast, self, mDNS, PTPv1, Dante info+heartbeat)\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void rx_gate_set(int on, uint32_t revert_ms)
{
    rx_gate_enable_write(on ? 1 : 0);
    s_enabled = on ? 1 : 0;

    if (on && revert_ms) {
        s_pending      = 1;
        s_revert_at_ms = gptp_uptime_ms() + revert_ms;
        printf("[rxgate] armed, auto-revert in %lu ms unless committed\n",
               (unsigned long)revert_ms);
    } else {
        s_pending = 0;
        printf("[rxgate] %s\n", on ? "armed" : "disarmed");
    }
}

void rx_gate_commit(void)
{
    if (s_pending) {
        s_pending = 0;
        printf("[rxgate] committed, auto-revert cancelled\n");
    }
}

void rx_gate_poll(void)
{
    if (!s_pending)
        return;
    // Unsigned wrap-safe comparison: the difference goes negative-as-huge only
    // after the deadline passes.
    if ((int32_t)(gptp_uptime_ms() - s_revert_at_ms) >= 0) {
        s_pending = 0;
        rx_gate_enable_write(0);
        s_enabled = 0;
        printf("[rxgate] AUTO-REVERTED to disabled (never committed)\n");
    }
}

void rx_gate_get_status(rx_gate_status_t *out)
{
    uint32_t hi = rx_gate_last_drop_hi_read();
    uint32_t lo = rx_gate_last_drop_lo_read();

    out->enabled        = s_enabled;
    out->pending_revert = s_pending;
    out->revert_in_ms   = 0;
    if (s_pending) {
        int32_t left = (int32_t)(s_revert_at_ms - gptp_uptime_ms());
        out->revert_in_ms = (left > 0) ? (uint32_t)left : 0;
    }
    out->match     = rx_gate_match_count_read();
    out->nomatch   = rx_gate_nomatch_count_read();
    out->discarded = rx_gate_discard_count_read();

    out->last_drop_mac[0] = (uint8_t)(hi >> 8);
    out->last_drop_mac[1] = (uint8_t)hi;
    out->last_drop_mac[2] = (uint8_t)(lo >> 24);
    out->last_drop_mac[3] = (uint8_t)(lo >> 16);
    out->last_drop_mac[4] = (uint8_t)(lo >> 8);
    out->last_drop_mac[5] = (uint8_t)lo;
}

int rx_gate_add_mac(unsigned slot, const uint8_t mac[6])
{
    uint32_t hi = 0, lo = 0;
    if (mac) {
        hi = ((uint32_t)mac[0] << 8) | mac[1];
        lo = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
             ((uint32_t)mac[4] << 8)  |  (uint32_t)mac[5];
    }
    switch (slot) {
        case 0: rx_gate_spare0_mac_hi_write(hi); rx_gate_spare0_mac_lo_write(lo); break;
        case 1: rx_gate_spare1_mac_hi_write(hi); rx_gate_spare1_mac_lo_write(lo); break;
        default: return -1;
    }
    return 0;
}
