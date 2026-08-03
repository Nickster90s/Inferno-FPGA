// rx_gate.h — control and readout for the gateware RX MAC allow-list.
//
// See rx_gate.py for what the gateware does and why. The short version: the
// unmanaged bench switch floods other devices' Dante audio at us, the MAC has
// only 2 RX slots, and ethmac_sram_writer_errors climbs ~25/s -- real frame
// loss, which mispairs PTPv1 FollowUp/DelayResp and injects +/-5-10 us offset
// errors. The gate drops non-allow-listed frames before they take a slot.

#ifndef RX_GATE_H
#define RX_GATE_H

#include <stdint.h>

// Program the gateware with our unicast MAC. Leaves the filter DISABLED --
// arming is always an explicit, separate act. Call after net_init().
void rx_gate_init(const uint8_t mac[6]);

// Arm (on != 0) or disarm the filter.
//
// `revert_ms` is the safety net: when non-zero and the filter is being armed,
// the filter disarms itself automatically after that many milliseconds unless
// rx_gate_commit() is called first. Use it for any change requested over the
// network -- if the allow-list is wrong, the very packet you would need to send
// to undo it is the one that gets dropped. Pass 0 for a permanent change (the
// console path, which cannot be locked out).
void rx_gate_set(int on, uint32_t revert_ms);

// Cancel a pending auto-revert: "networking still works, keep it".
void rx_gate_commit(void);

// Drive the auto-revert timer. Call from the main loop.
void rx_gate_poll(void);

// Readout for the console and the UDP stats endpoint.
typedef struct {
    uint8_t  enabled;
    uint8_t  pending_revert;   // 1 = auto-revert armed and counting
    uint32_t revert_in_ms;     // ms until auto-revert (0 if not pending)
    uint32_t match;            // frames classified ALLOW
    uint32_t nomatch;          // frames classified DROP (counts even when off)
    uint32_t discarded;        // frames actually dropped (only moves when on)
    uint8_t  last_drop_mac[6];
} rx_gate_status_t;

void rx_gate_get_status(rx_gate_status_t *out);

// Drop count from the SOFTWARE allow-list in main.c's dispatch_rx(), which
// filters the same traffic one level up -- after the frame has already consumed
// an RX slot, which is why it cannot fix writer_errors on its own. Defined in
// main.c, next to the counter. Reported alongside the gate's own counters so
// the two can be compared: once the gate is armed, this should go flat.
uint32_t rx_gate_sw_filtered(void);

// Add an extra allowed MAC at runtime (slot 0 or 1). Exists so a missing
// multicast group can be fixed without a 20-minute rebuild that also re-rolls
// the P&R seed pin. mac == NULL clears the slot.
int rx_gate_add_mac(unsigned slot, const uint8_t mac[6]);

#endif // RX_GATE_H
