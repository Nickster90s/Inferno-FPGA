// PTPv1 (IEEE 1588-2002) slave — Dante Phase 4.
//
// Dante's native clock protocol. This is what turns Dante Controller's Sync
// indicator from red to green and populates "Primary v1 Multicast" with
// Follower.
//
// SLAVE ONLY. We never transmit Sync, so we never participate in BMCA and can
// never be elected Leader -- which is the correct behaviour for this device and
// is also why Dante Controller should not offer it as a Preferred Leader.

#ifndef PTPV1_H
#define PTPV1_H

#include <stdint.h>
#include "gptp.h"

void ptpv1_init(const uint8_t mac[6]);
void ptpv1_poll(void);

// Runtime servo tuning, so variants can be interleaved without a reflash.
void ptpv1_set_tuning(uint8_t median_n, int32_t ki_num, uint8_t exact);
void ptpv1_get_tuning(uint8_t *median_n, int32_t *ki_num, uint8_t *exact);

typedef struct {
    uint8_t  locked;                 // servo has converged
    uint8_t  have_master;            // a Leader has been seen
    uint8_t  master_uuid[6];
    uint16_t master_port_id;

    int64_t  offset_ns;              // last offset from master
    int64_t  mean_path_delay_ns;

    uint32_t rx_sync, rx_followup, rx_delay_resp, rx_other;
    uint32_t tx_delay_req;
    uint32_t servo_updates;

    uint64_t base_addend_full;
    uint64_t current_addend_full;
    // Rate estimate ONLY, in ppb -- the servo integral, without the
    // proportional term. This is what a media clock should follow: the
    // proportional term is phase correction, and feeding it into an audio
    // sample rate is audible wander, not drift correction.
    int32_t  rate_ppb;

    // PHASE STEP ACCOUNTING.
    //
    // Any step moves absolute time out from under anything anchored to it. The
    // media clock is a free-running counter anchored ONCE to PTP, so a step
    // after that anchor leaves it on the old timeline permanently -- audio that
    // is out of sync from the moment the stream starts, with every counter
    // healthy. dante_tx watches step_count and re-anchors when it moves.
    uint32_t step_count;        // bumped on every absolute step or phase adjust
    uint8_t  phase_settled;     // 1 once the post-path-delay residual step is done
} ptpv1_state_t;
extern ptpv1_state_t g_ptpv1;

#endif // PTPV1_H
