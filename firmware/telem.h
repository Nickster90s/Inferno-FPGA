// telem.h — streaming telemetry ring. TELEMETRY_AND_PTP.md item 2.
//
// WHY THIS EXISTS, in the words of the failures that caused it:
//
//   "A snapshot cannot show a servo."
//
// Three separate faults in this project were invisible to snapshot counters and
// cost a bench session each:
//
//   * The media clock sat 227 ms in the future for two days. Every counter read
//     healthy -- 9001 pps, ring centred, zero underruns, PTP locked -- while no
//     audio played at all, because nothing measured PHASE.
//   * A phase servo hunted +/-0.5 ppm in a limit cycle. The only visible trace
//     was `drift` wandering between reads that were minutes apart, which got
//     read as accumulation rather than oscillation.
//   * Two wrong diagnoses in one evening, both from inferring behaviour through
//     0.5 Hz samples of already-filtered values.
//
// So: fixed-size records pushed ON EVENT into a ring, drained by the host, and
// analysable OFFLINE. Both error axes -- rate AND phase -- and state
// transitions, not just levels.
//
// RULES, taken from TELEMETRY_AND_PTP.md and paid for in flash-and-measure
// cycles:
//   * NEVER hand-index the reply. Every drain carries a version tag and an
//     explicit record size; the host parses by name. Two wrong conclusions in
//     this project came from stale offsets.
//   * Log STATE TRANSITIONS, not just levels. Levels hide events.
//   * Watch the reply size. Growing the stats reply 200 -> 208 bytes silently
//     killed that port once, cause never found. This is a SEPARATE opcode with
//     its own bounded reply, so it cannot regress that endpoint.

#ifndef TELEM_H
#define TELEM_H

#include <stdint.h>

#define TELEM_REC_BYTES   28
#define TELEM_RING_RECS   96          // ~2.7 KB of BSS

// Record types.
#define TELEM_T_PTP       1   // one per PTP servo update
#define TELEM_T_MCLK      2   // media clock, 1 Hz
#define TELEM_T_EVENT     3   // state transition

// TELEM_T_PTP flags
#define TELEM_F_LOCKED        0x01
#define TELEM_F_OUTLIER       0x02   // sample rejected by the outlier guard
#define TELEM_F_NO_FOLLOWUP   0x04
#define TELEM_F_MISPAIRED     0x08

// TELEM_T_MCLK flags
#define TELEM_F_DISCIPLINED   0x01
#define TELEM_F_PHASE_ON      0x02
#define TELEM_F_TALKER_ON     0x04

// TELEM_T_EVENT ids (carried in `aux`)
#define TELEM_E_PTP_LOCK      1
#define TELEM_E_PTP_UNLOCK    2
#define TELEM_E_ANCHOR        3
#define TELEM_E_TALKER_ON     4
#define TELEM_E_TALKER_OFF    5
#define TELEM_E_FLOW_BIND     6
#define TELEM_E_FLOW_UNBIND   7
#define TELEM_E_MCLK_TRIP     8
#define TELEM_E_MCLK_ARM      9
#define TELEM_E_MCLK_DISARM  10

void telem_init(void);

// Push one record. Safe to call from anywhere in the main loop; overwrites the
// oldest record when the ring is full and reports the loss to the host as a
// sequence gap rather than silently dropping it.
void telem_push(uint8_t type, uint8_t flags, uint16_t aux,
                int32_t v0, int32_t v1, int32_t v2, int32_t v3);

// Convenience for a bare state transition.
void telem_event(uint16_t event_id, int32_t a, int32_t b);

#endif // TELEM_H
