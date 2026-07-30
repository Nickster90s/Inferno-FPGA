// Packetizer stream geometry — shared between firmware and the gateware
// packetizer's CSR programming.
//
// Extracted in Dante Phase 0 from the (now parked) aaf.h so that main.c no
// longer includes the AVB AAF software stack just to learn how wide a stream
// is. These are the numbers the gateware was BUILT with; changing one here
// without changing aaf_packetizer.py's constructor args will silently
// mis-program the contexts.
//
// Phase 5 note: PKT_CHANNELS (8) survives the move to Dante unchanged --
// Dante's MAX_CHANNELS_IN_FLOW is also 8, so the 6x8 = 48 structure is
// preserved. PKT_PRESENTATION_OFFSET_NS does NOT survive: AVB stamps a
// 32-bit presentation time in ns, Dante stamps {seconds, subsec_samples}
// in units of samples with a small negative offset instead (it wants the
// timestamp slightly in the past, never the future).

#ifndef PKT_GEOM_H
#define PKT_GEOM_H

// Channels per stream/flow. 6 streams x 8 = 48 host channels.
#define PKT_CHANNELS                8

// Samples per packet. AVB AAF: 6 (8000 fps at 48 kHz).
// Phase 5 moves this to 16 for Dante (3000 fps, 435-byte frames).
#define PKT_SAMPLES_PER_PACKET      6

// AVTP presentation-time offset ahead of the media clock. AVB-only.
#define AAF_PRESENTATION_OFFSET_NS  2000000

// Legacy alias kept so the existing status prints and USB block loop read
// naturally. Phase 5 removes it along with the AVB naming.
#define AAF_CHANNELS                PKT_CHANNELS

#endif // PKT_GEOM_H
