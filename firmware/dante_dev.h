// Dante device identity — Phase 3.
//
// One place for everything that identifies this device on a Dante network, so
// mDNS, ARC, CMC and the info multicast all agree. They must: Dante Controller
// cross-checks the device ID it learns from mDNS against the one in the info
// multicast, and a mismatch shows up as a device that appears and then vanishes.
//
// Values follow REAL HARDWARE where captures settled it (captures/README.md),
// not inferno's defaults:
//   * device_id is the MAC in EUI-64 form (00:1d:c1 ff fe a1:72:3c on the
//     RedNet AM2), byte-identical to the mDNS `id=` TXT. inferno derives it
//     from the IP address instead.
//   * TXT string values are truncated to 8 bytes ("Focusrite" -> "Focusrit").

#ifndef DANTE_DEV_H
#define DANTE_DEV_H

#include <stdint.h>

#define DANTE_MAX_NAME      32     // device/channel name buffer
#define DANTE_TX_CHANNELS   48
// 2 RX channels, so a real transmitter will run a unicast flow setup at us and
// we can observe it -- the one part of the protocol we have no reference for.
//
// The first attempt at this CRASHED Dante Controller, because the receive
// descriptor was built by mirroring the TRANSMIT one. They are not the same
// shape: proto_arc.rs get_receive_channels::ChannelDescriptor is 20 bytes
// (channel_id, unknown1_6, common_offset, tx_channel_name_offset,
// tx_hostname_offset, friendly_name_offset, subscription_status:u32,
// unknown2_0:u32) against the transmit side's 8. DC read 8 bytes per item where
// it expected 20 and walked off the end of the array.
#define DANTE_RX_CHANNELS   2

// Ports, all confirmed on real hardware.
#define DANTE_PORT_ARC      4440   // _netaudio-arc  control/routing
#define DANTE_PORT_CMC      8800   // _netaudio-cmc  device advertisement
#define DANTE_PORT_FLOWS    4455   // _netaudio-chan SRV port (flow control)
#define DANTE_PORT_MEDIA    4321   // audio
#define DANTE_PORT_INFO_REQ 8700   // info request / heartbeat source
#define DANTE_PORT_INFO     8702   // device-info multicast
#define DANTE_PORT_HEARTBEAT 8708  // heartbeat multicast

typedef struct {
    uint8_t  mac[6];
    uint8_t  device_id[8];              // EUI-64 of the MAC
    char     name[DANTE_MAX_NAME];      // advertised device name
    char     hostname[DANTE_MAX_NAME];  // <hostname>.local
    uint16_t process_id;
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
} dante_dev_t;

extern dante_dev_t g_dante;

// ADVERTISED RECEIVE LATENCY, nanoseconds -- ONE source of truth.
//
// This was hardcoded in three places that had drifted apart: the mDNS chan
// record said 2 ms, the mDNS bundle record said 1 ms, and the ARC flow
// descriptor said 1 ms. Dante Controller reads the bundle value, so it showed
// 1.0 ms while the chan record claimed 2 ms -- a device disagreeing with itself
// about its own latency, and the reason the selectable choices start at 1 ms.
//
// A receiver takes max(this, its own minimum) as playout latency (inferno
// channels_subscriber.rs:807), so LOWERING this is what makes 0.5 / 0.25 ms
// offerable -- but it is only safe while every accepted flow's packet window
// (fpp/48000) fits inside it. fpp=60, which DVS demands, is 1.25 ms on its own.
extern uint32_t g_latency_ns;

// Largest fpp we will ACCEPT in a flow request. We advertise "fpp=8,2" in mDNS
// but historically accepted anything in FPP_TABLE, so an AM2 asking for 16 got
// it -- and fpp=16 is a 333 us packetization window, which cannot fit inside a
// 250 us advertised latency. Capping this is what lets a slow receiver be
// pushed onto small packets so a low latency is reachable for everyone.
// OFF BY DEFAULT. Clamping DVS from fpp=60 to 16 improves DVS and COSTS the AM2
// more than it gains, because it triples DVS's packet rate and the total load is
// what the AM2 cannot take. Measured A/B, same bench, audio playing:
//
//     clamp ON,  18,000 pps   A16R 0    AM2 50 (1.04 ms) RED   DVS 11 (0.42 ms)
//     clamp OFF,  9,800 pps   A16R 6    AM2 22 (0.46 ms) green DVS 91 (2.73 ms)
//
// The AM2's OWN fpp was 16 in both cases -- the clamp never touched its flow.
// What changed was everything else on the wire. Our transmit jitter grows with
// TOTAL packet rate, and the AM2 has the least margin, so it fails first.
//
// This was committed ON on the strength of a 15-second measurement and reverted
// after ten minutes of running. Short samples do not show it.
//
// The earlier per-cap figures, taken before that was understood:
//
//     cap 60 (accept anything)  DVS fpp=60  1.77 ms   AM2 0.46 ms   all green
//     cap  4 (like an A16R)     DVS fpp=4   0.42 ms   AM2 1.10 ms   AM2 RED
//     cap 16                    DVS fpp=16  0.70 ms   AM2 0.28 ms   all green
//
// A receiver's requested fpp encodes its own CAPABILITY, not just its latency.
// The AM2 asks for 16 because it cannot process 12,000 packets/s -- clamping it
// to 4 tripled its packet rate and pushed it to 1.1 ms and red, while our side
// stayed clean at 48,001 pps with zero overruns. DVS, a PC, handles 12,000 pps
// happily. So "cap everything at 4 like the A16R does" is wrong for a mixed
// bench; 16 only bites the outlier and leaves every other receiver alone.
//
// 60 = accept anything (the old behaviour).
extern uint8_t g_fpp_max_accept;

// fpp maximum we ADVERTISE in the channel record. An A16R advertises 4.
extern uint8_t g_fpp_adv_max;

// Raise the advertised latency to cover the largest bound fpp window.
extern uint8_t g_latency_autoraise;

// On an oversized fpp request: 0 = reject it, 1 = SERVE IT at g_fpp_max_accept.
// DVS cannot be talked down -- its own minimum latency is 4 ms so it always asks
// for fpp=60, and rejecting it made it retry the identical request 16 times
// rather than renegotiate. But it accepts fpp=4 from a RedNet A16R, which caps
// there, so it can evidently consume a smaller packet than it asked for. A
// receiver reassembles by TIMESTAMP, not by the fpp it requested.
extern uint8_t g_fpp_clamp;

// Collector for the ARC request mirror; all-zero disables it.
extern uint8_t g_arc_mirror_ip[4];

void dante_dev_init(const uint8_t mac[6]);

// Channel names are generated on demand rather than stored: 48 of them at 32
// bytes would be 1.5 KB of BSS for strings that are almost always "NN".
// Returns the length written.
int  dante_tx_channel_name(uint16_t index_1based, char *buf, int maxlen);

#endif // DANTE_DEV_H
