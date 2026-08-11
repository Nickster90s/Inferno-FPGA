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
// 60 = accept anything (the old behaviour).
extern uint8_t g_fpp_max_accept;

// fpp maximum we ADVERTISE in the channel record. An A16R advertises 4.
extern uint8_t g_fpp_adv_max;

// Raise the advertised latency to cover the largest bound fpp window.
extern uint8_t g_latency_autoraise;

// Collector for the ARC request mirror; all-zero disables it.
extern uint8_t g_arc_mirror_ip[4];

void dante_dev_init(const uint8_t mac[6]);

// Channel names are generated on demand rather than stored: 48 of them at 32
// bytes would be 1.5 KB of BSS for strings that are almost always "NN".
// Returns the length written.
int  dante_tx_channel_name(uint16_t index_1based, char *buf, int maxlen);

#endif // DANTE_DEV_H
