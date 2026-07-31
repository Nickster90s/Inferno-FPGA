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
// BACK TO 0. Advertising 2 RX channels CRASHED Dante Controller.
//
// The idea was sound -- be a receiver, and watch a shipping transmitter drive a
// unicast flow setup at us, which is the one part of the protocol we have no
// reference for. The execution was not: the RX channel descriptor was built by
// mirroring the TX one (put_common_descriptor + name offset), on the assumption
// that both sides share a format. There is no evidence for that assumption, and
// a receive descriptor almost certainly carries subscription fields a transmit
// descriptor does not. DC parsing a malformed descriptor is a good way to crash
// it.
//
// Doing this properly means reading a REAL device's 0x3000 reply first -- the
// AM2 and DVS both have RX channels and both answer it -- and matching that,
// rather than guessing from the transmit side.
#define DANTE_RX_CHANNELS   0      // transmit-only until 0x3000 is verified

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

void dante_dev_init(const uint8_t mac[6]);

// Channel names are generated on demand rather than stored: 48 of them at 32
// bytes would be 1.5 KB of BSS for strings that are almost always "NN".
// Returns the length written.
int  dante_tx_channel_name(uint16_t index_1based, char *buf, int maxlen);

#endif // DANTE_DEV_H
