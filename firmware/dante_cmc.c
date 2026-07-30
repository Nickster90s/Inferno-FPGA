// Dante CMC server (port 8800) — Phase 3.
//
// One opcode. Dante Controller uses it to confirm the device identity it
// learned from mDNS; the factory_device_id here MUST match the mDNS `id=` TXT
// and the info multicast, or the device appears and then disappears. That is
// why all three read it from dante_dev.c rather than each building their own.

#include "dante_cmc.h"
#include "dante_msg.h"
#include "dante_dev.h"
#include "net.h"
#include <string.h>
#include <stdio.h>

#define OP_REQUEST_DEVICE_ADVERTISEMENT  0x1001

dante_cmc_stats_t g_cmc_stats;

static void cmc_rx(const uint8_t src_ip[4], const uint8_t dst_ip[4],
                   uint16_t src_port, const uint8_t *req, uint32_t len)
{
    (void)dst_ip;
    if (len < DANTE_HDR_LEN) return;
    if (dante_req_opcode2(req) != 0) return;
    if (dante_req_opcode1(req) != OP_REQUEST_DEVICE_ADVERTISEMENT) return;

    g_cmc_stats.rx++;

    uint8_t hdr[DANTE_HDR_LEN];
    memcpy(hdr, req, DANTE_HDR_LEN);

    dante_msg_t m;
    dante_msg_begin(&m, net_udp_payload_buf(), hdr);

    // DeviceAdvertisement, proto_cmc.rs:5-14.
    dante_msg_u16  (&m, g_dante.process_id);
    dante_msg_bytes(&m, g_dante.device_id, 8);
    dante_msg_u16  (&m, 1);                       // unknown1_1
    dante_msg_u16  (&m, 0);                       // unknown2_0
    dante_msg_bytes(&m, g_net_ip, 4);
    dante_msg_u16  (&m, DANTE_PORT_INFO_REQ);
    dante_msg_u16  (&m, 0);                       // unknown3_0

    uint32_t total = dante_msg_finish(&m, DANTE_CODE_OK);
    if (net_udp_commit(src_ip, src_port, DANTE_PORT_CMC, total,
                       NET_TOS_BEST_EFFORT) == 0)
        g_cmc_stats.tx++;
}

void dante_cmc_init(void)
{
    net_udp_bind(DANTE_PORT_CMC, cmc_rx);
    printf("[cmc] listening on %u\n", DANTE_PORT_CMC);
}
