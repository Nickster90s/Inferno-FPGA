# Wire captures from real Dante hardware

The plan treats **the wire as the specification**: inferno is a reverse-engineering
reference to read, not a source of truth, and every constant it marks `???` needs
confirming against real gear. These captures are that confirmation.

Taken on the bench network (`ens5`, 169.254/16) with a **Focusrite RedNet AM2**
(`RF04-RedNetAM2-RFtech`, MAC `00:1d:c1:a1:72:3c`, IP 169.254.61.114) and a Dante
controller present.

| File | Contents |
|---|---|
| `dante_ptpv1.pcap` | 116 PTPv1 packets, 224.0.1.129 ports 319/320 |
| `dante_control.pcap` | 27 Dante heartbeat packets, 224.0.0.233:8708 |

Read with `tcpdump -r <file> -n -X`.

---

## PTPv1 (Phase 4) — wire format CONFIRMED

Transport matches the statime `inferno-dev` fork exactly: **UDP multicast
224.0.1.129, event port 319, general port 320**. The AM2 transmits Sync at ~4 Hz,
so it is acting as grandmaster — meaning there is a real PTPv1 master on the
bench to test `ptpv1.c` against, not just `tools/ptpv1_master.py`.

Header, verified byte for byte against
`statime/src/datastructures/messages_v1/header.rs`:

| Offset | Bytes observed | Field |
|---|---|---|
| 0–2 | `0001` | `version_ptp` = 1 |
| 2–4 | `0001` | `version_network` = 1 |
| 4–20 | `5f44 464c 5400…` | `subdomain[16]` = **`_DFLT`** |
| 20 | `01` | `port_type` (Event) |
| 21 | `01` | `source_communication_technology` = 1 |
| 22–28 | `001d c1a1 723c` | `source_uuid` = sender MAC |
| 28–30 | `0001` | `source_port_id` |
| 30–32 | `0aa4` | `sequence_id` |
| 32 | `00` | `control` (0 = Sync) |

Header is **40 bytes**. The Sync body then starts at payload offset 40 with
`originTimestamp` as **u32 seconds + u32 nanoseconds** (`000002bb` / `0ca875bc`)
— confirming it is *not* PTPv2's 6+4 layout.

**Two details the plan did not have**, both worth replicating in our TX:

- **IP TOS = `0xe0`** (DSCP 56 / CS7, highest priority)
- **TTL = 1**, with ephemeral source ports (49155/49156 here) → fixed 319/320

## Dante multicast framing (Phase 3) — CONFIRMED

Heartbeat sent from port **8700** to **224.0.0.233:8708**. The 32-byte header in
`inferno_aoip/src/protocol/mcast.rs` is confirmed field for field:

| Field | Observed |
|---|---|
| `start_code` u16 | `fffe` |
| `total_length` u16 | `00ac` (172) |
| `seqnum` u16 | `035a` |
| `process` u16 | `0000` |
| `factory_device_id[8]` | `001dc1fffea1723c` |
| `vendor[8]` | `"Audinate"` |
| `opcode[8]` | `0008 0001 1000 0000` |

The heartbeat opcode matches inferno's documented
`00 08 00 01 10 00 00 00` exactly. Content is a series of TLVs tagged
`0x8001`, `0x8002`, `0x8003`, `0x8004` — consistent with inferno's note that the
`0x8001` sub-record carries the frequency offset in ppb that drives Dante
Controller's clock histogram.

**`factory_device_id` is the MAC in EUI-64 form** (`00:1d:c1` + `ff:fe` +
`a1:72:3c`), and it is byte-identical to the mDNS `id=` TXT value. Note this
differs from inferno's default, which derives a device ID from the IP address —
we should follow the hardware, not inferno.

## mDNS (Phase 3) — CONFIRMED, including a constant inferno marks `???`

```
_netaudio-arc._udp  port 4440   RN-AM2-a1723c.local  169.254.61.114
  model=_0000000000000003  mf=Focusrit  router_info=ULTIMOX2
  router_vers=4.3.0  arcp_min=0.2.4  arcp_vers=2.8.9

_netaudio-cmc._udp  port 8800
  id=001dc1fffea1723c  process=0  cmcp_vers=1.2.0  cmcp_min=1.0.0
  channels=0x6000004d  server_vers=4.0.5  mf=Focusrit
  model=_0000000000000003
```

Ports 4440 and 8800 match. `cmcp_vers=1.2.0`, `cmcp_min=1.0.0` and `arcp_min=0.2.4`
match inferno's hardcoded values exactly. So does **`channels=0x6000004d`**, which
inferno emits with a `// ???` comment — the real device uses the identical value,
so it is a constant rather than a per-device field.

**`mf=Focusrit`** is "Focusrite" truncated to 8 characters, which is why inferno's
builder is called `add_txt_truncated`. Treat 8 bytes as the limit for these values.

Inferno emits **two empty TXT strings** on the CMC record, flagged "really
needed?". The real device shows none. Either they are unnecessary, or avahi
suppresses them — do not assume they are required.

## Still unconfirmed: `b.N=` multicast bundles (plan risk #1)

No `_netaudio-chan` or `_netaudio-bund` records appeared, because the RedNet AM2
is a **receive-only** device (a 2-channel output amp) and therefore advertises no
transmit channels.

Confirming the `b.<bundle>=<pos+1>` TXT key — which the whole Phase 5 gateware
design leans on — needs a Dante **transmitter** with a multicast flow configured
from the controller. That remains the highest-value outstanding measurement, and
it is the Phase 3 exit gate.
