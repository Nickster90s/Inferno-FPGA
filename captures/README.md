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

## A transmitter arrived: RedNet A16R — and it corrects several assumptions

Second device: **RedNet A16R** (`RN-A16R2-2d4a18.local`, 169.254.60.249,
MAC `00:1d:c1:2d:4a:18`), a 16×16 analogue interface, so it has inputs and
advertises transmit channels. 18 `_netaudio-chan` records captured in
`netaudio_chan_A16R.txt`. A representative one:

```
instance 01@RedNetA16R   host RN-A16R2-2d4a18.local   port 4455
  txtvers=2  dbcp=0x1004  dbcp1=0x1200  id=1  rate=48000
  pcm=3 4  enc=24  en=24  latency_ns=500000
  fpp=8,2  nchan=64  at2
```

**Port 4455 confirmed** — the flow-control port, not 4440, exactly as
`mdns_server.rs:110` has it.

### Corrections to earlier assumptions

| Assumption | Reality | Consequence |
|---|---|---|
| `MAX_CHANNELS_IN_FLOW = 8`, "Dante's max is also 8, mapping perfectly onto our 6×8" | **`nchan=64`** | **Wrong.** 8 is *inferno's own* limit, not a protocol limit. Our 6×8=48 is still legal (8 ≤ 64) but the justification was bogus — and larger flows are possible. |
| Phase 5 plan uses **`fpp = 16`** | Device advertises **`fpp=8,2`** (max 8, min 2) | **Design input.** 16 exceeds what this hardware accepts. Phase 5 should use fpp=8: payload 8×8×3 = 192 B, frame 51+192 = **243 B**, 6000 pps/flow. |
| `dbcp1=0x1102` (inferno forces this as the request `start_code`) | **`dbcp1=0x1200`** | Potential interop issue — real devices advertise a newer protocol version. Worth honouring the advertised value rather than forcing inferno's. |
| `channels=0x6000004d` is "a constant, not a per-device field" | AM2 `0x6000004d`, A16R **`0x6000017f`** | **Wrong** — it *is* per-device. My earlier note in this file said otherwise; disregard that. |
| inferno emits a `default` TXT key | Not present on either device | Probably optional. |
| — | **`at2`** | A TXT key inferno does not emit at all. Meaning unknown. |

`latency_ns=500000` (0.5 ms) here versus inferno's 10 ms default is also worth
noting — real hardware runs much tighter latencies than inferno's conservative
default.

## RESOLVED — plan risk #1: `b.N=` multicast bundles are REAL

A multicast flow was created on the A16R from Dante Controller. Both halves of
the mechanism inferno documents at `mdns_client.rs:240-270` appeared immediately.

**Bundle record** (`netaudio_bund.txt`):

```
instance 32@RedNetA16R   host RN-A16R2-2d4a18.local   port 4321
  txtvers=1  id=32  nchan=8  fpp=16  rate=48000  enc=24
  latency_ns=1000000  a.0=239.255.201.92  p.0=4321  at2
```

**Chan records** gained exactly the expected keys: `b.32=1` … `b.32=8` — bundle
id 32, channel positions 1 through 8, one per channel in the bundle.

So a receiver resolves a channel → reads `b.<bundle>=<pos>` → looks up
`<bundle>@<host>._netaudio-bund._udp.local` → reads `a.0`/`p.0` → joins the
group. **The transmitter is never contacted.** That is precisely the scope
reduction Phase 5's design depends on, and it is confirmed on real hardware.

Field-for-field this matches inferno's `mdns_server.rs:170-210` bundle TXT set
(`txtvers=1`, `id=`, `nchan=8`, `latency_ns=`, `fpp=16`, `rate=48000`, `enc=24`,
`a.0=`, `p.0=`), including `nchan=8` and `fpp=16`.

## Audio data plane (Phase 5) — CONFIRMED, every parameter

`dante_audio_multicast.pcap` (400-packet sample; 29,760 were captured in 10 s
≈ 2976 pps ≈ 48000/16).

```
IP   45 b8 ...  TOS 0xb8 (DSCP 46 EF)   ID 0000   TTL 32   proto 17
     src 169.254.60.249  dst 239.255.201.92
UDP  sport 61471  dport 4321  len 401  cksum 0x0000
payload (393 B):
     [0]      02                  constant
     [1..5]   00 00 03 ad         seconds        = 941
     [5..9]   00 00 9a 4c         subsec_samples = 39500
     [9..]    384 B               16 samples x 8 ch x 3 B, signed 24-bit BE
```

Timestamps advance by **exactly 16** per packet
(45207548 → 45207564 → 45207580 …), i.e. `sec*48000 + subsec` in units of
samples, incrementing by `fpp`.

Every value the plan specified for Phase 5 is confirmed:

| Plan | Hardware |
|---|---|
| byte[0] = `0x02` | ✅ |
| seconds u32 BE, subsec_samples u32 BE | ✅ |
| timestamp advances by `fpp` per packet | ✅ exactly +16 |
| 24-bit, big-endian, MSB-justified | ✅ signed 24-bit BE |
| `fpp = 16`, 8 ch/flow | ✅ |
| multicast `239.255.x.y:4321` | ✅ 239.255.201.92:4321 |
| IP TOS `0xB8` (DSCP EF) | ✅ |
| IP ID = 0 | ✅ |
| UDP checksum transmitted as `0x0000` | ✅ |
| frame = 14+20+8+9+384 = **435 B** | ✅ 435 B on the wire |

### Retraction: the `fpp=8` "correction" was wrong

The previous section corrected the plan's `fpp=16` down to 8, on the strength of
`fpp=8,2` in the **chan** records. That was a misreading. Those two values are
different things:

- **chan record `fpp=8,2`** — the min/max range this device will negotiate for a
  **unicast** flow.
- **bundle record `fpp=16`** — the fpp actually used by a **multicast** bundle.

Real Dante multicast uses **fpp=16**, so the plan's original figure was correct
and the 435-byte frame geometry stands unchanged. Phase 5 needs no change here.

Similarly, `nchan=64` on chan records is the unicast ceiling; multicast bundles
here carry `nchan=8`, which is what the 6×8 = 48 structure targets.
