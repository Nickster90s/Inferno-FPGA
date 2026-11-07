# Handoff: static addressing from Dante Controller

Everything a fresh session needs to make the IP address settable from Dante
Controller (DC). Written 2026-11-07. Nothing here is implemented or tested yet.

**Read first:** the firmware running on the board is newer than this repo's
`a9b541b`. The repo sends capabilities `0x1000` and one interface in `0x11`; the
board sends `0x10db` and two. Check the code you are editing against the live
bytes below, and re-query the board (see "Querying a device").

---

## The problem in one line

DC's Network Config tab says **"This device does not support static addressing"**
for N-Series-Switchover because the device itself reports no such capability.
It is not a network problem.

## Devices

| | Address | MAC | Notes |
|---|---|---|---|
| N-Series-Switchover | `169.254.9.200` | `02:00:00:00:00:42` | the FPGA |
| 01-Prodigy1-32 | `169.254.143.65` | `00:1d:c1:2d:9c:60` | real Dante (Brooklyn 3), the reference |

## How DC decides

From `com/audinate/gui/conmon/StaticAddressesPane` in `DanteController.jar`:

- `IVersions.hasStaticIp()` is `inferredCapabilities & 0x4000`, bit 14 of the
  capabilities in the versions status message (`0x60`). Clear means this message.
- `hasReadOnlyStaticIp()` tests the same bit in the read-only capabilities.
- It also shows this message for device types 3 and 4, which come from
  `router_info` "Dolby Lake Processor" and "Lab-Gruppen OEM". Neither applies.
- With the bit set, the form reads the interfaces from `0x11` and writes with
  `0x13`. It shows "must be rebooted" when the reboot config differs from the
  current config.

## Live comparison

Query replies, content only (from byte 32 of the UDP payload).

| | FPGA | Prodigy |
|---|---|---|
| Capabilities (`0x60`, u32 at `0x14`) | `0x000010db` | `0x0e7cd65b` |
| **Bit 14 `HAS_STATIC_IP`** | **off** | **on** |
| Interface 1 speed | `0x000103e8` (66536) | `0x000003e8` (1000) |
| Interface 1 flags | `0x0002` | `0x0001` |
| Interface 2 | shifted by 2 bytes | correct |
| Reboot config block | missing | present |

Capability bits set:

```
FPGA     identify sys_reset set_rate set_encoding tftp no_ext_wc manuf_versions
Prodigy  identify sys_reset set_rate set_encoding tftp pullups serial manuf_versions
         STATIC_IP metering switch_vlan audio_if ptp_log delay_ll clear_config
         no_reboot_after_upg aes67 can_lock
```

`0x11` from the FPGA:

```
000 00 02 00 00 00 01 03 e8 00 02 02 00 00 00 00 42
010 a9 fe 09 c8 ff ff 00 00 00 00 00 00 00 00 00 00
020 03 e8 00 02 02 00 00 00 00 43 ac 1f 09 c8 ff ff
030 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

`0x11` from the Prodigy:

```
000 00 02 00 01 00 00 03 e8 00 01 00 1d c1 2d 9c 60
010 a9 fe 8f 41 ff ff 00 00 00 00 00 00 00 00 00 00
020 00 00 00 00 00 00 00 1d c1 2d 9c 61 ac 1f 8f 42
030 ff ff 00 00 00 00 00 00 00 00 00 00 00 18 00 4c
040 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00
050 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
060 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
070 00 00 00 00
```

Parsed with the layout below, the FPGA's second interface has speed
`0x03e80002`, MAC `00:00:00:43:ac:1f` and IP `9.200.255.255`, instead of
`172.31.9.200`.

## Message layout

Taken from the accessors in `libDanteController.dylib`
(`conmon_audinate_interface_*`). Offsets are from the start of the content,
multi-byte fields are big-endian.

### `0x11` interface status

```
+0   u16 number of interfaces (n)
+2   u16 mode
+4   n records of 28 bytes:
       +0  u32 link speed, Mbps
       +4  u16 flags
       +6  mac[6]
       +12 ip
       +16 netmask
       +20 dns
       +24 gateway
then u16 0x0018   reboot record size
     u16 offset   of the reboot records: 28*n + 0x14, counted from the opcode,
                  so content offset + 8 (n=2 gives 0x4c)
     u16 flags    message-level flags, Prodigy sends 0x0003
     u16 0
then n reboot records of 0x18 bytes:
       +0  u16 flags, 0x0004 = valid
       +4  ip
       +8  netmask
       +12 dns
       +16 gateway
```

The reboot config is only honoured for message versions >= `0x0600`. The version
is the first two opcode bytes; ours is `0x072a`, so that is fine.

### `0x13` interface control

An empty body is a query; a body is the write.

```
+0   u8  number of interfaces (DC's setters write 1)
+1   u8  record size, 0x1c
+2   u8  field mask: 0x01 ip, 0x04 dns, 0x08 gateway (static writes 0x0f)
+3   u8  offset of the first record, 0x10, counted from the opcode
+4   u16 mask     (message version >= 0x0602)
+6   u16 flags    (message version >= 0x0602)
+8   record:
       +0  u16 interface index
       +2  u16 flag values
       +4  ip
       +8  netmask
       +12 dns
       +16 gateway
       +20 u16 which flags are valid
       +24 domain name offset (0 from the address setters)
```

Flags in `+2` and `+20`:

| Setter | `+20` | `+2` |
|---|---|---|
| static | `0x0002` | `0x0002` |
| dynamic | `0x0002` | `0` |
| DHCP on / off | `0x0020` | `0` / `0x0020` |
| link-local on / off | `0x0008` | `0` / `0x0008` |

### Inferred, not confirmed

- In status flags, bit 0 means link up and `0x0002` means static. The status
  initialiser derives bit 0 from speed being non-zero and masks the caller's
  flags with `0x0002`. If right, the FPGA should send `0x0003`.
- `0x02` in the field mask means netmask. The `0x01`, `0x04` and `0x08` bits were
  read from their getters; the netmask getter was not read.
- Whether the interface index counts from 0 or 1.
- What `mode` means (Prodigy 1, FPGA 0).
- What the reboot record's flags should be for a static address, probably
  `0x0006`.

## The work

In `firmware/dante_info.c`:

1. **Set the bit in `0x60`:** `c[0x16] |= 0x40` in `send_device_info`. DC should
   then show the form. Check it on the bench before going on.
2. **Fix `0x11`** in `send_network_info`: speed `0x000003e8`, the 2 missing bytes
   at the start of interface 2, and add the trailer and reboot records.
3. **Execute `0x13` with a body:** `info_rx` currently calls `send_network_info`
   for every `0x13`. Parse the body when the packet is longer than the header,
   validate the address, and answer with `0x11`.
4. **Persist and reboot:** `cfg_t` already has `osc_ip` and `osc_prefix`, check
   whether they are the right home and whether boot applies them. Report the new
   address in the reboot records. Whether the board handles DC's reboot command
   has not been checked.

Do step 1 and 2 first, then capture DC pressing Apply to settle the inferred
items.

## Capturing and querying

Sniffing needs root on the Mac. DC runs there, on `en8`:

```
sudo tcpdump -i en8 -s0 -w /tmp/dc-static.pcap host 169.254.9.200
```

### Querying a device

No root needed. Send a 32-byte header to UDP 8700; replies arrive on the
requesting socket or on `224.0.0.231:8702`, so listen on both (`SO_REUSEPORT`,
joined on the Dante interface). An empty `0x13` is only a query, and is safe on
real devices. Never add a body when querying the Prodigy.

```python
header = struct.pack('>HHHH', 0xffff, 32, seq, 0) + bytes(8) + b'Audinate' \
       + bytes([0x07, 0x2a, 0, op, 0, 0, 0, 0])   # op: 0x61 versions, 0x13 interfaces
```

## Also seen

DC's log has N-Series-Switchover repeatedly losing and regaining clock sync
(2026-11-06 18:07 to 18:43, 2026-11-07 09:41 to 09:45). Unrelated to addressing
and not investigated.
