# Handoff: static addressing from Dante Controller

Everything a fresh session needs to make the IP address settable from Dante
Controller (DC). Written 2026-11-07, updated the same day after an Apply capture.

**Status:** steps 1 to 4 work end to end: DC shows the form, Apply reaches the
firmware, and Reboot from DC brought the board up on the new address
(`169.254.9.201`). Open: the reboot record holds the old address after boot
(see "Reboot from DC").

**The board is now on `169.254.9.201`, not `.200`.**

**Read first:** the firmware running on the board is not what `main`
(`a9b541b`) contains. `main` sends capabilities `0x1000` and one interface in
`0x11`. The other branches (`ring-pointers-48bit`, `fpp-arbitrary-wip`) and the
November tags were not checked. Compare the code you edit against the live bytes
below.

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

### Confirmed by the Apply capture

- The interface index counts from 0.
- DC sends field mask `0x0f` with ip, netmask, dns and gateway, so `0x02` is
  netmask.
- Static is flag value `0x0002` with `0x0002` in the valid-flags field.

### Inferred, not confirmed

- In status flags, bit 0 means link up and `0x0002` means static. The status
  initialiser derives bit 0 from speed being non-zero and masks the caller's
  flags with `0x0002`.
- What `mode` means (Prodigy 1, FPGA 0 in the old firmware, 1 now).
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

## Apply capture (2026-11-07 10:35)

46 s on the Mac's `en8`, DC at `169.254.228.170`, after steps 1 and 2 were on
the board. The board now announces capabilities `0x000050db` (bit 14 set) and
this `0x11`, one interface:

```
000 00 01 00 01 00 00 03 e8 00 01 02 00 00 00 00 42
010 a9 fe 09 c8 ff ff 00 00 00 00 00 00 00 00 00 00
020 00 18 00 30 00 03 00 00 00 00 00 00 00 00 00 00
030 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### What DC sends on Apply

From `169.254.228.170:8700` to `169.254.9.200:8700`, opcode `07 3d 00 13 00 00 00 64`:

```
000 01 1c 0f 10 00 00 00 00 00 00 00 02 a9 fe 09 c8
010 ff ff 00 00 00 00 00 00 00 00 00 00 00 02 00 00
020 00 00 00 00
```

That is interface 0, static, `169.254.9.200` / `255.255.0.0`, no dns, no
gateway. It is the board's current address, so this capture cannot tell a
change from no change. Retest with a different address.

DC sent the same write again 2 s later. It repeats its other requests (`0xc1`,
the empty `0x13`) once after 2 s too, so the repeat is not by itself a failure.
No reboot command followed.

### What the board answers

`0x11` with interface flags `0x0003` instead of `0x0001`: the firmware applies
static at once. The reboot record stays empty (offset `0x30`, all zeros).

At 10:36:29, after the capture, the multicast `0x11` went back to flags
`0x0001`. Cause unknown: Revert or dynamic in DC, or a reboot.

### What a real device presumably does

Keeps the current record unchanged and fills the reboot record (flags
`0x0004 | 0x0002`, new ip, netmask, dns, gateway). DC then shows "This device
must be rebooted for the changes to take effect", and the address changes at
boot. This follows from DC's code (`equalConfig` of current vs reboot); it was
not captured from a real device.

## Reboot from DC (2026-11-07, about 11:40)

After an Apply with `169.254.9.201` (not captured), pressing Reboot in DC
restarted the board, and it came back on `169.254.9.201`. So the address is
stored and applied at boot.

### What DC sends

Not captured; this is from `conmon_audinate_init_sys_reset_control` and
`SysResetControl` in DC:

```
opcode  07 3d 00 90 00 00 00 64     SYS_RESET, type 144
content 00 00                       u16 mode: 0 soft reboot, 1 factory reset
```

The status reply is `SYS_RESET_STATUS` (`0x92`), content a u16 mode. Whether DC
needs that reply is unknown.

### What the board announces after the reboot

```
000 00 01 00 00 00 00 03 e8 00 03 02 00 00 00 00 42
010 a9 fe 09 c9 ff ff 00 00 00 00 00 00 00 00 00 00
020 00 18 00 30 00 03 00 00 00 04 00 00 a9 fe 09 c8
030 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

- Current: `169.254.9.201` / `255.255.0.0`, flags `0x0003`. Right.
- **Reboot record is backwards:** flags `0x0004` (valid) with `169.254.9.200`,
  the OLD address. DC reads this as "will be `.200` after the next reboot", so
  it presumably shows "must be rebooted" again, and if the firmware applies the
  reboot record at boot, the next reboot goes back to `.200`.
- The reboot flags lack `0x0002`, so DC may read the pending config as dynamic.
- `mode` went from 1 to 0. Meaning still unknown.

### Fix

After boot, the reboot record should describe what the next boot will apply:
the active address (`0x0006`, `.201`), or empty. Only a new Apply should put a
different address there. Check in DC whether Network Config shows "must be
rebooted" now; that confirms how DC reads the reboot record.

### Unrelated, seen in the same capture

- **ARC `0x4100` storm:** DC sent 11,732 identical `0x4100` requests to 4440 in
  46 s (up to ~1000/s), body
  `0000000000000000000100000001000000000000830283060310`. The firmware answers
  code `0x30` (`dante_arc.c`, copied from inferno), and DC keeps retrying. A real
  device's answer to `0x4100` is not known.
- **Queries from other ports go unanswered:** the board answers DC's 8700→8700
  requests, but no longer answered queries sent from an ephemeral port, as the
  old firmware did. Send queries from port 8700, or listen on `224.0.0.231:8702`
  for the 3 s announcements.

## Capturing and querying

Sniffing needs root on the Mac. DC runs there, on `en8`:

```
sudo tcpdump -i en8 -s0 -w /tmp/dc-static.pcap host 169.254.9.200
```

### Querying a device

No root needed. Send a 32-byte header to UDP 8700 (from source port 8700 for
the current firmware); replies arrive on the
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
