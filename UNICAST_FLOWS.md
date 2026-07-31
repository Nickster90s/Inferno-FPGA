# Unicast flow requirements, measured from real receivers

Captured off the wire and the console with `b.<bundle>=` removed from our
channel TXT records, so both devices fell back to the unicast path and
connected to our flow-control server on 4455.

## What two real receivers ask for

| field           | RedNet A16R (169.254.60.249) | RedNet AM2 (169.254.61.114) |
|-----------------|------------------------------|-----------------------------|
| sample_rate     | 48000                        | 48000                       |
| bits_per_sample | 24                           | 24                          |
| num_channels    | 4                            | 2                           |
| channel slots   | `[1, 2, 0, 0]`               | `[1, 2]`                    |
| fpp             | **8**                        | **16**                      |
| destination     | 169.254.60.249:14337         | 169.254.61.114:14337        |

Repeated every ~5 s. Those are KEEPALIVES, not retries -- flows_control.rs
lists `opcode2 = 0x0103 = stream expired (i.e. no keepalives)`, so a flow has
to be refreshed or it dies.

## What this forces on the gateware

Three things vary per flow, and all three change the packet:

1. **fpp differs per receiver** (8 vs 16). fpp sets the payload size, so
   packet length is per-flow, not a constant. `ip_totlen`, `udp_len` and
   `LAST_BE` all become per-context.
2. **Slot count differs** (4 vs 2), also changing packet length.
3. **Slot 0 means silence.** A flow is an arbitrary slot -> channel map, not a
   contiguous run. Today flow f is hard-wired to channels 8f-7..8f.

Note both receivers want the SAME two channels. Flows are per-receiver, not
per-channel-set, so two flows carrying overlapping channels is normal and the
channel map must allow it.

## Design (plan Phase 5(c), extended)

Per-context CSRs behind `ctx_select`, alongside the existing dst_ip/ip_csum/
dst_mac/udp ports:

- `chmap` 8 x 6 bits -- slot -> tx channel, plus a valid bit per slot for
  silence. Ring re-addressing per the plan: memory m holds channels 8m..8m+7 at
  address `(within << 8) | idx`, so the read for slot c is memory
  `chmap[c] >> 3`, address `((chmap[c] & 7) << 8) | (rd + f)`. Pure
  concatenation, no stride arithmetic -- which is what distinguishes it from
  the strided shared-ring read that produced garbage on hardware.
- `nslots` 4 bits (1..8)
- `fpp` (8 or 16; pacing already keys off `ts_sub[0:blk_bits]`, so blk_bits
  becomes per-context)
- `ip_totlen` / `udp_len` / `last_be` derived from nslots * fpp * 3

Firmware then binds the requester's IP and MAC (ARP cache, or learned from the
request frame) with a recomputed IP checksum, exactly as `bind_flow()` already
does for multicast, and sets `g_flows_stats.active` so the talker ungates.

## Why this replaced multicast

We sourced all six bundles unconditionally: 65.5 Mbit/s of 69.6 Mbit/s measured
on the segment, 94% of all traffic, flooded to every port by an unmanaged
switch. The A16R was filtering 65 Mbit/s in hardware while playing two
channels. After withdrawing it: 4.1 Mbit/s total, our share 0.03.
