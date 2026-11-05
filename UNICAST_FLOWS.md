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

## How a flow is torn down -- documented, then measured

Three mechanisms exist, and only two of them are ours to get right.

**1. The receiver says stop: flow control 0x0101.** Content is the 6-byte
handle from our 0x0100 reply (flow index u32 BE, then a u16 cookie). This is
the normal case and we were failing it: `dante_flows.c` handled 0x0100 only, so
a stop fell into the unhandled branch, got `0x0103` back, and the flow kept
transmitting. A DAC8 on this bench sends a stop on every patch change, which is
why contexts to devices that had long since unsubscribed were still on the wire
hours later. Handled as of 2026-11-05, with BOTH halves of the handle checked:
indices are reused between bindings, so a stale handle naming slot 2 would
otherwise tear down whatever was bound into slot 2 afterwards. The cookie used
to be a local `static` in `dante_flows.c` that incremented per reply and was
stored nowhere; it now lives in the flow and is kept across a refresh, so the
handle a receiver holds stays valid.

**2. Keepalive expiry -- on the MEDIA port, which is where the keepalives
actually are.** Measured here 2026-11-05, our single flow to `:14337`:

```
age 1147813 ms -> 1162815 -> 1177817 -> 1192819 -> 1207821
exactly +15002 ms per 15 s poll, i.e. ZERO refreshes in 19 minutes
```

and `tcpdump -i ens5 'udp port 4455'` saw 0 packets in 20 s. (Weak on its own:
flow control is unicast and the switch is unmanaged, so the host port does not
see it. `age_ms` is the stronger reading -- `dante_tx.c` writes `last_ms` in the
bind paths only and every flow-control request passes through
`dante_tx_bind_unicast`, so it IS the keepalive age.)

So 4455 request age is not a heartbeat and no timeout on it can be both safe
and useful -- which is exactly why both earlier attempts silenced live flows, at
300 s and at 600 s. UNICAST_FLOWS' own "repeated every ~5 s" was measured while
those requests were being REJECTED; once we started accepting, the refreshes
stopped.

The keepalives were on a port nothing was bound to. A receiver talks back on
**our media port 4321**, from the same socket it receives the flow on, so those
datagrams were being discarded by the UDP demux without so much as a counter.
`media_rx` in `dante_tx.c` binds it and counts them per peer; `tools/flows.py`
shows the count and its age; `tools/cap_fetch.py` dumps the payload. Captured
off the board's own ring, because the host cannot see it -- unicast, unmanaged
switch -- with both flows playing audio:

| source | device | payload | interval |
|--------|--------|---------|----------|
| 169.254.145.57 | Leine Prodigy `03-Prodigy65-96`, `server_vers=4.2.0` | `02 00 00 00` + u32 LE counter | **exactly 250 ms**, counter +1 per second |
| 169.254.127.2  | N-Series DAC8 (firmware written on this bench)      | `13 37 00 ..`, constant       | ~104 ms (min 100, max 109) |

The Prodigy matters: it is a shipping Dante receiver, so this is the protocol as
real hardware speaks it, not a private convention with one in-house device. The
DAC8's `1337` is its own invention and would not be enough to build on alone.

`dante_tx_expire()` is therefore RE-ENABLED, keyed on these counters, with three
rules that each answer a specific way the earlier versions failed:

- **No timestamp arithmetic.** The test is "did the counter move since the
  previous one-second poll", never "how long ago was the last one".
  `gptp_uptime_ms()` is PTP-derived and a clock step is what defeated every
  previous attempt -- it inflates every age at once and expires the lot. A step
  can now only delay a poll.
- **5 consecutive silent seconds.** The Prodigy sends 4/s and the DAC8 ~10/s, so
  a single silent poll already means 4-10 missed in a row.
- **A flow is only ever expired if it has sent at least one.** The A16R and AM2
  are off this network, so whether they use this port is UNMEASURED. Until it
  is, they behave exactly as before: released by an explicit 0x0101 or by
  eviction on demand, never by this.

**3. Eviction on demand**, in the bind paths. Reclaims a context only when
another receiver needs one, prefers unicast victims, picks the oldest. This is
the backstop for a receiver that loses power and so can never send a stop. It
cannot fire against a flow nobody is competing for, which is its whole virtue
and also its limit.

**4. The receiver gives up because WE stopped sending.** Not a teardown
mechanism at all, but it looks exactly like one from the flow table, and on
2026-11-05 it cost two sessions an afternoon of chasing the wrong thing.

Measured, from the event ring:

```
t=9014884  ptp_unlock (filtered -15892 ns, streak 4)
t=9014885  talker_off                      <- 1 ms later; all audio stops
t=9019944  flow_unbind f1  }  5.1 s later
t=9020218  flow_unbind f0  }  5.3 s later  <- BOTH receivers, as a CONSEQUENCE
```

Both bindings then jump to new cookies, `rebinds` climbs on both, and
`flows_requests` runs up -- the exact signature of "something is tearing our
flows down". Nothing was: a PTP unlock had disabled the talker, and after ~3 s
of silence each receiver released its own flow and re-requested.

**So before concluding a flow was released, check whether we were still
transmitting.** `tools/stats.py` gives `talker` and a `packets` delta; a
`packets` delta of zero means the flows are collateral, not cause. The per-cause
counters in `tools/flows.py` (`ub_stop`/`ub_expire`/`ub_drop` against
`off_noflow`/`off_unlock`) exist for this: `ub_stop` rising while `off_noflow`
follows it is receivers leaving; `off_unlock` rising first is us silencing them.

0x0102 (update) is deliberately not handled: no device here has been observed
sending one, and answering an opcode we have never seen would be guessing at
its content.

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
