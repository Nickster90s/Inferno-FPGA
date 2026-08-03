# rx_gate — a hardware RX MAC allow-list

Plan risk #8 ("CPU swamped by flooded Dante multicast"). This is the fix for the
`mac_writer_err` climb, which `TELEMETRY_AND_PTP.md` identifies as the *source*
of the PTP offset outliers that `ptpv1.c`'s unlock hysteresis currently masks.

## The problem, measured

The bench switch is unmanaged and floods every multicast group to every port. Of
3588 pps arriving here, 99.2% is other devices' audio on UDP 4321
(`main.c:168`). The LiteEth MAC has **2 RX slots** and cannot have more —
`nrxslots=4` silently kills TX, confirmed twice on two seeds
(`avb_soc.py:484`). The CPU cannot drain 2 slots against that rate, the writer's
status FIFO backs up, and `ethmac_sram_writer_errors` climbs ~25/s.

That counter is real received-frame loss, and what it loses is not only audio. A
dropped PTPv1 FollowUp or DelayResp mispairs with the wrong Sync and injects
±5–10 µs of offset error against a sub-microsecond steady state — which was
cycling PTP lock/unlock every ~30 s, and every unlock stops the talker and
re-anchors the media clock.

### Why the existing software filter is not enough

`dispatch_rx()` already has a MAC allow-list (`main.c:184-223`). It saves the
~435-byte Wishbone memcpy per frame, and it was worth adding — ICMP replies had
been lost at up to 46%. But it runs **after the frame has already been committed
to a slot**, and the slot is the resource that is actually exhausted. Software
cannot fix `writer_errors` from there, by construction. That is the gap this
module closes, and it is why both filters exist rather than one replacing the
other.

## What it does

`rx_gate.py` observes the MAC RX stream, latches the destination MAC out of the
first two 32-bit beats, and drives LiteEth's `discard_in` at end-of-frame. A
discarded frame never pushes the status FIFO, never raises `ev_pending`, and
**never advances the slot pointer** — so it never occupies one of the two slots.

It is **not** the `rx_wired` stream seam the plan expected. No LiteEth RX
re-plumbing was needed; see `LITEETH_PATCHES.md` #5 and #6.

Two registered pipeline stages keep the 48-bit comparator tree off the
end-of-frame path: beat 2 latches the address, beat 3 latches the verdict, and
the verdict is only trusted from beat 4 (a minimum Ethernet frame is 16 beats).
A frame shorter than that fails **open** — never discarded.

## The allow-list is wider than the task asked for, deliberately

Accepted:

| Rule | Covers |
|---|---|
| `ff:ff:ff:ff:ff:ff` | broadcast (ARP) |
| our unicast MAC (CSR) | ARC, CMC, stats, netload, unicast flow control |
| `01:00:5e:00:00:xx` | **all of 224.0.0.0/24** — mDNS `.251`, Dante info `.231`, heartbeat `.233`, **and IGMP's all-hosts `.1`** |
| `01:00:5e:00:01:81` | 224.0.1.129 — PTPv1 |
| `01:80:c2:xx:xx:xx` | 802.1 reserved (STP/LLDP/gPTP) |
| 2 × spare CSR slots | anything found missing, without a rebuild |

The brief named four exact multicast groups. That set is **narrower than the
software filter already running**, and `main.c:200-204` records why that matters:

> "An earlier version enumerated only the three Dante groups and silently
> swallowed IGMP queries. Nothing breaks immediately, which is what makes it
> nasty: memberships just age out and the switch quietly stops forwarding our
> groups."

So the gateware list is built to satisfy an invariant instead of an enumeration:

> **Every address the software filter keeps, the gateware keeps.**

Arming the gate therefore cannot change what the control plane sees — it can
only stop the flood from taking slots. `sims/sim_rx_gate.py` checks this by
modelling both filters and sweeping 262,401 group addresses for a counterexample,
rather than leaving it as a claim in a comment.

The one deliberate *tightening* is unicast: the software filter keeps all of it
(it only inspects group addresses), the gate keeps only ours. A switch floods
unicast only for an unlearned MAC, and that is the other route by which a
neighbour's traffic reaches this port.

Excluded is `239.x.x.x` → `01:00:5e:7f:xx:xx`, the Dante audio range. That is the
whole flood.

## Safety: it is off until asked, and it undoes itself

A wrong allow-list drops **all** RX — no ARP, no PTP, no mDNS, no ARC, no stats.
Then the packet you would need to send to turn it off is the one being dropped.
Three layers guard against that:

1. **`rx_gate.enable` resets to 0.** The new bitstream behaves exactly like the
   old one until firmware arms it. Firmware never arms it at init.
2. **Arming over the network is provisional.** `tools/rx_gate.py on` sets a 30 s
   auto-revert; `commit` cancels it. If contact is lost, wait 30 s.
3. **The console is the escape hatch.** `x` toggles the filter over UART, which a
   MAC filter cannot lock out. (Console arming is permanent — it needs no timer.)

## Measured on hardware, 2026-08-01

Same bitstream, same session, filter toggled at runtime — so this is a true A/B,
not a comparison across builds. 400 unicast stats round-trips each way, talker
running, two unicast flows active, PTP locked throughout.

| | filter ARMED | filter OFF |
|---|---|---|
| **control-plane round-trips lost** | **0 / 400 (0.00%)** | **84 / 400 (21.0%)** |
| `writer_errors` | 8.4/s | 61.1/s |
| gate `discarded` | 3000.7/s | 0 |
| `sw_filtered` (software filter) | 0/s | ~2957/s |
| `underrun` / `overrun` delta | 0 / 0 | 0 / 0 |
| PTP unlocks / re-anchors (90 s) | 0 / 0 | 0 / 0 |
| `offset_ns` range (90 s, n=60/31) | 255…469 ns | 635…954 ns |

**The headline is the first row, not the second.** With the filter off, more than
a fifth of unicast round-trips to the board simply never came back. With it on,
none were lost in 400 attempts. That is the frame loss `TELEMETRY_AND_PTP.md`
blames for the PTP outliers, measured directly at the thing that suffers from it.

`discarded` at 3000.7/s exactly matches one Dante flow's 48000/16 = 3000 pps, and
`sw_filtered` drops to precisely 0 — the gateware is taking every frame the
software filter used to, one level earlier.

### Why `writer_errors` does not go to zero, and why that is fine

It falls 61.1 → 8.4/s, not to 0. The residual is **not** lost control traffic:

`LiteEthMACSRAMWriter` increments `errors` on *any* beat that arrives while its
2-deep status FIFO is full (`sram.py:102-105`), and it does so *before* the
destination MAC has been classified. `discard_in` is only consulted on the
`sink.last` branch of the WRITE state. So when the CPU is briefly slow to ack two
pending slots, the next frame to arrive bumps the counter — and at 3000 flood
frames/s against ~30–60 wanted frames/s, that frame is a flood frame ~98% of the
time. It was going to be discarded anyway.

The 400-query test settles it. In a 13 s armed window the counter took 105
errors while 741 wanted frames arrived. If those 105 had been wanted frames, that
is a ~14% loss rate and we would have seen roughly 55 lost replies. We saw **0**.

So `mac_writer_err` currently conflates "aborted a frame" with "lost a frame we
wanted", and after rx_gate the two have almost nothing to do with each other.
Treat **round-trip loss**, not `writer_errors`, as the honest instrument. Making
the counter mean what its name implies is a separate one-line-ish change to
`sram.py` — defer the increment to end-of-frame and skip it when `discard_in` is
high — and is *not* done here.

## Measured baseline (before, on the pre-rx_gate bitstream)

Taken 2026-08-01 with two unicast flows active and the talker running, over a
30 s window via `tools/stats.py 169.254.9.200 30`:

| Counter | Delta over 30 s | Rate |
|---|---|---|
| `mac_writer_err` | **+1214** | **~40/s** |
| `underrun` | 0 | 0 |
| `overrun` | 0 | 0 |
| `ptp_locked` | 1 throughout | — |
| `offset_ns` | 111 → −183 | — |

Higher than the ~25/s previously recorded — the flood is not a fixed rate, so
the "after" number must be compared against a baseline taken in the same
session, not against this table. One `tools/stats.py` query in this run timed out
outright with no reply, which is the same frame loss seen from the other end.

## Verifying it, in order

The classifier runs whether or not the filter is armed, so there is a real dry
run before anything is risked.

```
tools/rx_gate.py status         # nomatch/last_drop with the filter still OFF
tools/rx_gate.py watch 30       # baseline writer_errors/s
tools/rx_gate.py on             # provisional: auto-reverts in 30 s
tools/rx_gate.py status         # THIS is the real test -- a reply at all means
                                # RX still works while armed
tools/rx_gate.py commit         # only once that reply came back
tools/rx_gate.py watch 30       # discarded climbing, writer_errors -> ~0
```

Commit *before* the long measurement window, not after: a 30 s `watch` would
otherwise race the 30 s auto-revert and measure a filter that switched off
halfway through.

1. **Flash with the filter disabled — networking unchanged.** `ping`,
   `tools/stats.py`, `avahi-browse -r _netaudio-chan._udp`, PTP lock.
2. **Dry run.** `nomatch` should climb at roughly the flood rate (~3500/s) and
   `last_drop` should read `01:00:5e:7f:xx:xx`. If it does not, the gateware is
   not seeing what you think it is — stop here. `discarded` must be 0.
3. **Arm it.** `discarded` must start climbing (this is the proof the enable
   reached hardware, not a counter that would have moved anyway), `writer_errors`
   delta must go ~25/s → 0, and `sw_filtered` must go flat as the gateware takes
   the frames the software filter used to.
4. **Control plane intact:** ARC, mDNS, PTP lock, `tools/stats.py` all still
   answer. Then `commit`.
5. **`[ptpv1] outlier ... ignored` console lines stop appearing.**
6. **Audio still plays**, `underrun`/`overrun` deltas at 0.

`tools/rx_gate.py off`, or `x` on the console, backs all of it out instantly.

## Scope note

A green `mac_writer_err` fixes the *source* of the PTP outliers. It does **not**
address the bad audio at stream start, which is a separate open problem and may
be unrelated. Do not read one as evidence for the other.

## Files

| File | Role |
|---|---|
| `rx_gate.py` | the Migen module |
| `sims/sim_rx_gate.py` | RTL sim + the superset-invariant sweep |
| `avb_soc.py` | instantiation, tap onto `mac.core.source`, `discard_in` |
| `firmware/rx_gate.[ch]` | CSR programming, auto-revert, readout |
| `firmware/dstats.c` | `'g'` opcode on UDP 7779 (separate reply, see below) |
| `firmware/main.c` | `rx_gate_init/poll`, `g` and `x` console commands |
| `tools/rx_gate.py` | host control and measurement |

The rx_gate readout is a **separate** UDP record, not extra fields on the main
stats reply: growing that reply from 200 to 208 bytes once killed the port
outright, cause never found (`dstats.c:60-65`). A different reply is not a bigger
one. It carries a version tag so the host parses by name, never by hand-counted
offset — the rule `TELEMETRY_AND_PTP.md` sets after two wrong conclusions came
from stale offsets.
