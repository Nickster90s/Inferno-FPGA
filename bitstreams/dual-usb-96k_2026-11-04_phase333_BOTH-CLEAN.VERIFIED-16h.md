# The triple that ran 16 h 8 min without a wedge (2026-11-05 → 2026-11-06)

This file exists so the exact combination can be reproduced, because the one
thing that has repeatedly gone wrong in this project is not knowing which
bitstream, which header and which firmware were actually in the board.

| part | identity |
|---|---|
| bitstream | `dual-usb-96k_2026-11-04_phase333_BOTH-CLEAN.bit`, md5 `c9f871a11e1b528a04b677f7cbc7a740` |
| CSR header | `dual-usb-96k_2026-11-04_phase333_BOTH-CLEAN.csr.h` (beside this file), md5 `6ac14ecefba0fad404ab13475201da4e` |
| firmware source | git `7008268`, tag `verified-16h-96k_2026-11-06` |
| firmware image | 92252 bytes, sha256 `5e2fdc9d…b93321c3`, body crc32 `0x423ddf3b` |
| sample rate | 96 kHz, 2 unicast flows (4 slots + 1 slot), fpp 8 |

The header is archived here on purpose. It was generated at 13:49:29 and the
bitstream placed at 13:58:09 on 2026-11-04 — the correct order — and it lived
only in `build_phase333_s1/`, which is not under git and which any later build
can delete. `flash.sh` already warns about this pairing; now it cannot be lost.

## What the run showed

Booted 2026-11-05 17:37:25, read out 2026-11-06 09:45 and still running:

    packets 1394118570   enables 1       disables 0     underrun 129
    flows_active 2       anchors 4       ptp_unlocks 1  ptp_steps 4
    off_unlock 0  off_noflow 0  off_rate 0  ub_arc 0  ub_expire 0  ub_drop 0

Both flows were still on the handles they were given at boot (`0/1` and `1/16`,
age 58089 s): no rebind, no talker drop, in sixteen hours.

`watch_long.sh` polled every 30 s for the first four hours — 480 polls, every
one `talker=1 disables=0` with the packet counter advancing at 24027 pps. The
log is `captures/ring-wrap_16h_2026-11-06.log`.

## Why this matters for the ring wrap

`rd` advances by `channels` per media strobe (`dante_packetizer.py:816`,
`channels = 8`), so the 2^32 wrap is rate-dependent:

    48 kHz:  8 x 48000 = 384000/s  ->  2^32/384000 = 11185 s = 3.107 h
    96 kHz:  8 x 96000 = 768000/s  ->  2^32/768000 =  5592 s = 1.553 h

`en` was high for the whole 58089 s, so the pointers crossed 2^32 about **ten
times** with `underrun` pinned at 129. The modulo level computation
(`dante_packetizer.py:448-456`) is therefore verified in hardware, not only in
`sims/sim_ring_wrap.py`. The 3.107 h figure belongs to 48 kHz; at 96 kHz the
old bug would have been nine times more frequent, not less.

## Telemetry over the night (235997 rows, archived outside git)

    events:   talker_on 1, ptp_lock 1, anchor 4, flow_bind 5, flow_unbind 5
              -- NO talker_off, NO ptp_unlock
    filtered: -3977 .. +6620 ns

That +6620 ns is worth reading twice: it is **above** `UNLOCK_THRESHOLD_NS`
(5000) and it did not unlock anything, because `UNLOCK_STREAK` is 8. The streak
widened on 2026-11-05 is doing real work, not sitting idle. It is also below
`OUTLIER_REJECT_NS` (10000), so the gate correctly left it in the servo.

The five bind/unbind pairs are boot churn: they are inside the first minute,
and `ub_stop` has not moved since.

## Raw data

Too large for git, kept at `/home/lisp/DANTE/FPGA/evidence/2026-11-06_ring-16h/`:
`wedge.log`, `event-ring.log`, `telemetry_overnight.csv.gz`,
`telemetry_afternoon.csv.gz`, `firmware_7008268.bin`, `csr.h`.

## What is NOT preserved

The board is running a **volatile** `pld load`. One power cycle rolls it back to
whatever is in SPI flash. `./flash.sh --permanent --bit bitstreams/dual-usb-96k_2026-11-04_phase333_BOTH-CLEAN.bit`
writes it for real, but that needs the audio stopped (flashing while streaming
took out both PHYs and the CH347 on 2026-11-04), so it is a deliberate
operator action, not something to slip in.
