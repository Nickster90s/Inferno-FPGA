# Migen simulation of rx_gate.RXGate.
#
# This is a REAL RTL sim (run_simulation), not a Python model, because the thing
# most likely to be wrong here is not the allow-list -- it is the byte order of
# the 32-bit MAC stream and the cycle on which discard_in is sampled. Both are
# properties of the RTL, and both have bitten this project before (the AVTP
# extractor's byte_at() shift, 2026-05-22).
#
# What is checked:
#   1. dst MAC is extracted correctly from the LSB-first 32-bit beats.
#   2. Broadcast / our unicast / each of the four allowed multicast groups pass.
#   3. Another device's audio multicast (239.255.x.y -> 01:00:5e:7f:xx:yy) drops.
#   4. discard is HIGH on the sink.last cycle for a dropped frame and LOW for an
#      allowed one -- that exact cycle is the only one LiteEthMACSRAMWriter looks
#      at (liteeth/mac/sram.py:96).
#   5. With enable=0 the classifier still counts, but discard NEVER asserts.
#   6. A runt (fewer than 4 beats) is never discarded -- fail open.
#   7. A spare CSR slot can whitelist an extra MAC at runtime.
#
#   run:  python3 sims/sim_rx_gate.py

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from migen import *

from rx_gate import (RXGate, MCAST_MDNS, MCAST_PTPV1,
                     MCAST_DANTE_INFO, MCAST_DANTE_HEART, MCAST_ALL_HOSTS)

OUR_MAC   = 0x02005e112233
BROADCAST = 0xffffffffffff
OTHER_AUDIO = 0x01005e7f0102   # 239.255.1.2 -- another device's Dante flow
DOT1_RESERVED = 0x0180c200000e # 802.1 reserved (LLDP)
SPARE_MAC   = 0x01005e401234   # 224.64.18.52 -- nothing uses it; a clean spare test


def frame_beats(dst_mac, n_bytes=64):
    """Build the 32-bit beat list for a frame with this destination MAC.

    Byte 0 of the frame sits in data[7:0], byte 3 in data[31:24].
    """
    b = [(dst_mac >> (8 * (5 - i))) & 0xff for i in range(6)]   # dst
    b += [0x02, 0x00, 0x5e, 0xaa, 0xbb, 0xcc]                   # src
    b += [0x08, 0x00]                                           # ethertype
    b += [(i * 7) & 0xff for i in range(max(0, n_bytes - len(b)))]  # filler
    b = b[:max(n_bytes, 6)]                                     # allow short frames
    beats = []
    for i in range(0, len(b), 4):
        chunk = b[i:i + 4]
        while len(chunk) < 4:
            chunk.append(0)
        beats.append(chunk[0] | (chunk[1] << 8) | (chunk[2] << 16) | (chunk[3] << 24))
    return beats


class Bench(Module):
    def __init__(self):
        self.submodules.dut = RXGate(n_spare=2)


def csr_write(dut, csr, value):
    """Poke a CSRStorage's storage register directly (no CSR bus in this sim)."""
    yield csr.storage.eq(value)
    yield


def send_frame(dut, dst_mac, n_bytes=64, gap=3):
    """Drive one frame; return (discard_on_last, beats_sent)."""
    beats = frame_beats(dst_mac, n_bytes)
    discard_on_last = None
    discard_seen_high = False
    yield dut.sink.ready.eq(1)
    for i, beat in enumerate(beats):
        last = (i == len(beats) - 1)
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(beat)
        yield dut.sink.last.eq(1 if last else 0)
        yield          # advance one clock: signals above are sampled at its edge
        if (yield dut.discard):
            discard_seen_high = True
        if last:
            discard_on_last = (yield dut.discard)
    yield dut.sink.valid.eq(0)
    yield dut.sink.last.eq(0)
    for _ in range(gap):
        yield
    return discard_on_last, discard_seen_high, len(beats)


def run():
    bench = Bench()
    dut = bench.dut
    results = []

    def tb():
        # ---- 8. Interlock: arming before the local MAC is programmed must do
        # nothing at all. Firmware is meant to program it at init, but the
        # failure mode if it ever does not is losing every unicast frame --
        # including the one that would turn the filter back off -- so the
        # gateware refuses rather than trusting the convention.
        yield from csr_write(dut, dut.enable, 1)
        for _ in range(4):
            yield
        d, high, _ = yield from send_frame(dut, OTHER_AUDIO)
        results.append(("interlock: armed with local_mac unset drops nothing",
                        (d == 0) and not high))
        results.append(("interlock: discard_count stays 0 while held off",
                        (yield dut.discard_count.status) == 0))
        yield from csr_write(dut, dut.enable, 0)
        for _ in range(4):
            yield

        # Program our unicast MAC, leave the filter DISABLED.
        yield from csr_write(dut, dut.local_mac_hi, OUR_MAC >> 32)
        yield from csr_write(dut, dut.local_mac_lo, OUR_MAC & 0xffffffff)
        yield from csr_write(dut, dut.enable, 0)
        for _ in range(4):
            yield

        # ---- 5. Dry run: classification works, discard never asserts. ----
        d, high, _ = yield from send_frame(dut, OTHER_AUDIO)
        results.append(("disabled: audio frame never discarded",
                        (d == 0) and not high))
        results.append(("disabled: audio frame counted as nomatch",
                        (yield dut.nomatch_count.status) == 2))
        results.append(("disabled: last_drop_lo captured the right MAC",
                        (yield dut.last_drop_lo.status) == (OTHER_AUDIO & 0xffffffff)))
        results.append(("disabled: last_drop_hi captured the right MAC",
                        (yield dut.last_drop_hi.status) == (OTHER_AUDIO >> 32)))
        results.append(("disabled: discard_count stays 0",
                        (yield dut.discard_count.status) == 0))

        # ---- Arm the filter. ----
        yield from csr_write(dut, dut.enable, 1)
        for _ in range(4):
            yield

        # ---- 2. Everything on the allow-list passes. ----
        for name, mac in [
            ("broadcast",       BROADCAST),
            ("our unicast",     OUR_MAC),
            ("mDNS",            MCAST_MDNS),
            ("PTPv1",           MCAST_PTPV1),
            ("Dante info",      MCAST_DANTE_INFO),
            ("Dante heartbeat", MCAST_DANTE_HEART),
            # 224.0.0.1. The software filter (main.c:200-204) records an
            # incident where enumerating only the Dante groups swallowed IGMP
            # queries and memberships silently aged out. This gate accepts the
            # whole 224.0.0.0/24 link-local scope so that cannot recur.
            ("IGMP all-hosts", MCAST_ALL_HOSTS),
            # Another address in the same /24 that nothing has enumerated --
            # proves the rule is a prefix, not a longer list of constants.
            ("unenumerated 224.0.0.x", 0x01005e000042),
            ("802.1 reserved (LLDP)", DOT1_RESERVED),
        ]:
            d, high, _ = yield from send_frame(dut, mac)
            results.append((f"enabled: {name} passes", (d == 0) and not high))

        # ---- 3./4. Another device's audio is discarded, on the last cycle. ----
        before = (yield dut.discard_count.status)
        d, _, _ = yield from send_frame(dut, OTHER_AUDIO)
        after = (yield dut.discard_count.status)
        results.append(("enabled: foreign audio discard high on sink.last", d == 1))
        results.append(("enabled: discard_count incremented", after == before + 1))

        # A different foreign group, to be sure it is not one specific value.
        d, _, _ = yield from send_frame(dut, 0x01005e7f00ff)
        results.append(("enabled: second foreign group discarded", d == 1))

        # ---- 6. A runt must fail OPEN (never discarded). ----
        # 8 bytes = 2 beats, so the verdict register never becomes valid.
        d, high, nb = yield from send_frame(dut, OTHER_AUDIO, n_bytes=8)
        results.append((f"enabled: {nb}-beat runt is not discarded (fail open)",
                        (d == 0) and not high))

        # And the frame AFTER a runt must still be classified correctly --
        # i.e. the runt did not leave stale state behind.
        d, _, _ = yield from send_frame(dut, MCAST_PTPV1)
        results.append(("enabled: PTPv1 still passes after a runt", d == 0))
        d, _, _ = yield from send_frame(dut, OTHER_AUDIO)
        results.append(("enabled: audio still dropped after a runt", d == 1))

        # ---- 7. Spare slot whitelists an extra MAC at runtime. ----
        d, _, _ = yield from send_frame(dut, SPARE_MAC)
        results.append(("enabled: unlisted group dropped before spare is set", d == 1))
        yield from csr_write(dut, dut.spare0_mac_hi, SPARE_MAC >> 32)
        yield from csr_write(dut, dut.spare0_mac_lo, SPARE_MAC & 0xffffffff)
        for _ in range(4):
            yield
        d, _, _ = yield from send_frame(dut, SPARE_MAC)
        results.append(("enabled: unlisted group passes once spare0 is set", d == 0))

        # ---- Back out instantly. ----
        yield from csr_write(dut, dut.enable, 0)
        for _ in range(4):
            yield
        d, high, _ = yield from send_frame(dut, OTHER_AUDIO)
        results.append(("enable=0 restores pass-through immediately",
                        (d == 0) and not high))

        # ---- 1. Sanity on the counters as a whole. ----
        # 8 allowed: broadcast, unicast, 4 multicast groups, PTPv1-after-runt,
        # spare0. 6 denied: 4 x OTHER_AUDIO (one while disabled and one after
        # enable was cleared -- classification is enable-independent by design),
        # the second foreign group, and 224.0.0.1 before spare0 was set.
        # The runt is in NEITHER: match + nomatch < frames sent is how an
        # undecided frame shows up.
        m = (yield dut.match_count.status)
        n = (yield dut.nomatch_count.status)
        results.append((f"match_count counted every allowed frame (got {m})", m == 11))
        results.append((f"nomatch_count counted every denied frame (got {n})", n == 7))

    run_simulation(bench, tb())

    fails = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if not ok:
            fails += 1
    print(f"\n{len(results) - fails}/{len(results)} checks passed")
    return 1 if fails else 0




# ---------------------------------------------------------------------------
# The superset invariant, checked rather than asserted in a comment.
#
# rx_gate's allow-list is only trustworthy to the extent that arming it cannot
# change what the control plane sees. That holds iff every address the SOFTWARE
# filter in firmware/main.c:184-223 keeps is also kept by the gateware. Model
# both and sweep the group-address space looking for a counterexample.
# ---------------------------------------------------------------------------

def sw_keeps(mac):
    """firmware/main.c:184-223, transcribed."""
    d = [(mac >> (8 * (5 - i))) & 0xff for i in range(6)]
    if not (d[0] & 0x01):
        return True                                  # all unicast kept
    if d[0] == 0xFF and d[1] == 0xFF and d[2] == 0xFF:
        return True                                  # broadcast
    if d[0] == 0x01 and d[1] == 0x00 and d[2] == 0x5E:
        if d[3] == 0x00 and d[4] == 0x00:
            return True                              # 224.0.0.0/24
        if d[3] == 0x00 and d[4] == 0x01 and d[5] == 0x81:
            return True                              # 224.0.1.129 PTPv1
        return False
    if d[0] == 0x01 and d[1] == 0x80 and d[2] == 0xC2:
        return True                                  # 802.1 reserved
    return False


def hw_keeps(mac, local_mac):
    from rx_gate import DEFAULT_ALLOW_RULES, MAC_BROADCAST
    if mac == MAC_BROADCAST or mac == local_mac:
        return True
    return any((mac & m) == v for m, v in DEFAULT_ALLOW_RULES)


def check_superset():
    bad = []

    def sweep(mac):
        # Unicast is deliberately narrowed (software keeps every unicast; the
        # gate keeps only ours), so the implication is asserted for GROUP
        # addresses only -- which is where the flood lives and where a wrong
        # answer breaks the control plane.
        if not ((mac >> 40) & 0x01):
            return
        if sw_keeps(mac) and not hw_keeps(mac, OUR_MAC):
            bad.append(mac)

    sweep(BROADCAST)
    for d2 in range(256):
        sweep(0x0180c2000000 | (d2 << 16) | d2)
    # IPv4 multicast: sweep the boundaries of the /24 rule plus the audio range.
    for d3 in (0x00, 0x01, 0x02, 0x7f):
        for d4 in range(256):
            for d5 in range(256):
                sweep(0x01005e000000 | (d3 << 16) | (d4 << 8) | d5)

    n = 4 * 256 * 256 + 256 + 1   # mcast sweep + 802.1 sweep + broadcast
    if bad:
        print(f"  FAIL  superset invariant: {len(bad)} addresses the software "
              f"filter keeps but the gate would drop, e.g. "
              f"{bad[0]:012x}")
        return 1
    print(f"  PASS  superset invariant holds over {n} swept group addresses "
          f"(software-keep => gateware-keep)")
    return 0


if __name__ == "__main__":
    rc = run()
    rc |= check_superset()
    sys.exit(rc)
