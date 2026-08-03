# rx_gate.py — a destination-MAC allow-list for the LiteEth RX path.
#
# WHY THIS EXISTS
#
# The bench switch is unmanaged: it floods every multicast group to every port.
# Other Dante devices' audio flows (thousands of frames/s) therefore land on our
# LiteEth MAC, which has exactly 2 RX slots (nrxslots cannot be raised -- 4
# silently kills TX, see avb_soc.py:484). The CPU cannot drain 2 slots fast
# enough against that rate, so LiteEthMACSRAMWriter's status FIFO backs up and
# `ethmac_sram_writer_errors` climbs ~25/s. That counter is REAL received-frame
# loss, and the frames it loses are not only audio: a dropped PTPv1 FollowUp or
# DelayResp mispairs with the wrong Sync and injects +/-5-10 us of offset error
# against a sub-microsecond steady state. See TELEMETRY_AND_PTP.md section 2.
#
# HOW IT WORKS -- and why there is no stream surgery here
#
# The plan (risk #8) assumed this needed an `rx_wired` seam in LiteEth mirroring
# the existing `tx_wired` patch, so the gate could sit IN the RX stream between
# core.source and interface.sink. It does not. LiteEthMACSRAMWriter already
# carries a local patch -- `discard_in` (liteeth/mac/sram.py:32) -- added for the
# AVB AAF extractor: when high on the cycle of sink.last, the writer's FSM goes
# to DISCARD instead of TERMINATE, so the frame
#
#   * never pushes the status FIFO  -> never raises ev_pending
#   * never advances the slot pointer -> NEVER CONSUMES AN RX SLOT
#   * never pushes the rx_ts ring    -> RX-timestamp ring stays in lock-step
#
# The frame's bytes are written into the current (still uncommitted) slot RAM and
# are simply overwritten by the next frame. That is exactly the outcome we want
# and it costs no stream mux, no backpressure handling, and no partial-frame
# rollback problem. So this module is an OBSERVER on mac.core.source that drives
# one wire into the writer. Nothing in the RX datapath is re-plumbed.
#
# RUNTIME-DISABLED BY DEFAULT -- READ THIS BEFORE CHANGING IT
#
# `enable` resets to 0. A wrong allow-list drops ALL RX: no ARP, no PTP, no mDNS,
# no ARC, no tools/stats.py -- and then the packet carrying "turn it off" is
# itself dropped. With enable defaulting to 0 the new bitstream behaves EXACTLY
# like the old one until firmware asks for the filter, and writing 0 backs it out
# instantly. Three more layers back that up:
#
#   * a HARDWARE INTERLOCK below: the gate is inert until the local unicast MAC
#     CSR is non-zero, so a firmware path that forgets to program it cannot lock
#     the board out;
#   * firmware auto-reverts a network-requested arm after 30 s unless committed
#     (firmware/rx_gate.c);
#   * the 'x' console command, which a MAC filter cannot lock out.
#
# The classifier runs unconditionally, whether or not `enable` is set. That is
# deliberate: `nomatch_count` and `last_drop_*` let you confirm from the host
# WHAT WOULD BE DROPPED, and that the gateware sees the MACs you think it sees,
# BEFORE you arm it. `discard_count` only moves when frames are actually being
# discarded, which is how you prove the enable reached hardware rather than
# inferring it from a counter that would have moved anyway.

from functools import reduce

from migen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, AutoCSR


# Destination MACs that are always accepted, as 48-bit integers ordered the way
# the address is written (0x01005e0000fb == 01:00:5e:00:00:fb).
#
# IPv4 multicast maps to 01:00:5e:(ip[1] & 0x7f):ip[2]:ip[3].
MCAST_MDNS         = 0x01005e0000fb   # 224.0.0.251   mDNS  (discovery)
MCAST_PTPV1        = 0x01005e000181   # 224.0.1.129   PTPv1 (Sync/FollowUp/DelayResp)
MCAST_DANTE_INFO   = 0x01005e0000e7   # 224.0.0.231   Dante device-info
MCAST_DANTE_HEART  = 0x01005e0000e9   # 224.0.0.233   Dante heartbeat
MCAST_ALL_HOSTS    = 0x01005e000001   # 224.0.0.1     IGMP general queries

MAC_BROADCAST = 0xffffffffffff

# THE ALLOW-LIST IS A DELIBERATE SUPERSET OF THE SOFTWARE ONE.
#
# firmware/main.c:184-223 already carries a MAC allow-list in dispatch_rx(). It
# runs too late to help here -- by then the frame has already been committed to
# an RX slot, which is the resource that is actually exhausted -- but it has been
# on the bench, and its comments record an incident this gate must not repeat:
#
#   "An earlier version enumerated only the three Dante groups and silently
#    swallowed IGMP queries. Nothing breaks immediately, which is what makes it
#    nasty: memberships just age out and the switch quietly stops forwarding our
#    groups."
#
# So the hardware list matches the software list rather than the narrower
# four-group list in the task description: accept ALL of the 224.0.0.0/24
# link-local control scope (which contains mDNS .251, Dante device-info .231,
# Dante heartbeat .233 AND IGMP's all-hosts .1), plus 224.0.1.129 for PTPv1,
# plus the 802.1 reserved range. Holding "hardware allow-list is a superset of
# the software one" as an invariant means arming the gate cannot change what the
# control plane sees -- it can only stop the flood from taking slots. That is a
# much easier property to trust than an enumeration nobody has re-derived.
#
# What is excluded is 239.x.x.x -- the Dante audio range, 01:00:5e:7f:xx:xx --
# which is 99.2% of received frames on this bench (main.c:168) and the entire
# reason this module exists.
#
# Each entry is (mask, value): dst_mac & mask == value.
DEFAULT_ALLOW_RULES = (
    # 224.0.0.0/24 link-local control scope: mDNS, Dante info+heartbeat, IGMP.
    (0xffffffff0000, 0x01005e000000),
    # 224.0.1.129 -- PTPv1, the one group outside the link-local scope we join.
    (0xffffffffffff, MCAST_PTPV1),
    # 802.1 reserved (01:80:c2:xx:xx:xx): STP/LLDP/PAUSE and gPTP. Low rate, and
    # accepting it keeps the superset invariant above exactly true.
    (0xffffff000000, 0x0180c2000000),
)


class RXGate(Module, AutoCSR):
    """Destination-MAC allow-list observer for the LiteEth MAC RX stream.

    Connect `sink` to the SAME stream the SRAM writer sees (mac.core.source,
    which liteeth comb-connects straight through to interface.sink -> the
    writer's sink, so beats are cycle-identical), and `discard` to
    mac.interface.sram.writer.discard_in.

    Accepted: broadcast, our own unicast MAC (CSR-programmed), every rule in
    `allow_rules`, and up to `n_spare` runtime-programmable extra MACs.
    Everything else is dropped when `enable` is set.

    Note that unicast is narrowed relative to the software filter, which keeps
    ALL unicast (it only inspects group addresses). Dropping unicast that is not
    ours is correct -- a switch only floods it for an unlearned MAC -- and it is
    the one tightening here that is worth having, since unknown-unicast flooding
    is the other way a neighbour's traffic reaches this port.

    n_spare exists because a rebuild costs ~20 minutes AND re-rolls the P&R seed
    pin (a seed pin is a property of the (seed, netlist) pair). If it turns out a
    group is missing from the list -- an IGMP querier on 224.0.0.1, say -- a
    spare slot fixes it from firmware instead of from nextpnr.
    """

    def __init__(self, dw=32, allow_rules=DEFAULT_ALLOW_RULES, n_spare=2):
        assert dw == 32, "byte extraction below assumes a 32-bit MAC stream"

        # Observe-only: we never drive sink.ready. The writer already holds it
        # at 1 (sram.py sets sink.ready.reset = 1 and nothing else drives it).
        self.sink = stream.Endpoint([("data", dw), ("last_be", dw // 8)])

        # -> LiteEthMACSRAMWriter.discard_in. Sampled by the writer only on the
        # cycle of sink.last, but held stable from beat 4 to end-of-frame.
        self.discard = Signal()

        # ---- Control ----
        self.enable = CSRStorage(1, reset=0, description=
            "1 = drop RX frames whose destination MAC is not in the allow-list. "
            "RESETS TO 0: the bitstream behaves exactly as before until firmware "
            "arms it. Write 0 to back out instantly.")
        self.local_mac_hi = CSRStorage(16, reset=0, description=
            "Our unicast MAC [47:32]. Firmware must program this BEFORE enable, "
            "or every unicast frame -- ARC, stats, netload -- is dropped.")
        self.local_mac_lo = CSRStorage(32, reset=0, description=
            "Our unicast MAC [31:0].")

        spare_hi = []
        spare_lo = []
        for i in range(n_spare):
            hi = CSRStorage(16, name=f"spare{i}_mac_hi", reset=0, description=
                f"Extra allowed MAC {i} [47:32]. Slot is inactive while hi and lo "
                f"are both 0.")
            lo = CSRStorage(32, name=f"spare{i}_mac_lo", reset=0, description=
                f"Extra allowed MAC {i} [31:0].")
            setattr(self, f"spare{i}_mac_hi", hi)
            setattr(self, f"spare{i}_mac_lo", lo)
            spare_hi.append(hi)
            spare_lo.append(lo)

        # ---- Observability ----
        # match/nomatch count regardless of `enable`, so the allow-list can be
        # validated on hardware before it is armed. discard_count moves only when
        # a frame was actually discarded -- the proof the enable took effect.
        self.match_count = CSRStatus(32, description=
            "Frames classified ALLOW (counted whether or not enable is set).")
        self.nomatch_count = CSRStatus(32, description=
            "Frames classified DROP (counted whether or not enable is set). "
            "With enable=0 this is the dry-run: what the filter WOULD drop.")
        self.discard_count = CSRStatus(32, description=
            "Frames actually discarded (enable was set at end-of-frame).")
        self.last_drop_hi = CSRStatus(16, description=
            "Destination MAC [47:32] of the last frame classified DROP.")
        self.last_drop_lo = CSRStatus(32, description=
            "Destination MAC [31:0] of the last frame classified DROP.")

        # # #

        beat_accept = Signal()
        self.comb += beat_accept.eq(self.sink.valid & self.sink.ready)

        # Beat index within the frame, saturating so long frames don't wrap it
        # back onto the header beats.
        beat = Signal(max=8)

        # Header bytes. LiteEth mac.core.source.data carries frame byte 0 in the
        # LSB (data[7:0]) and byte 3 in the MSB (data[31:24]) -- the writer's
        # endianness="big" byte reversal applies to SRAM storage, not to the
        # stream. This was established empirically on hardware (2026-05-22) when
        # the opposite assumption made the AVTP extractor read byte 15 as byte 12
        # and fail every match; see _avb_reference/avtp_extractor.py:198.
        w0 = Signal(32)   # frame bytes 0..3
        w1 = Signal(16)   # frame bytes 4..5

        self.sync += If(beat_accept,
            If(beat == 0, w0.eq(self.sink.data)),
            If(beat == 1, w1.eq(self.sink.data[0:16])),
            If(beat != 7, beat.eq(beat + 1)),
            If(self.sink.last, beat.eq(0)),
        )

        # Cat() is LSB-first, so byte 5 goes at position 0: the resulting 48-bit
        # value reads in the same order the address is written.
        dst_mac_c = Cat(w1[8:16], w1[0:8],
                        w0[24:32], w0[16:24], w0[8:16], w0[0:8])

        # Two registered stages keep the 48-bit comparator tree off the
        # end-of-frame path entirely. Stage 1 latches the address (valid from
        # beat 2, since w1 is written on beat 1); stage 2 latches the verdict.
        # A minimum-size Ethernet frame is 64 bytes = 16 beats, so both stages
        # settle long before sink.last for any legal frame.
        dst_mac = Signal(48)
        allow   = Signal()

        self.sync += [
            If(beat_accept & (beat == 2), dst_mac.eq(dst_mac_c)),
        ]

        local_mac = Signal(48)
        self.comb += local_mac.eq(Cat(self.local_mac_lo.storage,
                                      self.local_mac_hi.storage))

        allow_terms = [
            dst_mac == MAC_BROADCAST,
            dst_mac == local_mac,
        ]
        # Masked rules: a prefix match is a comparator on the masked bits only,
        # so 01:00:5e:00:00:xx costs 40 bits of compare rather than three
        # separate 48-bit ones.
        allow_terms += [(dst_mac & mask) == value for mask, value in allow_rules]
        for hi, lo in zip(spare_hi, spare_lo):
            spare = Signal(48)
            self.comb += spare.eq(Cat(lo.storage, hi.storage))
            # A cleared slot (0) must not accidentally match anything; the
            # all-zero MAC is not a legal destination, so comparing directly is
            # safe and saves an enable bit per slot.
            allow_terms.append((spare != 0) & (dst_mac == spare))

        allow_c = Signal()
        self.comb += allow_c.eq(reduce(lambda a, b: a | b, allow_terms))

        self.sync += If(beat_accept & (beat == 3), allow.eq(allow_c))

        # `allow` is valid from beat 4 onwards. Before that the registers still
        # hold the PREVIOUS frame's verdict, so a runt (a frame ending in fewer
        # than 4 beats -- only reachable via a truncated/errored frame, since the
        # PHY enforces a 64-byte minimum) must never be discarded. Fail open.
        decided = Signal()
        self.comb += decided.eq(beat >= 4)

        # HARDWARE INTERLOCK: the filter is inert until the local unicast MAC has
        # been programmed. Firmware is supposed to do that at init, before
        # anything can arm the gate -- but "supposed to" is exactly the kind of
        # convention that fails on a reordered init, a half-flashed image, or a
        # future refactor, and the failure mode is losing every unicast frame
        # including the one that would turn the filter off. Making it a property
        # of the gateware costs one comparator on a static CSR value.
        #
        # Registered because it is static: this keeps the 48-bit reduce out of
        # the combinational path into the writer's end-of-frame decision.
        armed = Signal()
        self.sync += armed.eq(self.enable.storage & (local_mac != 0))

        self.comb += self.discard.eq(armed & decided & ~allow)

        # ---- End-of-frame accounting ----
        eof = Signal()
        self.comb += eof.eq(beat_accept & self.sink.last)

        self.sync += If(eof & decided,
            If(allow,
                self.match_count.status.eq(self.match_count.status + 1),
            ).Else(
                self.nomatch_count.status.eq(self.nomatch_count.status + 1),
                self.last_drop_hi.status.eq(dst_mac[32:48]),
                self.last_drop_lo.status.eq(dst_mac[0:32]),
                # Counts `armed`, not `enable`: this must report what the
                # gateware DID, so that a discard_count stuck at 0 after arming
                # is a signal (the interlock is holding it off) rather than
                # something to be explained away.
                If(armed,
                    self.discard_count.status.eq(self.discard_count.status + 1),
                ),
            ),
        )
