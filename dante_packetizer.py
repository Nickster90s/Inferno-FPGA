#!/usr/bin/env python3
#
# Gateware AAF TX packetizer — moves the USB→AVB audio path out of firmware.
# Copyright 2025-2026 Nick (nick.eventslight@gmail.com)
# SPDX-License-Identifier: Apache-2.0
#
# This is the TX-side counterpart to avtp_extractor.py / crf_extractor.py:
# it keeps the CPU out of the per-sample audio path entirely. The CPU only
# writes the stream binding (dst_mac, stream_id, vlan, src_mac) once at ACMP
# CONNECT time and flips `enable`; it never touches a sample again.
#
# Data path (all sys domain):
#
#   usb_avb_subsystem sample handshake          (USB host clock, ~48 kHz)
#        │  (lo/hi/readable/pop — same signals firmware's usb_aaf_drain reads)
#        ▼
#   8-channel frame assembler  ──►  block_fifo (256-bit, elastic rate buffer)
#        │
#        ▼  read paced by mcr.sample_strobe   ◄── THE MEDIA CLOCK
#   pay[fill][blk]  (ping-pong, 6 blocks/packet)
#        │  every 6 strobes → send_req
#        ▼
#   builder FSM ──► frame_ram (59×32) ──► stream.Endpoint(32) ──► TX mux ──► MAC
#        ▲
#        └── presentation_time = (sec·1e9 + ns + offset) mod 2^32   (from TSU)
#
# MEDIA CLOCK / CRF NOTE
# ----------------------
# Egress is paced by `mcr.sample_strobe`, the same NCO strobe MCRI2STx uses.
# Firmware's PI servo (mcr.c) tunes the NCO increment from CRF timestamps
# whenever the selected clock source is CRF (cs=1) and CRF is locked — so in
# that regime the strobe rate, and therefore the AAF stream rate AND the
# presentation-time cadence, are the recovered CRF media clock by construction.
# When CRF is not locked / cs=0, the NCO free-runs at local 48 kHz. No extra
# logic is needed in the packetizer: "rate from CRF when locked+cs=1" falls
# out of pacing on the NCO, exactly like the I2S DAC.

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.fifo import SyncFIFO
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus

from liteeth.common import eth_phy_description


# 1_000_000_000 as set-bit positions, for a DSP-free constant multiply.
# (Same lesson as the LiteEth TSU addend*1e9 shift-add workaround — a `*`
#  here infers an unroutable DSP cascade on nextpnr-xilinx.)
_NS_PER_SEC_BITS = [29, 28, 27, 25, 24, 23, 20, 19, 17, 15, 14, 11, 9]


class TXFrameArbiter(LiteXModule):
    """Frame-atomic N:1 stream arbiter for the LiteEth MAC core sink.

    Priority = list order (index 0 highest — give that to the firmware SRAM
    reader so gPTP / AVDECC / MSRP are never delayed; the AAF talker waits a
    few µs at most, dwarfed by the 2 ms presentation offset). A frame in
    flight is never interrupted: once a source is granted, the grant holds
    until that source asserts valid & ready & last.
    """
    def __init__(self, sources, dw=32):
        self.source = source = stream.Endpoint(eth_phy_description(dw))
        n = len(sources)

        sel  = Signal(max=max(2, n))
        busy = Signal()

        # High while source index 0 (the firmware/control-plane path) holds the
        # grant — used to gate the gPTP TX-timestamp latch.
        self.firmware_granted = Signal()
        self.comb += self.firmware_granted.eq(busy & (sel == 0))

        # Granted source drives the output; backpressure routes to it alone.
        for i, s in enumerate(sources):
            self.comb += If(busy & (sel == i),
                source.valid.eq(s.valid),
                source.data.eq(s.data),
                source.last.eq(s.last),
                source.last_be.eq(s.last_be),
                source.error.eq(s.error),
                s.ready.eq(source.ready),
            )

        # Priority encoder: iterate high→low so index 0 wins when valid.
        nextsel = Signal(max=max(2, n))
        anyv    = Signal()
        self.comb += anyv.eq(reduce(or_, [s.valid for s in sources]))
        for i in reversed(range(n)):
            self.comb += If(sources[i].valid, nextsel.eq(i))

        self.sync += [
            If(~busy,
                If(anyv,
                    sel.eq(nextsel),
                    busy.eq(1),
                ),
            ).Elif(source.valid & source.ready & source.last,
                busy.eq(0),
            ),
        ]


def _mul_1e9_lo32(x):
    """Low 32 bits of x * 1_000_000_000, as a shift-add tree (no multiplier).

    Only the low 32 bits of `x` matter mod 2^32, but we let the slice handle
    that — yosys prunes the unused high bits."""
    return reduce(lambda a, b: a + b, [(x << s) for s in _NS_PER_SEC_BITS])


class DantePacketizer(LiteXModule):
    """Build + transmit AVTP-AAF (32-bit INT) frames from the USB sample
    stream, paced by the MCR NCO. Produces a LiteEth phy-description source
    to be muxed onto the MAC core sink.

    Parameters
    ----------
    mcr  : MCRNco          — provides `sample_strobe` (the media clock tick).
    tsu  : LiteEthTSU      — provides live `seconds` / `nanoseconds`.
    usb_* : the sample handshake exported by usb_avb_subsystem (sys domain):
        usb_sample_lo[8:32] = 24-bit audio, MSB-aligned in a 32-bit sample
        usb_sample_hi[0:3]  = channel index, usb_sample_hi[3] = first marker
        usb_readable        = FIFO head valid
        self.usb_pop        = OUTPUT: one-cycle pop strobe (mux into the
                              wrapper's sample_pop ONLY when self.enable.storage)
    channels           : audio channels per AAF frame (Milan default 8).
    samples_per_packet : AAF blocks per packet (Milan AAF @48k = 6).
    """
    def __init__(self, mcr, tsu, *, usb_sample_lo, usb_sample_hi,
                 usb_readable, channels=8, samples_per_packet=16,
                 fifo_depth=64, streams=6):
        dw = 32
        ch_bits  = max(1, log2_int(channels, need_pow2=False))
        blk_bits = max(1, log2_int(samples_per_packet, need_pow2=False))

        # TIME-MUX geometry: `streams` AAF frames of `channels` each share one
        # media clock and one USB ingress. The host delivers BLOCK_CH interleaved
        # channels per media frame (ch0..ch47 for 6x8); the ring stores them in
        # arrival order and the reader strides out each stream's 8-ch slice.
        # Frame s sample (row r, slice-ch c) is at ring offset r*BLOCK_CH+s*ch+c;
        # the reader advances +1 within a slice and +ROW_JUMP at the row boundary
        # (proven in sims/sim_stride_timemux.py).
        self.streams = streams
        BLOCK_CH      = streams * channels                 # 48 for 6x8
        ROW_JUMP      = BLOCK_CH - channels + 1             # 41: ch7 -> next row's slice
        FRAME_SAMPLES = channels * samples_per_packet       # 48 samples in one AAF frame
        BLOCK_SAMPLES = BLOCK_CH * samples_per_packet        # 288 ring samples per block
        # samp_hi channel-index field width; `first` marker sits just above it.
        first_bit     = max(1, (BLOCK_CH - 1).bit_length()) # 6 for 48ch, 3 for 8ch
        st_bits       = max(1, log2_int(streams, need_pow2=False))

        # ---- Frame geometry (computed once, Python-side) ----
        # Dante wire format, confirmed against captures/dante_audio_multicast.pcap:
        #   14 eth + 20 ip + 8 udp + 9 dante = 51 header bytes
        #   payload = fpp * channels * 3, big-endian MSB-justified 24-bit
        #   16 * 8 * 3 = 384  ->  435 total, which is what real devices emit.
        BYTES_PER_SAMPLE = 3                           # pcm=3 / enc=24 on the wire
        HDR_LEN  = 51
        PAY_LEN  = samples_per_packet * channels * BYTES_PER_SAMPLE
        TOTAL    = HDR_LEN + PAY_LEN                   # 435
        rem      = TOTAL % 4
        N_WORDS  = (TOTAL + 3) // 4                    # 59
        LAST_IDX = N_WORDS - 1
        # last_be is a ONE-HOT of the LAST VALID BYTE (LiteEth mac/sram.py:238-245:
        # 1 byte->0b0001, 2->0b0010, 3->0b0100, 4->0b1000), NOT a byte mask.
        # We had ((1<<rem)-1) = 0x3 for the 234-byte frame (rem=2) where LiteEth's
        # TX last_be stage + 32->8 converter expect 0x2 -> wrong final byte count
        # -> WRONG FRAME LENGTH -> BAD FCS -> every AAF frame dropped on the wire
        # as an RX error (firmware frames were fine because the SRAM reader sets
        # this correctly). THE bug that kept AAF off the wire end-to-end.
        LAST_BE  = (1 << 3) if rem == 0 else (1 << (rem - 1))

        # IP/UDP scalar lengths. The IP header checksum is NOT computed here --
        # firmware writes it once per flow (ip_csum CSR), because the header is
        # constant for the life of a multicast flow and a gateware ones-complement
        # adder would cost a DSP48 and buy nothing.
        IP_TOTAL_LEN  = 20 + 8 + 9 + PAY_LEN           # IP header onwards
        UDP_TOTAL_LEN = 8 + 9 + PAY_LEN

        # ---- Output stream (to be muxed onto mac.core.sink) ----
        self.source = source = stream.Endpoint(eth_phy_description(dw))

        # ---- Pop strobe back to the USB wrapper (gated by enable upstream) ----
        self.usb_pop = Signal()

        # ---- CSRs: binding (firmware writes once at CONNECT) ----
        self.enable        = CSRStorage(1,  description="1 = gateware sources the AAF stream (CPU out of the audio path).")
        self.src_mac_hi    = CSRStorage(16, description="Source MAC [47:32] (FPGA MAC).")
        self.src_mac_lo    = CSRStorage(32, description="Source MAC [31:0].")
        self.dst_mac_hi    = CSRStorage(16, description="Dest MAC [47:32] (SRP/ACMP learned multicast).")
        self.dst_mac_lo    = CSRStorage(32, description="Dest MAC [31:0].")
        # TIME-MUX: dst_mac + stream_id are PER-STREAM (indirect-addressed to keep
        # the CSR count low — see [[csr-mux-explodes-sys-clk]]). Firmware writes
        # ctx_select=s, then dst_mac_hi/lo + stream_id_hi/lo; the gateware latches
        # each {hi,lo} into context[s] on the _lo write strobe. streams=1 reduces
        # to the original single binding (ctx_select always 0).
        self.ctx_select    = CSRStorage(max(1, st_bits),
                             description="Per-stream binding context index (0..streams-1) for the writes below.")
        # Per-flow (indirect via ctx_select) IP/UDP binding. dst_mac is derived by
        # firmware from the group address (01:00:5e:xx:xx:xx) and written like the
        # AVTP dst_mac was; dst_ip/ip_csum/udp ports are new.
        self.dst_ip        = CSRStorage(32, description="Destination IPv4 for context ctx_select (239.255.x.y).")
        self.ip_csum       = CSRStorage(16, description="IPv4 header checksum for ctx_select, computed by FIRMWARE (header is constant per flow).")
        self.udp_sport     = CSRStorage(16, reset=4321, description="UDP source port for ctx_select.")
        self.udp_dport     = CSRStorage(16, reset=4321, description="UDP dest port for ctx_select (write LAST -> latches the context).")
        self.src_ip        = CSRStorage(32, description="Our IPv4 source address (shared by all flows).")
        self.ip_tos        = CSRStorage(8, reset=0xB8, description="IP TOS: DSCP EF for audio, as real Dante devices use.")
        self.ip_ttl        = CSRStorage(8, reset=1, description="IP TTL.")
        self._unused_tci   = CSRStorage(16, reset=(3 << 13) | 2,
                             description="802.1Q TCI = (pcp<<13)|vid. Class A default pcp=3, vid=2.")
        self.ts_offset     = CSRStorage(32, reset=0,
                             description="presentation_time offset (ns) added to gPTP now. Milan AAF = 2 ms.")

        # ANCHOR THE MEDIA CLOCK TO PTP.
        #
        # ts_sec/ts_sub below free-run from reset, so the header carried seconds
        # since BOOT while every other device on the network carries seconds
        # since the grandmaster's epoch. Measured on the wire:
        #
        #   A16R    239.255.121.190   sec=84625
        #   ours    239.255.0.66      sec=426      <- uptime, not PTP time
        #
        # Nothing about the stream looked wrong -- tag 0x02, step exactly 16,
        # correct group -- so a receiver subscribed happily and then discarded
        # every packet as ~23 hours stale. A green patch with no audio.
        #
        # The TSU already holds correct absolute time (ptpv1.c steps it via
        # tsu_step_seconds_write), so this only needs a load path. Firmware
        # writes the target and pulses ts_load; the value is adopted at the next
        # sample strobe, keeping the load in the media-clock domain.
        self.ts_load_sec   = CSRStorage(32, reset=0,
                             description="Seconds to load into the media-clock timestamp counter.")
        self.ts_load_sub   = CSRStorage(32, reset=0,
                             description="Subsecond samples (0..47999) to load alongside ts_load_sec.")
        # PER-FLOW SHAPE. A unicast flow is not a bundle: the receiver names the
        # channels it wants, how many slots, and its own fpp. Measured from two
        # real receivers asking us for the same two channels:
        #   A16R  4 slots [1,2,0,0]  fpp 8
        #   AM2   2 slots [1,2]      fpp 16
        # So slot count, fpp and the slot->channel map all vary per flow, and
        # all three change the packet length. Multicast is unchanged by this --
        # a bundle is just a flow whose destination is a group address and whose
        # map is eight consecutive channels.
        self.chmap_lo      = CSRStorage(32, description="Slots 0..3: per byte, [5:0] tx channel (0-based), [7] valid. 0 = silence.")
        self.chmap_hi      = CSRStorage(32, description="Slots 4..7, same encoding.")
        self.flow_cfg      = CSRStorage(8,  description="[3:0] slot count (0 = flow disabled), [4] fpp: 0 = 8, 1 = 16. Write LAST -> latches the map.")
        self.ts_load       = CSRStorage(1,
                             description="Write 1 to adopt ts_load_sec/ts_load_sub at the next sample strobe.")
        # REMOVED (2026-06-23): pres_base CSR. It fed the (now-deleted) pres-ramp
        # dilation, so it was dead; worse, on openXC7 the value mcr.c wrote into it
        # (gbase ~= the NCO increment) was ending up in the pres in place of
        # pres_offset -> cold-start avtp_ts = gPTP + ~increment (4123363) not +2ms.
        # The two adjacent 32-bit CSRs (pres_offset @0x24, pres_base @0x28) aliased.
        # Deleting pres_base removes the increment from the design entirely.

        # ---- CSRs: status (read-only diagnostics) ----
        self.packet_count   = CSRStatus(32, description="AAF frames transmitted.")
        self.underrun_count = CSRStatus(32, description="Media-clock ticks where block_fifo was empty (silence inserted).")
        self.overrun_count  = CSRStatus(32, description="send_req arriving while builder busy (packet skipped — should stay 0).")
        self.fifo_level     = CSRStatus(blk_bits + 8, description="block_fifo occupancy (blocks).")
        # Timestamp instrument: the ACTUAL emitted avtp_timestamp (pres) and the
        # live gPTP-ns(low32) latched at the SAME packet emit. effective offset =
        # dbg_last_pres - dbg_emit_gptp should hold ~= pres_offset (2ms); if it
        # drifts/goes negative the pres ramp is wrong (chasing AxC Late-Timestamp).
        self.dbg_last_ts    = CSRStatus(32, description="Last emitted Dante subsec_samples (sample index within the second).")
        self.dbg_last_sec   = CSRStatus(32, description="Last emitted Dante seconds.")
        _dbg_ts_r  = Signal(32)
        _dbg_sec_r = Signal(32)
        self.comb += [self.dbg_last_ts.status.eq(_dbg_ts_r),
                      self.dbg_last_sec.status.eq(_dbg_sec_r)]
        # NOTE: the soft-ILA debug CSRs (dbg_block_push/pop/first, level min/max,
        # raw_strobe, usb_samp, pres, frame_addr/data) were REMOVED 2026-06-12.
        # They had done their diagnostic job, and the large AAF CSR bank was the
        # sys_clk critical path: the CPU->CSR-decode routing (csrbank10) capped sys
        # at 43.97 MHz, and sub-50 setup violations corrupted the AAF builder's
        # high-bit channel paths (ch1/3/5/6 = wrong MSBs, low bytes clean). Trimming
        # the bank shrinks that decode mux. See [[csr-mux-explodes-sys-clk]].

        # MAC error lane is always 0 for our generated frames.
        self.comb += source.error.eq(0)

        # =========================================================
        # 1) USB ingress -> 8-channel frame assembler -> SRC ring
        # =========================================================
        # Async sample-rate converter (FIX (b), 2026-06-01). The USB host writes
        # whole 8ch frames into a BRAM ring at its own crystal rate; the gPTP NCO
        # strobe pulls ONE interpolated frame per tick (= the AVB media clock). A
        # Q1.31 phase accumulator (`src_step` = f_in/f_out) + per-channel linear
        # interpolation resamples host-rate -> gPTP-rate. `src_step` is servo'd IN
        # FIRMWARE from the ring level (level -> step) — the piece the earlier WIP
        # (6797563) lacked: at fixed step=1.0 the gPTP consumer out-paced the host
        # crystal and drained the ring. The block_fifo approach it replaces could
        # never centre because USB feedback is decoupled from block_fifo level by
        # the wrapper's free-running producer (root cause, 2026-06-01). Here the
        # OUTPUT rate is gPTP and does NOT feed back to the host, so no runaway;
        # USB feedback is pinned nominal (0x60000).
        # SAMPLE ring (32-bit entries) in BRAM — holds the raw interleaved stream
        # ch0,ch1,...,ch7,ch0,... read ONE SAMPLE AT A TIME by the byte builder.
        # No 256-bit frame is ever assembled in LUTs/FFs, so none of the wide
        # variable muxes / wide shift registers that openXC7 mis-synthesised
        # (pink noise, ch1/3/5/6, ch2/4) exist anymore — only BRAM is wide. depth
        # = 8x the old frame depth (same buffering time). Read pipeline verified in
        # /tmp/sim_sring.py.
        # SIX de-interleaved 8-ch rings (one per AAF stream), NOT one 48-ch ring with
        # a strided read. The 48ch USB frame is DEMUXED by channel_nr -> bundle=nr>>
        # ch_bits, within=nr&(channels-1); each ring is then written + read EXACTLY
        # like the proven 1x8ch path (contiguous, pow2 depth, no stride). This
        # replicates the working 8ch datapath x6 instead of the fragile shared-ring
        # strided read (which produced garbage + dropped streams on HW). Sim:
        # sims/sim_six_ring_demux.py. depth = per-ring (8ch) buffering, next pow2.
        _need_depth = fifo_depth * channels        # per-ring samples of buffering
        SRING_DEPTH = 1 << max(1, (_need_depth - 1).bit_length())
        log2depth   = log2_int(SRING_DEPTH)
        # 36-bit entries: the 32 sample bits are spread to AVOID the RAMB36 parity
        # positions (8,17,26,35), which nextpnr-xilinx drops when packing a 32-bit
        # BRAM. Dummies land on parity; real data survives. See pack/unpack below.
        mems = [Memory(36, SRING_DEPTH) for _ in range(streams)]
        wps  = []
        rps  = []
        for _m in mems:
            self.specials += _m
            _wp = _m.get_port(write_capable=True)
            _rp = _m.get_port()
            self.specials += _wp, _rp
            wps.append(_wp)
            rps.append(_rp)

        # Retained for CSR-layout / firmware compat — UNUSED now. The host is
        # rate-slaved by USB async feedback (tracks our NCO rate), so this is a
        # bit-exact passthrough: no resampler, no per-channel multiplier.
        self.fifo_depth = fifo_depth

        wr     = Signal(32)                         # sample write pointer
        rd     = Signal(32)                         # sample read/consumed pointer
        # SIGNED sample occupancy (negative if consumer overtakes producer).
        level  = Signal((34, True))
        self.comb += level.eq(wr - rd)
        # block_level for the wrapper PI servo — SAME scale as the old frame level
        # (samples >> 3 = frames), so the servo CENTER/KP/clamps carry over with no
        # change to the wrapper or the .v.
        # Scale the ring fill to 0..128 (servo CENTER=64) regardless of BLOCK_CH:
        # SRING_DEPTH >> _bl_sh == 128, so a half-full ring reads ~64. (8ch: shift
        # 5 == the old >>5; 48ch ring 32768 -> shift 8.)
        _bl_sh = log2depth - 7
        # Full-ring scale 0..128 (= SRING_DEPTH>>_bl_sh). This was CAPPED at
        # fifo_depth (64) = HALF the ring, which (a) hid the upper half from the
        # diagnostics and (b) made err=CENTRE-level ALWAYS >=0 in the .v feedback
        # loop -> the servo could only push the host FASTER, never slower -> NO
        # authority to pull a full ring back down (and no symmetric center for the
        # new integral). CENTRE=64 is now true mid; un-cap to the full 0..128.
        _bl_max = SRING_DEPTH >> _bl_sh                # 128 = full ring (any depth)
        self.block_level = Signal(max=_bl_max + 1)
        self.comb += If(level < 0,
            self.block_level.eq(0),
        ).Elif((level >> _bl_sh) > _bl_max,
            self.block_level.eq(_bl_max),
        ).Else(
            self.block_level.eq(level >> _bl_sh),   # ring samples -> 0..128, CENTER=mid
        )
        level_u = Signal(max=_bl_max + 1)
        self.comb += level_u.eq(self.block_level)   # fifo_level CSR = frame-equiv
        # rp.adr is driven by the byte builder's fetch pointer (rdf), below.

        # CHANNEL-ADDRESSED write (ADAT BundleDemultiplexer style): place every USB
        # sample at frame_base + channel_nr, NOT sequentially. The decoder labels
        # each sample with its explicit channel (0..BLOCK_CH-1) plus a `first`(ch0)
        # marker; frame_base advances ONE media-frame (BLOCK_CH) per `first`. The old
        # SEQUENTIAL write (wr+=1) inferred channel from POSITION, so a single dropped
        # sample (ring full) shifted the write phase and ROTATED ALL channels
        # permanently — the 48ch "corrupt after a while". 8ch survived only because
        # that rotation was a constant offset fixed once by chan_rot. With explicit
        # addressing a drop leaves ONE stale channel slot (a tiny per-channel glitch)
        # and the frame grid self-aligns every `first`, so drift heals within one
        # media-frame. Register the bridge output first (stable, timing-clean).
        do_pop = usb_readable
        samp_lo_r = Signal(32)
        samp_hi_r = Signal(first_bit + 1)   # [0:first_bit]=channel(0..BLOCK_CH-1), [first_bit]=first
        samp_vld  = Signal()
        self.sync += [
            samp_vld.eq(do_pop),
            If(do_pop,
                samp_lo_r.eq(usb_sample_lo),
                samp_hi_r.eq(usb_sample_hi[0:first_bit + 1]),
            ),
        ]
        first   = samp_hi_r[first_bit]              # 48ch-frame ch0 marker (channel_nr==0)
        chan_nr = samp_hi_r[0:first_bit]            # explicit channel 0..BLOCK_CH-1
        ch_bits = max(1, log2_int(channels))        # 3 for 8ch
        bundle  = chan_nr[ch_bits:]                 # which ring   (channel_nr >> 3)
        within  = chan_nr[0:ch_bits]                # ch in bundle (channel_nr & 7)
        samp32  = samp_lo_r                         # true 32-bit, no truncation
        en = self.enable.storage
        started   = Signal()                        # high once the first 48ch ch0 seen
        # frame_base = per-ring base of the current 8-sample frame; advances by
        # `channels` per `first` (one 48ch USB frame fills one 8-sample frame in EVERY
        # ring). within addresses the channel inside that frame -> a dropped sample is
        # one stale slot in one ring, never a rotation.
        frame_base = Signal(32)
        new_base   = Signal(32)
        self.comb += new_base.eq(frame_base + channels)
        base_now   = Signal(32)
        self.comb += base_now.eq(Mux(samp_vld & first, new_base, frame_base))
        wr_addr    = Signal(32)
        self.comb += wr_addr.eq(base_now + within)
        rd_anchor  = Signal(32)
        have_space = Signal()
        self.comb += have_space.eq(level < (SRING_DEPTH - 2))
        do_write = Signal()
        self.comb += [
            self.usb_pop.eq(do_pop),                # always drain the bridge FIFO
            do_write.eq(en & samp_vld & (started | first) & have_space),
        ]
        packed = Signal(36)                          # data spread around parity (8,17,26,35)
        self.comb += packed.eq(Cat(samp32[0:8],  Constant(0, 1),
                                   samp32[8:16], Constant(0, 1),
                                   samp32[16:24],Constant(0, 1),
                                   samp32[24:32],Constant(0, 1)))
        # DEMUX: drive all rings; only ring[bundle] takes the write this cycle.
        for _s in range(streams):
            self.comb += [
                wps[_s].adr.eq(wr_addr[0:log2depth]),
                wps[_s].dat_w.eq(packed),
                wps[_s].we.eq(do_write & (bundle == _s)),
            ]
        # wr (for level/servo) = frame_base: all rings filled up to here. Advance one
        # 8-sample frame per `first` REGARDLESS of have_space so the grid self-aligns.
        self.comb += wr.eq(frame_base)
        self.sync += [
            If(~en, started.eq(0)).Elif(samp_vld & first, started.eq(1)),
            If(samp_vld & first & (started | first), frame_base.eq(new_base)),
            If(en & ~started & samp_vld & first, rd_anchor.eq(new_base)),
        ]

        # =========================================================
        # 2) Media-clock-paced SRC read -> pay ping-pong buffer
        # =========================================================
        # 2) Media-clock PACING: one packet every samples_per_packet (=6) media
        #    strobes. The byte builder (section 4) drains 48 samples from the ring
        #    per packet and advances `rd` there — no per-strobe frame read, no wide
        #    packet accumulator. Nothing here is wider than a counter.
        send_req = Signal()
        # Prime to half-full (in SAMPLES) before consuming real audio; until then
        # the builder emits silence so the listener (AxC) can still lock.
        primed  = Signal()
        _center = SRING_DEPTH // 2                    # half the ring, in samples
        # HYSTERESIS: prime at half-full, but UN-PRIME on underrun (level < one
        # packet of samples). Without the un-prime, when the ring drains (USB
        # stopped / host not feeding), the reader kept advancing and WRAPPED through
        # stale ring data forever -> continuous noise on the live channels even with
        # NO USB input (the smoking-gun symptom). Un-priming makes the builder emit
        # silence (samp_hold=0, rd holds) until the ring refills past center.
        self.sync += [
            If(~en, primed.eq(0)
            ).Elif(level >= _center, primed.eq(1)
            ).Elif(level < FRAME_SAMPLES, primed.eq(0)),
        ]
        strobe = Signal()
        self.comb += strobe.eq(mcr.sample_strobe & en)

        # ---- Dante timestamp: two counters, no arithmetic ----------------------
        #
        # Dante's header is a SAMPLE INDEX, not a presentation time:
        #   timestamp = seconds * 48000 + subsec_samples
        # so all the AVTP presentation-time machinery (gPTP re-anchor, CRF
        # dilation, the *1e9 shift-add tree, pres_offset) is gone. Two counters
        # driven by the media clock give the header directly -- no multiply, no
        # divide, nothing wide.
        #
        # Because 48000 is divisible by fpp (16), pacing off the low bits of
        # subsec_samples makes EVERY emitted timestamp a multiple of fpp by
        # construction, which is what flows_tx.rs bootstraps by hand. That also
        # replaces the free-running blk_idx the AAF packetizer used for pacing.
        ts_sec  = Signal(32)
        ts_sub  = Signal(32)          # 0 .. 47999
        SUBSEC_MAX = 48000
        # A pending load is adopted at the next strobe, so the counter only ever
        # changes in the media-clock domain. Emitted timestamps stay multiples
        # of fpp regardless of what is loaded: the pacing condition below fires
        # when ts_sub[0:4] == 15 and subtracts 15, so the result is a multiple
        # of 16 by construction, not by luck with the loaded value.
        ts_load_pending = Signal()
        self.sync += [
            If(self.ts_load.re, ts_load_pending.eq(1)),
            If(strobe,
                If(ts_load_pending,
                    ts_sec.eq(self.ts_load_sec.storage),
                    ts_sub.eq(self.ts_load_sub.storage),
                    ts_load_pending.eq(0),
                ).Elif(ts_sub == (SUBSEC_MAX - 1),
                    ts_sub.eq(0),
                    ts_sec.eq(ts_sec + 1),
                ).Else(
                    ts_sub.eq(ts_sub + 1),
                ),
            ),
        ]
        # Emitted timestamp: the counter value at send_req, minus one packet
        # (the samples in THIS packet were captured over the preceding fpp
        # strobes), plus a firmware-tunable offset in samples.
        #
        # flows_tx.rs:44 is worth heeding here -- "it's better to have the clock
        # in the past than in the future - otherwise Dante devices receiving
        # from us go mad and fart" -- which is what ts_offset is for.
        ts_sec_emit = Signal(32)
        ts_sub_emit = Signal(32)

        underruns = Signal(32)
        self.comb += [self.underrun_count.status.eq(underruns),
                      self.fifo_level.status.eq(level_u)]
        self.sync += [
            send_req.eq(0),
            # underrun = media tick spent emitting silence (ring below the prime
            # floor). This counter was DEAD — declared + read into the CSR but
            # NEVER incremented — so every silence glitch was invisible
            # (underrun_count stuck at 0). Counts ticks, so it is severity-weighted;
            # a nonzero baseline at startup (initial prime) is expected, watch the
            # DELTA after audio is flowing.
            If(strobe & ~primed, underruns.eq(underruns + 1)),
            # PACING off the timestamp counter, not a free-running blk_idx.
            # 48000 % 16 == 0, so firing when the low log2(fpp) bits wrap makes
            # every emitted timestamp an exact multiple of fpp -- the property
            # flows_tx.rs bootstraps by hand -- with no arithmetic at all.
            # PACE AT THE FINEST fpp (8), not at a single global fpp. A flow with
            # fpp=8 is due every tick; one with fpp=16 every other tick. Both stay
            # aligned by construction: a flow is only due when the low log2(fpp)
            # bits of the sample counter are all ones, so its emitted timestamp
            # (counter - (fpp-1)) is a multiple of its own fpp and can never go
            # negative -- which is what makes the subtraction below safe without a
            # borrow into seconds.
            If(strobe,
                If(ts_sub[0:3] == 7,
                    send_req.eq(1),
                    # Latch the timestamp for THIS packet: the counter is about
                    # to land on the first sample of the NEXT packet, so the
                    # packet we are about to build starts fpp samples earlier.
                    # Latch the RAW counter; each flow subtracts its own fpp-1
                    # at build time, because flows of different fpp cover
                    # different windows ending at this same instant.
                    ts_sec_emit.eq(ts_sec),
                    ts_sub_emit.eq(ts_sub + self.ts_offset.storage),
                ),
            ),
        ]

        # 3) Header byte vector (LSB index = first byte on the wire)
        # =========================================================
        # Per-stream binding contexts (indirect-written above). dst_mac + stream_id
        # are selected by `stream_idx` (the frame being built); src_mac/tci/pres are
        # shared. Array indexing -> a clean Case mux (NOT a barrel shift), safe on
        # openXC7. seq is per-stream.
        dip_arr  = Array([Signal(32) for _ in range(streams)])
        csum_arr = Array([Signal(16) for _ in range(streams)])
        sport_arr= Array([Signal(16) for _ in range(streams)])
        dport_arr= Array([Signal(16) for _ in range(streams)])
        dmac_arr = Array([Signal(48) for _ in range(streams)])
        cmlo_arr = Array([Signal(32) for _ in range(streams)])
        cmhi_arr = Array([Signal(32) for _ in range(streams)])
        cfg_arr  = Array([Signal(8)  for _ in range(streams)])
        ctx_sel  = self.ctx_select.storage
        # udp_dport is the LATCH trigger: firmware writes dst_ip, ip_csum and
        # udp_sport first, then udp_dport last, exactly as it wrote stream_id_lo
        # last under AVB.
        self.sync += [
            If(self.udp_dport.re, [
                dip_arr[ctx_sel].eq(self.dst_ip.storage),
                csum_arr[ctx_sel].eq(self.ip_csum.storage),
                sport_arr[ctx_sel].eq(self.udp_sport.storage),
                dport_arr[ctx_sel].eq(self.udp_dport.storage),
            ]),
            If(self.dst_mac_lo.re,
               dmac_arr[ctx_sel].eq(Cat(self.dst_mac_lo.storage, self.dst_mac_hi.storage))),
            # flow_cfg last: it latches the channel map with it, so a half-written
            # map is never visible to the builder.
            If(self.flow_cfg.re, [
                cmlo_arr[ctx_sel].eq(self.chmap_lo.storage),
                cmhi_arr[ctx_sel].eq(self.chmap_hi.storage),
                cfg_arr[ctx_sel].eq(self.flow_cfg.storage),
            ]),
        ]

        stream_idx = Signal(max=streams) if streams > 1 else Signal()

        # ---- Per-flow packet geometry, derived combinationally ----------------
        # pay_len = nslots * fpp * 3. fpp is 8 or 16, so this is (nslots*3) shifted
        # by 3 or 4 -- an add and a mux, no multiplier and no DSP48.
        #
        # pay_len is ALWAYS a multiple of 8, so TOTAL = 51 + pay_len always has
        # TOTAL % 4 == 3. LAST_BE therefore stays the constant 0b0100 for every
        # flow shape, and N_WORDS = 13 + pay_len/4 needs no divider. That is luck
        # worth stating: had it not held, both would have been per-flow logic.
        cfg_now   = cfg_arr[stream_idx]
        nslots    = Signal(4)
        fpp16     = Signal()
        self.comb += [nslots.eq(cfg_now[0:4]), fpp16.eq(cfg_now[4])]
        cur_fpp   = Signal(8)
        self.comb += cur_fpp.eq(Mux(fpp16, 16, 8))
        # This flow's window START. ts_sub_emit holds the counter at the tick.
        f_ts_sub  = Signal(32)
        self.comb += f_ts_sub.eq(ts_sub_emit - (cur_fpp - 1))
        ns3       = Signal(6)
        self.comb += ns3.eq(nslots + (nslots << 1))            # nslots * 3
        pay_len   = Signal(10)
        self.comb += pay_len.eq(Mux(fpp16, ns3 << 4, ns3 << 3))
        f_total   = Signal(11)
        self.comb += f_total.eq(HDR_LEN + pay_len)
        f_last_idx = Signal(max=128)
        self.comb += f_last_idx.eq(12 + pay_len[2:])           # (52 + pay_len)/4 - 1
        f_ip_len  = Signal(16)
        f_udp_len = Signal(16)
        self.comb += [f_ip_len.eq(37 + pay_len), f_udp_len.eq(17 + pay_len)]

        src_mac = Cat(self.src_mac_lo.storage, self.src_mac_hi.storage)   # [0:48], byte0 = [40:48]
        dst_mac = dmac_arr[stream_idx]                                    # selected context
        dst_ip  = dip_arr[stream_idx]
        ipsum   = csum_arr[stream_idx]
        sport   = sport_arr[stream_idx]
        dport   = dport_arr[stream_idx]
        src_ip  = self.src_ip.storage

        # NOTE: no sequence number and no presentation time. Dante's header has
        # neither -- the sample-index timestamp IS the ordering, which is why the
        # AVTP seq_arr and the pres ramp are both gone.

        def mac_byte(sig, i):   # i=0 is the wire-first (MSB) byte of a 48-bit MAC
            hi = 48 - i * 8
            return sig[hi - 8:hi]

        def ip_byte(sig, i):    # i=0 is the wire-first (MSB) byte of a 32-bit IPv4
            hi = 32 - i * 8
            return sig[hi - 8:hi]

        def be16(sig, i):       # i=0 = high byte
            return sig[8:16] if i == 0 else sig[0:8]

        # Ethernet + IPv4 + UDP + Dante, 51 bytes. Field values confirmed against
        # captures/dante_audio_multicast.pcap from real hardware:
        #   IP  ID = 0, DF/frag = 0, TTL 1, TOS 0xB8 (DSCP EF), proto 17
        #   UDP checksum = 0x0000 (optional over IPv4 and genuinely omitted)
        #   Dante [0] = 0x02, then seconds:u32 and subsec_samples:u32, big-endian
        hb = [
            mac_byte(dst_mac, 0), mac_byte(dst_mac, 1), mac_byte(dst_mac, 2),
            mac_byte(dst_mac, 3), mac_byte(dst_mac, 4), mac_byte(dst_mac, 5),   # 0..5
            mac_byte(src_mac, 0), mac_byte(src_mac, 1), mac_byte(src_mac, 2),
            mac_byte(src_mac, 3), mac_byte(src_mac, 4), mac_byte(src_mac, 5),   # 6..11
            Constant(0x08, 8), Constant(0x00, 8),                               # 12,13 ethertype IPv4
            # ---- IPv4, 20 bytes (14..33) ----
            Constant(0x45, 8),                                                  # 14 ver/IHL
            self.ip_tos.storage,                                                # 15 TOS (0xB8)
            f_ip_len[8:16], f_ip_len[0:8],                                      # 16,17 total length (PER FLOW)
            Constant(0x00, 8), Constant(0x00, 8),                               # 18,19 ID = 0
            Constant(0x00, 8), Constant(0x00, 8),                               # 20,21 flags/frag = 0
            self.ip_ttl.storage,                                                # 22 TTL
            Constant(17, 8),                                                    # 23 proto = UDP
            be16(ipsum, 0), be16(ipsum, 1),                                     # 24,25 header checksum
            ip_byte(src_ip, 0), ip_byte(src_ip, 1),
            ip_byte(src_ip, 2), ip_byte(src_ip, 3),                             # 26..29 src IP
            ip_byte(dst_ip, 0), ip_byte(dst_ip, 1),
            ip_byte(dst_ip, 2), ip_byte(dst_ip, 3),                             # 30..33 dst IP
            # ---- UDP, 8 bytes (34..41) ----
            be16(sport, 0), be16(sport, 1),                                     # 34,35 src port
            be16(dport, 0), be16(dport, 1),                                     # 36,37 dst port
            f_udp_len[8:16], f_udp_len[0:8],                                    # 38,39 length (PER FLOW)
            Constant(0x00, 8), Constant(0x00, 8),                               # 40,41 checksum = 0
            # ---- Dante, 9 bytes (42..50) ----
            Constant(0x02, 8),                                                  # 42 constant tag
            ts_sec_emit[24:32], ts_sec_emit[16:24],
            ts_sec_emit[8:16],  ts_sec_emit[0:8],                                # 43..46 seconds
            f_ts_sub[24:32], f_ts_sub[16:24],
            f_ts_sub[8:16],  f_ts_sub[0:8],                                      # 47..50 subsec samples (PER FLOW)
        ]
        assert len(hb) == HDR_LEN
        header = Array(hb)

        # =========================================================
        # 4) Builder FSM: bytes → frame_ram, then stream frame_ram → source
        # =========================================================
        # FRAME RAM IN BRAM, not an Array of registers.
        #
        # AAF's 234-byte frame was 59 words; Dante's 435-byte frame is 109. As an
        # Array that is a 109:1 32-bit read mux -- exactly the wide-mux structure
        # that has repeatedly cost sys_clk on this flow, and it showed: the first
        # seed sweep of this design put sys at 52.6/53.9 MHz against a 55 MHz
        # build-reject floor, with 6 of 8 seeds stalling HeAP outright.
        #
        # One RAMB18 replaces the mux. The read is registered, so STREAM prefetches
        # the next word rather than reading combinationally -- see rd_idx handling.
        frame_mem = Memory(32, 128)
        frame_wp  = frame_mem.get_port(write_capable=True)
        frame_rp  = frame_mem.get_port(async_read=False)
        self.specials += frame_mem, frame_wp, frame_rp
        rd_idx_next = Signal(max=128)
        fr_we   = Signal()
        fr_adr  = Signal(max=128)
        fr_dat  = Signal(32)
        self.comb += [
            frame_wp.adr.eq(fr_adr), frame_wp.dat_w.eq(fr_dat), frame_wp.we.eq(fr_we),
            # Registered read: present rd_idx now, address rd_idx+1 so the next
            # beat's data is already latched when source.ready consumes this one.
            frame_rp.adr.eq(rd_idx_next),
        ]
        byte_idx  = Signal(max=TOTAL + 1)
        wacc      = Signal(24)            # holds lanes 0..2 of the in-progress word
        rd_idx    = Signal(max=N_WORDS)
        pkt_count = Signal(32)
        overruns  = Signal(32)
        # Ring-full write-drops: a USB sample arrives while ~have_space, so the
        # channel-addressed write is SUPPRESSED (line ~358) -> ONE stale channel
        # slot = an audible per-channel click. This was UNCOUNTED — overrun_count
        # only caught builder-busy skips, so a ring hitting the full rail was
        # invisible. Count it and fold into overrun_count.
        write_drops = Signal(32)
        self.sync += If(en & samp_vld & (started | first) & ~have_space,
                        write_drops.eq(write_drops + 1))
        self.comb += [
            self.packet_count.status.eq(pkt_count),
            self.overrun_count.status.eq(overruns + write_drops),
        ]

        # Current byte: header (idx<42), else the current sample's bytes (big-endian,
        # MSB first) from a 32-bit samp_hold register. samp_hold is fed one sample at
        # a time by the BRAM ring read pipeline in BUILD (rdf leads, latched at the
        # sample's last byte) — verified in /tmp/sim_sring.py. NOTHING here is wider
        # than 32 bits, so no wide LUT/FF structure for openXC7 to mis-synthesise
        # (the per-channel muxes AND the wide shift registers are both gone).
        cur_byte   = Signal(8)
        samp_hold  = Signal(32)
        rdf        = Signal(32)              # strided ring fetch address (combinational)
        pkt_primed = Signal()                # `primed` latched at packet start
        # Strip parity dummies; the sample comes from the SELECTED stream's ring
        # (stream_idx picks which of the de-interleaved rings).
        samp_rd = Signal(32)
        rdat    = Signal(36)
        # rdat's ring selector is cs_mem_d, defined with the slot addressing below;
        # the comb assignment therefore lives there, not here.
        self.comb += samp_rd.eq(Cat(rdat[0:8], rdat[9:17], rdat[18:26], rdat[27:35]))
        # 24-BIT PAYLOAD. AAF put 4 bytes per sample and could select the byte
        # straight from the low bits of the payload offset; Dante puts 3, and
        # modulo-3 of a byte counter is not something to build in logic. A small
        # 0..2 phase counter advanced in BUILD does the same job with an adder.
        #
        # MSB-justified: we emit the TOP three bytes of the 32-bit USB sample,
        # which is exactly what flows_tx.rs does with
        # i32.to_be_bytes()[0..bytes_per_sample].
        bph = Signal(2)                                # byte phase within a sample, 0..2
        self.comb += If(byte_idx < HDR_LEN,
            cur_byte.eq(header[byte_idx[0:6]]),
        ).Else(
            Case(bph, {
                0: cur_byte.eq(samp_hold[24:32]),
                1: cur_byte.eq(samp_hold[16:24]),
                2: cur_byte.eq(samp_hold[8:16]),
            }),
        )

        lane = byte_idx[0:2]
        widx = byte_idx[2:]

        # CONTIGUOUS read: `sc` = sample index 0..FRAME_SAMPLES-1, PREFETCHED on the
        # ring address; samp_hold lags rp.dat_r by one (BUILD pipeline). Each stream
        # reads its OWN 8-ch ring straight out (rd + sc) — no stride, no row math (the
        # de-interleave already happened at the demux write). All rings share the
        # address; stream_idx selects the data (samp_rd above).
        # SLOT/SAMPLE ADDRESSING, replacing the contiguous sc sweep.
        #
        # The ring already stores channel c of sample s at address s*8 + c within
        # ring c>>3, so an ARBITRARY channel is reachable by concatenation alone:
        #     mem  = ch >> 3
        #     addr = base + (samp << 3) + (ch & 7)
        # No stride arithmetic, no row math, no multiply -- the distinction that
        # matters, because the strided shared-ring read this design tried once
        # before produced garbage on hardware. The write path is untouched.
        #
        # Slot iterates fastest: Dante interleaves per sample, slot 0..n-1 within
        # each sample, which is exactly what the old sc sweep produced when every
        # flow carried 8 consecutive channels.
        cs_slot = Signal(4)              # slot whose address is on the RAM now
        cs_samp = Signal(8)              # sample index within this packet
        cm_all  = Signal(64)
        self.comb += cm_all.eq(Cat(cmlo_arr[stream_idx], cmhi_arr[stream_idx]))
        cs_ent  = Signal(8)
        self.comb += cs_ent.eq(Array([cm_all[i*8:(i+1)*8] for i in range(8)])[cs_slot])
        cs_ch   = Signal(6)
        cs_vld  = Signal()
        self.comb += [cs_ch.eq(cs_ent[0:6]), cs_vld.eq(cs_ent[7])]

        # Window START for this flow: all flows share `rd`, which advances one
        # 8-sample tick at a time, so an fpp=16 flow simply starts 8 samples
        # earlier. That keeps ONE read pointer for flows of different fpp.
        f_base = Signal(32)
        self.comb += f_base.eq(rd - Mux(fpp16, 64, 0))
        self.comb += rdf.eq(f_base + (cs_samp << 3) + cs_ch[0:3])
        for _s in range(streams):
            self.comb += rps[_s].adr.eq(rdf[0:log2depth])
        # The data mux follows the address by one cycle, so which ring and
        # whether the slot is silence must be delayed to match.
        cs_mem_d = Signal(max=max(streams, 2))
        cs_vld_d = Signal()
        self.sync += [cs_mem_d.eq(cs_ch[3:6]), cs_vld_d.eq(cs_vld)]
        self.comb += rdat.eq(Array([rps[_s].dat_r for _s in range(streams)])[cs_mem_d])
        # Next (slot, sample), slot fastest with wrap at nslots.
        nx_slot = Signal(4)
        nx_samp = Signal(8)
        self.comb += If(cs_slot == (nslots - 1),
            nx_slot.eq(0), nx_samp.eq(cs_samp + 1),
        ).Else(
            nx_slot.eq(cs_slot + 1), nx_samp.eq(cs_samp),
        )
        aligned_once = Signal()   # read anchored to the first block (anchor-once align)
        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm
        fsm.act("IDLE",
            If(send_req,
                NextValue(byte_idx, 0),
                NextValue(bph, 0),          # every packet starts on sample byte 0
                NextValue(cs_slot, 0), NextValue(cs_samp, 0),
                NextValue(stream_idx, 0),         # first stream of the block
                NextValue(pkt_primed, primed),    # whole block is silence OR audio
                If(~aligned_once, NextValue(rd, rd_anchor)),  # first block: anchor the read
                NextState("BUILD"),
            ),
        )
        # Latch on the IDLE->BUILD transition. No presentation accumulator to
        # advance any more: the timestamp counters run off the media clock and
        # were already latched at send_req, so all that survives here is the
        # read alignment and the diagnostic capture.
        do_emit = Signal()
        self.comb += do_emit.eq(send_req & fsm.ongoing("IDLE"))
        self.sync += [
            If(~en,
                aligned_once.eq(0),               # re-anchor the read on re-enable
            ).Elif(do_emit,
                aligned_once.eq(1),
                _dbg_ts_r.eq(ts_sub_emit),
                _dbg_sec_r.eq(ts_sec_emit),
            ),
        ]
        # BUILD: one byte/cycle into wacc; commit a word every 4th byte and on
        # the final (possibly partial) byte. 234 cycles ≈ 4.7 µs << 125 µs.
        commit_full = (lane == 3)
        is_last     = (byte_idx == (f_total - 1))
        # A flow with no slots is not configured. Skip it without building --
        # this is what lets six contexts exist while only the requested ones
        # transmit, and it is why removing the multicast bundles dropped us from
        # 65.5 Mbit/s to 0.03.
        flow_off = (nslots == 0)
        fsm.act("BUILD",
            If(flow_off,
                # Nothing configured here: move straight to the next context.
                If(stream_idx != (streams - 1),
                    NextValue(stream_idx, stream_idx + 1),
                    NextValue(byte_idx, 0), NextValue(bph, 0),
                    NextValue(cs_slot, 0), NextValue(cs_samp, 0),
                ).Else(
                    If(pkt_primed, NextValue(rd, rd + 64)),
                    NextState("IDLE"),
                ),
            ),
            Case(lane, {
                0: NextValue(wacc[0:8],   cur_byte),
                1: NextValue(wacc[8:16],  cur_byte),
                2: NextValue(wacc[16:24], cur_byte),
            }),
            If(commit_full,
                fr_we.eq(1), fr_adr.eq(widx), fr_dat.eq(Cat(wacc, cur_byte)),
            ),
            # BRAM ring read pipeline (verified sim_sring.py), overlapped with the
            # header so the first payload sample is ready at byte 42:
            #   byte 0: rp.adr=rdf=rd this cycle -> bump rdf so dat_r prefetches rd+1
            #   byte 1: latch samp_hold = ring[rd]
            #   payload last byte (pi&3==3): latch next sample, advance rd + rdf
            # A silence packet (~pkt_primed) holds samp_hold=0 and does NOT advance
            # rd, so the ring fills until primed; 48 samples/packet keeps ch-align.
            If(pkt_primed,
                If(byte_idx == 0,
                    NextValue(cs_slot, nx_slot), NextValue(cs_samp, nx_samp)),
                If(byte_idx == 1,
                    NextValue(samp_hold, Mux(cs_vld_d, samp_rd, 0))),
                If((byte_idx >= HDR_LEN) & (bph == (BYTES_PER_SAMPLE - 1)),
                    # Silence for an unmapped slot: the receiver asked for 4 slots
                    # but named only 2 channels, and the other two must carry zeros
                    # rather than whatever the ring happens to hold.
                    NextValue(samp_hold, Mux(cs_vld_d, samp_rd, 0)),
                    NextValue(cs_slot, nx_slot), NextValue(cs_samp, nx_samp),
                ),
            ).Else(
                NextValue(samp_hold, 0),
            ),
            # Byte phase: only runs inside the payload, wraps every 3 bytes.
            If(byte_idx >= HDR_LEN,
                If(bph == (BYTES_PER_SAMPLE - 1), NextValue(bph, 0)
                ).Else(NextValue(bph, bph + 1)),
            ),
            If(is_last,
                # Final word. pay_len is always a multiple of 4, so TOTAL % 4 == 3
                # for EVERY flow shape -- lanes 0,1,2 valid, rest zero-padded.
                fr_we.eq(1), fr_adr.eq(f_last_idx),
                fr_dat.eq(Cat(wacc[0:8], cur_byte, Constant(0, 32 - rem * 8))
                          if rem else Cat(wacc, cur_byte)),
                NextValue(rd_idx, 0),
                NextState("PREFETCH"),
            ).Else(
                NextValue(byte_idx, byte_idx + 1),
            ),
        )
        # One cycle to let the registered BRAM read land before the first beat.
        fsm.act("PREFETCH",
            NextState("STREAM"),
        )
        # Read address leads the presented word by one, except while PREFETCH is
        # loading word 0 into the output register.
        self.comb += If(fsm.ongoing("PREFETCH"),
            rd_idx_next.eq(0),
        ).Elif(fsm.ongoing("STREAM") & source.ready,
            rd_idx_next.eq(rd_idx + 1),
        ).Else(
            rd_idx_next.eq(rd_idx),
        )
        fsm.act("STREAM",
            source.valid.eq(1),
            source.data.eq(frame_rp.dat_r),
            source.last.eq(rd_idx == f_last_idx),
            If(rd_idx == f_last_idx,
                source.last_be.eq(LAST_BE),
            ).Else(
                source.last_be.eq(0xF),
            ),
            If(source.ready,
                If(source.last,
                    NextValue(pkt_count, pkt_count + 1),
                    # (no sequence number: Dante's header has none)
                    If(stream_idx != (streams - 1),
                        # More frames in this block: build the next stream's slice
                        # from the SAME 288-sample block (rd unchanged).
                        NextValue(stream_idx, stream_idx + 1),
                        NextValue(byte_idx, 0),
                        NextValue(bph, 0),
                        NextValue(cs_slot, 0), NextValue(cs_samp, 0),
                        NextState("BUILD"),
                    ).Else(
                        # Block done. rd advances ONE tick = 8 samples x 8 channel
                        # slots = 64 entries, because pacing is now at the finest
                        # fpp. An fpp=16 flow reads from rd-64 and so still sees a
                        # full, correctly aligned 16-sample window.
                        If(pkt_primed, NextValue(rd, rd + 64)),
                        NextState("IDLE"),
                    ),
                ).Else(
                    NextValue(rd_idx, rd_idx + 1),
                ),
            ),
        )
        # Safety: a send_req while not IDLE means the builder fell behind
        # (should never happen — build+stream ≪ 6 strobe periods). Count it.
        self.sync += If(send_req & ~fsm.ongoing("IDLE"), overruns.eq(overruns + 1))
