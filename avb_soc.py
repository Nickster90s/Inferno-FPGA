#!/usr/bin/env python3

#
# USB -> Dante SoC for Colorlight i9plus v6.1  (48 ch TX, native Dante protocol)
#
# Forked from avb-aes3 @ 466df96. The 48-channel USB ingress, media clock,
# TSU and gigabit LiteEth path are inherited unchanged and working; what is
# being replaced is the network transport (AVB/AVTP L2 -> Dante UDP/IP) and
# the clock protocol (802.1AS gPTP -> PTPv1).
#
# LiteX SoC with:
# - VexRiscv CPU + SRAM
# - LiteEth RGMII MAC (Wishbone interface for firmware raw frame access)
# - LiteEthTSU (PTP Timestamping Unit) with CSRs for the PTP firmware
# - MCR NCO (media clock; disciplined from the PTP addend ratio under Dante)
# - AAFPacketizer: 48 USB channels -> 6 x 8ch streams
#     (Phase 5 replaces this with dante_packetizer.py; same 6x8 structure,
#      since Dante's MAX_CHANNELS_IN_FLOW is also 8)
# - usb_avb_subsystem: LUNA-emitted 48ch UAC2 HS iso-async OUT on ULPI
# - UART (CH347)
#
# Removed in Phase 0 of the Dante conversion (parked in _avb_reference/):
#   AVTPSampleExtractor, CRFTimestampExtractor  (AVB RX observers)
#   firmware avdecc.c, srp.c, aaf.c             (1722.1 / MSRP / AAF)
# AES3 RX/TX were already removed upstream 2026-05-21 (rtl/_aes3_backup/).
#
# Build:
#   source /home/lisp/FPGA/env.sh
#   python3 avb_soc.py --build --firmware firmware/firmware.bin
#

import os
import sys

# REPRODUCIBLE BUILD: migen/litex name signals + the floorplan iterates dict/set
# collections whose order depends on PYTHONHASHSEED (randomized per process by
# default). That made the SAME --seed scatter 52-65 MHz run-to-run -> the seed
# roulette. PYTHONHASHSEED must be set BEFORE the interpreter starts, so if it is
# not already 0 we re-exec ourselves once with it fixed. Result: --seed 3 is now
# a STABLE 57.57 MHz, build once, no sweeping (see the --seed help below).
if os.environ.get("PYTHONHASHSEED") != "0":
    os.environ["PYTHONHASHSEED"] = "0"
    os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse

from migen import *
from litex.gen import *

from litex.build.io import DDROutput
from litex.build.generic_platform import Pins, IOStandard, Subsignal

from litex_boards.platforms import colorlight_i9plus

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.spi_flash import S7SPIFlash   # config-flash NV via STARTUPE2 (cs=/CRF persist)
from litex.soc.interconnect.csr import CSR, CSRStatus, CSRStorage, AutoCSR

from liteeth.phy.s7rgmii import LiteEthPHYRGMII
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "rtl"))
from rgmii_var_delay import LiteEthPHYRGMIIVar
from liteeth.mac import LiteEthMAC
from liteeth.core.ptp import LiteEthTSU

from dante_packetizer import DantePacketizer, TXFrameArbiter
from rx_gate import RXGate

from migen.genlib.fifo import SyncFIFO, AsyncFIFO

# CRG -------------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()


        # IDELAY reference clock (200 MHz).
        self.cd_idelay = ClockDomain()

        # Clk/Rst.
        clk25 = platform.request("clk25")

        # Single PLL for sys_clk, IDELAY, and audio clock.
        # The standalone audio MMCM never locked under openXC7 (whether driven
        # from clk25 or sys_clk), so we add the audio clock as a third PLLE2
        # output instead. PLLE2 only takes integer dividers — with a 1% margin
        # the solver finds a VCO that yields ~12.288 MHz, which is well within
        # the PCM5102A's internal-PLL lock range.
        self.pll = pll = S7PLL(speedgrade=-1)
        pll.vco_margin = 0.1
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_idelay, 200e6)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        # USB clock domain: 60 MHz recovered from the USB3300 ULPI CLK
        # output, re-emitted at phase 0 (the proven-good ULPI sampling
        # phase from avb-usb-host's eye-scan). Separate PLL since ulpi_clk
        # is async to clk25. VCO kept >=800 MHz by S7PLL's solver.
        self.cd_usb = ClockDomain()
        ulpi_clk = platform.request("ulpi_clock")
        self.usb_pll = usb_pll = S7PLL(speedgrade=-1)
        self.comb += usb_pll.reset.eq(self.rst)
        usb_pll.register_clkin(ulpi_clk, 60e6)
        usb_pll.create_clkout(self.cd_usb, 60e6, phase=0)
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_usb.clk)

        # PHASE-2: 2nd USB clock domain (Backup ULPI) — 60 MHz from ulpi2_clock
        # (V4, a bank-34 MRCC pin). 3rd S7PLL; async to clk25/sys/cd_usb. Only
        # instantiated when the platform has the ulpi2 extension (add_ulpi2).
        if hasattr(platform, "_has_ulpi2") and platform._has_ulpi2:
            self.cd_usb2 = ClockDomain()
            ulpi2_clk = platform.request("ulpi2_clock")
            self.usb2_pll = usb2_pll = S7PLL(speedgrade=-1)
            self.comb += usb2_pll.reset.eq(self.rst)
            usb2_pll.register_clkin(ulpi2_clk, 60e6)
            usb2_pll.create_clkout(self.cd_usb2, 60e6, phase=0)
            platform.add_false_path_constraints(self.cd_sys.clk, self.cd_usb2.clk)

# TSU CSR Wrapper --------------------------------------------------------------------------------------

class TSUWithCSRs(LiteXModule):
    """Wraps LiteEthTSU with CSR registers for firmware access."""
    def __init__(self, clk_freq):
        self.tsu = tsu = LiteEthTSU(clk_freq)

        # Timestamp output (80-bit: 48-bit seconds + 32-bit nanoseconds).
        self.timestamp = Signal(80)
        self.comb += self.timestamp.eq(Cat(tsu.nanoseconds, tsu.seconds))

        # --- Read-only status CSRs ---

        # Current time (read atomically: read seconds_hi first, which latches all).
        self._seconds_hi    = CSRStatus(16, description="PTP seconds [47:32] — read first to latch.")
        self._seconds_lo    = CSRStatus(32, description="PTP seconds [31:0].")
        self._nanoseconds   = CSRStatus(32, description="PTP nanoseconds.")

        # Latched seconds/nanoseconds for atomic read.
        seconds_latch    = Signal(48)
        nanoseconds_latch = Signal(32)
        latch_pulse      = Signal()

        # Latch on read of seconds_hi (active for 1 cycle on CSR read strobe).
        self.comb += latch_pulse.eq(self._seconds_hi.we)
        self.sync += If(latch_pulse,
            seconds_latch.eq(tsu.seconds),
            nanoseconds_latch.eq(tsu.nanoseconds),
        )
        self.comb += [
            self._seconds_hi.status.eq(seconds_latch[32:48]),
            self._seconds_lo.status.eq(seconds_latch[0:32]),
            self._nanoseconds.status.eq(nanoseconds_latch),
        ]

        # RX/TX captured timestamps.
        self._rx_ts_seconds  = CSRStatus(48, description="RX timestamp seconds.")
        self._rx_ts_nsec     = CSRStatus(32, description="RX timestamp nanoseconds.")
        self._tx_ts_seconds  = CSRStatus(48, description="TX timestamp seconds.")
        self._tx_ts_nsec     = CSRStatus(32, description="TX timestamp nanoseconds.")

        self.comb += [
            self._rx_ts_seconds.status.eq(tsu.rx_ts[32:80]),
            self._rx_ts_nsec.status.eq(tsu.rx_ts[0:32]),
            self._tx_ts_seconds.status.eq(tsu.tx_ts[32:80]),
            self._tx_ts_nsec.status.eq(tsu.tx_ts[0:32]),
        ]

        # --- Writable control CSRs ---

        # Addend (frequency word for tick accumulation).
        # Full 52-bit resolution: (addend << 20) | addend_frac. 1 LSB of
        # addend_frac changes ns/cycle by 1e9 / 2^52 ≈ 2.22e-7, i.e.
        # ~11.1 ns/sec rate adjustment per LSB at 50 MHz — sub-ppm tuning.
        # Routability of the 52x30 multiplier is fixed by patching
        # liteeth/core/ptp.py to use a shift-add tree instead of the `*`
        # operator (the natural form makes yosys infer a DSP48 cascade
        # that nextpnr-xilinx fails to route on XC7A50T).
        #
        # RESET VALUES MATTER (Dante Phase 0.5 hardware finding). These used to
        # reset to 0, which meant the TSU counter did not advance AT ALL until
        # firmware wrote an addend. Anything running before firmware -- notably
        # the boot loader -- then had a frozen timebase. Symptom found on
        # hardware: the loader's netload window never expired, so a board with
        # no netload host waited forever and never fell through to SPI flash.
        #
        # A timestamp unit should free-run at nominal rate from configuration.
        # gptp_init() still overwrites these with the identical value, so this
        # changes no firmware behaviour -- it just removes the dead window
        # between configuration and firmware start.
        #
        # Same formula as gptp.c:918 -- nominal 52-bit addend, rounded:
        #     (2^52 + clk/2) / clk, split as (addend << 20) | addend_frac
        _base_addend_full = ((1 << 52) + clk_freq // 2) // clk_freq
        self._addend      = CSRStorage(32, reset=(_base_addend_full >> 20),
            description="TSU addend integer part (resets to nominal for clk_freq).")
        self._addend_frac = CSRStorage(20, reset=(_base_addend_full & 0xFFFFF),
            description="TSU addend fractional part (20-bit, resets to nominal).")
        self.comb += [
            tsu.addend.eq(self._addend.storage),
            tsu.addend_frac.eq(self._addend_frac.storage),
        ]

        # Offset correction (signed, applied once then cleared by TSU).
        self._offset_hi = CSRStorage(17, description="Offset correction [48:32] (signed).")
        self._offset_lo = CSRStorage(32, description="Offset correction [31:0].")
        self._offset_apply = CSRStorage(1, description="Write 1 to apply offset correction.")

        # Apply offset on write strobe.
        # tsu.offset is 81-bit signed; CSRs hold 49 bits (lo 32 + hi 17).
        # The hi field's top bit (bit 16) is the sign bit at combined bit 48 —
        # replicate it into bits 49..80 so a negative firmware value (e.g.
        # -145000ns) doesn't get zero-extended into a huge positive offset.
        sign_bit = self._offset_hi.storage[16]
        self.sync += If(self._offset_apply.re,
            tsu.offset.eq(Cat(
                self._offset_lo.storage,                # bits  0..31
                self._offset_hi.storage,                # bits 32..48
                Replicate(sign_bit, 81 - 49),           # bits 49..80
            )),
        )

        # Coarse step (set time directly).
        self._step_seconds = CSRStorage(48, description="Step: target seconds.")
        self._step_nsec    = CSRStorage(32, description="Step: target nanoseconds.")
        self._step_apply   = CSRStorage(1,  description="Write 1 to step to target time.")

        self.comb += [
            tsu.step_target.eq(Cat(self._step_nsec.storage, self._step_seconds.storage)),
        ]
        self.sync += If(self._step_apply.re,
            tsu.step.eq(1),
        ).Else(
            tsu.step.eq(0),
        )

# MCR NCO -----------------------------------------------------------------------------------------------

class MCRNco(LiteXModule):
    """Numerically-controlled oscillator that emits a sample-rate strobe.

    fs = (increment * sys_clk_freq) / 2**32

    Firmware sets `increment` from a PI servo on CRF timestamps (mcr.c).
    A 33-bit add captures the carry-out as the sample strobe — that's the
    standard NCO trick: every time the 32-bit phase wraps, one sample tick
    has elapsed in fs-time.

    `sample_count` is a free-running count of strobes; firmware reads it to
    detect lock (delta_count over a known interval == expected sample count).
    """
    def __init__(self, sys_clk_freq, fs=48000):
        self.sample_strobe = Signal()

        # Default increment ≈ fs / sys_clk_freq, scaled to 32 bits.
        default_inc = int(round(fs * (1 << 32) / sys_clk_freq))

        self._increment = CSRStorage(32, reset=default_inc,
            description="NCO phase increment per sys_clk. fs = (inc*sys_clk_freq)/2^32.")
        # Exposed for the AAFPacketizer's CRF-dilated presentation-time ramp:
        # the nominal increment (constant) and the live servo'd increment.
        self.base_increment = default_inc
        self.increment      = self._increment.storage
        # FRACTIONAL INCREMENT -- sub-LSB rate resolution.
        #
        # `_increment` is a 32-bit integer, so one LSB is 1/4123169 = 0.2425 ppm
        # = 242 ppb. That is the FLOOR on how accurately the media clock rate can
        # be set: even a perfect servo leaves up to +/-121 ppb, which walks the
        # timestamp ~0.44 ms/hour with nothing to pull it back (the phase loop is
        # open). Measured on the bench 2026-08-04: residual +0.26 ppm = 1.07 LSB,
        # 0.9 ms/hour -- a receiver at 1 ms latency sees the timestamp cross its
        # whole buffer in about an hour. That is what "the media clock feels like
        # it is free running" actually was: quantisation, not a servo fault.
        #
        # Adding an 8-bit fractional part gives 256x finer resolution (0.95 ppb).
        # The dither happens at sys_clk (50 MHz), not at the servo update rate,
        # so the residual phase jitter is one NCO LSB at 20 ns scale -- orders of
        # magnitude below a sample period and inaudible. Dithering the integer
        # increment from FIRMWARE at 1 Hz would instead modulate the audio rate
        # by 0.24 ppm with a 1 s period, which is exactly the kind of wander that
        # has caused trouble here before.
        self._increment_frac = CSRStorage(8, reset=0,
            description="NCO increment fractional part, units of 1/256 LSB. "
                        "Effective increment = _increment + _increment_frac/256.")

        self._sample_count = CSRStatus(32,
            description="Free-running fs sample count (debug).")
        self._phase = CSRStatus(32,
            description="Current NCO phase (debug).")

        # Phase exposed as self.phase so MCRI2STx can pick BCK/LRCK/bit_idx
        # directly off the high bits (phase[31] = fs, phase[25] = 64*fs).
        self.phase   = Signal(32)
        next_phase   = Signal(33)
        sample_count = Signal(32)

        # Fractional accumulator: carries one extra LSB into the phase every
        # 256/frac cycles, so the AVERAGE increment is inc + frac/256.
        frac_acc = Signal(9)
        self.sync += frac_acc.eq(frac_acc[0:8] + self._increment_frac.storage)

        self.comb += next_phase.eq(self.phase + self._increment.storage
                                   + frac_acc[8])
        self.sync += [
            self.phase.eq(next_phase[:32]),
            self.sample_strobe.eq(next_phase[32]),
            If(self.sample_strobe,
                sample_count.eq(sample_count + 1)),
        ]
        self.comb += [
            self._sample_count.status.eq(sample_count),
            self._phase.status.eq(self.phase),
        ]


def ulpi_io():
    # USB3300 ULPI breakout on the P2 header. CLK=T4 (MRCC, clock-capable);
    # RST active-HIGH (USB3300 pin 9) so handled as plain Pins, driven low
    # by the gateware to release. Data is a bidirectional bus (TSTriple in
    # the SoC). Verified free vs eth/eth_clocks/sdram on this platform.
    # See avb-usb-host docs/phase3-bridge.md + memory ulpi-twisted-pair-wiring.
    #
    # USB_BACKUP=1: put the SINGLE (proven) USB subsystem on the BACKUP ULPI
    # pin locations (V4/R4/... — the 2nd physical connector) instead of Main.
    # This builds the small ~14k-cell netlist HeAP places in ~3 min, so it's the
    # fast way to validate the just-soldered Backup ULPI hardware WITHOUT the
    # dual-USB A/B netlist that jams the placer. Same resource names ("ulpi"/
    # "ulpi_clock"), so nothing downstream changes -- only the pin map.
    if os.environ.get("USB_BACKUP", "") == "1":
        return [
            ("ulpi_clock", 0, Pins("V4"), IOStandard("LVCMOS33")),   # MRCC (Backup)
            ("ulpi", 0,
                Subsignal("dir",  Pins("R4"), IOStandard("LVCMOS33")),
                Subsignal("nxt",  Pins("W4"), IOStandard("LVCMOS33")),
                Subsignal("stp",  Pins("T5"), IOStandard("LVCMOS33")),
                Subsignal("rst",  Pins("Y4"), IOStandard("LVCMOS33")),
                Subsignal("data", Pins("Y9 V8 W9 V9 W6 V7 Y3 Y6"),
                          IOStandard("LVCMOS33")),
            ),
        ]
    return [
        ("ulpi_clock", 0, Pins("T4"), IOStandard("LVCMOS33")),
        ("ulpi", 0,
            Subsignal("dir",  Pins("T3"),  IOStandard("LVCMOS33")),
            Subsignal("nxt",  Pins("U2"),  IOStandard("LVCMOS33")),
            Subsignal("stp",  Pins("U3"),  IOStandard("LVCMOS33")),
            Subsignal("rst",  Pins("R2"),  IOStandard("LVCMOS33")),
            Subsignal("data", Pins("V2 V3 W1 W2 Y1 AA1 AB1 Y2"),
                      IOStandard("LVCMOS33")),
        ),
    ]


def ulpi2_io():
    # PHASE-2: 2nd USB3300 ULPI breakout (the "Backup" A/B source). ALL bank 34
    # (single-bank like Main, for 60 MHz HS timing), HDMI-safe (avoids bank-34
    # T3-group pins used by the Ext-Board HDMI). CLK=V4 (MRCC). Pinout resolved
    # 2026-07-08 vs the fgg484 package + build XDC; see project_second_usb_timing.
    return [
        ("ulpi2_clock", 0, Pins("V4"), IOStandard("LVCMOS33")),   # MRCC
        ("ulpi2", 0,
            Subsignal("dir",  Pins("R4"), IOStandard("LVCMOS33")),
            Subsignal("nxt",  Pins("W4"), IOStandard("LVCMOS33")),
            Subsignal("stp",  Pins("T5"), IOStandard("LVCMOS33")),
            Subsignal("rst",  Pins("Y4"), IOStandard("LVCMOS33")),
            Subsignal("data", Pins("Y9 V8 W9 V9 W6 V7 Y3 Y6"),
                      IOStandard("LVCMOS33")),
        ),
    ]

# AVB SoC ----------------------------------------------------------------------------------------------

class AVBSoC(SoCCore):
    def add_csr_bridge(self, name="csr", origin=None, register=False):
        # PHASE-1 TIMING FIX (2026-07-08): force the Wishbone->CSR bridge to be
        # REGISTERED. LiteX only pipelines the CSR bus when SDRAM is present
        # (soc.py finalize: register=hasattr(self,'sdram')); this SoC is SRAM-only
        # so it was left combinational. The un-registered CSR read/write decode
        # fanning across all 13 banks into the TSU (csrbank11 seconds_hi) is THE
        # sys_clk critical path (13.7ns routing / 1.3ns logic). A registered bridge
        # turns it into flop->route->flop; cost is +1 CSR access wait-state, which
        # is invisible to firmware. See the timing deep-dive.
        return super().add_csr_bridge(name=name, origin=origin, register=True)

    def __init__(self, sys_clk_freq=int(50e6), **kwargs):
        # Pop our own kwarg before SoCCore sees it (it rejects unknown kwargs).
        bake_firmware = kwargs.pop("bake_firmware", False)
        litescope     = kwargs.pop("litescope", False)

        platform = colorlight_i9plus.Platform(toolchain="openxc7")

        # UART via CH347 on Ext-Board (TXD1/RXD1 routed to FPGA).
        platform.add_extension([
            ("serial", 0,
                Subsignal("tx", Pins("R3")),   # FPGA TX → CH347 RXD1
                Subsignal("rx", Pins("M3")),   # FPGA RX ← CH347 TXD1
                IOStandard("LVCMOS33"))
        ])

        # USB UAC2 ULPI (P2 header) — must be added before the CRG, which
        # requests ulpi_clock for the cd_usb PLL.
        platform.add_extension(ulpi_io())

        # PHASE-2 opt-in 2nd USB (Backup A/B source): env USB2=1. Adds the
        # bank-34 Backup ULPI extension so the CRG builds cd_usb2 and the SoC
        # instantiates the 2nd usb_avb_subsystem2 + A/B mux. Default OFF keeps
        # the proven single-USB build unchanged.
        platform._has_ulpi2 = os.environ.get("USB2", "") == "1"
        if platform._has_ulpi2:
            platform.add_extension(ulpi2_io())

        # CRG.
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore (VexRiscv + SRAM).
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident         = "Inferno-FPGA USB->Dante SoC on Colorlight i9+",
            ident_version = True,
            **kwargs
        )

        # ---- Executable RAM for runtime-loaded firmware (Dante Phase 0.5) ----
        #
        # WHY: every firmware change alters the BRAM ROM contents, which re-rolls
        # nextpnr's placement. That costs a 6-10 min P&R, a seed sweep, and a USB
        # hardware re-test PER FIRMWARE EDIT -- fatal for a project whose
        # remaining work is thousands of lines of new C against an undocumented
        # protocol (100+ iterations, not 5).
        #
        # FIX: the BRAM ROM now holds only a small, permanently-FROZEN loader.
        # Firmware lives here in `coderam` and is delivered at runtime over raw
        # Ethernet (fast: seconds) or from SPI flash (persistent). The bitstream
        # then never changes when firmware does.
        #
        # Must NOT be called "main_ram" -- LiteDRAM claims that name in Phase 1.
        # mode="rwx": the CPU fetches instructions from here, and the firmware's
        # crt0 also copies .data out of it, so it needs read+write+execute.
        #
        # Set --bake-firmware to skip this and use the legacy single-image map
        # (96 KB ROM, firmware baked, no coderam) for a standalone shippable
        # bitstream. Keep that path working: it is the fallback if the loader
        # ever bricks the boot sequence.
        if not bake_firmware:
            self.add_ram("coderam", origin=0x20000000, size=0x18000, mode="rwx")

        # ---- Config SPI flash (NV storage for cs= + CRF binding persistence) ----
        # Runtime read/write of the boot SPI flash via STARTUPE2 (CCLK) + the
        # XC7A50T-fgg484 config pins (FCS_B=T19, D00/MOSI=P22, D01/MISO=R22; CCLK
        # is internal through STARTUPE2 — the USRCCLKO BEL is present in the chipdb
        # so it places on openXC7). SPIMaster-based -> arbitrary commands (JEDEC ID,
        # read, sector-erase, page-program) from firmware. Config is stored in a
        # sector at the TOP of flash, far above the ~2.2 MB bitstream.
        self.platform.add_extension([
            ("cfg_spiflash", 0,
                Subsignal("cs_n", Pins("T19")),
                Subsignal("mosi", Pins("P22")),    # D00
                Subsignal("miso", Pins("R22")),    # D01
                Subsignal("hold", Pins("R21")),    # D03 / HOLD#
                IOStandard("LVCMOS33")),
        ])
        # NOTE: attribute name must NOT be "spiflash" or LiteX auto-adds the
        # liblitespi (litespi/mmap) software lib, which doesn't match this
        # SPIMaster-based S7SPIFlash and fails to compile. Use cfgflash -> CSRs
        # are cfgflash_spi_*.
        _cfg_pads = self.platform.request("cfg_spiflash")
        # Drive HOLD#(R21/D03) HIGH only (HOLD# low halts the flash -> reads all-
        # 0xFF). DO NOT drive WP#(P21/D02): it's only needed for WRITES, and the
        # USB regression appeared exactly when WP#/HOLD# were first driven — if the
        # board repurposed D02(P21) for a USB-side signal, driving it breaks USB
        # while the flash still reads via HOLD#. ISOLATION: USB back + JEDEC still
        # reads => P21 was the conflict (handle write-protect another way).
        self.comb += [_cfg_pads.hold.eq(1)]
        self.cfgflash = S7SPIFlash(_cfg_pads, sys_clk_freq, spi_clk_freq=2.5e6)

        # Ethernet PHY (RGMII, PHY1 / U9). Cable lands on U9 on this board —
        # confirmed by MDIO power-down test (addr 0 = U5/PHY0 unlinked, addr 1
        # = U9/PHY1 with link to MOTU AVB switch at 1000-FD).
        # B50612D needs ≥10 ms of reset after supply stabilizes; 1_000_000
        # cycles ≈ 20 ms at 50 MHz.
        # TX: revert to "as-it-was-when-Auvitran-crashed" — tx_delay=0,
        # no TX nibble swap (stock LiteEth ODDR), no PHY-side TXC delay.
        # That config sent frames out (the malformed AVDECC frames that
        # tripped Hive's parser), so TX path was at least intermittently
        # working.  Marginal but functional, and most importantly: real.
        # RX: keep what works — rx_delay=0, IDDR Q1/Q2 swap, PHY-side
        # RXC delay (shadow_07 bit 8 set by firmware).
        self.ethphy = LiteEthPHYRGMII(
            clock_pads      = platform.request("eth_clocks", 1),
            pads            = platform.request("eth", 1),
            tx_delay        = 0,
            rx_delay        = 0,
            hw_reset_cycles = 1_000_000,
        )

        # Heartbeat counter in eth_rx domain (now L3 via LiteEth) — proves RXC alive.
        from migen.genlib.cdc import MultiReg
        self.eth_rx_heartbeat = CSRStatus(8, description="Counter ticking in eth_rx (PHY1 L3) domain.")
        rx_hb = Signal(32)
        self.sync.eth_rx += rx_hb.eq(rx_hb + 1)
        self.specials += MultiReg(rx_hb[24:32], self.eth_rx_heartbeat.status)

        # PTP Timestamping Unit.
        self.tsu = TSUWithCSRs(sys_clk_freq)

        # nrxslots=2 — confirmed (again, 2026-05-21 with fresh firmware
        # rebuild) that increasing to 4 silently kills TX: entity vanishes
        # from Hive, no ADP egress. Tested both seed=4 and seed=23. The
        # slot-RAM expansion appears to actually break the LiteEth TX path,
        # not just a stale-firmware artifact. Don't try this again without
        # patching LiteEthMAC itself. The AAF flood overrun has to be
        # solved another way (drop early, deferred processing, etc.).
        self.add_ethernet(
            phy            = self.ethphy,
            data_width     = 8,
            with_timestamp = False,
            nrxslots       = 2,
            ntxslots       = 2,
        )

        # Wire TSU latch signals to MAC frame events.
        #
        # add_ethernet(data_width=8) maps to LiteEthMAC dw=32 internally
        # (litex/soc/integration/soc.py:1919), so mac.core.source.data
        # carries 4 wire bytes per beat. A byte-counted ethertype filter
        # at this point is the wrong abstraction. Instead: latch the
        # TSU's instantaneous time on the FIRST BEAT of every frame and
        # push it into a small ring buffer that firmware pops in lock-
        # step with slot consumption. No filter — works for all frames;
        # firmware uses the popped value only when the slot is a PTP
        # frame, and discards it for AVTP/MSRP/etc.
        mac = self.ethmac
        sram_writer = mac.interface.sram.writer

        rx_active     = Signal()
        rx_first_beat = Signal()
        self.comb += rx_first_beat.eq(
            mac.core.source.valid & mac.core.source.ready & ~rx_active
        )
        self.sync += [
            If(mac.core.source.valid & mac.core.source.ready,
                If(~rx_active,
                    rx_active.eq(1),
                ),
                If(mac.core.source.last,
                    rx_active.eq(0),
                ),
            ),
        ]

        # Capture instantaneous TSU time on first beat (registered so
        # subsequent frames can't overwrite before commit).
        captured_rx_ts = Signal(80)
        self.sync += If(rx_first_beat, captured_rx_ts.eq(self.tsu.timestamp))

        # Keep the legacy _rx_ts CSRs working too (single-shot, last-frame).
        self.comb += self.tsu.tsu.rx_latch.eq(rx_first_beat)

        # RX timestamp ring: push when the SRAM writer commits a frame
        # to a slot (stat_fifo.sink.valid pulses for one cycle during
        # FSM TERMINATE state). This guarantees one ring entry per slot
        # firmware will see — drops/MTU-discards don't desync the ring.
        self.rx_ts_fifo = rx_ts_fifo = SyncFIFO(80, 8)

        commit_pulse = sram_writer.stat_fifo.sink.valid
        self.comb += [
            rx_ts_fifo.din.eq(captured_rx_ts),
            rx_ts_fifo.we.eq(commit_pulse & rx_ts_fifo.writable),
        ]

        # ------------------------------------------------------------
        # DANTE PHASE 0: AVTPSampleExtractor and CRFTimestampExtractor removed.
        #
        # Both were AVB-specific RX-side observers of mac.core.source:
        #   - AVTPSampleExtractor pulled AAF audio samples into per-channel
        #     FIFOs and asserted match_at_eof -> sram.writer.discard_in to keep
        #     the 8000 fps AAF flood off the CPU.
        #   - CRFTimestampExtractor snooped CRF (AVB media clock) timestamps
        #     for the mcr.c PI servo.
        #
        # Dante has no AAF and no CRF: the media clock is disciplined from the
        # PTP addend ratio instead (see mcr.h), and milestone 1 is TX-only so
        # there is no network->device audio path at all. Removing these frees
        # ~4 BRAM sites and, more importantly, removes the RX fanout off
        # mac.core.source that previously cost sys_clk headroom.
        #
        # Reference copies: _avb_reference/{avtp_extractor,crf_extractor}.py
        # ------------------------------------------------------------

        # ------------------------------------------------------------
        # RX GATE (plan risk 8) — destination-MAC allow-list.
        #
        # The trigger fired: on the unmanaged bench switch, other Dante devices'
        # audio floods every port, and with only 2 RX slots (nrxslots CANNOT be
        # raised, see above) ethmac_sram_writer_errors climbs ~25/s. That is real
        # received-frame loss, and losing a PTPv1 FollowUp or DelayResp mispairs
        # it with the wrong Sync -> +/-5-10 us offset excursions against a
        # sub-microsecond steady state (TELEMETRY_AND_PTP.md section 2).
        #
        # NOT the rx_wired stream seam the plan expected. sram.writer.discard_in
        # (a local liteeth patch, already here for the AVB extractor) drops a
        # frame at the writer's FSM: no status-FIFO push, no ev_pending, and
        # crucially NO SLOT ADVANCE, so a filtered frame never occupies one of
        # the 2 RX slots. The gate is therefore a pure OBSERVER of the same
        # stream the writer sees; the RX datapath is not re-plumbed at all. This
        # also keeps the rx_ts ring consistent for free -- it is pushed on
        # commit_pulse, which discarded frames never reach.
        #
        # DISABLED AT RESET. rx_gate.enable is CSRStorage(reset=0), so this
        # bitstream behaves exactly like the previous one until firmware arms it,
        # and writing 0 backs it out instantly. That matters because a wrong
        # allow-list drops ALL RX -- no ARP, no PTP, no mDNS, no stats -- and the
        # CSR write is the only way back that does not involve a JTAG reflash.
        # ------------------------------------------------------------
        self.submodules.rx_gate = rx_gate = RXGate(dw=32)
        self.comb += [
            # Observe-only tap. Driving sink.ready from the real consumer's ready
            # (rather than tying it to 1) makes the gate's beat accounting
            # cycle-identical to the writer's, whatever the writer does with it.
            rx_gate.sink.valid.eq(mac.core.source.valid),
            rx_gate.sink.ready.eq(mac.core.source.ready),
            rx_gate.sink.last.eq(mac.core.source.last),
            rx_gate.sink.data.eq(mac.core.source.data),
            rx_gate.sink.last_be.eq(mac.core.source.last_be),
            # The one wire that does anything. Sampled by the writer only on the
            # cycle of sink.last (liteeth/mac/sram.py:96).
            sram_writer.discard_in.eq(rx_gate.discard),
        ]

        # CSRs: pop strobe + 80-bit popped value + level + overflow count.
        self.rx_ts_pop_lo  = CSRStatus(32, description="RX-ring popped nanoseconds.")
        self.rx_ts_pop_mid = CSRStatus(32, description="RX-ring popped seconds[31:0].")
        self.rx_ts_pop_hi  = CSRStatus(16, description="RX-ring popped seconds[47:32].")
        self.rx_ts_pop     = CSRStorage(1, description="Write 1 to pop next ring entry into pop_lo/mid/hi.")
        self.rx_ts_level   = CSRStatus(4, description="RX-ring fill level.")
        self.rx_ts_overflow_count = CSRStatus(32, description="Frames where ring was full at commit (lost ts).")
        self.rx_ts_commit_count   = CSRStatus(32, description="Total RX frames committed (sanity).")

        popped       = Signal(80)
        overflow_cnt = Signal(32)
        commit_cnt   = Signal(32)
        self.sync += [
            If(self.rx_ts_pop.re & rx_ts_fifo.readable,
                popped.eq(rx_ts_fifo.dout),
            ),
            If(commit_pulse,
                commit_cnt.eq(commit_cnt + 1),
                If(~rx_ts_fifo.writable,
                    overflow_cnt.eq(overflow_cnt + 1),
                ),
            ),
        ]
        self.comb += [
            rx_ts_fifo.re.eq(self.rx_ts_pop.re & rx_ts_fifo.readable),
            self.rx_ts_pop_lo.status.eq(popped[0:32]),
            self.rx_ts_pop_mid.status.eq(popped[32:64]),
            self.rx_ts_pop_hi.status.eq(popped[64:80]),
            self.rx_ts_level.status.eq(rx_ts_fifo.level),
            self.rx_ts_overflow_count.status.eq(overflow_cnt),
            self.rx_ts_commit_count.status.eq(commit_cnt),
        ]

        # TX timestamp: latch on first byte sent.
        tx_sof = Signal()
        tx_active = Signal()
        self.sync += [
            If(mac.core.sink.valid & mac.core.sink.ready,
                If(~tx_active,
                    tx_sof.eq(1),
                    tx_active.eq(1),
                ).Else(
                    tx_sof.eq(0),
                ),
                If(mac.core.sink.last,
                    tx_active.eq(0),
                ),
            ).Else(
                tx_sof.eq(0),
            ),
        ]
        # tx_latch is gated to firmware (control-plane) frames only — see the
        # TXFrameArbiter wiring further down. The gateware AAF talker pushes
        # 8000 frames/s through this same core.sink; without the gate it would
        # clobber the gPTP egress timestamp firmware reads after each Sync /
        # Pdelay. tx_sof is referenced there (it is an __init__-scope Signal).
        self._tx_sof = tx_sof

        # Media Clock Recovery NCO — driven by firmware PI servo on CRF.
        self.mcr = MCRNco(sys_clk_freq, fs=48000)

        # ---- USB UAC2 sink (Phase 3 / P3.2) ------------------------------
        # Drop-in Verilog from avb-usb-host (tag usb-hs-uac2-working):
        # ULPI link via ultraembedded ulpi_wrapper.v + LUNA USB device.
        # Goal of P3.2: enumerate as USB HS UAC2 INSIDE this AVB bitstream.
        # The decoded 8ch audio stream is exposed but, for now, just drained
        # (ready=1) and fed a nominal 48k feedback. P3.3 wires it to an
        # AsyncFIFO → AAF talker; P3.4 drives feedback from the MCR rate.
        _rtl = os.path.join(os.path.dirname(__file__), "rtl")
        platform.add_source(os.path.join(_rtl, "usb_avb_subsystem.v"))
        platform.add_source(os.path.join(_rtl, "ulpi_wrapper.v"))

        ulpi = platform.request("ulpi")
        ulpi_data_ts = TSTriple(8)
        self.specials += ulpi_data_ts.get_tristate(ulpi.data)

        # --- Runtime-tunable IDELAY on the ULPI INPUTS (deterministic
        #     60 MHz sampling, placement-independent) ----------------------
        # The ULPI bus is 60 MHz source-synchronous. yosys+nextpnr-xilinx
        # inserts no auto-IDDR/IDELAY, so the IOB→first-flop routing delay
        # (which varies every rebuild as the firmware ROM shuffles
        # placement) shifts the sampling point in/out of the data eye →
        # USB enumerates one build, full-speed/error-71 the next. IDELAYE2
        # in VAR_LOAD mode puts a firmware-tunable delay right at each
        # input pin so sampling is fixed by the tap, not by routing.
        # Shares the CRG's existing S7IDELAYCTRL (cd_idelay 200 MHz) — do
        # NOT add a second IDELAYCTRL (errors "more than one IDELAYCTRL for
        # the default IODELAY_GROUP"). Firmware sweeps ulpi_idelay_tap
        # (0..31, ~78 ps/tap) to centre the eye; the 'u' command steps it.
        self.ulpi_idelay_tap  = CSRStorage(5, reset=8,
            description="ULPI input IDELAY tap 0-31 (~78 ps each).")
        self.ulpi_idelay_load = CSRStorage(1,
            description="Write 1 to load ulpi_idelay_tap into all ULPI input IDELAYs.")
        _ulpi_ld = self.ulpi_idelay_load.re
        def _ulpi_idelay(sig_in):
            out = Signal()
            self.specials += Instance("IDELAYE2",
                p_IDELAY_TYPE="VAR_LOAD", p_DELAY_SRC="IDATAIN",
                p_HIGH_PERFORMANCE_MODE="TRUE", p_SIGNAL_PATTERN="DATA",
                p_REFCLK_FREQUENCY=200.0, p_CINVCTRL_SEL="FALSE",
                p_PIPE_SEL="FALSE", p_IDELAY_VALUE=0,
                i_C=ClockSignal("sys"), i_LD=_ulpi_ld,
                i_CNTVALUEIN=self.ulpi_idelay_tap.storage,
                i_CE=0, i_INC=0, i_LDPIPEEN=0, i_REGRST=0,
                i_IDATAIN=sig_in, o_DATAOUT=out)
            return out
        ulpi_dir_d  = _ulpi_idelay(ulpi.dir)
        ulpi_nxt_d  = _ulpi_idelay(ulpi.nxt)
        ulpi_data_d = Signal(8)
        for _i in range(8):
            self.comb += ulpi_data_d[_i].eq(_ulpi_idelay(ulpi_data_ts.i[_i]))

        # P3.3 (task #67): the wrapper now has an INTERNAL cd_usb→sys
        # AsyncFIFO (added in avb-usb-host/gateware/usb_avb_subsystem.py)
        # so we no longer expose the cd_usb-domain channel stream — the
        # consumer reads sys-domain samples via the CSRs below. This
        # keeps every cd_usb-side cell inside the wrapper hierarchy
        # (matched by floorplan_usb.py) and avoids the ULPI-timing
        # regression the prior external-FIFO attempt caused.
        sample_lo_w  = Signal(32)
        sample_hi_w  = Signal(32)
        sample_rdy_w = Signal()
        sample_pop_w = Signal()
        sample_ovf_w = Signal(32)
        dbg_rxbeats_w = Signal(32)   # core->EP raw byte beats (rx.next & rx.valid)
        dbg_epout_w   = Signal(32)   # EP->decoder beats (stream.valid & ready)
        # USB async-feedback inputs to the wrapper: the media-clock strobe and
        # the consumer-FIFO level. The wrapper now measures the rate + computes
        # the feedback internally (SOF-synchronised); we just feed it these two.
        # block_level is a forward Signal driven from aaf_pkt.block_level below
        # (aaf_pkt is created after this Instance).
        usb_block_level = Signal(8)
        # block_level (= wr-rd, sys domain) is consumed by the wrapper's feedback
        # loop in cd_usb. It was crossing sys->cd_usb UNSYNCHRONISED — nextpnr
        # timed it as a single-cycle path (-4.59 ns FAIL, the worst path capping
        # sys_clk Fmax) and it's a real multi-bit CDC metastability hazard. Add a
        # proper synchroniser: registers it into cd_usb (the MultiReg input is a
        # CDC endpoint, so the cross-domain path is no longer timed single-cycle).
        # block_level changes slowly (±1/sample) so per-bit MultiReg skew is a
        # benign ±1 transient on a coarse feedback trim.
        from migen.genlib.cdc import MultiReg as _MultiReg
        usb_block_level_usb = Signal(8)
        self.specials += _MultiReg(usb_block_level, usb_block_level_usb, odomain="usb")
        # Firmware feedback override (Q16.16 samples/µframe; 0 = auto loop).
        # Lets firmware sweep hardcoded async-feedback values live to test the
        # host's response without a gateware rebuild.
        self.usb_fb_ovr = CSRStorage(32, description="USB async-feedback override (0=auto).")

        self.specials += Instance("usb_avb_subsystem",
            i_clk       = ClockSignal("sys"),
            i_rst       = ResetSignal("sys"),
            i_usb_clk   = ClockSignal("usb"),
            i_ulpi_dir_i   = ulpi_dir_d,
            i_ulpi_nxt_i   = ulpi_nxt_d,
            i_ulpi_data_i  = ulpi_data_d,
            o_ulpi_data_o  = ulpi_data_ts.o,
            o_ulpi_data_oe = ulpi_data_ts.oe,
            o_ulpi_stp_o   = ulpi.stp,
            o_ulpi_rst_o   = ulpi.rst,
            o_sample_lo             = sample_lo_w,
            o_sample_hi             = sample_hi_w,
            o_sample_readable       = sample_rdy_w,
            i_sample_pop            = sample_pop_w,
            o_sample_overflow_count = sample_ovf_w,
            o_dbg_rx_beats          = dbg_rxbeats_w,
            o_dbg_ep_out            = dbg_epout_w,
            i_sample_strobe         = self.mcr.sample_strobe,
            i_block_level           = usb_block_level_usb,   # cd_usb-synchronised
            i_fb_ovr                = self.usb_fb_ovr.storage,
        )

        # Firmware-facing drain: 2 CSR ops/sample (read packed head, then
        # pulse pop) — half the original 4 (readable+lo+hi+pop), using the
        # proven CSRStorage pop. The earlier 1-op auto-pop-on-read had a
        # read/advance race that corrupted the head, so this keeps an
        # explicit pop. Packed 32-bit data word:
        #   [2:0]   channel index (0..7)
        #   [3]     first (channel-0-of-frame marker)
        #   [4]     valid (1 = FIFO non-empty; head is real)
        #   [7:5]   reserved
        #   [31:8]  24-bit audio, MSB-aligned → (v & 0xFFFFFF00) is the
        #           32-bit signed sample (= Milan AAF 32-bit INT), no shift.
        # Firmware: while(guard){ v=usb_sample_data; if(!(v&0x10)) break;
        #            ...; usb_sample_pop=1; }
        # 48ch bridge: sample_hi = {channel[0:6], first[6]} (6-bit channel 0..47).
        self.usb_sample_data     = CSRStatus(32, description="USB→AAF FIFO head: [31:8]audio [7]valid [6]first [5:0]ch.")
        self.usb_sample_pop      = CSRStorage(1, description="Write 1 to advance the USB→AAF FIFO read pointer.")
        self.usb_sample_overflow = CSRStatus(32, description="Samples dropped at the cd_usb-side FIFO write port.")
        # Localisation diagnostics (cd_usb, synced): where the ~25x replay enters.
        self.usb_dbg_rx_beats = CSRStatus(32, description="core->EP raw byte beats (rx.next & rx.valid).")
        self.usb_dbg_ep_out   = CSRStatus(32, description="EP->decoder beats (stream.valid & ready).")
        self.comb += [
            self.usb_sample_data.status.eq(Cat(sample_hi_w[0:7],   # [6:0] channel(6)+first
                                               sample_rdy_w,        # [7]   valid
                                               sample_lo_w[8:32])), # [31:8] audio MSB-aligned
            self.usb_sample_overflow.status.eq(sample_ovf_w),
            self.usb_dbg_rx_beats.status.eq(dbg_rxbeats_w),
            self.usb_dbg_ep_out.status.eq(dbg_epout_w),
        ]
        # usb_feedback (the async-feedback value the wrapper now transmits) is
        # driven from the MCR rate + a FIFO-level trim below, after aaf_pkt
        # exists.

        # ---- PHASE-2: 2nd USB (Backup) + A/B source mux ----------------------
        # AVDECC CONTROL (firmware writes usb_source_select on SET_CONTROL):
        # 0=Main(USB1), 1=Backup(USB2). Both hosts slave to the same gPTP/CRF
        # media clock via their own feedback loops, so switching is a clean
        # stream-select: the (single, shared) AAF packetizer reads whichever USB
        # is selected; the other is drained-and-discarded so its FIFO can't
        # overflow while hot-standby. No SRC — both are already at the gPTP rate.
        self.usb_source_select = CSRStorage(1,
            description="USB A/B source: 0=Main(USB1) 1=Backup(USB2). Set by AVDECC CONTROL.")
        sample_lo_mux  = Signal(32)
        sample_hi_mux  = Signal(32)
        sample_rdy_mux = Signal()
        self._usb2_enabled = platform._has_ulpi2
        if self._usb2_enabled:
            platform.add_source(os.path.join(_rtl, "usb_avb_subsystem2.v"))
            ulpi2 = platform.request("ulpi2")
            ulpi2_data_ts = TSTriple(8)
            self.specials += ulpi2_data_ts.get_tristate(ulpi2.data)
            self.ulpi2_idelay_tap  = CSRStorage(5, reset=8,
                description="Backup ULPI input IDELAY tap 0-31.")
            self.ulpi2_idelay_load = CSRStorage(1,
                description="Write 1 to load ulpi2_idelay_tap.")
            _u2_ld = self.ulpi2_idelay_load.re
            def _u2_idelay(sig_in):
                out = Signal()
                self.specials += Instance("IDELAYE2",
                    p_IDELAY_TYPE="VAR_LOAD", p_DELAY_SRC="IDATAIN",
                    p_HIGH_PERFORMANCE_MODE="TRUE", p_SIGNAL_PATTERN="DATA",
                    p_REFCLK_FREQUENCY=200.0, p_CINVCTRL_SEL="FALSE",
                    p_PIPE_SEL="FALSE", p_IDELAY_VALUE=0,
                    i_C=ClockSignal("sys"), i_LD=_u2_ld,
                    i_CNTVALUEIN=self.ulpi2_idelay_tap.storage,
                    i_CE=0, i_INC=0, i_LDPIPEEN=0, i_REGRST=0,
                    i_IDATAIN=sig_in, o_DATAOUT=out)
                return out
            ulpi2_dir_d  = _u2_idelay(ulpi2.dir)
            ulpi2_nxt_d  = _u2_idelay(ulpi2.nxt)
            ulpi2_data_d = Signal(8)
            for _i in range(8):
                self.comb += ulpi2_data_d[_i].eq(_u2_idelay(ulpi2_data_ts.i[_i]))
            sample_lo2_w  = Signal(32); sample_hi2_w  = Signal(32)
            sample_rdy2_w = Signal();   sample_pop2_w = Signal()
            sample_ovf2_w = Signal(32); dbg2_a = Signal(32); dbg2_b = Signal(32)
            self.usb2_fb_ovr = CSRStorage(32, description="Backup USB async-feedback override (0=auto).")
            usb2_block_level_usb2 = Signal(8)
            self.specials += _MultiReg(usb_block_level, usb2_block_level_usb2, odomain="usb2")
            self.specials += Instance("usb_avb_subsystem2",
                i_clk=ClockSignal("sys"), i_rst=ResetSignal("sys"),
                i_usb_clk=ClockSignal("usb2"),
                i_ulpi_dir_i=ulpi2_dir_d, i_ulpi_nxt_i=ulpi2_nxt_d, i_ulpi_data_i=ulpi2_data_d,
                o_ulpi_data_o=ulpi2_data_ts.o, o_ulpi_data_oe=ulpi2_data_ts.oe,
                o_ulpi_stp_o=ulpi2.stp, o_ulpi_rst_o=ulpi2.rst,
                o_sample_lo=sample_lo2_w, o_sample_hi=sample_hi2_w,
                o_sample_readable=sample_rdy2_w, i_sample_pop=sample_pop2_w,
                o_sample_overflow_count=sample_ovf2_w,
                o_dbg_rx_beats=dbg2_a, o_dbg_ep_out=dbg2_b,
                i_sample_strobe=self.mcr.sample_strobe,
                i_block_level=usb2_block_level_usb2, i_fb_ovr=self.usb2_fb_ovr.storage)
            self.usb2_sample_overflow = CSRStatus(32, description="Backup USB samples dropped.")
            self.comb += self.usb2_sample_overflow.status.eq(sample_ovf2_w)
            _sel = self.usb_source_select.storage
            self._usb2_pop = sample_pop2_w
            self._usb2_rdy = sample_rdy2_w
            self._sel = _sel
            self.comb += [
                If(_sel,
                   sample_lo_mux.eq(sample_lo2_w), sample_hi_mux.eq(sample_hi2_w),
                   sample_rdy_mux.eq(sample_rdy2_w),
                ).Else(
                   sample_lo_mux.eq(sample_lo_w), sample_hi_mux.eq(sample_hi_w),
                   sample_rdy_mux.eq(sample_rdy_w),
                ),
            ]
        else:
            self.comb += [
                sample_lo_mux.eq(sample_lo_w), sample_hi_mux.eq(sample_hi_w),
                sample_rdy_mux.eq(sample_rdy_w),
            ]

        # Gateware AAF TX packetizer (task #67 / D2-in-gateware). Drains the
        # SAME USB sample handshake firmware uses, assembles AVTP-AAF frames,
        # stamps presentation_time from the TSU, and emits on its own MAC TX
        # stream — keeping the CPU out of the per-sample audio path. Paced by
        # mcr.sample_strobe, so when firmware servos the NCO to CRF (cs=1,
        # locked) the stream rate IS the CRF media clock (see module header).
        # DANTE PHASE 5: the AAF packetizer becomes the Dante packetizer. Same
        # 6x8 time-mux, same ring, same builder -- only the wire format differs.
        # Kept as `aaf_pkt` so the surrounding USB feedback servo, arbiter wiring
        # and CSR names are untouched; renaming it would churn every call site
        # for no gain and break the firmware's csr accessors.
        self.submodules.aaf_pkt = aaf_pkt = DantePacketizer(
            mcr               = self.mcr,
            tsu               = self.tsu.tsu,
            usb_sample_lo     = sample_lo_mux,   # A/B-muxed (Main/Backup)
            usb_sample_hi     = sample_hi_mux,
            usb_readable      = sample_rdy_mux,
            channels          = 8,     # Dante MAX_CHANNELS_IN_FLOW
            samples_per_packet= 16,    # fpp=16, as the bundle records advertise
            streams           = 6,     # 6 multicast bundles = 48 ch
            fifo_depth        = 256,   # per-ring SRING_DEPTH = next_pow2(256*8) = 2048
                                       # samples = 5.3 ms. The old 64 (=512 samples=1.33ms)
                                       # was 8x SHALLOWER than the proven 8ch build's 10.7ms
                                       # and could not absorb the host's ~1ms bursty
                                       # block delivery: the ±200-sample swing clipped BOTH
                                       # rails (HW: underrun+ovr both climbed). 2048 centers
                                       # the swing ~816..1232, clear of floor(48)/full(2048),
                                       # with margin for Spotify pause/play transients. The
                                       # integral (usb_avb_subsystem) holds the center; this
                                       # gives it room. BRAM: ~24 RAMB18 (fits, 53 free).
        )
        # USB FIFO pop is ALWAYS owned by the gateware assembler now: its do_pop
        # drains-and-discards while the talker is disabled (see aaf_packetizer),
        # so the cd_usb→sys bridge never backs up. The old firmware-CSR drain
        # fallback couldn't keep up with 384 ksample/s and left the block_fifo
        # pinned full at stream start (zero jitter headroom). usb_sample_pop CSR
        # is retained for diagnostics but no longer the drain path.
        if self._usb2_enabled:
            # Selected USB feeds the AAF (pop = aaf_pkt.usb_pop); the other is
            # drained-and-discarded (pop = its own readable) so its cd_usb FIFO
            # can't overflow while it is the hot standby.
            self.comb += [
                If(self._sel,          # Backup selected -> USB2 feeds, USB1 discards
                   self._usb2_pop.eq(aaf_pkt.usb_pop),
                   sample_pop_w.eq(sample_rdy_w),
                ).Else(                # Main selected -> USB1 feeds, USB2 discards
                   sample_pop_w.eq(aaf_pkt.usb_pop),
                   self._usb2_pop.eq(self._usb2_rdy),
                ),
            ]
        else:
            self.comb += sample_pop_w.eq(aaf_pkt.usb_pop)

        # ---- USB async feedback (P3.4) ----
        # The rate measurement + FIFO-centering loop now lives INSIDE the
        # wrapper (usb domain, SOF-synchronised — the ADAT/LUNA canonical
        # topology; a per-sys-cycle loop here hunted/pinned the FIFO). We only
        # feed it the consumer-FIFO level. When the gateware doesn't own the USB
        # drain (firmware path), report mid-level so the wrapper applies no trim
        # (feedback = pure measured media-clock rate).
        # aaf_pkt.block_level is ALREADY scaled to 0..128 in the packetizer
        # (level>>5; samples 0..4096 -> 0..128), so feed it STRAIGHT THROUGH:
        # the bridge feedback's CENTRE=64 targets ring mid (block 64 = 2048 of
        # 4096 samples). A stale >>2 here (left over from an OLD 0..512
        # block_level) DOUBLE-scaled it to 0..32, so err = CENTRE - level was
        # ALWAYS positive -> feedback always told the host "send MORE" -> ring
        # pinned FULL (level=127) -> have_space gated writes DROPPED samples ->
        # not-clean audio. else-branch (gateware DISABLED, pre-gPTP-lock) MUST be the
        # servo CENTRE so the wrapper applies NO trim while we're held off: the talker
        # is gated off for ~6s until gPTP locks, and during that window do_pop still
        # drains the USB FIFO but do_write is gated -> if we feed a BELOW-centre level
        # the wrapper tells the host "send MORE" the whole time -> the host over-delivers
        # -> when enable finally asserts the ring is slammed full -> have_space drops
        # writes -> channel-phase corruption = the cold-start "bad audio". fifo_depth>>3
        # = 64>>3 = 8 (NOT 64 — the old comment assumed fifo_depth=512); 8 << centre ->
        # 6s of over-delivery. Feed the CENTRE constant (64) instead.
        _BL_CENTRE = 64
        self.comb += usb_block_level.eq(
            Mux(aaf_pkt.enable.storage,
                aaf_pkt.block_level, _BL_CENTRE))

        # Frame-atomic TX mux: firmware SRAM reader (priority) + gateware AAF
        # talker → MAC core sink. Claim the wishbone-TX seam (see the LiteEth
        # do_finalize hook) so we, not the library, drive core.sink.
        mac.tx_wired = True
        self.submodules.tx_arbiter = tx_arbiter = TXFrameArbiter(
            [mac.interface.source, aaf_pkt.source], dw=32)
        self.comb += tx_arbiter.source.connect(mac.core.sink)

        # gPTP TX timestamp: latch ONLY on firmware (port 0) frames so the AAF
        # talker can't overwrite the egress timestamp firmware reads.
        self.comb += self.tsu.tsu.tx_latch.eq(self._tx_sof & tx_arbiter.firmware_granted)

        # ---- LiteScope on the TX seam (--litescope) ------------------------
        #
        # Probes the PRODUCING end (packetizer source + its build pointers) and
        # the CONSUMING end (arbiter -> MAC core sink) in one trace, so a stall
        # can be attributed. The open question it exists for: 4 slots at fpp=60
        # binds, packet_count rises at exactly the right rate -- and that
        # counter increments on (source.last & source.ready), so the sink is
        # ACCEPTING -- yet no frame reaches the wire. Either the last beat never
        # carries the right byte enables, or the MAC discards the frame after
        # accepting it.
        if litescope:
            from litescope import LiteScopeAnalyzer
            analyzer_signals = [
                aaf_pkt.source.valid, aaf_pkt.source.ready,
                aaf_pkt.source.last,  aaf_pkt.source.last_be,
                tx_arbiter.source.valid, tx_arbiter.source.ready,
                tx_arbiter.source.last,  tx_arbiter.source.last_be,
                tx_arbiter.firmware_granted,
                mac.core.sink.valid, mac.core.sink.ready, mac.core.sink.last,
                aaf_pkt.dbg_byte_idx, aaf_pkt.dbg_rd_idx,
                aaf_pkt.dbg_f_last_idx, aaf_pkt.dbg_f_total,
                aaf_pkt.dbg_stream_idx, aaf_pkt.dbg_pay_len,
                aaf_pkt.dbg_send_req, aaf_pkt.dbg_any_pend,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(
                analyzer_signals, depth=2048, clock_domain="sys", csr_csv="analyzer.csv")
            self.add_jtagbone()

        # LED.
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq,
        )

# Build ------------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="AVB-AES3 SoC on Colorlight i9+")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--soft-only",    action="store_true", help="Generate software headers only (no P&R).")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--seed", default=8, type=int, help="nextpnr P&R seed. "
        "PINNED to 5 for the CURRENT netlist (Phase 0.5 loader with phy_wait_link, "
        "loader.bin 8268 B). Chosen on WORST-CASE margin against the real "
        "requirements (sys>=50 with a >=55 build-reject floor, usb>=60, "
        "eth_tx>=125, eth_rx>=125), not on any single headline figure: "
        "sys 59.21, usb 89.86, eth_tx 143.10, eth_rx 163.48 -- worst margin +14.5%. "
        "Runners-up: seed 7 (+7.7%), seed 6 (+3.9%), seed 9 (+2.6%), seed 1 (+0.9%). "
        "REJECTED: seed 8 has the 2nd-best sys (61.39) but eth_rx 109.90 FAILS the "
        "125 MHz gigabit requirement; seed 4 has the BEST sys (66.12) and still "
        "fails on eth_rx 122.06 -- picking on sys alone would ship a broken RX "
        "datapath. Seeds 2 and 3 fall below the sys floor. "
        "A SEED PIN IS A PROPERTY OF THE (seed, netlist) PAIR, NOT THE SEED. "
        "Since Phase 0.5 a FIRMWARE change no longer re-rolls placement (firmware "
        "lives in coderam and is delivered at runtime), so this pin survives "
        "firmware work -- but ANY gateware or LOADER change re-rolls it and needs "
        "a fresh tools/seed_sweep.sh. The loader counts as gateware because it is "
        "baked into the ROM; that is why it is meant to be frozen. "
        "AND: HeAP stalls are NOT reliably reproducible. Seed 5 stalled for 43 min "
        "on one run of this exact netlist and placed in 57.81s on another, so treat "
        "a stall as 'retry once, then move on', not as a permanent property. "
        "build.sh and seed_sweep.sh both carry a stall watchdog (STALL_SECS). "
        "Finally, global Fmax does NOT predict USB quality -- the seed shuffles "
        "intra-region USB placement, so any new pin must still be tested on HW.")
    parser.add_argument("--no-floorplan", action="store_true",
        help="Disable the USB-near-ULPI proximity floorplan (floorplan_usb.py "
             "constrains USB cells to X<=30, Y=10-70 — close to the ULPI pins "
             "at X=1, Y=23-49). ON by default: with the gateware CRF extractor "
             "+ MCRI2STx + AVB SoC present, nextpnr's free placement scatters "
             "USB cells too far from the ULPI pins → marginal 60 MHz HS chirp "
             "→ enumerate as full-speed + error -71. The proximity floorplan "
             "deterministically pins USB near the pins. Only pass --no-floorplan "
             "for debugging.")
    parser.add_argument("--litescope", action="store_true",
        help="Add a LiteScope analyzer on the TX seam plus jtagbone, for tracing "
             "why a bound context can count packets without any frame reaching "
             "the wire. Costs LUTs/BRAM and re-rolls placement, so it is off by "
             "default and its bitstream is a debug artefact, not a shipping one.")
    parser.add_argument("--sys-clk-freq", default=50e6, type=float, help="System clock frequency.")
    parser.add_argument("--firmware",     default=None,
        help="Image to embed in the BRAM ROM (replaces the LiteX BIOS). In the "
             "default split-image mode this is the LOADER (firmware/loader/"
             "loader.bin); with --bake-firmware it is the full firmware.")
    parser.add_argument("--bake-firmware", action="store_true",
        help="LEGACY single-image map: 96 KB ROM with the full firmware baked in, "
             "no coderam. Every firmware change then re-rolls P&R placement (the "
             "seed-roulette tax Phase 0.5 exists to remove), so this is only for "
             "shipping a standalone bitstream or for recovering if the loader "
             "boot path breaks.")
    builder_args = parser.add_argument_group("builder")
    builder_args.add_argument("--output-dir", default=None, help="Output directory.")
    args = parser.parse_args()

    soc_kwargs = dict(
        sys_clk_freq             = int(args.sys_clk_freq),
        cpu_type                 = "vexriscv",
        cpu_variant              = "minimal",
        # 64 KB integrated SRAM holds .data/.bss/stack. The default 8 KB
        # is too small once AAF per-channel buffers (4 KB total) are in
        # play. Use SRAM rather than a separate main_ram region so all
        # firmware data lives in the proven-bootable scratchpad.
        integrated_sram_size     = 0x10000,  # 64 KB SRAM (RW data + stack)
        # ROM size depends on the image map (Dante Phase 0.5):
        #   split (default)   -> 12 KB, holds only the frozen loader
        #   --bake-firmware   -> 96 KB, holds the whole firmware (legacy)
        #
        # 12 KB not 8: the loader measures 7068 bytes, which is 86% of 8 KB and
        # leaves no room to fix a bug in an artifact that is supposed to be
        # frozen. It will also plausibly grow a flash-WRITE path so netload can
        # persist an image in one step. 12 KB costs ~1 extra RAMB36.
        #
        # In split mode firmware .text/.rodata live in the 96 KB coderam added
        # in AVBSoC.__init__, so BRAM for CPU memory goes 96 -> 108 KB
        # (+12 KB, ~3 RAMB36 of the ~17 free). Milestone 1 needs no more BRAM.
        integrated_rom_size      = 0x18000 if args.bake_firmware else 0x3000,
        uart_name                = "serial",
        # 1 Mbaud + 64-byte HW FIFO. 115200/16 was the LiteX default and
        # at that rate periodic prints (gPTP dump + SRP poll + AVDECC etc)
        # stacked up faster than the main loop could drain the 128-byte
        # software ring, so each `s` command blocked for tens of ms. CH347
        # accepts up to ~9 Mbps over USB; 1 Mbaud is the sweet spot — 8.7×
        # less per-char wait, comfortable margin, every stty/picocom build
        # supports it. The bigger FIFO (16 → 64) also lets a full periodic
        # print fit without ever blocking the main loop.
        uart_baudrate            = 1_000_000,
        uart_fifo_depth          = 64,     # KEEP 64: bumping to 1024 shifted
                                           # placement and broke the marginal ULPI
                                           # HS-chirp (no USB enum). The verbose
                                           # gating (g_verbose default 0) removes
                                           # the print flood, so 64 is plenty.
    )
    soc_kwargs["bake_firmware"] = args.bake_firmware
    soc_kwargs["litescope"] = args.litescope

    if args.firmware:
        # Pre-load the image ourselves so SoCCore doesn't shrink the ROM region
        # down to the image size (which causes regions.ld to report a tiny ROM
        # on the next software-only build, masking real headroom).
        # Passing a list keeps the configured integrated_rom_size intact.
        from litex.soc.integration.common import get_mem_data
        rom_init = get_mem_data(args.firmware, endianness="little", data_width=32)
        rom_bytes = len(rom_init) * 4
        # MUST match integrated_rom_size in soc_kwargs below. Kept as a single
        # expression rather than a repeated literal: these drifted apart once
        # (check said 8 KB while the ROM was 12 KB), which would have falsely
        # rejected any loader over 8192 bytes.
        rom_size  = soc_kwargs["integrated_rom_size"]
        if rom_bytes > rom_size:
            raise SystemExit(
                "ROM image {} is {} bytes but the ROM is only {} bytes.\n"
                "  split mode expects the LOADER here (firmware/loader/loader.bin),\n"
                "  not the full firmware. For a baked full-firmware build pass\n"
                "  --bake-firmware.".format(args.firmware, rom_bytes, rom_size))
        print("[soc] ROM image {}: {} / {} bytes ({:.0f}% of {} KB {})".format(
            args.firmware, rom_bytes, rom_size, 100.0 * rom_bytes / rom_size,
            rom_size // 1024, "baked firmware" if args.bake_firmware else "loader"))
        soc_kwargs["integrated_rom_init"] = rom_init

    soc = AVBSoC(**soc_kwargs)

    builder_kwargs = {}
    if args.output_dir:
        builder_kwargs["output_dir"] = args.output_dir

    # In split-image mode the ROM is 12 KB and holds the loader, so the ~33 KB
    # LiteX BIOS cannot fit and must not be built. Builder skips it when the ROM
    # is already initialized (builder.py:396-401), which is automatic once
    # --firmware supplies an image. Assert it explicitly for the --soft-only
    # case, which supplies no image but only wants the generated headers.
    if not args.bake_firmware and not args.firmware:
        soc.integrated_rom_initialized = True

    builder = Builder(soc, **builder_kwargs)
    if args.soft_only:
        # Generate software headers (csr.h etc.) WITHOUT running P&R, so firmware
        # can be compiled against new CSRs before the (slow) gateware build.
        builder.build(run=False)
        return
    if args.build:
        # nextpnr-xilinx seed pinned to 4. Patch #3 (LITEETH_PATCHES.md,
        # TX-only sys-datapath) made eth_tx_clk robust across seeds (163 MHz
        # at seed 4 — well above the 125 MHz RGMII requirement), so the seed
        # is no longer a knife-edge timing knob — it's just pinned for build
        # reproducibility.
        #
        # USB proximity floorplan (ON by default since 2026-05-28). The
        # floorplan_usb.py hook constrains USB cells to X<=30, Y=10-70 —
        # immediately around the ULPI pins at X=1, Y=23-49. Without it the
        # post-CRF-extractor / post-MCRI2STx builds scatter USB cells far
        # enough from the pins that 60 MHz HS chirp fails (enumerate as
        # full-speed + error -71). Patch #3 (LITEETH_PATCHES.md) made
        # eth_tx robust without needing a floorplan, and the eth TX cluster
        # naturally lands at X=51-62 so it never collides with USB's X<=30.
        if not args.no_floorplan:
            fp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "floorplan_usb.py")
            soc.platform.toolchain._pnr_opts += " --pre-place {} ".format(fp)

        # Escape hatch: PLACER=sa forces nextpnr's simulated-annealing placer
        # instead of the default HeAP. HeAP's conjugate-gradient QP solve does
        # not converge on the dual-USB (USB2=1) netlist -- it spins the first
        # main-placer iteration indefinitely. SA always terminates (lower Fmax
        # but the design only needs ~50 MHz sys), so it's the reliable path to a
        # flashable dual-USB bitstream for hardware bring-up.
        _placer = os.environ.get("PLACER", "").strip()
        if _placer:
            soc.platform.toolchain._pnr_opts += " --placer {} ".format(_placer)

        # CLOCK CONSTRAINTS — the openxc7/yosys_nextpnr toolchain does NOT emit
        # usable create_clock for the PLL-derived clocks (it uses get_ports,
        # which matches only I/O pins) and emits NO clock groups. So nextpnr
        # times every PLL clock at the global `--freq 125` default and times
        # every sys<->usb<->audio CDC crossing as a real single-cycle path —
        # which made sys_clk placement-fragile (46-58 MHz, FAIL <50) and
        # corrupted audio bits on unlucky builds. Inject real per-clock periods
        # on the PLL output NETS + declare the async domains as a clock group so
        # the (properly synchronised) CDC crossings are NOT timed. See memory
        # openxc7-missing-pll-clock-constraints.
        # PHASE-2 dual-USB: the Backup ULPI runs on a 3rd PLL (cd_usb2 =
        # crg_s7pll2_clkout_buf, 60 MHz). WITHOUT a create_clock + a group
        # entry for it, nextpnr timed usb2 at the 125 MHz default AND timed every
        # usb2<->sys CDC (block_level MultiReg, A/B sample mux, pop routing) as a
        # REAL single-cycle path -> huge phantom negative slack -> the timing-
        # driven HeAP analytic placer spun forever without a single iteration
        # (the "stuck placer" was THIS, not cell density). Only inject it when the
        # 2nd ULPI is actually built, else get_nets matches nothing and naming an
        # undefined usb2_clk in set_clock_groups would abort the whole block.
        _usb2_xdc = getattr(soc.platform, "_has_ulpi2", False)
        _usb2_create = (
            ["create_clock -name usb2_clk   -period 16.667 "
             "[get_nets avbsoc_crg_s7pll2_clkout_buf]"]        # 60 MHz Backup ULPI
            if _usb2_xdc else [])
        _usb2_group = " -group {usb2_clk}" if _usb2_xdc else ""
        soc.platform.toolchain.additional_xdc_commands += _usb2_create + [
            # NOTE: the PLL output nets carry the CRG submodule prefix
            # `avbsoc_crg_…`. Without it get_nets matched NOTHING, so BOTH the
            # create_clock and the set_clock_groups below were silently dropped —
            # nextpnr then timed sys at the default 125 MHz (and timed every async
            # CDC crossing as a real path) → the placement-fragile 39-57 MHz Fmax.
            # audio_clk dropped from the group: the AAF bridge has no separate
            # audio clock domain (audio is the sys-domain MCR strobe), so naming
            # an undefined clock would make the now-live set_clock_groups fail.
            # sys is OVER-CONSTRAINED to 8 ns (125 MHz target), NOT its real
            # 20 ns / 50 MHz: binding sys at its true 50 MHz made nextpnr ease off
            # the moment it got "close" (~35-45 MHz); an aggressive target makes
            # the placer grind sys far harder, landing ~52 MHz — comfortably past
            # the real 50 MHz the audio path needs. The PLL still clocks sys at 50;
            # this only changes nextpnr's optimization pressure. Read the reported
            # Fmax number (the "FAIL at 125" is expected/ignored). This reproduces
            # the lean shift-fix build's 52.74 MHz, which had sys timed at 125.
            "create_clock -name sys_clk    -period  8.000 [get_nets avbsoc_crg_s7pll0_clkout_buf0]",  # target 125 MHz (real 50)
            "create_clock -name usb_clk    -period 16.667 [get_nets avbsoc_crg_s7pll1_clkout_buf]",   # 60 MHz
            "create_clock -name idelay_clk -period  5.000 [get_nets avbsoc_crg_s7pll0_clkout_buf1]",  # 200 MHz
            "set_clock_groups -asynchronous "
            "-group {sys_clk} -group {usb_clk} -group {idelay_clk}" + _usb2_group,
        ]
        builder.build(seed=args.seed)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
