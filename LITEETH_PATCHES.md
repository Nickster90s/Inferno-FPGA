# Required liteeth patches for this project

liteeth is patched in-place at /home/lisp/litex/liteeth/ (not vendored).
Re-apply these after any liteeth reinstall.

## 1. ptp.py — TSU addend multiplier (pre-existing)
Shift-add tree instead of `*` so nextpnr-xilinx can route the 52x30
multiply (the natural form infers an unrouteable DSP48 cascade). See
memory liteeth-tsu-dsp-workaround.

## 2. mac/core.py — TX skid buffer before the CDC (2026-05-26, P3.2)
Inserted `stream.Buffer(eth_phy_description(datapath_dw),
pipe_valid=True, pipe_ready=True)` in cd_tx before the with_sys_datapath
domain switch. Pipelines the CRC→CDC handshake. Backup: core.py.nen-bak.

RESULT: eth_tx_clk 112 -> 114 MHz with the USB block present — helps but
NOT enough (need 125 for gigabit RGMII). The TX valid/ready handshake is
a multi-stage combinational mesh (padding<->crc<->preamble<->last_be) and
the failure is routing/congestion-dominated: the USB UAC2 block scatters
the TX-datapath cells across the die (critical-path coords bounce
35,46 <-> 30,49 <-> 43,39). One buffer breaks one link only.

## 3. mac/core.py — TX-only sys-datapath (2026-05-26, P3.2) ★ THE FIX
The floorplan/seed route below only reached a fragile 114–134 MHz (seed +
netlist lottery; see floorplan_usb.py). ROOT CAUSE: the chip is only ~14%
full — eth_tx was a routing-LOCALITY problem, not congestion. The TX
datapath (preamble→CRC→tx_cdc) ran in the 125 MHz eth_tx domain.

FIX: added `with_sys_datapath_tx = True` right after the cd_tx/cd_rx block,
moving ONLY the TX datapath into sys clock (cd_tx="sys",
tx_datapath_dw=core_dw). CRC/preamble/padding now run 32-bit at 50 MHz;
then skid-buffer + CDC + 32→8 convert, so eth_tx carries only the
converter/last_be/gap. RX is left bit-identical (cd_rx, datapath_dw
unchanged) so the gPTP RX-timestamp ring is untouched — deliberately chosen
over add_ethernet(data_width=32), which would move RX too. Edits:
`datapath_dw`→`tx_datapath_dw` in TX add_padding/add_crc/add_preamble + the
skid Buffer; the two TX domain-switch `if`s use `with_sys_datapath_tx`.

RESULT: eth_tx_clk 114 -> **163 MHz** (eth_rx 138, audio 134) — all PASS at
125, ROBUST across seeds. Verified on FPGA (seed 4): loads, USB enumerates
(1209:eab1), AVB firmware boots (gPTP/SRP/AVDECC). On-wire TX + Hive entity
to be re-confirmed when next cabled to the AVB network.

The floorplan (#below) and skid buffer (#2) are kept (harmless, marginal)
but are NO LONGER the mechanism — this patch is. See memory
feedback_eth_tx_sys_datapath.

## 4. mac/__init__.py — deferred wishbone TX connect (2026-05-30, #67)
The wishbone-mode MAC hard-wired `interface.source.connect(core.sink)` in
`__init__`, leaving no seam to insert a second TX talker. Moved that one
connect into a new `do_finalize()`, guarded by `self.tx_wired`:

```python
# in __init__, wishbone branch — replaces the direct TX connect:
self.comb += self.core.source.connect(self.interface.sink)
self._wishbone_tx = True
def do_finalize(self):
    if getattr(self, "_wishbone_tx", False) and not getattr(self, "tx_wired", False):
        self.comb += self.interface.source.connect(self.core.sink)
```

Backward compatible: if nobody sets `mac.tx_wired`, finalize wires the
default direct path exactly as before. avb_soc.py sets `mac.tx_wired = True`
and drives `core.sink` through a frame-atomic `TXFrameArbiter`
(firmware SRAM reader + gateware AAF packetizer) — see `aaf_packetizer.py`.
This is the injection seam for the gateware USB→AAF talker (#67).

## 5. mac/sram.py — `discard_in` on the SRAM writer (pre-existing, undocumented)
`LiteEthMACSRAMWriter` gained a user-driven `discard_in` input (sram.py:32).
When it is high on the cycle of `sink.last`, the FSM goes to DISCARD instead
of TERMINATE, so the frame:

  * never pushes the status FIFO -> no `ev_pending`, CPU never sees it;
  * never advances the slot pointer -> **never consumes an RX slot**;
  * never reaches `stat_fifo.sink.valid` -> the `rx_ts` ring in avb_soc.py is
    not pushed either, so the RX-timestamp ring stays in lock-step for free.

The frame's bytes still land in the current (uncommitted) slot RAM and are
overwritten by the next frame. Added originally for the AVB `AVTPSampleExtractor`
(now parked in `_avb_reference/`); it was left in place and undocumented after
Dante Phase 0. `rx_gate.py` uses it — see patch #6.

## 6. mac/\_\_init\_\_.py — hybrid block restored to `__init__` (2026-08-01)
Patch #4 moved the wishbone TX connect into `do_finalize()` and accidentally
swept the **Hybrid Mode** block along with it, nested inside the
`not tx_wired` guard. That was broken two ways at once:

  * hybrid mode never sets `_wishbone_tx` (it is set only in the `"wishbone"`
    branch), so the block was unreachable;
  * it referenced `interface`, `dw` and `hw_mac` — `__init__` parameters, not in
    scope inside `do_finalize` — so it would have raised `NameError` if reached.

Moved back into `__init__` where it belongs. The TX seam is wishbone-only by
construction: in hybrid mode `LiteEthMACCoreCrossbar` owns `core.sink` through
its own TX arbiter FSM. Backup: `__init__.py.nen-bak`.

**No `rx_wired` seam was needed.** The plan (risk 8) assumed `rx_gate` would have
to sit *in* the RX stream between `core.source` and `interface.sink`, mirroring
`tx_wired`. It does not: patch #5's `discard_in` already drops a frame at the
point that matters — before it takes one of the 2 RX slots — so `rx_gate.py` is a
pure **observer** on `core.source` driving one wire into the writer. The RX
datapath is not re-plumbed, which also means the RX-timestamp ring and the
`eth_rx` timing closure are untouched. One less patch to a vendored dependency.

## (historical) NEXT LEVER notes — superseded by patch #3
region-constrain the USB block via nextpnr --pre-place (floorplan_usb.py);
recovered 114 MHz only. Kept for context.
