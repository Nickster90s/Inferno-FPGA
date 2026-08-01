# Replacing `mcr` — design for a Dante media clock

Written at the end of a session that hit three separate clock failures, all
centred on `mcr`. The conclusion, which the operator reached first: `mcr` is AVB
media-clock-recovery machinery and Dante does not need most of it.

## What mcr is, and why it fights us

`mcr` was built to recover a media clock from an **AVB CRF stream**. It carries:

- CRF stream binding (`crf_stream_id`, `crf_dmac`, talker EID), persisted in config
- a CRF-loss **watchdog** that snaps `current_increment` back to `base_increment`
- its own `servo_locked` state machine and a CRF PI servo
- a *secondary* path that derives the NCO from the gPTP addend ratio

**Dante has no CRF.** Nothing binds a stream, nothing feeds the CRF servo, the
watchdog can only ever fire in the "lost" direction, and the gPTP path is gated
on `gptp_t.servo_locked` — a flag set only by `gptp_servo_update()`, i.e. the
802.1AS servo, which this device no longer runs.

Net effect today: `mcr_compute_gptp_base()` always returns the undisciplined
`base_increment`. The media clock free-runs at the crystal rate.

## Measured evidence (this session)

| state | drift | underrun |
|---|---|---|
| undisciplined (current) | +4.34 ppm, +15.6 ms/hour | ~130 total, not growing |
| mcr pointed at `g_ptpv1` | **+0.17 ppm**, +0.61 ms/hour | 40547 in 4 min, then a 4.19M burst |
| following servo integral only | — | same underrun storm |
| firmware NCO trim on top | timestamp fell **5.3 s** behind in 115 s | — |

The drift fix works. Something about changing the NCO increment at runtime
starves the ring — and `fifo_level` reads centre (65-67) throughout, which no
one has explained. The 5.3 s divergence is three orders of magnitude beyond the
±50 ppm the trim was clamped to, so it is not an arithmetic error; it is the
same fault at its most extreme.

**Start the rewrite from that contradiction, not from the drift.**

## What a Dante media clock actually needs

One thing: an NCO whose rate is slaved to the PTPv1 rate estimate.

    increment = base_increment * (1 + rate_error)

where `rate_error` comes from the PTPv1 servo. `g_ptpv1` already exposes
`locked`, `base_addend_full` and `current_addend_full`; the ratio of the last two
IS the rate correction, in exactly the form needed.

Deliberately absent: CRF binding, CRF servo, the loss watchdog, `servo_locked`,
the config fields, the `cs` media-clock-source selector.

## Prime suspects for the underrun

1. **The watchdog.** `watchdog_reset_active` snaps the increment back to
   `base_increment`. If it fires while the gPTP path is also writing, the NCO
   oscillates between two rates — which would starve a ring while the *average*
   level still looks correct. This fits the `fifo_level`-at-centre contradiction
   better than anything else and should be checked first.
2. **CSR write path.** `mcr_increment_write()` is called from several places
   (`mcr.c:53, 388, 442`). Concurrent writers with different sources would
   produce exactly this.
3. **Deadband interaction.** The gPTP-base deadband suppresses rewrites; combined
   with a watchdog reset the effective rate could latch to the wrong value.

## Suggested order

1. Instrument before changing: log every `mcr_increment_write()` with its value
   and caller for 30 s with the clock disciplined. If the value oscillates,
   suspect 1 or 2 is confirmed and the rewrite is straightforward.
2. Write `mcr_dante.c`: NCO slaved to `g_ptpv1`, nothing else. ~50-80 lines.
3. Keep `mcr.c` in `_avb_reference/` — it is still the reference for the NCO
   arithmetic and the USB feedback servo scaling.
4. Verify in this order: `underrun` delta over 10 min (must stay 0), then drift
   rate over 1 h (target < 0.5 ppm), then an overnight run.

## Also still AVB-shaped, lower priority

- `config.h` carries `cs`, `crf_valid`, `crf_stream_id`, `crf_dmac`,
  `crf_talker_eid` — all dead under Dante.
- `gptp.c` remains as the TSU accessor and servo library; `ptpv1.c` reuses parts.
  That layering is fine, but `gptp_t.servo_locked` being read by `mcr` while only
  the 802.1AS servo sets it is exactly the trap that caused the undisciplined
  clock. Any remaining cross-module reads of gPTP state deserve the same audit.
