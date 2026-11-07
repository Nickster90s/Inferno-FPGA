# 18 uur ononderbroken, seed 8, 96 kHz -- gemeten 2026-11-07 10:10

Bitstream: dual-usb-96k_2026-11-06_peakmeter_s8 (48-bits ringpointers).
mpv op beide kaarten, gestart 2026-11-06 ~16:07, nog steeds spelend.

De oude 2^32-wrap bij 8 ch x 96 kHz valt elke 1,553 h, dus 18,07 h kruist hem
ruim ELF keer. De 3,107 h-markering van de oorspronkelijke wedge elke keer.

## Twee samples, 20 s apart
```
  talker           1
  packets          1567163421
  underrun         126
  enables          1
  disables         0
  ptp_locked       1
  offset_ns        -12608
  flows_active     2
  f0_age_ms        65301471
  f1_age_ms        65298123
  (t=10:14:38)
  talker           1
  packets          1567644306
  underrun         126
  enables          1
  disables         0
  ptp_locked       0
  offset_ns        -11942
  flows_active     2
  f0_age_ms        65321507
  f1_age_ms        65318160
  (t=10:14:58)
```

underrun staat STIL op 126 en disables op 0 over die 20 s en over de hele run.

## Wat hier NIET in staat, en waarom

De ring-health CSR's ('r'-opcode) zijn voor deze run NIET te citeren. De
firmware die op het bord liep was gebouwd tegen build/colorlight_i9plus,
waarvan csr.h van 4 november is en de zeven nieuwe CSR's mist. Daardoor viel
dstats.c:287 terug op de #else en bestond de opcode niet -- en dante_info.c:308
ook, dus de peakmeter meldde 18 uur lang 255 (stilte) op alle 48 kanalen
terwijl er muziek door liep. Gemeten op de heartbeat-multicast, niet aangenomen.

Het bewijs hierboven komt dus uit tellers die al voor die build bestonden.
firmware/Makefile noemt sinds nu bij elke build welke csr.h hij gebruikt en
welke #ifdef-features die header heeft uitgezet.

## Wel gezien, niet onderzocht: ptp_locked wipt

In de twee samples hierboven gaat ptp_locked van 1 naar 0 binnen 20 s, bij een
ruwe offset van -12,6 us en -11,9 us. Dat is GEEN klokfout-meting: offset_ns is
het ruwe sample, niet de servo-uitvoer (zie de kop van tools/stats.py).

Dit is wel dezelfde waarneming die DC's eigen log maakt --
STATIC_ADDRESSING_HANDOFF.md noemt N-Series-Switchover die herhaald sync
verliest en terugkrijgt, 2026-11-06 18:07-18:43 en 2026-11-07 09:41-09:45. Twee
onafhankelijke bronnen op hetzelfde verschijnsel.

Audio bleef hier 18 uur doorlopen, dus het kost geen samples, maar het hoort
niet. Voor de echte klokfout is tools/telemetry.py de bron (raw EN filtered per
servo-update), niet deze teller. Niet verder onderzocht.
