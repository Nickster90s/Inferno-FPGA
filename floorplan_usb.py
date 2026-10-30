#!/usr/bin/env python3
# nextpnr-xilinx --pre-place floorplan for the USB+AVB combined build.
# Goal: recover eth_tx_clk >= 125 MHz (gigabit RGMII) with the USB block present.
# See docs/phase3-bridge.md (P3.2).
#
# Background (measured, not guessed)
# ----------------------------------
# nextpnr grid: X 0..114, Y 0..156. ALL package I/O is on the LEFT edge
# (PAD bels only at grid X=0/1): eth TX pins X1,Y51-54; eth RX X1,Y55-72;
# ULPI X1,Y23-49.
#
# v1 (USB-only region, X>=60) made eth_tx WORSE (112 -> 104 MHz). The
# post-route critical path proved why: it is ENTIRELY the LiteEth TX
# datapath
#     preamble_inserter -> tx_crc_pipe -> txlastbe -> tx_cdc graycounter
#     -> storage_2 (the TX CDC FIFO)
# at 1.4 ns logic / 8.2 ns ROUTING. That cluster placed at X51..62, Y44..84,
# and the v1 USB boundary at X60 sliced right through it (a cluster cell sat
# at X62, inside the USB zone) — USB crowded the exact columns eth needed.
#
# v2 strategy (this file)
# -----------------------
# 1. Compact the eth TX datapath into a tight box (ETH_*), shrinking its
#    routing-dominated internal nets (the worst is a 2.3 ns graycounter->FIFO
#    net spanning ~15 rows). This is the direct fix for the critical path.
# 2. Confine the USB block to the right edge (USB_*), well clear of the eth
#    box, so it can never crowd those columns again.
# 3. A keep-out gap is left between the two boxes.
#
# constrainCellToRegion only PULLS the named cells in; it never evicts others.
# IOBs are not constrained (they keep their pinned LOC on the left edge); we
# only move logic. Placement/timing failures are loud — this cannot silently
# break the AVB stack.
#
# Tunable via env (comma lists "x0,y0,x1,y1"): NEXTPNR_USB_REGION,
# NEXTPNR_ETH_REGION. Set NEXTPNR_ETH_REGION="" to disable the eth box.

import os

# ---- USB block: LEFT edge, close to the ULPI pins at X=1, Y=23-49 ----------
# (Opposite of the original v2 region which pushed USB to X>=78.) Rationale:
# patch #3 (TX-only sys-datapath) made gigabit eth_tx robust WITHOUT a
# floorplan, so the v2 "USB to the right half" recipe is no longer needed —
# AND that distance from the ULPI pins re-introduced placement-marginal HS
# ULPI sampling once the CRF extractor + MCRI2STx were added (USB enumerated
# only as full-speed, error -71 during HS chirp).
# Goal here: keep USB cells within ~40 columns of X=1 so the 60 MHz ULPI
# input setup time is satisfied regardless of synthesis non-determinism.
# Eth TX cluster (X=51-62) is well clear of X<=40.
# ---- MASTER SWITCH: NEXTPNR_USB_FP=0 schakelt BEIDE USB-boxen uit ---------
#
# GEMETEN 2026-10-30, beslissend: dezelfde dual-USB netlist die ZEVEN keer in
# HeAP stalde met een floorplan, plaatst EN routeert compleet ZONDER floorplan:
#   Routing complete, 0 errors; sys 59.59 MHz, usb 86.09, usb2 87.90 MHz.
# Dus niet de BRAM (59/75 na ringhalvering, meer vrij dan de werkende single
# build) en niet de regio-geometrie (zes varianten, incl. nul overlap) -- het
# IS deze floorplan. USB1's box werkt voor EEN subsysteem en wordt onplaatsbaar
# zodra de cellen van het tweede bestaan: de net-connectivity _pull-pass sleept
# cellen die via de A/B-mux, sample_strobe en de block_level-CDC aan BEIDE
# subsystemen hangen in USB1's box, terwijl ze bij USB2's pads horen.
#
# LET OP WAAROM DEZE BOX BESTAAT (zie de comment boven USB_REGION): zonder hem
# werd HS ULPI-sampling placement-marginaal -- USB enumereerde alleen als
# full-speed, error -71 tijdens de HS-chirp. De statische timing is nu goed
# (usb 86 MHz, usb2 88 MHz, ver boven 60), maar of de echte HS-chirp het haalt
# is een HARDWARE-vraag, niet een timing-vraag. Test dat na het flashen.
#
# Als HS faalt is de juiste oplossing NIET deze box terugzetten, maar twee
# APARTE boxen rond de werkelijke pad-coordinaten van elke PHY -- op te meten
# met floorplan_probe.py, niet te raden uit pin-letters (dat heb ik gedaan en
# het was fout).
_USB_FP_ON = os.environ.get("NEXTPNR_USB_FP", "1") != "0"

USB_REGION = "usb_fp"
USB_PREFIX = "usb_avb_subsystem"          # clean + $flatten\ -escaped names
UX0, UY0, UX1, UY1 = 0, 0, 45, 156
_u = os.environ.get("NEXTPNR_USB_REGION", "").strip()
if _u:
    UX0, UY0, UX1, UY1 = (int(v) for v in _u.split(","))

# PHASE-2 dual-USB: if a 2nd subsystem (usb_avb_subsystem2) is present in the
# netlist, split the left edge — USB1 lower half, USB2 upper half — so each
# ULPI's cells cluster near their (bank-34, left-edge) pins and the placer
# doesn't scatter 2x USB logic across the die (which stalls the analytic placer).
# NOTE 'usb_avb_subsystem' is a substring of 'usb_avb_subsystem2', so USB1's
# matches below must EXCLUDE USB2_PREFIX.
USB2_REGION = "usb2_fp"
USB2_PREFIX = "usb_avb_subsystem2"
_HAS_USB2 = any(USB2_PREFIX in c for c, _ in ctx.cells)
# 2026-10-29 REWORK, ported from avb-aes3 (which hit this first and fixed it
# on 2026-09-09). The OLD scheme below split the left edge into
# USB1=lower(Y0-76) / USB2=upper(Y80-156), on the assumption that the two ULPIs'
# pins sit in different Y halves. THEY DO NOT -- both PHYs land on the far-left
# IO column at LOW Y. Constraining USB2's logic to Y80-156 therefore drags it
# far from its own pads, the HeAP analytic placer cannot satisfy the region
# against the IO anchor, and it STALLS: "Running main analytical placer" with
# zero iterations, forever.
#
# MEASURED HERE 2026-10-29, before the fix: seeds 3, 1 and 5 ALL stalled at
# exactly that line and were killed by build.sh's 300 s watchdog. Three seeds in
# a row is not seed luck -- the region was unplaceable by construction.
#
# An X-split cannot help either, since both PHYs share the same far-left IO
# column. FIX: give BOTH subsystems ONE SHARED tall left region and let the
# placer plus the seed cluster each PHY's cells near its own pins. The USB cell
# count roughly doubles (2994 + 2946 here) and fits easily in a 48x156 column;
# X is widened from 45 to 48 for the larger count.
#
# A Y-split is only valid if the two pad bands are genuinely disjoint, which is
# a property of the BOARD, not of the design. Left available but OFF by default
# via NEXTPNR_USB_SPLIT=1, because the shared column is the configuration that
# actually builds. Explicit boxes still override via
# NEXTPNR_USB_REGION / NEXTPNR_USB2_REGION.
_split = os.environ.get("NEXTPNR_USB_SPLIT", "") == "1"
if _HAS_USB2 and not _u:
    UX1, UY1 = 48, 156                        # USB1 -> tall left column (avb-aes3's bewezen breedte)
U2X0, U2Y0, U2X1, U2Y1 = 0, 0, 48, 156        # USB2 -> zelfde kolom, maar wordt niet gevuld (zie _u2_on)
# ---- GEMETEN PAD-COORDINATEN, 2026-10-30 ---------------------------------
# Uit een GEROUTEERD ontwerp (NEXTPNR_BEL in de routed json), niet geraden:
#
#   MAIN   ULPI  X1, Y23..49   clock Y23; data Y35-45; ctrl Y38-49
#   BACKUP ULPI  X1, Y1..32    data Y1-32; clock Y26; ctrl Y22-28
#
# LET OP: gemeten VOOR de BOARD=breakout pinout-port van 2026-10-30. Main is
# ongewijzigd (zelfde pins, alleen D[7:0] geroteerd -> zelfde pads, andere
# orde, dus zelfde band). Backup deelt nog vier pads (Y9 V8 W9 V9) maar ruilde
# de andere vier en alle ctrl-pins; alles blijft bank 34, dus de Y1-32 band is
# ONGEVEER nog geldig -- niet opnieuw gemeten. Hermeet met floorplan_probe.py
# voordat je een stall aan deze getallen toeschrijft.
#
# Beide op X=1 (dezelfde linker IO-kolom), maar de Y-banden zijn grotendeels
# GESCHEIDEN: Main hoog, Backup laag. Dat maakt een Y-SPLIT hier wel geldig --
# en het verklaart waarom het oude schema (USB2 -> Y80..156) stalde: de
# Backup-pads liggen op Y1-32, dus die box zat 50-120 rijen van zijn anker.
#
# avb-aes3's comment voorspelde dit bord bijna exact: "USB1 data/ctrl pads IOB
# Y35-Y49, USB2 Y1-Y13 ... so a Y-split finally has a basis". Onze meting geeft
# Main Y35-49 (identiek) en Backup Y1-32 (ruimer dan Y1-13, zelfde idee).
#
# Klok-pads ankeren de placement NIET: een ULPI-klok bereikt zijn logica via een
# BUFG, die het hele device overspant. Alleen data/ctrl-pads trekken.
_split = _split or os.environ.get("NEXTPNR_USB_SPLIT", "") == "1"
if _HAS_USB2 and _split and not _u:
    UX0, UY0, UX1, UY1     = 0, 28, 55, 156   # USB1 -> boven, dekt Y35-49
    U2X0, U2Y0, U2X1, U2Y1 = 0,  0, 55,  60   # USB2 -> onder, dekt Y1-32 ruim
_u2 = os.environ.get("NEXTPNR_USB2_REGION", "").strip()
if _u2:
    U2X0, U2Y0, U2X1, U2Y1 = (int(v) for v in _u2.split(","))

# ---- BOTH ULPIs ARE ON THE SAME SIDE: ONE SHARED, WIDE LEFT COLUMN -------
#
# CORRECTION (2026-10-29): an earlier version of this comment claimed the two
# PHYs sit on OPPOSITE edges and defaulted the USB2 region off on that basis.
# That was inferred from pin letters and it is WRONG. The actual assignments in
# avb_soc.py ulpi_io()/ulpi2_io():
#
# Pin-lijsten staan hier NIET meer: ze stonden er wel, en ze waren op
# 2026-10-30 een maand verouderd (ze noemden de Backup-klok V4 -- een pin die
# NC moet blijven). Lees ze uit de bron:
#   BOARD=breakout python3 -c 'import avb_soc; print(avb_soc._ULPI_PINS)'
#
# Beide buses liggen in bank 34, aangrenzende rijen, lage kolomnummers -- de
# ZELFDE IO-regio. Dus avb-aes3's shared-column schema is juist, en wat hier
# fout zat was de BREEDTE.
#
# avb-aes3: "5752 USB cells fit easily in the 56x156 left column. Widened X to
# 55 for the 2x cell count." This netlist has 2989 + 2945 = 5934 USB cells, MORE
# than that, and was being squeezed into X 0..48. Four placer stalls followed.
#
# STALL HISTORY -- SIX ATTEMPTS, ALL STALLED. Nothing below is speculation;
# every line is a build that spun at "Running main analytical placer" with zero
# iterations until the watchdog killed it. Do not re-try these:
#   1. Y-split  0,80..45,156  seed 3  -- USB2 dragged off its pads
#   2. Y-split  0,80..45,156  seed 1  -- same
#   3. Y-split  0,80..45,156  seed 5  -- same (3 seeds => the region, not the seed)
#   4. shared   0,0..48,156   seed 3  -- column too narrow for 5932 cells
#   5. USB2 off, USB1 0..48   seed 3  -- USB1 alone still cramped
#   6. shared   0,0..55,156   seed 3  -- overlapped cfgflash's X50..114
#   7. shared   0,0..58,156   seed 3  -- cfgflash moved to X60; NO overlap; STILL STALLS
#
#   8. USB2-pull UIT, USB1 0..48 (2972 cellen, = avb-aes3's werkende config),
#      seed 8 -- stall. Zie de correctie hieronder.
#   9. ZELFDE config, P&R-only sweep over seeds 1,2,3,4,5,6,7,9,10,11,12 --
#      ALLE ELF stallen, geen enkele rondt een HeAP-iteratie af, geen enkele
#      geeft een plaatsingsfout. De seed is daarmee UITGESLOTEN; zoek daar niet
#      meer. (Die sweep draaide nextpnr direct op de bestaande json: de seed
#      raakt alleen P&R, dus yosys per seed herhalen is ~2,5 min weggegooid
#      werk per poging. 11 seeds in een kwartier in plaats van een uur.)
#
# WAT WEL WERKT (2026-10-30): NEXTPNR_USB_FP_LIGHT=1 -- 697 cellen in plaats
# van 2972, alleen de naam-gematchte cellen plus de nets die de ULPI-PADS
# raken. HeAP 25,57 s, gerouteerd, sys 62,70 MHz. De volle doos trok via
# net-connectivity ook de hele LUNA-kern mee, die geen pad-timingrelatie heeft.
#
# Attempt 7 is the important one: regions correct, no overlap, 58 columns for
# 5929 cells, 1200 s watchdog -- and HeAP still never completes an iteration.
#
# CORRECTIE 2026-10-30: ik concludeerde hieruit "dus de blokkade is NIET de
# regio-geometrie". Dat was fout, en het heeft de zoektocht maanden de verkeerde
# kant op gestuurd. Dezelfde regio's met PLACER=sa lopen 205 iteraties en geven
# dan WEL een diagnose:
#
#   ERROR: failed to place chain starting at cell '...$PACKED_CARRY4$'
#   ERROR: Placing design failed.
#
# Een CARRY4-keten moet op aaneengesloten verticale SLICEs in EEN kolom liggen
# en kan niet over een regiogrens worden gesplitst. De regio's zijn dus niet
# krap maar ONHAALBAAR: er bestaat geen legale plaatsing. HeAP meldt dat niet --
# hij blijft zijn QP-oplossing herhalen op een onoplosbaar probleem en lijkt te
# hangen. SA meldt het wel.
#
# LES: een placer die hangt is een placer zonder legale oplossing. Draai bij een
# stall EERST PLACER=sa om een echte foutmelding te krijgen, in plaats van aan
# de doosafmetingen te schuiven. Dat had negen builds gescheeld.
# De oude conclusie luidde: ulpi_io()
# already recorded that the dual-USB netlist "jams the placer", and six builds
# agree with it.
#
# WHAT TO TRY INSTEAD OF ANOTHER BOX (in order of cost):
#   * USB_BACKUP=1 -- single proven subsystem on the Backup pins, ~14k cells,
#     places in ~3 min. Validates the Backup PHY hardware with no dual netlist.
#   * tools/seed_sweep.sh with USB2=1 -- unattended; placement on this flow is
#     chaotic and a seed may yet land. If none does, that is an answer.
#   * floorplan_probe.py -- get the REAL X/Y of the ULPI pads instead of
#     inferring from pin letters (an earlier version of this comment inferred
#     "opposite edges" from the letters and was wrong).
#   * Shrink the Backup subsystem (fewer channels on USB2) so the netlist fits,
#     or build this configuration with Vivado rather than openXC7.
#
# NOTE ulpi_io() already records that the dual-USB netlist "jams the placer",
# which is why USB_BACKUP=1 exists: it puts the SINGLE proven subsystem on the
# Backup pins (~14k cells, places in ~3 min) to validate that hardware without
# the dual netlist. That is the fallback if this still will not place.
# ---- USB2's PULL STAAT UIT (NODIG, MAAR NIET VOLDOENDE) --------------------
#
# WAARSCHUWING, GEMETEN 2026-10-30: ik schreef hier eerst dat dit DE oplossing
# voor de stalls was. Dat is weerlegd in dezelfde sessie. Met USB2's pull uit
# trekt deze floorplan 2972 cellen in X 0..48 -- vrijwel exact avb-aes3's
# werkende 3014 cellen in DEZELFDE doos -- en dante-usb stalt alsnog (907 s,
# HeAP rondt geen iteratie af) terwijl avb-aes3 in 39,95 s plaatst.
#
# Dus de pull-breedte en de geometrie zijn NIET het resterende verschil. Wat
# wel verschilt is de netlist:
#
#                  dante-usb      avb-aes3
#   RAMB36E1       65/75  86%     38/75  50%
#   RAMB18E1        5/150          47/150
#   SLICE_LUTX     15520          17613
#
# dante-usb gebruikt 86% van de 36 Kb-blokken (de Dante-ringen, 6 x 2048). Die
# kolommen staan vast, dus er is nauwelijks plaatsingsvrijheid over en daar
# vecht een regio-constraint tegen. Zoek daar, niet in de doosafmetingen.
# De volgende stap is BRAM-druk verlagen (ringdiepte of de USB2-bridge-FIFO),
# niet opnieuw een rechthoek verschuiven.
#
# De pull staat hieronder nog steeds uit, want minder cellen constrainen is
# hoe dan ook juister en de Backup-PHY heeft de box bewijsbaar niet nodig (hij
# haalde 480 Mbit/s zonder enige floorplan). Maar het lost de stall niet op.
#
# avb-aes3's reden om hem uit te zetten, van 2026-09-09:
#
#   "constraining BOTH USB subsystems (~5750 cells) into the left region
#    stiffened HeAP's analytic solve so much it HUNG (never returned from the
#    initial solve; 0 main-placer iters). Proven by: --no-floorplan cleared HeAP
#    in 50s, and the single-USB build (~3000 constrained cells) never stalled.
#    So pull ONLY USB1 (Main) by default -> Main keeps its proven HS
#    floorplan/timing, USB2 (Backup) floats."
#
# Het is dus niet de GEOMETRIE maar de STIJFHEID: 5929 cellen in regio's duwen
# maakt de analytische oplossing onoplosbaar (SA bewijst dat met een echte
# fout: "failed to place chain ... PACKED_CARRY4"). Vrij zwevende cellen laten
# HeAP ongemoeid -- de no-floorplan build plaatste in 24,6 s.
#
# En het hoeft ook niet: de Backup-PHY enumereerde op 2026-10-30 op High Speed
# (480 Mbit/s) in een build ZONDER enige floorplan. Hij heeft de proximity-
# constraint niet nodig; Main wel, want zonder floorplan haalt Main alleen een
# full-speed connect met error -71 -- exact het symptoom waarvoor de floorplan
# in mei werd toegevoegd.
#
# Dus: Main geboxt, Backup zwevend. Zet NEXTPNR_USB2_BOX=1 alleen als de
# Backup-HS ooit tuning nodig heeft, en verwacht dan de stall terug.
_u2_on = _USB_FP_ON and os.environ.get("NEXTPNR_USB2_BOX", "") == "1"

# ---- eth TX datapath: compact box near the TX pins (X1,Y51) ----------------
# Matches the cells seen on the critical path. storage_2 is the TX CDC FIFO
# RAM (anchored forms 'storage_2.' / 'storage_2$' / 'storage_2/' avoid
# matching storage_20.. ). All are LiteEth TX-clock-domain logic.
ETH_REGION   = "eth_tx_fp"
ETH_SUBSTR   = ("txdatapath", "tx_crc_pipe", "tx_cdc")
ETH_FIFO_PFX = ("storage_2.", "storage_2$", "storage_2/", "\\storage_2$")
EX0, EY0, EX1, EY1 = 38, 44, 66, 88   # only used if explicitly enabled
# eth box OFF by default: it is HARMFUL — it hung the placer for 90 min (v2,
# too few SLICEM cols for the storage_2 LUTRAM) and, when it did complete,
# dragged eth_tx_clk to 94 MHz. The winning config is USB-floorplan-only +
# a good seed. Enable only for experiments: NEXTPNR_ETH_REGION="38,44,66,88".
_e = os.environ.get("NEXTPNR_ETH_REGION", "").strip()
_eth_on = _e != ""
if _eth_on:
    EX0, EY0, EX1, EY1 = (int(v) for v in _e.split(","))


def _matches_eth(name):
    if any(s in name for s in ETH_SUBSTR):
        return True
    return any(p in name for p in ETH_FIFO_PFX)


if _USB_FP_ON:
    ctx.createRectangularRegion(USB_REGION, UX0, UY0, UX1, UY1)
if _HAS_USB2:
    if _u2_on:
        ctx.createRectangularRegion(USB2_REGION, U2X0, U2Y0, U2X1, U2Y1)
if _eth_on:
    ctx.createRectangularRegion(ETH_REGION, EX0, EY0, EX1, EY1)

# ---------------------------------------------------------------------------
# NET-CONNECTIVITY matching (the fix, 2026-05-28).
#
# Matching cells by name (`USB_PREFIX in cname`) catches only ~2% of the
# USB subsystem (388 / 18019 cells). yosys+ABC rename the bulk of the
# combinational logic to anonymous forms like `$auto$alumacc.cc:...` and
# `$abc$...$lut$aiger...` that DON'T carry the `usb_avb_subsystem`
# hierarchy — so the floorplan was leaving ~95% of USB cells (incl. the
# cd_usb-critical get_descriptor compare) to float wherever the placer
# liked. That's why USB enumeration was a per-build lottery and why the
# get_descriptor path spilled to X74 and dropped cd_usb to 54.5 MHz.
#
# But the NETS keep their hierarchy names (1584 / 22860 match the
# prefix). And `net.driver.cell` + `net.users[i].cell` reach the actual
# cells — including the anonymous `$auto$`/`$abc aiger` ones connected to
# USB nets. So: walk every usb-named net, pull its driver + all user
# cells into the region. This catches the floating soup by connectivity
# instead of by name.
#
# IOBs / already-located cells can't be region-constrained; constrain in
# a try/except and skip failures (the ULPI pins keep their pinned LOC).
# ---------------------------------------------------------------------------
_constrained = set()

# Only region-constrain FABRIC LOGIC. BRAM / DSP / IOB / clock primitives
# live in fixed device columns; pinning them into a narrow rectangle makes
# the analytic placer thrash trying to legalise (the same 90-min hang the
# eth-box LUTRAM caused). The cd_usb critical path is the get_descriptor
# LUT/carry compare — fabric logic — so constraining LUT/FF/CARRY is what
# matters; the 3 USB BRAMs + 11 ULPI IOBs float to their natural columns.
_CONSTRAIN_TYPES = ("SLICE_LUTX", "SLICE_FFX", "CARRY4", "SELMUX2")
def _constrainable(cell):
    t = getattr(cell, "type", "") or ""
    return any(t.startswith(p) for p in _CONSTRAIN_TYPES)

def _pull(cell, region):
    """Constrain a fabric-logic cell into a region; dedup + skip the rest."""
    if cell is None or not _constrainable(cell):
        return 0
    nm = cell.name
    if nm in _constrained:
        return 0
    try:
        ctx.constrainCellToRegion(nm, region)
        _constrained.add(nm)
        return 1
    except Exception:
        return 0   # already-located / packed — leave it be

nu = ne = 0

# Pass 1 — direct name match (fast path for the cells that DO carry the
# prefix: named FFs, ROM-derived ABC luts).
for cname, cell in ctx.cells:
    if USB_PREFIX in cname and USB2_PREFIX not in cname:   # USB1 only (excl USB2)
        nu += _pull(cell, USB_REGION) if _USB_FP_ON else 0
    elif _eth_on and _matches_eth(cname):
        if cname not in _constrained:
            try:
                ctx.constrainCellToRegion(cname, ETH_REGION)
                _constrained.add(cname)
                ne += 1
            except Exception:
                pass

# ---- LICHTE MODUS (NEXTPNR_USB_FP_LIGHT=1) ---------------------------------
# Pass 2 hieronder trekt driver+users van ELK net met de subsysteem-prefix, en
# dat blaast 162 naam-gematchte cellen op naar ~3000: yosys+ABC hernoemen de
# bulk naar $abc$...$lut$aiger..., dus de net-pass is de enige manier om die
# te vinden -- maar hij pakt daarmee ook de hele LUNA-kern, die GEEN
# pad-timingrelatie heeft.
#
# Waar de floorplan in mei voor is toegevoegd, is de IOB->eerste-flop
# vertraging op de 60 MHz source-synchrone bus. Dat zijn de nets die de ULPI
# PADS raken, niet de descriptor-ROM diep in het subsysteem. Deze modus
# constraint daarom alleen die: nets met "ulpi" in de naam (de pad-kant) plus
# de naam-gematchte cellen. Een paar honderd cellen in plaats van 3000.
#
# Waarom dit het proberen waard is: 11 seeds x HeAP met de volle doos stallen
# allemaal zonder ooit een iteratie af te ronden, en SA geeft op dezelfde doos
# een echte "failed to place chain ... PACKED_CARRY4". Een strikt ZWAKKERE
# constraint is de enige richting die nog niet gemeten is.
_light = os.environ.get("NEXTPNR_USB_FP_LIGHT", "") == "1"

# Pass 2 — net connectivity: pull driver + users of every usb-named net.
# This is what catches the anonymous combinational soup.
nnets = 0
for nname, net in ctx.nets:
    if _light:
        # Alleen de pad-kant: nets die de ULPI-pinnen raken.
        if "ulpi" not in nname.lower() or "ulpi2" in nname.lower():
            continue
    elif USB_PREFIX not in nname or USB2_PREFIX in nname:  # USB1 only (excl USB2)
        continue
    nnets += 1
    drv = getattr(net, "driver", None)
    if drv is not None:
        nu += _pull(getattr(drv, "cell", None), USB_REGION) if _USB_FP_ON else 0
    for u in getattr(net, "users", []):
        nu += _pull(getattr(u, "cell", None), USB_REGION) if _USB_FP_ON else 0

# ---- Pass 1+2 for USB2 (Backup), pulled into its own upper-half region --------
nu2 = 0
if _HAS_USB2:
    for cname, cell in ctx.cells:
        if USB2_PREFIX in cname:
            nu2 += _pull(cell, USB2_REGION) if _u2_on else 0
    for nname, net in ctx.nets:
        if USB2_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            nu2 += _pull(getattr(drv, "cell", None), USB2_REGION) if _u2_on else 0
        for u in getattr(net, "users", []):
            nu2 += _pull(getattr(u, "cell", None), USB2_REGION) if _u2_on else 0
    print("[floorplan] USB2 region %d,%d..%d,%d  cells=%d"
          % (U2X0, U2Y0, U2X1, U2Y1, nu2))

# ---- cfgflash SPIMaster: BIG region right of the USB box --------------------
# Keeps the SPIMaster cells out of the marginal USB zone (X<=45) AND gives the
# better-Fmax placements (61-64 MHz vs 54 with it floating). Big region (not the
# tight X95-114 box that thrashed) legalises fast.
CFG_REGION = "cfgflash_fp"
CFG_PREFIX = "cfgflash"
# X0 MOVED 50 -> 60 WHEN USB2 IS PRESENT. This box holds THIRTY-EIGHT cells in
# 65 columns, so it loses nothing by starting further right -- while the USB
# column needs every column it can get once there are TWO subsystems in it
# (5932 cells vs 2995 single). The old 50 overlapped a widened USB box, which
# breaks this region's own stated invariant ("right of the USB box").
# USB1 blijft nu op X<=48 (we boxen Backup niet meer), dus cfgflash kan terug
# naar avb-aes3's 50 -- de 60 was er alleen om een 58 brede USB-kolom te ontwijken.
_cf_x0 = 50
ctx.createRectangularRegion(CFG_REGION, _cf_x0, 0, 114, 156)
ncf = 0
for cname, cell in ctx.cells:
    if CFG_PREFIX in cname:                  # NAMED SPIMaster cells ONLY — pulling
        ncf += _pull(cell, CFG_REGION)       # net-DRIVERS too dragged long-net cells
                                             # and thrashed the placer; named is local.
print("[floorplan_usb] cfgflash: %d cells -> %s (X %d..114)" % (ncf, CFG_REGION, _cf_x0))

# ---- AAF packetizer: compact box in the clear right-center -----------------
# The sys-domain critical paths (pres-time t_next_value adder, ring rd/level,
# byte-build FSM) are ALL internal to the AAF packetizer and routing-dominated
# — left to float they scatter across the die and cap sys at ~40-45 MHz. Boxing
# them tight keeps those internal nets short so sys closes 50 deterministically,
# independent of the placer's seed. Clear of USB (X<=45) and the eth-TX pins.
# Same net-connectivity capture as USB (the bulk are anonymous $abc/$auto cells
# that only the NETS carry the 'aafpacketizer' hierarchy for). Tunable via
# NEXTPNR_AAF_REGION="x0,y0,x1,y1"; set "" to disable.
AAF_REGION = "aaf_fp"
AAF_PREFIX = "aafpacketizer"
AX0, AY0, AX1, AY1 = 72, 28, 104, 96
_a = os.environ.get("NEXTPNR_AAF_REGION", "").strip()   # default OFF: boxing the
na = 0
if _a != "":
    if "," in _a:
        AX0, AY0, AX1, AY1 = (int(v) for v in _a.split(","))
    ctx.createRectangularRegion(AAF_REGION, AX0, AY0, AX1, AY1)
    # SELECTIVE capture: unlike USB (self-contained), the AAF packetizer's nets
    # fan out to the MAC / CSR bus / MCR, so pulling net *users* dragged ~66% of
    # the design into the box (11896 cells — unplaceable). Pull only (a) cells
    # NAMED aafpacketizer and (b) the DRIVER of each aafpacketizer net (the cell
    # that PRODUCES the signal — i.e. AAF-internal logic, incl. the anonymous
    # $abc luts that drive AAF nets). Skip users (the external readers).
    for cname, cell in ctx.cells:
        if AAF_PREFIX in cname:
            na += _pull(cell, AAF_REGION)
    for nname, net in ctx.nets:
        if AAF_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            na += _pull(getattr(drv, "cell", None), AAF_REGION)
    print("[floorplan_usb] AAF: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (na, AAF_REGION, AX0, AX1, AY0, AY1))

# ---- VexRiscv CPU: compact box (Phase-1 timing, 2026-07-08) -----------------
# After registering the CSR bridge (avb_soc.py add_csr_bridge override), the
# sys_clk critical path moved OFF the CSR decode ONTO the VexRiscv CPU's own
# datapath: dBus_cmd -> maccmap multiplier -> ALU -> IBusSimplePlugin_pending,
# ROUTING-dominated (12.2ns route / 2.8ns logic), sprawling X56-67 Y12-51. The
# CPU is the only big sys-domain block with no region, so it scatters and its
# internal nets run 1.5-1.8ns each. Box it compact to shorten them. The path's
# nets ALL carry the 'VexRiscv' hierarchy, so named-cell + net-DRIVER capture
# (drivers only — pulling users would drag every peripheral the CPU talks to)
# catches the anonymous $abc/maccmap/alumacc soup on the path. Plain LUT/FF/
# CARRY (no LUTRAM) so it won't hit the eth-box legalisation hang. Clear of USB
# (X<=45). Tunable/disable via NEXTPNR_CPU_REGION="" ; default ON.
CPU_REGION = "cpu_fp"
CPU_PREFIX = "VexRiscv"
CX0, CY0, CX1, CY1 = 46, 6, 78, 104
# DEFAULT OFF (2026-07-08): this box THRASHED the analytical placer (>160s, no
# routing). Root: the CPU dBus/iBus tie to the SRAM+ROM BRAMs (16+43 blocks) in
# fixed device columns; constraining the CPU LUT/FF/CARRY to X46-78 while its
# BRAMs sit elsewhere makes legalisation fight. FOLLOW-UP to make it work: size
# the region to span the BRAM columns the CPU uses (or leave BRAMs out and widen
# Y so the CPU logic hugs its own BRAM rows). Enable to experiment:
# NEXTPNR_CPU_REGION="46,6,78,104" (or tuned coords).
_cpu = os.environ.get("NEXTPNR_CPU_REGION", "").strip()
ncpu = 0
if _cpu != "":
    if "," in _cpu:
        CX0, CY0, CX1, CY1 = (int(v) for v in _cpu.split(","))
    ctx.createRectangularRegion(CPU_REGION, CX0, CY0, CX1, CY1)
    for cname, cell in ctx.cells:
        if CPU_PREFIX in cname:
            ncpu += _pull(cell, CPU_REGION)
    for nname, net in ctx.nets:
        if CPU_PREFIX not in nname:
            continue
        drv = getattr(net, "driver", None)
        if drv is not None:
            ncpu += _pull(getattr(drv, "cell", None), CPU_REGION)
    print("[floorplan_usb] CPU: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ncpu, CPU_REGION, CX0, CX1, CY0, CY1))

print("[floorplan_usb] USB: %d cells (via %d nets + name match) -> %s (X %d..%d, Y %d..%d)"
      % (nu, nnets, USB_REGION, UX0, UX1, UY0, UY1))
if _eth_on:
    print("[floorplan_usb] eth TX: %d cells -> %s (X %d..%d, Y %d..%d)"
          % (ne, ETH_REGION, EX0, EX1, EY0, EY1))
if nu == 0:
    print("[floorplan_usb] WARNING: 0 USB cells matched — prefix changed? no-op.")
