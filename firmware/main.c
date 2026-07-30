// AVB-AES3 Firmware — Main entry point
// gPTP clock sync + AVTP audio streaming
// Runs as BIOS replacement on LiteX SoC (VexRiscv)

#include <stdio.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <system.h>
#include <libliteeth/mdio.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "gptp.h"
#include "cfgflash.h"
#include "config.h"
#include "cap.h"
#include "avtp_const.h"
#include "mcr.h"
#include "pkt_geom.h"
#include "net.h"
#include "dante_dev.h"
#include "mdns.h"
#include "dante_arc.h"
#include "dante_cmc.h"
#include "dante_info.h"

// MAC address — locally administered, unique per device.
// TODO: read from SPI flash or EEPROM in production.
static const uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x42};

static gptp_t gptp;
static mcr_state_t    mcr;

// 6x8ch time-mux talker: 6 streams feeding the gateware packetizer contexts.
// Under AVB these carried AVTP stream identities (stream_id + SR-class
// multicast dst_mac) shared with AVDECC and SRP. Under Dante the same 6x8
// structure survives -- Dante's MAX_CHANNELS_IN_FLOW is also 8 -- but each
// stream becomes a multicast BUNDLE addressed by IP, so Phase 5's dante_tx.c
// replaces these tables with {dst_ip, dst_mac (derived), ip_csum, udp ports}.
#define N_AAF_STREAMS     6

// RX EtherType counters for link-debug
static uint32_t rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other;
static uint32_t rx_filtered;         // frames early-dropped by the MAC allow-list
static uint16_t rx_last_ethertype;
static uint8_t  rx_last_dst[6];
static uint8_t  rx_last_src[6];
uint8_t         g_verbose = 0;       // 0 = quiet console (default); 1 = gPTP debug spam ('v' toggles)
static uint8_t  aaf_gw_enabled;      // 1 = gateware packetizer owns the USB stream
static void     aaf_gw_set(uint8_t on);          // defined below check_uart_cmd
static uint32_t usb_lock_calls;      // diag: USB-FIFO servo invocations
static uint8_t  usb_nco_freeze;      // diag: hold NCO at base (test implicit feedback)
static uint32_t usb_fb_manual;       // 0 = auto .v loop; nonzero = held fb_ovr (FBSWEEP 'F')
// SRC src_step PI servo gains — RUNTIME-TUNABLE over the console ('k'/'j') so
// the loop can be tuned live with no 20-min rebuild. KI=0 -> pure proportional.
static int32_t  g_src_kp = 16384;    // proportional: step units per frame of level error
static int32_t  g_src_ki = 1;        // integral: step units per (accumulated frame-error)
static int32_t  g_src_integ;         // integral accumulator

// Benchmark / rate-window state for the 'b' UART command. Lets us
// compare Stage-0 (firmware-on-audio) numbers against each later
// migration stage to prove the gateware work paid off.
typedef struct {
    uint32_t window_start_ms;
    // Snapshots taken at window start.
    uint32_t s_writer_errors;
    uint32_t s_sync_rx;
    uint32_t s_pdresp_rx;
    // Main-loop iteration timing (in TSU ns; resolution = ns).
    uint32_t last_iter_ns;
    uint32_t max_iter_ns;
    uint64_t sum_iter_ns;
    uint32_t iter_count;
    // Last computed rates (per-second).
    uint32_t r_writer_errors;
    uint32_t r_sync_rx;
    uint32_t r_pdresp_rx;
    uint32_t r_iter_per_sec;
    uint32_t r_iter_avg_ns;
    uint32_t r_iter_max_ns;
} bench_t;
static bench_t bench;

// ---------------------------------------------------------------------------
// Central Ethernet RX dispatcher
// ---------------------------------------------------------------------------

#define ETHMAC_EV_SRAM_WRITER 0x1
#define PTP_ETHERTYPE   0x88F7
// AVTP_ETHERTYPE is 0x22F0, defined in avtp.h

static void dispatch_rx(void)
{
    // Drain pending RX slots in one dispatcher call (bounded). nrxslots=2
    // is pinned (>2 silently breaks TX — see avb_soc.py:537); under MSRP /
    // AVDECC / CRF bursts the writer overruns within microseconds if we
    // service only one slot per call. But we MUST cap the drain — an
    // unbounded loop locks the CPU when wire-rate stream traffic exceeds
    // processing rate, never returning to the main loop. Symptom: UART
    // stops, Hive loses entity, gPTP servo can't update. 16 frames is
    // roughly one Class A burst window; main_loop drives dispatch_rx
    // continuously so any residual frames are picked up next call.
    // Drain budget. 16 was OK with nrxslots=2 in light traffic, but with
    // AAF + CRF + AVDECC + gPTP arriving simultaneously the 2-slot FIFO
    // overruns producing 1000+ writer_errors/sec under Hive refresh
    // (READ_DESCRIPTOR burst overlapping AAF flow). 64 lets one
    // dispatch_rx call empty an entire bridge burst window without
    // returning to main_loop where slower paths (printf, MCR servo,
    // descriptor builds) could let more frames pile up.
    int drain_budget = 64;
    while ((ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER)
           && (drain_budget-- > 0)) {

    uint32_t slot = ethmac_sram_writer_slot_read();
    uint8_t *slot_ptr = (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * slot);
    uint32_t len = ethmac_sram_writer_length_read();

    // ---- EARLY DROP: peek the destination MAC before doing any real work ----
    //
    // MEASURED on the bench with real Dante hardware: 3588 pps on the wire, of
    // which 17794/17941 = 99.2% is multicast audio on port 4321 destined for
    // OTHER devices. The switch floods it to every port, so we receive all of
    // it. Classifying that in software costs a ~435-byte Wishbone memcpy per
    // frame and starves everything else -- ICMP replies were being lost at up
    // to 46%.
    //
    // This is the failure mode the plan flagged as risk #8 ("CPU swamped by
    // flooded Dante multicast"), and Phase 0 made it worse by removing the old
    // AVTP peek-before-memcpy path in favour of always copying.
    //
    // Six byte reads from slot SRAM instead of a 435-byte copy: accept our own
    // unicast, broadcast, and only the multicast groups we actually use. Note
    // this is a MAC-level allow-list, deliberately NOT a protocol parser -- the
    // gateware rx_gate.py in the plan is the same idea one level down, and is
    // still the right answer if software proves insufficient.
    {
        uint8_t d0 = slot_ptr[0];
        if (d0 & 0x01) {                         // any group address
            uint8_t d1 = slot_ptr[1], d2 = slot_ptr[2];
            uint8_t d3 = slot_ptr[3], d4 = slot_ptr[4], d5 = slot_ptr[5];
            int keep = 0;
            if (d0 == 0xFF && d1 == 0xFF && d2 == 0xFF) {
                keep = 1;                        // broadcast (ARP)
            } else if (d0 == 0x01 && d1 == 0x00 && d2 == 0x5E) {
                // IPv4 multicast; the low 23 bits carry the group.
                //
                // Accept ALL of 224.0.0.0/24 (MAC 01:00:5e:00:00:xx), the
                // link-local control scope. That is mDNS (.251), Dante
                // device-info (.231) and heartbeat (.233) -- and critically
                // IGMP queries, which arrive on all-hosts 224.0.0.1 and on the
                // group address for group-specific queries.
                //
                // An earlier version enumerated only the three Dante groups and
                // silently swallowed IGMP queries. Nothing breaks immediately,
                // which is what makes it nasty: memberships just age out and
                // the switch quietly stops forwarding our groups.
                //
                // The whole /24 is link-local control traffic and low-rate, so
                // there is no reason to be more selective. What we are excluding
                // is the audio range (239.x), which is the actual flood.
                if (d3 == 0x00 && d4 == 0x00)
                    keep = 1;                    // 224.0.0.0/24 link-local
                else if (d3 == 0x00 && d4 == 0x01 && d5 == 0x81)
                    keep = 1;                    // 224.0.1.129 PTPv1
            } else if (d0 == 0x01 && d1 == 0x80 && d2 == 0xC2) {
                keep = 1;                        // 802.1 reserved (gPTP)
            }
            if (!keep) {
                rx_filtered++;
                main_rx_ts_pop_write(1);         // keep the ts ring in lock-step
                ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
                continue;
            }
        }
    }

    // Record every RX control frame into the boot capture ring (RAM only, no
    // printf — must NOT slow this drain path). See 'R' command.
    cap_record(0, slot_ptr, len);

    // Advance the RX-timestamp ring in lock-step with slot consumption.
    main_rx_ts_pop_write(1);

    // VLAN-strip into a scratch buffer.
    //
    // DANTE PHASE 0: the AVTP fast path is gone. It existed to avoid a
    // ~30 us/frame Wishbone memcpy on 9000 fps of inbound AAF+CRF audio by
    // handing handlers a VLAN-shifted pointer straight into slot SRAM.
    // Milestone 1 is TX-only, so we receive NO audio: everything left
    // (PTP, ARP, IPv4/UDP control plane) is low-rate and needs the memcpy
    // anyway, because handlers read the src MAC at frame[6..11] for reply
    // routing and a shifted pointer would put the VLAN tag there instead.
    //
    // Dante also does not VLAN-tag by default (it uses DSCP in the IP TOS
    // byte), but keep the strip: switches may add tags, and gPTP peers on a
    // shared bench network still do.
    uint32_t v = 0;
    if (len >= 18) {
        uint16_t et_pre = ((uint16_t)slot_ptr[12] << 8) | slot_ptr[13];
        if (et_pre == 0x8100) v = 4;
    }

    const uint8_t *frame;
    {
        static uint8_t scratch[1600];
        if (len > sizeof(scratch)) len = sizeof(scratch);
        memcpy(scratch, slot_ptr, len);
        if (v) {
            memmove(scratch + 12, scratch + 16, len - 16);
            len -= 4;
        }
        frame = scratch;
    }

    if (len >= 14) {
        uint16_t ethertype = ((uint16_t)frame[12] << 8) | frame[13];

        rx_total++;
        rx_last_ethertype = ethertype;
        // Read MAC fields from slot_ptr (NOT frame) — frame may be the
        // VLAN-shifted slot pointer where [0..5]/[6..11] are inside the
        // tag, not the real MAC fields.
        memcpy(rx_last_dst, slot_ptr, 6);
        memcpy(rx_last_src, slot_ptr + 6, 6);

        // DANTE PHASE 0: AVTP (0x22F0) and MSRP (0x22EA) branches removed.
        // AVTP carried AAF audio, CRF media clock and AVDECC control; MSRP
        // carried the SRP bandwidth reservations. Dante uses none of them --
        // its entire control plane and audio transport are IPv4/UDP, which is
        // why Phase 2 (net.c) is the gate for everything that follows.
        //
        // rx_avtp / rx_msrp counters are retained and should stay at ~0 on a
        // Dante network. If they climb, something is still speaking AVB at us
        // and it is worth knowing.
        switch (ethertype) {
            case PTP_ETHERTYPE:
                // 802.1AS gPTP (L2). Kept working through Phase 0-3 as the
                // reference servo; Phase 4 adds ptpv1.c (UDP 319/320) and
                // refactors the shared PI servo out into ptp_servo.c. Dante
                // needs PTPv1, so this branch eventually goes quiet -- but
                // keeping it lets us A/B the servo against a known-good
                // 802.1AS master while PTPv1 is brought up.
                rx_ptp++;
                gptp_process_rx(&gptp, frame, len);
                break;
            case ARP_ETHERTYPE:      // 0x0806 — ARP
            case IPV4_ETHERTYPE:     // 0x0800 — IPv4/UDP/ICMP/IGMP
                net_rx_frame(frame, len);
                break;
            default:
                rx_other++;
                break;
        }
    }

    // Acknowledge RX event — releases this slot back to the MAC and
    // advances slot_read to the next pending frame (if any). The
    // while-loop re-checks ev_pending to drain the second buffered slot
    // in the same call.
    ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
    }
}

// ---------------------------------------------------------------------------
// UART debug commands
// ---------------------------------------------------------------------------

// Bench tick — called once per main-loop iteration. Tracks max/avg
// iteration time (in TSU ns) and rolls a 1-second rate window for
// the counters that matter when comparing this Stage-0 firmware-on-
// audio baseline against later gateware-offload stages.
static void bench_tick(void)
{
    uint32_t now_ns = tsu_nanoseconds_read();
    if (bench.last_iter_ns) {
        // Modular arithmetic: TSU ns wraps at 1e9, so a single iter
        // crossing the second boundary shows as a huge negative dt.
        // Cap to 0 in that case (this iter "took no time" by this
        // measure — acceptable for the bench).
        int32_t dt = (int32_t)(now_ns - bench.last_iter_ns);
        if (dt < 0) dt += 1000000000;
        if ((uint32_t)dt > bench.max_iter_ns) bench.max_iter_ns = dt;
        bench.sum_iter_ns += (uint32_t)dt;
        bench.iter_count++;
    }
    bench.last_iter_ns = now_ns;

    uint32_t now_ms = gptp_uptime_ms();
    uint32_t elapsed = now_ms - bench.window_start_ms;
    if (elapsed > 2000000000u) elapsed = 0;   // wrap guard
    if (elapsed < 1000) return;

    bench.r_writer_errors   = ethmac_sram_writer_errors_read() - bench.s_writer_errors;
    bench.r_sync_rx         = gptp.rx_sync_count - bench.s_sync_rx;
    bench.r_pdresp_rx       = gptp.rx_pdelay_resp_count - bench.s_pdresp_rx;
    bench.r_iter_per_sec    = bench.iter_count;
    bench.r_iter_max_ns     = bench.max_iter_ns;
    bench.r_iter_avg_ns     = bench.iter_count
                                ? (uint32_t)(bench.sum_iter_ns / bench.iter_count) : 0;

    // Re-snapshot for next window.
    bench.s_writer_errors  = ethmac_sram_writer_errors_read();
    bench.s_sync_rx        = gptp.rx_sync_count;
    bench.s_pdresp_rx      = gptp.rx_pdelay_resp_count;
    bench.window_start_ms  = now_ms;
    bench.iter_count       = 0;
    bench.sum_iter_ns      = 0;
    bench.max_iter_ns      = 0;
}

static void check_uart_cmd(void)
{
    if (!readchar_nonblock())
        return;

    char c = getchar();
    switch (c) {
        case 's': {
            ptp_timestamp_t t = gptp_read_time();
            printf("\n[gPTP] state=%d time=%llu.%09lu\n",
                   gptp.state,
                   (unsigned long long)t.seconds,
                   (unsigned long)t.nanoseconds);
            printf("  sync=%lu pdelay=%lu offset=%lld ns delay=%lld ns\n",
                   (unsigned long)gptp.sync_count,
                   (unsigned long)gptp.pdelay_count,
                   (long long)gptp.offset_from_master_ns,
                   (long long)gptp.mean_path_delay_ns);
            printf("  addend=%lu/%lu locked=%d\n",
                   (unsigned long)(gptp.current_addend_full >> 20),
                   (unsigned long)(gptp.current_addend_full & 0xFFFFFu),
                   gptp.servo_locked);
            printf("  30s avg: off=%lld ns |off|=%lld ns (n=%lu)\n",
                   (long long)gptp.off_avg_ns_last,
                   (long long)gptp.off_abs_avg_ns_last,
                   (unsigned long)gptp.off_avg_count_last);
            printf("  nrr=%ld ppb pdelay_outliers=%lu pairs=%lu\n",
                   (long)gptp.nrr_ppb,
                   (unsigned long)gptp.pdelay_outlier_count,
                   (unsigned long)gptp.pdelay_pair_count);
            printf("  gm: id=%02x%02x%02x%02x%02x%02x%02x%02x p1=%u cc=%u p2=%u valid=%u\n",
                   gptp.gm_clock_id[0], gptp.gm_clock_id[1], gptp.gm_clock_id[2], gptp.gm_clock_id[3],
                   gptp.gm_clock_id[4], gptp.gm_clock_id[5], gptp.gm_clock_id[6], gptp.gm_clock_id[7],
                   gptp.gm_priority1, gptp.gm_clock_class, gptp.gm_priority2, gptp.gm_valid);
            printf("  rx: sync=%lu fup=%lu pdreq=%lu pdresp=%lu pdfup=%lu ann=%lu other=%lu wdom=%lu last=mt%u dom%u\n",
                   (unsigned long)gptp.rx_sync_count,
                   (unsigned long)gptp.rx_followup_count,
                   (unsigned long)gptp.rx_pdelay_req_count,
                   (unsigned long)gptp.rx_pdelay_resp_count,
                   (unsigned long)gptp.rx_pdelay_resp_fup_count,
                   (unsigned long)gptp.rx_announce_count,
                   (unsigned long)gptp.rx_other_count,
                   (unsigned long)gptp.rx_wrong_domain_count,
                   gptp.rx_last_msg_type, gptp.rx_last_domain);
            break;
        }
        case 'R':
            // Dump the on-FPGA control-plane capture ring: every control frame
            // RX'd + TX'd since boot. This is the point-to-point view ens5
            // cannot see, and it becomes MORE useful under Dante -- it shows
            // what the FPGA actually received when a switch is filtering.
            cap_dump();
            break;
        case 'z':
            // Clear + re-arm the capture ring. Press 'z', then do a MANUAL
            // re-patch on the MOTU, then 'R' — captures the post-boot re-patch
            // (the ring otherwise fills at boot and stops).
            cap_reset();
            printf("\n[CAP] cleared + re-armed. Do the manual re-patch now, then press R.\n\n");
            break;
        case 'u': {
            // Step the ULPI input IDELAY tap (0..31, wraps) and load it.
            // Sweep to centre 60 MHz ULPI sampling in the data eye: press
            // 'u', re-plug USB, check lsusb. The tap that enumerates
            // (1209:eab1 at HIGH speed) is the eye. Note the working tap
            // and pin it as the CSR reset in avb_soc.py for a one-build
            // deterministic result. CSRs are main_-prefixed (top-level).
            static uint8_t utap = 8;
            utap = (utap + 1) & 0x1F;
            main_ulpi_idelay_tap_write(utap);
            main_ulpi_idelay_load_write(1);
            printf("\n[ULPI] IDELAY tap = %u — re-plug USB + check lsusb\n",
                   (unsigned)utap);
            break;
        }
        case 'r':
            printf("\nRebooting...\n");
            ctrl_reset_write(1);
            break;
        case 'e':
            printf("\n[RX] total=%u ptp=%u avtp=%u msrp=%u other=%u\n",
                   rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other);
            printf("  (avtp/msrp should stay ~0 on a Dante network)\n");
            printf("  last et=%04x dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x\n",
                   rx_last_ethertype,
                   rx_last_dst[0], rx_last_dst[1], rx_last_dst[2],
                   rx_last_dst[3], rx_last_dst[4], rx_last_dst[5],
                   rx_last_src[0], rx_last_src[1], rx_last_src[2],
                   rx_last_src[3], rx_last_src[4], rx_last_src[5]);
            printf("  mac: writer_errors=%u preamble_err=%u crc_err=%u ev_en=%u ev_st=%u ev_pd=%u\n",
                   ethmac_sram_writer_errors_read(),
                   ethmac_rx_datapath_preamble_errors_read(),
                   ethmac_rx_datapath_crc_errors_read(),
                   ethmac_sram_writer_ev_enable_read(),
                   ethmac_sram_writer_ev_status_read(),
                   ethmac_sram_writer_ev_pending_read());
            printf("  rx_ts ring: commits=%lu level=%lu overflow=%lu\n",
                   (unsigned long)main_rx_ts_commit_count_read(),
                   (unsigned long)main_rx_ts_level_read(),
                   (unsigned long)main_rx_ts_overflow_count_read());
            {
                uint8_t hba = main_eth_rx_heartbeat_read();
                busy_wait(300);  // ≥1 top-byte tick at 125e6/2^24 ≈ 7.5 Hz
                uint8_t hbb = main_eth_rx_heartbeat_read();
                int d = (int)((uint8_t)(hbb - hba));
                printf("  eth_rx heartbeat (PHY1 L3): %u -> %u (delta=%d) %s\n",
                       hba, hbb, d, d ? "alive" : "*** DEAD ***");
            }
            break;
        case 'm':
            printf("\n[MCR] bound=%d locked=%d step=%lu rx=%lu seq_err=%lu other=%lu bad_type=%lu\n",
                   mcr.bound, mcr.servo_locked,
                   (unsigned long)mcr.servo_step_count,
                   (unsigned long)mcr.rx_count,
                   (unsigned long)mcr.seq_errors,
                   (unsigned long)mcr.rx_other_count,
                   (unsigned long)mcr.bad_type_count);
            printf("  base_freq=%lu ts_interval=%u ts/pdu=%u type=%u pull=%u\n",
                   (unsigned long)mcr.base_frequency,
                   mcr.timestamp_interval, mcr.timestamps_per_pdu,
                   mcr.type, mcr.pull);
            printf("  CRF-RATE(phc): cs=%u valid=%u warmup=%u/%u ppb=%ld last_err=%ld\n",
                   mcr.cs, mcr.crf_rate_valid, mcr.crf_meas_count, CRF_MEAS_SAMPLES,
                   (long)(int32_t)mcr.crf_ppb_filt, (long)(int32_t)mcr.crf_last_err_ppb);
            printf("  offset_ns=%08lx_%08lx integral=%08lx_%08lx\n",
                   (unsigned long)(uint32_t)(mcr.latest_offset_ns >> 32),
                   (unsigned long)(uint32_t)mcr.latest_offset_ns,
                   (unsigned long)(uint32_t)(mcr.servo_integral >> 32),
                   (unsigned long)(uint32_t)mcr.servo_integral);
            printf("  inc base=%08lx cur=%08lx hw_sample_count=%lu hw_phase=%08lx\n",
                   (unsigned long)mcr.base_increment,
                   (unsigned long)mcr.current_increment,
                   (unsigned long)mcr_sample_count_read(),
                   (unsigned long)mcr_phase_read());
            printf("  delta stats (n=%lu): max|d|=%ld ns avg|d|=%ld ns streak=%u outlier_rej=%lu\n",
                   (unsigned long)mcr.delta_window_count,
                   (long)mcr.delta_max_abs,
                   (long)(mcr.delta_window_count ? mcr.delta_sum_abs / mcr.delta_window_count : 0),
                   mcr.lock_streak,
                   (unsigned long)mcr.servo_outlier_rejects);
            // The CRF hardware-extractor stats are gone with the extractor
            // (Dante Phase 0). hw_rx_count stays as the firmware-side counter
            // and should remain 0: nothing feeds the CRF path any more.
            printf("  hw_rx=%lu (expect 0: CRF extractor removed)\n",
                   (unsigned long)mcr.hw_rx_count);
            // Reset window after print so next sample starts fresh
            mcr.delta_max_abs     = 0;
            mcr.delta_sum_abs     = 0;
            mcr.delta_window_count = 0;
            break;
        case 'a':
            /* Per-second RATES of the decisive counters, computed over the
             * interval since the last `a` press. strobe_rate = true NCO
             * consumer demand; usb_samp_rate/8 = true USB producer frame rate
             * (compare to usbmon ~46979); first_rate should match usb_samp/8 —
             * if first_rate is higher, `first` is glitching (phantom pushes). */
            {
            static uint32_t pr_ms, pr_rxb, pr_epo, pr_pkts;
            uint32_t cpkts = aaf_pkt_packet_count_read();    // AAF frames handed to MAC
            uint32_t crxb  = main_usb_dbg_rx_beats_read();   // core->EP raw byte beats
            uint32_t cepo  = main_usb_dbg_ep_out_read();     // EP->decoder beats
            uint32_t now = gptp_uptime_ms();
            uint32_t dms = now - pr_ms;
            if (dms == 0) dms = 1;
            // *** AAF TX FRAME RATE *** — should be ~8000/s (6 samples/pkt at
            // 48 kHz). AUTHORITATIVE "are we transmitting at rate": packet_count
            // increments only when the MAC ACCEPTS a frame's last beat = frames
            // on the wire. (The per-stage soft-ILA rates were removed with the
            // debug CSRs once the rate-match path was proven.)
            printf("  *** AAF TX = %lu pkt/s (expect ~48000 = 6 streams x 8000) ***\n",
                   (unsigned long)((uint64_t)(cpkts - pr_pkts) * 1000u / dms));
            // localisation: rx_beats = core->EP raw bytes/s (real RX rate; 48ch @
            // 48k x 4B = ~9.216M B/s clean); ep_out = EP->decoder bytes/s.
            printf("  rx-loc(/s): rx_beats=%lu ep_out=%lu  [clean 48ch ~9216000 B/s]\n",
                   (unsigned long)((uint64_t)(crxb - pr_rxb) * 1000u / dms),
                   (unsigned long)((uint64_t)(cepo - pr_epo) * 1000u / dms));
            pr_rxb = crxb; pr_epo = cepo; pr_ms = now; pr_pkts = cpkts;
            }
            uint32_t gw_pres = aaf_pkt_dbg_last_pres_read();
            uint32_t gw_gptp = aaf_pkt_dbg_emit_gptp_read();
            int32_t  eff_off = (int32_t)(gw_pres - gw_gptp);
            // Gateware packetizer + USB ingress health. The firmware-side AAF
            // software TX/RX counters are gone with aaf.c; what remains is the
            // authoritative view: what the gateware actually put on the wire
            // and how the USB ring is tracking. Phase 5 renames aaf_pkt_* to
            // dante_pkt_* and drops pres/gptp (Dante stamps sec+subsec instead).
            printf("\n[PKT] gw: en=%d pkts=%lu underrun=%lu ovr=%lu fifo=%lu\n"
                   "  usb: fifo_ovf=%lu level=%ld fbovr=0x%lx step=0x%lx inc=%lu calls=%lu\n"
                   "  pres(gw)=%08lx gptp@emit=%08lx eff_offset=%ld ns (expect ~%d)\n",
                   aaf_gw_enabled,
                   (unsigned long)aaf_pkt_packet_count_read(),
                   (unsigned long)aaf_pkt_underrun_count_read(),
                   (unsigned long)aaf_pkt_overrun_count_read(),
                   (unsigned long)aaf_pkt_fifo_level_read(),
                   (unsigned long)main_usb_sample_overflow_read(),
                   (long)mcr.usb_last_level,
                   (unsigned long)main_usb_fb_ovr_read(),
                   (unsigned long)aaf_pkt_src_step_read(),
                   (unsigned long)mcr.current_increment,
                   (unsigned long)usb_lock_calls,
                   (unsigned long)gw_pres,
                   (unsigned long)gw_gptp,
                   (long)eff_off,
                   AAF_PRESENTATION_OFFSET_NS);
            break;
        case 'f': {
            usb_nco_freeze = !usb_nco_freeze;
            printf("[USB] NCO freeze = %d (1=hold base 48k, 0=servo)\n", usb_nco_freeze);
            break;
        }
        case 'k': {
            // Cycle SRC servo KP (proportional). Live tuning, no rebuild.
            static const int32_t kps[] = {4096, 8192, 16384, 32768, 65536};
            static int ki_idx;
            ki_idx = (ki_idx + 1) % (int)(sizeof(kps)/sizeof(kps[0]));
            g_src_kp = kps[ki_idx]; g_src_integ = 0;
            printf("[SRC] KP = %ld (integ reset)\n", (long)g_src_kp);
            break;
        }
        case 'j': {
            // Cycle SRC servo KI (integral). KI=0 = pure proportional.
            static const int32_t kis[] = {0, 1, 2, 4, 8, 16};
            static int kj_idx;
            kj_idx = (kj_idx + 1) % (int)(sizeof(kis)/sizeof(kis[0]));
            g_src_ki = kis[kj_idx]; g_src_integ = 0;
            printf("[SRC] KI = %ld (integ reset)\n", (long)g_src_ki);
            break;
        }
        case 't':
            // Force-enable the gateware packetizer talker. Under AVB this also
            // bound a software AAF stream and opened an SRP reservation; both
            // are gone. What remains is exactly what Phase 0 needs: drain the
            // USB ring at media-clock rate and put frames on the wire, so the
            // async-feedback servo has a real consumer and USB can be verified.
            // Nothing on a Dante network will listen to these AVB-format
            // frames -- that is fine, Phase 5 changes the format.
            aaf_gw_set(1);
            printf("[DIAG] gateware talker ENABLED (AVB AAF format; Phase 5 -> Dante)\n");
            break;
        case 'T':
            aaf_gw_set(0);
            printf("[DIAG] gateware talker DISABLED\n");
            break;
        case 'F': {
            // USB feedback SWEEP: hold a FIXED async-feedback value (overriding
            // the .v auto loop) to characterise the host's delivery vs commanded
            // rate. Step the table; at each value watch [AAF] rx-loc ep_out and
            // aaf_pkt underrun. 0=auto. 0x60000=6.0 nominal=48000. The value where
            // ep_out hits ~9,216,000 B/s tells us: if that's 0x60000 the host
            // honors nominal -> our measured nco_rate reads LOW (measurement bug);
            // if it needs >0x60000 the host genuinely under-delivers.
            static const uint32_t fbtab[] = {
                0, 0x60000, 0x60800, 0x61000, 0x61800, 0x62000, 0x63000
            };
            static int fbi = 0;
            fbi = (fbi + 1) % (int)(sizeof(fbtab)/sizeof(fbtab[0]));
            usb_fb_manual = fbtab[fbi];
            if (usb_fb_manual == 0) {
                printf("[FBSWEEP] fb_ovr = AUTO (measured loop)\n");
            } else {
                long bp = ((long)usb_fb_manual - 0x60000) * 10000 / 0x60000; // x100 %
                printf("[FBSWEEP] fb_ovr = 0x%lx (%ld.%02ld%% vs 6.0) -- watch ep_out/underrun\n",
                       (unsigned long)usb_fb_manual, bp/100, (bp<0?-bp:bp)%100);
            }
            break;
        }
        case 'b':
            // Per-second windowed rates. The BENCHMARK_BASELINE.md thresholds
            // still apply and are the early-warning for plan risk 8 (CPU
            // swamped by RX flood): target >10000 iter/s and 0 writer_err/s.
            // If writer_err/s climbs or iter/s falls below ~2000 once Dante
            // multicast is on the wire, add the rx_gate.py MAC allow-list.
            printf("\n[BENCH] 1s window rates\n"
                   "  main_loop: %lu iter/s  avg %lu ns  max %lu ns\n"
                   "  RX:        %lu writer_err/s\n"
                   "  gPTP:      %lu sync_rx/s  %lu pdresp_rx/s\n",
                   (unsigned long)bench.r_iter_per_sec,
                   (unsigned long)bench.r_iter_avg_ns,
                   (unsigned long)bench.r_iter_max_ns,
                   (unsigned long)bench.r_writer_errors,
                   (unsigned long)bench.r_sync_rx,
                   (unsigned long)bench.r_pdresp_rx);
            break;
        case 'v':
            g_verbose = !g_verbose;
            printf("\n[main] verbose debug prints %s\n", g_verbose ? "ON" : "OFF");
            break;
        case 'G':
            gptp_dump_conv_log(&gptp);
            break;
        case 'C':
            mcr_dump_conv_log(&mcr);
            break;
        case 'N':
            // Clear the saved AVB CRF binding in NV and drop the live bind.
            // Under Dante there is no CRF -- the media clock is disciplined
            // from the PTP addend ratio -- so this is a migration cleanup
            // command: it wipes a stale AVB binding inherited from an
            // avb-aes3 config blob. The cfg fields themselves are retained
            // (config.h reserved[72] absorbs the Dante settings alongside).
            g_cfg.crf_valid = 0;
            for (int i = 0; i < 8; i++) g_cfg.crf_stream_id[i]  = 0;
            for (int i = 0; i < 6; i++) g_cfg.crf_dmac[i]       = 0;
            for (int i = 0; i < 8; i++) g_cfg.crf_talker_eid[i] = 0;
            cfg_save();
            mcr_unbind(&mcr);
            printf("[CFG] stale AVB CRF binding cleared from NV + MCR unbound.\n");
            break;
        case 'P': {
            // Sweep the AAF presentation-time offset (ns) to chase AxC "Late
            // Timestamp": +1 ms per press, wrap 2..10 ms. Larger offset = more
            // listener headroom (absorbs bridge queuing under the 6-stream load).
            static uint32_t po = AAF_PRESENTATION_OFFSET_NS;   // 2 ms reset
            po += 1000000;
            if (po > 10000000) po = 2000000;
            aaf_pkt_pres_offset_write(po);
            printf("[AAF] pres_offset = %lu ns (%lu ms)\n",
                   (unsigned long)po, (unsigned long)(po / 1000000));
            break;
        }
        case 'h':
        case '?':
            printf("\n  s   status (PTP servo, grandmaster, RX message counts)\n"
                     "  m   MCR servo state (NCO increment, offset)\n"
                     "  a   packetizer + USB ingress state\n"
                     "  e   RX ethertype counters + LiteEth heartbeat\n"
                     "  b   1-second rate window (loop rate, writer errors)\n"
                     "  t   enable gateware talker    T  disable\n"
                     "  f   toggle USB NCO freeze     F  sweep USB feedback value\n"
                     "  k/j cycle SRC servo KP / KI (live tuning)\n"
                     "  u   step ULPI IDELAY tap (re-plug USB to test)\n"
                     "  P   sweep presentation offset (AVB only; goes in Phase 5)\n"
                     "  N   clear stale AVB CRF binding from NV\n"
                     "  v   toggle verbose debug prints - default OFF\n"
                     "  G   dump PTP convergence ring-log (boot->lock curve)\n"
                     "  C   dump media-clock convergence ring-log\n"
                     "  R   dump capture ring     z  clear + re-arm capture ring\n"
                     "  r   reboot                h  help\n");
            break;
    }
}

// --- Gateware TX packetizer (aaf_pkt) control --------------------------------
//
// The gateware sources the 48-channel stream straight from the USB sample FIFO,
// paced by the MCR media clock, so the CPU is entirely OUT of the per-sample
// path. That property is the whole reason this design works at 48 channels and
// it is preserved verbatim through the Dante conversion.
//
// DANTE PHASE 0: the stream identities are now generated locally here instead
// of being handed down from AVDECC/SRP. Phase 5's dante_tx.c replaces this
// function outright: same 6 indirect contexts, but each carries
// {dst_mac (derived from the multicast IP), dst_ip, ip_csum, udp ports} and an
// 8-entry channel map, instead of {dst_mac, stream_id}.
static void aaf_gw_push_binding(void)
{
    // Shared across all 6 time-mux streams: src_mac + VLAN TCI.
    // PCP 3 / VID 2 = AVB Class A, as the bridge expects.
    aaf_pkt_src_mac_hi_write(((uint32_t)mac_addr[0] << 8) | mac_addr[1]);
    aaf_pkt_src_mac_lo_write(((uint32_t)mac_addr[2] << 24) |
                             ((uint32_t)mac_addr[3] << 16) |
                             ((uint32_t)mac_addr[4] <<  8) |
                              (uint32_t)mac_addr[5]);
    aaf_pkt_vlan_tci_write((3u << 13) | 2u);

    // Per-stream dst_mac + stream_id, written into the gateware's indirect
    // context (ctx_select=s, then the data regs). The packetizer latches
    // dmac[s] on the dst_mac_lo write and stream_id[s] on the stream_id_lo
    // write, so those must come LAST in each context.
    //
    // stream_id[s] = MAC + 00:s          (unique per stream)
    // dst_mac[s]   = 91:E0:F0:00:FE:(mac[5] & 0xF8 | s)   (6 SR multicasts)
    for (int s = 0; s < N_AAF_STREAMS; s++) {
        uint8_t dmac5 = (uint8_t)((mac_addr[5] & 0xF8) | s);
        aaf_pkt_ctx_select_write(s);
        aaf_pkt_dst_mac_hi_write(((uint32_t)0x91 << 8) | 0xE0);
        aaf_pkt_dst_mac_lo_write(((uint32_t)0xF0 << 24) |
                                 ((uint32_t)0x00 << 16) |
                                 ((uint32_t)0xFE <<  8) |
                                  (uint32_t)dmac5);
        aaf_pkt_stream_id_hi_write(((uint32_t)mac_addr[0] << 24) |
                                   ((uint32_t)mac_addr[1] << 16) |
                                   ((uint32_t)mac_addr[2] <<  8) |
                                    (uint32_t)mac_addr[3]);
        aaf_pkt_stream_id_lo_write(((uint32_t)mac_addr[4] << 24) |
                                   ((uint32_t)mac_addr[5] << 16) |
                                   ((uint32_t)0x00       <<  8) |
                                    (uint32_t)s);
    }
    // pres_offset keeps its 2 ms CSR reset value (= AAF_PRESENTATION_OFFSET_NS).
}

static void aaf_gw_set(uint8_t on)
{
    // Run the packetizer datapath whenever the stream is on. NOT gated on a
    // remote listener: doing that coupled the audio path to the AVB
    // reservation and cycled it every ~30 s (listener age-out). Any flood
    // protection must gate only MAC frame emission -- never this datapath.
    if (on) {
        aaf_gw_push_binding();
        // Do NOT enable the talker here. The PTP-lock gate in the main loop
        // turns aaf_pkt ON only once the clock is locked -- emitting before
        // that stamps wrong timestamps and no receiver can align. Phase 5
        // keeps this gate, driven by PTPv1 lock instead of gPTP lock.
    } else {
        aaf_pkt_enable_write(0);
    }
    aaf_gw_enabled = on;
}

// Drain-and-discard the USB sample FIFO.
//
// Only runs when the gateware talker is OFF. It exists purely so the FIFO
// cannot sit full while the host is streaming: a full FIFO makes
// main_usb_sample_overflow climb continuously, which destroys the value of
// that counter as a diagnostic, and it leaves the async-feedback servo
// measuring a pinned block_level.
//
// DANTE PHASE 0: this replaced usb_aaf_drain(), which reassembled 8-channel
// blocks and pushed them into the software AAF talker (aaf.c, now parked).
// The software audio path is gone for good -- it could never sustain 48
// channels (BENCHMARK_BASELINE.md: the main loop collapses to 1969 iter/s
// with only 8), and Phase 5 has no software fallback by design.
static void usb_fifo_drain(void)
{
    // One CSR read per sample (read-and-auto-pop). Bound the per-pass work so
    // a flood can't stall the main loop; 1024 = the full FIFO depth, so one
    // pass can clear a complete backlog.
    int guard = 1024;
    while (guard--) {
        uint32_t v = main_usb_sample_data_read();
        if (!(v & 0x10)) break;                // [4]=valid: 0 -> FIFO empty
        main_usb_sample_pop_write(1);          // advance to next head
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();

    printf("\n[DANTE-USB] 48ch TX  (Phase 0: AVB stack stripped)\n");

    // Enable PHY-side RGMII-ID delays at boot. The B50612D on i9plus needs:
    //   shadow_07 bit 8 = 1 (RXC delay) — reg 0x18 write 0xF1E7
    //   shadow_03 bit 9 = 1 (TXC delay) — reg 0x1C write 0x8E00
    // Without TXC delay our outgoing frames are mis-clocked at the PHY and
    // never make it onto the wire (tcpdump shows 0 packets from us even when
    // firmware reports adp_tx_count growing). PHY shadow regs persist across
    // bitstream reloads but NOT across power-cycle, so program at boot.
    {
        int a = 1;
        mdio_write(a, 0x00, 0x9140);       // soft-reset + autoneg + 1G + FD
        busy_wait(200);
        mdio_write(a, 0x18, 0xF1E7);       // shadow_07 bit 8 = 1
        busy_wait(10);
        mdio_write(a, 0x1C, 0x8E00);       // shadow_03 bit 9 = 1
        busy_wait(10);
    }
    busy_wait(100);

    // Config-flash NV — load the persisted AVDECC/system config (cs=, CRF binding,
    // ...). HW-proven read/write/erase/persist. cs is restored after avdecc_init
    // (below); CRF binding is used by the reconnect path. New params: add to cfg_t.
    {
        cfgflash_warmup();           // clock past STARTUPE2 first-edge masking
        uint32_t j = cfgflash_jedec();
        int found = cfg_load();
        printf("[CFG] JEDEC=0x%06lx  config %s  cs=%u  crf_valid=%u\n",
               (unsigned long)j, found ? "LOADED from NV" : "defaults (new flash)",
               g_cfg.cs, g_cfg.crf_valid);
    }

    // Init protocol stacks
    gptp_init(&gptp, mac_addr);
    // IP stack. The persisted address is reused from the old OSC config slot --
    // same field, and config.h's reserved[] still has room for Dante settings.
    {
        const uint8_t *ip = NULL;
        uint8_t prefix = 0;
        if ((g_cfg.osc_prefix == 16 || g_cfg.osc_prefix == 24) && g_cfg.osc_ip[0]) {
            ip = g_cfg.osc_ip;
            prefix = g_cfg.osc_prefix;
        }
        net_init(mac_addr, ip, prefix);
    }

    // PTPv1 lives on 224.0.1.129. Joining is what makes a snooping switch
    // forward it to our port; the Dante control groups are all in 224.0.0.0/24
    // (link-local scope) and are never pruned, so they need no join.
    {
        static const uint8_t ptp_group[4] = {224, 0, 1, 129};
        net_igmp_join(ptp_group);
    }

    // Dante identity + discovery (Phase 3).
    dante_dev_init(mac_addr);
    mdns_init();
    dante_arc_init();
    dante_cmc_init();
    dante_info_set_gptp(&gptp);
    dante_info_init();
    mcr_init(&mcr, CONFIG_CLOCK_FREQUENCY, 48000);
    // Give MCR the gPTP handle so the free-running (cs=0) NCO is disciplined to
    // the network media rate (exactly 48000 gPTP-Hz) instead of the raw crystal.
    mcr_set_gptp(&mcr, &gptp);
    // Start the gateware talker from boot so the USB ring always has a
    // consumer and the async-feedback servo has a real rate to measure.
    // The talker itself stays gated on clock lock inside the main loop.
    //
    // Under AVB this point also brought up: the 6 shared stream identities,
    // continuous MSRP TalkerAdvertise from boot (to avoid a reservation
    // chicken-and-egg with the listener), and the whole AVDECC entity with its
    // five ACMP/AEM callbacks. All of that is Dante's control plane now, and
    // it does not exist yet -- Phase 3 builds it (mDNS + ARC + CMC + info),
    // Phase 5 wires dante_tx.c into these contexts.
    aaf_gw_set(1);

    // Clock source: cs=0 disciplines the NCO from the PTP addend ratio, cs=1
    // used the AVB CRF stream. Dante has no CRF, so force cs=0 regardless of
    // what the NV blob says -- an inherited cs=1 would leave the media clock
    // waiting forever for CRF timestamps that never arrive.
    mcr_set_clock_source(&mcr, 0);
    if (g_cfg.cs)
        printf("[CFG] NV has cs=1 (AVB CRF) -- forcing cs=0 (PTP-disciplined NCO)\n");

    printf("[main] Press 'h' for commands.\n\n");

    while (1) {
        bench_tick();
        dispatch_rx();
        gptp_poll(&gptp);
        net_poll();
        mdns_poll();
        dante_info_poll();

        // Media clock servo. mcr_pump_hw() drained the gateware CRF timestamp
        // FIFO under AVB; with the CRF extractor gone it is a no-op, and the
        // cs=0 path (discipline the NCO from the PTP addend ratio) is what
        // actually runs. Phase 4 repoints this at the PTPv1 servo output.
        mcr_pump_hw(&mcr);
        mcr_servo_update(&mcr);
        mcr_watchdog_tick(&mcr, gptp_uptime_ms());

        // ---- Talker gate ----------------------------------------------------
        // Enable the gateware talker only once the clock is locked. Emitting
        // before lock stamps timestamps a receiver cannot align to, which
        // presents as broadband noise rather than as an obvious failure.
        // Phase 4 swaps gptp.servo_locked for the PTPv1 lock flag; Phase 5
        // keeps the gate exactly as-is.
        {
            static uint8_t talker_on = 0;
            uint8_t clock_ok = gptp.servo_locked;
            if (aaf_gw_enabled && clock_ok) {
                if (!talker_on) {
                    aaf_pkt_enable_write(1);
                    talker_on = 1;
                    printf("[main] clock locked -- talker enabled\n");
                }
            } else {
                if (talker_on) {
                    aaf_pkt_enable_write(0);
                    talker_on = 0;
                    printf("[main] talker OFF -- clock not locked\n");
                }
            }
        }

        // ---- USB rate matching ----------------------------------------------
        // Honest async feedback: the wrapper measures our own NCO-strobe-per-SOF
        // rate and reports it, so the host slaves its delivery to our media
        // clock and the ring stays balanced as a unit-ratio passthrough.
        // Nothing here chases the FIFO, so nothing can run away.
        //
        // fb_ovr = 0 selects the wrapper's measured loop; the 'F' command holds
        // a fixed value for characterisation sweeps.
        //
        // KEEP THIS UNCONDITIONAL. It deliberately runs even while the talker is
        // gated off: if feedback stops being written during a long hold-off, the
        // host over-delivers for the whole window and slams the ring full the
        // instant the talker is enabled. Phase 5 must preserve the same property
        // by feeding _BL_CENTRE while gated (see avb_soc.py:876-895).
        {
            uint32_t now_ms = gptp_uptime_ms();
            static uint32_t last_ms;
            if (now_ms != last_ms) {
                last_ms = now_ms;
                usb_lock_calls++;
                main_usb_fb_ovr_write(usb_fb_manual);   // 0 = auto measured loop
                mcr.usb_last_level = (int)aaf_pkt_fifo_level_read();
            }
        }

        // When the gateware talker is off nothing else pops the USB FIFO, so
        // drain it here to keep the overflow counter and block_level meaningful.
        if (!aaf_gw_enabled)
            usb_fifo_drain();

        check_uart_cmd();
    }

    return 0;
}
