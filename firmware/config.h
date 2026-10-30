// Persistent AVDECC/system configuration, stored in the config-flash NV sector
// (see cfgflash.c). General + EXTENSIBLE: to persist a new parameter, add a field
// before `reserved`, shrink `reserved` by the same size, and bump CFG_VERSION.
// Old/blank/corrupt blobs fail validation -> defaults are used (never corrupts).
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define CFG_MAGIC    0xCFA50701u
#define CFG_VERSION  4          // v4 adds the media-clock warm-start rate; v1-v3 blobs still load

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;              // sizeof(cfg_t) — guards struct-layout changes
    // ---- persisted parameters ----
    uint8_t  cs;               // media clock source (0=gPTP/INTERNAL, 1=CRF/STREAM)
    uint8_t  crf_valid;        // 1 if the crf binding below is real
    uint8_t  _pad[2];
    uint8_t  crf_stream_id[8]; // last bound CRF stream (auto-reconnect, #70)
    uint8_t  crf_dmac[6];      // its dest MAC (for the AVTP RX filter on reconnect)
    uint8_t  _pad2[2];
    uint8_t  crf_talker_eid[8];// talker entity-id — matches the re-advertise so the
                               // fast-connect path auto-binds at boot (the #70 method)
    // ---- OSC static IP (read-OSC feature; set via AVDECC UTF8 control) ----
    uint8_t  osc_ip[4];        // static IP (default 169.254.9.200)
    uint8_t  osc_prefix;       // subnet prefix: 16 or 24
    uint8_t  _pad3[3];
    // ---- room for future AVDECC params (entity name, talker fast-connect
    //      bindings, stream formats, pres_offset, chan_rot, ...) ----
    // ---- PTPv1 warm start ----
    // The addend the servo converged on last time. Restoring it at boot skips
    // most of frequency acquisition: the crystal error is a property of THIS
    // board and does not change between runs, so re-measuring it from scratch
    // every boot costs ~10-20 s of the ~30 s lock time for no new information.
    uint8_t  ptp_addend_valid;
    uint8_t  _pad4[3];
    uint64_t ptp_addend_full;   // last converged 52-bit TSU addend
    // Media-clock rate WARM START. PTP already warm-starts its addend above;
    // the media clock did not, so every boot began at nominal and slewed to
    // ~-4300 ppb at 100 ppb/s -- 43 seconds of wrong-and-changing sample rate
    // with the talker already running. That is the "bad audio for a while
    // after boot" the operator reported. Persisting the learned rate makes the
    // media clock start within ppb of correct on the first packet.
    // NOTE THE ORDER: the int32 comes FIRST so it lands on its natural
    // alignment straight after the uint64, and reserved shrinks by exactly the
    // 5 bytes used. Declaring the uint8 first inserted 3 bytes of padding,
    // which changed sizeof(cfg_t) -- and cfg_load() requires
    // `t.size == sizeof(cfg_t)`, so EVERY stored blob was rejected and reset to
    // defaults on every boot. The warm start could never engage because its own
    // config field was being wiped before it was read.
    int32_t  mclk_ppb;          // last converged media-clock rate correction
    uint8_t  mclk_ppb_valid;
    // SAMPLE RATE, gekozen in Dante Controller (info 0x81 SET) en hier
    // bewaard omdat de hele klokketen bij boot wordt opgezet: een
    // rate-wijziging herstart het bord en leest dit veld.
    //
    // DE PLAATS IS DWINGEND, NIET COSMETISCH. Twee eisen tegelijk:
    //  1. sizeof(cfg_t) moet GELIJK blijven (120), want cfg_load() eist
    //     t.size == sizeof(cfg_t) -- anders wordt elke opgeslagen blob
    //     verworpen en op defaults gezet. Daarom komt dit veld UIT reserved
    //     en niet erbovenop.
    //  2. Elk bestaand veld moet op zijn HUIDIGE offset blijven. Een eerdere
    //     versie van dit veld stond vlak na mclk_ppb, netjes uitgelijnd en met
    //     identieke sizeof -- maar dat schoof mclk_ppb_valid van offset 60
    //     naar 64, dus een v4-blob las dat vlag uit oude reserved-nullen en
    //     verloor stil zijn media-clock warm start: de 43 seconden
    //     wrong-and-changing samplerate na boot die hierboven beschreven staat.
    // Vandaar expliciete padding tot de volgende 4-grens: mclk_ppb 56,
    // mclk_ppb_valid 60, _pad5 61..63, sample_rate 64, reserved 68..115,
    // crc 116. Een v4-blob heeft nullen op 64..67, dus cfg_load() valideert
    // en valt terug op 48000.
    uint8_t  _pad5[3];
    uint32_t sample_rate;       // 48000 of 96000
    uint8_t  reserved[48];
    uint32_t crc;              // checksum over all bytes above
} cfg_t;

extern cfg_t g_cfg;            // the live config (loaded at boot, saved on change)

// Load NV into g_cfg. Returns 1 if a valid saved config was found, else 0 and
// g_cfg is filled with defaults.
// De ACTIEVE sample rate, eenmaal bij boot uit cfg gezet en daarna constant.
// Een global in plaats van een accessor omdat dante_tx hem per pakket gebruikt
// (timestamp = seconden * fs + subsec) en een call per gebruik daar niets
// toevoegt. Altijd 48000 of 96000 -- cfg_load() valideert.
extern uint32_t g_fs;

int  cfg_load(void);

// Persist g_cfg to NV (erase sector + program). Call after changing any field.
void cfg_save(void);

#endif // CONFIG_H
