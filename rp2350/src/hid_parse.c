// A report-descriptor parser that answers exactly one question: where are the
// buttons, X, Y and wheel of the mouse on this interface?
//
// WHY NOT BOOT PROTOCOL FOR THE MOUSE. Boot protocol is three bytes -- buttons,
// X, Y -- and has no wheel. A KVM without a scroll wheel is a thing people
// notice within a minute. So the mouse runs in report protocol, and report
// protocol means the layout is whatever the vendor chose: 8-, 12- or 16-bit
// axes, with or without a report ID. TinyUSB's own
// tuh_hid_parse_report_descriptor() only lists report IDs and usages, not
// where the fields sit, hence this file.
//
// The keyboard does NOT come through here: it stays in boot protocol, whose
// 8-byte layout is exactly the payload the FPGA's descriptor declares.

#include "hid_parse.h"
#include <string.h>

#define PAGE_GENERIC_DESKTOP 0x01
#define PAGE_BUTTON          0x09
#define USAGE_POINTER        0x01
#define USAGE_MOUSE          0x02
#define USAGE_X              0x30
#define USAGE_Y              0x31
#define USAGE_WHEEL          0x38

#define MAX_USAGES   16
#define MAX_IDS      8
#define MAX_DEPTH    8
#define MAX_PUSH     4

typedef struct {
    uint16_t page;
    uint32_t size, count;
    uint8_t  id;
} globals_t;

bool hid_parse_mouse(const uint8_t *d, uint16_t len, hid_mouse_layout_t *out)
{
    globals_t g = {0}, stack[MAX_PUSH];
    int       sp = 0;

    // Locals, reset after every main item.
    uint32_t usages[MAX_USAGES];            /* page << 16 | usage */
    int      n_usages = 0;
    uint32_t umin = 0, umax = 0;
    bool     have_range = false;

    // Bit position inside each report ID's INPUT report.
    uint8_t  ids[MAX_IDS];
    uint16_t bits[MAX_IDS];
    int      n_ids = 0;

    // Is the application collection we are inside a mouse?
    bool     app_is_mouse[MAX_DEPTH + 1] = { false };
    int      depth = 0;

    hid_mouse_layout_t l;
    memset(&l, 0, sizeof l);
    bool     done = false;                  /* a complete mouse was closed */

    uint16_t i = 0;
    while (i < len && !done) {
        uint8_t  pre = d[i++];
        if (pre == 0xFE) {                  /* long item: skip it whole */
            if (i >= len) break;
            i = (uint16_t)(i + 2 + d[i]);
            continue;
        }
        uint8_t  n    = pre & 3; if (n == 3) n = 4;
        uint8_t  type = (pre >> 2) & 3;
        uint8_t  tag  = pre >> 4;
        if (i + n > len) break;
        uint32_t v = 0;
        for (uint8_t k = 0; k < n; k++) v |= (uint32_t)d[i + k] << (8 * k);
        i = (uint16_t)(i + n);

        if (type == 1) {                    /* ---- global ---- */
            switch (tag) {
            case 0x0: g.page  = (uint16_t)v; break;
            case 0x7: g.size  = v; break;
            case 0x8: g.id    = (uint8_t)v; break;
            case 0x9: g.count = v; break;
            case 0xA: if (sp < MAX_PUSH) stack[sp++] = g; break;
            case 0xB: if (sp > 0) g = stack[--sp]; break;
            default: break;
            }
        } else if (type == 2) {             /* ---- local ---- */
            // A 4-byte usage carries its own page in the upper half.
            uint32_t u = (n == 4) ? v : ((uint32_t)g.page << 16) | v;
            switch (tag) {
            case 0x0: if (n_usages < MAX_USAGES) usages[n_usages++] = u; break;
            case 0x1: umin = u; have_range = true; break;
            case 0x2: umax = u; have_range = true; break;
            default: break;
            }
        } else if (type == 0) {             /* ---- main ---- */
            if (tag == 0xA) {               /* Collection */
                bool mouse = app_is_mouse[depth];
                if (v == 0x01) {            /* Application */
                    uint32_t u = n_usages ? usages[0] : 0;
                    mouse = (u == ((PAGE_GENERIC_DESKTOP << 16) | USAGE_MOUSE)) ||
                            (u == ((PAGE_GENERIC_DESKTOP << 16) | USAGE_POINTER));
                }
                if (depth < MAX_DEPTH) depth++;
                app_is_mouse[depth] = mouse;
            } else if (tag == 0xC) {        /* End Collection */
                if (depth > 0) depth--;
                if (depth == 0 && l.x.size && l.y.size) done = true;
            } else if (tag == 0x8) {        /* Input */
                int slot = -1;
                for (int k = 0; k < n_ids; k++)
                    if (ids[k] == g.id) slot = k;
                if (slot < 0 && n_ids < MAX_IDS) {
                    slot = n_ids++;
                    ids[slot] = g.id;
                    bits[slot] = 0;
                }
                if (slot < 0) break;        /* more report IDs than we track */

                bool in_mouse = app_is_mouse[depth] &&
                                (l.report_id == g.id || !(l.x.size || l.buttons.size));
                bool constant = v & 1, variable = v & 2, relative = v & 4;

                if (in_mouse && !constant && variable) {
                    for (uint32_t f = 0; f < g.count; f++) {
                        uint32_t u;
                        if (have_range)
                            u = (umin + f <= umax) ? umin + f : umax;
                        else if (n_usages)
                            u = usages[f < (uint32_t)n_usages ? f : (uint32_t)n_usages - 1];
                        else
                            continue;
                        uint16_t at   = (uint16_t)(bits[slot] + f * g.size);
                        uint16_t page = (uint16_t)(u >> 16), usage = (uint16_t)u;

                        if (page == PAGE_BUTTON && g.size == 1 &&
                            usage >= 1 && usage <= 3) {
                            // Buttons 1..3 are consecutive bits in every mouse
                            // this was written against; record the first and
                            // count the rest.
                            if (usage == 1) { l.buttons.bit = at; l.buttons.size = 1; }
                            else if (l.buttons.size == usage - 1 &&
                                     at == l.buttons.bit + usage - 1)
                                l.buttons.size = (uint8_t)usage;
                            l.report_id = g.id;
                        } else if (page == PAGE_GENERIC_DESKTOP && relative &&
                                   g.size >= 2 && g.size <= 32) {
                            hid_field_t *t = usage == USAGE_X     ? &l.x :
                                             usage == USAGE_Y     ? &l.y :
                                             usage == USAGE_WHEEL ? &l.wheel : 0;
                            if (t && !t->size) {
                                t->bit = at;
                                t->size = (uint8_t)g.size;
                                l.report_id = g.id;
                            }
                        }
                    }
                }
                bits[slot] = (uint16_t)(bits[slot] + g.size * g.count);
            }
            n_usages = 0;
            have_range = false;
            umin = umax = 0;
        }
    }

    l.valid = l.x.size && l.y.size;
    *out = l;
    return l.valid;
}

static bool get_bits(const uint8_t *p, uint16_t len, hid_field_t f, bool sign,
                     int32_t *out)
{
    if (!f.size || (uint32_t)f.bit + f.size > (uint32_t)len * 8) return false;
    uint32_t v = 0;
    for (uint8_t k = 0; k < f.size; k++) {
        uint16_t b = (uint16_t)(f.bit + k);
        if (p[b >> 3] & (1u << (b & 7))) v |= 1u << k;
    }
    if (sign && f.size < 32 && (v & (1u << (f.size - 1))))
        v |= ~0u << f.size;
    *out = (int32_t)v;
    return true;
}

bool hid_mouse_decode(const hid_mouse_layout_t *l, const uint8_t *r,
                      uint16_t len, hid_mouse_event_t *ev)
{
    if (!l->valid) return false;
    if (l->report_id) {
        if (len < 1 || r[0] != l->report_id) return false;
        r++; len--;
    }
    int32_t b = 0;
    memset(ev, 0, sizeof *ev);
    ev->buttons_valid = get_bits(r, len, l->buttons, false, &b);
    ev->buttons = (uint8_t)b;
    if (!get_bits(r, len, l->x, true, &ev->dx)) return false;
    if (!get_bits(r, len, l->y, true, &ev->dy)) return false;
    get_bits(r, len, l->wheel, true, &ev->dw);      /* optional */
    return true;
}
