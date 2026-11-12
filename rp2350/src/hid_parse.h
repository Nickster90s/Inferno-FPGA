#ifndef HID_PARSE_H
#define HID_PARSE_H

#include <stdbool.h>
#include <stdint.h>

/* Where a mouse keeps its buttons, X, Y and wheel inside its own report.
 * Pure C, no SDK: test/test_host.c runs it on the build machine. */

typedef struct {
    uint16_t bit;            /* offset from the start of the report DATA,     */
    uint8_t  size;           /* i.e. after the report-ID byte if there is one */
} hid_field_t;

typedef struct {
    bool        valid;       /* a relative X and Y were found                 */
    uint8_t     report_id;   /* 0 = this interface does not use report IDs    */
    hid_field_t buttons;     /* size = number of 1-bit buttons, capped at 3   */
    hid_field_t x, y, wheel; /* size 0 = absent                               */
} hid_mouse_layout_t;

typedef struct {
    uint8_t buttons;
    bool    buttons_valid;
    int32_t dx, dy, dw;
} hid_mouse_event_t;

/* Find the first Mouse/Pointer application collection with relative X and Y. */
bool hid_parse_mouse(const uint8_t *desc, uint16_t len, hid_mouse_layout_t *out);

/* Decode one received report. `report` is what the endpoint delivered,
 * including the ID byte when the layout has one. False = not this report. */
bool hid_mouse_decode(const hid_mouse_layout_t *l, const uint8_t *report,
                      uint16_t len, hid_mouse_event_t *ev);

#endif
