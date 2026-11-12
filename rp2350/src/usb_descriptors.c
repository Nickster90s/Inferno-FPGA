// Device descriptors for the Type-C port: one CDC-ACM interface, the console.
// Adapted from TinyUSB examples/dual/host_hid_to_device_cdc (MIT, Ha Thach).

#include "tusb.h"
#include "pico/unique_id.h"
#include <string.h>

// 1209 is the pid.codes VID the FPGA's own descriptors use (EAB1 Main, EAB2
// Backup). EAB3 is NOT a registered allocation, only the next number along.
#define USB_VID   0x1209
#define USB_PID   0xEAB3

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                       EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

static const char *const string_desc[] = {
    NULL,                       /* 0: language id, built below */
    "EventsLight",              /* 1: manufacturer */
    "Dante USB KVM link",       /* 2: product */
    NULL,                       /* 3: serial, from the flash unique id */
    "KVM console",              /* 4: CDC interface */
};

static uint16_t desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t n = 0;
    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    const char *s;

    if (index == 0) {
        desc_str[1] = 0x0409;
        n = 1;
    } else {
        if (index == 3) {
            pico_get_unique_board_id_string(serial, sizeof serial);
            s = serial;
        } else if (index < sizeof string_desc / sizeof string_desc[0]) {
            s = string_desc[index];
        } else {
            return NULL;
        }
        n = strlen(s);
        if (n > 32) n = 32;
        for (size_t i = 0; i < n; i++) desc_str[1 + i] = (uint8_t)s[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc_str;
}
