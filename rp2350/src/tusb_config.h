#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

// Two USB ports, two roles:
//   rhport 0 = the RP2350's native controller = Type-C  = DEVICE (CDC console)
//   rhport 1 = Pico-PIO-USB on GPIO12/13      = USB-A   = HOST   (keyboard, mouse)

// CFG_TUSB_OS and CFG_TUSB_MCU come from the SDK's command line (OPT_OS_PICO).
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG          0
#endif

#define BOARD_TUD_RHPORT        0
#define BOARD_TUH_RHPORT        1

#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED
#define CFG_TUH_ENABLED         1
#define CFG_TUH_MAX_SPEED       OPT_MODE_FULL_SPEED
#define CFG_TUH_RPI_PIO_USB     1

// ---- device: one CDC, the console ----
#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  64
// The FPGA can log at 1 Mbaud = 100 bytes per ms; a full-speed bulk endpoint
// moves 64 per transfer. 1024 absorbs a burst, and what does not fit is
// dropped (main.c) rather than allowed to stall the HID path.
#define CFG_TUD_CDC_TX_BUFSIZE  1024
#define CFG_TUD_CDC_EP_BUFSIZE  64

// ---- host: keyboard + mouse, possibly behind a hub or in one combo dongle ----
// 512, not the example's 256: the report descriptor is fetched INTO this
// buffer, and a descriptor that does not fit is silently not delivered -- the
// mouse would then fall back to boot protocol and lose its wheel.
#define CFG_TUH_ENUMERATION_BUFSIZE 512
#define CFG_TUH_HUB             1
#define CFG_TUH_DEVICE_MAX      4
#define CFG_TUH_HID             8
#define CFG_TUH_HID_EPIN_BUFSIZE  64
#define CFG_TUH_HID_EPOUT_BUFSIZE 64

#endif
