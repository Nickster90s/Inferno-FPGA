cmake_minimum_required(VERSION 3.13)

# Waveshare RP2350-USB-A: USB-A on GPIO12/13 (PIO-USB host), Type-C native.
set(PICO_BOARD waveshare_rp2350_usb_a CACHE STRING "")

# Pico-PIO-USB is not part of the SDK. TinyUSB's rp2040 family.cmake picks it
# up from this variable and creates the tinyusb_pico_pio_usb target.
if (NOT DEFINED PICO_PIO_USB_PATH)
    if (DEFINED ENV{PICO_PIO_USB_PATH})
        set(PICO_PIO_USB_PATH $ENV{PICO_PIO_USB_PATH})
    else ()
        set(PICO_PIO_USB_PATH $ENV{HOME}/pico/Pico-PIO-USB)
    endif ()
endif ()
if (NOT EXISTS ${PICO_PIO_USB_PATH}/src/pio_usb.c)
    message(FATAL_ERROR "Pico-PIO-USB not found at ${PICO_PIO_USB_PATH} -- set PICO_PIO_USB_PATH")
endif ()

include(pico_sdk_import.cmake)

project(kvm_rp2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

if (NOT TARGET tinyusb_pico_pio_usb)
    message(FATAL_ERROR "the SDK's TinyUSB did not create tinyusb_pico_pio_usb")
endif ()

# -DKVM_NATIVE_HOST=1: keyboard/mouse on the Type-C (the RP2350's real USB
# controller) instead of the bit-banged USB-A. See README, "Wired devices
# behind a hub".
option(KVM_NATIVE_HOST "USB host on the native controller (Type-C)" OFF)

add_executable(kvm_rp2350
    src/main.c
    src/kvm_sched.c
    src/hid_parse.c
    src/kvm_down.c
)
if (KVM_NATIVE_HOST)
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_NATIVE_HOST=1)
    target_link_libraries(kvm_rp2350 tinyusb_host)
else ()
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_NATIVE_HOST=0)
    target_sources(kvm_rp2350 PRIVATE src/usb_descriptors.c)
    target_link_libraries(kvm_rp2350 tinyusb_device tinyusb_host tinyusb_pico_pio_usb)
endif ()
target_include_directories(kvm_rp2350 PRIVATE src)
target_compile_options(kvm_rp2350 PRIVATE -Wall -Wextra)
# panic() -> our log ring, then the watchdog (main.c, kvm_panic).
target_compile_definitions(kvm_rp2350 PRIVATE PICO_PANIC_FUNCTION=kvm_panic)

# -DKVM_USB_DEBUG=1: TinyUSB's log on the CDC console (see tusb_config.h).
option(KVM_USB_DEBUG "route TinyUSB's debug log to the CDC console" OFF)
if (KVM_USB_DEBUG)
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_USB_DEBUG=1)
else ()
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_USB_DEBUG=0)
endif ()

target_link_libraries(kvm_rp2350
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_uart
    hardware_watchdog
    hardware_flash
    pico_flash
)

# stdio goes nowhere: UART0 is the FPGA link and must carry frames only, and
# pico_stdio_usb cannot coexist with a TinyUSB host. The console is the CDC
# interface in main.c.
# -DKVM_LINK_BEACON=1: bench only, see beacon() in main.c.
option(KVM_LINK_BEACON "signal USB progress through the FPGA's resync counter" OFF)
if (KVM_LINK_BEACON)
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_LINK_BEACON=1)
else ()
    target_compile_definitions(kvm_rp2350 PRIVATE KVM_LINK_BEACON=0)
endif ()

# Bench knobs for the Pico-PIO-USB patch (patches/), in quarter LS bit times.
# Empty = the patch's own defaults.
set(KVM_PRE_ACK_QBITS "" CACHE STRING "EOP -> handshake behind a hub")
set(KVM_PRE_GAP_QBITS "" CACHE STRING "extra idle before every PRE")
foreach (knob KVM_PRE_ACK_QBITS KVM_PRE_GAP_QBITS)
    if (NOT "${${knob}}" STREQUAL "")
        target_compile_definitions(kvm_rp2350 PRIVATE ${knob}=${${knob}})
    endif ()
endforeach ()

pico_enable_stdio_uart(kvm_rp2350 0)
pico_enable_stdio_usb(kvm_rp2350 0)

pico_add_extra_outputs(kvm_rp2350)   # kvm_rp2350.uf2
