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

add_executable(kvm_rp2350
    src/main.c
    src/kvm_sched.c
    src/hid_parse.c
    src/usb_descriptors.c
)
target_include_directories(kvm_rp2350 PRIVATE src)
target_compile_options(kvm_rp2350 PRIVATE -Wall -Wextra)

target_link_libraries(kvm_rp2350
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_uart
    tinyusb_device
    tinyusb_host
    tinyusb_pico_pio_usb
)

# stdio goes nowhere: UART0 is the FPGA link and must carry frames only, and
# pico_stdio_usb cannot coexist with a TinyUSB host. The console is the CDC
# interface in main.c.
pico_enable_stdio_uart(kvm_rp2350 0)
pico_enable_stdio_usb(kvm_rp2350 0)

pico_add_extra_outputs(kvm_rp2350)   # kvm_rp2350.uf2
