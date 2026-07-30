#!/usr/bin/env python3
"""Wrap a firmware .bin in the loader's image header.

Dante Phase 0.5. The output is written to SPI flash at FLASH_IMAGE_ADDR so the
board boots standalone with no host attached; firmware/loader/loader.c reads it
when netload supplies nothing.

    ./tools/mkimage.py firmware/firmware.bin firmware/firmware.img

Then write it with the existing openocd/bscan path -- no firmware support and no
new tooling needed, because openocd can write the SPI flash at an offset:

    TOOLS=/home/lisp/FPGA/Colorlight-FPGA-Projects/tools
    sudo /home/lisp/openocd/src/openocd -s /home/lisp/openocd/tcl -f $TOOLS/ch347.cfg -c "
      init
      pld load 0 $TOOLS/bscan_spi_xc7a50t.bit
      reset halt
      flash probe 0
      flash protect 0 0 50 off
      flash write_image erase firmware/firmware.img 0x300000 bin
    " -c exit

Note that writes the FIRMWARE image only, at 3 MB -- it does not touch the
bitstream at 0x0 or the config sector at the top of flash. Close picocom first;
openocd and picocom cannot both hold /dev/ttyACM0.

Header (big-endian, 12 bytes), matching loader.c:
    0  4  magic "INFN"
    4  4  length of the firmware payload
    8  4  crc32 (IEEE, as produced by binascii.crc32)
"""

import argparse
import binascii
import struct
import sys

MAGIC = 0x494E464E  # "INFN"
CODERAM_SIZE = 0x18000
FLASH_IMAGE_ADDR = 0x00300000


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="firmware .bin")
    ap.add_argument("output", help="image file to write")
    args = ap.parse_args()

    with open(args.input, "rb") as fh:
        payload = fh.read()

    if not payload:
        sys.exit("input is empty")
    if len(payload) > CODERAM_SIZE:
        sys.exit("firmware is {} bytes, coderam is {}".format(len(payload), CODERAM_SIZE))

    crc = binascii.crc32(payload) & 0xFFFFFFFF
    header = struct.pack("!III", MAGIC, len(payload), crc)

    with open(args.output, "wb") as fh:
        fh.write(header + payload)

    print("{}: {} bytes payload, crc32 {:#010x}".format(args.output, len(payload), crc))
    print("total {} bytes -> write to flash at {:#x}".format(
        len(header) + len(payload), FLASH_IMAGE_ADDR))
    return 0


if __name__ == "__main__":
    sys.exit(main())
