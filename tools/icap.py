#!/usr/bin/env python3
"""Read the FPGA's ICAP configuration registers ('Q' -> 'ICP1').

THE TEST THAT SETTLES WHETHER ICAPE2 WORKS AT ALL UNDER openXC7.

Measured 2026-11-07: Dante Controller's Reboot button (info opcode 0x90)
reached the firmware, the firmware verified the SPI image's CRC and wrote IPROG
to the CMD register, and the FPGA did not reconfigure -- USB stayed enumerated
on the same device numbers, so the fabric never reset. The flash read path is
therefore not the problem, and the question left is whether the configuration
engine ever saw the command.

IDCODE is the answer. It must read 0x0362c093, the same value JTAG reports for
this XC7A50T. If it does, the whole ICAP path works -- sync word, byte and bit
order, X32 width, clocking -- and the fault is specifically the warm boot. If it
reads 0x00000000 or 0xffffffff, ICAPE2 is not functional in an openXC7 bitstream
and self-reset through ICAP is a dead end on this flow.

    tools/icap.py [board]
"""
import socket
import struct
import sys

from dante_board import BOARD as _BOARD

IDCODE_EXPECTED = 0x0362C093

# UG470 register numbers, in the order the firmware reports them.
NAMES = {
    0x0C: "IDCODE",
    0x16: "BOOTSTS",
    0x07: "STAT",
    0x05: "CTL0",
    0x09: "COR0",
    0x10: "WBSTAR",
}

# BOOTSTS tells whether a warm boot was even attempted. UG470: the register is
# cleared only by power-on reset, PROGRAM_B or JPROGRAM -- so after a failed
# IPROG it still holds the history, and after a power cycle it does not.
BOOTSTS_BITS = [
    (0, "STATUS_VALID"),
    (1, "FALLBACK"),
    (2, "IPROG (internal PROG issued)"),
    (3, "WTO_ERROR (watchdog timeout)"),
    (4, "ID_ERROR"),
    (5, "CRC_ERROR"),
    (6, "WRAP_ERROR"),
    (7, "HMAC_ERROR"),
]


def main():
    board = sys.argv[1] if len(sys.argv) > 1 else _BOARD
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    s.sendto(b"Q", (board, 7779))
    try:
        d, _ = s.recvfrom(512)
    except socket.timeout:
        print(f"  NOTHING ANSWERED on {board}:7779. The board is off, on another"
              " address, or wedged. This is NOT a statement about ICAP.")
        return 2
    if len(d) < 8 or d[:4] != b"ICP1":
        print("  answered, but not with ICP1 -- that firmware predates the 'Q'"
              " opcode. Flash with ./flash.sh --fw-only.")
        return 2

    present = struct.unpack(">I", d[4:8])[0]
    if not present:
        print("  this GATEWARE has no ICAP (no CSR_ICAP_BASE), so there is"
              " nothing to read. Build with the ICAP core instantiated.")
        return 2

    idcode = None
    off = 8
    while off + 12 <= len(d):
        reg, val, timed_out = struct.unpack(">III", d[off:off + 12])
        off += 12
        name = NAMES.get(reg, f"reg 0x{reg:02x}")
        flag = "  TIMED OUT (icap_done never came)" if timed_out else ""
        print(f"  {name:8s} 0x{val:08x}{flag}")
        if reg == 0x0C:
            idcode = val
        if reg == 0x16 and not timed_out:
            set_bits = [n for b, n in BOOTSTS_BITS if val & (1 << b)]
            print("           bits: " + (", ".join(set_bits) or "none set"))

    print()
    if idcode is None:
        print("  VERDICT: no IDCODE in the reply -- cannot judge.")
        return 2
    if idcode == IDCODE_EXPECTED:
        print(f"  VERDICT: IDCODE reads 0x{idcode:08x}, exactly what JTAG reports"
              " for this part. THE ICAP PATH WORKS end to end: the site is"
              " alive, the sync word was recognised, byte and bit order are"
              " right, the X32 width is right and the divided clock reaches"
              " ICAPE2. So a failed IPROG is NOT a dead primitive -- look at"
              " the warm boot itself (BOOTSTS above, WBSTAR, and whether this"
              " bitstream carries the configuration options a warm boot from"
              " SPI needs).")
        return 0
    if idcode in (0x00000000, 0xFFFFFFFF):
        print(f"  VERDICT: IDCODE reads 0x{idcode:08x}, which is the bus idle"
              " value, not a device ID. ICAPE2 IS NOT FUNCTIONAL in this"
              " openXC7 bitstream even though the FASM routes the site"
              " completely. Self-reset through ICAP is a dead end on this flow;"
              " Controller's Reboot button needs another mechanism. Do not"
              " spend another build on the IPROG sequence.")
        return 1
    print(f"  VERDICT: IDCODE reads 0x{idcode:08x}, neither the expected"
          f" 0x{IDCODE_EXPECTED:08x} nor an idle value. Something is reaching"
          " the register and coming back mangled -- suspect byte/bit order or"
          " the read timing, not a dead site.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
