#!/usr/bin/env python3
"""Push application firmware into the FPGA's coderam over raw Ethernet.

Dante Phase 0.5. Pairs with firmware/loader/loader.c.

The loader opens a ~400 ms window at every reset listening for our START frame.
So the flow is: run this, then reset the board (the 'r' console command, or a
power cycle, or a JTAG reconfigure). This tool repeats START until the loader
answers, so you can start it first and reset whenever.

    sudo ./tools/netload.py ens5 firmware/firmware.bin

Needs raw-socket privileges (root or CAP_NET_RAW). Pure stdlib -- no scapy.

Protocol (ethertype 0x88B5, IEEE 802 local experimental 1):

    14  4  magic "INFN"
    18  1  opcode   START=1 DATA=2 EXEC=3 | ACK=0x80 NAK=0x81
    19  1  reserved
    20  4  arg0     START: total length   DATA: byte offset   ACK: next offset
    24  4  arg1     START: crc32          DATA: payload length
    28  .. payload  (DATA only)

DATA sends its payload length explicitly because Ethernet pads frames to a
60-byte minimum: a final chunk under 32 bytes would otherwise appear longer than
it is and corrupt the tail of the image.

Stop-and-wait: every frame is ACKed with the next expected offset, so a dropped
chunk self-corrects by resending from wherever the loader says it is.
"""

import argparse
import binascii
import socket
import struct
import sys
import time

ETHERTYPE = 0x88B5
MAGIC = 0x494E464E  # "INFN"

OP_START, OP_DATA, OP_EXEC = 1, 2, 3
OP_ACK, OP_NAK = 0x80, 0x81

HDR_LEN = 28
CHUNK = 1024  # must not exceed NL_MAX_PAYLOAD in loader.c
CODERAM_SIZE = 0x18000

BROADCAST = b"\xff" * 6


def build(src_mac, dst_mac, op, arg0=0, arg1=0, payload=b""):
    return (
        dst_mac
        + src_mac
        + struct.pack("!H", ETHERTYPE)
        + struct.pack("!IBBII", MAGIC, op, 0, arg0, arg1)
        + payload
    )


def parse_reply(frame):
    """Return (op, arg0, src_mac) for a loader reply, or None."""
    if len(frame) < HDR_LEN:
        return None
    if struct.unpack("!H", frame[12:14])[0] != ETHERTYPE:
        return None
    magic, op, _, arg0, _ = struct.unpack("!IBBII", frame[14:HDR_LEN])
    if magic != MAGIC:
        return None
    if op not in (OP_ACK, OP_NAK):
        return None
    return op, arg0, frame[6:12]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", help="network interface, e.g. ens5")
    ap.add_argument("image", help="firmware .bin to load")
    ap.add_argument("--timeout", type=float, default=0.4,
                    help="per-frame ACK timeout in seconds (default 0.4)")
    ap.add_argument("--retries", type=int, default=40,
                    help="retries per frame (default 40)")
    ap.add_argument("--start-wait", type=float, default=60.0,
                    help="how long to keep sending START while waiting for a "
                         "reset, in seconds (default 60)")
    args = ap.parse_args()

    with open(args.image, "rb") as fh:
        image = fh.read()

    if not image:
        sys.exit("image is empty")
    if len(image) > CODERAM_SIZE:
        sys.exit("image is {} bytes, coderam is {}".format(len(image), CODERAM_SIZE))

    crc = binascii.crc32(image) & 0xFFFFFFFF
    print("image {}: {} bytes, crc32 {:#010x}".format(args.image, len(image), crc))

    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE))
    except PermissionError:
        sys.exit("raw socket needs root: try sudo")
    sock.bind((args.interface, 0))
    src_mac = sock.getsockname()[4][:6]
    print("via {} (src {})".format(args.interface, src_mac.hex(":")))

    def send(op, arg0=0, arg1=0, payload=b"", dst=BROADCAST):
        sock.send(build(src_mac, dst, op, arg0, arg1, payload))

    def wait_ack(deadline):
        """Wait for an ACK/NAK until deadline. Returns (op, arg0, mac) or None."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            sock.settimeout(remaining)
            try:
                frame = sock.recv(2048)
            except (socket.timeout, TimeoutError):
                return None
            got = parse_reply(frame)
            if got:
                return got

    # --- START: repeat until the loader answers (it only listens after reset) --
    print("waiting for loader... (reset the board now: 'r' on the console)")
    dst = BROADCAST
    t_end = time.monotonic() + args.start_wait
    while True:
        if time.monotonic() > t_end:
            sys.exit("no loader responded. Is the board reset? Right interface?")
        send(OP_START, len(image), crc)
        got = wait_ack(time.monotonic() + 0.02)
        if got and got[0] == OP_ACK:
            dst = got[2]  # unicast the rest to the board
            print("loader responded at {}".format(dst.hex(":")))
            break

    # --- DATA -----------------------------------------------------------------
    off = 0
    t0 = time.monotonic()
    while off < len(image):
        chunk = image[off:off + CHUNK]
        for attempt in range(args.retries):
            # arg1 = explicit payload length (see the protocol note above).
            send(OP_DATA, off, len(chunk), chunk, dst=dst)
            got = wait_ack(time.monotonic() + args.timeout)
            if got is None:
                continue
            op, next_off, _ = got
            if op == OP_NAK:
                sys.exit("loader NAKed at offset {}".format(off))
            if next_off == off + len(chunk):
                off = next_off
                break
            # Loader is somewhere else (lost/duplicated frame): resync to it.
            if next_off != off:
                off = next_off
                break
        else:
            sys.exit("no ACK for offset {} after {} retries".format(off, args.retries))

        pct = 100 * off // len(image)
        print("\r  {:3d}%  {}/{} bytes".format(pct, off, len(image)), end="", flush=True)

    dt = time.monotonic() - t0
    print("\r  100%  {}/{} bytes in {:.2f}s".format(len(image), len(image), dt))

    # --- EXEC -----------------------------------------------------------------
    for attempt in range(args.retries):
        send(OP_EXEC, dst=dst)
        got = wait_ack(time.monotonic() + args.timeout)
        if got is None:
            continue
        op, arg0, _ = got
        if op == OP_ACK:
            print("loader verified CRC and is jumping to firmware.")
            return 0
        sys.exit("loader NAKed EXEC at offset {} (CRC mismatch or short image)".format(arg0))
    sys.exit("no ACK for EXEC")


if __name__ == "__main__":
    sys.exit(main())
