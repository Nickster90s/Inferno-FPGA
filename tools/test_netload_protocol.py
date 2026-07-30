#!/usr/bin/env python3
"""Protocol conformance test for netload — Dante Phase 0.5.

The loader's receive path (firmware/loader/loader.c) cannot be unit-tested on
the host: it is RISC-V code talking to LiteEth CSRs. So this file contains a
FAITHFUL PYTHON MODEL of that state machine and drives it with frames built by
the real sender logic, checking that the image reassembles bit-exactly.

That catches protocol-level defects -- offset handling, the Ethernet minimum-frame
padding trap, retransmission and duplicate handling, CRC agreement -- before the
first hardware attempt, where the only symptom would be "netload hangs".

It does NOT test: LiteEth RX/TX in the loader, the reset window, the jump to
coderam, or the flash fallback. Those need silicon.

    ./tools/test_netload_protocol.py

If the model and loader.c drift apart this test becomes a liability, so keep the
model's structure recognisably parallel to the C.
"""

import binascii
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import netload  # the real sender: reuse its frame builder and constants

CODERAM_SIZE = 0x18000
ETH_MIN_FRAME = 60


class LoaderModel:
    """Mirror of the netload() state machine in loader.c."""

    def __init__(self):
        self.coderam = bytearray(b"\x00" * CODERAM_SIZE)
        self.img_len = 0
        self.img_crc = 0
        self.next_off = 0
        self.started = False
        self.jumped = False

    def rx(self, frame):
        """Feed one Ethernet frame. Returns the reply frame, or None."""
        # LiteEth strips the FCS but pads to the 60-byte minimum, so model that:
        # the length the MAC reports is max(len(frame), 60).
        length = max(len(frame), ETH_MIN_FRAME)
        if length > len(frame):
            frame = frame + b"\x00" * (length - len(frame))

        if length < netload.HDR_LEN:
            return None
        if struct.unpack("!H", frame[12:14])[0] != netload.ETHERTYPE:
            return None
        magic, op, _, arg0, arg1 = struct.unpack("!IBBII", frame[14:netload.HDR_LEN])
        if magic != netload.MAGIC:
            return None

        if op == netload.OP_START:
            if arg0 == 0 or arg0 > CODERAM_SIZE:
                return self._reply(frame, netload.OP_NAK, 0)
            self.img_len, self.img_crc = arg0, arg1
            self.next_off, self.started = 0, True
            return self._reply(frame, netload.OP_ACK, 0)

        if op == netload.OP_DATA and self.started:
            off, plen = arg0, arg1
            avail = length - netload.HDR_LEN
            plen = min(plen, netload.CHUNK, avail)
            if plen and off == self.next_off and off + plen <= self.img_len:
                payload = frame[netload.HDR_LEN:netload.HDR_LEN + plen]
                self.coderam[off:off + plen] = payload
                self.next_off = off + plen
            return self._reply(frame, netload.OP_ACK, self.next_off)

        if op == netload.OP_EXEC and self.started:
            if self.next_off != self.img_len:
                return self._reply(frame, netload.OP_NAK, self.next_off)
            got = binascii.crc32(self.coderam[:self.img_len]) & 0xFFFFFFFF
            if got != self.img_crc:
                self.started = False
                return self._reply(frame, netload.OP_NAK, self.next_off)
            self.jumped = True
            return self._reply(frame, netload.OP_ACK, self.img_len)

        return None

    def _reply(self, req, op, next_off):
        return netload.build(b"\x02\x00\x00\x00\x00\x42", req[6:12], op, next_off, 0)


def transfer(image, drop=(), dupe=()):
    """Run a full transfer. `drop`/`dupe` are chunk indices to perturb."""
    host_mac = b"\xaa" * 6
    dev = LoaderModel()
    crc = binascii.crc32(image) & 0xFFFFFFFF

    r = dev.rx(netload.build(host_mac, netload.BROADCAST,
                             netload.OP_START, len(image), crc))
    assert r is not None, "no reply to START"
    assert struct.unpack("!IBBII", r[14:netload.HDR_LEN])[1] == netload.OP_ACK

    off, idx = 0, 0
    guard = 0
    while off < len(image):
        guard += 1
        assert guard < 10000, "transfer failed to converge"
        chunk = image[off:off + netload.CHUNK]
        if idx in drop:
            drop = tuple(x for x in drop if x != idx)   # drop once only
            idx += 1
            continue                                     # host will resync via ACK
        frame = netload.build(host_mac, dev_mac(), netload.OP_DATA,
                              off, len(chunk), chunk)
        r = dev.rx(frame)
        if idx in dupe:
            dev.rx(frame)                                # deliver twice
        _, op, _, next_off, _ = struct.unpack("!IBBII", r[14:netload.HDR_LEN])
        assert op == netload.OP_ACK, "NAK during DATA at offset %d" % off
        off = next_off
        idx += 1

    r = dev.rx(netload.build(host_mac, dev_mac(), netload.OP_EXEC))
    _, op, _, arg0, _ = struct.unpack("!IBBII", r[14:netload.HDR_LEN])
    return dev, op, arg0


def dev_mac():
    return b"\x02\x00\x00\x00\x00\x42"


def check(name, image, **kw):
    dev, op, arg0 = transfer(image, **kw)
    ok = (op == netload.OP_ACK
          and dev.jumped
          and bytes(dev.coderam[:len(image)]) == image)
    print("  %-46s %s" % (name, "PASS" if ok else "FAIL"))
    if not ok:
        print("     op=%#x jumped=%s len=%d" % (op, dev.jumped, len(image)))
        first = next((i for i in range(len(image))
                      if dev.coderam[i] != image[i]), None)
        print("     first differing byte: %s" % first)
    return ok


def main():
    print("netload protocol conformance (model of loader.c)")
    results = []

    # Sizes chosen to exercise the tail cases. The 8- and 20-byte remainders are
    # the Ethernet-padding trap: frame = 28 + rem, padded up to 60, so a length
    # derived from the frame would over-read by 32 - rem bytes.
    for size in (1, 8, 20, 31, 32, 33, 1023, 1024, 1025, 2048,
                 32136,                       # the real firmware.bin size
                 31 * 1024 + 8,               # last chunk 8 bytes -> padded
                 31 * 1024 + 20):             # last chunk 20 bytes -> padded
        img = bytes((i * 37 + 11) & 0xFF for i in range(size))
        results.append(check("size %d" % size, img))

    img = bytes((i * 91 + 3) & 0xFF for i in range(9000))
    results.append(check("9000 bytes, chunk 2 dropped once", img, drop=(2,)))
    results.append(check("9000 bytes, chunk 3 duplicated", img, dupe=(3,)))
    results.append(check("9000 bytes, drop 0 + dupe 5", img, drop=(0,), dupe=(5,)))

    # A real firmware image if one is built.
    fw = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "firmware", "firmware.bin")
    if os.path.exists(fw):
        with open(fw, "rb") as fh:
            data = fh.read()
        results.append(check("real firmware.bin (%d bytes)" % len(data), data))

    print("\n%d/%d passed" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
