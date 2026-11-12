#!/usr/bin/env python3
"""Show what the host receives from the board's HID devices (needs sudo).

    sudo tools/hidwatch.py            # Main + Backup, keyboard + mouse
    sudo tools/hidwatch.py main       # only names containing "main"

evtest is not installed on this box and the event nodes are root:input, so this
reads /dev/input/event* directly. Device numbers change on every re-enumeration;
the nodes are found by name from /proc/bus/input/devices.
"""
import os
import re
import select
import struct
import sys

sys.stdout.reconfigure(line_buffering=True)   # readable through a pipe or tee

EV = struct.Struct("qqHHi")            # struct input_event on 64-bit Linux
REL = {0: "x", 1: "y", 8: "wheel", 11: "wheel_hi_res"}
VAL = {0: "up", 1: "down", 2: "repeat"}
# Linux key codes -> what is printed on the key (US layout), so a typed word
# can be read back instead of looked up.
KEY = {1: "Esc", 14: "Backspace", 15: "Tab", 28: "Enter", 29: "LCtrl",
       42: "LShift", 54: "RShift", 56: "LAlt", 57: "Space", 58: "CapsLock",
       97: "RCtrl", 100: "RAlt", 103: "Up", 105: "Left", 106: "Right",
       108: "Down", 111: "Delete", 125: "LMeta", 126: "RMeta",
       272: "MouseLeft", 273: "MouseRight", 274: "MouseMiddle"}
KEY.update(zip(range(2, 12), "1234567890"))
KEY.update(zip(range(16, 26), "qwertyuiop"))
KEY.update(zip(range(30, 39), "asdfghjkl"))
KEY.update(zip(range(44, 51), "zxcvbnm"))
KEY.update({59 + i: f"F{i + 1}" for i in range(10)})


def devices(want):
    out = {}
    for block in open("/proc/bus/input/devices").read().split("\n\n"):
        name = re.search(r'N: Name="(.*)"', block)
        ev = re.search(r"H: Handlers=.*\b(event\d+)", block)
        if not name or not ev or "N-Series Dante" not in name.group(1):
            continue
        if want and want.lower() not in name.group(1).lower():
            continue
        out[name.group(1).replace("EventsLight N-Series Dante ", "")] = "/dev/input/" + ev.group(1)
    return out


def main():
    devs = devices(sys.argv[1] if len(sys.argv) > 1 else "")
    if not devs:
        sys.exit("no 'N-Series Dante' input devices found -- is the board enumerated?")
    fds = {}
    for name, path in sorted(devs.items()):
        try:
            fds[os.open(path, os.O_RDONLY)] = name
            print(f"watching {name:16} {path}")
        except PermissionError:
            sys.exit(f"{path}: permission denied -- run with sudo")
    print("type / move the mouse on the RP2350; Ctrl-C to stop\n")
    try:
        while True:
            for fd in select.select(list(fds), [], [])[0]:
                data = os.read(fd, EV.size * 64)
                for i in range(0, len(data) - EV.size + 1, EV.size):
                    _, _, typ, code, val = EV.unpack_from(data, i)
                    if typ == 1:
                        print(f"{fds[fd]:16} key {KEY.get(code, code)!s:11} {VAL.get(val, val)}")
                    elif typ == 2:
                        print(f"{fds[fd]:16} rel {REL.get(code, code)} {val:+d}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
