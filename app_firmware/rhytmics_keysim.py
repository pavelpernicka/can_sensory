#!/usr/bin/env python3
from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
import time

import can

FRAME_EVENT = 0x20
FRAME_EVENT_STATE = 0x32

EVENT_SECTOR_ACTIVATED = 1
EVENT_SECTOR_CHANGED = 2
EVENT_INTENSITY_CHANGE = 3
EVENT_SECTION_DEACTIVATED = 4

KEY_TO_SECTOR_MAIN = {
    "q": 0,
    "w": 1,
    "e": 2,
    "r": 3,
    "t": 4,
    "z": 5,
    "u": 6,
}

KEY_TO_SECTOR_SECOND = {
    "a": 0,
    "s": 1,
    "d": 2,
    "f": 3,
    "g": 4,
    "h": 5,
    "j": 6,
    "k": 7,
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time keyboard -> simulated CAN event sender")
    p.add_argument("--channel", default="slcan0", help="CAN channel (default: slcan0)")
    p.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    p.add_argument("--device-id", type=int, default=2, help="main key-bank source device id 0..127")
    p.add_argument("--device-id-2", type=int, default=3, help="second key-bank source device id 0..127")
    p.add_argument("--elev", type=int, default=180, help="event-state elevation 0..255")
    p.add_argument("--intensity", type=int, default=180, help="event intensity p1 0..255")
    p.add_argument("--interval-ms", type=int, default=20, help="repeat EVENT_STATE while sector>0")
    return p.parse_args()


def clamp_u8(v: int) -> int:
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def send_event(bus: can.BusABC, status_id: int, ev: int, p0: int = 0, p1: int = 0, p2: int = 0) -> None:
    stamp = int(time.monotonic() * 1000.0) & 0xFFFF
    data = bytearray(8)
    data[0] = 0
    data[1] = FRAME_EVENT
    data[2] = ev & 0xFF
    data[3] = p0 & 0xFF
    data[4] = p1 & 0xFF
    data[5] = p2 & 0xFF
    data[6] = stamp & 0xFF
    data[7] = (stamp >> 8) & 0xFF
    bus.send(can.Message(arbitration_id=status_id, data=data, is_extended_id=False))


def send_event_state(bus: can.BusABC, status_id: int, sector: int, elev: int) -> None:
    data = bytearray(8)
    data[0] = 0
    data[1] = FRAME_EVENT_STATE
    data[2] = sector & 0xFF
    data[3] = elev & 0xFF
    bus.send(can.Message(arbitration_id=status_id, data=data, is_extended_id=False))


def apply_sector(bus: can.BusABC, status_id: int, prev_sector: int, new_sector: int, intensity: int, elev: int) -> int:
    if new_sector == prev_sector:
        # Keep it alive; useful for repeated key presses.
        if new_sector > 0:
            send_event(bus, status_id, EVENT_INTENSITY_CHANGE, p0=new_sector, p1=intensity, p2=0)
            send_event_state(bus, status_id, new_sector, elev)
        return prev_sector

    if new_sector == 0:
        if prev_sector > 0:
            send_event(bus, status_id, EVENT_SECTION_DEACTIVATED, p0=prev_sector, p1=0, p2=0)
        send_event_state(bus, status_id, 0, 0)
        return 0

    if prev_sector == 0:
        send_event(bus, status_id, EVENT_SECTOR_ACTIVATED, p0=new_sector, p1=intensity, p2=0)
    else:
        send_event(bus, status_id, EVENT_SECTOR_CHANGED, p0=prev_sector, p1=new_sector, p2=0)

    send_event_state(bus, status_id, new_sector, elev)
    return new_sector


def main() -> int:
    args = parse_args()
    if args.device_id < 0 or args.device_id > 0x7F:
        print("--device-id must be 0..127")
        return 2
    if args.device_id_2 < 0 or args.device_id_2 > 0x7F:
        print("--device-id-2 must be 0..127")
        return 2

    elev = clamp_u8(int(args.elev))
    intensity = clamp_u8(int(args.intensity))
    heartbeat_s = max(0.0, float(args.interval_ms) / 1000.0)
    status_id_main = 0x580 + int(args.device_id)
    status_id_second = 0x580 + int(args.device_id_2)

    bus = can.Bus(interface=args.interface, channel=args.channel, receive_own_messages=False)

    print("rhytmics_keysim running")
    print(
        f"channel={args.channel} interface={args.interface} "
        f"dev1={args.device_id}(0x{status_id_main:03X}) dev2={args.device_id_2}(0x{status_id_second:03X})"
    )
    print("keys1: q=0, w=1, e=2, r=3, t=4, z=5, u=6")
    print("keys2: a=0, s=1, d=2, f=3, g=4, h=5, j=6, k=7")
    print("exit: x / Ctrl+C")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    prev_sector_by_status = {
        status_id_main: 0,
        status_id_second: 0,
    }
    last_state_tx_by_status = {
        status_id_main: 0.0,
        status_id_second: 0.0,
    }

    try:
        tty.setcbreak(fd)
        while True:
            now = time.monotonic()
            if heartbeat_s > 0.0:
                for sid, sec in prev_sector_by_status.items():
                    last_tx = last_state_tx_by_status[sid]
                    if sec > 0 and (now - last_tx) >= heartbeat_s:
                        send_event_state(bus, sid, sec, elev)
                        last_state_tx_by_status[sid] = now

            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not rlist:
                continue

            ch = sys.stdin.read(1).lower()
            if ch in ("\x03", "\x04", "x"):  # Ctrl-C, Ctrl-D, x
                break
            if ch in KEY_TO_SECTOR_MAIN:
                sid = status_id_main
                new_sector = KEY_TO_SECTOR_MAIN[ch]
            elif ch in KEY_TO_SECTOR_SECOND:
                sid = status_id_second
                new_sector = KEY_TO_SECTOR_SECOND[ch]
            else:
                continue

            prev_sector_by_status[sid] = apply_sector(
                bus,
                sid,
                prev_sector_by_status[sid],
                new_sector,
                intensity,
                elev,
            )
            last_state_tx_by_status[sid] = time.monotonic()
            dev = (sid - 0x580) & 0x7F
            print(f"key={ch} dev={dev} sector={new_sector}")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
