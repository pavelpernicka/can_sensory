#!/usr/bin/env python3
import argparse
import math
import sys
import time
from collections import deque
import can
from typing import Callable

CMD_PING = 0x01
CMD_ENTER_BOOTLOADER = 0x40
CMD_WS_SET_POWER = 0x50
CMD_WS_SET_BRIGHTNESS = 0x51
CMD_WS_SET_COLOR = 0x52
CMD_WS_SET_ALL = 0x53
CMD_WS_GET_STATE = 0x54
CMD_WS_SET_ANIM = 0x55
CMD_WS_GET_ANIM = 0x56
CMD_WS_SET_GRADIENT = 0x57
CMD_WS_GET_GRADIENT = 0x58
CMD_WS_SET_SECTOR_COLOR = 0x59
CMD_WS_GET_SECTOR_COLOR = 0x5A
CMD_WS_SET_SECTOR_MODE = 0x5B
CMD_WS_GET_SECTOR_MODE = 0x5C
CMD_WS_SET_SECTOR_ZONE = 0x5D
CMD_WS_GET_SECTOR_ZONE = 0x5E
CMD_WS_SET_LENGTH = 0x5F
CMD_WS_GET_LENGTH = 0x60
CMD_WS_SET_ACTIVE_SECTOR = 0x61
CMD_HMC_SET_CFG = 0x6E
CMD_HMC_GET_CFG = 0x6F
CMD_SET_INTERVAL = 0x70
CMD_GET_INTERVAL = 0x71
CMD_SET_STREAM_ENABLE = 0x72
CMD_GET_STATUS = 0x73
CMD_AHT20_READ = 0x74
CMD_CALIB_GET = 0x79
CMD_CALIB_SET = 0x7A
CMD_CALIB_SAVE = 0x7B
CMD_CALIB_LOAD = 0x7C
CMD_CALIB_RESET = 0x7D
CMD_CALIB_CAPTURE_EARTH = 0x7E

STATUS_OK = 0x00
STATUS_ERR_GENERIC = 0x01
STATUS_ERR_RANGE = 0x02
STATUS_ERR_STATE = 0x03
STATUS_ERR_SENSOR = 0x04

FRAME_PONG = 0x01
FRAME_STARTUP = 0x02
FRAME_MAG = 0x10
FRAME_ACC = 0x11
FRAME_ENV = 0x12
FRAME_EVENT = 0x20
FRAME_INTERVAL = 0x30
FRAME_STATUS = 0x31
FRAME_EVENT_STATE = 0x32
FRAME_AHT20_MEAS = 0x40
FRAME_AHT20_RAW = 0x41
FRAME_AHT20_STATUS = 0x42
FRAME_AHT20_REG = 0x43
FRAME_CALIB_VALUE = 0x44
FRAME_CALIB_INFO = 0x45
FRAME_HMC_CFG = 0x46
FRAME_WS_STATE = 0x47
FRAME_WS_ANIM = 0x48
FRAME_WS_GRADIENT = 0x49
FRAME_WS_SECTOR_COLOR = 0x4A
FRAME_WS_SECTOR_MODE = 0x4B
FRAME_WS_SECTOR_ZONE = 0x4C
FRAME_WS_LENGTH = 0x4D

EVENT_NAMES = {
    1: "SECTOR_ACTIVATED",
    2: "SECTOR_CHANGED",
    3: "INTENSITY_CHANGE",
    4: "SECTION_DEACTIVATED",
    5: "SESSION_STARTED",
    6: "SESSION_ENDED",
    7: "PASSING_SECTOR_CHANGE",
    8: "POSSIBLE_MECHANICAL_FAILURE",
    9: "ERROR_NO_DATA",
}

STREAM_NAME_TO_ID = {
    "mag": 1,
    "acc": 2,
    "env": 3,
    "event": 4,
    "all": 0,
}

STREAM_ID_TO_NAME = {
    1: "mag",
    2: "acc",
    3: "env",
    4: "event",
}

STATUS_TEXT = {
    STATUS_OK: "OK",
    STATUS_ERR_GENERIC: "ERR_GENERIC",
    STATUS_ERR_RANGE: "ERR_RANGE",
    STATUS_ERR_STATE: "ERR_STATE",
    STATUS_ERR_SENSOR: "ERR_SENSOR",
}

FRAME_TYPES = {
    FRAME_PONG,
    FRAME_STARTUP,
    FRAME_MAG,
    FRAME_ACC,
    FRAME_ENV,
    FRAME_EVENT,
    FRAME_INTERVAL,
    FRAME_STATUS,
    FRAME_EVENT_STATE,
    FRAME_AHT20_MEAS,
    FRAME_AHT20_RAW,
    FRAME_AHT20_STATUS,
    FRAME_AHT20_REG,
    FRAME_CALIB_VALUE,
    FRAME_CALIB_INFO,
    FRAME_HMC_CFG,
    FRAME_WS_STATE,
    FRAME_WS_ANIM,
    FRAME_WS_GRADIENT,
    FRAME_WS_SECTOR_COLOR,
    FRAME_WS_SECTOR_MODE,
    FRAME_WS_SECTOR_ZONE,
    FRAME_WS_LENGTH,
}

CMD_REPLY_FRAME_TYPES = {
    FRAME_PONG,
    FRAME_INTERVAL,
    FRAME_STATUS,
    FRAME_AHT20_MEAS,
    FRAME_AHT20_RAW,
    FRAME_AHT20_STATUS,
    FRAME_AHT20_REG,
    FRAME_CALIB_VALUE,
    FRAME_CALIB_INFO,
    FRAME_HMC_CFG,
    FRAME_WS_STATE,
    FRAME_WS_ANIM,
    FRAME_WS_GRADIENT,
    FRAME_WS_SECTOR_COLOR,
    FRAME_WS_SECTOR_MODE,
    FRAME_WS_SECTOR_ZONE,
    FRAME_WS_LENGTH,
}

WS_ANIM_NAME_TO_ID = {
    "static": 0,
    "blink": 1,
    "breathe": 2,
    "rainbow": 3,
    "wipe": 4,
    "gradient": 5,
    "sector-follow": 6,
    "sector_follow": 6,
    "sector": 6,
}
WS_ANIM_ID_TO_NAME = {
    0: "static",
    1: "blink",
    2: "breathe",
    3: "rainbow",
    4: "wipe",
    5: "gradient",
    6: "sector-follow",
}

CAL_FIELD_NAME_TO_ID = {
    "center_x": 1,
    "center_y": 2,
    "center_z": 3,
    "rotate_xy": 4,
    "rotate_xz": 5,
    "rotate_yz": 6,
    "keepout_rad": 7,
    "z_limit": 8,
    "data_radius": 9,
    "mag_offset_x": 10,
    "mag_offset_y": 11,
    "mag_offset_z": 12,
    "earth_x": 13,
    "earth_y": 14,
    "earth_z": 15,
    "earth_valid": 16,
    "num_sectors": 17,
    "z_max": 18,
    "elev_curve": 19,
    "all": 0,
}

CAL_FIELD_ID_TO_NAME = {v: k for k, v in CAL_FIELD_NAME_TO_ID.items() if v != 0}

HMC_RANGE_ID_TO_LABEL = {
    0: "0.88",
    1: "1.3",
    2: "1.9",
    3: "2.5",
    4: "4.0",
    5: "4.7",
    6: "5.6",
    7: "8.1",
}

HMC_DATA_RATE_ID_TO_HZ = {
    0: "0.75",
    1: "1.5",
    2: "3",
    3: "7.5",
    4: "15",
    5: "30",
    6: "75",
}

HMC_SAMPLES_ID_TO_COUNT = {
    0: "1",
    1: "2",
    2: "4",
    3: "8",
}

HMC_MODE_ID_TO_NAME = {
    0: "continuous",
    1: "single",
    2: "idle",
}


class AppCanClient:
    def __init__(self, channel: str, interface: str, device_id: int, timeout: float):
        self.channel = channel
        self.interface = interface
        self.timeout = timeout
        self.device_id = 0
        self.cmd_id = 0
        self.status_id = 0
        self._set_device_id_local(device_id)

        self.bus = self._open_bus_for_status_id(self.status_id)
        self._pending_frames = deque()

    def _set_device_id_local(self, device_id: int) -> None:
        if device_id < 0 or device_id > 0x7F:
            raise ValueError("device-id must be 0..127")
        self.device_id = device_id
        self.cmd_id = 0x600 + device_id
        self.status_id = 0x580 + device_id

    def _open_bus_for_status_id(self, status_id: int):
        return can.Bus(
            interface=self.interface,
            channel=self.channel,
            receive_own_messages=False,
            can_filters=[{
                "can_id": status_id,
                "can_mask": 0x7FF,
                "extended": False,
            }],
        )

    def close(self) -> None:
        self.bus.shutdown()

    def set_device_id(self, device_id: int) -> None:
        if device_id == self.device_id:
            return
        self._set_device_id_local(device_id)
        new_bus = self._open_bus_for_status_id(self.status_id)
        old_bus = self.bus
        self.bus = new_bus
        self._pending_frames.clear()
        old_bus.shutdown()

    def send(self, payload: bytes) -> None:
        if len(payload) > 8:
            raise ValueError("CAN payload must be <= 8 bytes")
        msg = can.Message(
            arbitration_id=self.cmd_id,
            is_extended_id=False,
            data=payload,
        )
        self.bus.send(msg)

    def send_command(self, payload: bytes) -> None:
        # Drop stale streamed frames on the command socket so we don't parse old backlog
        # before the current command response.
        self.flush_pending_rx()
        self.send(payload)

    def recv(self, timeout: float | None = None):
        return self.bus.recv(self.timeout if timeout is None else timeout)

    def flush_pending_rx(self, max_frames: int = 1024, max_ms: float = 30.0) -> int:
        self._pending_frames.clear()
        dropped = 0
        t0 = time.monotonic()
        while dropped < max_frames:
            if (time.monotonic() - t0) * 1000.0 >= max_ms:
                break
            msg = self.recv(0.0)
            if msg is None:
                break
            dropped += 1
        return dropped

    def _next_frame(self, timeout_s: float) -> bytes | None:
        if self._pending_frames:
            return self._pending_frames.popleft()
        msg = self.recv(timeout_s)
        if msg is None:
            return None
        return bytes(msg.data)

    @staticmethod
    def _is_possible_cmd_reply(data: bytes) -> bool:
        if len(data) >= 4 and data[:4] == b"PONG":
            return True
        if len(data) >= 2 and data[0] == 0 and data[1] in CMD_REPLY_FRAME_TYPES:
            return True
        return False

    @staticmethod
    def is_status_reply(data: bytes) -> bool:
        if len(data) < 2:
            return False
        if data[0] > STATUS_ERR_SENSOR:
            return False
        return all(b == 0 for b in data[2:])

    def wait_status(self, expected_extra: int, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        deferred = deque()
        last_mismatch_err: tuple[int, int] | None = None
        while time.monotonic() < deadline:
            data = self._next_frame(max(0.0, deadline - time.monotonic()))
            if data is None:
                continue
            if not self.is_status_reply(data):
                # Keep only potentially useful command-reply frames; drop high-rate stream traffic.
                if self._is_possible_cmd_reply(data):
                    if len(deferred) < 128:
                        deferred.append(data)
                continue

            if data[1] != expected_extra:
                if data[0] != STATUS_OK:
                    last_mismatch_err = (int(data[0]), int(data[1]))
                continue

            st = data[0]
            if st != STATUS_OK:
                raise RuntimeError(
                    f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"
                )
            if deferred:
                self._pending_frames.extendleft(reversed(deferred))
            return data

        if deferred:
            self._pending_frames.extendleft(reversed(deferred))
        if last_mismatch_err is not None:
            st, extra = last_mismatch_err
            raise TimeoutError(
                f"Timeout waiting for status extra=0x{expected_extra:02X}; "
                f"last status was {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{extra:02X}"
            )
        raise TimeoutError(f"Timeout waiting for status extra=0x{expected_extra:02X}")

    def wait_frame(self, predicate: Callable[[bytes], bool], timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            data = self._next_frame(max(0.0, deadline - time.monotonic()))
            if data is None:
                continue

            if self.is_status_reply(data):
                st = data[0]
                if st != STATUS_OK:
                    raise RuntimeError(
                        f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"
                    )
                continue

            if predicate(data):
                return data

        raise TimeoutError("Timeout waiting for frame")

    def ping(self) -> bytes:
        self.send_command(bytes([CMD_PING]))
        self.wait_status(expected_extra=0x01)
        return self.wait_frame(lambda d: len(d) >= 4 and d[:4] == b"PONG")

    def enter_bootloader(self) -> None:
        self.send_command(bytes([CMD_ENTER_BOOTLOADER]))
        self.wait_status(expected_extra=0x40)

    def hmc_get_config(self) -> dict:
        self.send_command(bytes([CMD_HMC_GET_CFG]))
        self.wait_status(expected_extra=0x6F)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_HMC_CFG)
        return parse_hmc_cfg_frame(frame)

    def hmc_set_config(self, range_id: int, data_rate_id: int, samples_id: int, mode_id: int) -> dict:
        if range_id < 0 or range_id > 7:
            raise ValueError("hmc range must be 0..7")
        if data_rate_id < 0 or data_rate_id > 6:
            raise ValueError("hmc data-rate must be 0..6")
        if samples_id < 0 or samples_id > 3:
            raise ValueError("hmc samples must be 0..3")
        if mode_id < 0 or mode_id > 2:
            raise ValueError("hmc mode must be 0..2")

        self.send_command(bytes([CMD_HMC_SET_CFG, range_id, data_rate_id, samples_id, mode_id]))
        self.wait_status(expected_extra=0x6E)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_HMC_CFG)
        return parse_hmc_cfg_frame(frame)

    def set_interval(self, stream_id: int, interval_ms: int) -> dict:
        if stream_id < 1 or stream_id > 4:
            raise ValueError("stream-id must be 1..4")
        if interval_ms < 0 or interval_ms > 60000:
            raise ValueError("interval must be 0..60000 ms")

        self.send_command(bytes([
            CMD_SET_INTERVAL,
            stream_id,
            interval_ms & 0xFF,
            (interval_ms >> 8) & 0xFF,
        ]))
        self.wait_status(expected_extra=stream_id)

        frame = self.wait_frame(
            lambda d: len(d) >= 6 and d[0] == 0 and d[1] == FRAME_INTERVAL and d[2] == stream_id
        )
        return parse_interval_frame(frame)

    def set_stream_enable(self, stream_id: int, enable: bool) -> dict:
        if stream_id < 1 or stream_id > 4:
            raise ValueError("stream-id must be 1..4")

        self.send_command(bytes([CMD_SET_STREAM_ENABLE, stream_id, 1 if enable else 0]))
        self.wait_status(expected_extra=stream_id)

        frame = self.wait_frame(
            lambda d: len(d) >= 6 and d[0] == 0 and d[1] == FRAME_INTERVAL and d[2] == stream_id
        )
        return parse_interval_frame(frame)

    def get_intervals(self, stream_id: int) -> list[dict]:
        if stream_id < 0 or stream_id > 4:
            raise ValueError("stream-id must be 0..4")

        self.send_command(bytes([CMD_GET_INTERVAL, stream_id]))

        expected_count = 4 if stream_id == 0 else 1
        results: dict[int, dict] = {}
        deadline = time.monotonic() + self.timeout

        while len(results) < expected_count and time.monotonic() < deadline:
            frame = self.wait_frame(
                lambda d: len(d) >= 6 and d[0] == 0 and d[1] == FRAME_INTERVAL,
                timeout=max(0.0, deadline - time.monotonic()),
            )
            parsed = parse_interval_frame(frame)
            sid = parsed["stream_id"]
            if stream_id == 0 or sid == stream_id:
                results[sid] = parsed

        if len(results) < expected_count:
            raise TimeoutError("Timeout waiting for interval response frames")

        return [results[sid] for sid in sorted(results.keys())]

    def get_status(self) -> dict:
        self.send_command(bytes([CMD_GET_STATUS]))
        self.wait_status(expected_extra=0x73)

        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_STATUS)
        return parse_status_frame(frame)

    def ws_get_state(self) -> dict:
        self.send_command(bytes([CMD_WS_GET_STATE]))
        self.wait_status(expected_extra=0x54)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_STATE)
        return parse_ws_state_frame(frame)

    def ws_set_power(self, enabled: bool) -> dict:
        self.send_command(bytes([CMD_WS_SET_POWER, 1 if enabled else 0]))
        self.wait_status(expected_extra=0x50)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_STATE)
        return parse_ws_state_frame(frame)

    def ws_set_brightness(self, brightness: int) -> dict:
        if brightness < 0 or brightness > 255:
            raise ValueError("brightness must be 0..255")
        self.send_command(bytes([CMD_WS_SET_BRIGHTNESS, brightness]))
        self.wait_status(expected_extra=0x51)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_STATE)
        return parse_ws_state_frame(frame)

    def ws_set_color(self, r: int, g: int, b: int) -> dict:
        for n, v in (("r", r), ("g", g), ("b", b)):
            if v < 0 or v > 255:
                raise ValueError(f"{n} must be 0..255")
        self.send_command(bytes([CMD_WS_SET_COLOR, r, g, b]))
        self.wait_status(expected_extra=0x52)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_STATE)
        return parse_ws_state_frame(frame)

    def ws_set_all(self, enabled: bool, brightness: int, r: int, g: int, b: int) -> dict:
        for n, v in (("brightness", brightness), ("r", r), ("g", g), ("b", b)):
            if v < 0 or v > 255:
                raise ValueError(f"{n} must be 0..255")
        self.send_command(bytes([CMD_WS_SET_ALL, 1 if enabled else 0, brightness, r, g, b]))
        self.wait_status(expected_extra=0x53)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_STATE)
        return parse_ws_state_frame(frame)

    def ws_get_anim(self) -> dict:
        self.send_command(bytes([CMD_WS_GET_ANIM]))
        self.wait_status(expected_extra=0x56)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_WS_ANIM)
        return parse_ws_anim_frame(frame)

    def ws_set_anim(self, mode: int, speed: int) -> dict:
        if mode < 0 or mode > 5:
            raise ValueError("animation mode must be 0..5")
        if speed < 0 or speed > 255:
            raise ValueError("animation speed must be 0..255")
        self.send_command(bytes([CMD_WS_SET_ANIM, mode, speed]))
        self.wait_status(expected_extra=0x55)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_WS_ANIM)
        return parse_ws_anim_frame(frame)

    def ws_get_gradient(self) -> dict:
        self.send_command(bytes([CMD_WS_GET_GRADIENT]))
        self.wait_status(expected_extra=0x58)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_GRADIENT)
        return parse_ws_gradient_frame(frame)

    def ws_set_gradient(self, split_idx: int, fade_px: int, c1_rgb565: int, c2_rgb565: int) -> dict:
        if split_idx < 1 or split_idx > 255:
            raise ValueError("split index must be 1..255")
        if fade_px < 0 or fade_px > 255:
            raise ValueError("fade pixels must be 0..255")
        if c1_rgb565 < 0 or c1_rgb565 > 0xFFFF or c2_rgb565 < 0 or c2_rgb565 > 0xFFFF:
            raise ValueError("colors must be RGB565 (0..65535)")
        self.send_command(bytes([
            CMD_WS_SET_GRADIENT,
            split_idx & 0xFF,
            fade_px & 0xFF,
            c1_rgb565 & 0xFF, (c1_rgb565 >> 8) & 0xFF,
            c2_rgb565 & 0xFF, (c2_rgb565 >> 8) & 0xFF,
        ]))
        self.wait_status(expected_extra=0x57)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_GRADIENT)
        return parse_ws_gradient_frame(frame)

    def ws_set_sector_color(self, idx: int, r: int, g: int, b: int) -> dict:
        if idx < 1 or idx > 8:
            raise ValueError("sector index must be 1..8")
        for n, v in (("r", r), ("g", g), ("b", b)):
            if v < 0 or v > 255:
                raise ValueError(f"{n} must be 0..255")
        self.send_command(bytes([CMD_WS_SET_SECTOR_COLOR, idx, r, g, b]))
        self.wait_status(expected_extra=0x59)
        frame = self.wait_frame(
            lambda d: len(d) >= 7 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_COLOR and d[2] == idx
        )
        return parse_ws_sector_color_frame(frame)

    def ws_get_sector_color(self, idx: int = 0) -> list[dict]:
        if idx < 0 or idx > 8:
            raise ValueError("sector index must be 0..8")
        self.send_command(bytes([CMD_WS_GET_SECTOR_COLOR, idx]))
        self.wait_status(expected_extra=0x5A)

        if idx != 0:
            frame = self.wait_frame(
                lambda d: len(d) >= 7 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_COLOR and d[2] == idx
            )
            return [parse_ws_sector_color_frame(frame)]

        out: dict[int, dict] = {}
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline and len(out) < 8:
            frame = self._next_frame(timeout_s=min(0.05, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            if self.is_status_reply(frame):
                st = frame[0]
                if st != STATUS_OK:
                    raise RuntimeError(
                        f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{frame[1]:02X}"
                    )
                continue
            if len(frame) < 7 or frame[0] != 0 or frame[1] != FRAME_WS_SECTOR_COLOR:
                continue
            parsed = parse_ws_sector_color_frame(frame)
            out[int(parsed["idx"])] = parsed
        if not out:
            raise TimeoutError("Timeout waiting for WS sector color frame(s)")
        return [out[k] for k in sorted(out.keys())]

    def ws_set_sector_mode(self, enabled: bool, fade_speed: int, sector_count: int) -> dict:
        if fade_speed < 0 or fade_speed > 255:
            raise ValueError("fade speed must be 0..255")
        if sector_count < 1 or sector_count > 255:
            raise ValueError("sector count must be 1..255")
        self.send_command(bytes([CMD_WS_SET_SECTOR_MODE, 1 if enabled else 0, fade_speed, sector_count]))
        self.wait_status(expected_extra=0x5B)
        frame = self.wait_frame(lambda d: len(d) >= 7 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_MODE)
        return parse_ws_sector_mode_frame(frame)

    def ws_get_sector_mode(self) -> dict:
        self.send_command(bytes([CMD_WS_GET_SECTOR_MODE]))
        self.wait_status(expected_extra=0x5C)
        frame = self.wait_frame(lambda d: len(d) >= 7 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_MODE)
        return parse_ws_sector_mode_frame(frame)

    def ws_set_sector_zone(self, idx: int, pos_led: int, color_rgb565: int, readback: bool = False) -> dict:
        if idx < 1 or idx > 32:
            raise ValueError("zone index must be 1..32")
        if pos_led < 0 or pos_led > 255:
            raise ValueError("pos_led must be 0..255 (0 disables stop)")
        if color_rgb565 < 0 or color_rgb565 > 0xFFFF:
            raise ValueError("color must be RGB565 (0..65535)")
        self.send_command(bytes([
            CMD_WS_SET_SECTOR_ZONE,
            idx & 0xFF,
            pos_led & 0xFF,
            color_rgb565 & 0xFF,
            (color_rgb565 >> 8) & 0xFF,
        ]))
        self.wait_status(expected_extra=0x5D)
        if readback:
            frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_ZONE and d[2] == idx)
            return parse_ws_sector_zone_frame(frame)
        return {
            "idx": int(idx),
            "pos_led": int(pos_led),
            "color_rgb565": int(color_rgb565),
        }

    def ws_get_sector_zone(self, idx: int = 0) -> list[dict]:
        if idx < 0 or idx > 32:
            raise ValueError("zone index must be 0..32")
        self.send_command(bytes([CMD_WS_GET_SECTOR_ZONE, idx & 0xFF]))
        self.wait_status(expected_extra=0x5E)

        if idx != 0:
            frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_ZONE and d[2] == idx)
            return [parse_ws_sector_zone_frame(frame)]

        out: dict[int, dict] = {}
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline and len(out) < 32:
            frame = self._next_frame(timeout_s=min(0.05, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            if self.is_status_reply(frame):
                st = frame[0]
                if st != STATUS_OK:
                    raise RuntimeError(
                        f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{frame[1]:02X}"
                    )
                continue
            if len(frame) < 8 or frame[0] != 0 or frame[1] != FRAME_WS_SECTOR_ZONE:
                continue
            parsed = parse_ws_sector_zone_frame(frame)
            out[int(parsed["idx"])] = parsed
        if not out:
            raise TimeoutError("Timeout waiting for WS sector zone frame(s)")
        return [out[k] for k in sorted(out.keys())]

    def ws_set_length(self, strip_len: int) -> dict:
        if strip_len < 1 or strip_len > 255:
            raise ValueError("strip length must be 1..255")
        self.send_command(bytes([CMD_WS_SET_LENGTH, strip_len & 0xFF]))
        self.wait_status(expected_extra=0x5F)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_WS_LENGTH)
        return parse_ws_length_frame(frame)

    def ws_get_length(self) -> dict:
        self.send_command(bytes([CMD_WS_GET_LENGTH]))
        self.wait_status(expected_extra=0x60)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_WS_LENGTH)
        return parse_ws_length_frame(frame)

    def ws_set_active_sector(self, sector: int, readback: bool = False) -> dict:
        if sector < 0 or sector > 255:
            raise ValueError("active sector must be 0..255 (0 disables CAN override)")
        self.send_command(bytes([CMD_WS_SET_ACTIVE_SECTOR, sector & 0xFF]))
        self.wait_status(expected_extra=0x61)
        if readback:
            frame = self.wait_frame(lambda d: len(d) >= 7 and d[0] == 0 and d[1] == FRAME_WS_SECTOR_MODE)
            return parse_ws_sector_mode_frame(frame)
        return {
            "active_sector": int(sector),
            "override_enabled": int(sector != 0),
        }

    def aht20_read(self) -> dict:
        self.send_command(bytes([CMD_AHT20_READ]))
        self.wait_status(expected_extra=0x74)

        meas = None
        raw = None
        deadline = time.monotonic() + self.timeout

        while (meas is None or raw is None) and time.monotonic() < deadline:
            frame = self.wait_frame(
                lambda d: len(d) >= 2 and d[0] == 0 and d[1] in (FRAME_AHT20_MEAS, FRAME_AHT20_RAW),
                timeout=max(0.0, deadline - time.monotonic()),
            )
            if frame[1] == FRAME_AHT20_MEAS:
                meas = parse_aht20_meas_frame(frame)
            elif frame[1] == FRAME_AHT20_RAW:
                raw = parse_aht20_raw_frame(frame)

        if meas is None or raw is None:
            raise TimeoutError("Timeout waiting for AHT20 read frames")

        return {
            **meas,
            **raw,
        }

    def calib_get(self, field_id: int = 0) -> list[dict]:
        if field_id < 0 or field_id > 19:
            raise ValueError("calib field-id must be 0..19")

        self.send_command(bytes([CMD_CALIB_GET, field_id]))
        if field_id == 0:
            self.wait_status(expected_extra=0x79)
            out: dict[int, dict] = {}
            deadline = time.monotonic() + self.timeout
            last_calib_s = 0.0

            while time.monotonic() < deadline:
                now_s = time.monotonic()
                if out and (now_s - last_calib_s) > 0.80:
                    # Calibration reply burst ended.
                    break

                frame = self._next_frame(timeout_s=min(0.05, max(0.0, deadline - now_s)))
                if frame is None:
                    continue

                # Keep existing error handling semantics for async status frames.
                if self.is_status_reply(frame):
                    st = frame[0]
                    if st != STATUS_OK:
                        raise RuntimeError(
                            f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{frame[1]:02X}"
                        )
                    continue

                if len(frame) >= 5 and frame[0] == 0 and frame[1] == FRAME_CALIB_VALUE:
                    parsed = parse_calib_value_frame(frame)
                    out[parsed["field_id"]] = parsed
                    last_calib_s = time.monotonic()
                    if len(out) >= len(CAL_FIELD_ID_TO_NAME):
                        break

            if not out:
                raise TimeoutError("Timeout waiting for calibration value frame(s)")
            return [out[fid] for fid in sorted(out.keys())]

        self.wait_status(expected_extra=field_id)
        frame = self.wait_frame(
            lambda d: len(d) >= 5 and d[0] == 0 and d[1] == FRAME_CALIB_VALUE and d[2] == field_id
        )
        return [parse_calib_value_frame(frame)]

    def calib_set(self, field_id: int, value: int) -> dict:
        if field_id < 1 or field_id > 19:
            raise ValueError("calib field-id must be 1..19")
        if value < -32768 or value > 32767:
            raise ValueError("calib value must be int16 range")

        payload = bytes([
            CMD_CALIB_SET,
            field_id,
            value & 0xFF,
            (value >> 8) & 0xFF,
        ])
        self.send_command(payload)
        self.wait_status(expected_extra=field_id)
        frame = self.wait_frame(
            lambda d: len(d) >= 5 and d[0] == 0 and d[1] == FRAME_CALIB_VALUE and d[2] == field_id
        )
        return parse_calib_value_frame(frame)

    def calib_save(self) -> dict:
        self.send_command(bytes([CMD_CALIB_SAVE]))
        self.wait_status(expected_extra=0x7B)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_load(self) -> dict:
        self.send_command(bytes([CMD_CALIB_LOAD]))
        self.wait_status(expected_extra=0x7C)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_reset(self) -> dict:
        self.send_command(bytes([CMD_CALIB_RESET]))
        self.wait_status(expected_extra=0x7D)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_capture_earth(self) -> dict:
        self.send_command(bytes([CMD_CALIB_CAPTURE_EARTH]))
        self.wait_status(expected_extra=0x7E)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)


def decode_sensor_bits(bits: int) -> list[str]:
    out = []
    if bits & (1 << 0):
        out.append("hmc")
    if bits & (1 << 1):
        out.append("lis")
    if bits & (1 << 2):
        out.append("aht")
    return out


def status_id_to_device_id(arbitration_id: int) -> int | None:
    if arbitration_id < 0x580 or arbitration_id > 0x5FF:
        return None
    return arbitration_id - 0x580


def discover_devices(
    channel: str,
    interface: str = "socketcan",
    timeout: float = 0.35,
    probe_ids: list[int] | None = None,
) -> list[dict]:
    if timeout <= 0.0:
        timeout = 0.01

    ids = list(range(128)) if probe_ids is None else [i for i in probe_ids if 0 <= i <= 0x7F]
    bus = can.Bus(
        interface=interface,
        channel=channel,
        receive_own_messages=False,
        can_filters=[{
            "can_id": 0x580,
            "can_mask": 0x780,
            "extended": False,
        }],
    )
    found: dict[int, dict] = {}

    try:
        for did in ids:
            bus.send(can.Message(arbitration_id=0x600 + did, is_extended_id=False, data=bytes([CMD_PING])))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = bus.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue

            did = status_id_to_device_id(msg.arbitration_id)
            if did is None:
                continue

            data = bytes(msg.data)
            rec = found.setdefault(did, {
                "device_id": did,
                "seen_frames": 0,
                "ping_ok": False,
                "pong": False,
                "proto": None,
                "sensors": [],
                "streams": [],
            })
            rec["seen_frames"] += 1

            if AppCanClient.is_status_reply(data) and len(data) >= 2 and data[0] == STATUS_OK and data[1] == CMD_PING:
                rec["ping_ok"] = True
            elif len(data) >= 4 and data[:4] == b"PONG":
                rec["pong"] = True
                if len(data) >= 6:
                    rec["proto"] = int(data[5])
            elif len(data) >= 8 and data[0] == 0 and data[1] == FRAME_STARTUP:
                rec["proto"] = int(data[3])
                rec["sensors"] = decode_sensor_bits(data[4])
                rec["streams"] = decode_stream_bits(data[5])

        return [found[k] for k in sorted(found.keys())]
    finally:
        bus.shutdown()


def decode_stream_bits(bits: int) -> list[str]:
    out = []
    if bits & (1 << 0):
        out.append("mag")
    if bits & (1 << 1):
        out.append("acc")
    if bits & (1 << 2):
        out.append("env")
    if bits & (1 << 3):
        out.append("event")
    return out


def parse_interval_frame(data: bytes) -> dict:
    stream_id = data[2]
    enabled = bool(data[3])
    interval_ms = int.from_bytes(data[4:6], "little")
    return {
        "stream_id": stream_id,
        "stream_name": STREAM_ID_TO_NAME.get(stream_id, f"{stream_id}"),
        "enabled": enabled,
        "interval_ms": interval_ms,
    }


def parse_status_frame(data: bytes) -> dict:
    return {
        "sensor_bits": data[2],
        "stream_bits": data[3],
        "sensors": decode_sensor_bits(data[2]),
        "streams": decode_stream_bits(data[3]),
        "interval_low_bytes": {
            "mag": data[4],
            "acc": data[5],
            "env": data[6],
            "event": data[7],
        },
    }


def parse_ws_state_frame(data: bytes) -> dict:
    return {
        "enabled": bool(data[2]),
        "brightness": int(data[3]),
        "r": int(data[4]),
        "g": int(data[5]),
        "b": int(data[6]),
        "strip_len": int(data[7]),
    }


def parse_ws_anim_frame(data: bytes) -> dict:
    mode = int(data[2])
    speed = int(data[3])
    return {
        "mode": mode,
        "mode_name": WS_ANIM_ID_TO_NAME.get(mode, f"{mode}"),
        "speed": speed,
    }


def parse_ws_gradient_frame(data: bytes) -> dict:
    return {
        "split_idx": int(data[2]),
        "fade_px": int(data[3]),
        "color1_rgb565": int.from_bytes(data[4:6], "little"),
        "color2_rgb565": int.from_bytes(data[6:8], "little"),
    }


def parse_ws_sector_color_frame(data: bytes) -> dict:
    return {
        "idx": int(data[2]),
        "r": int(data[3]),
        "g": int(data[4]),
        "b": int(data[5]),
        "max_sectors": int(data[6]),
    }


def parse_ws_sector_mode_frame(data: bytes) -> dict:
    return {
        "enabled": bool(data[2]),
        "fade_speed": int(data[3]),
        "sector_count": int(data[4]),
        "active_sector": int(data[5]),
        "target_sector": int(data[6]),
        "max_zones": int(data[7]) if len(data) >= 8 else 0,
    }


def parse_ws_sector_zone_frame(data: bytes) -> dict:
    return {
        "idx": int(data[2]),
        "pos_led": int(data[3]),
        "color_rgb565": int.from_bytes(data[4:6], "little"),
        "strip_len": int(data[6]) if len(data) >= 7 else 0,
        "max_strip_len": int(data[7]) if len(data) >= 8 else 0,
    }


def parse_ws_length_frame(data: bytes) -> dict:
    return {
        "strip_len": int(data[2]),
        "max_strip_len": int(data[3]) if len(data) >= 4 else 0,
    }


def parse_aht20_meas_frame(data: bytes) -> dict:
    temp_centi = int.from_bytes(data[2:4], "little", signed=True)
    rh_centi = int.from_bytes(data[4:6], "little", signed=False)
    return {
        "temp_centi_c": temp_centi,
        "temp_c": temp_centi / 100.0,
        "rh_centi_pct": rh_centi,
        "rh_pct": rh_centi / 100.0,
        "status": data[6],
        "crc_ok": bool(data[7]),
    }


def parse_aht20_raw_frame(data: bytes) -> dict:
    raw_h = data[2] | (data[3] << 8) | ((data[4] & 0x0F) << 16)
    raw_t = data[5] | (data[6] << 8) | ((data[7] & 0x0F) << 16)
    return {
        "raw_h": raw_h,
        "raw_t": raw_t,
    }


def parse_aht20_status_frame(data: bytes) -> dict:
    return {
        "status": data[2],
        "present": bool(data[3]),
        "env_valid": bool(data[4]),
        "crc_ok": bool(data[5]),
    }


def parse_aht20_reg_frame(data: bytes) -> dict:
    count = min(data[2], 5)
    payload = list(data[3:3 + count])
    return {
        "count": count,
        "data": payload,
    }


def parse_calib_value_frame(data: bytes) -> dict:
    fid = data[2]
    value = int.from_bytes(data[3:5], "little", signed=True)
    return {
        "field_id": fid,
        "field_name": CAL_FIELD_ID_TO_NAME.get(fid, f"field_{fid}"),
        "value": value,
    }


def parse_calib_info_frame(data: bytes) -> dict:
    return {
        "op": data[2],
        "result": data[3],
    }


def parse_hmc_cfg_frame(data: bytes) -> dict:
    range_id = data[2]
    data_rate_id = data[3]
    samples_id = data[4]
    mode_id = data[5]
    mg_centi = int.from_bytes(data[6:8], "little", signed=False)
    return {
        "range_id": range_id,
        "range_label": HMC_RANGE_ID_TO_LABEL.get(range_id, f"{range_id}"),
        "data_rate_id": data_rate_id,
        "data_rate_hz": HMC_DATA_RATE_ID_TO_HZ.get(data_rate_id, f"{data_rate_id}"),
        "samples_id": samples_id,
        "samples_count": HMC_SAMPLES_ID_TO_COUNT.get(samples_id, f"{samples_id}"),
        "mode_id": mode_id,
        "mode_name": HMC_MODE_ID_TO_NAME.get(mode_id, f"{mode_id}"),
        "mg_per_digit_centi": mg_centi,
        "mg_per_digit": mg_centi / 100.0,
    }


def format_hmc_cfg(info: dict) -> str:
    return (
        f"range={info['range_label']}ga({info['range_id']}) "
        f"sensitivity={info['mg_per_digit']:.2f}mG/LSB "
        f"data_rate={info['data_rate_hz']}Hz({info['data_rate_id']}) "
        f"samples={info['samples_count']}({info['samples_id']}) "
        f"mode={info['mode_name']}({info['mode_id']})"
    )


def decode_frame(data: bytes) -> str:
    if AppCanClient.is_status_reply(data):
        st = data[0]
        return f"STATUS {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"

    if len(data) >= 4 and data[:4] == b"PONG":
        return f"PONG dev={data[4]} proto={data[5]} flags=0x{data[6]:02X}"

    if len(data) >= 2 and data[0] == 0:
        subtype = data[1]

        if subtype == FRAME_STARTUP and len(data) >= 8:
            sensors = ",".join(decode_sensor_bits(data[4])) or "none"
            streams = ",".join(decode_stream_bits(data[5])) or "none"
            return (
                f"STARTUP dev={data[2]} proto={data[3]} "
                f"sensors={sensors} streams={streams} reset=0x{data[6]:02X}"
            )

        if subtype == FRAME_MAG and len(data) >= 8:
            x = int.from_bytes(data[2:4], "little", signed=True)
            y = int.from_bytes(data[4:6], "little", signed=True)
            z = int.from_bytes(data[6:8], "little", signed=True)
            rotation = math.degrees(math.atan2(y, x)) % 360.0
            return f"MAG x={x}mG y={y}mG z={z}mG rot={rotation:.1f}deg"

        if subtype == FRAME_ACC and len(data) >= 8:
            x = int.from_bytes(data[2:4], "little", signed=True)
            y = int.from_bytes(data[4:6], "little", signed=True)
            z = int.from_bytes(data[6:8], "little", signed=True)
            return f"ACC x={x}mg y={y}mg z={z}mg"

        if subtype == FRAME_ENV and len(data) >= 7:
            t_centi = int.from_bytes(data[2:4], "little", signed=True)
            rh_centi = int.from_bytes(data[4:6], "little", signed=False)
            return f"ENV T={t_centi/100.0:.2f}C RH={rh_centi/100.0:.2f}% valid={data[6]}"

        if subtype == FRAME_EVENT and len(data) >= 8:
            ev_id = data[2]
            p0 = data[3]
            p1 = data[4]
            p2 = data[5]
            p3 = int.from_bytes(data[6:8], "little")
            name = EVENT_NAMES.get(ev_id, f"UNKNOWN_{ev_id}")
            return f"EVENT {name} p0={p0} p1={p1} p2={p2} p3={p3}"

        if subtype == FRAME_INTERVAL and len(data) >= 6:
            parsed = parse_interval_frame(data)
            return (
                f"INTERVAL stream={parsed['stream_name']}({parsed['stream_id']}) "
                f"enabled={int(parsed['enabled'])} interval_ms={parsed['interval_ms']}"
            )

        if subtype == FRAME_STATUS and len(data) >= 8:
            parsed = parse_status_frame(data)
            sensors = ",".join(parsed["sensors"]) or "none"
            streams = ",".join(parsed["streams"]) or "none"
            return (
                f"STATUS_FRAME sensors={sensors} streams={streams} "
                f"interval_lsb={parsed['interval_low_bytes']}"
            )

        if subtype == FRAME_EVENT_STATE and len(data) >= 4:
            return f"EVENT_STATE sector={data[2]} elevation={data[3]}"

        if subtype == FRAME_AHT20_MEAS and len(data) >= 8:
            parsed = parse_aht20_meas_frame(data)
            return (
                f"AHT20_MEAS T={parsed['temp_c']:.2f}C RH={parsed['rh_pct']:.2f}% "
                f"status=0x{parsed['status']:02X} crc_ok={int(parsed['crc_ok'])}"
            )

        if subtype == FRAME_AHT20_RAW and len(data) >= 8:
            parsed = parse_aht20_raw_frame(data)
            return f"AHT20_RAW raw_h={parsed['raw_h']} raw_t={parsed['raw_t']}"

        if subtype == FRAME_AHT20_STATUS and len(data) >= 6:
            parsed = parse_aht20_status_frame(data)
            return (
                f"AHT20_STATUS status=0x{parsed['status']:02X} present={int(parsed['present'])} "
                f"env_valid={int(parsed['env_valid'])} crc_ok={int(parsed['crc_ok'])}"
            )

        if subtype == FRAME_AHT20_REG and len(data) >= 3:
            parsed = parse_aht20_reg_frame(data)
            hex_data = " ".join(f"{b:02X}" for b in parsed["data"])
            return f"AHT20_REG count={parsed['count']} data={hex_data}"

        if subtype == FRAME_CALIB_VALUE and len(data) >= 5:
            parsed = parse_calib_value_frame(data)
            return f"CALIB_VALUE {parsed['field_name']}({parsed['field_id']})={parsed['value']}"

        if subtype == FRAME_CALIB_INFO and len(data) >= 4:
            parsed = parse_calib_info_frame(data)
            return f"CALIB_INFO op=0x{parsed['op']:02X} result={parsed['result']}"

        if subtype == FRAME_HMC_CFG and len(data) >= 8:
            parsed = parse_hmc_cfg_frame(data)
            return f"HMC_CFG {format_hmc_cfg(parsed)}"

        if subtype == FRAME_WS_STATE and len(data) >= 8:
            parsed = parse_ws_state_frame(data)
            return (
                f"WS_STATE on={int(parsed['enabled'])} br={parsed['brightness']} "
                f"rgb=({parsed['r']},{parsed['g']},{parsed['b']}) len={parsed['strip_len']}"
            )

        if subtype == FRAME_WS_ANIM and len(data) >= 4:
            parsed = parse_ws_anim_frame(data)
            return f"WS_ANIM mode={parsed['mode_name']}({parsed['mode']}) speed={parsed['speed']}"

        if subtype == FRAME_WS_GRADIENT and len(data) >= 8:
            parsed = parse_ws_gradient_frame(data)
            return (
                f"WS_GRAD split={parsed['split_idx']} fade={parsed['fade_px']} "
                f"c1=0x{parsed['color1_rgb565']:04X} c2=0x{parsed['color2_rgb565']:04X}"
            )

        if subtype == FRAME_WS_SECTOR_COLOR and len(data) >= 7:
            parsed = parse_ws_sector_color_frame(data)
            return (
                f"WS_SECTOR_COLOR idx={parsed['idx']} rgb=({parsed['r']},{parsed['g']},{parsed['b']}) "
                f"max={parsed['max_sectors']}"
            )

        if subtype == FRAME_WS_SECTOR_MODE and len(data) >= 7:
            parsed = parse_ws_sector_mode_frame(data)
            return (
                f"WS_SECTOR_MODE on={int(parsed['enabled'])} fade={parsed['fade_speed']} "
                f"count={parsed['sector_count']} active={parsed['active_sector']} target={parsed['target_sector']} "
                f"max_zones={parsed['max_zones']}"
            )
        if subtype == FRAME_WS_SECTOR_ZONE and len(data) >= 8:
            parsed = parse_ws_sector_zone_frame(data)
            return (
                f"WS_SECTOR_ZONE idx={parsed['idx']} pos={parsed['pos_led']} "
                f"color=0x{parsed['color_rgb565']:04X} len={parsed['strip_len']}/{parsed['max_strip_len']}"
            )
        if subtype == FRAME_WS_LENGTH and len(data) >= 4:
            parsed = parse_ws_length_frame(data)
            return f"WS_LENGTH len={parsed['strip_len']} max={parsed['max_strip_len']}"

        return f"FRAME subtype=0x{subtype:02X} data={data.hex()}"

    return f"RAW {data.hex()}"


def stream_arg_to_id(value: str) -> int:
    key = value.strip().lower()
    if key in STREAM_NAME_TO_ID:
        return STREAM_NAME_TO_ID[key]
    iv = int(value, 0)
    if iv < 0 or iv > 4:
        raise argparse.ArgumentTypeError("stream must be all|mag|acc|env|event or 0..4")
    return iv


def calib_field_arg(value: str) -> int:
    key = value.strip().lower()
    if key in CAL_FIELD_NAME_TO_ID:
        return CAL_FIELD_NAME_TO_ID[key]
    iv = int(value, 0)
    if iv < 0 or iv > 19:
        raise argparse.ArgumentTypeError("calib field must be all|center_x..elev_curve or 0..19")
    return iv


def hmc_range_arg(value: str) -> int:
    key = value.strip().lower().replace("ga", "")
    labels = {
        "0.88": 0,
        "1.3": 1,
        "1.9": 2,
        "2.5": 3,
        "4": 4,
        "4.0": 4,
        "4.7": 5,
        "5.6": 6,
        "8.1": 7,
    }
    if key in labels:
        return labels[key]
    iv = int(value, 0)
    if iv < 0 or iv > 7:
        raise argparse.ArgumentTypeError("hmc range must be 0..7 or one of 0.88,1.3,1.9,2.5,4.0,4.7,5.6,8.1")
    return iv


def hmc_data_rate_arg(value: str) -> int:
    key = value.strip().lower().replace("hz", "")
    labels = {
        "0.75": 0,
        "1.5": 1,
        "3": 2,
        "7.5": 3,
        "15": 4,
        "30": 5,
        "75": 6,
    }
    if key in labels:
        return labels[key]
    iv = int(value, 0)
    if iv < 0 or iv > 6:
        raise argparse.ArgumentTypeError("hmc data-rate must be 0..6 or one of 0.75,1.5,3,7.5,15,30,75")
    return iv


def hmc_samples_arg(value: str) -> int:
    key = value.strip()
    labels = {
        "1": 0,
        "2": 1,
        "4": 2,
        "8": 3,
    }
    if key in labels:
        return labels[key]
    iv = int(value, 0)
    if iv < 0 or iv > 3:
        raise argparse.ArgumentTypeError("hmc samples must be 0..3 or one of 1,2,4,8")
    return iv


def hmc_mode_arg(value: str) -> int:
    key = value.strip().lower()
    labels = {
        "continuous": 0,
        "single": 1,
        "idle": 2,
    }
    if key in labels:
        return labels[key]
    iv = int(value, 0)
    if iv < 0 or iv > 2:
        raise argparse.ArgumentTypeError("hmc mode must be 0..2 or one of continuous,single,idle")
    return iv


def byte_arg(value: str) -> int:
    iv = int(value, 0)
    if iv < 0 or iv > 255:
        raise argparse.ArgumentTypeError("value must be 0..255")
    return iv


def ws_anim_arg(value: str) -> int:
    key = value.strip().lower()
    if key in WS_ANIM_NAME_TO_ID:
        return WS_ANIM_NAME_TO_ID[key]
    iv = int(value, 0)
    if iv < 0 or iv > 6:
        raise argparse.ArgumentTypeError(
            "animation must be static|blink|breathe|rainbow|wipe|gradient|sector-follow or 0..6"
        )
    return iv


def rgb565_arg(value: str) -> int:
    iv = int(value, 0)
    if iv < 0 or iv > 0xFFFF:
        raise argparse.ArgumentTypeError("RGB565 value must be 0..65535 (0x0000..0xFFFF)")
    return iv


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="app_firmware CAN host tool")
    parser.add_argument("--channel", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    parser.add_argument("--device-id", type=int, default=1, help="Device ID 0..127 (default: 1)")
    parser.add_argument("--timeout", type=float, default=2.5, help="Response timeout seconds (default: 2.5)")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping", help="Ping app")
    sub.add_parser("status", help="Get app status summary")

    p_seti = sub.add_parser("set-interval", help="Set stream interval")
    p_seti.add_argument("--stream", required=True, type=stream_arg_to_id, help="mag|acc|env|event or 1..4")
    p_seti.add_argument("--ms", required=True, type=int, help="interval in ms (0..60000)")
    p_seti.add_argument("--save", action="store_true", help="also persist to flash")

    p_geti = sub.add_parser("get-interval", help="Get stream interval(s)")
    p_geti.add_argument("--stream", default="all", type=stream_arg_to_id, help="all|mag|acc|env|event or 0..4")

    p_ena = sub.add_parser("stream-enable", help="Enable or disable stream")
    p_ena.add_argument("--stream", required=True, type=stream_arg_to_id, help="mag|acc|env|event or 1..4")
    p_ena.add_argument("--save", action="store_true", help="also persist to flash")
    grp = p_ena.add_mutually_exclusive_group(required=True)
    grp.add_argument("--on", action="store_true", help="Enable stream")
    grp.add_argument("--off", action="store_true", help="Disable stream")

    sub.add_parser("enter-bootloader", help="Ask app to reset into bootloader")

    p_hmc = sub.add_parser("hmc-config", help="Get or set HMC5883L config")
    p_hmc.add_argument("--range", dest="hmc_range", type=hmc_range_arg, help="0..7 or ga value (e.g. 8.1)")
    p_hmc.add_argument("--data-rate", type=hmc_data_rate_arg, help="0..6 or Hz value (e.g. 15)")
    p_hmc.add_argument("--samples", type=hmc_samples_arg, help="0..3 or sample count (1,2,4,8)")
    p_hmc.add_argument("--mode", type=hmc_mode_arg, help="0..2 or continuous|single|idle")
    p_hmc.add_argument("--save", action="store_true", help="also persist to flash")

    p_led = sub.add_parser("led", help="Get/set WS2812 strip state")
    p_led.add_argument("--get", action="store_true", help="Read current LED-strip state")
    p_led.add_argument("--on", action="store_true", help="Enable strip output")
    p_led.add_argument("--off", action="store_true", help="Disable strip output")
    p_led.add_argument("--brightness", type=byte_arg, help="0..255")
    p_led.add_argument("--r", type=byte_arg, help="red 0..255")
    p_led.add_argument("--g", type=byte_arg, help="green 0..255")
    p_led.add_argument("--b", type=byte_arg, help="blue 0..255")
    p_led.add_argument("--anim", type=ws_anim_arg, help="static|blink|breathe|rainbow|wipe|gradient|sector-follow or 0..6")
    p_led.add_argument("--anim-speed", type=byte_arg, help="animation speed 0..255")
    p_led.add_argument("--anim-get", action="store_true", help="Read animation config")
    p_led.add_argument("--grad-get", action="store_true", help="Read gradient config")
    p_led.add_argument("--grad-split", type=byte_arg, help="gradient split LED index (1-based)")
    p_led.add_argument("--grad-fade", type=byte_arg, help="gradient fade half-width in pixels")
    p_led.add_argument("--grad-c1", type=rgb565_arg, help="gradient color1 RGB565 (e.g. 0x001F)")
    p_led.add_argument("--grad-c2", type=rgb565_arg, help="gradient color2 RGB565 (e.g. 0xF800)")
    p_led.add_argument("--sector-mode-get", action="store_true", help="Read sector-follow mode config")
    p_led.add_argument("--sector-colors-get", action="store_true", help="Read all sector-follow colors")
    p_led.add_argument("--sector-on", action="store_true", help="Enable sector-follow mode")
    p_led.add_argument("--sector-off", action="store_true", help="Disable sector-follow mode")
    p_led.add_argument("--sector-fade", type=byte_arg, help="sector-follow fade speed 0..255")
    p_led.add_argument("--sector-count", type=int, help="sector-follow sector count 1..255")
    p_led.add_argument("--sector-idx", type=int, help="sector color index 1..8")
    p_led.add_argument("--sector-r", type=byte_arg, help="sector color red 0..255")
    p_led.add_argument("--sector-g", type=byte_arg, help="sector color green 0..255")
    p_led.add_argument("--sector-b", type=byte_arg, help="sector color blue 0..255")
    p_led.add_argument("--zone-get", action="store_true", help="Read zone mapping(s)")
    p_led.add_argument("--zone-id", type=int, help="zone index 1..32 (or with --zone-get, 0..32)")
    p_led.add_argument("--zone-pos", type=byte_arg, help="gradient stop position (1-based, 0 disables)")
    p_led.add_argument("--zone-color", type=rgb565_arg, help="zone color RGB565")
    p_led.add_argument("--len", type=byte_arg, help="active LED count (1..max)")
    p_led.add_argument("--len-get", action="store_true", help="Read active/max LED count")
    p_led.add_argument("--active-sector", type=byte_arg, help="set CAN-driven active sector (0 disables override)")

    p_mon = sub.add_parser("monitor", help="Live decode of incoming app frames")
    p_mon.add_argument("--duration", type=float, default=0.0, help="Seconds to monitor (0 = forever)")

    sub.add_parser("aht20-read", help="Trigger blocking AHT20 measurement and read decoded/raw values")

    p_cg = sub.add_parser("calib-get", help="Read calibration value(s)")
    p_cg.add_argument("--field", default="all", type=calib_field_arg, help="all or field name/id")

    p_cs = sub.add_parser("calib-set", help="Set one calibration value")
    p_cs.add_argument("--field", required=True, type=calib_field_arg, help="field name/id (1..19)")
    p_cs.add_argument("--value", required=True, type=int, help="int16 value")

    sub.add_parser("calib-save", help="Save current calibration to flash")
    sub.add_parser("calib-load", help="Load calibration from flash")
    sub.add_parser("calib-reset", help="Reset calibration to defaults (RAM only)")
    sub.add_parser("calib-capture-earth", help="Capture current earth magnetic field reference")

    return parser


def main() -> int:
    args = build_parser().parse_args()

    client = AppCanClient(
        channel=args.channel,
        interface=args.interface,
        device_id=args.device_id,
        timeout=args.timeout,
    )

    try:
        if args.cmd == "ping":
            pong = client.ping()
            if pong:
                print(f"PONG: dev={pong[4]} proto={pong[5]} flags=0x{pong[6]:02X}")
            else:
                print("PING OK (no PONG text frame received)")
            return 0

        if args.cmd == "status":
            st = client.get_status()
            print(
                "STATUS:",
                f"sensors={','.join(st['sensors']) or 'none'}",
                f"streams={','.join(st['streams']) or 'none'}",
                f"interval_lsb={st['interval_low_bytes']}",
            )
            return 0

        if args.cmd == "set-interval":
            if args.stream == 0:
                raise ValueError("set-interval stream must be 1..4")
            info = client.set_interval(args.stream, args.ms)
            if args.save:
                client.calib_save()
            print(
                f"SET_INTERVAL OK stream={info['stream_name']} enabled={int(info['enabled'])} "
                f"interval_ms={info['interval_ms']}"
            )
            if args.save:
                print("SET_INTERVAL persisted to flash")
            return 0

        if args.cmd == "get-interval":
            infos = client.get_intervals(args.stream)
            for info in infos:
                print(
                    f"INTERVAL stream={info['stream_name']} enabled={int(info['enabled'])} "
                    f"interval_ms={info['interval_ms']}"
                )
            return 0

        if args.cmd == "stream-enable":
            if args.stream == 0:
                raise ValueError("stream-enable stream must be 1..4")
            enable = args.on and not args.off
            info = client.set_stream_enable(args.stream, enable)
            if args.save:
                client.calib_save()
            print(
                f"STREAM_ENABLE OK stream={info['stream_name']} enabled={int(info['enabled'])} "
                f"interval_ms={info['interval_ms']}"
            )
            if args.save:
                print("STREAM_ENABLE persisted to flash")
            return 0

        if args.cmd == "enter-bootloader":
            client.enter_bootloader()
            print("ENTER_BOOTLOADER sent, device should reset into bootloader")
            return 0

        if args.cmd == "hmc-config":
            if (
                args.hmc_range is None
                and args.data_rate is None
                and args.samples is None
                and args.mode is None
            ):
                info = client.hmc_get_config()
                if args.save:
                    client.calib_save()
                    print(f"HMC_CONFIG_SAVE OK {format_hmc_cfg(info)}")
                    print("HMC_CONFIG persisted to flash")
                else:
                    print(f"HMC_CONFIG {format_hmc_cfg(info)}")
                return 0

            current = client.hmc_get_config()
            info = client.hmc_set_config(
                current["range_id"] if args.hmc_range is None else args.hmc_range,
                current["data_rate_id"] if args.data_rate is None else args.data_rate,
                current["samples_id"] if args.samples is None else args.samples,
                current["mode_id"] if args.mode is None else args.mode,
            )
            if args.save:
                client.calib_save()
            print(f"HMC_CONFIG_SET OK {format_hmc_cfg(info)}")
            if args.save:
                print("HMC_CONFIG persisted to flash")
            return 0

        if args.cmd == "led":
            if args.on and args.off:
                raise ValueError("choose only one of --on/--off")
            if args.sector_on and args.sector_off:
                raise ValueError("choose only one of --sector-on/--sector-off")

            has_rgb = any(v is not None for v in (args.r, args.g, args.b))
            if has_rgb and not all(v is not None for v in (args.r, args.g, args.b)):
                raise ValueError("set all of --r --g --b together")
            has_sector_rgb = any(v is not None for v in (args.sector_r, args.sector_g, args.sector_b))
            if has_sector_rgb and not all(v is not None for v in (args.sector_r, args.sector_g, args.sector_b)):
                raise ValueError("set all of --sector-r --sector-g --sector-b together")
            if has_sector_rgb and args.sector_idx is None:
                raise ValueError("--sector-idx is required when setting --sector-r/--sector-g/--sector-b")
            if args.sector_idx is not None and (args.sector_idx < 1 or args.sector_idx > 8):
                raise ValueError("--sector-idx must be 1..8")
            if args.sector_count is not None and (args.sector_count < 1 or args.sector_count > 255):
                raise ValueError("--sector-count must be 1..255")

            has_zone = any(v is not None for v in (args.zone_pos, args.zone_color))
            if has_zone:
                if not all(v is not None for v in (args.zone_pos, args.zone_color)):
                    raise ValueError("set both --zone-pos and --zone-color together")
                if args.zone_id is None:
                    raise ValueError("--zone-id is required when setting zone fields")
                if args.zone_id < 1 or args.zone_id > 32:
                    raise ValueError("--zone-id must be 1..32 when setting a zone")

            has_grad = any(v is not None for v in (args.grad_split, args.grad_fade, args.grad_c1, args.grad_c2))
            if has_grad:
                if not all(v is not None for v in (args.grad_split, args.grad_fade, args.grad_c1, args.grad_c2)):
                    raise ValueError("set all of --grad-split --grad-fade --grad-c1 --grad-c2 together")
                st = client.ws_set_gradient(
                    int(args.grad_split),
                    int(args.grad_fade),
                    int(args.grad_c1),
                    int(args.grad_c2),
                )
                print(
                    f"LED_GRAD_SET OK split={st['split_idx']} fade={st['fade_px']} "
                    f"c1=0x{st['color1_rgb565']:04X} c2=0x{st['color2_rgb565']:04X}"
                )
                return 0
            if args.grad_get:
                st = client.ws_get_gradient()
                print(
                    f"LED_GRAD split={st['split_idx']} fade={st['fade_px']} "
                    f"c1=0x{st['color1_rgb565']:04X} c2=0x{st['color2_rgb565']:04X}"
                )
                return 0

            if args.sector_mode_get:
                st = client.ws_get_sector_mode()
                print(
                    f"LED_SECTOR_MODE on={int(st['enabled'])} fade={st['fade_speed']} "
                    f"count={st['sector_count']} active={st['active_sector']} target={st['target_sector']}"
                )
                return 0

            if args.sector_colors_get:
                colors = client.ws_get_sector_color(0)
                for c in colors:
                    print(
                        f"LED_SECTOR_COLOR idx={c['idx']} "
                        f"rgb=({c['r']},{c['g']},{c['b']}) max={c['max_sectors']}"
                )
                return 0

            if args.zone_get:
                zone_idx = 0 if args.zone_id is None else int(args.zone_id)
                if zone_idx < 0 or zone_idx > 32:
                    raise ValueError("--zone-id for --zone-get must be 0..32")
                zones = client.ws_get_sector_zone(zone_idx)
                for z in zones:
                    print(
                        f"LED_ZONE idx={z['idx']} pos={z['pos_led']} color=0x{z['color_rgb565']:04X} "
                        f"len={z.get('strip_len',0)}/{z.get('max_strip_len',0)}"
                    )
                return 0

            if args.len_get:
                st = client.ws_get_length()
                print(f"LED_LENGTH len={st['strip_len']} max={st['max_strip_len']}")
                return 0

            if args.len is not None:
                st = client.ws_set_length(int(args.len))
                print(f"LED_LENGTH_SET OK len={st['strip_len']} max={st['max_strip_len']}")
                return 0

            if args.active_sector is not None:
                st = client.ws_set_active_sector(int(args.active_sector))
                print(
                    f"LED_ACTIVE_SECTOR_SET OK active={st['active_sector']} "
                    f"override={st['override_enabled']}"
                )
                return 0

            has_sector_mode = (
                args.sector_on
                or args.sector_off
                or args.sector_fade is not None
                or args.sector_count is not None
            )
            if has_sector_mode:
                cur = client.ws_get_sector_mode()
                enabled = cur["enabled"]
                if args.sector_on:
                    enabled = True
                elif args.sector_off:
                    enabled = False
                fade_speed = cur["fade_speed"] if args.sector_fade is None else int(args.sector_fade)
                sector_count = cur["sector_count"] if args.sector_count is None else int(args.sector_count)
                st = client.ws_set_sector_mode(enabled, fade_speed, sector_count)
                print(
                    f"LED_SECTOR_MODE_SET OK on={int(st['enabled'])} fade={st['fade_speed']} "
                    f"count={st['sector_count']} active={st['active_sector']} target={st['target_sector']}"
                )
                return 0

            if has_zone:
                z = client.ws_set_sector_zone(
                    int(args.zone_id),
                    int(args.zone_pos),
                    int(args.zone_color),
                )
                print(
                    f"LED_ZONE_SET OK idx={z['idx']} pos={z['pos_led']} color=0x{z['color_rgb565']:04X} "
                    f"len={z.get('strip_len',0)}/{z.get('max_strip_len',0)}"
                )
                return 0

            if has_sector_rgb:
                st = client.ws_set_sector_color(
                    int(args.sector_idx),
                    int(args.sector_r),
                    int(args.sector_g),
                    int(args.sector_b),
                )
                print(
                    f"LED_SECTOR_COLOR_SET OK idx={st['idx']} "
                    f"rgb=({st['r']},{st['g']},{st['b']}) max={st['max_sectors']}"
                )
                return 0

            has_anim = args.anim is not None or args.anim_speed is not None
            if has_anim:
                cur = client.ws_get_anim()
                mode = cur["mode"] if args.anim is None else int(args.anim)
                speed = cur["speed"] if args.anim_speed is None else int(args.anim_speed)
                st = client.ws_set_anim(mode, speed)
                print(f"LED_ANIM_SET OK mode={st['mode_name']}({st['mode']}) speed={st['speed']}")
                return 0
            if args.anim_get:
                st = client.ws_get_anim()
                print(f"LED_ANIM mode={st['mode_name']}({st['mode']}) speed={st['speed']}")
                return 0

            has_changes = args.on or args.off or args.brightness is not None or has_rgb
            if args.get or not has_changes:
                st = client.ws_get_state()
                print(
                    f"LED on={int(st['enabled'])} br={st['brightness']} "
                    f"rgb=({st['r']},{st['g']},{st['b']}) len={st['strip_len']}"
                )
                return 0

            current = client.ws_get_state()
            enabled = current["enabled"]
            if args.on:
                enabled = True
            elif args.off:
                enabled = False

            brightness = int(current["brightness"] if args.brightness is None else args.brightness)
            r = int(current["r"] if args.r is None else args.r)
            g = int(current["g"] if args.g is None else args.g)
            b = int(current["b"] if args.b is None else args.b)

            st = client.ws_set_all(enabled, brightness, r, g, b)
            print(
                f"LED_SET OK on={int(st['enabled'])} br={st['brightness']} "
                f"rgb=({st['r']},{st['g']},{st['b']}) len={st['strip_len']}"
            )
            return 0

        if args.cmd == "aht20-read":
            info = client.aht20_read()
            print(
                "AHT20:",
                f"T={info['temp_c']:.2f}C",
                f"RH={info['rh_pct']:.2f}%",
                f"status=0x{info['status']:02X}",
                f"crc_ok={int(info['crc_ok'])}",
                f"raw_h={info['raw_h']}",
                f"raw_t={info['raw_t']}",
            )
            return 0

        if args.cmd == "calib-get":
            values = client.calib_get(args.field)
            for item in values:
                print(f"CALIB {item['field_name']}({item['field_id']})={item['value']}")
            return 0

        if args.cmd == "calib-set":
            if args.field == 0:
                raise ValueError("calib-set requires a specific field (1..19)")
            item = client.calib_set(args.field, args.value)
            print(f"CALIB_SET OK {item['field_name']}({item['field_id']})={item['value']}")
            return 0

        if args.cmd == "calib-save":
            info = client.calib_save()
            print(f"CALIB_SAVE OK op=0x{info['op']:02X} result={info['result']}")
            return 0

        if args.cmd == "calib-load":
            info = client.calib_load()
            print(f"CALIB_LOAD OK op=0x{info['op']:02X} result={info['result']}")
            return 0

        if args.cmd == "calib-reset":
            info = client.calib_reset()
            print(f"CALIB_RESET OK op=0x{info['op']:02X} result={info['result']}")
            return 0

        if args.cmd == "calib-capture-earth":
            info = client.calib_capture_earth()
            print(f"CALIB_CAPTURE_EARTH OK op=0x{info['op']:02X} result={info['result']}")
            return 0

        if args.cmd == "monitor":
            deadline = None if args.duration <= 0 else (time.monotonic() + args.duration)
            while True:
                if deadline is not None and time.monotonic() >= deadline:
                    break
                msg = client.recv(timeout=0.2)
                if msg is None:
                    continue
                data = bytes(msg.data)
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] id=0x{msg.arbitration_id:03X} {decode_frame(data)}")
            return 0

        raise RuntimeError("Unknown command")

    finally:
        client.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (TimeoutError, RuntimeError, OSError, ValueError, can.CanError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
