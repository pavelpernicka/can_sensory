#!/usr/bin/env python3
import argparse
import math
import sys
import time
import can
from typing import Callable

CMD_PING = 0x01
CMD_ENTER_BOOTLOADER = 0x40
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
        if device_id < 0 or device_id > 0x7F:
            raise ValueError("device-id must be 0..127")

        self.device_id = device_id
        self.cmd_id = 0x600 + device_id
        self.status_id = 0x580 + device_id
        self.timeout = timeout

        self.bus = can.Bus(
            interface=interface,
            channel=channel,
            receive_own_messages=False,
            can_filters=[{
                "can_id": self.status_id,
                "can_mask": 0x7FF,
                "extended": False,
            }],
        )

    def close(self) -> None:
        self.bus.shutdown()

    def send(self, payload: bytes) -> None:
        if len(payload) > 8:
            raise ValueError("CAN payload must be <= 8 bytes")
        msg = can.Message(
            arbitration_id=self.cmd_id,
            is_extended_id=False,
            data=payload,
        )
        self.bus.send(msg)

    def recv(self, timeout: float | None = None):
        return self.bus.recv(self.timeout if timeout is None else timeout)

    @staticmethod
    def is_status_reply(data: bytes) -> bool:
        if len(data) < 2:
            return False
        if data[0] > STATUS_ERR_SENSOR:
            return False
        return all(b == 0 for b in data[2:])

    def wait_status(self, expected_extra: int, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if not self.is_status_reply(data):
                continue

            if data[1] != expected_extra:
                continue

            st = data[0]
            if st != STATUS_OK:
                raise RuntimeError(
                    f"Device error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"
                )
            return data

        raise TimeoutError(f"Timeout waiting for status extra=0x{expected_extra:02X}")

    def wait_frame(self, predicate: Callable[[bytes], bool], timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)

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
        self.send(bytes([CMD_PING]))
        self.wait_status(expected_extra=0x01)

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if len(data) >= 4 and data[:4] == b"PONG":
                return data
        return b""

    def enter_bootloader(self) -> None:
        self.send(bytes([CMD_ENTER_BOOTLOADER]))
        self.wait_status(expected_extra=0x40)

    def hmc_get_config(self) -> dict:
        self.send(bytes([CMD_HMC_GET_CFG]))
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

        self.send(bytes([CMD_HMC_SET_CFG, range_id, data_rate_id, samples_id, mode_id]))
        self.wait_status(expected_extra=0x6E)
        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_HMC_CFG)
        return parse_hmc_cfg_frame(frame)

    def set_interval(self, stream_id: int, interval_ms: int) -> dict:
        if stream_id < 1 or stream_id > 4:
            raise ValueError("stream-id must be 1..4")
        if interval_ms < 0 or interval_ms > 60000:
            raise ValueError("interval must be 0..60000 ms")

        self.send(bytes([
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

        self.send(bytes([CMD_SET_STREAM_ENABLE, stream_id, 1 if enable else 0]))
        self.wait_status(expected_extra=stream_id)

        frame = self.wait_frame(
            lambda d: len(d) >= 6 and d[0] == 0 and d[1] == FRAME_INTERVAL and d[2] == stream_id
        )
        return parse_interval_frame(frame)

    def get_intervals(self, stream_id: int) -> list[dict]:
        if stream_id < 0 or stream_id > 4:
            raise ValueError("stream-id must be 0..4")

        self.send(bytes([CMD_GET_INTERVAL, stream_id]))

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
        self.send(bytes([CMD_GET_STATUS]))
        self.wait_status(expected_extra=0x73)

        frame = self.wait_frame(lambda d: len(d) >= 8 and d[0] == 0 and d[1] == FRAME_STATUS)
        return parse_status_frame(frame)

    def aht20_read(self) -> dict:
        self.send(bytes([CMD_AHT20_READ]))
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
        if field_id < 0 or field_id > 16:
            raise ValueError("calib field-id must be 0..16")

        self.send(bytes([CMD_CALIB_GET, field_id]))
        if field_id == 0:
            self.wait_status(expected_extra=0x79)
            expected = len(CAL_FIELD_ID_TO_NAME)
        else:
            self.wait_status(expected_extra=field_id)
            expected = 1

        out: dict[int, dict] = {}
        deadline = time.monotonic() + self.timeout
        while len(out) < expected and time.monotonic() < deadline:
            frame = self.wait_frame(
                lambda d: len(d) >= 5 and d[0] == 0 and d[1] == FRAME_CALIB_VALUE,
                timeout=max(0.0, deadline - time.monotonic()),
            )
            parsed = parse_calib_value_frame(frame)
            fid = parsed["field_id"]
            if field_id == 0 or fid == field_id:
                out[fid] = parsed

        if len(out) < expected:
            raise TimeoutError("Timeout waiting for calibration value frame(s)")

        return [out[fid] for fid in sorted(out.keys())]

    def calib_set(self, field_id: int, value: int) -> dict:
        if field_id < 1 or field_id > 16:
            raise ValueError("calib field-id must be 1..16")
        if value < -32768 or value > 32767:
            raise ValueError("calib value must be int16 range")

        payload = bytes([
            CMD_CALIB_SET,
            field_id,
            value & 0xFF,
            (value >> 8) & 0xFF,
        ])
        self.send(payload)
        self.wait_status(expected_extra=field_id)
        frame = self.wait_frame(
            lambda d: len(d) >= 5 and d[0] == 0 and d[1] == FRAME_CALIB_VALUE and d[2] == field_id
        )
        return parse_calib_value_frame(frame)

    def calib_save(self) -> dict:
        self.send(bytes([CMD_CALIB_SAVE]))
        self.wait_status(expected_extra=0x7B)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_load(self) -> dict:
        self.send(bytes([CMD_CALIB_LOAD]))
        self.wait_status(expected_extra=0x7C)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_reset(self) -> dict:
        self.send(bytes([CMD_CALIB_RESET]))
        self.wait_status(expected_extra=0x7D)
        frame = self.wait_frame(lambda d: len(d) >= 4 and d[0] == 0 and d[1] == FRAME_CALIB_INFO)
        return parse_calib_info_frame(frame)

    def calib_capture_earth(self) -> dict:
        self.send(bytes([CMD_CALIB_CAPTURE_EARTH]))
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
    if iv < 0 or iv > 16:
        raise argparse.ArgumentTypeError("calib field must be all|center_x..earth_valid or 0..16")
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="app_firmware CAN host tool")
    parser.add_argument("--channel", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    parser.add_argument("--device-id", type=int, default=1, help="Device ID 0..127 (default: 1)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Response timeout seconds (default: 1.0)")

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

    p_mon = sub.add_parser("monitor", help="Live decode of incoming app frames")
    p_mon.add_argument("--duration", type=float, default=0.0, help="Seconds to monitor (0 = forever)")

    sub.add_parser("aht20-read", help="Trigger blocking AHT20 measurement and read decoded/raw values")

    p_cg = sub.add_parser("calib-get", help="Read calibration value(s)")
    p_cg.add_argument("--field", default="all", type=calib_field_arg, help="all or field name/id")

    p_cs = sub.add_parser("calib-set", help="Set one calibration value")
    p_cs.add_argument("--field", required=True, type=calib_field_arg, help="field name/id (1..16)")
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
                raise ValueError("calib-set requires a specific field (1..16)")
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
