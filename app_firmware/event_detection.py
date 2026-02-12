#!/usr/bin/env python3
"""
Reusable event-detection state machine for magnet sensor data.

This module intentionally mirrors firmware logic (event_detector.c).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Mapping

EVENT_BUFFER_SIZE = 5
EVENT_MIN_SECTORS = 1
EVENT_MAX_SECTORS = 16
EVENT_DEFAULT_SECTORS = 6

EVENT_SECTOR_ACTIVATED = 1
EVENT_SECTOR_CHANGED = 2
EVENT_INTENSITY_CHANGE = 3
EVENT_SECTION_DEACTIVATED = 4
EVENT_SESSION_STARTED = 5
EVENT_SESSION_ENDED = 6
EVENT_PASSING_SECTOR_CHANGE = 7
EVENT_POSSIBLE_MECHANICAL_FAILURE = 8
EVENT_ERROR_NO_DATA = 9

EVENT_NAMES = {
    EVENT_SECTOR_ACTIVATED: "SECTOR_ACTIVATED",
    EVENT_SECTOR_CHANGED: "SECTOR_CHANGED",
    EVENT_INTENSITY_CHANGE: "INTENSITY_CHANGE",
    EVENT_SECTION_DEACTIVATED: "SECTION_DEACTIVATED",
    EVENT_SESSION_STARTED: "SESSION_STARTED",
    EVENT_SESSION_ENDED: "SESSION_ENDED",
    EVENT_PASSING_SECTOR_CHANGE: "PASSING_SECTOR_CHANGE",
    EVENT_POSSIBLE_MECHANICAL_FAILURE: "POSSIBLE_MECHANICAL_FAILURE",
    EVENT_ERROR_NO_DATA: "ERROR_NO_DATA",
}


def _sanitize_num_sectors(value: int) -> int:
    if value < EVENT_MIN_SECTORS or value > EVENT_MAX_SECTORS:
        return EVENT_DEFAULT_SECTORS
    return int(value)


def _clamp_u8(value: float) -> int:
    iv = int(value)
    if iv < 0:
        return 0
    if iv > 255:
        return 255
    return iv


@dataclass
class Event:
    type: int
    p0: int = 0
    p1: int = 0
    p2: int = 0
    p3: int = 0

    def name(self) -> str:
        return EVENT_NAMES.get(self.type, f"UNKNOWN_{self.type}")

    def to_text(self) -> str:
        if self.type == EVENT_SECTOR_ACTIVATED:
            return f"SECTOR_ACTIVATED -> {self.p0} elev={self.p1} speed={self.p2}"
        if self.type == EVENT_SECTOR_CHANGED:
            return f"SECTOR_CHANGED {self.p0}->{self.p1}"
        if self.type == EVENT_INTENSITY_CHANGE:
            return f"INTENSITY_CHANGE sec={self.p0} elev={self.p1} speed={self.p2}"
        if self.type == EVENT_SECTION_DEACTIVATED:
            return f"SECTION_DEACTIVATED {self.p0}"
        if self.type == EVENT_SESSION_STARTED:
            return "SESSION_STARTED"
        if self.type == EVENT_SESSION_ENDED:
            return "SESSION_ENDED"
        if self.type == EVENT_PASSING_SECTOR_CHANGE:
            return f"PASSING_SECTOR_CHANGE -> {self.p0}"
        if self.type == EVENT_POSSIBLE_MECHANICAL_FAILURE:
            return f"POSSIBLE_MECHANICAL_FAILURE sec={self.p0}"
        if self.type == EVENT_ERROR_NO_DATA:
            return "ERROR_NO_DATA"
        return f"{self.name()} p0={self.p0} p1={self.p1} p2={self.p2} p3={self.p3}"


@dataclass
class EventDetectionConfig:
    center_x: float = 0.0
    center_y: float = 0.0
    center_z: float = 0.0
    rotate_xy_deg: float = 0.0
    rotate_xz_deg: float = 0.0
    rotate_yz_deg: float = 0.0
    keepout_rad: float = 1000.0
    z_limit: float = 150.0
    data_radius: float = 3000.0
    num_sectors: int = EVENT_DEFAULT_SECTORS
    change_threshold: float = 3.0
    deactivation_timeout_ms: int = 5000
    session_timeout_ms: int = 10000

    def __post_init__(self):
        self.num_sectors = _sanitize_num_sectors(int(self.num_sectors))

    @classmethod
    def from_calibration(cls, calib: Mapping[str, int]) -> "EventDetectionConfig":
        return cls(
            center_x=float(calib.get("center_x", 0)),
            center_y=float(calib.get("center_y", 0)),
            center_z=float(calib.get("center_z", 0)),
            rotate_xy_deg=float(calib.get("rotate_xy", 0)) / 100.0,
            rotate_xz_deg=float(calib.get("rotate_xz", 0)) / 100.0,
            rotate_yz_deg=float(calib.get("rotate_yz", 0)) / 100.0,
            keepout_rad=float(calib.get("keepout_rad", 1000)),
            z_limit=float(calib.get("z_limit", 150)),
            data_radius=float(calib.get("data_radius", 3000)),
            num_sectors=int(calib.get("num_sectors", EVENT_DEFAULT_SECTORS)),
        )


class EventDetector:
    def __init__(self, config: EventDetectionConfig | None = None):
        self.config = config if config is not None else EventDetectionConfig()
        self.reset_state()

    def apply_config(self, config: EventDetectionConfig) -> None:
        self.config = config

    def reset_state(self, now_s: float | None = None) -> None:
        if now_s is None:
            now_s = time.monotonic()

        self._sector_buf: deque[int] = deque(maxlen=EVENT_BUFFER_SIZE)
        self._elev_buf: deque[float] = deque(maxlen=EVENT_BUFFER_SIZE)
        self._last_sector = 0
        self._last_elevation = 0.0
        self._last_state_elevation = 0
        self._last_event_s = float(now_s)
        self._last_nonzero_s = float(now_s)
        self._session_active = False
        self._last_sector_event_s: dict[int, float] = {}
        self._deactivated: set[int] = set()
        self._last_no_data_s = 0.0

    def _rotate_3d(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        rad_xy = math.radians(self.config.rotate_xy_deg)
        x1 = x * math.cos(rad_xy) - y * math.sin(rad_xy)
        y1 = x * math.sin(rad_xy) + y * math.cos(rad_xy)
        z1 = z

        rad_xz = math.radians(self.config.rotate_xz_deg)
        x2 = x1 * math.cos(rad_xz) - z1 * math.sin(rad_xz)
        z2 = x1 * math.sin(rad_xz) + z1 * math.cos(rad_xz)
        y2 = y1

        rad_yz = math.radians(self.config.rotate_yz_deg)
        y3 = y2 * math.cos(rad_yz) - z2 * math.sin(rad_yz)
        z3 = y2 * math.sin(rad_yz) + z2 * math.cos(rad_yz)
        return x2, y3, z3

    def compute_sector_elevation(self, x: float, y: float, z: float) -> tuple[int, int]:
        z_adj = z - self.config.center_z
        xr, yr, zr = self._rotate_3d(x, y, z_adj)

        dx = xr - self.config.center_x
        dy = yr - self.config.center_y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance <= self.config.keepout_rad or zr < self.config.z_limit:
            return 0, 0

        azimuth = math.degrees(math.atan2(dy, dx))
        while azimuth < 0.0:
            azimuth += 360.0
        while azimuth >= 360.0:
            azimuth -= 360.0

        sector = int(azimuth / (360.0 / float(self.config.num_sectors))) + 1
        elevation = _clamp_u8(max(0.0, zr - self.config.z_limit))
        return sector, elevation

    @staticmethod
    def _stamp_ms16(now_s: float) -> int:
        return int(now_s * 1000.0) & 0xFFFF

    def process_mag_sample(self, x: float, y: float, z: float, now_s: float | None = None) -> list[Event]:
        if now_s is None:
            now_s = time.monotonic()
        now_s = float(now_s)

        out: list[Event] = []
        sector, elev_u8 = self.compute_sector_elevation(x, y, z)
        self._sector_buf.append(sector)
        self._elev_buf.append(float(elev_u8))

        if len(self._elev_buf) < EVENT_BUFFER_SIZE:
            self._last_event_s = now_s
            self._last_sector = sector
            self._last_elevation = float(elev_u8)
            self._last_state_elevation = elev_u8
            return out

        elev_avg = sum(self._elev_buf) / float(len(self._elev_buf))
        dt = max(0.001, now_s - self._last_event_s)
        speed = _clamp_u8(abs(elev_avg - self._last_elevation) / dt)
        stamp = self._stamp_ms16(now_s)

        if sector != self._last_sector:
            if self._last_sector == 0:
                out.append(Event(EVENT_SECTOR_ACTIVATED, p0=sector, p1=_clamp_u8(elev_avg), p2=speed, p3=stamp))
                if not self._session_active:
                    out.append(Event(EVENT_SESSION_STARTED, p3=stamp))
                    self._session_active = True
            elif sector != 0:
                diff = abs(self._last_sector - sector)
                wrap_diff = self.config.num_sectors - diff
                if (diff == 1 or wrap_diff == 1) and (now_s - self._last_event_s) < 0.020:
                    out.append(Event(EVENT_PASSING_SECTOR_CHANGE, p0=sector, p3=stamp))
                else:
                    out.append(Event(EVENT_SECTOR_CHANGED, p0=self._last_sector, p1=sector, p3=stamp))

            if EVENT_MIN_SECTORS <= sector <= self.config.num_sectors:
                self._deactivated.discard(sector)
                self._last_sector_event_s[sector] = now_s
        elif sector != 0 and abs(elev_avg - self._last_elevation) > self.config.change_threshold:
            if sector not in self._deactivated:
                out.append(Event(EVENT_INTENSITY_CHANGE, p0=sector, p1=_clamp_u8(elev_avg), p2=speed, p3=stamp))
                self._last_sector_event_s[sector] = now_s

        if self._last_sector != 0:
            self._last_nonzero_s = now_s

        if self._last_sector != 0 and self._last_sector <= self.config.num_sectors:
            sec_last = self._last_sector_event_s.get(self._last_sector, 0.0)
            if sec_last > 0.0 and (now_s - sec_last) > (self.config.deactivation_timeout_ms / 1000.0):
                out.append(Event(EVENT_SECTION_DEACTIVATED, p0=self._last_sector, p3=stamp))
                if self._session_active:
                    out.append(Event(EVENT_SESSION_ENDED, p3=stamp))
                    self._session_active = False
                self._deactivated.add(self._last_sector)
                self._last_sector_event_s.pop(self._last_sector, None)

        if self._last_sector != 0 and (now_s - self._last_event_s) > (self.config.session_timeout_ms / 1000.0):
            out.append(Event(EVENT_POSSIBLE_MECHANICAL_FAILURE, p0=self._last_sector, p3=stamp))
        elif self._last_sector == 0 and (now_s - self._last_nonzero_s) > (self.config.session_timeout_ms / 1000.0):
            if self._session_active:
                out.append(Event(EVENT_SESSION_ENDED, p3=stamp))
                self._session_active = False

        self._last_sector = sector
        self._last_elevation = elev_avg
        self._last_state_elevation = _clamp_u8(elev_avg)
        self._last_event_s = now_s
        return out

    def post_no_data(self, now_s: float | None = None) -> list[Event]:
        if now_s is None:
            now_s = time.monotonic()
        now_s = float(now_s)

        if (now_s - self._last_no_data_s) < (self.config.session_timeout_ms / 1000.0):
            return []
        self._last_no_data_s = now_s
        return [Event(EVENT_ERROR_NO_DATA, p3=self._stamp_ms16(now_s))]

    def get_sector_state(self) -> tuple[int, int]:
        return self._last_sector, self._last_state_elevation
