#!/usr/bin/env python3
"""
GTK control and calibration app for app_firmware over CAN.
"""

import argparse
import json
import math
import threading
import time
from collections import deque
from queue import Empty, SimpleQueue

import can
import gi
import numpy as np
from matplotlib.backends.backend_gtk4agg import FigureCanvasGTK4Agg as FigureCanvas
from matplotlib.figure import Figure
from event_detection import Event, EventDetectionConfig, EventDetector, EVENT_NAMES

from app_can_tool import (
    AppCanClient,
    CAL_FIELD_ID_TO_NAME,
    CAL_FIELD_NAME_TO_ID,
    FRAME_ACC,
    FRAME_AHT20_MEAS,
    FRAME_AHT20_RAW,
    FRAME_CALIB_VALUE,
    FRAME_ENV,
    FRAME_EVENT,
    FRAME_EVENT_STATE,
    FRAME_HMC_CFG,
    FRAME_INTERVAL,
    FRAME_MAG,
    FRAME_STARTUP,
    FRAME_STATUS,
    HMC_DATA_RATE_ID_TO_HZ,
    HMC_MODE_ID_TO_NAME,
    HMC_RANGE_ID_TO_LABEL,
    HMC_SAMPLES_ID_TO_COUNT,
    decode_sensor_bits,
    decode_stream_bits,
    decode_frame,
    discover_devices,
    parse_aht20_meas_frame,
    parse_aht20_raw_frame,
    parse_hmc_cfg_frame,
    parse_interval_frame,
    parse_status_frame,
    status_id_to_device_id,
)

gi.require_version("Gtk", "4.0")
gi.require_version("Gdk", "4.0")
from gi.repository import GLib, Gdk, Gtk, Pango


CALIB_FIELDS = [
    ("center_x", "Center X (mG)", -32768, 32767, 1),
    ("center_y", "Center Y (mG)", -32768, 32767, 1),
    ("center_z", "Center Z (mG)", -32768, 32767, 1),
    ("rotate_xy", "Rotate XY (cdeg)", -36000, 36000, 10),
    ("rotate_xz", "Rotate XZ (cdeg)", -36000, 36000, 10),
    ("rotate_yz", "Rotate YZ (cdeg)", -36000, 36000, 10),
    ("keepout_rad", "Keepout Radius (mG)", 0, 32767, 10),
    ("z_limit", "Z Min (mG)", -32768, 32767, 10),
    ("z_max", "Z Max (mG)", -32768, 32767, 10),
    ("elev_curve", "Elev Curve x100", 10, 500, 5),
    ("data_radius", "Data Radius (mG)", 10, 32767, 10),
    ("num_sectors", "Sector Count", 1, 16, 1),
    ("mag_offset_x", "Mag Offset X", -32768, 32767, 1),
    ("mag_offset_y", "Mag Offset Y", -32768, 32767, 1),
    ("mag_offset_z", "Mag Offset Z", -32768, 32767, 1),
    ("earth_x", "Earth X", -32768, 32767, 1),
    ("earth_y", "Earth Y", -32768, 32767, 1),
    ("earth_z", "Earth Z", -32768, 32767, 1),
    ("earth_valid", "Earth Valid", 0, 1, 1),
]
CALIB_FIELD_BOUNDS = {key: (vmin, vmax) for key, _label, vmin, vmax, _step in CALIB_FIELDS}
STREAM_IDS = (1, 2, 3, 4)
STREAM_INTERVAL_MIN_MS = 0
STREAM_INTERVAL_MAX_MS = 60000

STREAM_META = {
    1: "mag",
    2: "acc",
    3: "env",
    4: "event",
}

PLOT_INTERVAL_MS = 33
PLOT_MAX_HZ = 12.0
HISTORY_MAX_POINTS = 14000
ENV_HISTORY_MAX_POINTS = 6000
MAX_RENDER_MAG_POINTS = 1000
MAX_RENDER_ACC_POINTS = 1000
MAX_RENDER_ENV_POINTS = 2000
CMD_TIMEOUT_SHORT = 3.0
CMD_TIMEOUT_LONG = 5.0
EVENT_TYPE_IDS = sorted(EVENT_NAMES.keys())


def compute_slope_cdeg(x_mg: int, y_mg: int, z_mg: int) -> int:
    horizontal = math.sqrt(float(x_mg) * float(x_mg) + float(y_mg) * float(y_mg))
    angle_deg = math.degrees(math.atan2(horizontal, abs(float(z_mg)) + 1e-6))
    return int(round(angle_deg * 100.0))


def compute_plane_slopes_cdeg(x_mg: int, y_mg: int, z_mg: int) -> tuple[int, int, int]:
    slope_xy = int(round(math.degrees(math.atan2(float(y_mg), float(x_mg) + 1e-6)) * 100.0))
    slope_xz = int(round(math.degrees(math.atan2(float(z_mg), float(x_mg) + 1e-6)) * 100.0))
    slope_yz = int(round(math.degrees(math.atan2(float(z_mg), float(y_mg) + 1e-6)) * 100.0))
    return slope_xy, slope_xz, slope_yz


def decimate_points(data, limit: int):
    if limit <= 0:
        return []
    data_list = list(data)
    n = len(data_list)
    if n <= limit:
        return data_list
    step = max(1, n // limit)
    return data_list[::step]


def transform_mag_point_for_detector(x: float, y: float, z: float, calib: dict[str, int]) -> tuple[float, float, float]:
    # Mirror detector-space transform:
    # 1) subtract center_z from Z
    # 2) rotate XY, then XZ, then YZ
    # 3) keep center_x/center_y as XY center reference in transformed frame
    z_adj = z - float(calib["center_z"])

    rxy = math.radians(float(calib["rotate_xy"]) / 100.0)
    x1 = x * math.cos(rxy) - y * math.sin(rxy)
    y1 = x * math.sin(rxy) + y * math.cos(rxy)
    z1 = z_adj

    rxz = math.radians(float(calib["rotate_xz"]) / 100.0)
    x2 = x1 * math.cos(rxz) - z1 * math.sin(rxz)
    z2 = x1 * math.sin(rxz) + z1 * math.cos(rxz)
    y2 = y1

    ryz = math.radians(float(calib["rotate_yz"]) / 100.0)
    y3 = y2 * math.cos(ryz) - z2 * math.sin(ryz)
    z3 = y2 * math.sin(ryz) + z2 * math.cos(ryz)
    return x2, y3, z3


def transform_mag_points_for_detector(arr: np.ndarray, calib: dict[str, int]) -> np.ndarray:
    if arr.size == 0:
        return arr

    out = arr.astype(float, copy=True)
    out[:, 2] = out[:, 2] - float(calib["center_z"])

    rxy = math.radians(float(calib["rotate_xy"]) / 100.0)
    x1 = out[:, 0] * math.cos(rxy) - out[:, 1] * math.sin(rxy)
    y1 = out[:, 0] * math.sin(rxy) + out[:, 1] * math.cos(rxy)
    z1 = out[:, 2]

    rxz = math.radians(float(calib["rotate_xz"]) / 100.0)
    x2 = x1 * math.cos(rxz) - z1 * math.sin(rxz)
    z2 = x1 * math.sin(rxz) + z1 * math.cos(rxz)
    y2 = y1

    ryz = math.radians(float(calib["rotate_yz"]) / 100.0)
    y3 = y2 * math.cos(ryz) - z2 * math.sin(ryz)
    z3 = y2 * math.sin(ryz) + z2 * math.cos(ryz)

    out[:, 0] = x2
    out[:, 1] = y3
    out[:, 2] = z3
    return out

class CalibApp(Gtk.Application):
    def __init__(self, channel: str, interface: str, device_id: int, timeout: float):
        super().__init__(application_id="cz.magnetomuzika.Calibration")

        self.channel = channel
        self.interface = interface
        self.client = AppCanClient(channel, interface, device_id, timeout)
        self.active_device_id = int(device_id)
        self.known_device_ids: set[int] = {int(device_id)}
        self.monitor_bus = can.Bus(
            interface=interface,
            channel=channel,
            receive_own_messages=False,
            can_filters=[{
                "can_id": 0x580,
                "can_mask": 0x780,
                "extended": False,
            }],
        )
        self.sim_tx_bus = can.Bus(
            interface=interface,
            channel=channel,
            receive_own_messages=False,
        )

        self.running = True
        self.cmd_lock = threading.Lock()

        self.rx_queue: SimpleQueue[tuple[int, bytes]] = SimpleQueue()
        self.log_queue: SimpleQueue[str] = SimpleQueue()
        self.device_stats: dict[int, dict] = {}
        self.last_stats_text = ""

        self.mag_points = deque(maxlen=HISTORY_MAX_POINTS)
        self.acc_points = deque(maxlen=HISTORY_MAX_POINTS)
        self.env_points = deque(maxlen=ENV_HISTORY_MAX_POINTS)  # (ts, temp_c, rh_pct)
        self.local_event_detector = EventDetector()
        self.local_event_cfg_dirty = True

        self.preview_events = deque(maxlen=200)
        self.device_events = deque(maxlen=200)
        self.preview_dirty = True
        self.device_event_dirty = True
        self.frame_counts: dict[int, int] = {}

        self.latest_mag_xyz = (0, 0, 0)
        self.latest_acc_xyz = (0, 0, 0)
        self.latest_acc_slope_cdeg = 0
        self.latest_temp_c = 0.0
        self.latest_rh_pct = 0.0
        self.latest_env_valid = 0

        self.latest_sector_local = 0
        self.latest_elev_local = 0
        self.latest_sector_fw = 0
        self.latest_elev_fw = 0

        self.chart_points_limit = 1000
        self.mag_show_every_n = 1
        self.mag_freeze_first_n = False
        self.mag_frozen_points: list[tuple[int, int, int]] = []

        self.mag_dirty = True
        self.acc_dirty = True
        self.env_dirty = True
        self.last_plot_ts = 0.0

        self.suppress_spin_events = False
        self.suppress_stream_events = False
        self.suppress_hmc_events = False
        self.apply_timer_id = 0
        self.pending_apply_fields: set[str] = set()

        self.calib = {
            "center_x": 0,
            "center_y": 0,
            "center_z": 0,
            "rotate_xy": 0,
            "rotate_xz": 0,
            "rotate_yz": 0,
            "keepout_rad": 1000,
            "z_limit": 150,
            "z_max": 405,
            "elev_curve": 100,
            "data_radius": 3000,
            "num_sectors": 6,
            "mag_offset_x": 0,
            "mag_offset_y": 0,
            "mag_offset_z": 0,
            "earth_x": 0,
            "earth_y": 0,
            "earth_z": 0,
            "earth_valid": 0,
        }

        self.stream_cfg = {
            1: {"enabled": True, "interval_ms": 200},
            2: {"enabled": True, "interval_ms": 200},
            3: {"enabled": True, "interval_ms": 1000},
            4: {"enabled": True, "interval_ms": 250},
        }

        self.hmc_cfg = {
            "range_id": 7,
            "data_rate_id": 6,
            "samples_id": 0,
            "mode_id": 0,
            "mg_per_digit": 4.35,
        }

        self.window = None
        self.left_stack = None
        self.left_active = "mag"
        self.fullscreen_toggle = None
        self.is_fullscreen = False

        self.mag_ax = None
        self.mag_canvas = None
        self.acc_ax = None
        self.acc_canvas = None
        self.env_ax = None
        self.env_ax_rh = None
        self.env_canvas = None

        self.mag_scatter = None
        self.mag_last_scatter = None
        self.mag_center_scatter = None
        self.mag_keep_line = None
        self.mag_outer_line = None
        self.mag_sector_lines = []
        self.mag_info_text = None

        self.acc_scatter = None
        self.acc_last_scatter = None
        self.acc_vec_line = None

        self.env_temp_line = None
        self.env_rh_line = None

        self.status_label = None
        self.chart_state_row = None
        self.chart_meta_label = None
        self.local_state_label = None
        self.fw_state_label = None
        self.preview_label = None
        self.acc_status_label = None
        self.mag_live_label = None
        self.acc_live_label = None
        self.env_live_label = None
        self.aht_label = None
        self.aht_raw_label = None
        self.health_label = None

        self.event_local_buffer = None
        self.event_device_buffer = None
        self.log_buffer = None
        self.device_stats_buffer = None
        self.sim_event_combo = None
        self.sim_p0_spin = None
        self.sim_p1_spin = None
        self.sim_p2_spin = None
        self.sim_p3_spin = None
        self.sim_auto_stamp_check = None
        self.sim_sector_spin = None
        self.sim_to_sector_spin = None
        self.sim_elev_spin = None
        self.sim_speed_spin = None

        self.stream_switches: dict[int, Gtk.Switch] = {}
        self.stream_spins: dict[int, Gtk.SpinButton] = {}
        self.calib_spins: dict[str, Gtk.SpinButton] = {}

        self.auto_apply_check = None
        self.log_fast_check = None
        self.chart_points_spin = None
        self.mag_every_n_spin = None
        self.mag_freeze_check = None

        self.hmc_range_spin = None
        self.hmc_dr_spin = None
        self.hmc_samples_spin = None
        self.hmc_mode_spin = None
        self.hmc_desc_label = None
        self.device_id_spin = None
        self.known_ids_label = None

        self.last_status_text = ""
        self.last_chart_meta_text = ""
        self.last_local_state_markup = ""
        self.last_fw_state_markup = ""
        self.last_preview_label_text = ""
        self.last_event_local_text = ""
        self.last_event_device_text = ""
        self.last_acc_status_text = ""
        self.last_mag_title_text = ""
        self.last_acc_title_text = ""
        self.last_env_title_text = ""
        self.last_mag_limits = None
        self.last_acc_limits = None
        self.last_env_limits = None

    def do_activate(self):
        self.window = Gtk.ApplicationWindow(application=self)
        self.window.set_title("Magnetomuzika Control")
        self.window.set_default_size(1600, 950)
        self.window.connect("close-request", self.on_close_request)

        key_controller = Gtk.EventControllerKey()
        key_controller.connect("key-pressed", self.on_key_pressed)
        self.window.add_controller(key_controller)

        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        root.set_margin_start(8)
        root.set_margin_end(8)
        root.set_margin_top(8)
        root.set_margin_bottom(8)
        self.window.set_child(root)

        left = self.build_left_panel()
        root.append(left)

        right = self.build_right_panel()
        root.append(right)

        self.window.present()

        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

        GLib.timeout_add(PLOT_INTERVAL_MS, self.on_ui_tick)
        self.run_async("initial-load", self.initial_load)

    def build_left_panel(self) -> Gtk.Widget:
        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_hexpand(True)
        left.set_vexpand(True)

        switcher = Gtk.StackSwitcher()
        self.left_stack = Gtk.Stack()
        self.left_stack.connect("notify::visible-child-name", self.on_left_stack_changed)
        switcher.set_stack(self.left_stack)
        left.append(switcher)

        self.chart_state_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.chart_state_row.set_margin_top(2)
        self.chart_state_row.set_margin_bottom(2)

        local_frame = Gtk.Frame(label="LOCAL")
        local_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        local_box.set_margin_start(8)
        local_box.set_margin_end(8)
        local_box.set_margin_top(6)
        local_box.set_margin_bottom(6)
        self.local_state_label = Gtk.Label(label="")
        self.local_state_label.set_use_markup(True)
        self.local_state_label.set_xalign(0.5)
        local_box.append(self.local_state_label)
        local_frame.set_child(local_box)
        local_frame.set_hexpand(True)
        self.chart_state_row.append(local_frame)

        fw_frame = Gtk.Frame(label="FW")
        fw_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        fw_box.set_margin_start(8)
        fw_box.set_margin_end(8)
        fw_box.set_margin_top(6)
        fw_box.set_margin_bottom(6)
        self.fw_state_label = Gtk.Label(label="")
        self.fw_state_label.set_use_markup(True)
        self.fw_state_label.set_xalign(0.5)
        fw_box.append(self.fw_state_label)
        fw_frame.set_child(fw_box)
        fw_frame.set_hexpand(True)
        self.chart_state_row.append(fw_frame)

        left.append(self.chart_state_row)

        self.chart_meta_label = Gtk.Label(label="")
        self.chart_meta_label.set_xalign(0)
        self.chart_meta_label.set_wrap(True)
        self.chart_meta_label.set_selectable(False)
        self.chart_meta_label.set_margin_bottom(2)
        left.append(self.chart_meta_label)

        self.left_stack.add_titled(self.build_mag_plot_page(), "mag", "MAG 3D")
        self.left_stack.add_titled(self.build_acc_plot_page(), "acc", "ACC 3D")
        self.left_stack.add_titled(self.build_env_plot_page(), "env", "ENV Charts")
        self.left_stack.set_vexpand(True)
        left.append(self.left_stack)

        self.status_label = Gtk.Label(label="No data")
        self.configure_dynamic_label(self.status_label)
        left.append(self.status_label)

        self.preview_label = Gtk.Label(label="Event preview idle")
        self.configure_dynamic_label(self.preview_label)
        left.append(self.preview_label)

        self.update_chart_info_label()
        return left

    def build_mag_plot_page(self) -> Gtk.Widget:
        fig = Figure(figsize=(8, 7), dpi=90)
        self.mag_ax = fig.add_subplot(111, projection="3d")
        self.mag_canvas = FigureCanvas(fig)
        self.mag_canvas.set_hexpand(True)
        self.mag_canvas.set_vexpand(True)
        self.init_mag_plot_artists()
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.append(self.mag_canvas)
        return box

    def build_acc_plot_page(self) -> Gtk.Widget:
        fig = Figure(figsize=(8, 7), dpi=90)
        self.acc_ax = fig.add_subplot(111, projection="3d")
        self.acc_canvas = FigureCanvas(fig)
        self.acc_canvas.set_hexpand(True)
        self.acc_canvas.set_vexpand(True)
        self.init_acc_plot_artists()

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        box.append(self.acc_canvas)

        self.acc_status_label = Gtk.Label(label="ACC: no data")
        self.configure_dynamic_label(self.acc_status_label)
        box.append(self.acc_status_label)
        return box

    def build_env_plot_page(self) -> Gtk.Widget:
        fig = Figure(figsize=(8, 7), dpi=90)
        self.env_ax = fig.add_subplot(111)
        self.env_ax_rh = self.env_ax.twinx()
        self.env_canvas = FigureCanvas(fig)
        self.env_canvas.set_hexpand(True)
        self.env_canvas.set_vexpand(True)
        self.init_env_plot_artists()

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.append(self.env_canvas)
        return box

    def init_mag_plot_artists(self):
        self.mag_ax.set_facecolor("#ffffff")
        self.mag_ax.grid(True, alpha=0.25)
        self.mag_ax.set_xlabel("X (mG)")
        self.mag_ax.set_ylabel("Y (mG)")
        self.mag_ax.set_zlabel("Z (mG)")
        self.mag_ax.view_init(elev=23, azim=-58)

        self.mag_scatter = self.mag_ax.scatter([], [], [], c=[], cmap="viridis", s=7, alpha=0.85, depthshade=False)
        self.mag_last_scatter = self.mag_ax.scatter([], [], [], c="red", s=42, marker="x", depthshade=False)
        self.mag_center_scatter = self.mag_ax.scatter([], [], [], c="black", s=36, marker="o", depthshade=False)
        self.mag_keep_line, = self.mag_ax.plot([], [], [], "r--", linewidth=1.0)
        self.mag_outer_line, = self.mag_ax.plot([], [], [], color="gray", linestyle=":", linewidth=1.0)
        self.mag_zmax_line, = self.mag_ax.plot([], [], [], color="#1f77b4", linestyle="-.", linewidth=1.0)
        self.mag_sector_lines = [self.mag_ax.plot([], [], [], "k--", linewidth=0.8)[0] for _ in range(16)]
        self.mag_sector_labels = [
            self.mag_ax.text(0.0, 0.0, 0.0, "", fontsize=11, color="#222222", ha="center", va="center")
            for _ in range(16)
        ]
        self.mag_info_text = self.mag_ax.text2D(0.02, 0.98, "", transform=self.mag_ax.transAxes, va="top", fontsize=8)

    def init_acc_plot_artists(self):
        self.acc_ax.set_facecolor("#ffffff")
        self.acc_ax.grid(True, alpha=0.25)
        self.acc_ax.set_xlabel("X (mg)")
        self.acc_ax.set_ylabel("Y (mg)")
        self.acc_ax.set_zlabel("Z (mg)")
        self.acc_ax.view_init(elev=24, azim=-50)

        self.acc_scatter = self.acc_ax.scatter([], [], [], c=[], cmap="plasma", s=7, alpha=0.85, depthshade=False)
        self.acc_last_scatter = self.acc_ax.scatter([], [], [], c="red", s=42, marker="x", depthshade=False)
        self.acc_vec_line, = self.acc_ax.plot([], [], [], color="black", linewidth=1.0)

    def init_env_plot_artists(self):
        self.env_ax.set_facecolor("#ffffff")
        self.env_ax.grid(True, alpha=0.25)
        self.env_ax.set_xlabel("Time (s)")
        self.env_ax.set_ylabel("Temp (C)", color="red")
        self.env_ax_rh.set_ylabel("RH (%)", color="blue")
        self.env_temp_line, = self.env_ax.plot([], [], color="red", linewidth=1.5, label="Temp")
        self.env_rh_line, = self.env_ax_rh.plot([], [], color="blue", linewidth=1.5, label="RH")

    def build_right_panel(self) -> Gtk.Widget:
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        outer.set_size_request(640, -1)
        outer.set_hexpand(False)
        outer.set_vexpand(True)

        switcher = Gtk.StackSwitcher()
        stack = Gtk.Stack()
        switcher.set_stack(stack)
        outer.append(switcher)

        stack.add_titled(self.wrap_page(self.build_page_device()), "device", "Device")
        stack.add_titled(self.wrap_page(self.build_page_streams()), "streams", "Streams")
        stack.add_titled(self.wrap_page(self.build_page_sensors()), "sensors", "Sensors")
        stack.add_titled(self.wrap_page(self.build_page_calibration()), "calibration", "Calibration")
        stack.add_titled(self.wrap_page(self.build_page_events()), "events", "Events")
        stack.set_vexpand(True)
        outer.append(stack)

        outer.append(Gtk.Label(label="Log", xalign=0))
        self.log_buffer = Gtk.TextBuffer()
        log_view = Gtk.TextView(buffer=self.log_buffer)
        log_view.set_editable(False)
        log_view.set_monospace(True)
        log_scroll = Gtk.ScrolledWindow()
        log_scroll.set_min_content_height(240)
        log_scroll.set_child(log_view)
        outer.append(log_scroll)

        return outer

    def wrap_page(self, widget: Gtk.Widget) -> Gtk.Widget:
        scrolled = Gtk.ScrolledWindow()
        scrolled.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scrolled.set_child(widget)
        return scrolled

    def build_page_device(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(6)
        box.set_margin_end(6)
        box.set_margin_top(6)
        box.set_margin_bottom(6)

        self.device_info_label = Gtk.Label(label="")
        self.device_info_label.set_xalign(0)
        box.append(self.device_info_label)
        self.refresh_device_info_label()

        row1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row1.append(Gtk.Label(label="Active device ID", xalign=0))
        self.device_id_spin = Gtk.SpinButton()
        self.device_id_spin.set_range(0, 127)
        self.device_id_spin.set_increments(1, 8)
        self.device_id_spin.set_value(float(self.active_device_id))
        row1.append(self.device_id_spin)
        b_switch = Gtk.Button(label="Switch")
        b_switch.connect("clicked", lambda *_: self.run_async("switch-device", self.action_switch_device))
        b_discover = Gtk.Button(label="Auto-Discover")
        b_discover.connect("clicked", lambda *_: self.run_async("discover", self.action_discover_devices))
        row1.append(b_switch)
        row1.append(b_discover)
        box.append(row1)

        self.known_ids_label = Gtk.Label(label="")
        self.known_ids_label.set_xalign(0)
        box.append(self.known_ids_label)
        self.update_known_devices_label()

        row2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_ping = Gtk.Button(label="Ping")
        b_ping.connect("clicked", lambda *_: self.run_async("ping", self.action_ping))
        b_status = Gtk.Button(label="Get Status")
        b_status.connect("clicked", lambda *_: self.run_async("status", self.action_status))
        b_boot = Gtk.Button(label="Enter Bootloader")
        b_boot.connect("clicked", lambda *_: self.run_async("boot", self.action_enter_bootloader))
        self.fullscreen_toggle = Gtk.ToggleButton(label="Fullscreen")
        self.fullscreen_toggle.connect("toggled", self.on_fullscreen_toggled)
        row2.append(b_ping)
        row2.append(b_status)
        row2.append(b_boot)
        row2.append(self.fullscreen_toggle)
        box.append(row2)

        row3 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row3.append(Gtk.Label(label="Chart points", xalign=0))
        self.chart_points_spin = Gtk.SpinButton()
        self.chart_points_spin.set_range(100, 5000)
        self.chart_points_spin.set_increments(50, 200)
        self.chart_points_spin.set_value(float(self.chart_points_limit))
        self.chart_points_spin.connect("value-changed", self.on_chart_points_changed)
        row3.append(self.chart_points_spin)
        row3.append(Gtk.Label(label="MAG every Nth", xalign=0))
        self.mag_every_n_spin = Gtk.SpinButton()
        self.mag_every_n_spin.set_range(1, 50)
        self.mag_every_n_spin.set_increments(1, 5)
        self.mag_every_n_spin.set_value(float(self.mag_show_every_n))
        self.mag_every_n_spin.connect("value-changed", self.on_mag_every_n_changed)
        row3.append(self.mag_every_n_spin)
        box.append(row3)

        row4 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.mag_freeze_check = Gtk.CheckButton(label="MAG freeze after first N chart points")
        self.mag_freeze_check.set_active(False)
        self.mag_freeze_check.connect("toggled", self.on_mag_freeze_toggled)
        self.log_fast_check = Gtk.CheckButton(label="Log fast frames (MAG/ACC/ENV/EVENT_STATE)")
        self.log_fast_check.set_active(False)
        b_clear_points = Gtk.Button(label="Clear Plot Data")
        b_clear_points.connect("clicked", self.on_clear_points)
        b_clear_log = Gtk.Button(label="Clear Log")
        b_clear_log.connect("clicked", self.on_clear_log)
        row4.append(self.mag_freeze_check)
        row4.append(self.log_fast_check)
        row4.append(b_clear_points)
        row4.append(b_clear_log)
        box.append(row4)

        box.append(Gtk.Label(label="Per-device stats", xalign=0))
        self.device_stats_buffer = Gtk.TextBuffer()
        stats_view = Gtk.TextView(buffer=self.device_stats_buffer)
        stats_view.set_editable(False)
        stats_view.set_monospace(True)
        stats_scroll = Gtk.ScrolledWindow()
        stats_scroll.set_min_content_height(180)
        stats_scroll.set_child(stats_view)
        box.append(stats_scroll)

        hint = Gtk.Label(label="Tip: F11 toggles fullscreen.")
        hint.set_xalign(0)
        box.append(hint)

        return box

    def build_page_streams(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(6)
        box.set_margin_end(6)
        box.set_margin_top(6)
        box.set_margin_bottom(6)

        top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_refresh = Gtk.Button(label="Refresh From Device")
        b_refresh.connect("clicked", lambda *_: self.run_async("stream-get", self.action_refresh_streams))
        b_apply = Gtk.Button(label="Apply Streams")
        b_apply.connect("clicked", lambda *_: self.run_async("stream-apply", self.action_apply_streams))
        b_apply_save = Gtk.Button(label="Apply + Save")
        b_apply_save.connect("clicked", lambda *_: self.run_async("stream-apply-save", lambda: self.action_apply_streams(save=True)))
        top.append(b_refresh)
        top.append(b_apply)
        top.append(b_apply_save)
        box.append(top)

        grid = Gtk.Grid(column_spacing=10, row_spacing=8)
        grid.attach(Gtk.Label(label="Stream", xalign=0), 0, 0, 1, 1)
        grid.attach(Gtk.Label(label="Enable", xalign=0), 1, 0, 1, 1)
        grid.attach(Gtk.Label(label="Interval ms", xalign=0), 2, 0, 1, 1)

        for ridx, sid in enumerate((1, 2, 3, 4), start=1):
            grid.attach(Gtk.Label(label=f"{sid}: {STREAM_META[sid]}", xalign=0), 0, ridx, 1, 1)

            sw = Gtk.Switch()
            sw.set_active(True)
            sw.connect("notify::active", self.on_stream_switch_changed, sid)
            self.stream_switches[sid] = sw
            grid.attach(sw, 1, ridx, 1, 1)

            spin = Gtk.SpinButton()
            spin.set_range(0, 60000)
            spin.set_increments(1, 10)
            spin.set_numeric(True)
            spin.set_value(float(self.stream_cfg[sid]["interval_ms"]))
            spin.connect("value-changed", self.on_stream_spin_changed, sid)
            self.stream_spins[sid] = spin
            grid.attach(spin, 2, ridx, 1, 1)

        box.append(grid)
        return box

    def build_page_sensors(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(6)
        box.set_margin_end(6)
        box.set_margin_top(6)
        box.set_margin_bottom(6)

        hmc_frame = Gtk.Frame(label="HMC5883L")
        hmc_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        hmc_box.set_margin_start(6)
        hmc_box.set_margin_end(6)
        hmc_box.set_margin_top(6)
        hmc_box.set_margin_bottom(6)

        grid = Gtk.Grid(column_spacing=10, row_spacing=8)
        grid.attach(Gtk.Label(label="Range ID", xalign=0), 0, 0, 1, 1)
        self.hmc_range_spin = Gtk.SpinButton()
        self.hmc_range_spin.set_range(0, 7)
        self.hmc_range_spin.set_increments(1, 1)
        self.hmc_range_spin.connect("value-changed", self.on_hmc_spin_changed)
        grid.attach(self.hmc_range_spin, 1, 0, 1, 1)

        grid.attach(Gtk.Label(label="Data Rate ID", xalign=0), 0, 1, 1, 1)
        self.hmc_dr_spin = Gtk.SpinButton()
        self.hmc_dr_spin.set_range(0, 6)
        self.hmc_dr_spin.set_increments(1, 1)
        self.hmc_dr_spin.connect("value-changed", self.on_hmc_spin_changed)
        grid.attach(self.hmc_dr_spin, 1, 1, 1, 1)

        grid.attach(Gtk.Label(label="Samples ID", xalign=0), 0, 2, 1, 1)
        self.hmc_samples_spin = Gtk.SpinButton()
        self.hmc_samples_spin.set_range(0, 3)
        self.hmc_samples_spin.set_increments(1, 1)
        self.hmc_samples_spin.connect("value-changed", self.on_hmc_spin_changed)
        grid.attach(self.hmc_samples_spin, 1, 2, 1, 1)

        grid.attach(Gtk.Label(label="Mode ID", xalign=0), 0, 3, 1, 1)
        self.hmc_mode_spin = Gtk.SpinButton()
        self.hmc_mode_spin.set_range(0, 2)
        self.hmc_mode_spin.set_increments(1, 1)
        self.hmc_mode_spin.connect("value-changed", self.on_hmc_spin_changed)
        grid.attach(self.hmc_mode_spin, 1, 3, 1, 1)

        hmc_box.append(grid)

        self.hmc_desc_label = Gtk.Label(label="")
        self.hmc_desc_label.set_xalign(0)
        self.hmc_desc_label.set_wrap(True)
        hmc_box.append(self.hmc_desc_label)

        hrow = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_hmc_get = Gtk.Button(label="Get HMC Config")
        b_hmc_get.connect("clicked", lambda *_: self.run_async("hmc-get", self.action_hmc_get))
        b_hmc_set = Gtk.Button(label="Apply HMC Config")
        b_hmc_set.connect("clicked", lambda *_: self.run_async("hmc-set", self.action_hmc_set))
        b_hmc_set_save = Gtk.Button(label="Apply + Save")
        b_hmc_set_save.connect("clicked", lambda *_: self.run_async("hmc-set-save", lambda: self.action_hmc_set(save=True)))
        hrow.append(b_hmc_get)
        hrow.append(b_hmc_set)
        hrow.append(b_hmc_set_save)
        hmc_box.append(hrow)

        hmc_frame.set_child(hmc_box)
        box.append(hmc_frame)

        live_frame = Gtk.Frame(label="Live Sensors")
        live_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        live_box.set_margin_start(6)
        live_box.set_margin_end(6)
        live_box.set_margin_top(6)
        live_box.set_margin_bottom(6)

        aht_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_aht = Gtk.Button(label="Read AHT20")
        b_aht.connect("clicked", lambda *_: self.run_async("aht20-read", self.action_aht20_read))
        b_health = Gtk.Button(label="Healthcheck")
        b_health.connect("clicked", lambda *_: self.run_async("healthcheck", self.action_healthcheck))
        b_live_streams = Gtk.Button(label="Enable Live Streams")
        b_live_streams.connect("clicked", lambda *_: self.run_async("live-streams", self.action_enable_live_streams))
        b_acc_snapshot = Gtk.Button(label="Log Latest ACC")
        b_acc_snapshot.connect("clicked", lambda *_: self.action_log_latest_acc())
        aht_row.append(b_aht)
        aht_row.append(b_health)
        aht_row.append(b_live_streams)
        aht_row.append(b_acc_snapshot)
        live_box.append(aht_row)

        self.mag_live_label = Gtk.Label(label="MAG live: -")
        self.mag_live_label.set_xalign(0)
        self.acc_live_label = Gtk.Label(label="ACC live: -")
        self.acc_live_label.set_xalign(0)
        self.env_live_label = Gtk.Label(label="ENV live: -")
        self.env_live_label.set_xalign(0)
        self.aht_label = Gtk.Label(label="AHT20: -")
        self.aht_label.set_xalign(0)
        self.aht_raw_label = Gtk.Label(label="AHT20 raw: -")
        self.aht_raw_label.set_xalign(0)
        self.health_label = Gtk.Label(label="Health: -")
        self.health_label.set_xalign(0)
        self.health_label.set_wrap(True)
        live_box.append(self.mag_live_label)
        live_box.append(self.acc_live_label)
        live_box.append(self.env_live_label)
        live_box.append(self.aht_label)
        live_box.append(self.aht_raw_label)
        live_box.append(self.health_label)

        live_frame.set_child(live_box)
        box.append(live_frame)

        return box

    def build_page_calibration(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(6)
        box.set_margin_end(6)
        box.set_margin_top(6)
        box.set_margin_bottom(6)

        header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_load_runtime = Gtk.Button(label="Load Runtime")
        b_load_runtime.connect("clicked", lambda *_: self.run_async("calib-get", self.action_load_runtime_calib))
        b_apply_all = Gtk.Button(label="Apply All")
        b_apply_all.connect("clicked", lambda *_: self.run_async("calib-apply", self.action_apply_all_calib))
        b_save = Gtk.Button(label="Save Flash")
        b_save.connect("clicked", lambda *_: self.run_async("calib-save", self.action_calib_save))
        b_load_flash = Gtk.Button(label="Load Flash")
        b_load_flash.connect("clicked", lambda *_: self.run_async("calib-load", self.action_calib_load))
        b_reset = Gtk.Button(label="Reset Defaults")
        b_reset.connect("clicked", lambda *_: self.run_async("calib-reset", self.action_calib_reset))
        header.append(b_load_runtime)
        header.append(b_apply_all)
        header.append(b_save)
        header.append(b_load_flash)
        header.append(b_reset)
        box.append(header)

        h2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_capture = Gtk.Button(label="Capture Earth")
        b_capture.connect("clicked", lambda *_: self.run_async("calib-capture-earth", self.action_capture_earth))
        b_center = Gtk.Button(label="Auto Center (last 250)")
        b_center.connect("clicked", self.on_auto_center)
        self.auto_apply_check = Gtk.CheckButton(label="Auto apply changed fields")
        self.auto_apply_check.set_active(False)
        b_flush = Gtk.Button(label="Apply Pending")
        b_flush.connect("clicked", lambda *_: self.flush_pending_apply())
        h2.append(b_capture)
        h2.append(b_center)
        h2.append(self.auto_apply_check)
        h2.append(b_flush)
        box.append(h2)

        h3 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_export_json = Gtk.Button(label="Export JSON")
        b_export_json.connect("clicked", self.on_export_calib_json)
        b_import_json = Gtk.Button(label="Import JSON")
        b_import_json.connect("clicked", lambda *_: self.on_import_calib_json(False, False))
        b_import_apply = Gtk.Button(label="Import + Apply")
        b_import_apply.connect("clicked", lambda *_: self.on_import_calib_json(True, False))
        b_import_apply_save = Gtk.Button(label="Import + Apply + Save")
        b_import_apply_save.connect("clicked", lambda *_: self.on_import_calib_json(True, True))
        h3.append(b_export_json)
        h3.append(b_import_json)
        h3.append(b_import_apply)
        h3.append(b_import_apply_save)
        box.append(h3)

        grid = Gtk.Grid(column_spacing=10, row_spacing=8)
        for row, (key, label, vmin, vmax, step) in enumerate(CALIB_FIELDS):
            grid.attach(Gtk.Label(label=label, xalign=0), 0, row, 1, 1)
            spin = Gtk.SpinButton()
            spin.set_range(vmin, vmax)
            spin.set_increments(step, step * 5)
            spin.set_numeric(True)
            spin.set_value(float(self.calib[key]))
            spin.connect("value-changed", self.on_calib_spin_changed, key)
            self.calib_spins[key] = spin
            grid.attach(spin, 1, row, 1, 1)

        box.append(grid)
        return box

    def build_page_events(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(6)
        box.set_margin_end(6)
        box.set_margin_top(6)
        box.set_margin_bottom(6)

        info = Gtk.Label(
            label=(
                "Preview is computed locally from live MAG data with current calibration fields. "
                "Firmware event stream is shown in a separate pane."
            )
        )
        info.set_wrap(True)
        info.set_xalign(0)
        box.append(info)

        split = Gtk.Paned.new(Gtk.Orientation.HORIZONTAL)
        split.set_wide_handle(True)
        split.set_resize_start_child(True)
        split.set_resize_end_child(True)

        left_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        left_box.append(Gtk.Label(label="Local events (computed in GUI)", xalign=0))
        self.event_local_buffer = Gtk.TextBuffer()
        ev_local_view = Gtk.TextView(buffer=self.event_local_buffer)
        ev_local_view.set_editable(False)
        ev_local_view.set_monospace(True)
        ev_local_scroll = Gtk.ScrolledWindow()
        ev_local_scroll.set_vexpand(True)
        ev_local_scroll.set_child(ev_local_view)
        left_box.append(ev_local_scroll)

        right_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        right_box.append(Gtk.Label(label="Device events (from FRAME_EVENT)", xalign=0))
        self.event_device_buffer = Gtk.TextBuffer()
        ev_dev_view = Gtk.TextView(buffer=self.event_device_buffer)
        ev_dev_view.set_editable(False)
        ev_dev_view.set_monospace(True)
        ev_dev_scroll = Gtk.ScrolledWindow()
        ev_dev_scroll.set_vexpand(True)
        ev_dev_scroll.set_child(ev_dev_view)
        right_box.append(ev_dev_scroll)

        split.set_start_child(left_box)
        split.set_end_child(right_box)
        split.set_vexpand(True)
        box.append(split)

        btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        b_clear_local = Gtk.Button(label="Clear Local Events")
        b_clear_local.connect("clicked", self.on_clear_preview_events)
        b_clear_device = Gtk.Button(label="Clear Device Events")
        b_clear_device.connect("clicked", self.on_clear_device_events)
        btns.append(b_clear_local)
        btns.append(b_clear_device)
        box.append(btns)

        sim_frame = Gtk.Frame(label="Manual Event Simulation (CAN Inject)")
        sim_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        sim_box.set_margin_start(6)
        sim_box.set_margin_end(6)
        sim_box.set_margin_top(6)
        sim_box.set_margin_bottom(6)

        sim_info = Gtk.Label(
            label=(
                "Transmit synthetic FRAME_EVENT over CAN with source ID "
                "(0x580 + active device id), so it is processed like a real device event."
            ),
            xalign=0,
        )
        sim_info.set_wrap(True)
        sim_box.append(sim_info)

        def make_spin(vmin: int, vmax: int, value: int, step: int = 1) -> Gtk.SpinButton:
            sp = Gtk.SpinButton()
            sp.set_range(vmin, vmax)
            sp.set_increments(step, max(step, step * 5))
            sp.set_numeric(True)
            sp.set_value(float(value))
            return sp

        row_type = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row_type.append(Gtk.Label(label="Event type", xalign=0))
        self.sim_event_combo = Gtk.ComboBoxText()
        for ev_id in EVENT_TYPE_IDS:
            self.sim_event_combo.append(str(ev_id), f"{ev_id}: {EVENT_NAMES.get(ev_id, ev_id)}")
        self.sim_event_combo.set_active_id(str(EVENT_TYPE_IDS[0]))
        row_type.append(self.sim_event_combo)
        row_type.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))
        self.sim_auto_stamp_check = Gtk.CheckButton(label="Auto p3 timestamp")
        self.sim_auto_stamp_check.set_active(True)
        row_type.append(self.sim_auto_stamp_check)
        sim_box.append(row_type)

        p_grid = Gtk.Grid(column_spacing=8, row_spacing=6)
        self.sim_p0_spin = make_spin(0, 255, 1)
        self.sim_p1_spin = make_spin(0, 255, 0)
        self.sim_p2_spin = make_spin(0, 255, 0)
        self.sim_p3_spin = make_spin(0, 65535, 0)
        p_grid.attach(Gtk.Label(label="p0", xalign=0), 0, 0, 1, 1)
        p_grid.attach(self.sim_p0_spin, 1, 0, 1, 1)
        p_grid.attach(Gtk.Label(label="p1", xalign=0), 2, 0, 1, 1)
        p_grid.attach(self.sim_p1_spin, 3, 0, 1, 1)
        p_grid.attach(Gtk.Label(label="p2", xalign=0), 4, 0, 1, 1)
        p_grid.attach(self.sim_p2_spin, 5, 0, 1, 1)
        p_grid.attach(Gtk.Label(label="p3", xalign=0), 6, 0, 1, 1)
        p_grid.attach(self.sim_p3_spin, 7, 0, 1, 1)
        b_emit_custom = Gtk.Button(label="Emit Custom")
        b_emit_custom.connect("clicked", self.on_sim_emit_custom)
        p_grid.attach(b_emit_custom, 8, 0, 1, 1)
        sim_box.append(p_grid)

        quick_grid = Gtk.Grid(column_spacing=8, row_spacing=6)
        self.sim_sector_spin = make_spin(1, 16, 1)
        self.sim_to_sector_spin = make_spin(1, 16, 2)
        self.sim_elev_spin = make_spin(0, 255, 80)
        self.sim_speed_spin = make_spin(0, 255, 40)

        quick_grid.attach(Gtk.Label(label="sector", xalign=0), 0, 0, 1, 1)
        quick_grid.attach(self.sim_sector_spin, 1, 0, 1, 1)
        quick_grid.attach(Gtk.Label(label="to sector", xalign=0), 2, 0, 1, 1)
        quick_grid.attach(self.sim_to_sector_spin, 3, 0, 1, 1)
        quick_grid.attach(Gtk.Label(label="elev", xalign=0), 4, 0, 1, 1)
        quick_grid.attach(self.sim_elev_spin, 5, 0, 1, 1)
        quick_grid.attach(Gtk.Label(label="speed", xalign=0), 6, 0, 1, 1)
        quick_grid.attach(self.sim_speed_spin, 7, 0, 1, 1)

        b_q_activate = Gtk.Button(label="Sector Activate")
        b_q_activate.connect("clicked", self.on_sim_quick_activate)
        quick_grid.attach(b_q_activate, 0, 1, 2, 1)

        b_q_change = Gtk.Button(label="Sector Change")
        b_q_change.connect("clicked", self.on_sim_quick_change)
        quick_grid.attach(b_q_change, 2, 1, 2, 1)

        b_q_intensity = Gtk.Button(label="Intensity")
        b_q_intensity.connect("clicked", self.on_sim_quick_intensity)
        quick_grid.attach(b_q_intensity, 4, 1, 2, 1)

        b_q_deactivate = Gtk.Button(label="Deactivate")
        b_q_deactivate.connect("clicked", self.on_sim_quick_deactivate)
        quick_grid.attach(b_q_deactivate, 6, 1, 2, 1)

        b_q_session_start = Gtk.Button(label="Session Start")
        b_q_session_start.connect("clicked", self.on_sim_quick_session_start)
        quick_grid.attach(b_q_session_start, 0, 2, 2, 1)

        b_q_session_end = Gtk.Button(label="Session End")
        b_q_session_end.connect("clicked", self.on_sim_quick_session_end)
        quick_grid.attach(b_q_session_end, 2, 2, 2, 1)

        b_q_nodata = Gtk.Button(label="No Data")
        b_q_nodata.connect("clicked", self.on_sim_quick_no_data)
        quick_grid.attach(b_q_nodata, 4, 2, 2, 1)

        b_q_failure = Gtk.Button(label="Mechanical Failure")
        b_q_failure.connect("clicked", self.on_sim_quick_failure)
        quick_grid.attach(b_q_failure, 6, 2, 2, 1)

        sim_box.append(quick_grid)
        sim_frame.set_child(sim_box)
        box.append(sim_frame)

        return box

    def configure_dynamic_label(self, label: Gtk.Label):
        label.set_xalign(0)
        label.set_wrap(False)
        label.set_ellipsize(Pango.EllipsizeMode.END)
        label.set_max_width_chars(180)
        label.set_hexpand(True)

    def on_close_request(self, *_args):
        self.running = False
        try:
            self.client.close()
        except Exception:
            pass
        try:
            self.monitor_bus.shutdown()
        except Exception:
            pass
        try:
            self.sim_tx_bus.shutdown()
        except Exception:
            pass
        return False

    def on_key_pressed(self, _controller, keyval, _keycode, _state):
        if keyval == Gdk.KEY_F11:
            self.set_fullscreen(not self.is_fullscreen)
            return True
        if keyval == Gdk.KEY_Escape and self.is_fullscreen:
            self.set_fullscreen(False)
            return True
        return False

    def on_fullscreen_toggled(self, btn: Gtk.ToggleButton):
        self.set_fullscreen(btn.get_active())

    def set_fullscreen(self, enabled: bool):
        self.is_fullscreen = bool(enabled)
        if self.is_fullscreen:
            self.window.fullscreen()
        else:
            self.window.unfullscreen()

        if self.fullscreen_toggle is not None and self.fullscreen_toggle.get_active() != self.is_fullscreen:
            self.fullscreen_toggle.set_active(self.is_fullscreen)

    def refresh_device_info_label(self):
        if self.device_info_label is None:
            return
        cmd_id = 0x600 + self.active_device_id
        status_id = 0x580 + self.active_device_id
        self.device_info_label.set_label(
            f"Device ID: {self.active_device_id}   CMD:0x{cmd_id:03X}   STATUS:0x{status_id:03X}"
        )

    def update_known_devices_label(self):
        if self.known_ids_label is None:
            return
        ids = sorted(self.known_device_ids)
        text = ", ".join(str(v) for v in ids) if ids else "(none)"
        self.known_ids_label.set_label(f"Known device IDs: {text}")

    def refresh_device_stats_view(self):
        if self.device_stats_buffer is None:
            return

        now = time.time()
        lines = []
        for did in sorted(self.device_stats.keys()):
            s = self.device_stats[did]
            seen_age = now - float(s.get("last_seen", 0.0)) if s.get("last_seen", 0.0) else 0.0
            proto = s.get("proto")
            sensors = ",".join(s.get("sensors", [])) or "-"
            streams = ",".join(s.get("streams", [])) or "-"
            frame_counts: dict[int, int] = s.get("frame_counts", {})
            counts_txt = " ".join(f"{k:02X}:{v}" for k, v in sorted(frame_counts.items()))
            lines.append(
                f"id={did:3d} seen={seen_age:5.2f}s proto={proto if proto is not None else '-':>2} "
                f"sensors={sensors:10s} streams={streams:14s} total={int(s.get('frames_total', 0)):6d} "
                f"{counts_txt}"
            )

        text = "\n".join(lines)
        if text != self.last_stats_text:
            self.device_stats_buffer.set_text(text)
            self.last_stats_text = text

    def reset_active_device_runtime(self):
        self.frame_counts.clear()
        self.mag_points.clear()
        self.mag_frozen_points.clear()
        self.acc_points.clear()
        self.env_points.clear()
        self.preview_events.clear()
        self.device_events.clear()
        self.local_event_detector.reset_state()
        self.local_event_cfg_dirty = True
        self.latest_sector_fw = 0
        self.latest_elev_fw = 0
        self.latest_sector_local = 0
        self.latest_elev_local = 0
        self.preview_dirty = True
        self.device_event_dirty = True
        self.mag_dirty = True
        self.acc_dirty = True
        self.env_dirty = True

    def sync_local_event_detector_config(self):
        if not self.local_event_cfg_dirty:
            return
        self.local_event_detector.apply_config(EventDetectionConfig.from_calibration(self.calib))
        self.local_event_cfg_dirty = False

    def log(self, message: str):
        ts = time.strftime("%H:%M:%S")
        self.log_queue.put(f"{ts} {message}")

    def append_log_messages(self):
        lines = []
        for _ in range(200):
            try:
                lines.append(self.log_queue.get_nowait())
            except Empty:
                break

        if not lines:
            return

        end = self.log_buffer.get_end_iter()
        self.log_buffer.insert(end, "\n".join(lines) + "\n")

    def run_async(self, name: str, fn):
        def _runner():
            try:
                fn()
            except Exception as exc:
                self.log(f"[ERR] {name}: {exc}")

        threading.Thread(target=_runner, daemon=True).start()

    def with_lock(self, fn, min_timeout: float | None = None):
        with self.cmd_lock:
            old_timeout = self.client.timeout
            if min_timeout is not None and self.client.timeout < min_timeout:
                self.client.timeout = min_timeout
            try:
                return fn()
            finally:
                self.client.timeout = old_timeout

    def initial_load(self):
        def _load_locked():
            self.action_status_locked()
            self.action_refresh_streams_locked()
            self.action_hmc_get_locked()
            self.action_load_runtime_calib_locked()

        self.with_lock(_load_locked, min_timeout=CMD_TIMEOUT_SHORT)
        self.log("Initial state loaded")

    def monitor_loop(self):
        while self.running:
            try:
                msg = self.monitor_bus.recv(0.1)
                if msg is None:
                    continue
                self.rx_queue.put((int(msg.arbitration_id), bytes(msg.data)))
            except Exception as exc:
                self.log(f"[ERR] monitor: {exc}")
                time.sleep(0.1)

    def on_ui_tick(self):
        if not self.running:
            return False

        self.process_rx_queue()
        self.emit_preview_no_data_if_needed()
        self.append_log_messages()
        self.refresh_status_labels()

        now = time.monotonic()
        if (now - self.last_plot_ts) >= (1.0 / PLOT_MAX_HZ):
            if self.left_active == "mag" and self.mag_dirty:
                self.draw_mag_plot()
                self.mag_dirty = False
                self.last_plot_ts = now
            elif self.left_active == "acc" and self.acc_dirty:
                self.draw_acc_plot()
                self.acc_dirty = False
                self.last_plot_ts = now
            elif self.left_active == "env" and self.env_dirty:
                self.draw_env_plot()
                self.env_dirty = False
                self.last_plot_ts = now

        return True

    def on_left_stack_changed(self, _stack, _pspec):
        name = self.left_stack.get_visible_child_name()
        if name:
            self.left_active = name
        self.mag_dirty = True
        self.acc_dirty = True
        self.env_dirty = True
        self.update_chart_info_label()

    def update_chart_info_label(self):
        if self.chart_meta_label is None:
            return

        show_sector_state = self.left_active == "mag"
        if self.chart_state_row is not None:
            self.chart_state_row.set_visible(show_sector_state)

        local_markup = (
            f"<span size='x-large' weight='bold'>SEC {self.latest_sector_local}</span>   "
            f"<span size='x-large' weight='bold'>ELEV {self.latest_elev_local}</span>"
        )
        if self.local_state_label is not None and local_markup != self.last_local_state_markup:
            self.local_state_label.set_markup(local_markup)
            self.last_local_state_markup = local_markup

        fw_markup = (
            f"<span size='x-large' weight='bold'>SEC {self.latest_sector_fw}</span>   "
            f"<span size='x-large' weight='bold'>ELEV {self.latest_elev_fw}</span>"
        )
        if self.fw_state_label is not None and fw_markup != self.last_fw_state_markup:
            self.fw_state_label.set_markup(fw_markup)
            self.last_fw_state_markup = fw_markup

        if self.left_active == "mag":
            src_points = self.mag_frozen_points if self.mag_freeze_first_n else list(self.mag_points)[-self.chart_points_limit:]
            nth = max(1, self.mag_show_every_n)
            shown = len(src_points[::nth]) if src_points else 0
            text = (
                "MAG 3D detector space | "
                f"center=({self.calib['center_x']},{self.calib['center_y']},{self.calib['center_z']}) "
                f"rot=({self.calib['rotate_xy']/100.0:.1f},{self.calib['rotate_xz']/100.0:.1f},{self.calib['rotate_yz']/100.0:.1f})deg | "
                f"keepout={self.calib['keepout_rad']} z_min={self.calib['z_limit']} z_max={self.calib['z_max']} "
                f"curve={self.calib['elev_curve']/100.0:.2f} sectors={self.calib['num_sectors']} | "
                f"every={nth} freeze={'on' if self.mag_freeze_first_n else 'off'} shown={shown} recorded={len(self.mag_points)}"
            )
        elif self.left_active == "acc":
            samples = decimate_points(
                list(self.acc_points)[-self.chart_points_limit:],
                min(self.chart_points_limit, MAX_RENDER_ACC_POINTS),
            )
            lim = 1200.0
            if self.last_acc_limits is not None:
                lim = float(self.last_acc_limits[1])
            sxy, sxz, syz = compute_plane_slopes_cdeg(
                self.latest_acc_xyz[0],
                self.latest_acc_xyz[1],
                self.latest_acc_xyz[2],
            )
            text = (
                f"ACC 3D | latest=({self.latest_acc_xyz[0]},{self.latest_acc_xyz[1]},{self.latest_acc_xyz[2]})mg | "
                f"slope={self.latest_acc_slope_cdeg}cdeg "
                f"xy={sxy} xz={sxz} yz={syz} cdeg | "
                f"shown={len(samples)} recorded={len(self.acc_points)} axis=+/-{lim:.0f}mg"
            )
        else:
            samples = decimate_points(
                list(self.env_points)[-self.chart_points_limit:],
                min(self.chart_points_limit, MAX_RENDER_ENV_POINTS),
            )
            span_s = 0.0
            if len(samples) >= 2:
                span_s = float(samples[-1][0] - samples[0][0])
            text = (
                f"ENV charts | latest T={self.latest_temp_c:.2f}C RH={self.latest_rh_pct:.2f}% valid={self.latest_env_valid} | "
                f"shown={len(samples)} recorded={len(self.env_points)} window={span_s:.1f}s"
            )

        if text != self.last_chart_meta_text:
            self.chart_meta_label.set_label(text)
            self.last_chart_meta_text = text

    def process_rx_queue(self):
        processed = 0
        while processed < 300:
            try:
                arbitration_id, data = self.rx_queue.get_nowait()
            except Empty:
                break

            processed += 1
            if len(data) < 2:
                continue

            # Per-device accounting from CAN ID.
            device_id = status_id_to_device_id(arbitration_id)
            if device_id is None:
                continue
            now_wall = time.time()
            st = self.device_stats.setdefault(device_id, {
                "last_seen": 0.0,
                "last_mag": 0.0,
                "last_acc": 0.0,
                "last_env": 0.0,
                "last_event": 0.0,
                "sensors": [],
                "streams": [],
                "proto": None,
                "frames_total": 0,
                "frame_counts": {},
            })
            st["last_seen"] = now_wall
            st["frames_total"] = int(st["frames_total"]) + 1
            self.known_device_ids.add(device_id)

            log_fast = self.log_fast_check is not None and self.log_fast_check.get_active()
            subtype = data[1] if data[0] == 0 else -1
            if subtype >= 0:
                sub_counts: dict[int, int] = st["frame_counts"]
                sub_counts[subtype] = sub_counts.get(subtype, 0) + 1
                if subtype == FRAME_MAG:
                    st["last_mag"] = now_wall
                elif subtype == FRAME_ACC:
                    st["last_acc"] = now_wall
                elif subtype == FRAME_ENV:
                    st["last_env"] = now_wall
                elif subtype == FRAME_EVENT:
                    st["last_event"] = now_wall

            if data[0] == 0 and subtype == FRAME_STARTUP and len(data) >= 8:
                st["proto"] = int(data[3])
                st["sensors"] = decode_sensor_bits(data[4])
                st["streams"] = decode_stream_bits(data[5])
            elif data[0] == 0 and subtype == FRAME_STATUS and len(data) >= 8:
                parsed_status = parse_status_frame(data)
                st["sensors"] = parsed_status["sensors"]
                st["streams"] = parsed_status["streams"]
            elif len(data) >= 4 and data[:4] == b"PONG":
                st["proto"] = int(data[5]) if len(data) >= 6 else st["proto"]

            # Ignore payload processing for non-selected devices.
            if device_id != self.active_device_id:
                continue

            if subtype >= 0:
                self.frame_counts[subtype] = self.frame_counts.get(subtype, 0) + 1

            if data[0] == 0 and subtype == FRAME_MAG and len(data) >= 8:
                x = int.from_bytes(data[2:4], "little", signed=True)
                y = int.from_bytes(data[4:6], "little", signed=True)
                z_raw = int.from_bytes(data[6:8], "little", signed=True)
                # Use detector-space Z in GUI so plotted values match event-analysis input.
                z = -z_raw
                self.latest_mag_xyz = (x, y, z)
                self.mag_points.append((x, y, z))
                st["last_mag"] = now_wall
                if self.mag_freeze_first_n and len(self.mag_frozen_points) < self.chart_points_limit:
                    self.mag_frozen_points.append((x, y, z))

                self.update_preview_events_from_mag(x, y, z, time.monotonic())
                self.mag_dirty = True

            elif data[0] == 0 and subtype == FRAME_ACC and len(data) >= 8:
                x = int.from_bytes(data[2:4], "little", signed=True)
                y = int.from_bytes(data[4:6], "little", signed=True)
                z = int.from_bytes(data[6:8], "little", signed=True)
                self.latest_acc_xyz = (x, y, z)
                self.latest_acc_slope_cdeg = compute_slope_cdeg(x, y, z)
                self.acc_points.append((x, y, z))
                st["last_acc"] = now_wall
                self.acc_dirty = True
                if log_fast:
                    self.log(decode_frame(data))

            elif data[0] == 0 and subtype == FRAME_ENV and len(data) >= 7:
                temp_centi = int.from_bytes(data[2:4], "little", signed=True)
                rh_centi = int.from_bytes(data[4:6], "little", signed=False)
                valid = int(data[6])
                self.latest_temp_c = temp_centi / 100.0
                self.latest_rh_pct = rh_centi / 100.0
                self.latest_env_valid = valid
                st["last_env"] = now_wall
                if valid:
                    now_ts = time.time()
                    self.env_points.append((now_ts, self.latest_temp_c, self.latest_rh_pct))
                    self.env_dirty = True
                self.aht_label.set_label(f"AHT20(stream): T={self.latest_temp_c:.2f}C RH={self.latest_rh_pct:.2f}% valid={valid}")
                if log_fast:
                    self.log(decode_frame(data))

            elif data[0] == 0 and subtype == FRAME_EVENT_STATE and len(data) >= 4:
                self.latest_sector_fw = data[2]
                self.latest_elev_fw = data[3]
                if log_fast:
                    self.log(decode_frame(data))

            elif data[0] == 0 and subtype == FRAME_EVENT and len(data) >= 8:
                ev_id = data[2]
                st["last_event"] = now_wall
                ts = time.strftime("%H:%M:%S")
                self.device_events.appendleft(
                    f"{ts} {EVENT_NAMES.get(ev_id, ev_id)} "
                    f"p0={data[3]} p1={data[4]} p2={data[5]} p3={int.from_bytes(data[6:8], 'little')}"
                )
                self.device_event_dirty = True

            elif data[0] == 0 and subtype == FRAME_INTERVAL and len(data) >= 6:
                parsed = parse_interval_frame(data)
                sid = parsed["stream_id"]
                if sid in self.stream_cfg:
                    self.stream_cfg[sid]["enabled"] = bool(parsed["enabled"])
                    self.stream_cfg[sid]["interval_ms"] = int(parsed["interval_ms"])
                    self.update_stream_widgets()

            elif data[0] == 0 and subtype == FRAME_HMC_CFG and len(data) >= 8:
                parsed = parse_hmc_cfg_frame(data)
                self.hmc_cfg["range_id"] = int(parsed["range_id"])
                self.hmc_cfg["data_rate_id"] = int(parsed["data_rate_id"])
                self.hmc_cfg["samples_id"] = int(parsed["samples_id"])
                self.hmc_cfg["mode_id"] = int(parsed["mode_id"])
                self.hmc_cfg["mg_per_digit"] = float(parsed["mg_per_digit"])
                self.update_hmc_widgets()

            elif data[0] == 0 and subtype == FRAME_CALIB_VALUE and len(data) >= 5:
                fid = data[2]
                if fid in CAL_FIELD_ID_TO_NAME:
                    key = CAL_FIELD_ID_TO_NAME[fid]
                    value = int.from_bytes(data[3:5], "little", signed=True)
                    self.calib[key] = value
                    self.set_calib_spin_values()
                    self.local_event_cfg_dirty = True
                    self.mag_dirty = True

            elif data[0] == 0 and subtype == FRAME_AHT20_MEAS and len(data) >= 8:
                meas = parse_aht20_meas_frame(data)
                self.latest_temp_c = meas["temp_c"]
                self.latest_rh_pct = meas["rh_pct"]
                self.latest_env_valid = 1 if meas["crc_ok"] else 0
                self.aht_label.set_label(
                    f"AHT20: T={meas['temp_c']:.2f}C RH={meas['rh_pct']:.2f}% "
                    f"status=0x{meas['status']:02X} crc={int(meas['crc_ok'])}"
                )

            elif data[0] == 0 and subtype == FRAME_AHT20_RAW and len(data) >= 8:
                raw = parse_aht20_raw_frame(data)
                self.aht_raw_label.set_label(f"AHT20 raw: h={raw['raw_h']} t={raw['raw_t']}")

            elif data[0] == 0 and subtype == FRAME_STATUS and len(data) >= 8:
                # Keep status frame out of the bottom log to reduce console noise.
                pass

            else:
                if log_fast:
                    self.log(decode_frame(data))

    def refresh_status_labels(self):
        mx, my, mz = self.latest_mag_xyz
        ax, ay, az = self.latest_acc_xyz
        slope_xy, slope_xz, slope_yz = compute_plane_slopes_cdeg(ax, ay, az)
        now = time.time()

        counts = ", ".join(f"0x{k:02X}:{v}" for k, v in sorted(self.frame_counts.items()))
        status_text = (
            f"dev={self.active_device_id} "
            f"MAG x={mx} y={my} z={mz} | "
            f"ACC x={ax} y={ay} z={az} slope={self.latest_acc_slope_cdeg}cdeg | "
            f"ENV T={self.latest_temp_c:.2f}C RH={self.latest_rh_pct:.2f}% valid={self.latest_env_valid} | "
            f"fw state sec={self.latest_sector_fw} elev={self.latest_elev_fw} | "
            f"frames[{counts}]"
        )
        if status_text != self.last_status_text:
            self.status_label.set_label(status_text)
            self.last_status_text = status_text

        preview_label = (
            f"Preview events: {len(self.preview_events)} | "
            f"local sec={self.latest_sector_local} elev={self.latest_elev_local}"
        )
        if preview_label != self.last_preview_label_text:
            self.preview_label.set_label(preview_label)
            self.last_preview_label_text = preview_label

        acc_text = (
            f"ACC live: x={ax}mg y={ay}mg z={az}mg "
            f"slope={self.latest_acc_slope_cdeg} cdeg "
            f"xy={slope_xy} xz={slope_xz} yz={slope_yz} cdeg"
        )
        if self.acc_status_label is not None and acc_text != self.last_acc_status_text:
            self.acc_status_label.set_label(acc_text)
            self.last_acc_status_text = acc_text

        active_stats = self.device_stats.get(self.active_device_id, {})
        mag_age = now - float(active_stats.get("last_mag", 0.0)) if active_stats.get("last_mag", 0.0) else None
        acc_age = now - float(active_stats.get("last_acc", 0.0)) if active_stats.get("last_acc", 0.0) else None
        env_age = now - float(active_stats.get("last_env", 0.0)) if active_stats.get("last_env", 0.0) else None

        if self.mag_live_label is not None:
            self.mag_live_label.set_label(
                f"MAG live: x={mx} y={my} z={mz} age={'-' if mag_age is None else f'{mag_age:.2f}s'}"
            )
        if self.acc_live_label is not None:
            self.acc_live_label.set_label(
                f"ACC live: x={ax} y={ay} z={az} slope={self.latest_acc_slope_cdeg} cdeg "
                f"age={'-' if acc_age is None else f'{acc_age:.2f}s'}"
            )
        if self.env_live_label is not None:
            self.env_live_label.set_label(
                f"ENV live: T={self.latest_temp_c:.2f}C RH={self.latest_rh_pct:.2f}% valid={self.latest_env_valid} "
                f"age={'-' if env_age is None else f'{env_age:.2f}s'}"
            )

        if self.event_local_buffer is not None and self.preview_dirty:
            text = "\n".join(self.preview_events)
            if text != self.last_event_local_text:
                self.event_local_buffer.set_text(text)
                self.last_event_local_text = text
            self.preview_dirty = False

        if self.event_device_buffer is not None and self.device_event_dirty:
            text = "\n".join(self.device_events)
            if text != self.last_event_device_text:
                self.event_device_buffer.set_text(text)
                self.last_event_device_text = text
            self.device_event_dirty = False

        self.update_known_devices_label()
        self.refresh_device_info_label()
        self.refresh_device_stats_view()
        self.update_chart_info_label()

    def draw_mag_plot(self):
        if self.mag_ax is None or self.mag_canvas is None:
            return

        center = np.array([
            float(self.calib["center_x"]),
            float(self.calib["center_y"]),
            0.0,
        ], dtype=float)

        if self.mag_freeze_first_n:
            src_points = self.mag_frozen_points
        else:
            src_points = list(self.mag_points)[-self.chart_points_limit:]

        nth = max(1, self.mag_show_every_n)
        if nth > 1:
            src_points = src_points[::nth]

        samples = decimate_points(
            src_points,
            min(max(1, len(src_points)), MAX_RENDER_MAG_POINTS),
        )
        if samples:
            arr = np.array(samples, dtype=float)
            arr_det = transform_mag_points_for_detector(arr, self.calib)
            self.mag_scatter._offsets3d = (arr_det[:, 0], arr_det[:, 1], arr_det[:, 2])
            self.mag_scatter.set_array(np.linspace(0.0, 1.0, len(arr_det)))
        else:
            empty = np.array([], dtype=float)
            self.mag_scatter._offsets3d = (empty, empty, empty)
            self.mag_scatter.set_array(empty)

        lx, ly, lz = self.latest_mag_xyz
        ltx, lty, ltz = transform_mag_point_for_detector(float(lx), float(ly), float(lz), self.calib)
        self.mag_last_scatter._offsets3d = ([ltx], [lty], [ltz])
        self.mag_center_scatter._offsets3d = ([center[0]], [center[1]], [center[2]])

        keepout = float(max(0, self.calib["keepout_rad"]))
        radius = float(max(100.0, self.calib["data_radius"]))
        z_limit_world = float(self.calib["z_limit"])
        z_max_world = float(self.calib["z_max"])
        angles = np.linspace(0.0, 2.0 * np.pi, 90)

        keep_x = center[0] + keepout * np.cos(angles)
        keep_y = center[1] + keepout * np.sin(angles)
        keep_z = np.full_like(keep_x, z_limit_world)
        self.mag_keep_line.set_data_3d(keep_x, keep_y, keep_z)

        outer_x = center[0] + radius * np.cos(angles)
        outer_y = center[1] + radius * np.sin(angles)
        outer_z = np.full_like(outer_x, z_limit_world)
        self.mag_outer_line.set_data_3d(outer_x, outer_y, outer_z)
        zmax_z = np.full_like(outer_x, z_max_world)
        self.mag_zmax_line.set_data_3d(outer_x, outer_y, zmax_z)

        num_sectors = max(1, min(16, int(self.calib.get("num_sectors", 6))))
        for idx, line in enumerate(self.mag_sector_lines):
            if idx < num_sectors:
                deg = 360.0 * (float(idx) / float(num_sectors))
                rad = math.radians(deg)
                x2 = center[0] + radius * math.cos(rad)
                y2 = center[1] + radius * math.sin(rad)
                line.set_data_3d([center[0], x2], [center[1], y2], [z_limit_world, z_limit_world])
            else:
                line.set_data_3d([], [], [])
        for idx, txt in enumerate(self.mag_sector_labels):
            if idx < num_sectors:
                # Place sector numbers on z_min plane, centered within each sector slice.
                mid_deg = 360.0 * ((float(idx) + 0.5) / float(num_sectors))
                mid_rad = math.radians(mid_deg)
                label_r = max(40.0, radius * 0.87)
                lx = center[0] + label_r * math.cos(mid_rad)
                ly = center[1] + label_r * math.sin(mid_rad)
                txt.set_position_3d((lx, ly, z_limit_world), zdir=None)
                txt.set_text(str(idx + 1))
                txt.set_visible(True)
            else:
                txt.set_text("")
                txt.set_visible(False)

        lim = radius + 200.0
        z_center = (z_limit_world + z_max_world) * 0.5
        mag_limits = (
            center[0] - lim,
            center[0] + lim,
            center[1] - lim,
            center[1] + lim,
            z_center - lim,
            z_center + lim,
        )
        if self.last_mag_limits != mag_limits:
            self.mag_ax.set_xlim(mag_limits[0], mag_limits[1])
            self.mag_ax.set_ylim(mag_limits[2], mag_limits[3])
            self.mag_ax.set_zlim(mag_limits[4], mag_limits[5])
            self.last_mag_limits = mag_limits

        if self.last_mag_title_text:
            self.mag_ax.set_title("")
            self.last_mag_title_text = ""

        self.mag_info_text.set_text("")
        self.mag_canvas.draw_idle()

    def draw_acc_plot(self):
        if self.acc_ax is None or self.acc_canvas is None:
            return

        samples = decimate_points(
            list(self.acc_points)[-self.chart_points_limit:],
            min(self.chart_points_limit, MAX_RENDER_ACC_POINTS),
        )
        if samples:
            arr = np.array(samples, dtype=float)
            self.acc_scatter._offsets3d = (arr[:, 0], arr[:, 1], arr[:, 2])
            self.acc_scatter.set_array(np.linspace(0.0, 1.0, len(arr)))
            lim = float(max(1200.0, np.max(np.abs(arr)) + 200.0))
        else:
            empty = np.array([], dtype=float)
            self.acc_scatter._offsets3d = (empty, empty, empty)
            self.acc_scatter.set_array(empty)
            lim = 1200.0

        x, y, z = self.latest_acc_xyz
        self.acc_last_scatter._offsets3d = ([float(x)], [float(y)], [float(z)])
        self.acc_vec_line.set_data_3d([0.0, float(x)], [0.0, float(y)], [0.0, float(z)])

        acc_limits = (-lim, lim, -lim, lim, -lim, lim)
        if self.last_acc_limits != acc_limits:
            self.acc_ax.set_xlim(acc_limits[0], acc_limits[1])
            self.acc_ax.set_ylim(acc_limits[2], acc_limits[3])
            self.acc_ax.set_zlim(acc_limits[4], acc_limits[5])
            self.last_acc_limits = acc_limits

        if self.last_acc_title_text:
            self.acc_ax.set_title("")
            self.last_acc_title_text = ""
        self.acc_canvas.draw_idle()

    def draw_env_plot(self):
        if self.env_ax is None or self.env_canvas is None:
            return

        samples = decimate_points(
            list(self.env_points)[-self.chart_points_limit:],
            min(self.chart_points_limit, MAX_RENDER_ENV_POINTS),
        )
        if not samples:
            self.env_temp_line.set_data([], [])
            self.env_rh_line.set_data([], [])
            env_limits = (0.0, 10.0, 0.0, 40.0, 0.0, 100.0)
            if self.last_env_limits != env_limits:
                self.env_ax.set_xlim(env_limits[0], env_limits[1])
                self.env_ax.set_ylim(env_limits[2], env_limits[3])
                self.env_ax_rh.set_ylim(env_limits[4], env_limits[5])
                self.last_env_limits = env_limits
            if self.last_env_title_text:
                self.env_ax.set_title("")
                self.last_env_title_text = ""
            self.env_canvas.draw_idle()
            return

        arr = np.array(samples, dtype=float)
        t = arr[:, 0] - arr[0, 0]
        temp = arr[:, 1]
        rh = arr[:, 2]
        self.env_temp_line.set_data(t, temp)
        self.env_rh_line.set_data(t, rh)

        xmin = float(np.min(t))
        xmax = float(np.max(t))
        if xmax - xmin < 1.0:
            xmax = xmin + 1.0
        env_xlim = (xmin, xmax)

        tmin = float(np.min(temp))
        tmax = float(np.max(temp))
        tpad = max(0.5, (tmax - tmin) * 0.1)
        env_ylim = (tmin - tpad, tmax + tpad)

        rhmin = float(np.min(rh))
        rhmax = float(np.max(rh))
        rhpad = max(1.0, (rhmax - rhmin) * 0.1)
        env_rh_lim = (max(0.0, rhmin - rhpad), min(100.0, rhmax + rhpad))

        env_limits = (env_xlim[0], env_xlim[1], env_ylim[0], env_ylim[1], env_rh_lim[0], env_rh_lim[1])
        if self.last_env_limits != env_limits:
            self.env_ax.set_xlim(env_xlim[0], env_xlim[1])
            self.env_ax.set_ylim(env_ylim[0], env_ylim[1])
            self.env_ax_rh.set_ylim(env_rh_lim[0], env_rh_lim[1])
            self.last_env_limits = env_limits

        if self.last_env_title_text:
            self.env_ax.set_title("")
            self.last_env_title_text = ""
        self.env_canvas.draw_idle()

    def update_preview_events_from_mag(self, x: int, y: int, z: int, now_ts: float):
        self.sync_local_event_detector_config()
        events = self.local_event_detector.process_mag_sample(float(x), float(y), float(z), now_ts)
        sec, elev = self.local_event_detector.get_sector_state()
        self.latest_sector_local = sec
        self.latest_elev_local = elev
        if events:
            ts = time.strftime("%H:%M:%S")
            for evt in events:
                self.preview_events.appendleft(f"{ts} {evt.to_text()}")
            self.preview_dirty = True

    def emit_preview_no_data_if_needed(self):
        active_stats = self.device_stats.get(self.active_device_id, {})
        last_mag = float(active_stats.get("last_mag", 0.0))
        if last_mag <= 0.0:
            return
        self.sync_local_event_detector_config()
        if (time.time() - last_mag) <= (self.local_event_detector.config.session_timeout_ms / 1000.0):
            return
        events = self.local_event_detector.post_no_data(time.monotonic())
        if events:
            ts = time.strftime("%H:%M:%S")
            for evt in events:
                self.preview_events.appendleft(f"{ts} {evt.to_text()}")
            self.preview_dirty = True

    def sim_stamp_now(self) -> int:
        return int(time.monotonic() * 1000.0) & 0xFFFF

    def _sim_get_stamp(self) -> int:
        if self.sim_auto_stamp_check is not None and self.sim_auto_stamp_check.get_active():
            stamp = self.sim_stamp_now()
            if self.sim_p3_spin is not None:
                self.sim_p3_spin.set_value(float(stamp))
            return stamp
        if self.sim_p3_spin is None:
            return self.sim_stamp_now()
        return int(self.sim_p3_spin.get_value())

    def simulate_local_event(self, ev_id: int, p0: int = 0, p1: int = 0, p2: int = 0, p3: int | None = None):
        if p3 is None:
            p3 = self._sim_get_stamp()
        evt = Event(type=int(ev_id), p0=int(p0) & 0xFF, p1=int(p1) & 0xFF, p2=int(p2) & 0xFF, p3=int(p3) & 0xFFFF)
        frame = bytes([
            0x00,
            FRAME_EVENT,
            evt.type & 0xFF,
            evt.p0 & 0xFF,
            evt.p1 & 0xFF,
            evt.p2 & 0xFF,
            evt.p3 & 0xFF,
            (evt.p3 >> 8) & 0xFF,
        ])
        can_id = 0x580 + int(self.active_device_id)
        msg = can.Message(arbitration_id=can_id, is_extended_id=False, data=frame)
        try:
            self.sim_tx_bus.send(msg)
        except Exception as exc:
            self.log(f"[ERR] sim-can-send: {exc}")
            return
        self.log(
            f"Simulated device event via CAN id=0x{can_id:03X}: "
            f"{EVENT_NAMES.get(ev_id, ev_id)} p0={evt.p0} p1={evt.p1} p2={evt.p2} p3={evt.p3}"
        )

    def on_sim_emit_custom(self, *_args):
        if self.sim_event_combo is None:
            return
        ev_id_raw = self.sim_event_combo.get_active_id()
        if ev_id_raw is None:
            return
        ev_id = int(ev_id_raw)
        p0 = int(self.sim_p0_spin.get_value()) if self.sim_p0_spin is not None else 0
        p1 = int(self.sim_p1_spin.get_value()) if self.sim_p1_spin is not None else 0
        p2 = int(self.sim_p2_spin.get_value()) if self.sim_p2_spin is not None else 0
        p3 = self._sim_get_stamp()
        self.simulate_local_event(ev_id, p0, p1, p2, p3)

    def on_sim_quick_activate(self, *_args):
        sector = int(self.sim_sector_spin.get_value()) if self.sim_sector_spin is not None else 1
        elev = int(self.sim_elev_spin.get_value()) if self.sim_elev_spin is not None else 80
        speed = int(self.sim_speed_spin.get_value()) if self.sim_speed_spin is not None else 40
        self.simulate_local_event(1, sector, elev, speed)

    def on_sim_quick_change(self, *_args):
        sector = int(self.sim_sector_spin.get_value()) if self.sim_sector_spin is not None else 1
        to_sector = int(self.sim_to_sector_spin.get_value()) if self.sim_to_sector_spin is not None else 2
        self.simulate_local_event(2, sector, to_sector, 0)

    def on_sim_quick_intensity(self, *_args):
        sector = int(self.sim_sector_spin.get_value()) if self.sim_sector_spin is not None else 1
        elev = int(self.sim_elev_spin.get_value()) if self.sim_elev_spin is not None else 80
        speed = int(self.sim_speed_spin.get_value()) if self.sim_speed_spin is not None else 40
        self.simulate_local_event(3, sector, elev, speed)

    def on_sim_quick_deactivate(self, *_args):
        sector = int(self.sim_sector_spin.get_value()) if self.sim_sector_spin is not None else 1
        self.simulate_local_event(4, sector, 0, 0)

    def on_sim_quick_session_start(self, *_args):
        self.simulate_local_event(5, 0, 0, 0)

    def on_sim_quick_session_end(self, *_args):
        self.simulate_local_event(6, 0, 0, 0)

    def on_sim_quick_no_data(self, *_args):
        self.simulate_local_event(9, 0, 0, 0)

    def on_sim_quick_failure(self, *_args):
        sector = int(self.sim_sector_spin.get_value()) if self.sim_sector_spin is not None else 1
        self.simulate_local_event(8, sector, 0, 0)

    def on_chart_points_changed(self, spin: Gtk.SpinButton):
        self.chart_points_limit = int(spin.get_value())
        if self.mag_freeze_first_n:
            self.mag_frozen_points = list(self.mag_points)[:self.chart_points_limit]
        self.mag_dirty = True
        self.acc_dirty = True
        self.env_dirty = True

    def on_mag_every_n_changed(self, spin: Gtk.SpinButton):
        self.mag_show_every_n = max(1, int(spin.get_value()))
        self.mag_dirty = True

    def on_mag_freeze_toggled(self, check: Gtk.CheckButton):
        self.mag_freeze_first_n = bool(check.get_active())
        if self.mag_freeze_first_n:
            self.mag_frozen_points = list(self.mag_points)[:self.chart_points_limit]
        else:
            self.mag_frozen_points = []
        self.mag_dirty = True

    def on_clear_points(self, *_args):
        self.mag_points.clear()
        self.mag_frozen_points.clear()
        self.acc_points.clear()
        self.env_points.clear()
        self.mag_dirty = True
        self.acc_dirty = True
        self.env_dirty = True
        self.log("Plot data cleared")

    def on_clear_log(self, *_args):
        self.log_buffer.set_text("")

    def on_clear_preview_events(self, *_args):
        self.preview_events.clear()
        self.local_event_detector.reset_state()
        self.local_event_cfg_dirty = True
        self.preview_dirty = True

    def on_clear_device_events(self, *_args):
        self.device_events.clear()
        self.device_event_dirty = True

    def action_log_latest_acc(self):
        x, y, z = self.latest_acc_xyz
        self.log(f"ACC latest: x={x} y={y} z={z} slope={self.latest_acc_slope_cdeg} cdeg")

    def on_auto_center(self, *_args):
        if len(self.mag_points) < 20:
            self.log("[WARN] not enough MAG points for auto center")
            return

        arr = np.array(list(self.mag_points)[-250:], dtype=float)
        cx, cy, cz = np.mean(arr, axis=0)
        self.calib["center_x"] = int(cx)
        self.calib["center_y"] = int(cy)
        self.calib["center_z"] = int(cz)
        self.local_event_cfg_dirty = True
        self.set_calib_spin_values()
        self.mag_dirty = True
        self.log(f"Auto center -> ({int(cx)}, {int(cy)}, {int(cz)})")

        if self.auto_apply_check is not None and self.auto_apply_check.get_active():
            self.pending_apply_fields.update(("center_x", "center_y", "center_z"))
            self.schedule_pending_apply()

    def on_calib_spin_changed(self, spin: Gtk.SpinButton, key: str):
        if self.suppress_spin_events:
            return
        self.calib[key] = int(spin.get_value())
        self.local_event_cfg_dirty = True
        self.mag_dirty = True

        if self.auto_apply_check is not None and self.auto_apply_check.get_active():
            self.pending_apply_fields.add(key)
            self.schedule_pending_apply()

    def schedule_pending_apply(self):
        if self.apply_timer_id != 0:
            return

        def _flush():
            self.apply_timer_id = 0
            self.flush_pending_apply()
            return False

        self.apply_timer_id = GLib.timeout_add(180, _flush)

    def flush_pending_apply(self):
        fields = sorted(self.pending_apply_fields)
        if not fields:
            return
        self.pending_apply_fields.clear()

        def _run():
            def _apply_locked():
                for key in fields:
                    fid = CAL_FIELD_NAME_TO_ID[key]
                    self.client.calib_set(fid, int(self.calib[key]))

            self.with_lock(_apply_locked, min_timeout=CMD_TIMEOUT_SHORT)
            self.log(f"Applied fields: {', '.join(fields)}")

        self.run_async("calib-apply-fields", _run)

    def build_calib_export_payload(self) -> dict:
        streams = {
            str(sid): {
                "enabled": bool(self.stream_cfg[sid]["enabled"]),
                "interval_ms": int(self.stream_cfg[sid]["interval_ms"]),
                "name": STREAM_META.get(sid, f"stream_{sid}"),
            }
            for sid in STREAM_IDS
        }
        hmc = {
            "range_id": int(self.hmc_cfg["range_id"]),
            "data_rate_id": int(self.hmc_cfg["data_rate_id"]),
            "samples_id": int(self.hmc_cfg["samples_id"]),
            "mode_id": int(self.hmc_cfg["mode_id"]),
        }
        return {
            "format": "magnetomuzika-app-settings",
            "version": 1,
            "device_id": int(self.active_device_id),
            "saved_at_unix": int(time.time()),
            "calibration": {key: int(self.calib[key]) for key, *_rest in CALIB_FIELDS},
            "streams": streams,
            "hmc": hmc,
        }

    def parse_calib_import_payload(self, data: dict) -> tuple[list[str], list[str]]:
        src = data.get("calibration") if isinstance(data.get("calibration"), dict) else data
        if not isinstance(src, dict):
            raise ValueError("expected JSON object with calibration values")

        updated: list[str] = []
        clamped: list[str] = []
        for key, *_rest in CALIB_FIELDS:
            if key not in src:
                continue
            value = int(src[key])
            vmin, vmax = CALIB_FIELD_BOUNDS[key]
            clamped_value = max(vmin, min(vmax, value))
            if clamped_value != value:
                clamped.append(key)
            self.calib[key] = int(clamped_value)
            updated.append(key)

        if not updated:
            raise ValueError("no known calibration fields in file")
        return updated, clamped

    def parse_streams_import_payload(self, data: dict) -> tuple[list[int], list[int]]:
        src = data.get("streams")
        if not isinstance(src, dict):
            return [], []

        updated: list[int] = []
        clamped: list[int] = []
        for sid in STREAM_IDS:
            raw = src.get(str(sid))
            if not isinstance(raw, dict):
                continue
            if "enabled" in raw:
                self.stream_cfg[sid]["enabled"] = bool(raw["enabled"])
            if "interval_ms" in raw:
                val = int(raw["interval_ms"])
                val2 = max(STREAM_INTERVAL_MIN_MS, min(STREAM_INTERVAL_MAX_MS, val))
                if val2 != val:
                    clamped.append(sid)
                self.stream_cfg[sid]["interval_ms"] = int(val2)
            updated.append(sid)
        return updated, clamped

    def parse_hmc_import_payload(self, data: dict) -> tuple[list[str], list[str]]:
        src = data.get("hmc")
        if not isinstance(src, dict):
            return [], []

        bounds = {
            "range_id": (0, 7),
            "data_rate_id": (0, 6),
            "samples_id": (0, 3),
            "mode_id": (0, 2),
        }
        updated: list[str] = []
        clamped: list[str] = []
        for key, (vmin, vmax) in bounds.items():
            if key not in src:
                continue
            val = int(src[key])
            val2 = max(vmin, min(vmax, val))
            if val2 != val:
                clamped.append(key)
            self.hmc_cfg[key] = int(val2)
            updated.append(key)
        return updated, clamped

    def on_export_calib_json(self, *_args):
        if self.window is None:
            return
        dialog = Gtk.FileChooserNative.new(
            "Export App Settings JSON",
            self.window,
            Gtk.FileChooserAction.SAVE,
            "Save",
            "Cancel",
        )
        dialog.set_current_name(f"app_settings_dev{int(self.active_device_id)}.json")
        dialog.connect("response", self.on_export_calib_json_response)
        dialog.show()

    def on_export_calib_json_response(self, dialog: Gtk.FileChooserNative, response: int):
        try:
            if response != Gtk.ResponseType.ACCEPT:
                return
            gfile = dialog.get_file()
            if gfile is None:
                return
            path = gfile.get_path()
            if not path:
                self.log("[ERR] export calibration: no filesystem path selected")
                return

            payload = self.build_calib_export_payload()
            with open(path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=True, indent=2, sort_keys=True)
                f.write("\n")
            self.log(f"App settings exported -> {path}")
        except Exception as exc:
            self.log(f"[ERR] export settings: {exc}")
        finally:
            dialog.destroy()

    def on_import_calib_json(self, apply: bool, save: bool):
        if self.window is None:
            return
        dialog = Gtk.FileChooserNative.new(
            "Import App Settings JSON",
            self.window,
            Gtk.FileChooserAction.OPEN,
            "Open",
            "Cancel",
        )
        dialog.connect("response", lambda d, r: self.on_import_calib_json_response(d, r, apply, save))
        dialog.show()

    def on_import_calib_json_response(self, dialog: Gtk.FileChooserNative, response: int, apply: bool, save: bool):
        try:
            if response != Gtk.ResponseType.ACCEPT:
                return
            gfile = dialog.get_file()
            if gfile is None:
                return
            path = gfile.get_path()
            if not path:
                self.log("[ERR] import settings: no filesystem path selected")
                return

            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if not isinstance(payload, dict):
                raise ValueError("root JSON must be an object")

            updated_cal, clamped_cal = self.parse_calib_import_payload(payload)
            updated_streams, clamped_streams = self.parse_streams_import_payload(payload)
            updated_hmc, clamped_hmc = self.parse_hmc_import_payload(payload)
            self.local_event_cfg_dirty = True
            self.mag_dirty = True
            self.acc_dirty = True
            self.env_dirty = True
            self.set_calib_spin_values()
            self.update_stream_widgets()
            self.update_hmc_widgets()
            self.update_chart_info_label()

            clamp_parts: list[str] = []
            if clamped_cal:
                clamp_parts.append("calib=" + ",".join(clamped_cal))
            if clamped_streams:
                clamp_parts.append("streams=" + ",".join(str(s) for s in sorted(set(clamped_streams))))
            if clamped_hmc:
                clamp_parts.append("hmc=" + ",".join(clamped_hmc))
            suffix = f" (clamped: {'; '.join(clamp_parts)})" if clamp_parts else ""
            self.log(
                "Settings imported "
                f"(calib={len(updated_cal)} stream_entries={len(updated_streams)} hmc={len(updated_hmc)})"
                f"{suffix} <- {path}"
            )

            if apply:
                self.run_async(
                    "settings-import-apply",
                    lambda: self.action_apply_all_flash_settings(save=save),
                )
        except Exception as exc:
            self.log(f"[ERR] import settings: {exc}")
        finally:
            dialog.destroy()

    def set_calib_spin_values(self):
        self.suppress_spin_events = True
        try:
            for key, spin in self.calib_spins.items():
                spin.set_value(float(self.calib[key]))
        finally:
            self.suppress_spin_events = False

    def on_stream_switch_changed(self, switch: Gtk.Switch, _pspec, sid: int):
        if self.suppress_stream_events:
            return
        self.stream_cfg[sid]["enabled"] = bool(switch.get_active())

    def on_stream_spin_changed(self, spin: Gtk.SpinButton, sid: int):
        if self.suppress_stream_events:
            return
        self.stream_cfg[sid]["interval_ms"] = int(spin.get_value())

    def update_stream_widgets(self):
        self.suppress_stream_events = True
        try:
            for sid in (1, 2, 3, 4):
                self.stream_switches[sid].set_active(bool(self.stream_cfg[sid]["enabled"]))
                self.stream_spins[sid].set_value(float(self.stream_cfg[sid]["interval_ms"]))
        finally:
            self.suppress_stream_events = False

    def on_hmc_spin_changed(self, _spin: Gtk.SpinButton):
        if self.suppress_hmc_events:
            return
        self.hmc_cfg["range_id"] = int(self.hmc_range_spin.get_value())
        self.hmc_cfg["data_rate_id"] = int(self.hmc_dr_spin.get_value())
        self.hmc_cfg["samples_id"] = int(self.hmc_samples_spin.get_value())
        self.hmc_cfg["mode_id"] = int(self.hmc_mode_spin.get_value())
        self.update_hmc_desc()

    def update_hmc_widgets(self):
        self.suppress_hmc_events = True
        try:
            self.hmc_range_spin.set_value(float(self.hmc_cfg["range_id"]))
            self.hmc_dr_spin.set_value(float(self.hmc_cfg["data_rate_id"]))
            self.hmc_samples_spin.set_value(float(self.hmc_cfg["samples_id"]))
            self.hmc_mode_spin.set_value(float(self.hmc_cfg["mode_id"]))
        finally:
            self.suppress_hmc_events = False
        self.update_hmc_desc()

    def update_hmc_desc(self):
        rid = int(self.hmc_cfg["range_id"])
        did = int(self.hmc_cfg["data_rate_id"])
        sid = int(self.hmc_cfg["samples_id"])
        mid = int(self.hmc_cfg["mode_id"])
        mg = float(self.hmc_cfg.get("mg_per_digit", 0.0))

        desc = (
            f"range={HMC_RANGE_ID_TO_LABEL.get(rid, rid)}ga({rid})  "
            f"data-rate={HMC_DATA_RATE_ID_TO_HZ.get(did, did)}Hz({did})  "
            f"samples={HMC_SAMPLES_ID_TO_COUNT.get(sid, sid)}({sid})  "
            f"mode={HMC_MODE_ID_TO_NAME.get(mid, mid)}({mid})  "
            f"sensitivity={mg:.2f}mG/LSB"
        )
        self.hmc_desc_label.set_label(desc)

    def action_switch_device(self):
        if self.device_id_spin is None:
            return
        new_id = int(self.device_id_spin.get_value())

        def _switch_locked():
            self.client.set_device_id(new_id)
            self.action_status_locked()
            self.action_refresh_streams_locked()
            self.action_hmc_get_locked()
            self.action_load_runtime_calib_locked()

        self.with_lock(_switch_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.active_device_id = new_id
        self.known_device_ids.add(new_id)
        self.reset_active_device_runtime()
        GLib.idle_add(self.refresh_device_info_label)
        GLib.idle_add(self.update_known_devices_label)
        self.log(f"Active device switched to {new_id}")

    def action_discover_devices(self):
        def _scan_locked():
            return discover_devices(
                channel=self.channel,
                interface=self.interface,
                timeout=max(0.30, self.client.timeout),
            )

        found = self.with_lock(_scan_locked, min_timeout=CMD_TIMEOUT_SHORT)
        if not found:
            self.log("Discovery: no devices replied")
            return

        ids = sorted({int(item["device_id"]) for item in found})
        self.known_device_ids.update(ids)
        first = ids[0]
        if self.device_id_spin is not None:
            GLib.idle_add(self.device_id_spin.set_value, float(first))

        summary = ", ".join(
            f"{item['device_id']}(pong={int(bool(item.get('pong')))} proto={item.get('proto')})"
            for item in found
        )
        self.log(f"Discovery: {summary}")

    def action_enable_live_streams(self):
        def _enable_locked():
            self.client.set_stream_enable(1, True)
            self.client.set_stream_enable(2, True)
            self.client.set_stream_enable(3, True)
            self.client.set_stream_enable(4, True)
            self.client.set_interval(1, 100)
            self.client.set_interval(2, 100)
            self.client.set_interval(3, 500)
            self.client.set_interval(4, 100)
            self.action_refresh_streams_locked()

        self.with_lock(_enable_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.log("Live stream preset applied (mag/acc/event=100ms, env=500ms)")

    def action_healthcheck(self):
        checks = []

        def _health_locked():
            pong = self.client.ping()
            st = self.client.get_status()
            hmc = self.client.hmc_get_config()
            aht = self.client.aht20_read()
            return pong, st, hmc, aht

        pong, st, hmc, aht = self.with_lock(_health_locked, min_timeout=CMD_TIMEOUT_LONG)

        checks.append(("Ping", pong is not None))
        sensors = set(st.get("sensors", []))
        checks.append(("HMC present", "hmc" in sensors))
        checks.append(("LIS present", "lis" in sensors))
        checks.append(("AHT present", "aht" in sensors))
        checks.append(("HMC cfg read", hmc.get("range_id", -1) >= 0))
        checks.append(("AHT crc", bool(aht.get("crc_ok", False))))
        checks.append(("AHT humidity sane", 0.0 <= float(aht.get("rh_pct", -1.0)) <= 100.0))
        checks.append(("AHT temperature sane", -40.0 <= float(aht.get("temp_c", -999.0)) <= 90.0))

        active_stats = self.device_stats.get(self.active_device_id, {})
        now = time.time()
        last_mag = float(active_stats.get("last_mag", 0.0))
        last_acc = float(active_stats.get("last_acc", 0.0))
        checks.append(("MAG stream fresh", last_mag > 0.0 and (now - last_mag) < 2.0))
        checks.append(("ACC stream fresh", last_acc > 0.0 and (now - last_acc) < 2.0))

        ok = all(flag for _, flag in checks)
        detail = " | ".join(f"{name}:{'OK' if flag else 'FAIL'}" for name, flag in checks)
        text = ("PASS" if ok else "FAIL") + " " + detail
        if self.health_label is not None:
            GLib.idle_add(self.health_label.set_label, f"Health: {text}")
        self.log(f"Healthcheck {text}")

    def action_ping(self):
        pong = self.with_lock(self.client.ping, min_timeout=CMD_TIMEOUT_SHORT)
        if pong:
            self.log(f"PING OK -> dev={pong[4]} proto={pong[5]} flags=0x{pong[6]:02X}")
        else:
            self.log("PING OK")

    def action_status_locked(self):
        st = self.client.get_status()
        self.log(
            f"STATUS sensors={','.join(st['sensors']) or 'none'} "
            f"streams={','.join(st['streams']) or 'none'}"
        )

    def action_status(self):
        self.with_lock(self.action_status_locked, min_timeout=CMD_TIMEOUT_SHORT)

    def action_enter_bootloader(self):
        self.with_lock(self.client.enter_bootloader, min_timeout=CMD_TIMEOUT_SHORT)
        self.log("ENTER_BOOTLOADER sent")

    def action_refresh_streams_locked(self):
        infos = self.client.get_intervals(0)
        for info in infos:
            sid = int(info["stream_id"])
            if sid in self.stream_cfg:
                self.stream_cfg[sid]["enabled"] = bool(info["enabled"])
                self.stream_cfg[sid]["interval_ms"] = int(info["interval_ms"])
        GLib.idle_add(self.update_stream_widgets)
        self.log("Streams refreshed")

    def action_refresh_streams(self):
        self.with_lock(self.action_refresh_streams_locked, min_timeout=CMD_TIMEOUT_SHORT)

    def action_apply_streams(self, save: bool = False):
        def _apply_locked():
            for sid in (1, 2, 3, 4):
                cfg = self.stream_cfg[sid]
                try:
                    self.client.set_stream_enable(sid, bool(cfg["enabled"]))
                    self.client.set_interval(sid, int(cfg["interval_ms"]))
                except Exception as exc:
                    raise RuntimeError(f"stream {sid} apply failed: {exc}") from exc
            if save:
                try:
                    self.client.calib_save()
                except Exception as exc:
                    raise RuntimeError(f"save failed: {exc}") from exc

        self.with_lock(_apply_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.log("Streams applied" + (" and saved" if save else ""))

    def action_hmc_get_locked(self):
        last_exc = None
        for _ in range(3):
            try:
                info = self.client.hmc_get_config()
                self.hmc_cfg["range_id"] = int(info["range_id"])
                self.hmc_cfg["data_rate_id"] = int(info["data_rate_id"])
                self.hmc_cfg["samples_id"] = int(info["samples_id"])
                self.hmc_cfg["mode_id"] = int(info["mode_id"])
                self.hmc_cfg["mg_per_digit"] = float(info["mg_per_digit"])
                GLib.idle_add(self.update_hmc_widgets)
                self.log("HMC config refreshed")
                return
            except Exception as exc:
                last_exc = exc
                time.sleep(0.05)
        raise last_exc

    def action_hmc_get(self):
        self.with_lock(self.action_hmc_get_locked, min_timeout=CMD_TIMEOUT_LONG)

    def action_hmc_set(self, save: bool = False):
        rid = int(self.hmc_cfg["range_id"])
        did = int(self.hmc_cfg["data_rate_id"])
        sid = int(self.hmc_cfg["samples_id"])
        mid = int(self.hmc_cfg["mode_id"])

        def _set_locked():
            info = self.client.hmc_set_config(rid, did, sid, mid)
            if save:
                self.client.calib_save()
            return info

        info = self.with_lock(_set_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.hmc_cfg["range_id"] = int(info["range_id"])
        self.hmc_cfg["data_rate_id"] = int(info["data_rate_id"])
        self.hmc_cfg["samples_id"] = int(info["samples_id"])
        self.hmc_cfg["mode_id"] = int(info["mode_id"])
        self.hmc_cfg["mg_per_digit"] = float(info["mg_per_digit"])
        GLib.idle_add(self.update_hmc_widgets)
        self.log("HMC config applied" + (" and saved" if save else ""))

    def action_aht20_read(self):
        def _read_locked():
            last_exc = None
            for _ in range(3):
                try:
                    return self.client.aht20_read()
                except Exception as exc:
                    last_exc = exc
                    time.sleep(0.08)
            raise last_exc

        info = self.with_lock(_read_locked, min_timeout=CMD_TIMEOUT_LONG)

        tc = float(info["temp_c"])
        rh = float(info["rh_pct"])
        status = int(info["status"])
        crc = bool(info["crc_ok"])
        raw_h = int(info["raw_h"])
        raw_t = int(info["raw_t"])

        self.latest_temp_c = tc
        self.latest_rh_pct = rh
        self.latest_env_valid = 1 if crc else 0
        now_ts = time.time()
        self.env_points.append((now_ts, tc, rh))
        self.env_dirty = True

        GLib.idle_add(self.aht_label.set_label, f"AHT20: T={tc:.2f}C RH={rh:.2f}% status=0x{status:02X} crc={int(crc)}")
        GLib.idle_add(self.aht_raw_label.set_label, f"AHT20 raw: h={raw_h} t={raw_t}")
        self.log(f"AHT20 T={tc:.2f}C RH={rh:.2f}% status=0x{status:02X} crc={int(crc)} raw_h={raw_h} raw_t={raw_t}")

    def action_load_runtime_calib_locked(self):
        vals = self.client.calib_get(0)
        seen_fids: set[int] = set()
        for item in vals:
            fid = int(item["field_id"])
            if fid in CAL_FIELD_ID_TO_NAME:
                self.calib[CAL_FIELD_ID_TO_NAME[fid]] = int(item["value"])
                seen_fids.add(fid)

        missing_keys: list[str] = []
        for key, *_rest in CALIB_FIELDS:
            fid = int(CAL_FIELD_NAME_TO_ID[key])
            if fid in seen_fids:
                continue
            missing_keys.append(key)
            single = self.client.calib_get(fid)
            if single:
                self.calib[key] = int(single[0]["value"])
                seen_fids.add(fid)

        self.local_event_cfg_dirty = True
        GLib.idle_add(self.set_calib_spin_values)
        self.mag_dirty = True
        if missing_keys:
            self.log(
                f"Runtime calibration loaded (all={len(vals)}; recovered={len(missing_keys)} "
                f"missing fields: {', '.join(missing_keys)})"
            )
        else:
            self.log(f"Runtime calibration loaded ({len(vals)} fields)")

    def action_load_runtime_calib(self):
        self.with_lock(self.action_load_runtime_calib_locked, min_timeout=CMD_TIMEOUT_LONG)

    def action_apply_all_calib(self, save: bool = False):
        def _apply_locked():
            for key, fid in CAL_FIELD_NAME_TO_ID.items():
                if fid == 0:
                    continue
                self.client.calib_set(fid, int(self.calib[key]))
            if save:
                self.client.calib_save()

        self.with_lock(_apply_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.log("All calibration fields applied" + (" and saved" if save else ""))

    def action_apply_all_flash_settings(self, save: bool = False):
        def _apply_locked():
            for key, fid in CAL_FIELD_NAME_TO_ID.items():
                if fid == 0:
                    continue
                self.client.calib_set(fid, int(self.calib[key]))

            for sid in STREAM_IDS:
                cfg = self.stream_cfg[sid]
                self.client.set_stream_enable(sid, bool(cfg["enabled"]))
                self.client.set_interval(sid, int(cfg["interval_ms"]))

            info = self.client.hmc_set_config(
                int(self.hmc_cfg["range_id"]),
                int(self.hmc_cfg["data_rate_id"]),
                int(self.hmc_cfg["samples_id"]),
                int(self.hmc_cfg["mode_id"]),
            )
            self.hmc_cfg["range_id"] = int(info["range_id"])
            self.hmc_cfg["data_rate_id"] = int(info["data_rate_id"])
            self.hmc_cfg["samples_id"] = int(info["samples_id"])
            self.hmc_cfg["mode_id"] = int(info["mode_id"])
            self.hmc_cfg["mg_per_digit"] = float(info["mg_per_digit"])

            if save:
                self.client.calib_save()

        self.with_lock(_apply_locked, min_timeout=CMD_TIMEOUT_LONG)
        GLib.idle_add(self.update_hmc_widgets)
        GLib.idle_add(self.update_stream_widgets)
        self.log("All flash-saved settings applied" + (" and saved" if save else ""))

    def action_calib_save(self):
        self.with_lock(self.client.calib_save, min_timeout=CMD_TIMEOUT_LONG)
        self.log("Calibration saved to flash")

    def action_calib_load(self):
        def _load_locked():
            self.client.calib_load()
            self.action_load_runtime_calib_locked()
            self.action_refresh_streams_locked()
            self.action_hmc_get_locked()

        self.with_lock(_load_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.log("Calibration loaded from flash")

    def action_calib_reset(self):
        def _reset_locked():
            self.client.calib_reset()
            self.action_load_runtime_calib_locked()
            self.action_refresh_streams_locked()
            self.action_hmc_get_locked()

        self.with_lock(_reset_locked, min_timeout=CMD_TIMEOUT_LONG)
        self.log("Calibration reset to defaults (RAM)")

    def action_capture_earth(self):
        def _capture_locked():
            self.client.calib_capture_earth()
            return self.client.calib_get(0)

        vals = self.with_lock(_capture_locked, min_timeout=CMD_TIMEOUT_LONG)
        for item in vals:
            fid = item["field_id"]
            if fid in (13, 14, 15, 16):
                self.calib[CAL_FIELD_ID_TO_NAME[fid]] = int(item["value"])
        self.local_event_cfg_dirty = True
        GLib.idle_add(self.set_calib_spin_values)
        self.mag_dirty = True
        self.log("Captured earth reference")


def main():
    parser = argparse.ArgumentParser(description="GTK control/calibration tool over CAN")
    parser.add_argument("--channel", default="slcan0", help="CAN channel (default: slcan0)")
    parser.add_argument("--interface", default="socketcan", help="python-can interface")
    parser.add_argument("--device-id", type=int, default=1, help="device id 0..127")
    parser.add_argument("--timeout", type=float, default=2.5, help="command timeout seconds")
    args = parser.parse_args()

    app = CalibApp(
        channel=args.channel,
        interface=args.interface,
        device_id=args.device_id,
        timeout=args.timeout,
    )
    app.run(None)


if __name__ == "__main__":
    main()
