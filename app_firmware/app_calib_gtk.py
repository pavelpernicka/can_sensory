#!/usr/bin/env python3
"""
GTK calibration and visualization app
"""

import argparse
import math
import threading
import time
from collections import deque

import can
import gi
import numpy as np
from matplotlib.backends.backend_gtk4agg import FigureCanvasGTK4Agg as FigureCanvas
from matplotlib.figure import Figure

from app_can_tool import (
    AppCanClient,
    CAL_FIELD_ID_TO_NAME,
    CAL_FIELD_NAME_TO_ID,
    FRAME_CALIB_VALUE,
    FRAME_EVENT_STATE,
    FRAME_MAG,
)

gi.require_version("Gtk", "4.0")
from gi.repository import GLib, Gtk


SPIN_FIELDS = [
    ("center_x", "Center X (mG)"),
    ("center_y", "Center Y (mG)"),
    ("center_z", "Center Z (mG)"),
    ("rotate_xy", "Rotate XY (deg*100)"),
    ("rotate_xz", "Rotate XZ (deg*100)"),
    ("rotate_yz", "Rotate YZ (deg*100)"),
    ("keepout_rad", "Keepout Radius"),
    ("z_limit", "Z Limit"),
    ("data_radius", "Data Radius"),
    ("mag_offset_x", "Mag Offset X"),
    ("mag_offset_y", "Mag Offset Y"),
    ("mag_offset_z", "Mag Offset Z"),
]


def rotate_3d(x: float, y: float, z: float, axy: float, axz: float, ayz: float) -> tuple[float, float, float]:
    rad_xy = math.radians(axy)
    x1 = x * math.cos(rad_xy) - y * math.sin(rad_xy)
    y1 = x * math.sin(rad_xy) + y * math.cos(rad_xy)

    rad_xz = math.radians(axz)
    x2 = x1 * math.cos(rad_xz) - z * math.sin(rad_xz)
    z2 = x1 * math.sin(rad_xz) + z * math.cos(rad_xz)

    rad_yz = math.radians(ayz)
    y3 = y1 * math.cos(rad_yz) - z2 * math.sin(rad_yz)
    z3 = y1 * math.sin(rad_yz) + z2 * math.cos(rad_yz)
    return x2, y3, z3


class CalibApp(Gtk.Application):
    def __init__(self, channel: str, interface: str, device_id: int, timeout: float):
        super().__init__(application_id="cz.magnetomuzika.Calibration")
        self.channel = channel
        self.interface = interface
        self.device_id = device_id
        self.timeout = timeout

        self.client = AppCanClient(channel, interface, device_id, timeout)
        self.monitor_bus = can.Bus(
            interface=interface,
            channel=channel,
            receive_own_messages=False,
            can_filters=[{
                "can_id": self.client.status_id,
                "can_mask": 0x7FF,
                "extended": False,
            }],
        )

        self.running = True
        self.cmd_lock = threading.Lock()
        self.points = deque(maxlen=1500)
        self.latest_sector = 0
        self.latest_elevation = 0
        self.latest_xyz = (0, 0, 0)

        self.calib = {
            "center_x": 0,
            "center_y": 0,
            "center_z": 0,
            "rotate_xy": 0,
            "rotate_xz": 0,
            "rotate_yz": 0,
            "keepout_rad": 1000,
            "z_limit": 150,
            "data_radius": 3000,
            "mag_offset_x": 0,
            "mag_offset_y": 0,
            "mag_offset_z": 0,
            "earth_x": 0,
            "earth_y": 0,
            "earth_z": 0,
            "earth_valid": 0,
        }
        self.spin_widgets: dict[str, Gtk.SpinButton] = {}
        self.suppress_spin_events = False

        self.window = None
        self.ax = None
        self.canvas = None
        self.status_label = None
        self.log_buffer = None

    def do_activate(self):
        self.window = Gtk.ApplicationWindow(application=self)
        self.window.set_title("Magnetometer CAN Calibration")
        self.window.set_default_size(1300, 800)
        self.window.connect("close-request", self.on_close_request)

        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.window.set_child(root)

        fig = Figure(figsize=(7, 6), dpi=100)
        self.ax = fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvas(fig)
        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.append(self.canvas)
        self.status_label = Gtk.Label(label="No data yet")
        left.append(self.status_label)
        root.append(left)

        right_scroll = Gtk.ScrolledWindow()
        right_scroll.set_min_content_width(420)
        root.append(right_scroll)

        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, margin_start=8, margin_end=8, margin_top=8, margin_bottom=8)
        right_scroll.set_child(right)

        for key, label in SPIN_FIELDS:
            row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
            row.append(Gtk.Label(label=label, xalign=0))
            spin = Gtk.SpinButton()
            spin.set_range(-32768, 32767)
            spin.set_increments(1, 10)
            spin.set_numeric(True)
            spin.set_value(self.calib[key])
            spin.connect("value-changed", self.on_spin_changed, key)
            row.append(spin)
            self.spin_widgets[key] = spin
            right.append(row)

        btn_row1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        btn_load = Gtk.Button(label="Load From Device")
        btn_load.connect("clicked", self.on_load_clicked)
        btn_apply = Gtk.Button(label="Apply All")
        btn_apply.connect("clicked", self.on_apply_clicked)
        btn_save = Gtk.Button(label="Save To Flash")
        btn_save.connect("clicked", self.on_save_clicked)
        btn_row1.append(btn_load)
        btn_row1.append(btn_apply)
        btn_row1.append(btn_save)
        right.append(btn_row1)

        btn_row2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        btn_reset = Gtk.Button(label="Reset Defaults")
        btn_reset.connect("clicked", self.on_reset_clicked)
        btn_capture = Gtk.Button(label="Capture Earth")
        btn_capture.connect("clicked", self.on_capture_earth_clicked)
        btn_center = Gtk.Button(label="Auto Center (200 pts)")
        btn_center.connect("clicked", self.on_auto_center_clicked)
        btn_row2.append(btn_reset)
        btn_row2.append(btn_capture)
        btn_row2.append(btn_center)
        right.append(btn_row2)

        right.append(Gtk.Label(label="Log", xalign=0))
        self.log_buffer = Gtk.TextBuffer()
        log_view = Gtk.TextView(buffer=self.log_buffer)
        log_view.set_editable(False)
        log_view.set_monospace(True)
        log_scroll = Gtk.ScrolledWindow()
        log_scroll.set_min_content_height(220)
        log_scroll.set_child(log_view)
        right.append(log_scroll)

        self.window.present()

        self.log("Starting monitor thread...")
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        GLib.timeout_add(200, self.update_plot)
        GLib.idle_add(self.load_from_device)

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
        return False

    def log(self, message: str):
        def _append():
            end = self.log_buffer.get_end_iter()
            self.log_buffer.insert(end, f"{time.strftime('%H:%M:%S')} {message}\n")
            return False
        GLib.idle_add(_append)

    def on_spin_changed(self, spin: Gtk.SpinButton, key: str):
        if self.suppress_spin_events:
            return
        self.calib[key] = int(spin.get_value())

    def set_spin_values(self):
        self.suppress_spin_events = True
        try:
            for key, spin in self.spin_widgets.items():
                spin.set_value(float(self.calib[key]))
        finally:
            self.suppress_spin_events = False

    def load_from_device(self):
        try:
            with self.cmd_lock:
                vals = self.client.calib_get(0)
            for item in vals:
                key = CAL_FIELD_ID_TO_NAME[item["field_id"]]
                self.calib[key] = item["value"]
            self.set_spin_values()
            self.log("Calibration loaded from device")
        except Exception as exc:
            self.log(f"[ERR] load_from_device: {exc}")
        return False

    def apply_all(self):
        try:
            with self.cmd_lock:
                for key, fid in CAL_FIELD_NAME_TO_ID.items():
                    if fid == 0:
                        continue
                    self.client.calib_set(fid, int(self.calib[key]))
            self.log("Calibration applied to device")
        except Exception as exc:
            self.log(f"[ERR] apply_all: {exc}")

    def on_load_clicked(self, _btn):
        GLib.idle_add(self.load_from_device)

    def on_apply_clicked(self, _btn):
        threading.Thread(target=self.apply_all, daemon=True).start()

    def on_save_clicked(self, _btn):
        def run():
            try:
                with self.cmd_lock:
                    self.client.calib_save()
                self.log("Calibration saved to flash")
            except Exception as exc:
                self.log(f"[ERR] save: {exc}")
        threading.Thread(target=run, daemon=True).start()

    def on_reset_clicked(self, _btn):
        def run():
            try:
                with self.cmd_lock:
                    self.client.calib_reset()
                    vals = self.client.calib_get(0)
                for item in vals:
                    self.calib[CAL_FIELD_ID_TO_NAME[item["field_id"]]] = item["value"]
                GLib.idle_add(self.set_spin_values)
                self.log("Calibration reset to defaults (RAM)")
            except Exception as exc:
                self.log(f"[ERR] reset: {exc}")
        threading.Thread(target=run, daemon=True).start()

    def on_capture_earth_clicked(self, _btn):
        def run():
            try:
                with self.cmd_lock:
                    self.client.calib_capture_earth()
                    for fid in (13, 14, 15, 16):
                        value = self.client.calib_get(fid)[0]
                        self.calib[CAL_FIELD_ID_TO_NAME[fid]] = value["value"]
                GLib.idle_add(self.set_spin_values)
                self.log("Captured earth magnetic reference")
            except Exception as exc:
                self.log(f"[ERR] capture-earth: {exc}")
        threading.Thread(target=run, daemon=True).start()

    def on_auto_center_clicked(self, _btn):
        if len(self.points) < 20:
            self.log("[WARN] not enough points for auto center")
            return
        arr = np.array(list(self.points)[-200:])
        cx = int(np.mean(arr[:, 0]))
        cy = int(np.mean(arr[:, 1]))
        cz = int(np.mean(arr[:, 2]))
        self.calib["center_x"] = cx
        self.calib["center_y"] = cy
        self.calib["center_z"] = cz
        self.set_spin_values()
        self.log(f"Auto center set: ({cx}, {cy}, {cz})")

    def compute_section(self, x: int, y: int, z: int) -> tuple[int, int]:
        xr, yr, zr = rotate_3d(
            float(x),
            float(y),
            float(z - self.calib["center_z"]),
            self.calib["rotate_xy"] / 100.0,
            self.calib["rotate_xz"] / 100.0,
            self.calib["rotate_yz"] / 100.0,
        )
        dx = xr - self.calib["center_x"]
        dy = yr - self.calib["center_y"]
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < self.calib["keepout_rad"] or zr < self.calib["z_limit"]:
            return 0, 0
        angle = math.degrees(math.atan2(dy, dx)) % 360.0
        section = int(angle // 60.0) + 1
        elevation = int(max(0.0, min(255.0, zr - self.calib["z_limit"])))
        return section, elevation

    def monitor_loop(self):
        while self.running:
            try:
                msg = self.monitor_bus.recv(0.2)
                if msg is None:
                    continue
                data = bytes(msg.data)
                if len(data) < 2 or data[0] != 0:
                    continue
                subtype = data[1]

                if subtype == FRAME_MAG and len(data) >= 8:
                    x = int.from_bytes(data[2:4], "little", signed=True)
                    y = int.from_bytes(data[4:6], "little", signed=True)
                    z = int.from_bytes(data[6:8], "little", signed=True)
                    self.latest_xyz = (x, y, z)
                    sec, elev = self.compute_section(x, y, z)
                    self.latest_sector = sec
                    self.latest_elevation = elev
                    self.points.append((x, y, z))

                elif subtype == FRAME_EVENT_STATE and len(data) >= 4:
                    self.latest_sector = data[2]
                    self.latest_elevation = data[3]

                elif subtype == FRAME_CALIB_VALUE and len(data) >= 5:
                    fid = data[2]
                    if fid in CAL_FIELD_ID_TO_NAME:
                        value = int.from_bytes(data[3:5], "little", signed=True)
                        self.calib[CAL_FIELD_ID_TO_NAME[fid]] = value
                        GLib.idle_add(self.set_spin_values)
            except Exception as exc:
                self.log(f"[ERR] monitor: {exc}")

    def draw_boundaries(self):
        radius = max(100.0, float(self.calib["data_radius"]))
        center_x = float(self.calib["center_x"])
        center_y = float(self.calib["center_y"])
        center_z = float(self.calib["center_z"])
        keepout = max(0.0, float(self.calib["keepout_rad"]))

        angles = np.linspace(0, 2 * np.pi, 120)
        keep_x = center_x + keepout * np.cos(angles)
        keep_y = center_y + keepout * np.sin(angles)
        keep_z = np.full_like(keep_x, center_z)
        self.ax.plot(keep_x, keep_y, keep_z, "r--", linewidth=1.0)

        for deg in range(0, 360, 60):
            rad = math.radians(deg)
            x2 = center_x + radius * math.cos(rad)
            y2 = center_y + radius * math.sin(rad)
            self.ax.plot([center_x, x2], [center_y, y2], [center_z, center_z], "k--", linewidth=0.8)

    def update_plot(self):
        if not self.running:
            return False

        self.ax.clear()
        if self.points:
            arr = np.array(self.points)
            self.ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2], c=np.linspace(0, 1, len(arr)), cmap="viridis", s=8)
        self.draw_boundaries()
        self.ax.set_xlabel("X (mG)")
        self.ax.set_ylabel("Y (mG)")
        self.ax.set_zlabel("Z (mG)")
        self.ax.set_title(f"Section={self.latest_sector} Elev={self.latest_elevation}")
        self.canvas.draw()

        x, y, z = self.latest_xyz
        self.status_label.set_label(f"Latest MAG: x={x} y={y} z={z} | section={self.latest_sector} elev={self.latest_elevation}")
        return True


def main():
    parser = argparse.ArgumentParser(description="GTK magnetometer calibration tool over CAN")
    parser.add_argument("--channel", default="slcan0", help="CAN channel (default: slcan0)")
    parser.add_argument("--interface", default="socketcan", help="python-can interface")
    parser.add_argument("--device-id", type=int, default=1, help="device id 0..127")
    parser.add_argument("--timeout", type=float, default=1.0, help="command timeout seconds")
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
