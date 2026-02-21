#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field

import can

try:
    import fluidsynth
except ModuleNotFoundError:
    fluidsynth = None

from app_can_tool import AppCanClient
from rhytmics_io import (
    CanQueueListener,
    DeviceConfig,
    LedConfig,
    load_device_configs,
    load_global_player_config,
    resolve_local,
)

CMD_WS_SET_ALL = 0x53
CMD_WS_SET_ANIM = 0x55
CMD_WS_SET_SECTOR_ZONE = 0x5D
CMD_WS_SET_LENGTH = 0x5F


@dataclass
class DeviceVoice:
    cfg: DeviceConfig
    channel: int
    held_notes: set[int] = field(default_factory=set)
    note_on_counts: dict[int, int] = field(default_factory=dict)
    note_deadlines: dict[int, deque[float]] = field(default_factory=dict)
    program_key: tuple[int, int, int] | None = None


@dataclass
class LedDeviceState:
    cfg: LedConfig
    simple_mode: bool = False
    is_playing: bool = False
    current_stops: list[tuple[int, int]] = field(default_factory=list)  # (pos,color565)
    last_keepalive_s: float = 0.0


class LedCanController:
    def __init__(
        self,
        bus: can.BusABC,
        devices: dict[int, DeviceConfig],
        debug: bool = False,
        simple_devices: set[int] | None = None,
    ):
        self.bus = bus
        self.debug = bool(debug)
        self._states: dict[int, LedDeviceState] = {}
        simple_set = set() if simple_devices is None else {int(x) for x in simple_devices}
        for did, cfg in devices.items():
            if cfg.led is not None and cfg.led.enabled:
                self._states[int(did)] = LedDeviceState(
                    cfg=cfg.led,
                    simple_mode=(int(did) in simple_set),
                )
        self._q: queue.Queue[tuple[int, bytes, int]] = queue.Queue(maxsize=8192)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if self._states:
            self._thread = threading.Thread(target=self._worker, daemon=True, name="rhytmics-led")

    def enabled(self) -> bool:
        return bool(self._states)

    def start(self) -> None:
        if self._thread is None:
            return
        self._thread.start()
        for did in sorted(self._states.keys()):
            self.apply_base(did, clear_all=True)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def _queue_cmd(self, did: int, payload: bytes, retries: int) -> None:
        if did not in self._states:
            return
        item = (int(did), bytes(payload[:8]), int(max(0, retries)))
        try:
            self._q.put_nowait(item)
        except queue.Full:
            try:
                self._q.get_nowait()
                self._q.put_nowait(item)
            except (queue.Empty, queue.Full):
                pass

    @staticmethod
    def _cmd_set_length(strip_len: int) -> bytes:
        return bytes([CMD_WS_SET_LENGTH, int(max(1, min(255, strip_len))) & 0xFF])

    @staticmethod
    def _cmd_set_all(enabled: bool, brightness: int, r: int, g: int, b: int) -> bytes:
        return bytes(
            [
                CMD_WS_SET_ALL,
                1 if enabled else 0,
                int(max(0, min(255, brightness))) & 0xFF,
                int(max(0, min(255, r))) & 0xFF,
                int(max(0, min(255, g))) & 0xFF,
                int(max(0, min(255, b))) & 0xFF,
            ]
        )

    @staticmethod
    def _cmd_set_zone(idx: int, pos: int, color565: int) -> bytes:
        c = int(max(0, min(0xFFFF, color565)))
        return bytes(
            [
                CMD_WS_SET_SECTOR_ZONE,
                int(max(1, min(32, idx))) & 0xFF,
                int(max(0, min(255, pos))) & 0xFF,
                c & 0xFF,
                (c >> 8) & 0xFF,
            ]
        )

    @staticmethod
    def _cmd_set_anim(mode: int, speed: int) -> bytes:
        return bytes([CMD_WS_SET_ANIM, int(max(0, min(6, mode))) & 0xFF, int(max(0, min(255, speed))) & 0xFF])

    @staticmethod
    def _rgb565_to_rgb888(c: int) -> tuple[int, int, int]:
        c = int(max(0, min(0xFFFF, c)))
        r5 = (c >> 11) & 0x1F
        g6 = (c >> 5) & 0x3F
        b5 = c & 0x1F
        r = (r5 * 255 + 15) // 31
        g = (g6 * 255 + 31) // 63
        b = (b5 * 255 + 15) // 31
        return int(r), int(g), int(b)

    @staticmethod
    def _stops_to_pairs(stops: list) -> list[tuple[int, int]]:
        out: list[tuple[int, int]] = []
        for s in stops[:32]:
            try:
                out.append((int(s.pos), int(s.color_rgb565)))
            except Exception:
                continue
        return out

    def _stops_for_state(self, state: LedDeviceState, playing: bool) -> list[tuple[int, int]]:
        cfg = state.cfg
        if playing and cfg.play_gradient:
            return self._stops_to_pairs(cfg.play_gradient)
        return self._stops_to_pairs(cfg.gradient)

    def _drop_pending_for_device(self, did: int) -> None:
        kept: list[tuple[int, bytes, int]] = []
        dropped = 0
        while True:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            if int(item[0]) == int(did):
                dropped += 1
            else:
                kept.append(item)
        for item in kept:
            try:
                self._q.put_nowait(item)
            except queue.Full:
                break
        if dropped > 0 and self.debug:
            print(f"[dbg led] dropped {dropped} stale pending cmds for dev={did:02d}")

    def _queue_apply_stops(
        self,
        did: int,
        state: LedDeviceState,
        stops: list[tuple[int, int]],
        clear_all: bool,
        set_state: bool,
        anim_speed: int,
    ) -> None:
        cfg = state.cfg
        retries = int(cfg.send_retries)
        if set_state and cfg.strip_len is not None:
            self._queue_cmd(did, self._cmd_set_length(cfg.strip_len), retries)
        if set_state:
            if stops:
                base_r, base_g, base_b = self._rgb565_to_rgb888(stops[0][1])
            else:
                base_r, base_g, base_b = (255, 255, 255)
            self._queue_cmd(did, self._cmd_set_all(True, cfg.brightness, base_r, base_g, base_b), retries)

        if state.simple_mode:
            # Minimal fallback path for older firmware.
            self._queue_cmd(did, self._cmd_set_anim(5, anim_speed), retries)
            return

        for idx, (pos, color) in enumerate(stops[:32], start=1):
            self._queue_cmd(did, self._cmd_set_zone(idx, pos, color), retries)
        if clear_all:
            for idx in range(len(stops) + 1, 33):
                self._queue_cmd(did, self._cmd_set_zone(idx, 0, 0), retries)

        # Sector-follow mode is reused as smooth gradient transition mode.
        self._queue_cmd(did, self._cmd_set_anim(6, anim_speed), retries)

    def apply_base(self, did: int, clear_all: bool) -> None:
        self.set_playing(did, False, force=clear_all)

    def set_playing(self, did: int, playing: bool, force: bool = False) -> None:
        state = self._states.get(did)
        if state is None:
            return
        want_play = bool(playing)
        if (not force) and state.is_playing == want_play:
            return

        target_stops = self._stops_for_state(state, want_play)
        speed = state.cfg.play_speed if want_play else state.cfg.base_speed
        self._drop_pending_for_device(did)
        self._queue_apply_stops(
            did,
            state,
            target_stops,
            clear_all=True,
            set_state=bool(force),
            anim_speed=int(speed),
        )
        state.is_playing = want_play
        state.current_stops = list(target_stops)
        state.last_keepalive_s = time.monotonic()
        if self.debug:
            mode = "playing" if want_play else "idle"
            print(f"[dbg led] dev={did:02d} -> {mode} gradient speed={int(speed)}")

    def _service_timers(self) -> None:
        if not self._states:
            return
        now_s = time.monotonic()
        for did, state in self._states.items():
            cfg = state.cfg
            if cfg.keepalive_ms > 0:
                period_s = float(cfg.keepalive_ms) / 1000.0
                if period_s > 0.0 and (now_s - state.last_keepalive_s) >= period_s:
                    self.set_playing(did, state.is_playing, force=True)

    def _worker(self) -> None:
        while not self._stop.is_set():
            self._service_timers()
            try:
                did, payload, retries = self._q.get(timeout=0.01)
            except queue.Empty:
                continue
            state = self._states.get(did)
            spacing_s = 0.0 if state is None else float(max(0, state.cfg.command_spacing_ms)) / 1000.0
            tries = max(1, int(retries) + 1)
            for attempt in range(tries):
                try:
                    msg = can.Message(
                        arbitration_id=0x600 + int(did),
                        data=payload,
                        is_extended_id=False,
                    )
                    self.bus.send(msg, timeout=0.01)
                    break
                except Exception as exc:
                    if self.debug and attempt == (tries - 1):
                        print(f"[dbg led] send failed dev={did:02d} cmd=0x{payload[0]:02X} err={exc}")
                if spacing_s > 0.0:
                    time.sleep(spacing_s)


def apply_led_profile_verified(
    interface: str,
    channel: str,
    device_id: int,
    led: LedConfig,
    debug: bool = False,
) -> bool:
    retries = max(0, int(led.send_retries))
    delay_s = float(max(0, int(led.command_spacing_ms))) / 1000.0
    stops = list(led.gradient[:32])
    strip_len = led.strip_len if led.strip_len is not None else None

    for attempt in range(retries + 1):
        client = None
        try:
            client = AppCanClient(channel=channel, interface=interface, device_id=device_id, timeout=0.35)
            if strip_len is not None:
                try:
                    client.ws_set_length(int(strip_len))
                    if delay_s > 0.0:
                        time.sleep(delay_s)
                except Exception:
                    # Older firmware may not support strip-length command.
                    pass
            if stops:
                c0 = int(stops[0].color_rgb565)
                r5 = (c0 >> 11) & 0x1F
                g6 = (c0 >> 5) & 0x3F
                b5 = c0 & 0x1F
                r0 = (r5 * 255 + 15) // 31
                g0 = (g6 * 255 + 31) // 63
                b0 = (b5 * 255 + 15) // 31
                client.ws_set_all(True, int(led.brightness), int(r0), int(g0), int(b0))
            else:
                client.ws_set_all(True, int(led.brightness), 255, 255, 255)
            if delay_s > 0.0:
                time.sleep(delay_s)
            for idx, stop in enumerate(stops, start=1):
                client.ws_set_sector_zone(idx, int(stop.pos), int(stop.color_rgb565), readback=False)
                if delay_s > 0.0:
                    time.sleep(delay_s)
            for idx in range(len(stops) + 1, 33):
                client.ws_set_sector_zone(idx, 0, 0, readback=False)
                if delay_s > 0.0:
                    time.sleep(delay_s)
            client.ws_set_anim(6, int(led.base_speed))
            if debug:
                print(
                    f"[dbg led] verified apply ok dev={device_id:02d} len={strip_len} "
                    f"stops={len(stops)} mode=6/{int(led.base_speed)}"
                )
            return True
        except Exception as exc:
            if debug:
                print(f"[dbg led] verified apply failed dev={device_id:02d} attempt={attempt+1} err={exc}")
            if attempt >= retries:
                return False
            time.sleep(0.04)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
    return False


class FaustRuntime:
    def __init__(self, cfg: DeviceConfig, channel: int, debug: bool = False):
        self.cfg = cfg
        self.channel = int(max(0, min(15, channel)))
        self.debug = bool(debug)
        self.proc: subprocess.Popen[bytes] | None = None
        self._raw = None
        self._tx_q: queue.Queue[bytes] | None = None
        self._tx_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._port_sender: str | None = None
        self._port_arg: str | None = None

        inst = cfg.instrument
        if inst.faust_command:
            try:
                cmd0 = Path(inst.faust_command[0]).name.lower()
                if cmd0.startswith("faust2"):
                    self.proc = self._compile_and_start(inst.faust_command)
                else:
                    self.proc = self._start_with_device_fallback(list(inst.faust_command))
            except Exception as exc:
                raise RuntimeError(f"Failed to start Faust command for device {cfg.device_id}: {exc}") from exc

        if inst.faust_midi_device:
            try:
                self._raw = open(inst.faust_midi_device, "wb", buffering=0)
            except Exception as exc:
                raise RuntimeError(f"Failed to open MIDI device '{inst.faust_midi_device}': {exc}") from exc
        elif inst.faust_midi_port:
            port = str(inst.faust_midi_port).strip()
            if port.startswith("hw:") or port.startswith("plughw:"):
                if shutil.which("amidi") is None:
                    raise RuntimeError("Faust instrument requires 'amidi' for raw MIDI ports (hw:...)")
                self._port_sender = "amidi"
                self._port_arg = port
            else:
                if shutil.which("aseqsend") is None:
                    raise RuntimeError("Faust instrument requires 'aseqsend' for ALSA sequencer ports")
                self._port_sender = "aseqsend"
                self._port_arg = port
            self._tx_q = queue.Queue(maxsize=2048)
            self._tx_thread = threading.Thread(target=self._amidi_worker, daemon=True)
            self._tx_thread.start()
        else:
            raise RuntimeError(
                f"Faust instrument for device {cfg.device_id} needs faust_midi_device or faust_midi_port"
            )

    def _find_dsp_path(self, cmd: list[str]) -> Path:
        # Prefer explicit configured DSP path.
        if self.cfg.instrument.faust_dsp:
            p = Path(self.cfg.instrument.faust_dsp).expanduser()
            if p.exists():
                return p
        for tok in reversed(cmd):
            if str(tok).lower().endswith(".dsp"):
                p = Path(tok).expanduser()
                if not p.is_absolute():
                    p = (Path.cwd() / p).resolve()
                return p
        raise RuntimeError("Faust builder command needs a .dsp path")

    def _compile_and_start(self, cmd: list[str]) -> subprocess.Popen[bytes]:
        dsp = self._find_dsp_path(cmd)
        build_cwd = dsp.parent
        out_target = None if self.debug else subprocess.DEVNULL

        compile_cmd = []
        for tok in cmd:
            s = str(tok)
            if s.lower().endswith(".dsp"):
                compile_cmd.append(str(dsp))
            else:
                compile_cmd.append(s)
        rc = subprocess.run(
            compile_cmd,
            cwd=str(build_cwd),
            check=False,
            stdout=out_target,
            stderr=out_target,
        ).returncode
        if rc != 0:
            raise RuntimeError(
                f"Faust builder failed (rc={rc}) for DSP '{dsp}'. "
                "Try running the command manually to inspect missing dependencies."
            )

        exe = dsp.with_suffix("")
        if not exe.exists():
            # Some wrappers may emit to current working directory.
            alt = (Path.cwd() / exe.name)
            if alt.exists():
                exe = alt
            else:
                raise RuntimeError(f"Faust build succeeded but executable '{exe.name}' was not found")

        run_cmd = [str(exe)]
        return self._start_with_device_fallback(run_cmd, cwd=str(exe.parent))

    def _command_has_device_arg(self, cmd: list[str]) -> bool:
        return ("--device" in cmd) or ("-d" in cmd)

    def _audio_device_candidates(self) -> list[str]:
        out: list[str] = []
        configured = self.cfg.instrument.faust_audio_device
        env_val = os.environ.get("FAUST2ALSA_DEVICE", "").strip()

        # Probe ALSA card indices (card 0/1/2...) from local machine.
        card_idxs: list[str] = []
        try:
            ap = subprocess.run(
                ["aplay", "-l"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            for m in re.finditer(r"^card\s+(\d+):", ap.stdout or "", flags=re.MULTILINE):
                idx = m.group(1)
                if idx not in card_idxs:
                    card_idxs.append(idx)
        except Exception:
            pass

        base = [configured, env_val, "pipewire", "pulse", "default", "sysdefault"]
        # Common ALSA device names by discovered card index first.
        for idx in card_idxs:
            base.extend([f"plughw:{idx}", f"hw:{idx}", f"front:{idx}", f"surround51:{idx}"])
        # Keep fallback hardcoded indices too, in case aplay is unavailable.
        base.extend(["plughw:0", "hw:0", "plughw:1", "hw:1", "plughw:2", "hw:2"])

        for item in base:
            if not item:
                continue
            if item not in out:
                out.append(item)
        return out

    def _wait_start_and_poll(self, proc: subprocess.Popen[bytes]) -> int | None:
        wait_s = max(0.0, float(self.cfg.instrument.faust_startup_wait_ms) / 1000.0)
        if wait_s > 0.0:
            time.sleep(wait_s)
        return proc.poll()

    def _start_with_device_fallback(self, base_cmd: list[str], cwd: str | None = None) -> subprocess.Popen[bytes]:
        out_target = None if self.debug else subprocess.DEVNULL
        attempted: list[str] = []

        def _spawn(cmd: list[str]) -> subprocess.Popen[bytes]:
            return subprocess.Popen(cmd, cwd=cwd, stdout=out_target, stderr=out_target)

        if self._command_has_device_arg(base_cmd):
            proc = _spawn(base_cmd)
            rc = self._wait_start_and_poll(proc)
            if rc is not None:
                raise RuntimeError(
                    f"Faust command exited early with code {rc}. "
                    "Run the command manually to inspect its error output."
                )
            return proc

        for dev in self._audio_device_candidates():
            cmd = [*base_cmd, "--device", dev]
            attempted.append(dev)
            proc = _spawn(cmd)
            rc = self._wait_start_and_poll(proc)
            if rc is None:
                if self.debug:
                    print(f"[faust] started with --device {dev}")
                return proc
            try:
                proc.wait(timeout=0.2)
            except Exception:
                pass

        # Final try without explicit device.
        proc = _spawn(base_cmd)
        rc = self._wait_start_and_poll(proc)
        if rc is None:
            if self.debug:
                print("[faust] started without explicit --device")
            return proc
        tried = ", ".join(attempted) if attempted else "(none)"
        raise RuntimeError(
            f"Faust command exited early with code {rc}. "
            f"Tried audio devices: {tried}. "
            "Set instrument.audio_device in config to a valid ALSA device."
        )

    def _amidi_worker(self) -> None:
        assert self._tx_q is not None
        assert self._port_sender is not None and self._port_arg is not None
        while not self._stop.is_set():
            try:
                msg = self._tx_q.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                hexmsg = msg.hex().upper()
                if self._port_sender == "amidi":
                    cmd = ["amidi", "-p", self._port_arg, "-S", hexmsg]
                else:
                    cmd = ["aseqsend", "-p", self._port_arg, hexmsg]
                out_target = None if self.debug else subprocess.DEVNULL
                subprocess.run(cmd, check=False, stdout=out_target, stderr=out_target)
            except Exception:
                # Keep real-time loop resilient.
                pass

    def _send(self, msg: bytes) -> None:
        if self._raw is not None:
            try:
                self._raw.write(msg)
            except Exception:
                pass
            return
        if self._tx_q is None:
            return
        try:
            self._tx_q.put_nowait(msg)
        except queue.Full:
            try:
                self._tx_q.get_nowait()
                self._tx_q.put_nowait(msg)
            except (queue.Empty, queue.Full):
                pass

    def note_on(self, note: int, vel: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0x90 | ch, int(note) & 0x7F, int(vel) & 0x7F]))

    def note_off(self, note: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0x80 | ch, int(note) & 0x7F, 0]))

    def cc(self, cc_num: int, value: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0xB0 | ch, int(cc_num) & 0x7F, int(value) & 0x7F]))

    def pitch_bend(self, bend: int) -> None:
        ch = self.channel & 0x0F
        v = int(max(0, min(16383, int(bend) + 8192)))
        lsb = v & 0x7F
        msb = (v >> 7) & 0x7F
        self._send(bytes([0xE0 | ch, lsb, msb]))

    def channel_pressure(self, value: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0xD0 | ch, int(value) & 0x7F]))

    def set_program(self, bank: int, preset: int) -> None:
        ch = self.channel & 0x0F
        b = max(0, int(bank))
        self._send(bytes([0xB0 | ch, 0x00, (b >> 7) & 0x7F]))
        self._send(bytes([0xB0 | ch, 0x20, b & 0x7F]))
        self._send(bytes([0xC0 | ch, int(preset) & 0x7F]))

    def close(self) -> None:
        self._stop.set()
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=0.2)
        if self._raw is not None:
            try:
                self._raw.close()
            except Exception:
                pass
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=0.3)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass


class MidiRuntime:
    def __init__(self, cfg: DeviceConfig, channel: int, debug: bool = False):
        self.cfg = cfg
        self.channel = int(max(0, min(15, channel)))
        self.debug = bool(debug)
        self._raw = None
        self._tx_q: queue.Queue[bytes] | None = None
        self._tx_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._port_sender: str | None = None
        self._port_arg: str | None = None

        inst = cfg.instrument
        midi_device = inst.midi_device or inst.faust_midi_device
        midi_port = inst.midi_port or inst.faust_midi_port
        if midi_device:
            try:
                self._raw = open(midi_device, "wb", buffering=0)
            except Exception as exc:
                raise RuntimeError(f"Failed to open MIDI device '{midi_device}': {exc}") from exc
        elif midi_port:
            port = str(midi_port).strip()
            if port.startswith("hw:") or port.startswith("plughw:"):
                if shutil.which("amidi") is None:
                    raise RuntimeError("MIDI instrument requires 'amidi' for raw MIDI ports (hw:...)")
                self._port_sender = "amidi"
                self._port_arg = port
            else:
                if shutil.which("aseqsend") is None:
                    raise RuntimeError("MIDI instrument requires 'aseqsend' for ALSA sequencer ports")
                self._port_sender = "aseqsend"
                self._port_arg = port
            self._tx_q = queue.Queue(maxsize=2048)
            self._tx_thread = threading.Thread(target=self._amidi_worker, daemon=True)
            self._tx_thread.start()
        else:
            raise RuntimeError(
                f"MIDI instrument for device {cfg.device_id} needs instrument.midi_device or instrument.midi_port"
            )

    def _amidi_worker(self) -> None:
        assert self._tx_q is not None
        assert self._port_sender is not None and self._port_arg is not None
        while not self._stop.is_set():
            try:
                msg = self._tx_q.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                hexmsg = msg.hex().upper()
                if self._port_sender == "amidi":
                    cmd = ["amidi", "-p", self._port_arg, "-S", hexmsg]
                else:
                    cmd = ["aseqsend", "-p", self._port_arg, hexmsg]
                out_target = None if self.debug else subprocess.DEVNULL
                subprocess.run(cmd, check=False, stdout=out_target, stderr=out_target)
            except Exception:
                pass

    def _send(self, msg: bytes) -> None:
        if self._raw is not None:
            try:
                self._raw.write(msg)
            except Exception:
                pass
            return
        if self._tx_q is None:
            return
        try:
            self._tx_q.put_nowait(msg)
        except queue.Full:
            try:
                self._tx_q.get_nowait()
                self._tx_q.put_nowait(msg)
            except (queue.Empty, queue.Full):
                pass

    def note_on(self, note: int, vel: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0x90 | ch, int(note) & 0x7F, int(vel) & 0x7F]))

    def note_off(self, note: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0x80 | ch, int(note) & 0x7F, 0]))

    def cc(self, cc_num: int, value: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0xB0 | ch, int(cc_num) & 0x7F, int(value) & 0x7F]))

    def pitch_bend(self, bend: int) -> None:
        ch = self.channel & 0x0F
        v = int(max(0, min(16383, int(bend) + 8192)))
        lsb = v & 0x7F
        msb = (v >> 7) & 0x7F
        self._send(bytes([0xE0 | ch, lsb, msb]))

    def channel_pressure(self, value: int) -> None:
        ch = self.channel & 0x0F
        self._send(bytes([0xD0 | ch, int(value) & 0x7F]))

    def set_program(self, bank: int, preset: int) -> None:
        ch = self.channel & 0x0F
        b = max(0, int(bank))
        self._send(bytes([0xB0 | ch, 0x00, (b >> 7) & 0x7F]))
        self._send(bytes([0xB0 | ch, 0x20, b & 0x7F]))
        self._send(bytes([0xC0 | ch, int(preset) & 0x7F]))

    def close(self) -> None:
        self._stop.set()
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=0.2)
        if self._raw is not None:
            try:
                self._raw.close()
            except Exception:
                pass


class Mixer:
    def __init__(
        self,
        driver: str,
        debug: bool = False,
        note_duration_ms: int = 0,
        fadein_ms: int | None = None,
        fadeout_ms: int = 220,
    ):
        self.debug = bool(debug)
        self.default_note_duration_s = max(0.0, float(note_duration_ms) / 1000.0)
        self.default_fadein_ms = None if fadein_ms is None else max(0, int(fadein_ms))
        self.default_fadeout_ms = max(0, int(fadeout_ms))
        self._driver = str(driver)
        self.fs = None
        self._sfid_cache: dict[str, int] = {}
        self._voices: dict[int, DeviceVoice] = {}
        self._faust_by_device: dict[int, FaustRuntime] = {}
        self._midi_by_device: dict[int, MidiRuntime] = {}

    def _effective_note_duration_s(self, voice: DeviceVoice) -> float:
        if voice.cfg.note_duration_ms is None:
            return self.default_note_duration_s
        return max(0.0, float(voice.cfg.note_duration_ms) / 1000.0)

    def _effective_fadeout_ms(self, voice: DeviceVoice) -> int:
        if voice.cfg.fadeout_ms is None:
            return self.default_fadeout_ms
        return max(0, int(voice.cfg.fadeout_ms))

    def _effective_fadein_ms(self, voice: DeviceVoice) -> int | None:
        if voice.cfg.fadein_ms is not None:
            return max(0, int(voice.cfg.fadein_ms))
        return self.default_fadein_ms

    def _ensure_fs_started(self):
        if self.fs is None:
            self.fs = self._start_synth(self._driver)
        return self.fs

    def _ensure_faust_runtime(self, voice: DeviceVoice) -> FaustRuntime:
        rt = self._faust_by_device.get(voice.cfg.device_id)
        if rt is not None:
            return rt
        rt = FaustRuntime(voice.cfg, voice.channel, debug=self.debug)
        self._faust_by_device[voice.cfg.device_id] = rt
        return rt

    def _ensure_midi_runtime(self, voice: DeviceVoice) -> MidiRuntime:
        rt = self._midi_by_device.get(voice.cfg.device_id)
        if rt is not None:
            return rt
        rt = MidiRuntime(voice.cfg, voice.channel, debug=self.debug)
        self._midi_by_device[voice.cfg.device_id] = rt
        return rt

    def _apply_channel_controls(self, voice: DeviceVoice) -> None:
        att_ms = self._effective_fadein_ms(voice)
        rel_cc = max(0, min(127, int((self._effective_fadeout_ms(voice) / 3000.0) * 127.0)))
        cc_values: dict[int, int] = {7: 127}
        cc_values.update(voice.cfg.midi_cc)
        # fadein_ms controls attack time when specified.
        if att_ms is not None:
            cc_values[73] = max(0, min(127, int((att_ms / 3000.0) * 127.0)))
        # Keep duration/fade semantics deterministic:
        # fadeout_ms is the release/decay time control.
        cc_values[72] = rel_cc
        if voice.cfg.instrument.type == "midi":
            rt = self._ensure_midi_runtime(voice)
            for cc_num in sorted(cc_values.keys()):
                rt.cc(int(cc_num), int(max(0, min(127, cc_values[cc_num]))))
            if voice.cfg.channel_pressure is not None:
                rt.channel_pressure(int(max(0, min(127, voice.cfg.channel_pressure))))
            if voice.cfg.pitch_bend is not None:
                rt.pitch_bend(int(max(-8192, min(8191, voice.cfg.pitch_bend))))
            return
        if voice.cfg.instrument.type == "faust":
            rt = self._ensure_faust_runtime(voice)
            for cc_num in sorted(cc_values.keys()):
                rt.cc(int(cc_num), int(max(0, min(127, cc_values[cc_num]))))
            if voice.cfg.channel_pressure is not None:
                rt.channel_pressure(int(max(0, min(127, voice.cfg.channel_pressure))))
            if voice.cfg.pitch_bend is not None:
                rt.pitch_bend(int(max(-8192, min(8191, voice.cfg.pitch_bend))))
            return

        fs = self._ensure_fs_started()
        for cc_num in sorted(cc_values.keys()):
            fs.cc(voice.channel, int(cc_num), int(max(0, min(127, cc_values[cc_num]))))

        if voice.cfg.channel_pressure is not None:
            fn = getattr(fs, "channel_pressure", None)
            if callable(fn):
                fn(voice.channel, int(max(0, min(127, voice.cfg.channel_pressure))))

        if voice.cfg.pitch_bend is not None:
            fn = getattr(fs, "pitch_bend", None)
            if callable(fn):
                pb14 = int(max(0, min(16383, int(voice.cfg.pitch_bend) + 8192)))
                fn(voice.channel, pb14)

    def _start_synth(self, driver: str):
        if fluidsynth is None:
            raise RuntimeError("Missing python module 'fluidsynth' (install pyfluidsynth)")
        ctor = getattr(fluidsynth, "Synth", None)
        if ctor is None:
            nested = getattr(fluidsynth, "fluidsynth", None)
            ctor = getattr(nested, "Synth", None) if nested is not None else None
        if ctor is None:
            raise RuntimeError("Incompatible fluidsynth module: no Synth class")

        candidates = [driver] if driver != "auto" else ["pulseaudio", "pipewire", "alsa", "jack", "oss", "sdl3"]
        last_error = None
        for drv in candidates:
            fs = ctor()
            try:
                if hasattr(fs, "setting"):
                    try:
                        fs.setting("audio.period-size", 64)
                        fs.setting("audio.periods", 2)
                    except Exception:
                        pass
                fs.start(driver=drv)
                if getattr(fs, "audio_driver", None):
                    return fs
                try:
                    fs.delete()
                except Exception:
                    pass
            except Exception as exc:
                last_error = exc
                try:
                    fs.delete()
                except Exception:
                    pass
        raise RuntimeError(f"Failed to start FluidSynth audio backend (last_error={last_error})")

    def _get_sfid(self, path: str) -> int:
        sfid = self._sfid_cache.get(path)
        if sfid is not None:
            return sfid
        fs = self._ensure_fs_started()
        sfid = fs.sfload(path)
        if sfid < 0:
            raise RuntimeError(f"sfload failed: {path}")
        self._sfid_cache[path] = sfid
        return sfid

    def _note_on(self, voice: DeviceVoice, note: int, velocity: int) -> None:
        if voice.cfg.instrument.type == "midi":
            self._ensure_midi_runtime(voice).note_on(note, velocity)
        elif voice.cfg.instrument.type == "faust":
            self._ensure_faust_runtime(voice).note_on(note, velocity)
        else:
            self._ensure_fs_started().noteon(voice.channel, note, velocity)

    def _note_off(self, voice: DeviceVoice, note: int) -> None:
        if voice.cfg.instrument.type == "midi":
            self._ensure_midi_runtime(voice).note_off(note)
        elif voice.cfg.instrument.type == "faust":
            self._ensure_faust_runtime(voice).note_off(note)
        else:
            self._ensure_fs_started().noteoff(voice.channel, note)

    def register_device(self, cfg: DeviceConfig, channel: int) -> None:
        self._voices[cfg.device_id] = DeviceVoice(cfg=cfg, channel=channel)
        voice = self._voices[cfg.device_id]
        if cfg.instrument.type == "midi":
            self._ensure_midi_runtime(voice)
        elif cfg.instrument.type == "faust":
            self._ensure_faust_runtime(voice)
        self._apply_channel_controls(self._voices[cfg.device_id])

    def _select_program(self, voice: DeviceVoice) -> None:
        inst = voice.cfg.instrument
        if inst.type == "midi":
            key = (int(inst.bank), int(inst.preset), 2)
            if voice.program_key != key:
                rt = self._ensure_midi_runtime(voice)
                rt.set_program(int(inst.bank), int(inst.preset))
                self._apply_channel_controls(voice)
                voice.program_key = key
            return
        if inst.type == "faust":
            key = (int(inst.bank), int(inst.preset), 0)
            if voice.program_key != key:
                rt = self._ensure_faust_runtime(voice)
                rt.set_program(int(inst.bank), int(inst.preset))
                self._apply_channel_controls(voice)
                voice.program_key = key
            return

        sfid = self._get_sfid(inst.soundfont)
        key = (sfid, int(inst.bank), int(inst.preset))
        if voice.program_key == key:
            return
        self._ensure_fs_started().program_select(voice.channel, sfid, int(inst.bank), int(inst.preset))
        self._apply_channel_controls(voice)
        voice.program_key = key

    def play_chord(
        self,
        device_id: int,
        notes: set[int],
        force_retrigger: bool = False,
        fade_out_existing: bool = False,
    ) -> tuple[list[int], list[int], list[int]]:
        voice = self._voices.get(device_id)
        if voice is None:
            return [], [], []
        self._select_program(voice)

        faded_on_change: list[int] = []
        if fade_out_existing and voice.note_on_counts:
            for n in sorted(voice.note_on_counts.keys()):
                cnt = int(voice.note_on_counts.get(n, 0))
                if cnt <= 0:
                    continue
                for _ in range(cnt):
                    self._note_off(voice, n)
                faded_on_change.append(n)
            voice.held_notes.clear()
            voice.note_on_counts.clear()
            voice.note_deadlines.clear()

        cleaned = {int(n) for n in notes if 0 <= int(n) <= 127}
        # Additive polyphony: new notes are layered on top of currently held ones.
        target_held = set(voice.held_notes) | cleaned
        to_stop: list[int] = []
        to_start = sorted(target_held - voice.held_notes)
        retrigger_candidates = sorted(cleaned & voice.held_notes)
        # Re-entering an already held note after sector change should retrigger it
        # (e.g. 1->2->1), even while older voices are still decaying.
        auto_retrigger = bool(retrigger_candidates) and not bool(to_start)
        to_retrigger = retrigger_candidates if (force_retrigger or auto_retrigger) else []

        # Start newly pressed notes.
        now_s = time.monotonic()
        note_duration_s = self._effective_note_duration_s(voice)
        for n in to_start:
            self._note_on(voice, n, int(max(0, min(127, voice.cfg.velocity))))
            voice.note_on_counts[n] = int(voice.note_on_counts.get(n, 0)) + 1
            if note_duration_s > 0.0:
                voice.note_deadlines.setdefault(n, deque()).append(now_s + note_duration_s)
            if not self.debug:
                print(f"dev={device_id:02d} note={n}")

        # Optional retrigger stacks another voice without cutting the current one.
        retrig_released: list[int] = []
        for n in to_retrigger:
            active_before = int(voice.note_on_counts.get(n, 0))
            if active_before > 0:
                # Release one currently playing instance so it decays, then
                # start a fresh one right away (crossfade on same note).
                self._note_off(voice, n)
                retrig_released.append(n)
                voice.note_on_counts[n] = active_before - 1
                if note_duration_s > 0.0:
                    dq = voice.note_deadlines.get(n)
                    if dq:
                        dq.popleft()
                        if not dq:
                            voice.note_deadlines.pop(n, None)
            self._note_on(voice, n, int(max(0, min(127, voice.cfg.velocity))))
            voice.note_on_counts[n] = int(voice.note_on_counts.get(n, 0)) + 1
            if note_duration_s > 0.0:
                voice.note_deadlines.setdefault(n, deque()).append(now_s + note_duration_s)
            if not self.debug:
                print(f"dev={device_id:02d} note={n}")

        voice.held_notes = target_held
        started = to_start + to_retrigger
        stopped = to_stop + retrig_released + faded_on_change
        return started, stopped, sorted(voice.held_notes)

    def stop_device(self, device_id: int) -> list[int]:
        voice = self._voices.get(device_id)
        if voice is None:
            return []
        stopped = sorted(voice.note_on_counts.keys())
        for n in sorted(voice.note_on_counts.keys()):
            for _ in range(int(voice.note_on_counts.get(n, 0))):
                self._note_off(voice, n)
        voice.held_notes.clear()
        voice.note_on_counts.clear()
        voice.note_deadlines.clear()
        return stopped

    def any_active_notes(self) -> bool:
        return any(v.note_on_counts for v in self._voices.values())

    def device_has_active_notes(self, device_id: int) -> bool:
        voice = self._voices.get(device_id)
        return bool(voice and voice.note_on_counts)

    def process_timeouts(self, now_s: float | None = None) -> None:
        if now_s is None:
            now_s = time.monotonic()
        for voice in self._voices.values():
            expired_notes: list[int] = []
            for n, deadlines in list(voice.note_deadlines.items()):
                expired = 0
                while deadlines and deadlines[0] <= now_s:
                    deadlines.popleft()
                    expired += 1
                if expired <= 0:
                    continue
                active = int(voice.note_on_counts.get(n, 0))
                off_cnt = min(active, expired)
                for _ in range(off_cnt):
                    self._note_off(voice, n)
                remaining = active - off_cnt
                if remaining > 0:
                    voice.note_on_counts[n] = remaining
                else:
                    voice.note_on_counts.pop(n, None)
                    voice.held_notes.discard(n)
                if not deadlines:
                    expired_notes.append(n)
            for n in expired_notes:
                voice.note_deadlines.pop(n, None)

    def close(self) -> None:
        for did in list(self._voices.keys()):
            self.stop_device(did)
        for did in list(self._midi_by_device.keys()):
            try:
                self._midi_by_device[did].close()
            except Exception:
                pass
        self._midi_by_device.clear()
        for did in list(self._faust_by_device.keys()):
            try:
                self._faust_by_device[did].close()
            except Exception:
                pass
        self._faust_by_device.clear()
        self._voices.clear()
        if self.fs is not None:
            self.fs.delete()
            self.fs = None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Beat-quantized minimal CAN rhythm player")
    p.add_argument("--channel", default=None)
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--driver", default="auto")
    p.add_argument("--bpm", type=float, default=None)
    p.add_argument("--idle-reset-s", type=float, default=None)
    p.add_argument("--note-duration-ms", type=int, default=450)
    p.add_argument("--fadein-ms", type=int, default=None)
    p.add_argument("--fadeout-ms", type=int, default=220)
    p.add_argument("--ignore-sector-zero", action=argparse.BooleanOptionalAction, default=None)
    p.add_argument("--beat-quantize", action=argparse.BooleanOptionalAction, default=None)
    p.add_argument("--debug", action="store_true")
    p.add_argument("--config", default="rhytmisc_conf.json")
    p.add_argument("--default-config", default="rhytmics_conf_default.json")
    return p.parse_args()


def sector_to_note(cfg: DeviceConfig, sector: int) -> int:
    base = int(cfg.note_map[(int(sector) - 1) % len(cfg.note_map)])
    return int(max(0, min(127, base + int(cfg.transpose))))


def instrument_label(cfg: DeviceConfig) -> str:
    inst = cfg.instrument
    if inst.type == "faust":
        if inst.faust_dsp:
            return Path(inst.faust_dsp).name
        if inst.faust_command:
            return Path(inst.faust_command[0]).name
        return "faust"
    if inst.type == "midi":
        if inst.midi_device:
            return f"midi:{Path(inst.midi_device).name}"
        if inst.midi_port:
            return f"midi:{inst.midi_port}"
        return "midi"
    if inst.soundfont:
        return Path(inst.soundfont).name
    return "soundfont"


def run() -> int:
    args = parse_args()
    default_cfg = resolve_local(args.default_config)
    user_cfg = resolve_local(args.config)
    global_cfg = load_global_player_config(default_cfg, user_cfg)
    devices = load_device_configs(default_cfg, user_cfg)
    if not devices:
        print("No valid devices in config")
        return 1
    if len(devices) > 16:
        print("Too many devices (max 16)")
        return 1

    can_channel = str(args.channel) if args.channel is not None else (global_cfg.channel if global_cfg.channel else "slcan0")
    bpm = float(args.bpm) if args.bpm is not None else (global_cfg.bpm if global_cfg.bpm is not None else 120.0)
    idle_reset_s = (
        float(args.idle_reset_s)
        if args.idle_reset_s is not None
        else (global_cfg.idle_reset_s if global_cfg.idle_reset_s is not None else 2.0)
    )
    ignore_sector_zero = (
        bool(args.ignore_sector_zero)
        if args.ignore_sector_zero is not None
        else (global_cfg.ignore_sector_zero if global_cfg.ignore_sector_zero is not None else False)
    )
    beat_quantize = (
        bool(args.beat_quantize)
        if args.beat_quantize is not None
        else (global_cfg.beat_quantize if global_cfg.beat_quantize is not None else True)
    )

    mixer = Mixer(
        args.driver,
        debug=args.debug,
        note_duration_ms=args.note_duration_ms,
        fadein_ms=args.fadein_ms,
        fadeout_ms=args.fadeout_ms,
    )
    bus: can.BusABC | None = None
    notifier: can.Notifier | None = None
    led_controller: LedCanController | None = None
    try:
        channels = list(range(16))
        for idx, did in enumerate(sorted(devices.keys())):
            mixer.register_device(devices[did], channels[idx])

        bus = can.Bus(
            interface=args.interface,
            channel=can_channel,
            receive_own_messages=False,
            can_filters=[{"can_id": 0x580, "can_mask": 0x780, "extended": False}],
        )
        q: queue.Queue[tuple[int, int]] = queue.Queue(maxsize=4096)
        notifier = can.Notifier(bus, [CanQueueListener(q)], timeout=0.001)
        led_profiles = {
            did: cfg.led
            for did, cfg in devices.items()
            if cfg.led is not None and cfg.led.enabled
        }
        led_simple_devices: set[int] = set()
        for did, led_cfg in led_profiles.items():
            ok = apply_led_profile_verified(
                interface=args.interface,
                channel=can_channel,
                device_id=did,
                led=led_cfg,
                debug=args.debug,
            )
            if not ok:
                led_simple_devices.add(int(did))
                print(f"[warn] LED verified setup failed for device {did}; falling back to async LED sender")
        try:
            led_controller = LedCanController(bus, devices, debug=args.debug, simple_devices=led_simple_devices)
            led_controller.start()
            if args.debug and led_controller.enabled():
                print("[dbg led] controller enabled")
        except Exception as exc:
            led_controller = None
            print(f"[warn] LED control disabled: {exc}")

        stop_event = threading.Event()

        def _sig_handler(_sig, _frame):
            stop_event.set()

        signal.signal(signal.SIGINT, _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)

        beat_period_s = 60.0 / max(1e-6, bpm)
        idle_reset_s = max(0.0, float(idle_reset_s))
        pending_notes: dict[int, set[int]] = {did: set() for did in devices.keys()}
        pending_clear: dict[int, bool] = {did: False for did in devices.keys()}
        pending_fade_on_change: dict[int, bool] = {did: False for did in devices.keys()}
        zero_rearm: dict[int, bool] = {did: False for did in devices.keys()}
        pending_retrigger: dict[int, bool] = {did: False for did in devices.keys()}
        last_sector_seen: dict[int, int | None] = {did: None for did in devices.keys()}
        last_input_s = 0.0
        beat_running = False
        next_beat_s = 0.0
        beat_idx = 0
        beat_window_start_s = 0.0

        def queue_sector(device_id: int, sector: int, now_s: float) -> None:
            nonlocal beat_running, next_beat_s, last_input_s, beat_window_start_s
            cfg = devices.get(device_id)
            if cfg is None:
                return
            prev_sector = last_sector_seen[device_id]
            sector_changed = (prev_sector != sector)
            if not sector_changed:
                return
            if sector <= 0:
                if ignore_sector_zero:
                    # Zero only rearms retrigger of the next non-zero sector.
                    pending_notes[device_id].clear()
                    pending_clear[device_id] = False
                    zero_rearm[device_id] = True
                    if args.debug and sector_changed and prev_sector not in (None, 0):
                        print(f"[dbg collect] dev={device_id:02d} sector={prev_sector} -> 0 (rearm)")
                else:
                    had_notes = bool(pending_notes[device_id])
                    was_clear = pending_clear[device_id]
                    pending_notes[device_id].clear()
                    pending_clear[device_id] = True
                    if args.debug and sector_changed and (had_notes or not was_clear or prev_sector not in (None, 0)):
                        print(f"[dbg collect] dev={device_id:02d} sector={prev_sector} -> 0 (clear)")
            else:
                note = sector_to_note(cfg, sector)
                was_clear = pending_clear[device_id]
                was_added = note not in pending_notes[device_id]
                pending_notes[device_id].add(note)
                pending_clear[device_id] = False
                if (
                    bool(cfg.instrument.fade_out_on_sector_change)
                    and prev_sector not in (None, 0)
                    and int(sector) > 0
                    and int(sector) != int(prev_sector)
                ):
                    pending_fade_on_change[device_id] = True
                if ignore_sector_zero and zero_rearm[device_id]:
                    pending_retrigger[device_id] = True
                    zero_rearm[device_id] = False
                if args.debug and sector_changed and (was_added or was_clear or prev_sector is None):
                    sf = instrument_label(cfg)
                    print(
                        f"[dbg collect] dev={device_id:02d} sf={sf} "
                        f"sector={prev_sector} -> {sector} add_note={note} pending={sorted(pending_notes[device_id])}"
                    )
            last_sector_seen[device_id] = sector
            last_input_s = now_s
            if not beat_running:
                beat_running = True
                beat_window_start_s = now_s
                next_beat_s = now_s + beat_period_s

        def play_sector_immediate(device_id: int, sector: int, now_s: float) -> None:
            nonlocal last_input_s
            cfg = devices.get(device_id)
            if cfg is None:
                return
            prev_sector = last_sector_seen[device_id]
            if prev_sector == sector:
                return

            started: list[int] = []
            stopped: list[int] = []
            active: list[int] = []
            if sector <= 0:
                if ignore_sector_zero:
                    zero_rearm[device_id] = True
                    if args.debug and prev_sector not in (None, 0):
                        print(f"[dbg collect] dev={device_id:02d} sector={prev_sector} -> 0 (rearm)")
                else:
                    stopped = mixer.stop_device(device_id)
                    if args.debug and prev_sector not in (None, 0):
                        print(f"[dbg collect] dev={device_id:02d} sector={prev_sector} -> 0 (clear)")
            else:
                note = sector_to_note(cfg, sector)
                retrig = bool(ignore_sector_zero and zero_rearm[device_id])
                fade_on_change = bool(
                    cfg.instrument.fade_out_on_sector_change
                    and prev_sector not in (None, 0)
                    and int(sector) > 0
                    and int(sector) != int(prev_sector)
                )
                if retrig:
                    zero_rearm[device_id] = False
                started, stopped, active = mixer.play_chord(
                    device_id,
                    {note},
                    force_retrigger=retrig,
                    fade_out_existing=fade_on_change,
                )
                if args.debug and (started or stopped):
                    sf = instrument_label(cfg)
                    print(
                        f"[dbg play dev] dev={device_id:02d} sf={sf} "
                        f"sector={prev_sector}->{sector} start={started} stop={stopped} active={active}"
                    )

            last_sector_seen[device_id] = sector
            last_input_s = now_s
            if led_controller is not None:
                led_controller.set_playing(device_id, mixer.device_has_active_notes(device_id))

        def apply_beat(now_s: float) -> None:
            nonlocal beat_running, next_beat_s, beat_idx, beat_window_start_s
            if not beat_running or now_s < next_beat_s:
                return

            while beat_running and now_s >= next_beat_s:
                beat_idx += 1
                beat_ts = next_beat_s
                beat_debug_rows: list[str] = []

                for did in sorted(devices.keys()):
                    cfg = devices[did]
                    notes = sorted(pending_notes[did])
                    started: list[int] = []
                    stopped: list[int] = []
                    active: list[int] = []
                    if pending_clear[did] and not notes:
                        stopped = mixer.stop_device(did)
                    elif notes:
                        started, stopped, active = mixer.play_chord(
                            did,
                            set(notes),
                            force_retrigger=pending_retrigger[did],
                            fade_out_existing=pending_fade_on_change[did],
                        )
                        pending_retrigger[did] = False

                    if args.debug and (started or stopped):
                        sf = instrument_label(cfg)
                        beat_debug_rows.append(
                            f"[dbg beat dev] dev={did:02d} sf={sf} "
                            f"registered={notes} start={started} stop={stopped} active={active}"
                        )

                    pending_notes[did].clear()
                    pending_clear[did] = False
                    pending_fade_on_change[did] = False
                    if led_controller is not None:
                        led_controller.set_playing(did, mixer.device_has_active_notes(did))

                if args.debug and beat_debug_rows:
                    print(
                        f"[dbg beat] idx={beat_idx} "
                        f"window={beat_window_start_s:.3f}->{beat_ts:.3f} period={beat_period_s:.3f}s"
                    )
                    for row in beat_debug_rows:
                        print(row)

                beat_window_start_s = beat_ts
                next_beat_s += beat_period_s

                if (not mixer.any_active_notes()) and last_input_s > 0.0 and (now_s - last_input_s) >= idle_reset_s:
                    beat_running = False
                    next_beat_s = 0.0
                    break

        def process_frame(did: int, sector: int) -> None:
            cfg = devices.get(did)
            if cfg is None or cfg.event_source != "hardware":
                return

            now_s = time.monotonic()
            if beat_quantize and not cfg.exclude_from_beat_quantize:
                queue_sector(did, sector, now_s)
            else:
                play_sector_immediate(did, sector, now_s)

        while not stop_event.is_set():
            now = time.monotonic()
            mixer.process_timeouts(now)
            if led_controller is not None:
                for did in devices.keys():
                    led_controller.set_playing(int(did), mixer.device_has_active_notes(int(did)))
            if beat_quantize:
                apply_beat(now)

            try:
                first_did, first_sector = q.get(timeout=0.001)
            except queue.Empty:
                continue

            process_frame(first_did, first_sector)

            # Drain queue in bursts to reduce latency/backlog under high event rate.
            for _ in range(1023):
                try:
                    did, sector = q.get_nowait()
                except queue.Empty:
                    break
                process_frame(did, sector)

            now = time.monotonic()
            mixer.process_timeouts(now)
            if led_controller is not None:
                for did in devices.keys():
                    led_controller.set_playing(int(did), mixer.device_has_active_notes(int(did)))
            if beat_quantize:
                apply_beat(now)

    finally:
        if led_controller is not None:
            try:
                led_controller.close()
            except Exception:
                pass
        if notifier is not None:
            try:
                notifier.stop()
            except Exception:
                pass
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        mixer.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(run())
