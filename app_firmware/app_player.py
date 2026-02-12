#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import queue
import signal
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import can
try:
    import fluidsynth
except ModuleNotFoundError:
    fluidsynth = None

from app_can_tool import (
    AppCanClient,
    FRAME_EVENT,
    FRAME_MAG,
    EVENT_NAMES,
    discover_devices,
    status_id_to_device_id,
)
from event_detection import (
    Event,
    EventDetectionConfig,
    EventDetector,
    EVENT_ERROR_NO_DATA,
    EVENT_INTENSITY_CHANGE,
    EVENT_PASSING_SECTOR_CHANGE,
    EVENT_POSSIBLE_MECHANICAL_FAILURE,
    EVENT_SECTION_DEACTIVATED,
    EVENT_SECTOR_ACTIVATED,
    EVENT_SECTOR_CHANGED,
    EVENT_SESSION_ENDED,
)

@dataclass
class InstrumentProfile:
    soundfont: str
    bank: int = 0
    preset: int = 0


@dataclass
class DeviceProfile:
    device_id: int
    event_source: str = "hardware"  # hardware | software
    note_map: list[int] = field(default_factory=lambda: [60, 61, 63, 65, 66, 68])
    gain: float = 1.0
    crossfade_ms: int = 90
    release_ms: int = 450
    intensity_full_scale: int = 100
    min_level: float = 0.18
    max_level: float = 1.0
    instrument: InstrumentProfile = field(default_factory=lambda: InstrumentProfile(soundfont=""))


@dataclass
class DeviceRuntime:
    profile: DeviceProfile
    detector: EventDetector | None = None
    last_mag_s: float = 0.0
    last_rx_s: float = 0.0
    last_event_s: float = 0.0
    frames_rx: int = 0
    events_rx: int = 0
    hw_event_frames_rx: int = 0
    hw_event_frames_ignored: int = 0
    local_events_rx: int = 0
    note_on_actions: int = 0
    note_off_actions: int = 0
    level_actions: int = 0
    dropped_frames: int = 0
    current_sector: int = 0
    current_level: float = 0.0


@dataclass
class VoiceSlot:
    channel: int
    note: int | None = None
    gain: float = 0.0
    target_gain: float = 0.0
    fade_ms: int = 100
    cc_last: int = -1
    program_key: tuple[int, int, int] | None = None


@dataclass
class DeviceVoice:
    device_id: int
    slots: list[VoiceSlot]
    active_slot: int = 0
    device_gain: float = 1.0
    instrument: InstrumentProfile = field(default_factory=lambda: InstrumentProfile(soundfont=""))


class SynthMixer:
    def __init__(self, soundfont: str, driver: str, tick_ms: float = 5.0, debug: bool = False):
        self.debug = bool(debug)
        self.fs = None
        self.audio_driver_name = None
        self.audio_start_attempts: list[str] = []
        self.fs = self._create_started_synth(driver)
        self._sfid_cache: dict[str, int] = {}
        # Preload fallback instrument for fast first note.
        self._get_sfid(soundfont)
        self.tick_s = max(0.001, tick_ms / 1000.0)

        self._lock = threading.Lock()
        self._voices: dict[int, DeviceVoice] = {}
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _dbg(self, msg: str) -> None:
        if self.debug:
            ts = time.strftime("%H:%M:%S")
            print(f"[audio {ts}] {msg}")

    def _create_started_synth(self, driver: str):
        if driver == "auto":
            candidates = ["pulseaudio", "pipewire", "alsa", "jack", "oss", "sdl3"]
        else:
            candidates = [driver]

        last_error = None
        for drv in candidates:
            fs = create_fluidsynth_synth()
            try:
                fs.start(driver=drv)
                audio_driver = getattr(fs, "audio_driver", None)
                if audio_driver:
                    self.audio_driver_name = drv
                    self.audio_start_attempts.append(f"{drv}:ok")
                    return fs
                self.audio_start_attempts.append(f"{drv}:no-audio-handle")
                try:
                    fs.delete()
                except Exception:
                    pass
            except Exception as exc:
                self.audio_start_attempts.append(f"{drv}:exc:{exc}")
                last_error = exc
                try:
                    fs.delete()
                except Exception:
                    pass

        msg = "Failed to open FluidSynth audio driver. Attempts: " + ", ".join(self.audio_start_attempts)
        if last_error is not None:
            msg += f" | last_error={last_error}"
        raise RuntimeError(msg)

    @staticmethod
    def _clampf(v: float, lo: float, hi: float) -> float:
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def _get_sfid(self, soundfont: str) -> int:
        sfid = self._sfid_cache.get(soundfont)
        if sfid is not None:
            return sfid
        sfid = self.fs.sfload(soundfont)
        if sfid < 0:
            raise RuntimeError(f"sfload failed for soundfont: {soundfont}")
        self._dbg(f"sfload ok sfid={sfid} path={soundfont}")
        self._sfid_cache[soundfont] = sfid
        return sfid

    def _select_program(self, slot: VoiceSlot, sfid: int, bank: int, preset: int) -> None:
        key = (sfid, int(bank), int(preset))
        if slot.program_key == key:
            return
        rc = self.fs.program_select(slot.channel, sfid, int(bank), int(preset))
        if rc != 0:
            self._dbg(
                f"program_select failed rc={rc} ch={slot.channel} sfid={sfid} "
                f"bank={bank} preset={preset}"
            )
        else:
            self._dbg(
                f"program_select ok ch={slot.channel} sfid={sfid} "
                f"bank={bank} preset={preset}"
            )
        slot.program_key = key

    def register_device(self, device_id: int, chan_a: int, chan_b: int, gain: float,
                        instrument: InstrumentProfile) -> None:
        with self._lock:
            slots = [VoiceSlot(chan_a), VoiceSlot(chan_b)]
            for slot in slots:
                self.fs.cc(slot.channel, 7, 0)
            self._voices[device_id] = DeviceVoice(
                device_id=device_id,
                slots=slots,
                device_gain=self._clampf(gain, 0.0, 2.0),
                instrument=instrument,
            )

    def _play_on_slot(self, slot: VoiceSlot, inst: InstrumentProfile, midi_note: int, vel: int) -> None:
        sfid = self._get_sfid(inst.soundfont)
        self._select_program(slot, sfid, inst.bank, inst.preset)
        if slot.note is not None and slot.note != midi_note:
            rc_off = self.fs.noteoff(slot.channel, slot.note)
            self._dbg(f"noteoff ch={slot.channel} note={slot.note} rc={rc_off}")
            slot.note = None
        if slot.note is None:
            rc_on = self.fs.noteon(slot.channel, midi_note, vel)
            self._dbg(f"noteon ch={slot.channel} note={midi_note} vel={vel} rc={rc_on}")
            slot.note = midi_note

    def play_note(self, device_id: int, midi_note: int, level: float, fade_ms: int) -> None:
        with self._lock:
            voice = self._voices.get(device_id)
            if voice is None:
                return
            inst = voice.instrument
            level = self._clampf(level, 0.0, 1.0)
            active = voice.slots[voice.active_slot]
            if active.note == midi_note:
                active.target_gain = level
                active.fade_ms = max(1, fade_ms)
                return

            inactive_idx = 1 - voice.active_slot
            new_slot = voice.slots[inactive_idx]
            vel = int(self._clampf(level, 0.0, 1.0) * 90.0 + 30.0)
            self._play_on_slot(new_slot, inst, midi_note, vel)
            new_slot.gain = 0.0
            new_slot.target_gain = level
            new_slot.fade_ms = max(1, fade_ms)

            active.target_gain = 0.0
            active.fade_ms = max(1, fade_ms)
            voice.active_slot = inactive_idx

    def set_level(self, device_id: int, level: float, fade_ms: int) -> None:
        with self._lock:
            voice = self._voices.get(device_id)
            if voice is None:
                return
            slot = voice.slots[voice.active_slot]
            if slot.note is None:
                return
            slot.target_gain = self._clampf(level, 0.0, 1.0)
            slot.fade_ms = max(1, fade_ms)

    def stop_device(self, device_id: int, release_ms: int) -> None:
        with self._lock:
            voice = self._voices.get(device_id)
            if voice is None:
                return
            for slot in voice.slots:
                slot.target_gain = 0.0
                slot.fade_ms = max(1, release_ms)

    def _worker(self) -> None:
        last = time.monotonic()
        while self._running:
            now = time.monotonic()
            dt = max(0.0005, now - last)
            last = now

            with self._lock:
                for voice in self._voices.values():
                    for slot in voice.slots:
                        if slot.gain != slot.target_gain:
                            step = dt * 1000.0 / float(max(1, slot.fade_ms))
                            if slot.gain < slot.target_gain:
                                slot.gain = min(slot.target_gain, slot.gain + step)
                            else:
                                slot.gain = max(slot.target_gain, slot.gain - step)

                        vol = self._clampf(slot.gain * voice.device_gain, 0.0, 1.0)
                        cc = int(vol * 127.0)
                        if cc != slot.cc_last:
                            self.fs.cc(slot.channel, 7, cc)
                            slot.cc_last = cc

                        if slot.note is not None and slot.gain <= 0.001 and slot.target_gain <= 0.001:
                            self.fs.noteoff(slot.channel, slot.note)
                            slot.note = None

            time.sleep(self.tick_s)

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        with self._lock:
            for voice in self._voices.values():
                for slot in voice.slots:
                    if slot.note is not None:
                        self.fs.noteoff(slot.channel, slot.note)
            self._voices.clear()
        self.fs.delete()

    def audio_self_test(self, duration_s: float = 0.35, note: int = 72, vel: int = 100) -> None:
        with self._lock:
            for voice in self._voices.values():
                if not voice.slots:
                    continue
                slot = voice.slots[voice.active_slot]
                inst = voice.instrument
                sfid = self._get_sfid(inst.soundfont)
                self._select_program(slot, sfid, inst.bank, inst.preset)
                # Force audible channel volume for the self-test tone.
                self.fs.cc(slot.channel, 7, 127)
                self._dbg(f"selftest cc7 ch={slot.channel} value=127")
                rc_on = self.fs.noteon(slot.channel, int(note), int(vel))
                self._dbg(f"selftest noteon ch={slot.channel} note={note} vel={vel} rc={rc_on}")
        time.sleep(max(0.05, float(duration_s)))
        with self._lock:
            for voice in self._voices.values():
                for slot in voice.slots:
                    if slot.note is not None:
                        rc_off = self.fs.noteoff(slot.channel, slot.note)
                        self._dbg(f"selftest noteoff ch={slot.channel} note={slot.note} rc={rc_off}")
                # Ensure test note is also released even if slot.note not tracked.
                for slot in voice.slots:
                    self.fs.noteoff(slot.channel, int(note))


def parse_device_ids(value: str) -> list[int]:
    out = []
    for tok in value.split(","):
        tok = tok.strip()
        if not tok:
            continue
        did = int(tok, 0)
        if did < 0 or did > 0x7F:
            raise argparse.ArgumentTypeError("device IDs must be 0..127")
        out.append(did)
    if not out:
        raise argparse.ArgumentTypeError("empty device-id list")
    return sorted(set(out))


def create_fluidsynth_synth():
    if fluidsynth is None:
        raise RuntimeError("python module 'fluidsynth' is not installed")

    synth_ctor = getattr(fluidsynth, "Synth", None)
    if callable(synth_ctor):
        return synth_ctor()

    nested = getattr(fluidsynth, "fluidsynth", None)
    if nested is not None:
        synth_ctor = getattr(nested, "Synth", None)
        if callable(synth_ctor):
            return synth_ctor()

    mod_file = getattr(fluidsynth, "__file__", "<unknown>")
    raise RuntimeError(
        "Installed 'fluidsynth' module is incompatible (no Synth class). "
        f"module={mod_file}. Install 'pyfluidsynth' and remove conflicting 'fluidsynth' package."
    )


def load_json(path: str | None) -> dict[str, Any]:
    if not path:
        return {}
    p = Path(path).expanduser()
    if not p.is_absolute():
        p = (Path(__file__).resolve().parent / p).resolve()
    if not p.exists():
        return {}
    with p.open("r", encoding="utf-8") as f:
        return json.load(f)


def resolve_config_path(path: str) -> Path:
    p = Path(path).expanduser()
    if not p.is_absolute():
        p = (Path(__file__).resolve().parent / p).resolve()
    return p


def resolve_soundfont_path(soundfont_arg: str) -> str:
    p = Path(soundfont_arg).expanduser()
    if not p.is_absolute():
        p = (Path(__file__).resolve().parent / p).resolve()
    if p.is_dir():
        preferred = p / "piano.sf2"
        if preferred.is_file():
            return str(preferred)
        candidates = sorted(p.glob("*.sf2"))
        if candidates:
            return str(candidates[0])
        raise RuntimeError(f"No .sf2 files found in directory: {p}")
    return str(p)


def build_device_instrument(default_soundfont: str, default_bank: int, default_preset: int,
                            defaults: dict[str, Any], override: dict[str, Any]) -> InstrumentProfile:
    soundfont = default_soundfont
    bank = int(default_bank)
    preset = int(default_preset)

    d_inst = defaults.get("instrument")
    if isinstance(d_inst, str):
        soundfont = d_inst
    elif isinstance(d_inst, dict):
        if "soundfont" in d_inst:
            soundfont = str(d_inst.get("soundfont"))
        if "bank" in d_inst:
            bank = int(d_inst.get("bank"))
        if "preset" in d_inst:
            preset = int(d_inst.get("preset"))

    o_inst = override.get("instrument")
    if isinstance(o_inst, str):
        soundfont = o_inst
    elif isinstance(o_inst, dict):
        if "soundfont" in o_inst:
            soundfont = str(o_inst.get("soundfont"))
        if "bank" in o_inst:
            bank = int(o_inst.get("bank"))
        if "preset" in o_inst:
            preset = int(o_inst.get("preset"))

    # Backward-compat: if old config used sensor_instruments, prefer "event" then "mag".
    if "instrument" not in override and isinstance(override.get("sensor_instruments"), dict):
        legacy = override.get("sensor_instruments")
        cand = legacy.get("event", legacy.get("mag"))
        if isinstance(cand, str):
            soundfont = cand
        elif isinstance(cand, dict):
            if "soundfont" in cand:
                soundfont = str(cand.get("soundfont"))
            if "bank" in cand:
                bank = int(cand.get("bank"))
            if "preset" in cand:
                preset = int(cand.get("preset"))

    soundfont = resolve_soundfont_path(soundfont)
    return InstrumentProfile(soundfont=soundfont, bank=bank, preset=preset)


def merge_device_profile(device_id: int, defaults: dict[str, Any], override: dict[str, Any],
                         global_source: str, default_soundfont: str) -> DeviceProfile:
    cfg = {}
    cfg.update(defaults)
    cfg.update(override)

    source = str(cfg.get("event_source", global_source)).strip().lower()
    if source not in ("hardware", "software"):
        source = global_source

    note_map = [int(x) for x in cfg.get("note_map", [60, 61, 63, 65, 66, 68])]
    if not note_map:
        note_map = [60, 61, 63, 65, 66, 68]

    instrument = build_device_instrument(
        default_soundfont=default_soundfont,
        default_bank=int(cfg.get("bank", 0)),
        default_preset=int(cfg.get("preset", 0)),
        defaults=defaults,
        override=override,
    )

    return DeviceProfile(
        device_id=device_id,
        event_source=source,
        note_map=note_map,
        gain=float(cfg.get("gain", 1.0)),
        crossfade_ms=int(cfg.get("crossfade_ms", 90)),
        release_ms=int(cfg.get("release_ms", 450)),
        intensity_full_scale=max(1, int(cfg.get("intensity_full_scale", 100))),
        min_level=float(cfg.get("min_level", 0.18)),
        max_level=float(cfg.get("max_level", 1.0)),
        instrument=instrument,
    )


def parse_frame_event(data: bytes) -> Event | None:
    if len(data) < 8 or data[0] != 0 or data[1] != FRAME_EVENT:
        return None
    return Event(
        type=int(data[2]),
        p0=int(data[3]),
        p1=int(data[4]),
        p2=int(data[5]),
        p3=int.from_bytes(data[6:8], "little"),
    )


def parse_frame_mag(data: bytes) -> tuple[int, int, int] | None:
    if len(data) < 8 or data[0] != 0 or data[1] != FRAME_MAG:
        return None
    x = int.from_bytes(data[2:4], "little", signed=True)
    y = int.from_bytes(data[4:6], "little", signed=True)
    z = int.from_bytes(data[6:8], "little", signed=True)
    return x, y, z


def fetch_calibration(channel: str, interface: str, timeout: float, device_id: int) -> dict[str, int] | None:
    client = AppCanClient(channel=channel, interface=interface, device_id=device_id, timeout=timeout)
    try:
        vals = client.calib_get(0)
        out: dict[str, int] = {}
        for v in vals:
            out[str(v["field_name"])] = int(v["value"])
        return out
    except Exception:
        return None
    finally:
        client.close()


def ensure_streams(channel: str, interface: str, timeout: float, profile: DeviceProfile, mag_ms: int, evt_ms: int) -> None:
    client = AppCanClient(channel=channel, interface=interface, device_id=profile.device_id, timeout=timeout)
    try:
        if profile.event_source == "software":
            client.set_stream_enable(1, True)
            client.set_interval(1, mag_ms)
        else:
            client.set_stream_enable(4, True)
            client.set_interval(4, evt_ms)
    finally:
        client.close()


def level_from_intensity(profile: DeviceProfile, intensity: int) -> float:
    t = float(max(0, intensity)) / float(max(1, profile.intensity_full_scale))
    if t > 1.0:
        t = 1.0
    lo = max(0.0, min(1.0, profile.min_level))
    hi = max(lo, min(1.0, profile.max_level))
    return lo + (hi - lo) * t


def note_for_sector(profile: DeviceProfile, sector: int) -> int:
    idx = max(1, sector) - 1
    return int(profile.note_map[idx % len(profile.note_map)])


def apply_event_to_mixer(runtime: DeviceRuntime, ev: Event, mixer: SynthMixer, print_events: bool,
                         verbose_debug: bool = False) -> None:
    p = runtime.profile
    runtime.events_rx += 1
    runtime.last_event_s = time.monotonic()

    if print_events:
        print(
            f"[event {runtime.profile.event_source}] dev={p.device_id:02d} "
            f"{EVENT_NAMES.get(ev.type, ev.type)} p0={ev.p0} p1={ev.p1} p2={ev.p2}"
        )

    if ev.type == EVENT_SECTOR_ACTIVATED:
        runtime.current_sector = ev.p0
        runtime.current_level = level_from_intensity(p, ev.p1)
        mixer.play_note(p.device_id, note_for_sector(p, ev.p0), runtime.current_level, p.crossfade_ms)
        runtime.note_on_actions += 1
    elif ev.type == EVENT_SECTOR_CHANGED:
        runtime.current_sector = ev.p1
        if runtime.current_level <= 0.01:
            runtime.current_level = level_from_intensity(p, 60)
        mixer.play_note(p.device_id, note_for_sector(p, ev.p1), runtime.current_level, p.crossfade_ms)
        runtime.note_on_actions += 1
    elif ev.type == EVENT_PASSING_SECTOR_CHANGE:
        runtime.current_sector = ev.p0
        if runtime.current_level <= 0.01:
            runtime.current_level = level_from_intensity(p, 60)
        mixer.play_note(p.device_id, note_for_sector(p, ev.p0), runtime.current_level, max(20, p.crossfade_ms // 2))
        runtime.note_on_actions += 1
    elif ev.type == EVENT_INTENSITY_CHANGE:
        runtime.current_level = level_from_intensity(p, ev.p1)
        mixer.set_level(p.device_id, runtime.current_level, 60)
        runtime.level_actions += 1
    elif ev.type in (EVENT_SECTION_DEACTIVATED, EVENT_SESSION_ENDED, EVENT_POSSIBLE_MECHANICAL_FAILURE, EVENT_ERROR_NO_DATA):
        runtime.current_sector = 0
        runtime.current_level = 0.0
        mixer.stop_device(p.device_id, p.release_ms)
        runtime.note_off_actions += 1
    elif verbose_debug:
        print(
            f"[dbg apply] dev={p.device_id:02d} evt={EVENT_NAMES.get(ev.type, ev.type)} "
            f"received but does not trigger sound action"
        )

    if verbose_debug:
        print(
            f"[dbg apply] dev={p.device_id:02d} mode={p.event_source} "
            f"evt={EVENT_NAMES.get(ev.type, ev.type)} -> sector={runtime.current_sector} "
            f"level={runtime.current_level:.3f}"
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Multi-device CAN music player (FluidSynth)")
    p.add_argument("--channel", default="slcan0", help="CAN channel (default: slcan0)")
    p.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    p.add_argument("--device-ids", type=parse_device_ids, help="comma-separated IDs, e.g. 1,2,3")
    p.add_argument("--device-id", type=int, help="single device ID (alias for --device-ids)")
    p.add_argument("--discover-timeout", type=float, default=0.35, help="autodiscovery timeout in seconds")
    p.add_argument("--config", default="app_player_config.json", help="JSON config path (default: app_player_config.json)")
    p.add_argument("--source", choices=["hardware", "software"], default="hardware", help="default event source")
    p.add_argument("--soundfont", default="../../sounds/", help="path to .sf2 soundfont or directory")
    p.add_argument("--driver", default="auto", help="FluidSynth driver (default: auto)")
    p.add_argument("--bank", type=int, default=0, help="default MIDI bank")
    p.add_argument("--preset", type=int, default=0, help="default MIDI preset/program")
    p.add_argument("--crossfade-ms", type=int, default=90, help="default crossfade time")
    p.add_argument("--release-ms", type=int, default=450, help="default fade-out time")
    p.add_argument("--tick-ms", type=float, default=5.0, help="audio update tick")
    p.add_argument("--timeout", type=float, default=1.2, help="per-command timeout for setup/calibration")
    p.add_argument("--setup-streams", dest="setup_streams", action="store_true", help="set low-latency stream config")
    p.add_argument("--no-setup-streams", dest="setup_streams", action="store_false", help="leave stream config unchanged")
    p.set_defaults(setup_streams=True)
    p.add_argument("--mag-ms", type=int, default=20, help="mag stream interval for software mode")
    p.add_argument("--event-ms", type=int, default=20, help="event stream interval for hardware mode")
    p.add_argument("--stats-interval", type=float, default=2.0, help="periodic stats print interval")
    p.add_argument("--print-events", action="store_true", help="print every incoming/generated event")
    p.add_argument("--verbose-debug", action="store_true", help="verbose runtime debug logs (very chatty)")
    p.add_argument("--audio-self-test", action="store_true", help="play short startup note per selected device")
    return p


class CanQueueListener(can.Listener):
    def __init__(self, q: queue.Queue[tuple[int, bytes, float]], runtimes: dict[int, DeviceRuntime]):
        super().__init__()
        self.q = q
        self.runtimes = runtimes

    def on_message_received(self, msg: can.Message) -> None:
        did = status_id_to_device_id(msg.arbitration_id)
        if did is None:
            return
        rt = self.runtimes.get(did)
        if rt is None:
            return
        ts = float(msg.timestamp) if msg.timestamp is not None else time.time()
        try:
            self.q.put_nowait((did, bytes(msg.data), ts))
        except queue.Full:
            rt.dropped_frames += 1


def run() -> int:
    args = build_parser().parse_args()

    def dbg(msg: str):
        if args.verbose_debug:
            ts = time.strftime("%H:%M:%S")
            print(f"[dbg {ts}] {msg}")

    if fluidsynth is None:
        print("Missing dependency: python FluidSynth module (`fluidsynth`).")
        print("Install package `pyfluidsynth` and ensure system FluidSynth library is available.")
        return 2
    if args.device_id is not None:
        if args.device_id < 0 or args.device_id > 0x7F:
            print("--device-id must be 0..127")
            return 2
        args.device_ids = [args.device_id]
    try:
        args.soundfont = resolve_soundfont_path(args.soundfont)
    except Exception as e:
        print(f"Soundfont path error: {e}")
        return 2
    config_path = resolve_config_path(args.config)
    cfg = load_json(args.config)
    cfg_defaults = dict(cfg.get("defaults", {}))
    dev_cfg_map = dict(cfg.get("devices", {}))

    device_select_source = "unknown"
    if args.device_ids is not None:
        device_ids = args.device_ids
        device_select_source = "cli"
    elif dev_cfg_map:
        ids = []
        for key in dev_cfg_map.keys():
            try:
                did = int(key)
            except ValueError:
                continue
            if 0 <= did <= 0x7F:
                ids.append(did)
        device_ids = sorted(set(ids))
        device_select_source = "config"
    else:
        found = discover_devices(args.channel, args.interface, timeout=args.discover_timeout)
        device_ids = [int(rec["device_id"]) for rec in found]
        device_select_source = "discovery"

    if not device_ids:
        print("No devices selected/discovered")
        return 1

    if len(device_ids) > 8:
        print("Too many devices for 2-channel-per-device mixer (max 8)")
        return 1

    if config_path.exists():
        print(f"Config: {config_path}")
    else:
        print(f"Config not found: {config_path} (continuing with defaults/discovery)")
    print(f"Selected devices ({device_select_source}): {','.join(str(x) for x in device_ids)}")
    dbg(
        f"startup channel={args.channel} iface={args.interface} source_default={args.source} "
        f"setup_streams={int(args.setup_streams)} print_events={int(args.print_events)}"
    )

    runtimes: dict[int, DeviceRuntime] = {}
    for did in device_ids:
        dev_override = dict(dev_cfg_map.get(str(did), {}))
        profile = merge_device_profile(
            device_id=did,
            defaults={
                "crossfade_ms": args.crossfade_ms,
                "release_ms": args.release_ms,
                "bank": args.bank,
                "preset": args.preset,
            } | cfg_defaults,
            override=dev_override,
            global_source=args.source,
            default_soundfont=args.soundfont,
        )
        runtimes[did] = DeviceRuntime(profile=profile, last_rx_s=time.monotonic())

    # Software event source needs calibration for detector parity with firmware.
    for did, rt in runtimes.items():
        if rt.profile.event_source != "software":
            continue
        override_cal = dev_cfg_map.get(str(did), {}).get("calibration")
        if isinstance(override_cal, dict):
            cal = {str(k): int(v) for k, v in override_cal.items()}
        else:
            cal = fetch_calibration(args.channel, args.interface, args.timeout, did)
        if cal is None:
            print(f"dev={did:02d}: calibration read failed, using defaults for software event detector")
            rt.detector = EventDetector(EventDetectionConfig())
        else:
            rt.detector = EventDetector(EventDetectionConfig.from_calibration(cal))

    if args.setup_streams:
        for did, rt in runtimes.items():
            try:
                ensure_streams(args.channel, args.interface, args.timeout, rt.profile, args.mag_ms, args.event_ms)
                dbg(
                    f"stream-setup dev={did:02d} mode={rt.profile.event_source} "
                    f"mag_ms={args.mag_ms} event_ms={args.event_ms}"
                )
            except Exception as e:
                print(f"dev={did:02d}: stream setup warning: {e}")

    try:
        mixer = SynthMixer(soundfont=args.soundfont, driver=args.driver, tick_ms=args.tick_ms, debug=args.verbose_debug)
    except Exception as e:
        print(f"FluidSynth init failed: {e}")
        return 2
    print(f"Audio driver: {mixer.audio_driver_name} (requested: {args.driver})")
    if args.verbose_debug and mixer.audio_start_attempts:
        print("Audio attempts: " + ", ".join(mixer.audio_start_attempts))
    try:
        channels = list(range(16))
        for idx, did in enumerate(device_ids):
            ch_a = channels[idx * 2]
            ch_b = channels[idx * 2 + 1]
            p = runtimes[did].profile
            mixer.register_device(did, ch_a, ch_b, p.gain, p.instrument)
            print(
                f"dev={did:02d} mode={p.event_source} channels=({ch_a},{ch_b}) "
                f"notes={p.note_map}"
            )
            sf_name = Path(p.instrument.soundfont).name
            print(f"  instrument: sf={sf_name} bank={p.instrument.bank} preset={p.instrument.preset}")
            dbg(
                f"device dev={did:02d} mode={p.event_source} gain={p.gain:.3f} "
                f"note_map={p.note_map} sf={p.instrument.soundfont}"
            )

        if args.audio_self_test:
            print("Running audio self-test (startup note)...")
            mixer.audio_self_test()
            print("Audio self-test done.")

        bus = can.Bus(
            interface=args.interface,
            channel=args.channel,
            receive_own_messages=False,
            can_filters=[{
                "can_id": 0x580,
                "can_mask": 0x780,
                "extended": False,
            }],
        )
        q: queue.Queue[tuple[int, bytes, float]] = queue.Queue(maxsize=8192)
        listener = CanQueueListener(q, runtimes)
        notifier = can.Notifier(bus, [listener], timeout=0.02)

        stop_event = threading.Event()

        def _sig_handler(_sig, _frame):
            stop_event.set()

        signal.signal(signal.SIGINT, _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)

        last_stats = time.monotonic()
        unknown_ids_seen: set[int] = set()
        print("Player running. Press Ctrl+C to stop.")
        while not stop_event.is_set():
            now = time.monotonic()
            try:
                did, data, _ts = q.get(timeout=0.05)
            except queue.Empty:
                did = -1
                data = b""

            if did >= 0:
                rt = runtimes.get(did)
                if rt is None:
                    if did not in unknown_ids_seen:
                        unknown_ids_seen.add(did)
                        print(f"Ignoring frames from non-selected device id {did}")
                    continue
                rt.frames_rx += 1
                rt.last_rx_s = now

                if len(data) >= 2 and data[0] == 0 and data[1] == FRAME_MAG and rt.profile.event_source == "software":
                    mag = parse_frame_mag(data)
                    if mag is not None and rt.detector is not None:
                        x, y, z = mag
                        rt.last_mag_s = now
                        events = rt.detector.process_mag_sample(float(x), float(y), float(-z), now)
                        if args.verbose_debug and events:
                            dbg(f"sw-detector dev={did:02d} produced={len(events)}")
                        for ev in events:
                            rt.local_events_rx += 1
                            apply_event_to_mixer(rt, ev, mixer, args.print_events, args.verbose_debug)
                elif len(data) >= 2 and data[0] == 0 and data[1] == FRAME_EVENT:
                    rt.hw_event_frames_rx += 1
                    ev = parse_frame_event(data)
                    if ev is not None:
                        if rt.profile.event_source == "hardware":
                            apply_event_to_mixer(rt, ev, mixer, args.print_events, args.verbose_debug)
                            dbg(f"hw-event-apply dev={did:02d} evt={EVENT_NAMES.get(ev.type, ev.type)}")
                        else:
                            rt.hw_event_frames_ignored += 1
                            dbg(f"hw-event-ignore dev={did:02d} mode=software evt={EVENT_NAMES.get(ev.type, ev.type)}")
                            if args.print_events:
                                print(
                                    f"[event ignored] dev={did:02d} mode=software "
                                    f"{EVENT_NAMES.get(ev.type, ev.type)} p0={ev.p0} p1={ev.p1} p2={ev.p2}"
                                )
                    else:
                        dbg(f"hw-event-parse-fail dev={did:02d} raw={data.hex()}")

            for rt in runtimes.values():
                if rt.profile.event_source != "software" or rt.detector is None:
                    continue
                timeout_s = rt.detector.config.session_timeout_ms / 1000.0
                if rt.last_mag_s > 0.0 and (now - rt.last_mag_s) > timeout_s:
                    evs = rt.detector.post_no_data(now)
                    if evs:
                        dbg(f"sw-post-no-data dev={rt.profile.device_id:02d} emitted={len(evs)}")
                        for ev in evs:
                            rt.local_events_rx += 1
                            apply_event_to_mixer(rt, ev, mixer, args.print_events, args.verbose_debug)

            if args.stats_interval > 0 and (now - last_stats) >= args.stats_interval:
                last_stats = now
                print("---- stats ----")
                for did in sorted(runtimes.keys()):
                    rt = runtimes[did]
                    age_ms = int((now - rt.last_rx_s) * 1000.0) if rt.last_rx_s > 0 else -1
                    print(
                        f"dev={did:02d} mode={rt.profile.event_source} "
                        f"rx={rt.frames_rx} hw_rx={rt.hw_event_frames_rx} hw_ign={rt.hw_event_frames_ignored} "
                        f"applied={rt.events_rx} sw_ev={rt.local_events_rx} "
                        f"on={rt.note_on_actions} lvlchg={rt.level_actions} off={rt.note_off_actions} "
                        f"drop={rt.dropped_frames} sector={rt.current_sector} "
                        f"lvl={rt.current_level:.2f} last_rx={age_ms}ms"
                    )

        notifier.stop()
        bus.shutdown()
    finally:
        mixer.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(run())
