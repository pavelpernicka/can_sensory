#!/usr/bin/env python3
from __future__ import annotations

import json
import queue
import shlex
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import can

from app_can_tool import FRAME_EVENT, FRAME_EVENT_STATE, status_id_to_device_id

EVENT_SECTOR_ACTIVATED = 1
EVENT_SECTOR_CHANGED = 2
EVENT_PASSING_SECTOR_CHANGE = 7
EVENT_SECTION_DEACTIVATED = 4
EVENT_SESSION_ENDED = 6
EVENT_POSSIBLE_MECHANICAL_FAILURE = 8
EVENT_ERROR_NO_DATA = 9


@dataclass
class Instrument:
    type: str = "soundfont"
    soundfont: str = ""
    bank: int = 0
    preset: int = 0
    midi_port: str | None = None
    midi_device: str | None = None
    faust_dsp: str | None = None
    faust_command: list[str] | None = None
    faust_midi_port: str | None = None
    faust_midi_device: str | None = None
    faust_startup_wait_ms: int = 300
    faust_audio_device: str | None = None


@dataclass
class DeviceConfig:
    device_id: int
    event_source: str
    note_map: list[int]
    instrument: Instrument
    velocity: int = 110
    transpose: int = 0
    midi_cc: dict[int, int] = field(default_factory=dict)
    pitch_bend: int | None = None
    channel_pressure: int | None = None
    note_duration_ms: int | None = None
    fadein_ms: int | None = None
    fadeout_ms: int | None = None
    exclude_from_beat_quantize: bool = False


@dataclass
class GlobalPlayerConfig:
    bpm: float | None = None
    channel: str | None = None
    idle_reset_s: float | None = None
    ignore_sector_zero: bool | None = None
    beat_quantize: bool | None = None


CC_NAME_TO_NUM: dict[str, int] = {
    "mod": 1,
    "modulation": 1,
    "vibrato": 1,
    "volume": 7,
    "vol": 7,
    "pan": 10,
    "expression": 11,
    "expr": 11,
    "sustain": 64,
    "damper": 64,
    "resonance": 71,
    "filter_q": 71,
    "release": 72,
    "release_time": 72,
    "attack": 73,
    "attack_time": 73,
    "brightness": 74,
    "cutoff": 74,
    "filter_cutoff": 74,
    "reverb": 91,
    "reverb_send": 91,
    "chorus": 93,
    "chorus_send": 93,
}


class CanQueueListener(can.Listener):
    def __init__(self, q: queue.Queue[tuple[int, int]]):
        super().__init__()
        self.q = q
        self._last_sector_by_did: dict[int, int] = {}

    def on_message_received(self, msg: can.Message) -> None:
        did = status_id_to_device_id(msg.arbitration_id)
        if did is None:
            return
        sector = extract_sector_for_player(bytes(msg.data))
        if sector is None:
            return
        if self._last_sector_by_did.get(did) == sector:
            return
        self._last_sector_by_did[did] = sector
        item = (did, sector)
        try:
            self.q.put_nowait(item)
        except queue.Full:
            # Keep most recent frames under overload: drop one stale frame and retry.
            try:
                self.q.get_nowait()
                self.q.put_nowait(item)
            except (queue.Empty, queue.Full):
                pass


def parse_event_frame(data: bytes) -> tuple[int, int, int, int] | None:
    if len(data) < 8 or data[0] != 0 or data[1] != FRAME_EVENT:
        return None
    return int(data[2]), int(data[3]), int(data[4]), int(data[5])


def parse_event_state_frame(data: bytes) -> tuple[int, int] | None:
    if len(data) < 4 or data[0] != 0 or data[1] != FRAME_EVENT_STATE:
        return None
    return int(data[2]), int(data[3])


def extract_sector_for_player(data: bytes) -> int | None:
    ev = parse_event_frame(data)
    if ev is not None:
        ev_id, p0, p1, _p2 = ev
        if ev_id == EVENT_SECTOR_ACTIVATED and p0 > 0:
            return p0
        if ev_id == EVENT_SECTOR_CHANGED and p1 > 0:
            return p1
        if ev_id == EVENT_PASSING_SECTOR_CHANGE and p0 > 0:
            return p0
        if ev_id in (
            EVENT_SECTION_DEACTIVATED,
            EVENT_SESSION_ENDED,
            EVENT_POSSIBLE_MECHANICAL_FAILURE,
            EVENT_ERROR_NO_DATA,
        ):
            return 0
        return None

    st = parse_event_state_frame(data)
    if st is not None:
        return int(st[0])
    return None


def resolve_local(path_str: str) -> Path:
    p = Path(path_str).expanduser()
    if not p.is_absolute():
        p = (Path(__file__).resolve().parent / p).resolve()
    return p


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def deep_merge(dst: dict[str, Any], src: dict[str, Any]) -> dict[str, Any]:
    out = dict(dst)
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def resolve_soundfont(path_str: str) -> str:
    p = resolve_local(path_str)
    if p.is_dir():
        preferred = p / "piano.sf2"
        if preferred.is_file():
            return str(preferred)
        cands = sorted(p.glob("*.sf2"))
        if cands:
            return str(cands[0])
        raise RuntimeError(f"No .sf2 files in directory: {p}")
    return str(p)


def _resolve_optional_local_path(path_str: Any) -> str | None:
    if path_str is None:
        return None
    s = str(path_str).strip()
    if not s:
        return None
    p = Path(s).expanduser()
    if not p.is_absolute():
        p = (Path(__file__).resolve().parent / p).resolve()
    return str(p)


def _clamp_int(value: Any, lo: int, hi: int, default: int) -> int:
    try:
        iv = int(value)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, iv))


def _opt_clamp_int(value: Any, lo: int, hi: int) -> int | None:
    if value is None:
        return None
    try:
        iv = int(value)
    except (TypeError, ValueError):
        return None
    return max(lo, min(hi, iv))


def _opt_clamp_float(value: Any, lo: float, hi: float) -> float | None:
    if value is None:
        return None
    try:
        fv = float(value)
    except (TypeError, ValueError):
        return None
    return max(lo, min(hi, fv))


def _opt_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        s = value.strip().lower()
        if s in {"1", "true", "yes", "on"}:
            return True
        if s in {"0", "false", "no", "off"}:
            return False
    return None


def _parse_midi_cc(raw: Any) -> dict[int, int]:
    out: dict[int, int] = {}
    if not isinstance(raw, dict):
        return out
    for key, value in raw.items():
        cc_num: int | None = None
        if isinstance(key, int):
            cc_num = key
        elif isinstance(key, str):
            key_s = key.strip().lower()
            if key_s.lstrip("+-").isdigit():
                cc_num = int(key_s)
            else:
                cc_num = CC_NAME_TO_NUM.get(key_s)
        if cc_num is None or cc_num < 0 or cc_num > 127:
            continue
        out[int(cc_num)] = _clamp_int(value, 0, 127, 0)
    return out


def _parse_any_cc(raw: dict[str, Any], keys: list[str]) -> dict[int, int]:
    out: dict[int, int] = {}
    for k in keys:
        out.update(_parse_midi_cc(raw.get(k, {})))
    return out


def _parse_command(raw: Any) -> list[str] | None:
    if raw is None:
        return None
    if isinstance(raw, list):
        out = [str(x).strip() for x in raw if str(x).strip()]
        return out or None
    if isinstance(raw, str):
        parts = shlex.split(raw)
        return [p for p in parts if p] or None
    return None


def load_device_configs(default_cfg_path: Path, user_cfg_path: Path) -> dict[int, DeviceConfig]:
    merged = deep_merge(load_json(default_cfg_path), load_json(user_cfg_path))
    raw_devices = merged.get("devices", {})
    out: dict[int, DeviceConfig] = {}
    if not isinstance(raw_devices, dict):
        return out

    for key, raw in raw_devices.items():
        if not isinstance(raw, dict):
            continue
        try:
            did = int(key)
        except ValueError:
            continue
        if did < 0 or did > 0x7F:
            continue

        source = str(raw.get("event_source", "hardware")).strip().lower()
        note_map = [int(x) for x in raw.get("note_map", [60, 61, 63, 65, 66, 68])]
        if not note_map:
            note_map = [60, 61, 63, 65, 66, 68]

        inst_raw = raw.get("instrument", {})
        inst_opts: dict[str, Any] = inst_raw if isinstance(inst_raw, dict) else {}
        inst_cc: dict[int, int] = {}
        if isinstance(inst_raw, str):
            inst = Instrument(
                type="soundfont",
                soundfont=resolve_soundfont(inst_raw),
            )
        elif isinstance(inst_raw, dict):
            inst_type = str(inst_raw.get("type", "soundfont")).strip().lower() or "soundfont"
            if inst_type == "faust":
                dsp = _resolve_optional_local_path(inst_raw.get("dsp", inst_raw.get("faust_dsp", None)))
                cmd = _parse_command(inst_raw.get("command", inst_raw.get("faust_command", None)))
                if cmd is None and dsp is not None:
                    cmd = ["faust2alsaconsole", "-midi", dsp]
                elif cmd is not None and len(cmd) >= 1:
                    exe = Path(cmd[0]).name.lower()
                    if exe.startswith("faust2") and ("-midi" not in cmd) and ("--midi" not in cmd):
                        # Most Faust standalone targets need explicit MIDI enable.
                        if len(cmd) >= 2:
                            cmd = [cmd[0], "-midi", *cmd[1:]]
                        else:
                            cmd = [cmd[0], "-midi"]
                inst = Instrument(
                    type="faust",
                    soundfont="",
                    bank=int(inst_raw.get("bank", 0)),
                    preset=int(inst_raw.get("preset", 0)),
                    faust_dsp=dsp,
                    faust_command=cmd,
                    faust_midi_port=(
                        str(inst_raw.get("midi_port", inst_raw.get("faust_midi_port", ""))).strip() or None
                    ),
                    faust_midi_device=_resolve_optional_local_path(
                        inst_raw.get("midi_device", inst_raw.get("faust_midi_device", None))
                    ),
                    faust_startup_wait_ms=_clamp_int(
                        inst_raw.get(
                            "startup_wait_ms",
                            inst_raw.get("faust_startup_wait_ms", 300),
                        ),
                        0,
                        15000,
                        300,
                    ),
                    faust_audio_device=(
                        str(inst_raw.get("audio_device", inst_raw.get("faust_audio_device", ""))).strip() or None
                    ),
                )
            elif inst_type == "midi":
                midi_port = str(
                    inst_raw.get(
                        "midi_port",
                        inst_raw.get("port", inst_raw.get("faust_midi_port", "")),
                    )
                ).strip() or None
                midi_device = _resolve_optional_local_path(
                    inst_raw.get(
                        "midi_device",
                        inst_raw.get("device", inst_raw.get("faust_midi_device", None)),
                    )
                )
                inst = Instrument(
                    type="midi",
                    soundfont="",
                    bank=int(inst_raw.get("bank", 0)),
                    preset=int(inst_raw.get("preset", 0)),
                    midi_port=midi_port,
                    midi_device=midi_device,
                )
            else:
                inst = Instrument(
                    type="soundfont",
                    soundfont=resolve_soundfont(str(inst_raw.get("soundfont", "../../sounds/piano.sf2"))),
                    bank=int(inst_raw.get("bank", 0)),
                    preset=int(inst_raw.get("preset", 0)),
                )
            inst_cc = _parse_any_cc(inst_raw, ["midi_cc", "cc", "controls", "soundfont_cc"])
        else:
            inst = Instrument(type="soundfont", soundfont=resolve_soundfont("../../sounds/piano.sf2"))

        dev_cc = _parse_any_cc(raw, ["midi_cc", "cc", "controls", "soundfont_cc"])
        midi_cc = dict(inst_cc)
        midi_cc.update(dev_cc)

        velocity = _clamp_int(raw.get("velocity", inst_opts.get("velocity", 110)), 0, 127, 110)
        transpose = _clamp_int(raw.get("transpose", inst_opts.get("transpose", 0)), -48, 48, 0)

        pb_raw = raw.get("pitch_bend", inst_opts.get("pitch_bend", None))
        pitch_bend = _clamp_int(pb_raw, -8192, 8191, 0) if pb_raw is not None else None
        cp_raw = raw.get("channel_pressure", inst_opts.get("channel_pressure", None))
        channel_pressure = _clamp_int(cp_raw, 0, 127, 0) if cp_raw is not None else None
        nd_raw = raw.get(
            "note_duration_ms",
            raw.get("duration_ms", inst_opts.get("note_duration_ms", inst_opts.get("duration_ms", None))),
        )
        note_duration_ms = _opt_clamp_int(nd_raw, 0, 60000)
        fi_raw = raw.get(
            "fadein_ms",
            raw.get("attack_ms", inst_opts.get("fadein_ms", inst_opts.get("attack_ms", None))),
        )
        fadein_ms = _opt_clamp_int(fi_raw, 0, 60000)
        fo_raw = raw.get(
            "fadeout_ms",
            raw.get("release_ms", inst_opts.get("fadeout_ms", inst_opts.get("release_ms", None))),
        )
        fadeout_ms = _opt_clamp_int(fo_raw, 0, 60000)
        exbq_raw = raw.get(
            "exclude_from_beat_quantize",
            raw.get("no_beat_quantize", raw.get("unquantized", False)),
        )
        exclude_from_beat_quantize = bool(_opt_bool(exbq_raw))

        out[did] = DeviceConfig(
            device_id=did,
            event_source=source,
            note_map=note_map,
            instrument=inst,
            velocity=velocity,
            transpose=transpose,
            midi_cc=midi_cc,
            pitch_bend=pitch_bend,
            channel_pressure=channel_pressure,
            note_duration_ms=note_duration_ms,
            fadein_ms=fadein_ms,
            fadeout_ms=fadeout_ms,
            exclude_from_beat_quantize=exclude_from_beat_quantize,
        )
    return out


def load_global_player_config(default_cfg_path: Path, user_cfg_path: Path) -> GlobalPlayerConfig:
    merged = deep_merge(load_json(default_cfg_path), load_json(user_cfg_path))
    global_raw = merged.get("global", {})
    if not isinstance(global_raw, dict):
        global_raw = {}
    bpm_raw = global_raw.get("bpm", merged.get("bpm", None))
    bpm = _opt_clamp_float(bpm_raw, 1.0, 1000.0)

    channel_raw = global_raw.get("channel", global_raw.get("can_channel", merged.get("channel", None)))
    channel = str(channel_raw).strip() if channel_raw is not None and str(channel_raw).strip() else None

    idle_raw = global_raw.get(
        "idle_reset_s",
        global_raw.get("idle-reset-s", merged.get("idle_reset_s", merged.get("idle-reset-s", None))),
    )
    idle_reset_s = _opt_clamp_float(idle_raw, 0.0, 3600.0)

    ignore_raw = global_raw.get(
        "ignore_sector_zero",
        global_raw.get("ignore-sector-zero", merged.get("ignore_sector_zero", merged.get("ignore-sector-zero", None))),
    )
    ignore_sector_zero = _opt_bool(ignore_raw)

    beat_quant_raw = global_raw.get(
        "beat_quantize",
        global_raw.get("beat-quantize", merged.get("beat_quantize", merged.get("beat-quantize", None))),
    )
    beat_quantize = _opt_bool(beat_quant_raw)
    if beat_quantize is None:
        disable_raw = global_raw.get(
            "disable_beats",
            global_raw.get("no_beats", merged.get("disable_beats", merged.get("no_beats", None))),
        )
        disable_beats = _opt_bool(disable_raw)
        if disable_beats is not None:
            beat_quantize = not disable_beats

    return GlobalPlayerConfig(
        bpm=bpm,
        channel=channel,
        idle_reset_s=idle_reset_s,
        ignore_sector_zero=ignore_sector_zero,
        beat_quantize=beat_quantize,
    )
