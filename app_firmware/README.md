# App Firmware

- MCU: `STM32L432xx`
- CAN ID is get from bootloader

## Build

```bash
cd app_firmware
make -j4
```

## Flash

```bash
cd app_firmware
make flash_openocd
```
Or using bootloader CLI tool:
```bash
cd app_firmware
./app_can_tool.py --interface socketcan --channel slcan0 --device-id 3 enter-bootloader
cd ..
./bl_can_tool.py --interface socketcan --channel slcan0 --device-id 3 update app_firmware/app_firmware.bin --boot
```

When flashing manually, make sure to enter right address `0x08004000`. 



## Python App Tool

```bash
# Ping
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 ping

# Read status summary
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 status

# Configure stream intervals
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 set-interval --stream mag --ms 100
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 get-interval --stream all
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 set-interval --stream mag --ms 100 --save

# Enable / disable streams
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --off
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --on
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --on --save

# Reset into bootloader
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 enter-bootloader

# HMC5883L configuration
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 hmc-config
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 hmc-config --range 8.1 --save

# AHT20 read
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 aht20-read

# Calibration
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-get --field all
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-set --field rotate_xy --value 150
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-capture-earth
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-save
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-load

# Monitor mode
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 monitor

# GTK visualizer
python3 app_calib_gtk.py --interface socketcan --channel slcan0 --device-id 1
```

## Multi-Device Player

`app_player.py` is a low-latency FluidSynth player for one or more CAN devices.
Each device can use:
- `hardware` events (`FRAME_EVENT` from firmware), or
- `software` events (computed locally from `FRAME_MAG` using `event_detection.py`).
- one instrument per CAN device (separate soundfont/program per device).

### Basic usage

```bash
# Autodiscover devices, hardware events.
# By default, settings are loaded from app_player_config.json.
python3 app_player.py --interface socketcan --channel slcan0

# Explicit devices, software events
python3 app_player.py --interface socketcan --channel slcan0 --device-ids 1,2 --source software
```

### Optional JSON config

```json
{
  "defaults": {
    "event_source": "hardware",
    "note_map": [60, 61, 63, 65, 66, 68],
    "crossfade_ms": 90,
    "release_ms": 450,
    "gain": 1.0,
    "instrument": { "soundfont": "../../sounds/piano.sf2", "bank": 0, "preset": 0 }
  },
  "devices": {
    "1": {
      "event_source": "software",
      "note_map": [60, 62, 64, 67, 69, 72],
      "instrument": { "soundfont": "../../sounds/random.sf2", "bank": 0, "preset": 0 }
    },
    "2": {
      "event_source": "hardware",
      "note_map": [48, 50, 52, 55, 57, 60],
      "instrument": { "soundfont": "../../sounds/guitar.sf2", "bank": 0, "preset": 0 }
    }
  }
}
```

Run with:

```bash
python3 app_player.py --interface socketcan --channel slcan0
# or explicit config:
python3 app_player.py --interface socketcan --channel slcan0 --config app_player_config.json
```

Or start from the included template:

```bash
cp app_player_config.example.json app_player_config.json
python3 app_player.py --interface socketcan --channel slcan0
```
