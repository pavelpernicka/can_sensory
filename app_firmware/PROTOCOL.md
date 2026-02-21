# App CAN Protocol

## Bus IDs

- Command ID (host -> app): `0x600 + device_id`
- Response ID (app -> host): `0x580 + device_id`

`device_id`:
- from bootloader metadata at `0x0801F800`
- uses encoded id from metadata `reserved` field when present/valid
- fallback to `APP_DEVICE_ID` (`1`) if missing/invalid

## Common Status Frame

- Byte0: status
- Byte1: extra
- Byte2..7: `0x00`

Status codes:

- `0x00` `OK`
- `0x01` `ERR_GENERIC`
- `0x02` `ERR_RANGE`
- `0x03` `ERR_STATE`
- `0x04` `ERR_SENSOR`

## Commands

### `0x01` `PING`

Payload:
- `[0x01]`

Response:
- status `OK, extra=0x01`
- text frame `"PONG"`:
  - Byte0..3: ASCII `PONG`
  - Byte4: `device_id`
  - Byte5: protocol version
  - Byte6: `0x5A`
  - Byte7: `0x00`

### `0x40` `ENTER_BOOTLOADER`

Payload:
- `[0x40]`

Response:
- status `OK, extra=0x40`
- app then requests reset to bootloader

### `0x50` `WS_SET_POWER`

Payload:
- `[0x50, on]`
- `on`: `0|1`

Response:
- status `OK, extra=0x50`
- `WS_STATE` frame (`subtype=0x47`)

### `0x51` `WS_SET_BRIGHTNESS`

Payload:
- `[0x51, brightness]`
- `brightness`: `0..255`

Response:
- status `OK, extra=0x51`
- `WS_STATE` frame (`subtype=0x47`)

### `0x52` `WS_SET_COLOR`

Payload:
- `[0x52, r, g, b]`
- each channel: `0..255`

Response:
- status `OK, extra=0x52`
- `WS_STATE` frame (`subtype=0x47`)

### `0x53` `WS_SET_ALL`

Payload:
- `[0x53, on, brightness, r, g, b]`

Response:
- status `OK, extra=0x53`
- `WS_STATE` frame (`subtype=0x47`)

### `0x54` `WS_GET_STATE`

Payload:
- `[0x54]`

Response:
- status `OK, extra=0x54`
- `WS_STATE` frame (`subtype=0x47`)

### `0x6E` `HMC_SET_CFG`

Payload:
- `[0x6E, range, data_rate, samples, mode]`
- valid ranges:
  - `range`: `0..7`
  - `data_rate`: `0..6`
  - `samples`: `0..3`
  - `mode`: `0..2`

Response:
- status `OK, extra=0x6E`
- `HMC_CFG` frame (`subtype=0x46`)

### `0x6F` `HMC_GET_CFG`

Payload:
- `[0x6F]`

Response:
- status `OK, extra=0x6F`
- `HMC_CFG` frame (`subtype=0x46`)

### `0x70` `SET_INTERVAL`

Payload:
- `[0x70, stream_id, interval_lo, interval_hi]`
- stream IDs:
  - `1` mag
  - `2` acc
  - `3` env
  - `4` event_state
- `interval_ms`: `0..60000`

Response:
- status `OK, extra=stream_id`
- `INTERVAL` frame (`subtype=0x30`) for that stream

### `0x71` `GET_INTERVAL`

Payload:
- `[0x71]` or `[0x71, stream_id]`
- `stream_id=0` means all streams

Response:
- interval frame(s) only (`subtype=0x30`)
- no explicit OK status on success path
- invalid stream -> status error

### `0x72` `SET_STREAM_ENABLE`

Payload:
- `[0x72, stream_id, enable]`
- `enable`: `0|1`

Response:
- status `OK, extra=stream_id`
- `INTERVAL` frame (`subtype=0x30`) for that stream

### `0x73` `GET_STATUS`

Payload:
- `[0x73]`

Response:
- status `OK, extra=0x73`
- `STATUS` frame (`subtype=0x31`)

### `0x74` `AHT20_READ`

Payload:
- `[0x74]`

Response:
- status `OK, extra=0x74`
- `AHT20_MEAS` frame (`subtype=0x40`)
- `AHT20_RAW` frame (`subtype=0x41`)

### `0x75` `AHT20_GET_STATUS`

Payload:
- `[0x75]`

Response:
- status `OK, extra=0x75`
- `AHT20_STATUS` frame (`subtype=0x42`)

### `0x76` `AHT20_RESET`

Payload:
- `[0x76]`

Response:
- status `OK, extra=0x76`
- `AHT20_STATUS` frame (`subtype=0x42`)

### `0x77` `AHT20_SET_REG`

Payload:
- `[0x77, 1..5 bytes]`

Response:
- status `OK, extra=0x77`
- `AHT20_REG` frame (`subtype=0x43`) echo/readback payload

### `0x78` `AHT20_GET_REG`

Payload:
- `[0x78, count]`, `count=1..5`

Response:
- status `OK, extra=0x78`
- `AHT20_REG` frame (`subtype=0x43`)

### `0x79` `CALIB_GET`

Payload:
- `[0x79]` or `[0x79, field_id]`
- `field_id=0` means all fields

Response:
- for all-fields: status `OK, extra=0x79`, then all `CALIB_VALUE` frames
- for single-field: status `OK, extra=field_id`, then one `CALIB_VALUE`

Calibration field IDs:
- `1` center_x
- `2` center_y
- `3` center_z
- `4` rotate_xy
- `5` rotate_xz
- `6` rotate_yz
- `7` keepout_rad
- `8` z_limit
- `9` data_radius
- `10` mag_offset_x
- `11` mag_offset_y
- `12` mag_offset_z
- `13` earth_x
- `14` earth_y
- `15` earth_z
- `16` earth_valid
- `17` num_sectors
- `18` z_max
- `19` elev_curve

### `0x7A` `CALIB_SET`

Payload:
- `[0x7A, field_id, value_lo, value_hi]` (int16)

Response:
- status `OK, extra=field_id`
- updated `CALIB_VALUE` frame

### `0x7B` `CALIB_SAVE`

Payload:
- `[0x7B]`

Response:
- status `OK, extra=0x7B`
- `CALIB_INFO` frame (`op=0x7B`)

Saves to flash page `0x0801F000`:
- calibration data
- stream intervals and enable mask
- HMC config

### `0x7C` `CALIB_LOAD`

Payload:
- `[0x7C]`

Response:
- status `OK, extra=0x7C`
- `CALIB_INFO` frame (`op=0x7C`)
- all `CALIB_VALUE` frames
- `INTERVAL` frames for streams 1..4
- `HMC_CFG` frame

### `0x7D` `CALIB_RESET`

Payload:
- `[0x7D]`

Response:
- status `OK, extra=0x7D`
- `CALIB_INFO` frame (`op=0x7D`)
- all `CALIB_VALUE` frames (default RAM values)
- `INTERVAL` frames for streams 1..4
- `HMC_CFG` frame

### `0x7E` `CALIB_CAPTURE_EARTH`

Payload:
- `[0x7E]`

Response:
- status `OK, extra=0x7E`
- `CALIB_INFO` frame (`op=0x7E`)
- `CALIB_VALUE` frames for earth fields:
  - `earth_x` (13)
  - `earth_y` (14)
  - `earth_z` (15)
  - `earth_valid` (16)

## Data/Info Frames

All binary protocol frames use:
- Byte0: `0x00`
- Byte1: subtype

### `0x02` `STARTUP`

- Byte2: `device_id`
- Byte3: protocol version
- Byte4: sensor bitfield
  - bit0 hmc present
  - bit1 lis present
  - bit2 aht present
- Byte5: stream enable bitfield
  - bit0 mag
  - bit1 acc
  - bit2 env
  - bit3 event
- Byte6: reset-cause low byte (`RCC->CSR & 0xFF`)
- Byte7: `0x00`

### `0x10` `MAG`

- Byte2..3: x (int16, mG)
- Byte4..5: y (int16, mG)
- Byte6..7: z (int16, mG)

### `0x11` `ACC`

- Byte2..3: x (int16, mg)
- Byte4..5: y (int16, mg)
- Byte6..7: z (int16, mg)

### `0x12` `ENV`

- Byte2..3: temperature (int16, centi-C)
- Byte4..5: humidity (uint16, centi-%RH)
- Byte6: valid flag
- Byte7: `0x00`

### `0x20` `EVENT`

- Byte2: event type
- Byte3: p0
- Byte4: p1
- Byte5: p2
- Byte6..7: p3 (uint16, LE)

Event type IDs:
- `1` sector_activated
- `2` sector_changed
- `3` intensity_change
- `4` section_deactivated
- `5` session_started
- `6` session_ended
- `7` passing_sector_change
- `8` possible_mechanical_failure
- `9` error_no_data

### `0x30` `INTERVAL`

- Byte2: stream_id
- Byte3: enabled (0/1)
- Byte4..5: interval_ms (uint16 LE)
- Byte6: `device_id`
- Byte7: protocol version

### `0x47` `WS_STATE`

- Byte2: strip enabled (`0|1`)
- Byte3: brightness (`0..255`)
- Byte4: red (`0..255`)
- Byte5: green (`0..255`)
- Byte6: blue (`0..255`)
- Byte7: configured strip length (LED count, low byte)

### `0x31` `STATUS`

- Byte2: sensor bitfield
- Byte3: stream bitfield
- Byte4: mag interval low byte
- Byte5: acc interval low byte
- Byte6: env interval low byte
- Byte7: event interval low byte

### `0x32` `EVENT_STATE`

- Byte2: current sector
- Byte3: current elevation
- Byte4..7: `0x00`

### `0x40` `AHT20_MEAS`

- Byte2..3: temperature (int16 centi-C)
- Byte4..5: humidity (uint16 centi-%RH)
- Byte6: sensor status byte
- Byte7: CRC ok flag

### `0x41` `AHT20_RAW`

- Byte2..4(low nibble): humidity raw 20-bit
- Byte5..7(low nibble): temperature raw 20-bit

### `0x42` `AHT20_STATUS`

- Byte2: status
- Byte3: sensor present (0/1)
- Byte4: env valid (0/1)
- Byte5: CRC ok (0/1)
- Byte6..7: `0x00`

### `0x43` `AHT20_REG`

- Byte2: count (<=5)
- Byte3..7: raw bytes

### `0x44` `CALIB_VALUE`

- Byte2: field_id
- Byte3..4: value (int16 LE)
- Byte5: `0x00`
- Byte6: `device_id`
- Byte7: protocol version

### `0x45` `CALIB_INFO`

- Byte2: op code (`0x7B`/`0x7C`/`0x7D`/`0x7E`)
- Byte3: result (currently `0` on success)
- Byte4: `device_id`
- Byte5: protocol version
- Byte6..7: `0x00`

### `0x46` `HMC_CFG`

- Byte2: range id
- Byte3: data-rate id
- Byte4: samples id
- Byte5: mode id
- Byte6..7: sensitivity in centi-mG-per-digit (uint16 LE)

## Error Behavior

- Unknown command: status `ERR_GENERIC, extra=0xFF`.
- Commands may return status error with context-specific `extra` code.
- `GET_INTERVAL` success path does not emit an explicit status frame.
