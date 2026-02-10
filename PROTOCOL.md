# Bootloader CAN Protocol

## Bus IDs

- Command ID (host -> bootloader): `0x600 + device_id`
- Response ID (bootloader -> host): `0x580 + device_id`

`device_id` is harcoded in bootloader (`BL_DEVICE_ID`).

## Device ID and Metadata

Bootloader stores metadata in last flash page (`0x0801F800`):

```c
typedef struct {
    uint32_t magic;    // BL_META_MAGIC
    uint32_t size;     // app size in bytes
    uint32_t crc32;    // app CRC32
    uint32_t reserved; // encoded device_id tag
} bl_meta_t;
```

`reserved` encodes device id as:
- tag: `0xA5D10000`
- low byte: `device_id`

On firmware update end (`CMD_END`), bootloader writes metadata including encoded `device_id`.
On boot, if app is valid but metadata does not contain current encoded ID, bootloader rewrites metadata with current ID.

## Common Status Frame

Most command acks/errors are sent as:

- Byte0: status
- Byte1: extra
- Byte2..7: `0x00`

Status codes:

- `0x00` `OK`
- `0x01` `ERR_GENERIC`
- `0x02` `ERR_RANGE`
- `0x03` `ERR_STATE`
- `0x04` `ERR_CRC`

Boot error codes (used with `ERR_STATE`/`BOOT_STATUS`):

- `0x00` `NONE`
- `0xE1` `APP_INVALID`
- `0xE2` `VECTOR_EMPTY`
- `0xE3` `STACK_ALIGN`
- `0xE4` `STACK_RANGE`
- `0xE5` `ENTRY_RANGE`
- `0xE6` `JUMP_RETURNED`

## Commands

### `0x01` `PING`

Payload:
- `[0x01]` normal ping
- `[0x01, 0x42]` request stay in bootloader

Response:
- status: `OK, extra=0x01`
- text frame `"PONG"`:
  - Byte0..3: ASCII `PONG`
  - Byte4: `device_id`
  - Byte5: protocol version
  - Byte6: `stay_in_bl` flag
  - Byte7: `0xA5`

### `0x02` `CHECK`

Payload:
- `[0x02]`

Response (two data frames, not plain status frame):

1) summary frame (`subtype=0x20`)
- Byte0: `0x00` (`OK`)
- Byte1: `0x20`
- Byte2: `valid_app` (0/1)
- Byte3: `updating` (0/1)
- Byte4..7: `app_size` (LE uint32)

2) CRC frame (`subtype=0x21`)
- Byte0: `0x00` (`OK`)
- Byte1: `0x21`
- Byte2..5: `app_crc32` (LE uint32)
- Byte6: `device_id`
- Byte7: protocol version

### `0x10` `START`

Payload:
- `[0x10, size_u32_le]`

Action:
- validates size (`1..APP_MAX_SIZE`)
- erases app area
- enters update mode

Response:
- status `OK` on success
- errors: `ERR_RANGE` (invalid size), `ERR_GENERIC` (erase/program prep failure)

### `0x20` `DATA`

Payload:
- `[0x20, up to 7 data bytes]`

Action:
- appends firmware bytes in update mode
- updates running CRC

Response:
- status `OK` on accepted chunk
- errors: `ERR_STATE` (not updating), `ERR_RANGE` (overflow), `ERR_GENERIC` (flash write)

### `0x30` `END`

Payload:
- `[0x30, crc32_u32_le]`

Action:
- finalizes write tail
- compares CRC/size
- writes metadata (`magic,size,crc,reserved=device_id`)

Response:
- status `OK` on success
- errors: `ERR_CRC` (CRC/size mismatch), `ERR_GENERIC` (flush/meta write)

### `0x40` `BOOT_APP`

Payload:
- `[0x40]`

Response:
- immediate status: `OK, extra=0x40`
- then bootloader attempts jump to app
- if jump fails, bootloader may emit `ERR_STATE, extra=<boot_error_code>`

### `0x41` `BOOT_STATUS`

Payload:
- `[0x41]`

Response:
- status frame with `status=OK`, `extra=<last_boot_error_code>`

### `0x50` `I2C_BUF_CLEAR`

Payload:
- `[0x50]`

Action:
- clears internal TX staging buffer for I2C transfer

Response:
- status `OK`

### `0x51` `I2C_BUF_APPEND`

Payload:
- `[0x51, 1..7 bytes]`

Action:
- appends bytes to internal TX staging buffer (max 48 bytes)

Response:
- status `OK, extra=<new_tx_len>`
- errors: `ERR_STATE` if I2C not ready, `ERR_RANGE` on overflow

### `0x52` `I2C_XFER`

Payload:
- `[0x52, addr7, rx_len]`

Action:
- performs optional TX (from staged buffer) and optional RX (`rx_len<=32`)

Response:
- on success: chunked frame stream subtype `0x61` containing RX bytes
- on error: status error (`ERR_GENERIC` with HAL I2C error low byte, or `ERR_RANGE`/`ERR_STATE`)

### `0x53` `I2C_SCAN`

Payload:
- `[0x53]` (default scan range)
- `[0x53, first_addr, last_addr]`

Action:
- scans addresses and returns 16-byte bitmap for `0x00..0x7F`

Response:
- on success: chunked frame stream subtype `0x60` (16-byte mask)
- on error: status error (`ERR_RANGE`/`ERR_STATE`)

## Chunked Frame Format

Used by `I2C_SCAN` and `I2C_XFER`.

- Byte0: `0x00` (`OK`)
- Byte1: subtype (`0x60` scan, `0x61` i2c rx)
- Byte2: offset in full payload (step 4)
- Byte3: total payload length
- Byte4..7: up to 4 payload bytes for this chunk

## Asynchronous Text/Startup Frames

### Startup frame `"BLST"`

Sent on bootloader CAN init.

- Byte0..3: ASCII `BLST`
- Byte4: `device_id`
- Byte5: protocol version
- Byte6: flags
  - bit0: valid app present
  - bit1: I2C bridge ready
  - bit2: force-stay compile option active
- Byte7: reset cause low byte (`RCC->CSR & 0xFF`)

### Ping frame `"PONG"`

Sent after `PING` (see `CMD_PING`).

## Notes

- Unknown command: status `ERR_GENERIC, extra=0xFF`.
- Bootloader receives only standard 11-bit data frames on its command ID.
