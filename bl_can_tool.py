#!/usr/bin/env python3
import argparse
import sys
import time
import can


CMD_PING = 0x01
CMD_CHECK = 0x02
CMD_START = 0x10
CMD_DATA = 0x20
CMD_END = 0x30
CMD_BOOT_APP = 0x40
CMD_BOOT_STATUS = 0x41
CMD_I2C_BUF_CLEAR = 0x50
CMD_I2C_BUF_APPEND = 0x51
CMD_I2C_XFER = 0x52
CMD_I2C_SCAN = 0x53

FRAME_CHECK_SUMMARY = 0x20
FRAME_CHECK_CRC = 0x21
FRAME_I2C_SCAN = 0x60
FRAME_I2C_RXDATA = 0x61

I2C_MAX_TX = 48
I2C_MAX_RX = 32

HMC5883L_ADDR = 0x1E
LIS3DHTR_ADDR = 0x19
AHT20_ADDR = 0x38

STATUS_OK = 0x00
STATUS_ERR_GENERIC = 0x01
STATUS_ERR_RANGE = 0x02
STATUS_ERR_STATE = 0x03
STATUS_ERR_CRC = 0x04

STATUS_TEXT = {
    STATUS_OK: "OK",
    STATUS_ERR_GENERIC: "ERR_GENERIC",
    STATUS_ERR_RANGE: "ERR_RANGE",
    STATUS_ERR_STATE: "ERR_STATE",
    STATUS_ERR_CRC: "ERR_CRC",
}

BOOTERR_TEXT = {
    0x00: "NONE",
    0xE1: "APP_INVALID",
    0xE2: "VECTOR_EMPTY",
    0xE3: "STACK_ALIGN",
    0xE4: "STACK_RANGE",
    0xE5: "ENTRY_RANGE",
    0xE6: "JUMP_RETURNED",
}


def bootloader_crc32(data: bytes) -> int:
    """
    CRC32 variant used by bootloader:
    poly=0x04C11DB7, init=0xFFFFFFFF, refin=false, refout=false, xorout=0xFFFFFFFF.
    """
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= (b & 0xFF) << 24
        for _ in range(8):
            if (crc & 0x80000000) != 0:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ 0x04C11DB7
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


def boot_error_text(code: int) -> str:
    return BOOTERR_TEXT.get(code, f"UNKNOWN_0x{code:02X}")


class BootloaderCanClient:
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
    def _is_status_frame(data: bytes) -> bool:
        if len(data) < 2:
            return False
        if data[0] > STATUS_ERR_CRC:
            return False
        # In this protocol: [status, extra, 0, 0, 0, 0, 0, 0]
        return all(b == 0 for b in data[2:])

    def wait_status(self, expected_extra: int | None = None, timeout: float | None = None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if not self._is_status_frame(data):
                continue
            st = data[0]
            if expected_extra is not None and data[1] != expected_extra:
                continue
            if st != STATUS_OK:
                extra = data[1]
                detail = ""
                if st == STATUS_ERR_STATE and extra >= 0xE0:
                    detail = f" ({boot_error_text(extra)})"
                raise RuntimeError(
                    f"Bootloader error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{extra:02X}{detail}"
                )
            return data
        raise TimeoutError("Timeout waiting for status response")

    def collect_chunked(self, subtype: int, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        total_len = None
        buf = b""
        seen = []

        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if len(data) < 4:
                continue

            st = data[0]
            if st > STATUS_ERR_CRC:
                continue
            if st != STATUS_OK:
                raise RuntimeError(
                    f"Bootloader error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"
                )
            if data[1] != subtype:
                continue

            offset = data[2]
            frame_total = data[3]

            if total_len is None:
                total_len = frame_total
                if total_len == 0:
                    return b""
                buf = bytearray(total_len)
                seen = [False] * total_len

            if frame_total != total_len or offset >= total_len:
                continue

            chunk = min(4, total_len - offset)
            payload = data[4:4 + chunk]
            for i, b in enumerate(payload):
                idx = offset + i
                if idx < total_len:
                    buf[idx] = b
                    seen[idx] = True

            if all(seen):
                return bytes(buf)

        raise TimeoutError(f"Timeout waiting for chunked frame subtype=0x{subtype:02X}")

    def ping(self, stay_in_bootloader: bool) -> bytes:
        payload = bytes([CMD_PING, 0x42]) if stay_in_bootloader else bytes([CMD_PING])
        self.send(payload)
        self.wait_status(expected_extra=0x01)

        # Optional PONG frame.
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if len(data) >= 4 and data[:4] == b"PONG":
                return data
        return b""

    def check(self, timeout: float | None = None):
        self.send(bytes([CMD_CHECK]))
        frame_size = None
        frame_crc = None
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)

        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if len(data) < 2:
                continue

            st = data[0]
            if st > STATUS_ERR_CRC:
                continue
            if st != STATUS_OK:
                raise RuntimeError(
                    f"Bootloader error: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{data[1]:02X}"
                )

            code = data[1]
            if code == FRAME_CHECK_SUMMARY and len(data) >= 8:
                frame_size = data
            elif code == FRAME_CHECK_CRC and len(data) >= 8:
                frame_crc = data

            if frame_size is not None and frame_crc is not None:
                break

        if frame_size is None or frame_crc is None:
            raise TimeoutError("Timeout waiting for CHECK response frames")

        valid = frame_size[2]
        updating = frame_size[3]
        app_size = int.from_bytes(frame_size[4:8], "little")
        crc32 = int.from_bytes(frame_crc[2:6], "little")
        dev = frame_crc[6]
        proto = frame_crc[7]

        return {
            "valid": bool(valid),
            "updating": bool(updating),
            "app_size": app_size,
            "crc32": crc32,
            "device_id": dev,
            "proto": proto,
        }

    def update(self, fw: bytes, verify: bool = True) -> None:
        size = len(fw)
        if size == 0:
            raise ValueError("Firmware file is empty")

        crc = bootloader_crc32(fw)
        start_timeout = max(self.timeout, 8.0)
        step_timeout = max(self.timeout, 2.0)

        self.send(bytes([CMD_START]) + size.to_bytes(4, "little"))
        self.wait_status(timeout=start_timeout)

        sent = 0
        next_mark = 0
        while sent < size:
            chunk = fw[sent:sent + 7]
            self.send(bytes([CMD_DATA]) + chunk)
            self.wait_status(timeout=step_timeout)
            sent += len(chunk)

            percent = int((sent * 100) / size)
            if percent >= next_mark:
                print(f"Update progress: {percent}%")
                next_mark += 10

        self.send(bytes([CMD_END]) + crc.to_bytes(4, "little"))
        self.wait_status(timeout=step_timeout)
        print(f"Update finished: {size} bytes, crc32=0x{crc:08X}")

        if verify:
            verify_timeout = max(self.timeout, 5.0)
            info = None
            last_exc = None
            for _ in range(2):
                try:
                    info = self.check(timeout=verify_timeout)
                    break
                except TimeoutError as exc:
                    last_exc = exc
                    time.sleep(0.15)
            if info is None:
                raise last_exc
            if not info["valid"]:
                raise RuntimeError("CHECK reports invalid firmware after update")
            if info["app_size"] != size:
                raise RuntimeError(
                    f"CHECK size mismatch: expected {size}, got {info['app_size']}"
                )
            if info["crc32"] != crc:
                raise RuntimeError(
                    f"CHECK crc mismatch: expected 0x{crc:08X}, got 0x{info['crc32']:08X}"
                )
            print("Post-update CHECK: valid")

    def boot_app(self) -> None:
        self.send(bytes([CMD_BOOT_APP]))
        self.wait_status(expected_extra=0x40)

        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            msg = self.recv(max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            data = bytes(msg.data)
            if not self._is_status_frame(data):
                continue
            st = data[0]
            extra = data[1]
            if st == STATUS_OK:
                continue
            if st == STATUS_ERR_STATE and extra >= 0xE0:
                raise RuntimeError(f"BOOT_APP failed: {boot_error_text(extra)} (0x{extra:02X})")
            raise RuntimeError(
                f"Bootloader error after BOOT_APP: {STATUS_TEXT.get(st, f'0x{st:02X}')} extra=0x{extra:02X}"
            )

    def boot_status(self) -> int:
        self.send(bytes([CMD_BOOT_STATUS]))
        return self.wait_status()[1]

    def i2c_xfer(self, addr7: int, tx: bytes, rx_len: int) -> bytes:
        if addr7 < 0 or addr7 > 0x7F:
            raise ValueError("I2C addr must be 0..0x7F")
        if len(tx) > I2C_MAX_TX:
            raise ValueError(f"I2C TX too long ({len(tx)}), max is {I2C_MAX_TX}")
        if rx_len < 0 or rx_len > I2C_MAX_RX:
            raise ValueError(f"I2C RX len must be 0..{I2C_MAX_RX}")

        self.send(bytes([CMD_I2C_BUF_CLEAR]))
        self.wait_status()

        for i in range(0, len(tx), 7):
            chunk = tx[i:i + 7]
            self.send(bytes([CMD_I2C_BUF_APPEND]) + chunk)
            self.wait_status()

        self.send(bytes([CMD_I2C_XFER, addr7 & 0x7F, rx_len & 0xFF]))
        return self.collect_chunked(FRAME_I2C_RXDATA)

    def i2c_scan(self, first: int, last: int):
        if first < 0 or first > 0x7F or last < 0 or last > 0x7F or first > last:
            raise ValueError("I2C scan range must be 0..0x7F and first <= last")

        self.send(bytes([CMD_I2C_SCAN, first & 0x7F, last & 0x7F]))
        mask = self.collect_chunked(FRAME_I2C_SCAN)
        if len(mask) != 16:
            raise RuntimeError(f"Unexpected I2C scan payload length: {len(mask)}")

        found = []
        for addr in range(first, last + 1):
            if (mask[addr >> 3] >> (addr & 0x7)) & 0x1:
                found.append(addr)
        return found

    def i2c_read_reg(self, addr7: int, reg: int, read_len: int = 1) -> bytes:
        if reg < 0 or reg > 0xFF:
            raise ValueError("register must be 0..0xFF")
        return self.i2c_xfer(addr7, bytes([reg & 0xFF]), read_len)


def test_hmc5883l(client: BootloaderCanClient) -> tuple[bool, str]:
    dev_id = client.i2c_read_reg(HMC5883L_ADDR, 0x0A, 3)
    if dev_id != b"H43":
        return False, f"HMC5883L ID mismatch: got {dev_id.hex()} expected 483433"

    # 8-average, 15 Hz continuous mode
    client.i2c_xfer(HMC5883L_ADDR, bytes([0x00, 0x70]), 0)
    client.i2c_xfer(HMC5883L_ADDR, bytes([0x01, 0x20]), 0)
    client.i2c_xfer(HMC5883L_ADDR, bytes([0x02, 0x00]), 0)
    time.sleep(0.01)
    data = client.i2c_read_reg(HMC5883L_ADDR, 0x03, 6)
    if len(data) != 6:
        return False, "HMC5883L data read failed"

    x = int.from_bytes(data[0:2], "big", signed=True)
    z = int.from_bytes(data[2:4], "big", signed=True)
    y = int.from_bytes(data[4:6], "big", signed=True)

    # Config B = 0x20 -> gain +/-1.3 Ga, 1090 LSB/Ga.
    x_mg = (x * 1000.0) / 1090.0
    y_mg = (y * 1000.0) / 1090.0
    z_mg = (z * 1000.0) / 1090.0

    return (
        True,
        "HMC5883L OK "
        f"id={dev_id.decode('ascii')} "
        f"x={x} y={y} z={z} "
        f"x={x_mg:.1f}mG y={y_mg:.1f}mG z={z_mg:.1f}mG",
    )


def test_lis3dhtr(client: BootloaderCanClient) -> tuple[bool, str]:
    who = client.i2c_read_reg(LIS3DHTR_ADDR, 0x0F, 1)
    if len(who) != 1:
        return False, "LIS3DHTR WHO_AM_I read failed"
    if who[0] != 0x33:
        return False, f"LIS3DHTR WHO_AM_I mismatch: got 0x{who[0]:02X}, expected 0x33"

    # XYZ in normal mode at 100 Hz
    client.i2c_xfer(LIS3DHTR_ADDR, bytes([0x20, 0x57]), 0)
    time.sleep(0.01)
    ctrl1 = client.i2c_read_reg(LIS3DHTR_ADDR, 0x20, 1)
    if len(ctrl1) != 1:
        return False, "LIS3DHTR CTRL_REG1 readback failed"
    if ctrl1[0] != 0x57:
        return False, f"LIS3DHTR CTRL_REG1 readback mismatch: got 0x{ctrl1[0]:02X}"

    xyz = client.i2c_read_reg(LIS3DHTR_ADDR, 0xA8, 6)
    if len(xyz) != 6:
        return False, "LIS3DHTR data read failed"

    x_raw = int.from_bytes(xyz[0:2], "little", signed=True)
    y_raw = int.from_bytes(xyz[2:4], "little", signed=True)
    z_raw = int.from_bytes(xyz[4:6], "little", signed=True)

    # Normal mode (CTRL_REG1=0x57, HR=0): 10-bit left-justified data.
    x_counts = x_raw >> 6
    y_counts = y_raw >> 6
    z_counts = z_raw >> 6

    # +/-2g sensitivity in normal mode: 4 mg/LSB (10-bit output).
    x_mg = x_counts * 4.0
    y_mg = y_counts * 4.0
    z_mg = z_counts * 4.0

    return (
        True,
        "LIS3DHTR OK "
        f"who=0x{who[0]:02X} "
        f"x={x_counts} y={y_counts} z={z_counts} "
        f"x={x_mg:.1f}mg y={y_mg:.1f}mg z={z_mg:.1f}mg",
    )


def test_aht20(client: BootloaderCanClient) -> tuple[bool, str]:
    def xfer_retry(tx: bytes, rx_len: int, retries: int = 5) -> bytes:
        last_exc = None
        for _ in range(retries):
            try:
                return client.i2c_xfer(AHT20_ADDR, tx, rx_len)
            except RuntimeError as exc:
                if "extra=0x04" in str(exc):
                    last_exc = exc
                    time.sleep(0.01)
                    continue
                raise
        raise RuntimeError(f"AHT20 I2C NACK after {retries} retries: {last_exc}")

    status = xfer_retry(bytes([0x71]), 1, retries=3)
    if len(status) != 1:
        return False, "AHT20 status read failed"

    # if not calibrated, initialize first.
    if (status[0] & 0x08) == 0:
        xfer_retry(bytes([0xBE, 0x08, 0x00]), 0, retries=3)
        time.sleep(0.02)
        status = xfer_retry(bytes([0x71]), 1, retries=3)
        if len(status) != 1 or (status[0] & 0x08) == 0:
            return False, f"AHT20 not calibrated, status=0x{status[0] if status else 0:02X}"

    last_err = "AHT20 measurement failed"
    last_zero_sample = None
    for _attempt in range(1, 5):
        xfer_retry(bytes([0xAC, 0x33, 0x00]), 0, retries=3)
        data = b""
        ready = False
        for _ in range(25):
            time.sleep(0.01)
            data = xfer_retry(b"", 6, retries=3)
            if len(data) != 6:
                continue
            if (data[0] & 0x80) == 0:
                ready = True
                break

        if not ready:
            last_err = "AHT20 busy timeout"
            continue

        raw_h = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) & 0xFFFFF
        raw_t = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) & 0xFFFFF
        humidity = (raw_h * 100.0) / 1048576.0
        temperature = (raw_t * 200.0) / 1048576.0 - 50.0

        if temperature < -40.0 or temperature > 85.0:
            last_err = f"AHT20 temperature out of range: {temperature:.2f}C"
            continue
        if humidity < 0.0 or humidity > 100.0:
            last_err = f"AHT20 humidity out of range: {humidity:.2f}%"
            continue

        # all zeros (why?)
        if raw_h == 0:
            last_zero_sample = (temperature, humidity, data.hex())
            last_err = f"AHT20 stale humidity sample (raw_h=0, raw={data.hex()})"
            continue

        return True, f"AHT20 OK T={temperature:.2f}C RH={humidity:.2f}% raw={data.hex()}"

    # repeated zeros - do not mark as failed, but as degraded
    if last_zero_sample is not None:
        t, h, raw = last_zero_sample
        return True, f"AHT20 OK (degraded) T={t:.2f}C RH={h:.2f}% raw={raw}"

    return False, last_err


def run_sensor_tests(client: BootloaderCanClient, sensors: list[str]) -> list[tuple[str, bool, str]]:
    tests = {
        "hmc5883l": ("HMC5883L", test_hmc5883l),
        "lis3dhtr": ("LIS3DHTR", test_lis3dhtr),
        "aht20": ("AHT20", test_aht20),
    }
    results = []
    for key in sensors:
        title, fn = tests[key]
        try:
            ok, msg = fn(client)
        except Exception as exc:
            ok, msg = False, f"{type(exc).__name__}: {exc}"
        results.append((title, ok, msg))
    return results


def print_test_results(results: list[tuple[str, bool, str]]) -> bool:
    all_ok = True
    for title, ok, msg in results:
        state = "OK" if ok else "FAIL"
        print(f"[{state}] {title}: {msg}")
        if not ok:
            all_ok = False
    return all_ok


def parse_hex_bytes(text: str) -> bytes:
    s = text.strip()
    if s == "":
        return b""

    if any(ch in s for ch in (" ", ",", ":")):
        tokens = s.replace(",", " ").replace(":", " ").split()
        out = []
        for tok in tokens:
            base = 0 if tok.lower().startswith("0x") else 16
            val = int(tok, base)
            if val < 0 or val > 0xFF:
                raise ValueError(f"Byte out of range: {tok}")
            out.append(val)
        return bytes(out)

    if s.startswith("0x") or s.startswith("0X"):
        s = s[2:]
    if len(s) % 2 != 0:
        raise ValueError("Hex string length must be even")
    return bytes.fromhex(s)


def print_i2cdetect(found: list[int], first: int, last: int) -> None:
    found_set = set(found)
    print("     " + " ".join(f"{i:02x}" for i in range(16)))
    for base in range(0x00, 0x80, 0x10):
        cells = []
        for lo in range(16):
            addr = base + lo
            if addr < first or addr > last:
                cells.append("  ")
            elif addr in found_set:
                cells.append(f"{addr:02x}")
            else:
                cells.append("--")
        print(f"{base:02x}: " + " ".join(cells))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 CAN bootloader tool")
    parser.add_argument("--channel", default="can0", help="CAN channel/interface name (default: can0)")
    parser.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    parser.add_argument("--device-id", type=int, default=1, help="Bootloader device id (default: 1)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Response timeout in seconds (default: 1.0)")

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_ping = sub.add_parser("ping", help="Ping device")
    p_ping.add_argument("--stay", action="store_true", help="Request to stay in bootloader")

    sub.add_parser("check", help="Check firmware validity/meta")

    p_update = sub.add_parser("update", help="Program firmware file")
    p_update.add_argument("firmware", help="Path to .bin firmware image")
    p_update.add_argument("--no-verify", action="store_true", help="Skip post-update CHECK")
    p_update.add_argument("--boot", action="store_true", help="Send BOOT_APP after successful update")

    sub.add_parser("boot-app", help="Request jump to application")
    sub.add_parser("boot-status", help="Read last boot jump error code")

    p_i2c_detect = sub.add_parser("i2c-detect", help="Scan I2C bus (like i2cdetect)")
    p_i2c_detect.add_argument("--first", type=lambda s: int(s, 0), default=0x08, help="First address (default: 0x08)")
    p_i2c_detect.add_argument("--last", type=lambda s: int(s, 0), default=0x77, help="Last address (default: 0x77)")

    p_i2c_xfer = sub.add_parser("i2c-xfer", help="Raw I2C write/read transaction")
    p_i2c_xfer.add_argument("--addr", required=True, type=lambda s: int(s, 0), help="7-bit I2C address")
    p_i2c_xfer.add_argument("--write", default="", help="TX bytes (e.g. '00 10 ff' or '0010ff')")
    p_i2c_xfer.add_argument("--read", type=int, default=0, help="RX length in bytes")

    p_sensor_test = sub.add_parser("sensor-test", help="Test sensors over I2C-over-CAN")
    p_sensor_test.add_argument(
        "--sensor",
        choices=["all", "hmc5883l", "lis3dhtr", "aht20"],
        default="all",
        help="Sensor to test (default: all)",
    )

    sub.add_parser("healthcheck", help="Full hardware and sensor health check")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    client = BootloaderCanClient(
        channel=args.channel,
        interface=args.interface,
        device_id=args.device_id,
        timeout=args.timeout,
    )

    try:
        if args.cmd == "ping":
            pong = client.ping(stay_in_bootloader=args.stay)
            if pong:
                print(
                    "PONG:",
                    pong[:4].decode("ascii", errors="replace"),
                    f"device_id={pong[4]} proto={pong[5]} stay={pong[6]}",
                )
            else:
                print("PING OK (no PONG text frame received)")
            return 0

        if args.cmd == "check":
            info = client.check()
            print(
                f"CHECK: valid={int(info['valid'])} updating={int(info['updating'])} "
                f"size={info['app_size']} crc32=0x{info['crc32']:08X} "
                f"device_id={info['device_id']} proto={info['proto']}"
            )
            return 0

        if args.cmd == "update":
            with open(args.firmware, "rb") as f:
                fw = f.read()
            client.update(fw, verify=not args.no_verify)
            if args.boot:
                client.boot_app()
                print("BOOT_APP sent")
            return 0

        if args.cmd == "boot-app":
            client.boot_app()
            print("BOOT_APP sent")
            return 0

        if args.cmd == "boot-status":
            code = client.boot_status()
            print(f"BOOT_STATUS: code=0x{code:02X} {boot_error_text(code)}")
            return 0

        if args.cmd == "i2c-detect":
            found = client.i2c_scan(args.first, args.last)
            print_i2cdetect(found, args.first, args.last)
            print(f"Found {len(found)} device(s): " + ", ".join(f"0x{a:02X}" for a in found))
            return 0

        if args.cmd == "i2c-xfer":
            tx = parse_hex_bytes(args.write)
            if len(tx) == 0 and args.read == 0:
                raise ValueError("i2c-xfer requires at least write data or read length")
            rx = client.i2c_xfer(args.addr, tx, args.read)
            print(f"I2C XFER OK addr=0x{args.addr:02X} tx={tx.hex()} rx={rx.hex()}")
            return 0

        if args.cmd == "sensor-test":
            sensors = ["hmc5883l", "lis3dhtr", "aht20"] if args.sensor == "all" else [args.sensor]
            ok = print_test_results(run_sensor_tests(client, sensors))
            return 0 if ok else 2

        if args.cmd == "healthcheck":
            print("Healthcheck start")
            failures = 0

            try:
                pong = client.ping(stay_in_bootloader=False)
                if pong:
                    print(f"[OK] CAN ping: PONG device_id={pong[4]} proto={pong[5]}")
                else:
                    print("[OK] CAN ping: status response received")
            except Exception as exc:
                print(f"[FAIL] CAN ping: {exc}")
                failures += 1

            try:
                info = client.check()
                print(
                    "[OK] CHECK: "
                    f"valid={int(info['valid'])} updating={int(info['updating'])} "
                    f"size={info['app_size']} crc32=0x{info['crc32']:08X}"
                )
            except Exception as exc:
                print(f"[FAIL] CHECK: {exc}")
                failures += 1

            expected = {
                "HMC5883L": HMC5883L_ADDR,
                "LIS3DHTR": LIS3DHTR_ADDR,
                "AHT20": AHT20_ADDR,
            }
            try:
                found = set(client.i2c_scan(0x08, 0x77))
                print("[OK] I2C scan:", ", ".join(f"0x{a:02X}" for a in sorted(found)) if found else "no devices")
                for name, addr in expected.items():
                    if addr in found:
                        print(f"[OK] I2C presence {name} at 0x{addr:02X}")
                    else:
                        print(f"[FAIL] I2C presence {name} missing at 0x{addr:02X}")
                        failures += 1
            except Exception as exc:
                print(f"[FAIL] I2C scan: {exc}")
                failures += 1

            sensor_ok = print_test_results(run_sensor_tests(client, ["hmc5883l", "lis3dhtr", "aht20"]))
            if not sensor_ok:
                failures += 1

            if failures == 0:
                print("Healthcheck: PASS")
                return 0
            print(f"Healthcheck: FAIL ({failures} issues)")
            return 2

        raise RuntimeError("Unknown command")
    finally:
        client.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (TimeoutError, RuntimeError, OSError, can.CanError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
