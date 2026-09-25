#!/usr/bin/env python3
"""Four-sensor Raspberry Pi motion logger with a two- to four-row display.

Keys: A/B/C/D selects a row, 1-7 selects its source, F shows a spectrogram,
T shows time-domain data, and Q quits. Run with --config to load the INI file.
"""

from __future__ import annotations

import argparse
import collections
import configparser
import json
import signal
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Callable

import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons
import numpy as np
import spidev
from smbus2 import SMBus


CODE_VERSION = "3.0.0"
SOURCE_NAMES = {
    1: "mpu_accel",
    2: "mpu_gyro",
    3: "lsm_accel",
    4: "lsm_gyro",
    5: "adxl355_accel",
    6: "scl3300_accel",
    7: "scl3300_angle",
}
SOURCE_LABELS = {
    "mpu_accel": "MPU-6050 acceleration",
    "mpu_gyro": "MPU-6050 gyroscope",
    "lsm_accel": "LSM6DSO acceleration",
    "lsm_gyro": "LSM6DSO gyroscope",
    "adxl355_accel": "ADXL355 acceleration",
    "scl3300_accel": "SCL3300 acceleration",
    "scl3300_angle": "SCL3300 inclination",
}
SOURCE_UNITS = {
    "mpu_accel": "g", "mpu_gyro": "degrees/s",
    "lsm_accel": "g", "lsm_gyro": "degrees/s",
    "adxl355_accel": "g", "scl3300_accel": "g",
    "scl3300_angle": "degrees",
}
SOURCE_BINARY_FORMATS = {
    "mpu_accel": "<3h", "mpu_gyro": "<3h",
    "lsm_accel": "<3h", "lsm_gyro": "<3h",
    "adxl355_accel": "<3i", "scl3300_accel": "<3h",
    "scl3300_angle": "<3h",
}
SOURCE_DEVICES = {
    "mpu_accel": "mpu6050", "mpu_gyro": "mpu6050",
    "lsm_accel": "lsm6dso", "lsm_gyro": "lsm6dso",
    "adxl355_accel": "adxl355", "scl3300_accel": "scl3300",
    "scl3300_angle": "scl3300",
}


def signed16(low: int, high: int) -> int:
    value = low | (high << 8)
    return value - 65536 if value & 0x8000 else value


def signed20(a: int, b: int, c: int) -> int:
    value = (a << 12) | (b << 4) | (c >> 4)
    return value - (1 << 20) if value & (1 << 19) else value


def make_spectrogram(values: np.ndarray, sample_rate: float, fft_samples: int,
                     overlap_percent: float) -> tuple[np.ndarray, np.ndarray, int]:
    length = min(fft_samples, len(values))
    if length < 16:
        return np.empty(0), np.empty((0, 0)), 0
    overlap = int(length * overlap_percent / 100.0)
    hop = max(1, length - min(overlap, length - 1))
    frames = np.lib.stride_tricks.sliding_window_view(values, length)[::hop]
    frames = frames - np.mean(frames, axis=1, keepdims=True)
    window = np.hanning(length)
    amplitude = np.abs(np.fft.rfft(frames * window, axis=1)) / max(window.sum(), 1.0)
    spectrum_db = 20.0 * np.log10(np.maximum(amplitude, np.finfo(float).tiny))
    return np.fft.rfftfreq(length, 1.0 / sample_rate), spectrum_db.T, hop


class MPU6050:
    name = "mpu6050"
    sources = ("mpu_accel", "mpu_gyro")
    binary_format = "<6h"

    def __init__(self, bus: int, address: int, accel_range: int, gyro_range: int):
        self.bus = SMBus(bus)
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.accel_scale = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}[accel_range]
        self.gyro_scale = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}[gyro_range]
        identity = self.bus.read_byte_data(address, 0x75)
        if identity not in (0x68, 0x69):
            raise RuntimeError(f"MPU-6050 WHO_AM_I=0x{identity:02X}, expected 0x68/0x69")
        self.bus.write_byte_data(address, 0x6B, 0x80)
        time.sleep(0.10)
        self.bus.write_byte_data(address, 0x6B, 0x01)
        self.bus.write_byte_data(address, 0x6C, 0x00)
        self.bus.write_byte_data(address, 0x1A, 0x00)
        self.bus.write_byte_data(address, 0x19, 7)  # 8 kHz / (7+1) = 1 kHz
        self.bus.write_byte_data(address, 0x1B,
                                 {250: 0, 500: 1, 1000: 2, 2000: 3}[gyro_range] << 3)
        self.bus.write_byte_data(address, 0x1C,
                                 {2: 0, 4: 1, 8: 2, 16: 3}[accel_range] << 3)

    def read(self) -> tuple[dict[str, tuple[float, float, float]],
                            dict[str, tuple[int, int, int]]]:
        data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        raw_a = tuple(signed16(data[i + 1], data[i]) for i in (0, 2, 4))
        raw_g = tuple(signed16(data[i + 1], data[i]) for i in (8, 10, 12))
        return {
            "mpu_accel": tuple(v / self.accel_scale for v in raw_a),
            "mpu_gyro": tuple(v / self.gyro_scale for v in raw_g),
        }, {"mpu_accel": raw_a, "mpu_gyro": raw_g}

    def close(self) -> None:
        self.bus.close()

    def metadata(self) -> dict[str, object]:
        return {"format": "little-endian 6 x int16", "channels":
                ["ax", "ay", "az", "gx", "gy", "gz"],
                "accel_range_g": self.accel_range,
                "accel_lsb_per_g": self.accel_scale,
                "gyro_range_dps": self.gyro_range,
                "gyro_lsb_per_dps": self.gyro_scale, "native_odr_hz": 1000}


class LSM6DSO:
    name = "lsm6dso"
    sources = ("lsm_accel", "lsm_gyro")
    binary_format = "<6h"

    def __init__(self, bus: int, address: int, accel_range: int, gyro_range: int):
        self.bus = SMBus(bus)
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.accel_scale = {2: 0.000061, 4: 0.000122, 8: 0.000244, 16: 0.000488}[accel_range]
        self.gyro_scale = {125: 0.004375, 250: 0.00875, 500: 0.0175,
                           1000: 0.035, 2000: 0.070}[gyro_range]
        identity = self.bus.read_byte_data(address, 0x0F)
        if identity != 0x6C:
            raise RuntimeError(f"LSM6DSO WHO_AM_I=0x{identity:02X}, expected 0x6C")
        self.bus.write_byte_data(address, 0x12, 0x01)  # software reset
        time.sleep(0.03)
        self.bus.write_byte_data(address, 0x12, 0x44)  # BDU + register increment
        accel_fs = {2: 0, 16: 1, 4: 2, 8: 3}[accel_range]
        self.bus.write_byte_data(address, 0x10, 0x80 | (accel_fs << 2))  # 1.666 kHz
        if gyro_range == 125:
            gyro_bits = 0x02
        else:
            gyro_bits = {250: 0, 500: 1, 1000: 2, 2000: 3}[gyro_range] << 2
        self.bus.write_byte_data(address, 0x11, 0x80 | gyro_bits)

    def read(self) -> tuple[dict[str, tuple[float, float, float]],
                            dict[str, tuple[int, int, int]]]:
        data = self.bus.read_i2c_block_data(self.address, 0x22, 12)
        raw_g = tuple(signed16(data[i], data[i + 1]) for i in (0, 2, 4))
        raw_a = tuple(signed16(data[i], data[i + 1]) for i in (6, 8, 10))
        return {
            "lsm_accel": tuple(v * self.accel_scale for v in raw_a),
            "lsm_gyro": tuple(v * self.gyro_scale for v in raw_g),
        }, {"lsm_accel": raw_a, "lsm_gyro": raw_g}

    def close(self) -> None:
        self.bus.close()

    def metadata(self) -> dict[str, object]:
        return {"format": "little-endian 6 x int16", "channels":
                ["ax", "ay", "az", "gx", "gy", "gz"],
                "accel_range_g": self.accel_range,
                "accel_g_per_lsb": self.accel_scale,
                "gyro_range_dps": self.gyro_range,
                "gyro_dps_per_lsb": self.gyro_scale,
                "native_odr_hz": 1666, "host_capture_rate_hz": 1000,
                "note": "Native 1.666 kHz output observed on a 1 kHz host grid."}


class ADXL355:
    name = "adxl355"
    sources = ("adxl355_accel",)
    binary_format = "<3i"

    def __init__(self, bus: int, device: int, speed_hz: int, accel_range: int):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0
        self.spi.max_speed_hz = speed_hz
        self.accel_range = accel_range
        self.scale = {2: 256000.0, 4: 128000.0, 8: 64000.0}[accel_range]
        ids = (self._read(0x00), self._read(0x01), self._read(0x02))
        if ids != (0xAD, 0x1D, 0xED):
            raise RuntimeError(
                "ADXL355 IDs=" + "/".join(f"0x{x:02X}" for x in ids)
                + ", expected 0xAD/0x1D/0xED"
            )
        self._write(0x2F, 0x52)
        time.sleep(0.02)
        self._write(0x28, 0x02)  # 1 kHz ODR, 250 Hz low-pass corner
        self._write(0x2C, {2: 1, 4: 2, 8: 3}[accel_range])
        self._write(0x2D, 0x00)  # measurement mode
        time.sleep(0.02)

    def _read_many(self, register: int, count: int) -> list[int]:
        return self.spi.xfer2([(register << 1) | 1] + [0] * count)[1:]

    def _read(self, register: int) -> int:
        return self._read_many(register, 1)[0]

    def _write(self, register: int, value: int) -> None:
        self.spi.xfer2([register << 1, value & 0xFF])

    def read(self) -> tuple[dict[str, tuple[float, float, float]],
                            dict[str, tuple[int, int, int]]]:
        data = self._read_many(0x08, 9)
        raw = tuple(signed20(*data[i:i + 3]) for i in (0, 3, 6))
        return {"adxl355_accel": tuple(v / self.scale for v in raw)}, \
            {"adxl355_accel": raw}

    def close(self) -> None:
        self._write(0x2D, 0x01)
        self.spi.close()

    def metadata(self) -> dict[str, object]:
        return {"format": "little-endian 3 x int32 (sign-extended 20-bit)",
                "channels": ["ax", "ay", "az"], "range_g": self.accel_range,
                "lsb_per_g": self.scale, "native_odr_hz": 1000,
                "low_pass_hz": 250}


class SCL3300:
    name = "scl3300"
    sources = ("scl3300_accel", "scl3300_angle")
    binary_format = "<6h"
    COMMANDS = {
        "acc_x": 0x040000F7, "acc_y": 0x080000FD, "acc_z": 0x0C0000FB,
        "ang_x": 0x240000C7, "ang_y": 0x280000CD, "ang_z": 0x2C0000CB,
        "status": 0x180000E5, "whoami": 0x40000091,
        "reset": 0xB4002098, "angles": 0xB0001F6F,
        "mode1": 0xB400001F, "mode2": 0xB4000102,
        "mode3": 0xB4000225, "mode4": 0xB4000338,
    }

    def __init__(self, bus: int, device: int, speed_hz: int, mode: int):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0
        self.spi.max_speed_hz = speed_hz
        self.mode = mode
        self.accel_scale = {1: 6000.0, 2: 3000.0, 3: 12000.0, 4: 12000.0}[mode]
        self.angle_scale = 182.0
        self.crc_errors = 0
        self.status_errors = 0
        self._transfer(self.COMMANDS["reset"])
        time.sleep(0.005)
        self._transfer(self.COMMANDS[f"mode{mode}"])
        time.sleep(0.100 if mode in (3, 4) else 0.025)
        self._transfer(self.COMMANDS["angles"])
        time.sleep(0.005)
        self._transfer(self.COMMANDS["whoami"])
        whoami_response = self._transfer(self.COMMANDS["whoami"])
        if self._data(whoami_response, check_status=False) & 0xFF != 0xC1:
            raise RuntimeError("SCL3300 WHOAMI did not return 0xC1")
        self._transfer(self.COMMANDS["status"])
        self._transfer(self.COMMANDS["status"])

    @staticmethod
    def _crc(word: int) -> int:
        crc = 0xFF
        for bit_index in range(31, 7, -1):
            bit = (word >> bit_index) & 1
            top = crc & 0x80
            if bit:
                top ^= 0x80
            crc = (crc << 1) & 0xFF
            if top:
                crc ^= 0x1D
        return (~crc) & 0xFF

    def _transfer(self, command: int) -> int:
        tx = command.to_bytes(4, "big")
        rx = bytes(self.spi.xfer2(list(tx)))
        time.sleep(0.000010)
        return int.from_bytes(rx, "big")

    def _data(self, response: int, check_status: bool = True) -> int:
        if self._crc(response) != (response & 0xFF):
            self.crc_errors += 1
        if check_status and ((response >> 24) & 0x03) != 0x01:
            self.status_errors += 1
        value = (response >> 8) & 0xFFFF
        return value - 65536 if value & 0x8000 else value

    def read(self) -> tuple[dict[str, tuple[float, float, float]],
                            dict[str, tuple[int, int, int]]]:
        self._transfer(self.COMMANDS["acc_x"])
        ax = self._data(self._transfer(self.COMMANDS["acc_y"]))
        ay = self._data(self._transfer(self.COMMANDS["acc_z"]))
        az = self._data(self._transfer(self.COMMANDS["ang_x"]))
        gx = self._data(self._transfer(self.COMMANDS["ang_y"]))
        gy = self._data(self._transfer(self.COMMANDS["ang_z"]))
        gz = self._data(self._transfer(self.COMMANDS["acc_x"]))
        raw_a, raw_angle = (ax, ay, az), (gx, gy, gz)
        return {
            "scl3300_accel": tuple(v / self.accel_scale for v in raw_a),
            "scl3300_angle": tuple(v / self.angle_scale for v in raw_angle),
        }, {"scl3300_accel": raw_a, "scl3300_angle": raw_angle}

    def close(self) -> None:
        self.spi.close()

    def metadata(self) -> dict[str, object]:
        return {"format": "little-endian 6 x int16", "channels":
                ["ax", "ay", "az", "angle_x", "angle_y", "angle_z"],
                "mode": self.mode, "accel_lsb_per_g": self.accel_scale,
                "angle_lsb_per_degree": self.angle_scale,
                "physical_bandwidth_hz": {1: 40, 2: 70, 3: 10, 4: 10}[self.mode],
                "host_poll_rate_hz": 1000}


@dataclass
class WorkerStats:
    samples: int = 0
    errors: int = 0
    missed_deadlines: int = 0
    logged_samples: dict[str, int] = field(default_factory=dict)


class AcquisitionWorker(threading.Thread):
    def __init__(self, device: object, sample_rate: float, start_time: float,
                 start_event: threading.Event, stop_event: threading.Event,
                 buffers: dict[str, collections.deque], lock: threading.Lock,
                 log_streams: dict[str, BinaryIO],
                 timestamp_streams: dict[str, BinaryIO],
                 logged_sources: set[str]):
        super().__init__(name=f"acquire-{device.name}", daemon=True)
        self.device = device
        self.period = 1.0 / sample_rate
        self.start_time = start_time
        self.start_event = start_event
        self.stop_event = stop_event
        self.buffers = buffers
        self.lock = lock
        self.log_streams = log_streams
        self.timestamp_streams = timestamp_streams
        self.logged_sources = logged_sources
        self.stats = WorkerStats(logged_samples={source: 0 for source in device.sources})
        self.last_error = ""

    def run(self) -> None:
        self.start_event.wait()
        deadline = time.perf_counter()
        try:
            while not self.stop_event.is_set():
                try:
                    values, raw_values = self.device.read()
                    timestamp = time.perf_counter() - self.start_time
                    with self.lock:
                        for source, xyz in values.items():
                            self.buffers[source].append((timestamp, *xyz))
                        sources_to_log = tuple(
                            source for source in values
                            if source in self.logged_sources
                        )
                    for source in sources_to_log:
                        self.log_streams[source].write(struct.pack(
                            SOURCE_BINARY_FORMATS[source], *raw_values[source]
                        ))
                        self.timestamp_streams[source].write(struct.pack(
                            "<q", int(round(timestamp * 1_000_000_000.0))
                        ))
                        self.stats.logged_samples[source] += 1
                    self.stats.samples += 1
                except OSError as exc:
                    self.stats.errors += 1
                    self.last_error = str(exc)

                deadline += self.period
                delay = deadline - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -self.period:
                    self.stats.missed_deadlines += max(1, int(-delay / self.period))
                    deadline = time.perf_counter()
        finally:
            for source in self.device.sources:
                if source in self.log_streams:
                    self.log_streams[source].flush()
                    self.timestamp_streams[source].flush()


def get_bool(section: configparser.SectionProxy, key: str, default: bool) -> bool:
    return section.getboolean(key, fallback=default)


def load_settings(path: Path | None) -> configparser.ConfigParser:
    config = configparser.ConfigParser()
    config.read_dict({
        "logger": {"sample_rate_hz": "1000", "window_seconds": "10",
                   "plot_hz": "20", "scale_hz": "3", "spectrogram_hz": "2",
                   "fft_samples": "256", "fft_overlap_percent": "75",
                   "log_enabled": "true", "output_directory": "recordings",
                   "log_scope": "displayed", "display_rows": "4",
                   "section_a_source": "mpu_accel", "section_a_view": "time",
                   "section_b_source": "mpu_gyro", "section_b_view": "time",
                   "section_c_source": "adxl355_accel",
                   "section_c_view": "time",
                   "section_d_source": "scl3300_accel",
                   "section_d_view": "time"},
        "mpu6050": {"enabled": "true", "bus": "1", "address": "0x68",
                    "accel_range": "2", "gyro_range": "250"},
        "lsm6dso": {"enabled": "true", "bus": "2", "address": "0x6B",
                    "accel_range": "2", "gyro_range": "250"},
        "adxl355": {"enabled": "true", "spi_bus": "0", "spi_device": "0",
                    "spi_speed_hz": "5000000", "accel_range": "2"},
        "scl3300": {"enabled": "true", "spi_bus": "1", "spi_device": "0",
                    "spi_speed_hz": "2000000", "mode": "1"},
    })
    if path is not None:
        if not path.is_file():
            raise FileNotFoundError(f"configuration file not found: {path}")
        config.read(path, encoding="utf-8")
    return config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        help="explicitly load an INI configuration file")
    parser.add_argument("--no-log", action="store_true", help="disable binary logging")
    parser.add_argument("--no-plot", action="store_true", help="run without the GUI")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_settings(args.config)
    logger = config["logger"]
    sample_rate = logger.getfloat("sample_rate_hz")
    window_seconds = logger.getfloat("window_seconds")
    display_rows = logger.getint("display_rows", fallback=4)
    if display_rows not in (2, 3, 4):
        raise ValueError("display_rows must be 2, 3, or 4")
    row_names = tuple("abcd"[:display_rows])
    log_enabled = get_bool(logger, "log_enabled", True) and not args.no_log
    log_scope = logger.get("log_scope", "displayed").strip().lower()
    if log_scope not in ("displayed", "all"):
        raise ValueError("log_scope must be displayed or all")
    output_root = Path(logger.get("output_directory", "recordings"))
    session_dir = output_root / time.strftime("motion_%Y%m%d_%H%M%S")
    if log_enabled:
        session_dir.mkdir(parents=True, exist_ok=True)

    devices: list[object] = []
    streams: dict[str, BinaryIO] = {}
    timestamp_streams: dict[str, BinaryIO] = {}
    require_all = get_bool(logger, "require_all_sensors", False)

    def add_sensor(label: str, factory: Callable[[], object]) -> None:
        try:
            device = factory()
        except (OSError, RuntimeError) as exc:
            message = f"Skipping {label}: {exc}"
            if require_all:
                raise RuntimeError(message) from exc
            print(f"WARNING: {message}")
            return
        devices.append(device)
        print(f"Detected {label}")

    try:
        section = config["mpu6050"]
        if get_bool(section, "enabled", True):
            add_sensor(
                f"MPU-6050 on I2C bus {section.getint('bus')}",
                lambda: MPU6050(section.getint("bus"), int(section["address"], 0),
                                section.getint("accel_range"),
                                section.getint("gyro_range")),
            )
        section = config["lsm6dso"]
        if get_bool(section, "enabled", True):
            add_sensor(
                f"LSM6DSO on I2C bus {section.getint('bus')}",
                lambda: LSM6DSO(section.getint("bus"), int(section["address"], 0),
                                section.getint("accel_range"),
                                section.getint("gyro_range")),
            )
        section = config["adxl355"]
        if get_bool(section, "enabled", True):
            add_sensor(
                f"ADXL355 on SPI{section.getint('spi_bus')}."
                f"{section.getint('spi_device')}",
                lambda: ADXL355(section.getint("spi_bus"),
                                section.getint("spi_device"),
                                section.getint("spi_speed_hz"),
                                section.getint("accel_range")),
            )
        section = config["scl3300"]
        if get_bool(section, "enabled", True):
            add_sensor(
                f"SCL3300 on SPI{section.getint('spi_bus')}."
                f"{section.getint('spi_device')}",
                lambda: SCL3300(section.getint("spi_bus"),
                                section.getint("spi_device"),
                                section.getint("spi_speed_hz"),
                                section.getint("mode")),
            )
    except BaseException:
        for device in devices:
            device.close()
        raise

    available_sources = {source for device in devices for source in device.sources}
    if not available_sources:
        raise RuntimeError("No sensors are enabled")

    default_sources = {"a": "mpu_accel", "b": "mpu_gyro",
                       "c": "adxl355_accel", "d": "scl3300_accel"}
    view = {
        row_name: {
            "source": logger.get(f"section_{row_name}_source",
                                 default_sources[row_name]).strip().lower(),
            "mode": logger.get(f"section_{row_name}_view", "time").strip().lower(),
        }
        for row_name in row_names
    }
    for row in view.values():
        if row["source"] != "none" and row["source"] not in available_sources:
            row["source"] = sorted(available_sources)[0]
        if row["mode"] not in ("time", "spectrogram"):
            row["mode"] = "time"

    max_points = max(2000, int(window_seconds * sample_rate * 1.25))
    buffers = {name: collections.deque(maxlen=max_points) for name in SOURCE_LABELS}
    lock = threading.Lock()
    stop_event = threading.Event()
    start_event = threading.Event()
    start_time = time.perf_counter()
    logged_sources = (
        (set(available_sources) if log_scope == "all" else
         {row["source"] for row in view.values() if row["source"] != "none"})
        if log_enabled else set()
    )

    if log_enabled:
        for source in sorted(available_sources):
            streams[source] = (session_dir / f"{source}.bin").open(
                "wb", buffering=1024 * 1024
            )
            timestamp_streams[source] = (
                session_dir / f"{source}_time.bin"
            ).open("wb", buffering=1024 * 1024)

    workers = [
        AcquisitionWorker(device, sample_rate, start_time, start_event, stop_event,
                          buffers, lock, streams, timestamp_streams,
                          logged_sources)
        for device in devices
    ]

    def request_stop(*_unused: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    for worker in workers:
        worker.start()
    start_time = time.perf_counter()
    for worker in workers:
        worker.start_time = start_time
    start_event.set()

    try:
        if args.no_plot:
            while not stop_event.wait(0.25):
                pass
        else:
            # Matplotlib normally assigns F to fullscreen. Reserve it for the
            # selected row's frequency-domain display instead.
            plt.rcParams["keymap.fullscreen"] = [
                key for key in plt.rcParams["keymap.fullscreen"]
                if key.lower() != "f"
            ]
            # Preserve short transients as the scrolling window advances. Manual
            # decimation and path simplification can otherwise select a different
            # subset of a peak on successive redraws.
            plt.rcParams["path.simplify"] = False
            fig, axes = plt.subplots(
                display_rows, 3,
                figsize=(14, min(10.0, 3.2 * display_rows + 0.6)),
                sharex="row"
            )
            fig.canvas.manager.set_window_title(f"Multi Motion Logger v{CODE_VERSION}")
            active_row = ["a"]
            artists: dict[str, list[object]] = {name: [] for name in row_names}
            selectors: dict[str, dict[str, RadioButtons]] = {}
            last_scale = {name: 0.0 for name in row_names}
            last_spectrum = {name: 0.0 for name in row_names}
            dimensions = ("X", "Y", "Z")
            colors = ("tab:blue", "tab:orange", "tab:green")

            def spectrum_limit(source: str) -> float:
                if source == "adxl355_accel":
                    return 250.0
                if source.startswith("scl3300"):
                    mode = config["scl3300"].getint("mode")
                    return float({1: 40, 2: 70, 3: 10, 4: 10}[mode])
                if source == "mpu_accel":
                    return 260.0
                if source == "mpu_gyro":
                    return 256.0
                return min(500.0, sample_rate / 2)

            def setup_row(row_name: str) -> None:
                row_index = row_names.index(row_name)
                source = view[row_name]["source"]
                mode = view[row_name]["mode"]
                artists[row_name] = []
                for dimension, color, axis in zip(dimensions, colors, axes[row_index, :]):
                    axis.clear()
                    axis.set_axis_on()
                    prefix = "*" if active_row[0] == row_name else ""
                    if source == "none":
                        axis.set_title(f"{prefix}{row_name.upper()}: None {dimension}")
                        axis.set_axis_off()
                        continue
                    axis.set_title(f"{prefix}{row_name.upper()}: {SOURCE_LABELS[source]} {dimension}")
                    axis.set_xlabel("Time (s)")
                    if mode == "time":
                        axis.set_ylabel(SOURCE_UNITS[source])
                        axis.grid(True, alpha=0.3)
                        artists[row_name].append(axis.plot([], [], color=color, linewidth=1)[0])
                    else:
                        axis.set_ylabel("Frequency (Hz)")
                        image = axis.imshow(
                            np.zeros((2, 2)), origin="lower", aspect="auto",
                            interpolation="nearest", cmap="viridis",
                            extent=(0, window_seconds, 0, spectrum_limit(source))
                        )
                        artists[row_name].append(image)
                fig.canvas.draw_idle()

            def on_key(event: object) -> None:
                key = (event.key or "").lower()
                if key in row_names:
                    old = active_row[0]
                    active_row[0] = key
                    setup_row(old)
                    setup_row(key)
                elif key == "0" or key in tuple(str(number) for number in SOURCE_NAMES):
                    source = "none" if key == "0" else SOURCE_NAMES[int(key)]
                    if source == "none" or source in available_sources:
                        row_name = active_row[0]
                        selectors[row_name]["source"].set_active(
                            source_values.index(source)
                        )
                elif key in ("f", "t"):
                    selectors[active_row[0]]["mode"].set_active(
                        1 if key == "f" else 0
                    )
                elif key in ("q", "escape"):
                    stop_event.set()

            fig.canvas.mpl_connect("key_press_event", on_key)
            for row_name in row_names:
                setup_row(row_name)
            source_choices = [
                ("None", "none"),
                ("MPU Accel", "mpu_accel"),
                ("MPU Gyro", "mpu_gyro"),
                ("LSM Accel", "lsm_accel"),
                ("LSM Gyro", "lsm_gyro"),
                ("ADXL Accel", "adxl355_accel"),
                ("SCL Accel", "scl3300_accel"),
                ("SCL Angle", "scl3300_angle"),
            ]
            source_choices = [choice for choice in source_choices
                              if choice[1] == "none" or
                              choice[1] in available_sources]
            source_labels = [choice[0] for choice in source_choices]
            source_values = [choice[1] for choice in source_choices]

            def select_source(label: str, row_name: str) -> None:
                view[row_name]["source"] = source_values[source_labels.index(label)]
                if log_enabled and log_scope == "displayed":
                    with lock:
                        logged_sources.clear()
                        logged_sources.update(
                            row["source"] for row in view.values()
                            if row["source"] != "none"
                        )
                setup_row(row_name)

            def select_mode(label: str, row_name: str) -> None:
                view[row_name]["mode"] = ("time" if label == "Time"
                                          else "spectrogram")
                setup_row(row_name)

            controls_top = 0.90
            controls_bottom = 0.09
            controls_row_height = ((controls_top - controls_bottom) /
                                   display_rows)
            for row_index, row_name in enumerate(row_names):
                row_bottom = controls_top - (row_index + 1) * controls_row_height
                source_axis = fig.add_axes(
                    (0.805, row_bottom + 0.02, 0.115,
                     controls_row_height - 0.04)
                )
                mode_axis = fig.add_axes(
                    (0.925, row_bottom + (controls_row_height - 0.16) / 2,
                     0.07, 0.16)
                )
                source_axis.set_title(f"Row {row_name.upper()} source", fontsize=9)
                mode_axis.set_title("View", fontsize=9)
                source_radio = RadioButtons(
                    source_axis, source_labels,
                    active=source_values.index(view[row_name]["source"]),
                )
                mode_radio = RadioButtons(
                    mode_axis, ("Time", "Frequency"),
                    active=0 if view[row_name]["mode"] == "time" else 1,
                )
                for label in (*source_radio.labels, *mode_radio.labels):
                    label.set_fontsize(8)
                source_radio.on_clicked(
                    lambda label, row=row_name: select_source(label, row)
                )
                mode_radio.on_clicked(
                    lambda label, row=row_name: select_mode(label, row)
                )
                selectors[row_name] = {"source": source_radio, "mode": mode_radio}

            help_text = fig.text(
                0.4, 0.005,
                f"{'/'.join(name.upper() for name in row_names)} select row | "
                "0 None | 1 MPU-A | 2 MPU-G | 3 LSM-A | 4 LSM-G | 5 ADXL | "
                "6 SCL-A | 7 SCL-angle | F frequency | T time | Q quit",
                ha="center", fontsize=9
            )
            title = fig.suptitle(f"Multi Motion Logger v{CODE_VERSION}")
            fig.subplots_adjust(left=0.06, right=0.78, bottom=0.09,
                                top=0.90, hspace=0.38, wspace=0.28)
            plot_period = 1.0 / max(1.0, logger.getfloat("plot_hz"))
            scale_period = 1.0 / max(0.1, logger.getfloat("scale_hz"))
            spectrum_period = 1.0 / max(0.1, logger.getfloat("spectrogram_hz"))
            fft_samples = logger.getint("fft_samples")
            fft_overlap = logger.getfloat("fft_overlap_percent")
            last_rate_time = time.perf_counter()
            last_rate_counts = {worker.device.name: 0 for worker in workers}
            displayed_rates = {worker.device.name: 0.0 for worker in workers}

            while not stop_event.is_set() and plt.fignum_exists(fig.number):
                now = time.perf_counter()
                for row_index, row_name in enumerate(row_names):
                    source = view[row_name]["source"]
                    if source == "none":
                        continue
                    with lock:
                        points = list(buffers[source])
                    if not points:
                        continue
                    data = np.asarray(points)
                    data = data[data[:, 0] >= data[-1, 0] - window_seconds]
                    left = max(0.0, data[-1, 0] - window_seconds)
                    right = max(window_seconds, data[-1, 0])
                    if view[row_name]["mode"] == "time":
                        rescale = now - last_scale[row_name] >= scale_period
                        for channel, axis, line in zip(
                                range(1, 4), axes[row_index, :], artists[row_name]):
                            line.set_data(data[:, 0], data[:, channel])
                            axis.set_xlim(left, right)
                            if rescale:
                                low, high = np.min(data[:, channel]), np.max(data[:, channel])
                                margin = max(0.001, float(high - low) * 0.1)
                                axis.set_ylim(float(low) - margin, float(high) + margin)
                        if rescale:
                            last_scale[row_name] = now
                    elif now - last_spectrum[row_name] >= spectrum_period:
                        if len(data) < fft_samples:
                            continue
                        elapsed = data[-1, 0] - data[0, 0]
                        effective_rate = ((len(data) - 1) / elapsed
                                          if elapsed > 0 else sample_rate)
                        for channel, axis, image in zip(
                                range(1, 4), axes[row_index, :], artists[row_name]):
                            frequencies, spectrum, hop = make_spectrogram(
                                data[:, channel], effective_rate,
                                fft_samples, fft_overlap
                            )
                            if spectrum.size:
                                limit = spectrum_limit(source)
                                visible = frequencies <= limit
                                frequencies, spectrum = frequencies[visible], spectrum[visible]
                                center_indices = (fft_samples / 2 +
                                                  np.arange(spectrum.shape[1]) * hop)
                                frame_times = np.interp(
                                    center_indices,
                                    np.arange(len(data), dtype=float),
                                    data[:, 0],
                                )
                                image.set_data(spectrum)
                                image.set_extent((frame_times[0], frame_times[-1],
                                                  frequencies[0], frequencies[-1]))
                                low = float(np.percentile(spectrum, 5))
                                high = float(np.percentile(spectrum, 99.5))
                                image.set_clim(low, max(low + 1.0, high))
                                axis.set_xlim(left, right)
                        last_spectrum[row_name] = now

                rate_interval = now - last_rate_time
                if rate_interval >= 0.5:
                    for worker in workers:
                        name = worker.device.name
                        displayed_rates[name] = (
                            worker.stats.samples - last_rate_counts[name]
                        ) / rate_interval
                        last_rate_counts[name] = worker.stats.samples
                    last_rate_time = now
                rates = " | ".join(
                    f"{worker.device.name} {displayed_rates[worker.device.name]:.1f} Hz"
                    for worker in workers
                )
                title.set_text(f"Multi Motion Logger v{CODE_VERSION} — {rates}")
                fig.canvas.draw_idle()
                plt.pause(plot_period)
            stop_event.set()
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=3.0)
        for stream in streams.values():
            stream.close()
        for stream in timestamp_streams.values():
            stream.close()
        for device in devices:
            device.close()

    elapsed = max(0.001, time.perf_counter() - start_time)
    manifest = {
        "program": "multi_motion_logger", "version": CODE_VERSION,
        "common_host_rate_hz": sample_rate,
        "log_scope": log_scope,
        "timestamp_format": "little-endian int64 nanoseconds from common start",
        "start_unix_time_s": time.time() - elapsed,
        "duration_s": elapsed,
        "devices": {
            worker.device.name: {
                **worker.device.metadata(),
                "samples": worker.stats.samples,
                "delivered_rate_hz": worker.stats.samples / elapsed,
                "read_errors": worker.stats.errors,
                "missed_deadlines": worker.stats.missed_deadlines,
                **({"crc_errors": worker.device.crc_errors,
                    "status_errors": worker.device.status_errors}
                   if worker.device.name == "scl3300" else {}),
            } for worker in workers
        },
        "sources": {
            source: {
                "device": SOURCE_DEVICES[source],
                "data_file": f"{source}.bin",
                "timestamp_file": f"{source}_time.bin",
                "data_struct": SOURCE_BINARY_FORMATS[source],
                "data_format": ("little-endian 3 x int32" if
                                SOURCE_BINARY_FORMATS[source] == "<3i" else
                                "little-endian 3 x int16"),
                "channels": ["x", "y", "z"],
                "units": SOURCE_UNITS[source],
                "logged_samples": worker.stats.logged_samples[source],
            }
            for worker in workers for source in worker.device.sources
        },
    }
    if log_enabled:
        (session_dir / "session.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Session saved to {session_dir.resolve()}")
    for name, details in manifest["devices"].items():
        print(f"{name}: {details['samples']} samples, "
              f"{details['delivered_rate_hz']:.1f} Hz, "
              f"{details['read_errors']} read errors")
    if log_enabled:
        for source, details in manifest["sources"].items():
            print(f"  {source}: {details['logged_samples']} logged samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
