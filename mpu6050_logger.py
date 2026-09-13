#!/usr/bin/env python3
"""Log MPU-6050 acceleration and angular velocity at 1 kHz over I2C.

The live display is decoupled from acquisition. Binary output contains six
little-endian signed int16 values per sample: ax, ay, az, gx, gy, gz.
"""

from __future__ import annotations

import argparse
import collections
import json
import signal
import struct
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from smbus2 import SMBus, i2c_msg


# MPU-6050 registers
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
FIFO_EN = 0x23
INT_STATUS = 0x3A
USER_CTRL = 0x6A
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
FIFO_COUNT_H = 0x72
FIFO_R_W = 0x74
WHO_AM_I = 0x75

PACKET_BYTES = 12  # accel XYZ followed by gyro XYZ; temperature is not in FIFO
SAMPLE_RATE_HZ = 1000.0
CODE_VERSION = "1.1.0"

ACCEL_SCALE = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_SCALE = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}


def signed16(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


class MPU6050:
    def __init__(self, bus_number: int, address: int):
        self.address = address
        self.bus = SMBus(bus_number)

    def close(self) -> None:
        self.bus.close()

    def read_register(self, register: int) -> int:
        return self.bus.read_byte_data(self.address, register)

    def write_register(self, register: int, value: int) -> None:
        self.bus.write_byte_data(self.address, register, value)

    def configure(self, accel_range: int, gyro_range: int) -> None:
        identity = self.read_register(WHO_AM_I)
        if identity not in (0x68, 0x69):
            raise RuntimeError(
                f"MPU-6050 not found at 0x{self.address:02X} "
                f"(WHO_AM_I=0x{identity:02X}, expected 0x68/0x69)."
            )

        self.write_register(PWR_MGMT_1, 0x80)  # device reset
        time.sleep(0.10)
        self.write_register(PWR_MGMT_1, 0x01)  # wake; PLL with X gyro reference
        self.write_register(PWR_MGMT_2, 0x00)  # enable every axis
        time.sleep(0.05)

        # DLPF_CFG=0: 260 Hz accel BW, 256 Hz gyro BW. Internal gyro rate is
        # 8 kHz; divider 7 gives a synchronized 1 kHz FIFO rate. The accel
        # itself only produces unique data at up to 1 kHz.
        self.write_register(CONFIG, 0x00)
        self.write_register(SMPLRT_DIV, 7)
        accel_bits = {2: 0, 4: 1, 8: 2, 16: 3}[accel_range]
        gyro_bits = {250: 0, 500: 1, 1000: 2, 2000: 3}[gyro_range]
        self.write_register(GYRO_CONFIG, gyro_bits << 3)
        self.write_register(ACCEL_CONFIG, accel_bits << 3)

        self.write_register(FIFO_EN, 0x00)
        self.write_register(USER_CTRL, 0x04)  # reset FIFO
        time.sleep(0.01)
        self.write_register(USER_CTRL, 0x40)  # enable FIFO
        self.write_register(FIFO_EN, 0x78)    # accel + X/Y/Z gyro

    def fifo_count(self) -> int:
        raw = self.bus.read_i2c_block_data(self.address, FIFO_COUNT_H, 2)
        return (raw[0] << 8) | raw[1]

    def fifo_overflowed(self) -> bool:
        return bool(self.read_register(INT_STATUS) & 0x10)

    def reset_fifo(self) -> None:
        self.write_register(FIFO_EN, 0x00)
        self.write_register(USER_CTRL, 0x04)
        self.write_register(USER_CTRL, 0x40)
        self.write_register(FIFO_EN, 0x78)

    def read_fifo(self, byte_count: int) -> list[int]:
        """Read many FIFO bytes in one I2C transaction."""
        pointer = i2c_msg.write(self.address, [FIFO_R_W])
        data = i2c_msg.read(self.address, byte_count)
        self.bus.i2c_rdwr(pointer, data)
        return list(data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--no-log", action="store_true",
                        help="run without creating binary or metadata files")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x68,
                        help="I2C address, usually 0x68; use 0x69 if AD0 is high")
    parser.add_argument("--accel-range", type=int, choices=(2, 4, 8, 16), default=2,
                        help="accelerometer full scale in g (default: 2, most sensitive)")
    parser.add_argument("--gyro-range", type=int, choices=(250, 500, 1000, 2000), default=250,
                        help="gyroscope full scale in degrees/s (default: 250)")
    parser.add_argument("--window", type=float, default=5.0,
                        help="seconds visible in the plot")
    parser.add_argument("--plot-hz", type=float, default=20.0,
                        help="screen refresh rate; does not change logging rate")
    parser.add_argument("--scale-hz", type=float, default=3.0,
                        help="y-axis rescale rate (default: 3 times/s)")
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output or Path(time.strftime("mpu6050_%Y%m%d_%H%M%S.bin"))
    if not args.no_log:
        output.parent.mkdir(parents=True, exist_ok=True)

    recent = collections.deque(maxlen=max(5000, int(args.window * 1500)))
    lock = threading.Lock()
    stop = threading.Event()
    errors: list[BaseException] = []
    stats = {"samples": 0, "overflows": 0, "start": 0.0}

    def request_stop(*_unused: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def acquire() -> None:
        sensor: MPU6050 | None = None
        try:
            sensor = MPU6050(args.bus, args.address)
            sensor.configure(args.accel_range, args.gyro_range)
            accel_scale = ACCEL_SCALE[args.accel_range]
            gyro_scale = GYRO_SCALE[args.gyro_range]
            start_mono = time.perf_counter()
            start_unix = time.time()
            stats["start"] = start_mono
            sample_number = 0

            stream = None if args.no_log else output.open("wb", buffering=1024 * 1024)
            try:
                if stream is not None:
                    metadata = {
                        "format": "headerless little-endian signed int16",
                        "channels": ["ax", "ay", "az", "gx", "gy", "gz"],
                        "values_per_sample": 6,
                        "bytes_per_sample": PACKET_BYTES,
                        "sample_rate_hz": SAMPLE_RATE_HZ,
                        "start_unix_time_s": start_unix,
                        "accel_range_g": args.accel_range,
                        "accel_lsb_per_g": accel_scale,
                        "gyro_range_dps": args.gyro_range,
                        "gyro_lsb_per_dps": gyro_scale,
                    }
                    metadata_path = output.with_suffix(output.suffix + ".json")
                    metadata_path.write_text(
                        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
                    )

                pending = bytearray()
                last_flush = start_mono

                while not stop.is_set():
                    count = sensor.fifo_count()
                    if count >= 1024 or sensor.fifo_overflowed():
                        stats["overflows"] += 1
                        sensor.reset_fifo()
                        continue

                    packets = count // PACKET_BYTES
                    if packets == 0:
                        time.sleep(0)
                        continue

                    # Keep transactions comfortably below the 1024-byte FIFO.
                    packets = min(packets, 64)
                    raw = sensor.read_fifo(packets * PACKET_BYTES)
                    batch_end = time.perf_counter()
                    first_time = batch_end - (packets - 1) / SAMPLE_RATE_HZ
                    new_points = []

                    for index in range(packets):
                        p = raw[index * PACKET_BYTES:(index + 1) * PACKET_BYTES]
                        raw_ax = signed16(p[0], p[1])
                        raw_ay = signed16(p[2], p[3])
                        raw_az = signed16(p[4], p[5])
                        raw_gx = signed16(p[6], p[7])
                        raw_gy = signed16(p[8], p[9])
                        raw_gz = signed16(p[10], p[11])
                        ax, ay, az = (raw_ax / accel_scale, raw_ay / accel_scale,
                                      raw_az / accel_scale)
                        gx, gy, gz = (raw_gx / gyro_scale, raw_gy / gyro_scale,
                                      raw_gz / gyro_scale)
                        elapsed = max(
                            0.0, first_time + index / SAMPLE_RATE_HZ - start_mono
                        )
                        if stream is not None:
                            pending.extend(struct.pack(
                                "<6h", raw_ax, raw_ay, raw_az,
                                raw_gx, raw_gy, raw_gz
                            ))
                        new_points.append((elapsed, ax, ay, az, gx, gy, gz))
                        sample_number += 1

                    with lock:
                        recent.extend(new_points)
                    stats["samples"] = sample_number

                    now = time.perf_counter()
                    if stream is not None and (len(pending) >= 65536 or
                                               now - last_flush >= 1.0):
                        stream.write(pending)
                        pending.clear()
                        stream.flush()
                        last_flush = now

                if stream is not None and pending:
                    stream.write(pending)
            finally:
                if stream is not None:
                    stream.close()
        except BaseException as exc:
            errors.append(exc)
            stop.set()
        finally:
            if sensor is not None:
                sensor.close()

    worker = threading.Thread(target=acquire, name="MPU6050-acquisition", daemon=True)
    worker.start()

    try:
        if args.no_plot:
            while worker.is_alive():
                worker.join(timeout=0.25)
        else:
            plt.ion()
            fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
            fig.canvas.manager.set_window_title(f"MPU-6050 Logger v{CODE_VERSION}")
            channel_names = ("Accel X", "Accel Y", "Accel Z",
                             "Gyro X", "Gyro Y", "Gyro Z")
            channel_units = ("g", "g", "g",
                             "degrees/s", "degrees/s", "degrees/s")
            colors = ("tab:blue", "tab:orange", "tab:green",
                      "tab:blue", "tab:orange", "tab:green")
            flat_axes = list(axes.flat)
            lines = []
            for axis, name, unit, color in zip(
                    flat_axes, channel_names, channel_units, colors):
                lines.append(axis.plot([], [], linewidth=1, color=color)[0])
                axis.set_title(name)
                axis.set_ylabel(unit)
                axis.grid(True, alpha=0.3)
            for axis in axes[1, :]:
                axis.set_xlabel("Time (s)")
            fig.suptitle(
                f"MPU-6050 Logger v{CODE_VERSION} — close window to stop"
            )
            fig.tight_layout(rect=(0, 0, 1, 0.95))
            last_scale_update = 0.0

            while not stop.is_set() and plt.fignum_exists(fig.number):
                with lock:
                    points = list(recent)
                if points:
                    data = np.asarray(points)
                    data = data[data[:, 0] >= data[-1, 0] - args.window]
                    stride = max(1, len(data) // 4000)
                    shown = data[::stride]
                    left = max(0.0, data[-1, 0] - args.window)
                    right = max(args.window, data[-1, 0])
                    now = time.perf_counter()
                    update_scale = (
                        last_scale_update == 0.0
                        or now - last_scale_update >= 1.0 / max(0.1, args.scale_hz)
                    )
                    for channel, (axis, line) in enumerate(
                            zip(flat_axes, lines), start=1):
                        line.set_data(shown[:, 0], shown[:, channel])
                        axis.set_xlim(left, right)
                        if update_scale:
                            low = float(np.min(data[:, channel]))
                            high = float(np.max(data[:, channel]))
                            margin = max(0.02, (high - low) * 0.1)
                            axis.set_ylim(low - margin, high + margin)
                    if update_scale:
                        last_scale_update = now
                fig.canvas.draw_idle()
                plt.pause(max(0.001, 1.0 / args.plot_hz))
            stop.set()
    finally:
        stop.set()
        worker.join(timeout=3.0)

    if errors:
        raise errors[0]
    elapsed = max(0.001, time.perf_counter() - stats["start"])
    rate = stats["samples"] / elapsed
    if args.no_log:
        print(f"Processed {stats['samples']} samples; logging was disabled")
    else:
        print(f"Saved {stats['samples']} samples to {output.resolve()}")
        print(f"Metadata: {output.with_suffix(output.suffix + '.json').resolve()}")
    print(f"Average delivered rate: {rate:.1f} samples/s")
    print(f"FIFO overflows (data gaps): {stats['overflows']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
