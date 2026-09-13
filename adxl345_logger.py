#!/usr/bin/env python3
"""High-rate ADXL345 SPI logger with a live three-axis plot.

Output is a tab-delimited text file with one sample per row:
sample, elapsed_s, unix_time_s, ax_g, ay_g, az_g
"""

from __future__ import annotations

import argparse
import collections
import signal
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import spidev


# ADXL345 registers
DEVID = 0x00
BW_RATE = 0x2C
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
DATAX0 = 0x32
FIFO_CTL = 0x38
FIFO_STATUS = 0x39

READ = 0x80
MULTI_BYTE = 0x40
EXPECTED_DEVICE_ID = 0xE5
G_PER_LSB_FULL_RES = 0.0039


class ADXL345:
    """Minimal ADXL345 SPI driver for Raspberry Pi spidev."""

    def __init__(self, bus: int = 0, device: int = 0, speed_hz: int = 5_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0b11  # ADXL345 supports SPI modes 0 and 3
        self.spi.bits_per_word = 8

    def close(self) -> None:
        self.spi.close()

    def read_register(self, address: int) -> int:
        return self.spi.xfer2([READ | address, 0x00])[1]

    def write_register(self, address: int, value: int) -> None:
        self.spi.xfer2([address & 0x3F, value & 0xFF])

    def configure(self, rate_code: int = 0x0F, range_g: int = 16) -> None:
        device_id = self.read_register(DEVID)
        if device_id != EXPECTED_DEVICE_ID:
            raise RuntimeError(
                f"ADXL345 not found (DEVID=0x{device_id:02X}, expected 0xE5). "
                "Check power, SPI wiring, CS, and that SPI is enabled."
            )

        range_bits = {2: 0, 4: 1, 8: 2, 16: 3}[range_g]
        self.write_register(POWER_CTL, 0x00)             # standby while configuring
        self.write_register(DATA_FORMAT, 0x08 | range_bits)  # full resolution, 4-wire SPI
        self.write_register(BW_RATE, rate_code)          # 0x0F = 3200 Hz ODR
        self.write_register(FIFO_CTL, 0x80)              # stream mode, 32-sample FIFO
        self.write_register(POWER_CTL, 0x08)             # measurement mode
        time.sleep(0.02)

    def fifo_count(self) -> int:
        return self.read_register(FIFO_STATUS) & 0x3F

    def read_xyz_g(self) -> tuple[float, float, float]:
        raw = self.spi.xfer2([READ | MULTI_BYTE | DATAX0, 0, 0, 0, 0, 0, 0])[1:]
        values = []
        for low, high in zip(raw[0::2], raw[1::2]):
            value = (high << 8) | low
            if value & 0x8000:
                value -= 0x10000
            values.append(value * G_PER_LSB_FULL_RES)
        return values[0], values[1], values[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="output file (default: adxl345_YYYYMMDD_HHMMSS.txt)")
    parser.add_argument("--bus", type=int, default=0, help="SPI bus (default: 0)")
    parser.add_argument("--device", type=int, default=0, help="SPI device/CE (default: 0)")
    parser.add_argument("--spi-speed", type=int, default=5_000_000, help="SPI clock in Hz")
    parser.add_argument("--range", type=int, choices=(2, 4, 8, 16), default=16,
                        dest="range_g", help="measurement range in g (default: 16)")
    parser.add_argument("--window", type=float, default=5.0,
                        help="seconds displayed in live plot (default: 5)")
    parser.add_argument("--plot-hz", type=float, default=20.0,
                        help="plot refresh rate, not sensor rate (default: 20)")
    parser.add_argument("--no-plot", action="store_true",
                        help="log without opening the graph")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output or Path(time.strftime("adxl345_%Y%m%d_%H%M%S.txt"))
    output.parent.mkdir(parents=True, exist_ok=True)

    # Tuples are (elapsed_s, ax_g, ay_g, az_g). A deque bounds memory usage.
    recent = collections.deque(maxlen=max(10_000, int(args.window * 5000)))
    lock = threading.Lock()
    stop = threading.Event()
    error: list[BaseException] = []
    stats = {"samples": 0, "start": 0.0}

    def request_stop(*_unused: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def acquire() -> None:
        sensor = ADXL345(args.bus, args.device, args.spi_speed)
        try:
            sensor.configure(range_g=args.range_g)
            start_mono = time.perf_counter()
            start_unix = time.time()
            stats["start"] = start_mono
            sample_number = 0
            with output.open("w", encoding="ascii", buffering=1024 * 1024) as stream:
                stream.write("sample\telapsed_s\tunix_time_s\tax_g\tay_g\taz_g\n")
                pending: list[str] = []
                last_flush = start_mono

                while not stop.is_set():
                    count = sensor.fifo_count()
                    if count == 0:
                        time.sleep(0)  # yield without imposing a millisecond-scale delay
                        continue

                    batch_end = time.perf_counter()
                    # Approximate acquisition times within the FIFO batch at 3200 Hz.
                    first_time = batch_end - (count - 1) / 3200.0
                    new_points = []
                    for i in range(count):
                        ax, ay, az = sensor.read_xyz_g()
                        elapsed = max(0.0, first_time + i / 3200.0 - start_mono)
                        unix_time = start_unix + elapsed
                        pending.append(
                            f"{sample_number}\t{elapsed:.9f}\t{unix_time:.6f}\t"
                            f"{ax:.6f}\t{ay:.6f}\t{az:.6f}\n"
                        )
                        new_points.append((elapsed, ax, ay, az))
                        sample_number += 1

                    with lock:
                        recent.extend(new_points)
                    stats["samples"] = sample_number

                    now = time.perf_counter()
                    if len(pending) >= 4096 or now - last_flush >= 1.0:
                        stream.writelines(pending)
                        pending.clear()
                        stream.flush()
                        last_flush = now

                if pending:
                    stream.writelines(pending)
        except BaseException as exc:
            error.append(exc)
            stop.set()
        finally:
            sensor.close()

    worker = threading.Thread(target=acquire, name="ADXL345-acquisition", daemon=True)
    worker.start()

    try:
        if args.no_plot:
            while worker.is_alive():
                worker.join(timeout=0.25)
        else:
            plt.ion()
            fig, ax = plt.subplots(figsize=(10, 5))
            lines = [ax.plot([], [], label=label, linewidth=1)[0] for label in ("X", "Y", "Z")]
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Acceleration (g)")
            ax.set_title("ADXL345 live acceleration — close window to stop")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")
            fig.tight_layout()

            while not stop.is_set() and plt.fignum_exists(fig.number):
                with lock:
                    points = list(recent)
                if points:
                    data = np.asarray(points)
                    cutoff = data[-1, 0] - args.window
                    data = data[data[:, 0] >= cutoff]
                    # Limit rendering work; every sample is still written to disk.
                    stride = max(1, len(data) // 4000)
                    shown = data[::stride]
                    for axis_index, line in enumerate(lines, start=1):
                        line.set_data(shown[:, 0], shown[:, axis_index])
                    ax.set_xlim(max(0.0, data[-1, 0] - args.window),
                                max(args.window, data[-1, 0]))
                    ymin = float(np.min(data[:, 1:4]))
                    ymax = float(np.max(data[:, 1:4]))
                    margin = max(0.05, (ymax - ymin) * 0.1)
                    ax.set_ylim(ymin - margin, ymax + margin)
                fig.canvas.draw_idle()
                plt.pause(max(0.001, 1.0 / args.plot_hz))
            stop.set()
    finally:
        stop.set()
        worker.join(timeout=3.0)

    if error:
        raise error[0]
    elapsed = max(0.001, time.perf_counter() - stats["start"]) if stats["start"] else 0.001
    print(f"Saved {stats['samples']} samples to {output.resolve()}")
    print(f"Average delivered sample rate: {stats['samples'] / elapsed:.1f} samples/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
