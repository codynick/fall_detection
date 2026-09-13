# Raspberry Pi 5 + ADXL345 high-rate movement logger

This project reads an ADXL345 over **4-wire SPI**, displays X/Y/Z acceleration in
real time, and saves every sample as a tab-delimited text row that MATLAB can read.
The sensor is configured for its maximum 3200 Hz output-data rate and ±16 g full
resolution. The live plot is deliberately refreshed more slowly than acquisition.

## 1. Wire the sensor (Pi powered off)

Use a breakout board that accepts **3.3 V logic**. Do not put 5 V on a bare ADXL345.

| ADXL345 pin | Raspberry Pi 5 pin | GPIO/function |
|---|---:|---|
| `VCC` or `3V3` | physical pin 1 | 3.3 V |
| `GND` | physical pin 6 | Ground |
| `CS` | physical pin 24 | GPIO8 / SPI0 CE0 |
| `SDO` | physical pin 21 | GPIO9 / SPI0 MISO |
| `SDA` / `SDI` | physical pin 19 | GPIO10 / SPI0 MOSI |
| `SCL` / `SCLK` | physical pin 23 | GPIO11 / SPI0 SCLK |

Keep the wires short, especially at high sample rates. `INT1` and `INT2` are not
needed by this polling/FIFO version.

## 2. Enable SPI and install software

In a terminal on Raspberry Pi OS:

```bash
sudo raspi-config
```

Choose **Interface Options > SPI > Yes**, finish, and reboot:

```bash
sudo reboot
```

After reconnecting, confirm that `/dev/spidev0.0` exists:

```bash
ls -l /dev/spidev0.0
```

Install the packaged scientific libraries and Python SPI binding:

```bash
sudo apt update
sudo apt install -y python3-numpy python3-matplotlib python3-spidev
```

Copy `adxl345_logger.py` to the Pi, enter its directory, and run:

```bash
python3 adxl345_logger.py
```

Close the graph or press **Ctrl+C** to finish cleanly. A file such as
`adxl345_20260912_153000.txt` will be created in the current directory. Choose a
name or location with:

```bash
python3 adxl345_logger.py --output test_01.txt
```

For a headless/SSH session, disable the graph:

```bash
python3 adxl345_logger.py --no-plot --output test_01.txt
```

If access to SPI is denied, add the current user to the `spi` group, log out, and
log in again:

```bash
sudo usermod -aG spi "$USER"
```

## 3. Read the result in MATLAB

The first row contains column names; each following row is one measurement.

```matlab
T = readtable("test_01.txt", "FileType", "text", "Delimiter", "\t");

plot(T.elapsed_s, [T.ax_g T.ay_g T.az_g]);
xlabel("Time (s)");
ylabel("Acceleration (g)");
legend("X", "Y", "Z");
grid on;
```

The columns are sample number, elapsed monotonic time, Unix time, and X/Y/Z in g.
Multiply acceleration columns by `9.80665` for m/s².

## Rate and reliability notes

- `3200 Hz` is the ADXL345's configured output-data rate, not a guarantee that a
  non-real-time Linux/Python process will save every sample. At the end, the program
  prints its average delivered rate. The sensor FIFO absorbs short scheduling gaps.
- Use a fast local microSD/NVMe destination rather than a network drive. The logger
  buffers file writes to reduce pauses.
- If the reported rate is substantially below 3200 samples/s, try `--no-plot`, close
  other programs, shorten the wires, or reduce the sensor rate in the source from
  `0x0F` (3200 Hz) to `0x0E` (1600 Hz). A microcontroller or a kernel/interrupt-based
  acquisition system is a better choice when lossless, precisely timed 3200 Hz data
  is mandatory.
- The per-row timestamps inside each FIFO batch are reconstructed at 1/3200-second
  spacing. They are suitable for plotting and ordinary motion analysis, but they are
  not hardware timestamps.

Useful options:

```bash
python3 adxl345_logger.py --help
python3 adxl345_logger.py --range 8 --window 10 --plot-hz 15
```
