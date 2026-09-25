# Four-sensor motion logger for Raspberry Pi 5

`multi_motion_logger.py` acquires the MPU-6050, LSM6DSO, ADXL355 and SCL3300
simultaneously. Each sensor has its own bus/interface. The program logs compact raw
binary data while displaying two to four independently selectable XYZ views. This
is a prototype driver and must be verified on the actual boards before relying on
its measurements.

## Important rate interpretation

The default 1000 Hz setting is a common **host polling schedule**, not a statement
that all four sensors have a 500 Hz usable measurement bandwidth:

- MPU-6050 is configured for 1000 samples/s.
- ADXL355 is configured for 1000 samples/s with its 250 Hz low-pass setting.
- LSM6DSO runs natively at 1666 samples/s and is observed on the 1000 Hz host grid.
  This first version does not resample its native stream.
- SCL3300 is polled at 1000 Hz, but its physical bandwidth is approximately 40 Hz
  in mode 1, 70 Hz in mode 2, and 10 Hz in modes 3/4. Repeated/strongly correlated
  readings are therefore expected.

The live title reports each worker's recent delivered sample rate. Linux and Python
are not hard real-time systems, so occasional timing jitter or missed deadlines is
normal. For rigorous synchronization or guaranteed lossless high-rate capture, the
next version should use sensor FIFOs/data-ready signals or a microcontroller.

## Exact wiring map

Power the Pi off before wiring. All signals are 3.3 V; never apply 5 V to a Pi GPIO.
Use short wires and a common ground. Confirm the header pin numbering on the exact
breakout board before applying power.

### MPU-6050 (I2C1)

| MPU-6050 board | Pi 5 physical pin | GPIO/function |
|---|---:|---|
| VCC | 1 | 3.3 V |
| GND | 6 | Ground |
| SDA | 3 | GPIO2 / SDA1 |
| SCL | 5 | GPIO3 / SCL1 |
| AD0 | Ground | Address 0x68 |
| INT | Not connected | — |

### SparkFun LSM6DSO SEN-18020 (I2C2)

| LSM6DSO board | Pi 5 physical pin | GPIO/function |
|---|---:|---|
| 3V3/VCC | 17 | 3.3 V |
| GND | 9 | Ground |
| SDA | 7 | GPIO4 / SDA2 |
| SCL | 29 | GPIO5 / SCL2 |
| INT1/INT2 | Not connected | — |

The default address is 0x6B. Leave the address-selection connection in its default
state. The breakout's Qwiic connector may be used instead of its SDA/SCL header,
but not both.

### Analog Devices EVAL-ADXL355Z (SPI0)

| EVAL-ADXL355Z | Pi 5 physical pin | GPIO/function |
|---|---:|---|
| P1-1 VDDIO | 1 | 3.3 V |
| P1-3 VDD | 17 | 3.3 V |
| P1-5 GND | 14 | Ground |
| P2-2 CS/SCL | 24 | GPIO8 / SPI0 CE0 |
| P2-4 SCLK/VSSIO | 23 | GPIO11 / SPI0 SCLK |
| P2-5 MISO/SDA | 21 | GPIO9 / SPI0 MISO |
| P2-6 MOSI/SDA | 19 | GPIO10 / SPI0 MOSI |
| P2-1 and P2-3 | Not connected | V1P8 outputs—do not power |

### Murata SCL3300-D01-PCB (SPI1)

| SCL3300 PCB | Pi 5 physical pin | GPIO/function |
|---|---:|---|
| J2-1 AVSS | 20 | Ground |
| J2-2 AVDD | 17 | 3.3 V |
| J2-3 CSB | 12 | GPIO18 / SPI1 CE0 |
| J2-4 MISO | 35 | GPIO19 / SPI1 MISO |
| J2-5 MOSI | 38 | GPIO20 / SPI1 MOSI |
| J2-6 SCK | 40 | GPIO21 / SPI1 SCLK |
| J2-7 | Not connected | — |
| J1-5 DVDD/DVIO | 1 | 3.3 V |
| J1-6 DVSS | 30 | Ground |
| J1-7 DVSS | 34 | Ground |
| Other J1 pins | Not connected | — |

Multiple rows use the same Pi 3.3 V rail and ground; the different physical power
pins in the tables are only a convenient wiring allocation. Check the total current
against the board specifications.

## Enable the interfaces

Edit `/boot/firmware/config.txt` and add:

```ini
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
dtoverlay=i2c2-pi5,pins_4_5,baudrate=400000
dtparam=spi=on
dtoverlay=spi1-1cs
```

Reboot, then verify that the interfaces exist:

```bash
ls -l /dev/i2c-1 /dev/i2c-2 /dev/spidev0.0 /dev/spidev1.0
```

If `i2c2-pi5` is unknown, update Raspberry Pi OS and inspect the overlay help with
`dtoverlay -h i2c2-pi5`. Do not substitute a different overlay without checking its
pin assignment.

Install the software:

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus2 python3-spidev python3-numpy python3-matplotlib
sudo usermod -aG i2c,spi "$USER"
```

Log out and back in after changing groups. Check the two I2C sensors:

```bash
i2cdetect -y 1
i2cdetect -y 2
```

Bus 1 should show `68`; bus 2 should show `6b`. `i2cdetect` cannot identify the SPI
sensors—the program validates their identity registers during startup.

## Personal configuration without Git conflicts

`multi_motion_logger.cfg` is the tracked reference configuration and can change in
future releases. Do not make routine experiment changes directly in that file.
Create a personal copy once:

```bash
cp multi_motion_logger.cfg multi_motion_logger.local.cfg
```

Edit `multi_motion_logger.local.cfg` and run the logger with that file. It is listed
in `.gitignore`, so local parameter changes will neither appear in `git status` nor
block a future `git pull`. After a pull, compare the reference file for newly added
settings and copy those settings into the local file when needed.

The shipped defaults are a 10-second window and four display rows. Running without
any configuration file also uses those defaults.

## Safe first test

Test one board at a time. In `multi_motion_logger.local.cfg`, set `enabled = false` for
the other three sensors, enable the board under test, then run:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.local.cfg --no-log
```

The program prints a detection line. Gently rotate the board and confirm the
expected axis responds. Stop with `Q`, Escape or Ctrl+C. Repeat for every board,
then enable all four and run the same command. For actual logging, omit `--no-log`:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.local.cfg
```

The configuration file is loaded only when explicitly named with `--config`.
`--no-plot` runs logging without the graph. `--no-log` disables file output.

All four sensor sections are enabled in the supplied configuration, but
`require_all_sensors = false` makes startup tolerant of incremental assembly. If an
enabled interface does not exist, a sensor is disconnected, or its identity check
fails, the program prints `WARNING: Skipping ...` and continues with every sensor
it did detect. It stops only when no sensor is available. Set an uninstalled
sensor's `enabled = false` to suppress its warning, or set
`require_all_sensors = true` when you want any missing sensor to be a fatal test
failure. Invalid configuration values are still treated as errors rather than
silently ignored.

## Live display controls

- `A`, `B`, `C`, or `D`: select a display row when that row is enabled.
- `0`: set the selected row source to `None`.
- `1`: MPU-6050 acceleration.
- `2`: MPU-6050 gyroscope.
- `3`: LSM6DSO acceleration.
- `4`: LSM6DSO gyroscope.
- `5`: ADXL355 acceleration.
- `6`: SCL3300 acceleration.
- `7`: SCL3300 inclination angles.
- `F`: show a frequency-domain spectrogram in the selected row.
- `T`: show time-domain traces in the selected row.
- `Q` or Escape: stop cleanly.

Each row has independent source and display mode. Switching views affects only
drawing; every enabled sensor continues to be acquired and logged.

Set `display_rows` to `2`, `3`, or `4` in the `[logger]` section of the
configuration file. The default is four. Additional rows use
`section_c_source`/`section_c_view` and `section_d_source`/`section_d_view` for
their initial selections. Invalid row counts stop with a clear error. Four-row mode
caps the figure height at 10 inches so it fits a typical 1080p display; maximize the
window if additional plotting space is available.

The right side of the graph window also has independent radio-button panels for
every displayed row. Each panel selects a source and either `Time` or `Frequency`. The same
source may be selected in multiple rows, allowing its time trace and spectrogram to be
viewed together. Only detected sensors appear in the panels. Mouse selections and
keyboard shortcuts remain synchronized. Select `None` to leave a row blank; a None
row does not request any source for display-scoped logging.

Time-domain plots draw every sample in the active window and disable Matplotlib's
line-path simplification. Short peaks therefore retain the same sampled shape while
moving across the window. This increases rendering work for long windows; lower
`plot_hz` if display responsiveness becomes poor. This setting affects only the
display and never the binary logging rate.

With the default 256-sample FFT and 75% overlap at 1000 Hz, each FFT frame spans
256 ms, adjacent columns start 64 ms apart, and frequency-bin spacing is 3.90625 Hz.
`spectrogram_hz` controls how often the image is redrawn, not how much logged data
is acquired.

The spectrogram derives its time extent and frequency scale from the timestamps of
the selected source. Consequently it fills the active time window even when that
sensor's delivered rate is below the requested host rate, and its displayed
frequency calibration follows the achieved rate. The figures above are exact only
when the source is delivering 1000 samples/s.

## Binary output and MATLAB

Every run creates a timestamped directory under `recordings`. `session.json`
contains the scaling, channel order, actual sample counts, achieved average rates,
errors, logging scope and file names.

The default `log_scope = displayed` records only logical sources currently selected
in at least one non-None row. Time and Frequency views are equivalent for logging,
and selecting the same source in multiple rows does not duplicate it. Logging for a
source starts or stops when a row selection changes. Use `log_scope = all` to record
every source from every enabled and detected sensor, regardless of the display.
Acquisition and live buffers continue for all detected sensors in either mode.

Every logical source has a raw data file and a matching timestamp file. Data rows
contain only three sensor integers. Each timestamp is a little-endian signed int64
number of nanoseconds from the common program start, recorded at completion of the
host read. Timestamp and data row counts therefore match, including when a source
is displayed during several separated intervals.

| Source data file | One row |
|---|---|
| `mpu_accel.bin`, `mpu_gyro.bin` | X/Y/Z, three little-endian int16 |
| `lsm_accel.bin`, `lsm_gyro.bin` | X/Y/Z, three little-endian int16 |
| `adxl355_accel.bin` | X/Y/Z, three little-endian int32 containing sign-extended 20-bit values |
| `scl3300_accel.bin`, `scl3300_angle.bin` | X/Y/Z, three little-endian int16 |

The corresponding timestamp file inserts `_time` before `.bin`, for example
`mpu_accel_time.bin`. Files for available but never selected sources may be empty in
`displayed` mode.

MATLAB example:

```matlab
folder = 'recordings/motion_YYYYMMDD_HHMMSS';
meta = jsondecode(fileread(fullfile(folder, 'session.json')));

fid = fopen(fullfile(folder, 'mpu_accel.bin'), 'rb', 'ieee-le');
mpu_accel = fread(fid, [3 Inf], 'int16=>double').'; fclose(fid);
mpu_accel = mpu_accel / meta.devices.mpu6050.accel_lsb_per_g;

fid = fopen(fullfile(folder, 'mpu_accel_time.bin'), 'rb', 'ieee-le');
t_mpu = fread(fid, Inf, 'int64=>double'); fclose(fid);
t_mpu = t_mpu * 1e-9;  % seconds from common program start

fid = fopen(fullfile(folder, 'adxl355_accel.bin'), 'rb', 'ieee-le');
adxl = fread(fid, [3 Inf], 'int32=>double').'; fclose(fid);
adxl = adxl / meta.devices.adxl355.lsb_per_g;
```

Use the same three-column `int16=>double` pattern for LSM6DSO and SCL3300,
applying the scale fields in `session.json`. Host timestamps describe when Python
completed each read; they are not hardware conversion timestamps.

## Future geophone

There are enough free GPIO pins, but a Raspberry Pi has no analog input. The output
of the low-noise amplifier must feed an external ADC whose input range/common-mode
matches the amplifier. Add an analog anti-alias filter appropriate to the chosen
sample rate. Choose the ADC based on the geophone bandwidth, signal level, dynamic
range and whether differential input is required—not merely pin availability.

SPI5 can be reserved for that ADC:

| Future ADC SPI signal | Pi 5 physical pin | GPIO/function |
|---|---:|---|
| MOSI | 8 | GPIO14 / SPI5 MOSI |
| MISO | 33 | GPIO13 / SPI5 MISO |
| SCLK | 10 | GPIO15 / SPI5 SCLK |
| CS | 32 | GPIO12 / SPI5 CE0 |
| DRDY (optional) | 36 | GPIO16 |
| Power/Ground | suitable 3.3 V/GND pin | Check ADC board requirements |

Pins 8 and 10 are also the primary UART pins, so the serial console/UART must not
use them. Do not enable SPI5 yet: the correct overlay and wiring depend on the ADC
selected. Once that part is known, the logger can add a fifth independent worker
without changing the present four buses.

## References

- Raspberry Pi GPIO/SPI documentation: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- ADXL355 evaluation board guide UG-1030: https://www.analog.com/media/en/technical-documentation/user-guides/EVAL-ADXL354-355-UG-1030.pdf
- Murata SCL3300 PCB specification: https://www.murata.com/-/media/webrenewal/products/sensor/pdf/specification/sca3300-scl3300-scl3400-sca3400-pcb-specification.ashx
