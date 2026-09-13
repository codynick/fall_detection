# Raspberry Pi 5 + MPU-6050 movement logger

This logger uses the MPU-6050's 1024-byte FIFO to acquire synchronized acceleration
and angular-velocity samples at 1 kHz. The six channels appear in a 2-by-3 live
display, with an independent vertical scale for every channel. The graph refreshes
independently. Samples are saved compactly as raw signed 16-bit binary values for
MATLAB.

## Wiring (turn off the Pi first)

Typical MPU-6050 boards are marked `GY-521`.

| MPU-6050/GY-521 | Raspberry Pi 5 physical pin | Function |
|---|---:|---|
| `VCC` | pin 1 | 3.3 V |
| `GND` | pin 6 | Ground |
| `SCL` | pin 5 | GPIO3 / I2C1 SCL |
| `SDA` | pin 3 | GPIO2 / I2C1 SDA |

Leave `INT`, `XDA`, and `XCL` disconnected. Leave `AD0` low/unconnected for address
`0x68`; connect `AD0` to 3.3 V only if you need address `0x69`. Using 3.3 V avoids
the risk of breakout-board pull-ups placing 5 V on the Pi's GPIO pins.

## Enable and test I2C

```bash
sudo raspi-config
```

Choose **Interface Options > I2C > Yes**, then reboot:

```bash
sudo reboot
```

Install the required packages:

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus2 python3-numpy python3-matplotlib
```

Detect the sensor:

```bash
i2cdetect -y 1
```

The table should contain `68` (or `69` if AD0 is high). If it shows neither, stop
and check power, ground, SDA/SCL orientation, and solder joints.

For reliable high-rate reads, set I2C to 400 kHz. Edit `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Ensure this line is present (replace a simpler `dtparam=i2c_arm=on` line):

```ini
dtparam=i2c_arm=onزی
```

Reboot afterward. Short wires are strongly recommended at 400 kHz.

## Run

```bash
python3 mpu6050_logger.py --output movement_01.bin
```

Close the plot or press **Ctrl+C** to stop cleanly. The logger creates:

- `movement_01.bin`: headerless binary sensor data
- `movement_01.bin.json`: sample rate, channel order, ranges, scale factors, and
  recording start time

Each binary sample is exactly 12 bytes: six little-endian `int16` values in the
order `ax, ay, az, gx, gy, gz`. At 1 kHz this is approximately 43.2 MB per hour.

For logging over SSH without a desktop:

```bash
python3 mpu6050_logger.py --no-plot --output movement_01.bin
```

To show the real-time graph without saving any files:

```bash
python3 mpu6050_logger.py --no-log
```

The traces update at 20 Hz by default, while each graph recalculates its independent
y-axis scale only 3 times per second. Change these rates separately if needed:

```bash
python3 mpu6050_logger.py --plot-hz 20 --scale-hz 2 -o movement_01.bin
```

The defaults use ±2 g and ±250 degrees/s for maximum sensitivity. For impacts or
falls that clip those limits, use wider ranges:

```bash
python3 mpu6050_logger.py --accel-range 8 --gyro-range 1000 -o fall_01.bin
```

If `i2cdetect` reports address `69`:

```bash
python3 mpu6050_logger.py --address 0x69 -o movement_01.bin
```

## MATLAB

```matlab
filename = "movement_01.bin";
meta = jsondecode(fileread(filename + ".json"));

fid = fopen(filename, "r", "ieee-le");
raw = fread(fid, [6 Inf], "int16=>double")';
fclose(fid);

% Each row is one sample; preserve all raw counts in 'raw'.
accel_g = raw(:, 1:3) / meta.accel_lsb_per_g;
gyro_dps = raw(:, 4:6) / meta.gyro_lsb_per_dps;
t = (0:size(raw,1)-1)' / meta.sample_rate_hz;

figure;
subplot(2,1,1);
plot(t, accel_g);
ylabel("Acceleration (g)");
legend("X", "Y", "Z"); grid on;

subplot(2,1,2);
plot(t, gyro_dps);
xlabel("Time (s)"); ylabel("Angular velocity (degrees/s)");
legend("X", "Y", "Z"); grid on;
```

The final console report gives the delivered sample rate and FIFO-overflow count.
An overflow means a gap in the saved data. Linux and Python are not hard-real-time;
if overflows occur, first use `--no-plot`, reduce other system load, verify 400 kHz
I2C, and save to local storage.

The MPU-6050 accelerometer produces unique data at up to 1 kHz. Configuring an 8 kHz
sensor/FIFO rate would repeat accelerometer samples, so this program uses a genuine
1 kHz six-axis stream rather than writing duplicates.
