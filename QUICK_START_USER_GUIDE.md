# Safety Node: step-by-step user guide

This guide is for a first-time Windows user who wants to:

1. connect to the Safety Node;
2. view live motion and spectrograms without recording;
3. make a simple vibration test; and
4. make and retrieve a short recording.

The normal connection is the Safety Node's own Wi-Fi hotspot. Plan B uses the
USB Wi-Fi dongle and an existing Wi-Fi network. Connecting a monitor directly to
the Raspberry Pi is reserved for recovery if both network methods fail.

## Before starting

Have these items ready:

- the assembled Safety Node and its power supply;
- a Windows computer with Wi-Fi;
- a sealed **plastic** bottle partly filled with water;
- the Raspberry Pi login password supplied by the device administrator; and
- Windows Remote Desktop Connection, which is included with Windows.

The user works in the Raspberry Pi desktop through Windows Remote Desktop, then
opens a terminal inside Raspberry Pi OS. The hotspot password and Raspberry Pi
login password are separate passwords.

Place the Safety Node on a stable surface. Clear people, animals, glass and other
fragile objects away from the test area. Do not throw a glass bottle.

## Part 1 — Power on and use the normal hotspot connection

1. Apply power to the Safety Node.
2. Allow approximately one minute for the Raspberry Pi to finish starting.
3. On Windows, select the Wi-Fi/network icon in the taskbar.
4. Select the following network:

   ```text
   Safetynodes
   ```

5. Enter the Wi-Fi password:

   ```text
   safetynodes
   ```

6. Select **Connect automatically**, then select **Connect**.
7. Windows may report **No internet**. This is normal: the connection is a direct
   local link to the Safety Node. Remain connected to it.

The Raspberry Pi has the fixed address `10.42.0.1` on this hotspot.

## Part 2 — Log in with Windows Remote Desktop

1. Open the Windows Start menu, type **Remote Desktop Connection**, and open it.
   Its executable name is `mstsc`.
2. In **Computer**, enter:

   ```text
   10.42.0.1
   ```

3. Select **Connect**.
4. If Windows displays an identity or certificate warning, confirm that the
   address is `10.42.0.1`, then accept it for this Safety Node.
5. At the Raspberry Pi login screen, enter:

   ```text
   Username: admin
   Password: the Raspberry Pi login password
   ```

6. Wait for the Raspberry Pi OS desktop to appear inside the Remote Desktop
   window.
7. Open a terminal from the Raspberry Pi desktop. Use this Raspberry Pi terminal
   for all commands in the following sections.

## Part 3 — Start a live test without logging

Change to the program folder:

```bash
cd babak/fall_detection/
```

Start the application with logging explicitly disabled:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.cfg --no-log
```

Wait for the graph window to open on Windows. The `--no-log` option is the
important safety check for this first run: no binary sensor recording is made,
even if logging is enabled in the configuration file.

Warnings about a missing sensor are acceptable if that sensor is intentionally
not fitted. The program continues with the sensors it detects. If no graph opens,
see the troubleshooting section before continuing.

## Part 4 — Navigate between sensors and views

The graph has rows named A, B, C and D. Each row has controls on the right for
selecting its sensor and choosing **Time** or **Frequency**. The same sensor can be
shown in two rows at once.

The keyboard reference is:

| Key | Source or action |
|---|---|
| `A`, `B`, `C`, `D` | Select a graph row |
| `0` | Make the selected row empty |
| `1` | MPU-6050 acceleration |
| `2` | MPU-6050 gyroscope |
| `3` | LSM6DSO acceleration |
| `4` | LSM6DSO gyroscope |
| `5` | ADXL355 acceleration |
| `6` | SCL3300 acceleration |
| `7` | SCL3300 inclination angles |
| `T` | Show the selected row in time view |
| `F` | Show the selected row as a spectrogram |
| `N` | Calibrate the noise reference |
| `Q` or `Escape` | Stop the program cleanly |

Try this example:

1. Click inside the graph window so it receives keyboard input.
2. Press `A`, then `1`, then `T`. Row A now shows MPU acceleration in time.
3. Press `B`, then `1`, then `F`. Row B now shows the same MPU acceleration as a
   spectrogram.
4. Try other sensor numbers in the remaining rows, or use the on-screen controls.

Only detected sensors are offered by the on-screen selectors. Changing a row does
not stop the other installed sensors from being measured.

## Part 5 — Calibrate and perform the vibration demonstration

1. Leave the Safety Node and the test surface completely still.
2. Click inside the graph window and press `N` once.
3. Keep the complete assembly still for the displayed calibration interval,
   normally five seconds.
4. Wait for the calibration result. If channels are rejected, remove nearby
   vibration and repeat the calibration.
5. Confirm that one row shows the desired acceleration source in **Time** and
   another shows the same source in **Frequency**.
6. From a short, consistent height, drop the sealed plastic water bottle onto the
   clear floor near the sensor. Do not strike the Raspberry Pi or its wiring.
7. Observe the transient in the time plot and the corresponding energy in the
   spectrogram. The default display contains approximately the latest ten seconds,
   so the event moves left and eventually leaves the window.
8. Repeat only if needed, using approximately the same bottle, water quantity,
   floor position and drop height.
9. Press `Q` or `Escape` to stop the no-log run cleanly.

## Optional — Change the configuration

The supplied configuration is suitable for the standard demonstration. A new user
does not need to edit it. If a test requires different display settings, first
make a personal copy so that future Git updates are not blocked by a locally edited
tracked file:

```bash
cp multi_motion_logger.cfg multi_motion_logger.local.cfg
```

Open the personal copy using the Raspberry Pi text editor, or from the terminal:

```bash
nano multi_motion_logger.local.cfg
```

The most useful `[logger]` settings are:

| Setting | What the user can change |
|---|---|
| `sample_rate_hz` | Requested common host polling rate; keep the tested default `1000` unless the test plan requires otherwise |
| `window_seconds` | Number of recent seconds shown; default `10` |
| `display_rows` | Number of visible rows: `2`, `3`, or `4` |
| `plot_hz` | Time-plot redraw rate; reducing it can make a remote desktop smoother |
| `scale_hz` | How often time-plot vertical limits are recalculated |
| `spectrogram_hz` | Spectrogram redraw rate; it does not change sensor sampling |
| `fft_samples` | FFT length; larger values improve frequency resolution but reduce time resolution |
| `fft_overlap_percent` | Overlap between adjacent FFT frames |
| `power_enabled` | `true` shows RMS/peak calculations; `false` disables them |
| `power_frames` | Number of overlapping FFT-length intervals used for live power figures |
| `snr_enabled` | `true` enables SNR calculation and manual noise calibration |
| `snr_calibration_seconds` | Duration of the quiet calibration after pressing `N` |
| `log_enabled` | Master logging switch; `--no-log` always disables logging for that run |
| `log_scope` | `displayed` logs selected row sources; `all` logs every detected source |
| `output_directory` | Parent folder for new recording sessions |

Initial content for each row can also be changed with:

```ini
section_a_source = mpu_accel
section_a_view = time
section_b_source = mpu_gyro
section_b_view = time
```

Rows C and D use equivalent `section_c_...` and `section_d_...` settings. Valid
views are `time` and `frequency`. A source can be `none`, `mpu_accel`, `mpu_gyro`,
`lsm_accel`, `lsm_gyro`, `adxl355_accel`, `scl3300_accel`, or `scl3300_angle`.

Do not change the I2C/SPI bus numbers, addresses, sensor ranges or sensor modes
unless instructed by the device maintainer. Those settings must agree with the
physical wiring and affect measurement conversion.

To use the personal configuration without logging:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.local.cfg --no-log
```

To use it with logging, omit only `--no-log`:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.local.cfg
```

## Part 6 — Make a short recording

Return to the terminal in the Raspberry Pi desktop. Start the same program
**without** `--no-log`:

```bash
python3 multi_motion_logger.py --config multi_motion_logger.cfg
```

This run is logging. Follow these steps:

1. Select the sensor rows and views needed for the test. With the default
   `log_scope = displayed`, a source is recorded while it is selected in at least
   one non-empty row. Showing one source in both Time and Frequency does not make a
   duplicate recording.
2. If SNR is required, keep the assembly still and press `N`; wait for calibration
   to finish.
3. Allow a few quiet seconds before the event.
4. Drop the sealed plastic water bottle once using the same safe procedure.
5. Allow a few quiet seconds after the event.
6. Press `Q` or `Escape` to stop cleanly. Do not simply remove power while the
   program is writing files.

Each logging run creates a new timestamped folder under:

```text
/home/admin/babak/fall_detection/recordings
```

Show the newest recording folder with:

```bash
cd /home/admin/babak/fall_detection
ls -dt recordings/motion_* | head -1
```

The folder contains `session.json` plus binary data and timestamp files for the
recorded sources. Keep the whole folder together.

## What is saved in a log?

The logger uses compact, headerless binary files rather than text. Every run has
one `session.json` file and, for each detected logical source, a data file plus a
matching timestamp file.

Examples are:

```text
mpu_accel.bin
mpu_accel_time.bin
adxl355_accel.bin
adxl355_accel_time.bin
session.json
```

The format is:

| File | Contents of each row |
|---|---|
| MPU-6050 and LSM6DSO data | X, Y, Z as three little-endian signed `int16` values |
| SCL3300 data | X, Y, Z as three little-endian signed `int16` values |
| ADXL355 data | X, Y, Z as three little-endian signed `int32` values containing its sign-extended 20-bit readings |
| Every `_time.bin` file | One little-endian signed `int64` timestamp per data row |

Each timestamp is the number of nanoseconds from the common start of the program.
The data file and its matching timestamp file have the same number of rows. Values
are raw sensor counts; `session.json` records the units, channel order, scale
factors, delivered rates, sample counts, filenames and errors needed to convert
them. Keep every file in the session folder together.

With the default `log_scope = displayed`, a detected source that was never placed
in a row can have an empty file. Selecting one source in two rows does not duplicate
its logged samples. The detailed README contains a MATLAB loading example.

To copy a recording to Windows, open Windows PowerShell in the destination folder.
While connected to the Safety Node hotspot, use the folder name printed when the
program stopped:

```powershell
scp -r admin@10.42.0.1:/home/admin/babak/fall_detection/recordings/motion_YYYYMMDD_HHMMSS .
```

Replace `motion_YYYYMMDD_HHMMSS` with the actual session folder. For Plan B,
replace `10.42.0.1` with the address found by the network-scanning tool.

## Plan B — Use the USB Wi-Fi dongle

Use this route only when the `Safetynodes` hotspot cannot be found or cannot be
joined. The USB adapter (`wlan1`) is expected to connect automatically to:

```text
Wi-Fi name: codynick
Wi-Fi password: codynick
```

1. Connect the Windows computer to the same `codynick` Wi-Fi network.
2. Wait approximately one minute after powering the Safety Node.
3. Find the Raspberry Pi's address using a trusted copy of one of these tools:
   - **Advanced IP Scanner** on Windows; or
   - **Fing** on a phone connected to the same Wi-Fi.
4. Scan the local network and look for hostname `Sohaware`, a Raspberry Pi device,
   or a newly appearing device. Record its IPv4 address, for example
   `192.168.1.57`. The actual address depends on the router and may change.
5. Open Windows Remote Desktop Connection as described in Part 2, but enter the
   discovered address instead of `10.42.0.1`.
6. Log in as `admin`, open a terminal in Raspberry Pi OS, and continue from Part 3.

Do not run `nmcli device disconnect wlan1`; that command deliberately disconnects
the USB Wi-Fi adapter.

## Troubleshooting

### The `Safetynodes` hotspot does not appear

1. Wait a full minute after power-on.
2. Turn Windows Wi-Fi off and on, then refresh the network list.
3. Move the Windows computer closer to the Safety Node. The hotspot operates on
   5 GHz, which generally has less range through walls than 2.4 GHz.
4. Try Plan B.

### Windows connects but Remote Desktop cannot reach `10.42.0.1`

1. Confirm Windows is still connected to `Safetynodes`, even if it reports no
   internet access.
2. Open Windows Command Prompt and run:

   ```text
   ping 10.42.0.1
   ```

3. If there is no reply, forget the `Safetynodes` network in Windows, reconnect
   using password `safetynodes`, and try again.
4. If it still fails, use Plan B.

### Remote Desktop works but the graph does not appear

1. Confirm that the command was entered in a terminal opened **inside the Raspberry
   Pi OS desktop**, not in Windows Command Prompt or PowerShell.
2. Look at the Raspberry Pi terminal for an error message.
3. Stop a stalled run with `Ctrl+C`, then start the no-log command again.

### A sensor is missing

Read the startup warnings in the terminal. The program skips an unavailable sensor
and continues with the sensors it detects. Do not reconnect sensor wiring while
the Raspberry Pi is powered; ask the device maintainer to power it down and check
the wiring and interface configuration.

### The spectrogram shows little or no response

Confirm that the intended sensor is selected, the row is in **Frequency** mode,
and the vibration occurs within the current ten-second display window. Try a safe
repeat at the same location. Do not increase impact energy enough to risk damaging
the sensor assembly or floor.

## Final recovery — connect a monitor to the Raspberry Pi

Use this only if both the built-in hotspot and the USB-dongle method fail.

1. Shut down or remove power from the device before attaching equipment where
   practical.
2. Connect a monitor to the Raspberry Pi's HDMI output and connect a USB keyboard
   and mouse.
3. Power the Safety Node and log in locally as `admin`.
4. Open a terminal and inspect both network adapters:

   ```bash
   nmcli device status
   ```

5. To restore the built-in hotspot, run:

   ```bash
   sudo nmcli connection up "SafetyNodes"
   ```

6. To restore the USB Wi-Fi connection, run:

   ```bash
   sudo nmcli connection up "codynick-wlan1" ifname wlan1
   ```

7. Display the current IPv4 addresses:

   ```bash
   ip -4 address show wlan0
   ip -4 address show wlan1
   ```

8. Once either network works again, return to the normal Windows procedure. The
   locally attached monitor should remain a recovery method rather than the normal
   operating workflow.
