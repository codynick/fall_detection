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
- [MobaXterm](https://mobaxterm.mobatek.net/) installed on Windows.

MobaXterm is used because it provides both an SSH terminal and the graphical
forwarding needed to show the Python graph on Windows. The hotspot password and
the Raspberry Pi login password are separate passwords.

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

## Part 2 — Open the Safety Node from Windows

1. Start MobaXterm.
2. Confirm that its X server is running. MobaXterm normally starts it
   automatically and shows its status near the upper-right corner.
3. Select **Session**, then **SSH**.
4. In **Remote host**, enter:

   ```text
   10.42.0.1
   ```

5. Select **Specify username** and enter:

   ```text
   admin
   ```

6. Ensure **X11-forwarding** is enabled in the session's advanced SSH settings.
7. Start the session.
8. If this is the first connection, accept the host-key prompt only after
   confirming that you are connecting to the Safety Node.
9. Enter the Raspberry Pi login password when requested.

At the terminal prompt, check graphical forwarding:

```bash
echo $DISPLAY
```

A value such as `localhost:10.0` means graphical forwarding is available. An
empty response means it is not; close the session, confirm that MobaXterm's X
server and X11-forwarding are enabled, and reconnect.

## Part 3 — Start a live test without logging

Change to the program folder:

```bash
cd /home/admin/babak/fall_detection
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

## Part 6 — Make a short recording

Return to the MobaXterm terminal. Start the same program **without** `--no-log`:

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

To copy it to Windows, use MobaXterm's SFTP file browser, normally shown beside
the SSH terminal. Open `/home/admin/babak/fall_detection/recordings`, select the
newest `motion_...` folder and download it to the chosen Windows test folder.

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
5. In MobaXterm, create the same SSH session described in Part 2, but enter the
   discovered address instead of `10.42.0.1`.
6. Log in as `admin`, verify `echo $DISPLAY`, and continue from Part 3.

Do not run `nmcli device disconnect wlan1`; that command deliberately disconnects
the USB Wi-Fi adapter.

## Troubleshooting

### The `Safetynodes` hotspot does not appear

1. Wait a full minute after power-on.
2. Turn Windows Wi-Fi off and on, then refresh the network list.
3. Move the Windows computer closer to the Safety Node. The hotspot operates on
   5 GHz, which generally has less range through walls than 2.4 GHz.
4. Try Plan B.

### Windows connects but MobaXterm cannot reach `10.42.0.1`

1. Confirm Windows is still connected to `Safetynodes`, even if it reports no
   internet access.
2. Open Windows Command Prompt and run:

   ```text
   ping 10.42.0.1
   ```

3. If there is no reply, forget the `Safetynodes` network in Windows, reconnect
   using password `safetynodes`, and try again.
4. If it still fails, use Plan B.

### The terminal works but the graph does not appear

1. Stop the program with `Ctrl+C`.
2. Run `echo $DISPLAY` in MobaXterm.
3. If the result is empty, enable MobaXterm's X server and X11-forwarding, then
   reconnect the SSH session.
4. Start the no-log command again.

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
