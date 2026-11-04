# Firmware

Our own firmware for the Yahboom MicroROS control board (ESP32-S3).

Yahboom's firmware is written for a skid-steer car: it throws away `linear.y`
and always reports `vy` as 0, so a mecanum robot cannot strafe on it. Its source
is not published, and no setting changes this. This firmware does the same job
with mecanum kinematics, on the same topics, so nothing on the Pi has to change.

**Status: running on the robot, and it strafes on the floor** at 0.3 and 0.5 m/s. Only
brief runs so far. Not yet checked or done:

- Strafing at low speed. 0.15 m/s stalled at the old 60% power cap and has not been
  retried at 85%, and the gains are still starting values
- Motor temperature at 85% power, and turning on the floor
- It turned about 6° over a 30 cm strafe, so heading will drift
- `/imu` has no start-up calibration, so it shows a small gyro drift and reads about
  10.0 m/s² at rest
- Encoder pulses per wheel turn (1040) have not been checked

The bring-up below is how to set up a board from Yahboom's firmware. It has been done
on ours. The backup is on the Pi in `~/firmware-backup/`, not in git.

## What it does

| Topic | Type | Direction | Rate | Notes |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Pi → board | | `linear.y` strafes. Stops after 0.5 s without one |
| `/odom_raw` | `nav_msgs/Odometry` | board → Pi | 20 Hz | now includes `vy` |
| `/scan` | `sensor_msgs/LaserScan` | board → Pi | 12.5 Hz | 360 points, -180 to 180 degrees, no intensities |
| `/imu` | `sensor_msgs/Imu` | board → Pi | 25 Hz | SI units, `orientation` not filled in |
| `/battery` | `std_msgs/UInt16` | board → Pi | 1 Hz | volts x 10 |
| `/beep` | `std_msgs/UInt16` | Pi → board | | 0 off, 1 stays on, otherwise milliseconds |
| `/servo_s1`, `/servo_s2` | `std_msgs/Int32` | Pi → board | | -90 to 90 degrees |

Differences from Yahboom's firmware, on purpose:

- **`scripts/config_robot.py` does not work with it.** Settings are in
  [`main/chassis.h`](main/chassis.h) and [`main/config.h`](main/config.h): change,
  rebuild, flash.
- **It stops itself on a wheel fault.** If a wheel runs against its command or
  stalls at the power limit, all four stop until a reset, and the buzzer beeps
  once a second. A backwards encoder would otherwise run away at full power.
- **The node is called `unibots_board`**, not `YB_Car_Node`.

## Layout

| File | What |
|---|---|
| `main/chassis.h` | Wheel size and spacing, motor directions. **Edit this when the chassis changes** |
| `main/config.h` | Tuning, limits and the ROS domain ID |
| `main/mecanum.c` | Wheel maths. No hardware, tested on a laptop |
| `main/wheel_ctl.c` | Speed control and fault detection. Tested on a laptop |
| `main/ms200.c` | Lidar packet parser. Tested on a laptop |
| `main/motors.c` | PWM, encoders, control loop, command watchdog |
| `main/lidar.c`, `imu.c`, `periph.c` | The other hardware |
| `main/main.c`, `transport.c` | micro-ROS topics and the serial link |

## Build and test

```bash
firmware/test/run.sh    # hardware-free tests, using the laptop's compiler
firmware/build.sh       # ESP32 build in Docker, the first run is slow
```

`build.sh` uses ESP-IDF 5.2.5 and clones micro-ROS at a pinned commit into
`components/`, which git ignores. The binaries end up in `build/`. Don't move the
pin past `b5eee69` without checking: the next commit's component manifest does not
work with ESP-IDF 5.2.5.

## Bring-up

Do this with the **battery charged and the wheels off the ground**, and stop at
the first step that goes wrong. All of it runs on the Pi, from the repo root.

### 1. Back up Yahboom's firmware

Nothing here writes to the board. Yahboom's download is only V2.0.0 and ours runs
2.1.0, so this backup is the only copy of what is on the board now. It can contain
WiFi details, so it stays out of git (`firmware/backup/` is ignored).

```bash
docker stop microros-agent        # or Ctrl-C it: the agent holds the serial port
python3 -m venv ~/esptool-env && ~/esptool-env/bin/pip install esptool
mkdir -p firmware/backup && cd firmware/backup
ESPTOOL="$HOME/esptool-env/bin/esptool.py --chip esp32s3 -p /dev/ttyUSB0"
$ESPTOOL flash_id
$ESPTOOL -b 921600 read_flash 0 0x400000 yahboom-2.1.0-full.bin
$ESPTOOL -b 921600 verify_flash 0 yahboom-2.1.0-full.bin
sha256sum yahboom-2.1.0-full.bin > yahboom-2.1.0-full.bin.sha256
```

If `esptool` cannot connect: hold BOOT, press reset, release BOOT, and try again.
Press reset afterwards to run the firmware normally. Copy both files to a laptop.

### 2. Record how Yahboom's firmware behaves

With the agent and robot container running as usual, save this for comparison:
`ros2 topic list -v`, `ros2 topic info -v` for `/scan` and `/odom_raw` (QoS), one
message from `/imu` with the robot flat, and `/imu` while turning it anticlockwise.

### 3. Fill in `main/chassis.h`

Measure the wheel diameter, the track width and the wheel base. Turn one wheel
exactly once by hand against the encoder count later, to confirm 1040 pulses.

### 4. Flash

Build on a laptop (`chassis.h` must already be filled in), and copy the result across.
`build/` is not in git, so a `git pull` will not bring it.

```bash
# on the laptop, from the repo root
rsync -a --relative firmware/build/./flash_args firmware/build/./bootloader/bootloader.bin \
    firmware/build/./partition_table/partition-table.bin firmware/build/./unibots_firmware.bin \
    rpi5:Projects/unibots-redemption/firmware/build/

# on the Pi, with the agent stopped
cd ~/Projects/unibots-redemption/firmware/build
$ESPTOOL -b 921600 write_flash @flash_args
```

Then press reset and start the agent. Check every topic in the table above appears
with sensible values before anything drives.

### 5. Check the wheels, lifted

Send each with `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{...}"`, starting
slowly. Ctrl-C stops it, and the watchdog stops the motors 0.5 s later.

| Command | LF | LR | RF | RR |
|---|---|---|---|---|
| `linear: {x: 0.1}` forward | fwd | fwd | fwd | fwd |
| `linear: {y: 0.1}` strafe left | **back** | fwd | fwd | **back** |
| `linear: {y: -0.1}` strafe right | fwd | **back** | **back** | fwd |
| `angular: {z: 0.5}` turn left | **back** | **back** | fwd | fwd |

- A wheel turns the wrong way: set its `MOTOR_PWM_INVERT` flag in `chassis.h`, or
  swap M+ and M-.
- All stop and the buzzer beeps straight away: an encoder reads backwards. Set its
  `MOTOR_ENC_INVERT` flag in `chassis.h`. Yahboom's lessons only show motor 1's wiring, so this is
  the most likely thing to need fixing.

### 6. Tune, then drive

Tune `PID_KP`, `PID_KI` and `PWM_DEAD_ZONE` until measured speed follows the command
smoothly. Raise `PWM_LIMIT` (240 of 400) only once that works. Then put it on the
floor and run `ros2 run robot_control strafe_test`.

The floor test is also the first real check of the rollers: it should slide
sideways facing the same way. If it spins or barely moves, swap the wheels
diagonally ([chassis.md](../docs/hardware/chassis.md#wheels)).

## When the chassis changes

1. Update the numbers in [`main/chassis.h`](main/chassis.h). A value that looks
   wrong, such as metres instead of millimetres, stops the build.
2. Rebuild and flash ([step 4](#4-flash)).
3. If the motors were rewired, recheck the wheels lifted ([step 5](#5-check-the-wheels-lifted)).
4. If the lidar or IMU moved, update `base_link_to_laser` and `base_link_to_imu` in
   `ws/src/robot_bringup/launch/robot.launch.py`. That needs no reflash.

## Going back to Yahboom's firmware

```bash
cd firmware/backup
$ESPTOOL -b 921600 write_flash 0 yahboom-2.1.0-full.bin
```

Press reset. This restores the board's stored settings too.
