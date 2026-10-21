# CLAUDE.md

Guidance for Claude Code, and the hardware reference for everyone else.

## What this is

A ROS 2 Humble stack for our robot for **Unibots UK 2026-27**: a 4WD mecanum
robot on our own 3D-printed 200 x 200 mm chassis, using the motors, control
board, lidar and battery from a Yahboom MicroROS-Pi5 car, with a Raspberry Pi 5.
It ships with a 2D simulator so the code can be developed and tested on any
laptop, with no hardware.

Full hardware documentation lives in [`docs/hardware/`](docs/hardware/README.md).
The rulebook is [`docs/unibots-rulebook-2026-27-v1.0.pdf`](docs/unibots-rulebook-2026-27-v1.0.pdf).

Three small Python packages, no C++, no vendored third-party source:

| Package | What it does | Runs on |
|---|---|---|
| `robot_control` | Driving patterns and lidar logic. **The interesting code.** | both |
| `robot_sim` | 2D simulator + browser visualiser | laptop |
| `robot_bringup` | Launch files, and `odom_tf` for the real robot | both |

## The one idea that matters

**The simulator publishes exactly the topics the hardware publishes.** Nothing
in `robot_control` knows whether it is talking to a simulation or to a real
motor. That is the whole design:

```
/cmd_vel  (Twist)      you  -> robot     linear.x forward, linear.y strafe, angular.z turn
/scan     (LaserScan)  robot -> you      360 deg, angle 0 is straight ahead
/odom     (Odometry)   robot -> you      pose + velocity, frame odom -> base_footprint
```

When adding a feature, add it against these topics. If you find yourself
branching on "am I in simulation", something has gone wrong.

## The hardware

Details, specs and sources are in [`docs/hardware/`](docs/hardware/README.md).
Read it before changing anything that touches the real robot. The short version:

| Part | What |
|---|---|
| Chassis | our own 3D print, 200 x 200 mm footprint, **mecanum** wheels |
| Control board | Yahboom MicroROS board, ESP32-S3: motors, encoders, odometry, IMU, 2 servo ports |
| IMU | ICM42670P 6-axis on the board, no magnetometer |
| Lidar | Oradar MS200, plugged into the control board, which publishes `/scan` |
| Battery | 7.4 V 2000 mAh Li-ion (2S 18650) |
| Computer | Raspberry Pi 5, 8 GB |
| Camera | **none**. Don't design features that need one |
| Collection mechanism | not decided yet |

Three software layers, and it matters that they are separate.

### 1. ESP32-S3 (on the control board)

The board is Yahboom's **MicroROS Control Board**, built on an
**ESP32-S3-WROOM-1U-N4R2**. Not the STM32F103 board Yahboom also sells for the
Pi 5 — different firmware, different IMU, near-identical marketing copy.

It does all the real-time work: mecanum inverse kinematics, per-wheel PID using
the encoders (decoded in hardware by the ESP32's PCNT units), and integrating
wheel odometry. **The Raspberry Pi is never in the motor control loop.**

The motor drivers and encoders connect only to this MCU. There is no motor
wiring to the Pi at all — the single USB cable is the entire link. The board
also powers the Pi, over USB-C at 5.1 V / 5 A.

Firmware ships as a prebuilt `.bin`, so there is **no source to modify**. You
almost never touch it. What you do change is stored configuration, via
`scripts/config_robot.py`: motor PID gains, servo offsets, `CAR_TYPE`, ROS
domain ID, WiFi.

⚠️ `config_robot.py` talks at **115200 baud** using Yahboom's own byte protocol.
The micro-ROS agent uses **921600**. Same cable, two protocols, mutually
exclusive — stop the agent before configuring, and reboot the board afterwards.

The board publishes `/scan`, `/imu` (SI units, but `orientation` is never
filled in), `/odom_raw` and `/battery` (volts x 10), and listens on
`/cmd_vel`, `/beep`, `/servo_s1` and `/servo_s2`. There is no button topic, so
the start button must go on the Pi's GPIO. Full table:
[`docs/hardware/esp32-board.md`](docs/hardware/esp32-board.md#topics).

### 2. micro-ROS agent (on the Pi)

`scripts/start_agent.sh` runs a Docker container that bridges the ESP32's serial
link onto the DDS graph. It is what turns the MCU's messages into real ROS
topics. **Nothing on the robot works until this is running.**

### 3. ROS 2 (on the Pi)

`ws/` — the packages above. Runs inside a container on the Pi.

Note the ESP32 does **not** run ROS 2. It runs micro-ROS, a cut-down client
library compiled into the firmware. There is no `ros2` CLI on it.

### Data flow

```
your node ──/cmd_vel──► micro-ROS agent ──USB serial──► ESP32 ──PWM──► motors
                                                          │
your node ◄──/odom──── odom_tf ◄──/odom_raw───────────────┘  (encoders)
your node ◄──/scan──── micro-ROS agent ◄──USB serial──── ESP32 ◄──UART──── lidar
```

The lidar plugs into the **control board**, and the ESP32 publishes `/scan`
itself. The single USB cable to the Pi carries everything.

## Build & run

### Simulator (any laptop)

```bash
docker compose up                       # builds, launches sim, serves :8080
docker compose exec sim bash            # second terminal
source install/setup.bash
ros2 run robot_control square_driver
```

### Real robot (on the Pi)

First-time setup (Docker, udev rule, board config, first drive) is in
[`docs/hardware/pi-setup.md`](docs/hardware/pi-setup.md). After that:

Clone this same repo on the Pi. `robot_control` runs there unchanged -- only the
launch file differs.

```bash
./scripts/start_agent.sh                       # terminal 1, leave running
docker compose --profile robot up robot        # terminal 2: TFs + odom_tf
docker compose exec robot bash                 # terminal 3
ros2 run robot_control sweep_planner
```

The board's power switch also powers the Pi: `sudo shutdown now` before
switching off. To stop the motors without cutting power, see
[`docs/hardware/pi-setup.md`](docs/hardware/pi-setup.md#stopping-the-motors).

Check both halves are alive before blaming your code:

```bash
ros2 topic echo /scan --once     # lidar, via the ESP32
ros2 topic echo /odom --once     # ESP32 + micro-ROS agent + odom_tf
```

### The lidar: Oradar MS200

Plugs into the control board's radar port; the ESP32 publishes `/scan` (360
points at 1°, angle_min -180°, so **index 0 is behind the robot**: always use
`angle_min + i * angle_increment`). Nothing on the Pi reads the lidar.

The repo also has a Pi-side MS200 driver (`oradar_lidar`, vendored in
`docker/vendor/`), for the lidar plugged into the Pi over USB instead. It is
**off by default** (`lidar:=false` in `robot.launch.py`); turning it on while
the board also publishes `/scan` gives two publishers on one topic. Details:
[`docs/hardware/lidar.md`](docs/hardware/lidar.md).

⚠️ **`ROS_DOMAIN_ID` must be 20 on the Pi** — that is what the ESP32 is
configured with (`set_ros_domain_id(20)` in `config_robot.py`). The simulator
uses 42, which is fine and deliberate: it keeps laptop testing isolated. But if
the Pi's shell is not on 20, the MCU's topics simply will not appear and nothing
will say why.


## The competition

Unibots UK 2026-27, "The Laboratory". Four robots share a 2500 x 2500 mm arena
with 157 mm walls for **180 s** matches, collecting 20 mm steel balls (Atoms)
and 30 mm icosahedrons (Compounds) into a scoring net behind their own coloured
wall, and parking back in the start zone at the end. Rules the code must obey:

- **Started by a physical button on the robot** (R12, G17). No starting over
  SSH, and no SSH or remote control at all during a match (R8). Everything must
  start on boot and wait for the button.
- **Stops by itself at 180 s** (G20). Points scored after the end don't count.
- **Collision avoidance is required** (R10) and checked at inspection.
- **Fully autonomous, all processing on the robot** (R8).
- The team's colour (yellow, orange, purple, green) is announced before each
  match (G9) and decides the start wall and scoring net. Make it a parameter.
- Starts and must park touching its own wall, between the two grey lines that
  mark the 760 mm net (G11, G22).
- Starts inside a 200 mm cube, may expand to 300 mm by itself (R5, R6).

The full rules are in [the rulebook](docs/unibots-rulebook-2026-27-v1.0.pdf).

## Conventions

- **Python only.** No C++ packages. Keep it that way unless there is a
  performance reason.
- **Register new nodes in `setup.py`** under `console_scripts`, or `ros2 run`
  will not find them. This catches everyone once.
- **Parameters, not constants.** Use `declare_parameter`, override with
  `--ros-args -p name:=value`.
- **Publish `/cmd_vel` continuously on a timer.** Both the simulator and the
  real firmware stop the robot if commands go quiet — that is a safety feature,
  not a bug.
- **Publish a zero `Twist` on shutdown, and catch `ExternalShutdownException`.**
  On Ctrl-C rclpy's SIGINT handler destroys the ROS context *before* `spin()`
  returns, so the exit arrives as `ExternalShutdownException`, not
  `KeyboardInterrupt`. Catch both, or the node exits 1 with a traceback. Guard
  the stop and `rclpy.shutdown()` with `if rclpy.ok():`, because by then the
  publisher's context is already invalid.

  So the zero `Twist` is belt-and-braces, not the real safety net: what actually
  stops the car after Ctrl-C is the **0.5 s command watchdog** in the simulator
  and in the ESP32 firmware. Do not disable rclpy's signal handling to work
  around this (`signal_handler_options=SignalHandlerOptions.NO`) — it makes
  `spin()` ignore Ctrl-C altogether and the node has to be killed, which is
  worse. The CI smoke test checks every node stops cleanly.
- `ws/build/`, `ws/install/`, `ws/log/` are generated. Never commit them.

## Deliberately not here

Previous versions of this project vendored the whole Yahboom stack. It was ~48 MB
and almost none of it was used. Left out on purpose:

- **SLAM (gmapping/cartographer) and Nav2** — the sweep does not need a map.
- **`robot_localization` EKF + IMU filter** — `odom_tf` does the job in 40 lines
  of Python. The trade-off is real: this is raw wheel odometry, and mecanum
  wheels slip, so heading drifts over long runs. If that starts to hurt,
  `apt install ros-humble-robot-localization` and fuse the IMU — do not
  re-vendor the source.
- **URDF and meshes** — 17 MB of STLs for RViz visuals. The launch files publish
  the handful of static transforms directly instead. Add a URDF only if you
  actually want RViz or a 3D simulator.
- **Joystick/keyboard teleop** — the browser does it in sim;
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard` does it on hardware.

If you need one of these, add it as a dependency. Do not copy source in.
