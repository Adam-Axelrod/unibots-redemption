# CLAUDE.md

Guidance for Claude Code, and the hardware reference for everyone else.

## What this is

A ROS 2 Humble stack for a **Yahboom X3** 4WD mecanum robot car that measures a
room and sweeps it. It ships with a 2D simulator so the code can be developed
and tested on any laptop, with no hardware.

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

Three layers, and it matters that they are separate.

### 1. ESP32-S3 (on the expansion board)

The board is Yahboom's **MicroROS Control Board**, built on an
**ESP32-S3-WROOM-1U-N4R2**. Not the STM32F103 board Yahboom also sells for the
Pi 5 — different firmware, different tooling, near-identical marketing copy.

It does all the real-time work: mecanum inverse kinematics, per-wheel PID using
the encoders (decoded in hardware by the ESP32's PCNT units), and integrating
wheel odometry. **The Raspberry Pi is never in the motor control loop.**

The motor drivers and encoders connect only to this MCU. There is no motor
wiring to the Pi at all — the single USB cable is the entire link.

Firmware ships as a prebuilt `.bin`, so there is **no source to modify**. You
almost never touch it. What you do change is stored configuration, via
`scripts/config_robot.py`: motor PID gains, servo offsets, `CAR_TYPE`, ROS
domain ID, WiFi.

⚠️ `config_robot.py` talks at **115200 baud** using Yahboom's own byte protocol.
The micro-ROS agent uses **921600**. Same cable, two protocols, mutually
exclusive — stop the agent before configuring, and reboot the board afterwards.

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
your node ◄──/scan──── lidar driver ◄──USB──────────────── lidar (straight to the Pi)
```

The lidar plugs into the **Pi**, not the MCU — so `/scan` never touches the ESP32.

## Build & run

### Simulator (any laptop)

```bash
docker compose up                       # builds, launches sim, serves :8080
docker compose exec sim bash            # second terminal
source install/setup.bash
ros2 run robot_control square_driver
```

### Real robot (on the Pi)

```bash
./scripts/start_agent.sh                # terminal 1, leave running
# terminal 2: start your lidar driver, then
ros2 launch robot_bringup robot.launch.py
ros2 run robot_control sweep_planner
```

⚠️ **`ROS_DOMAIN_ID` must be 20 on the Pi** — that is what the ESP32 is
configured with (`set_ros_domain_id(20)` in `config_robot.py`). The simulator
uses 42, which is fine and deliberate: it keeps laptop testing isolated. But if
the Pi's shell is not on 20, the MCU's topics simply will not appear and nothing
will say why.

The lidar driver is **not** in this repo — it is apt-installed in the Pi's
container and depends on which lidar is fitted. Find it with:

```bash
ros2 topic info /scan                   # names the publishing node
```

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
- **Always publish a zero `Twist` on shutdown.** Otherwise the real robot keeps
  driving after Ctrl-C.
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
