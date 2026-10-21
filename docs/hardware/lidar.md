# Oradar MS200 lidar

A 360° time-of-flight lidar. It plugs into the **Pi** over USB, not into the
ESP32, so `/scan` never touches the control board.

| | |
|---|---|
| ROS package | `oradar_lidar`, executable `oradar_scan` |
| Topic | `/scan` (`sensor_msgs/LaserScan`), 360°, angle 0 is straight ahead |
| Frame | `laser_frame` |
| Serial | `ttyACM` device, USB VID `1a86` PID `55d4`, 230400 baud |
| Stable name | `/dev/oradar`, via our udev rule |

## The driver

Vendored as `docker/vendor/oradar_ros.tar.xz` and built into `/opt/lidar_ws` by
`docker/Dockerfile.robot`.

**Why vendored when nothing else is:** Yahboom ships this driver only as a
Google Drive download. Their own GitHub link (`YahboomTechnology/MS200Lidar`)
is a 404, so there is no upstream to depend on, and a Drive URL is not a
reproducible build source. The tarball is 75 KB and builds as-is: its
`package.xml` is already the ROS 2 one and its `CMakeLists.txt` already sets
`COMPILE_METHOD COLCON`.

**We do not use the driver's own `ms200_scan.launch.py`**, for two reasons:

1. It uses Foxy-era syntax (`node_executable=`, `node_name=`), which was
   **removed in Humble**. Running it raises `TypeError`.
2. It publishes its own `base_link` → `lidar` transform at 0.18 m, which would
   fight with the static transform in `robot.launch.py`.

Instead `robot.launch.py` starts `oradar_scan` directly and overrides its
defaults so that frames and topics match the simulator exactly:

| Parameter | Driver default | We use | Why |
|---|---|---|---|
| `frame_id` | `lidar` | `laser_frame` | same frame name in sim and on hardware |
| `scan_topic` | `/scan` | `/scan` | unchanged |
| `port_name` | `/dev/oradar` | `/dev/oradar` | udev symlink, see below |
| `baudrate` | `230400` | `230400` | unchanged |

## The serial port

The MS200 is a **`ttyACM`** device (USB CDC); the ESP32 is a **`ttyUSB`**
device. Different classes, so they can never take each other's number.
`scripts/oradar.rules` adds a stable `/dev/oradar` symlink on top. Installing
it is part of [pi-setup.md](pi-setup.md#3-install-the-lidar-udev-rule).

Without the udev rule, pass the raw device instead:

```bash
ros2 launch robot_bringup robot.launch.py lidar_port:=/dev/ttyACM0
```

## Mounting

The lidar must see over everything else on the robot. Anything on the chassis
at the same height as the lidar shows up in every scan as a fixed blind spot.
Its height and offset from the robot's centre go in `robot.launch.py`
(`base_link_to_laser`); see [chassis.md](chassis.md).

The arena walls are 157 mm tall, so the lidar sees them as long as it sits
below about 150 mm.
