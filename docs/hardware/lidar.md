# Oradar MS200 lidar

A 360° time-of-flight lidar. It plugs into the control board's **radar port**,
not into the Pi. The ESP32 reads it over a serial port and publishes `/scan`
over micro-ROS, along with everything else the board publishes. Nothing on the
Pi talks to the lidar directly.

## Specs

From Yahboom's *Introduction to lidar*:

| | |
|---|---|
| Range | 0.03-12 m (at 90% reflectivity) |
| Accuracy | ≤4 mm from 0.2-2 m, ≤15 mm from 2-12 m |
| Scan | 360°, 10 Hz, 4500 points/s, 0.8° resolution |
| Laser tilt | 0.5-2° upwards from the base |
| Power | 5 V, about 260 mA (under 500 mA at start-up) |
| Size, weight | 37.7 x 37.5 x 33.0 mm, about 40 g |
| Interface | 4-pin UART (TX, RX, GND, 5 V), 230400 baud, 3.3 V logic |

## What `/scan` looks like

| | |
|---|---|
| Topic | `/scan`, `sensor_msgs/LaserScan` |
| Frame | `laser_frame` |

Yahboom's sample firmware publishes **360 points at 1° spacing, from -180° to
180°, range 0.12-8 m, at 11 Hz**. The shipped firmware probably matches; confirm
it during [Pi setup](pi-setup.md#7-record-what-the-board-publishes).

Two things follow from that:

- **Index 0 is behind the robot, not in front.** Angle 0 (straight ahead) is
  index 180. Always compute the angle as
  `msg.angle_min + i * msg.angle_increment`, as `sweep_planner` and
  `room_mapper` already do, and never assume an index.
- **Resolution is 1°, coarser than the lidar's own 0.8°**, and anything past
  8 m is dropped. At 1 m, 1° is about 17 mm between points, so a 20 mm Atom on
  the floor is at the edge of what it can see, even if the lidar is mounted low
  enough to hit it. Fine for walls and other robots.

## The Pi-side driver (not used)

The repo also contains a driver for running the MS200 **directly from the Pi**,
over a USB-serial adapter. We don't use it, because our lidar goes through the
board. `robot.launch.py` leaves it off by default (`lidar:=false`).

It's kept in case we ever want the lidar's full resolution and range, which
the ESP32 path cuts down. To switch, plug the MS200 into the Pi through a USB
adapter instead of the board, then:

```bash
sudo cp scripts/oradar.rules /etc/udev/rules.d/       # makes /dev/oradar
sudo udevadm control --reload && sudo udevadm trigger
ros2 launch robot_bringup robot.launch.py lidar:=true
```

Only one of the two may publish `/scan` at a time.

Notes on that driver, should you need it:

- Package `oradar_lidar`, executable `oradar_scan`, vendored as
  `docker/vendor/oradar_ros.tar.xz` and built into `/opt/lidar_ws` by
  `docker/Dockerfile.robot`. Vendored because Yahboom ships it only as a
  Google Drive download and their GitHub link (`YahboomTechnology/MS200Lidar`)
  is a 404.
- `robot.launch.py` starts `oradar_scan` directly, with
  `frame_id:=laser_frame`. The driver's own `ms200_scan.launch.py` uses
  Foxy-era syntax that raises `TypeError` on Humble, and publishes a
  conflicting `base_link` → `lidar` transform.
- `scripts/oradar.rules` matches the USB adapter the driver was set up for
  (VID `1a86`, PID `55d4`, a `ttyACM` device), so it never clashes with the
  ESP32's `ttyUSB0`. A different adapter needs a different rule.

## Mounting

The lidar must see over everything else on the robot. Anything on the chassis
at the same height as the lidar shows up in every scan as a fixed blind spot.
Its height and offset from the robot's centre go in `robot.launch.py`
(`base_link_to_laser`); see [chassis.md](chassis.md).

The arena walls are 157 mm tall, so the lidar sees them as long as it sits
below about 150 mm. Allow for the laser's 0.5-2° upward tilt: across the
2.5 m arena the beam rises by up to about 9 cm, so mount it low enough that
it still hits the far wall.
