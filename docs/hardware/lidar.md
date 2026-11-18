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

The board publishes **360 points at 1° spacing, from -180° to 180°, range
0.12-8 m, at about 12.5 Hz** (checked on our board, firmware 2.1.0). A
direction with no return reads `0`, which is below `range_min`, so filter on
`range_min`/`range_max` as `sweep_planner` does.

Two things follow from that:

- **The lidar's angle 0 is not the robot's straight ahead.** On our robot the
  lidar's 0° points out of the **right** side, even though its arrow points
  forward (checked 2026-11-11). So it is mounted at **-90°**, and that is
  recorded once, in `ws/src/localisation/config/sensors.yaml`.
  Compute a beam's angle as `msg.angle_min + i * msg.angle_increment`, then add
  the mounting yaw from `robot_control.lidar_mount`, as all the nodes here do.
  Never assume an index, and never assume 0° is ahead. The simulator uses the same mounting.
- **Resolution is 1°, coarser than the lidar's own 0.8°**, and anything past
  8 m is dropped. At 1 m, 1° is about 17 mm between points, so a 20 mm Atom on
  the floor is at the edge of what it can see, even if the lidar is mounted low
  enough to hit it. Fine for walls and other robots.

## Seeing it live: `lidar_viewer`

A browser view of `/scan`: the robot in the middle, front pointing up, and every
return as a dot. It is the quickest way to check the mounting (below) and the
real range. It only subscribes, so it is safe to run next to a driving node.

On the Pi, with the agent and `docker compose --profile robot up robot` running:

```bash
docker compose exec robot bash
ros2 run robot_bringup lidar_viewer
```

Then on your Mac, with Tailscale on, open **<http://raspberrypi:8081>**. The robot
container uses host networking, so the port is already on the Pi. If the name does
not resolve, use the Pi's full Tailscale name (see
[remote-access.md](../remote-access.md)), or tunnel it over SSH and open
<http://localhost:8081>:

```bash
ssh -L 8081:localhost:8081 rpi5
```

It works the same against the simulator, at <http://localhost:8081>. Add `?r=4`
to the URL to start zoomed out to 4 m.

**To check the orientation**, hold a hand about 30 cm in front of the robot. The
red nearest-point dot must appear straight above the robot, near 0°. Move round to
its left and the dot must follow to +90°. If the dot shows up at some other angle,
the lidar is rotated by that angle. If left and right are mirrored, its angle
direction is reversed. Hover over any dot to read its angle and range.

**To check the range**, zoom out and read *farthest* in the side panel, then walk a
flat object away from the robot until its dots disappear. Remember that the board
cuts everything past 8 m.

Not for matches: no remote connections are allowed during one (R8).

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

**The lidar's arrow must point at the front of the robot.** The arrow is angle 0, and
the code assumes angle 0 is straight ahead. If it points another way, wall sensing and
collision avoidance look the wrong way, with no error.

The battery wires currently sit in the lidar's view, at about +145° and 13 cm.

The lidar must see over everything else on the robot. Anything on the chassis
at the same height as the lidar shows up in every scan as a fixed blind spot.
Its height and offset from the robot's centre go in
`ws/src/localisation/config/sensors.yaml`; see [chassis.md](chassis.md).

The arena walls are 157 mm tall, so the lidar sees them as long as it sits
below about 150 mm. Allow for the laser's 0.5-2° upward tilt: across the
2.5 m arena the beam rises by up to about 9 cm, so mount it low enough that
it still hits the far wall.
