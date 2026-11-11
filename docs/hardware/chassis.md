# Chassis

We are not using Yahboom's chassis. We are 3D printing our own, and reusing
the Yahboom car's motors, control board, lidar and battery.

## Size

The footprint is **200 x 200 mm**, exactly at the limit.

- **R5** At the start of a match the whole robot must fit inside a
  **200 x 200 x 200 mm** cube, checked with a sizing tool. Wheels, lidar,
  wires and the team number sign all count.
- **R6** Once the match has started it may expand to **300 x 300 x 300 mm**,
  but only by itself: nothing actuated by hand. Both sizes are inspected.

At exactly 200 mm there is no margin for print tolerance or a protruding
screw. Measure the printed robot against a 200 mm box before an event.

## Wheels

**Mecanum**, but the real robot **cannot strafe yet**: the simulator honours
`linear.y`, and Yahboom's firmware ignores it
([esp32-board.md](esp32-board.md#firmware)).

Mecanum wheels come as two left-handed and two right-handed wheels, and must
be fitted in the right pattern or the robot cannot strafe. Looking down on the
robot, the rollers on the tops of the four wheels should form an **X**. To be
sure, test it with the wheels on the floor, once the firmware can strafe: a pure
strafe command (`linear.y` only) should slide the robot sideways. If it spins or
barely moves instead, swap the wheels diagonally.

Mecanum wheels slip, so wheel odometry drifts, especially in heading. The IMU
on the control board can correct heading; see
[esp32-board.md](esp32-board.md#imu).

## Required by the rules

To design in from the start, all checked at inspection:

| Rule | Requirement |
|---|---|
| R7 | Team number on the side, at least 50 x 50 mm |
| R11 | An easily reached **power switch**, so a field resetter can turn the robot off. The control board has one; make sure it's reachable |
| R12, G17 | A physical **start button** on the robot. Starting over SSH or a remote is not allowed. See [esp32-board.md](esp32-board.md#buttons-switch-and-buzzer) for the options |
| R13, R14 | Battery held securely, removable, and enclosed (see [battery.md](battery.md)) |
| R15 | No loose or dangling wires |
| R16 | No sharp edges or corners |
| R10 | Collision avoidance, so we don't ram other robots |

## Sensor positions

`ws/src/robot_bringup/launch/robot.launch.py` publishes where the lidar and IMU
sit relative to the robot's centre. The lidar's mounting comes from
`ws/src/localisation/config/sensors.yaml`, relative to `base_footprint`; its
20 mm forward offset is provisional. The current numbers were
measured on **Yahboom's** car and are wrong for ours. Once the chassis is
printed, measure and update:

| Transform | What to measure |
|---|---|
| `base_footprint_to_base_link` | height of the chassis centre above the floor |
| `lidar.translation_m` in `sensors.yaml` | lidar's centre above the floor point: forward/back, left/right, height |
| `base_link_to_imu` | control board's IMU chip: forward/back, left/right, height |

Distances in metres, forward is +x, left is +y, up is +z. If a sensor is
mounted rotated, set its yaw too, anticlockwise positive seen from above. The
lidar's yaw is **-90°** (its 0° points out of the right side): see
[lidar.md](lidar.md). The nodes read it from the transform, so change it in
`sensors.yaml` and nowhere else.

## Collection mechanism

Not decided yet. The control board has two PWM servo ports; anything more needs
extra hardware. Game elements are 20 mm steel balls (Atoms, magnetic) and 30 mm
printed icosahedrons (Compounds); see the rulebook for scoring.
