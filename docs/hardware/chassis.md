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

**Mecanum.** The robot can strafe: `linear.y` in `/cmd_vel` really moves it
sideways. The simulator and the ESP32 firmware both assume this.

Mecanum wheels come as two left-handed and two right-handed wheels, and must
be fitted in the right pattern or the robot cannot strafe. Looking down on the
robot, the rollers on the tops of the four wheels should form an **X**. To be
sure, test it with the wheels on the floor: a pure strafe command
(`linear.y` only) should slide the robot sideways. If it spins or barely moves
instead, swap the wheels diagonally.

Mecanum wheels slip, so wheel odometry drifts, especially in heading. The IMU
on the control board can correct heading; see
[esp32-board.md](esp32-board.md#imu).

## Required by the rules

To design in from the start, all checked at inspection:

| Rule | Requirement |
|---|---|
| R7 | Team number on the side, at least 50 x 50 mm |
| R11 | An easily reached **power switch**, so a field resetter can turn the robot off |
| R12, G17 | A physical **start button** on the robot. Starting over SSH or a remote is not allowed |
| R13, R14 | Battery held securely, removable, and enclosed (see [battery.md](battery.md)) |
| R15 | No loose or dangling wires |
| R16 | No sharp edges or corners |
| R10 | Collision avoidance, so we don't ram other robots |

## Sensor positions

`ws/src/robot_bringup/launch/robot.launch.py` publishes where the lidar and IMU
sit relative to the robot's centre (`base_link`). The current numbers were
measured on **Yahboom's** car and are wrong for ours. Once the chassis is
printed, measure and update:

| Transform | What to measure |
|---|---|
| `base_footprint_to_base_link` | height of the chassis centre above the floor |
| `base_link_to_laser` | lidar's centre: forward/back, left/right, height |
| `base_link_to_imu` | control board's IMU chip: forward/back, left/right, height |

Distances in metres, forward is +x, left is +y, up is +z. The lidar's 0° must
point straight ahead; if it's mounted rotated, add the yaw angle too.

## Collection mechanism

Not decided yet. The control board has two PWM servo ports; anything more needs
extra hardware. Game elements are 20 mm steel balls (Atoms, magnetic) and 30 mm
printed icosahedrons (Compounds); see the rulebook for scoring.
