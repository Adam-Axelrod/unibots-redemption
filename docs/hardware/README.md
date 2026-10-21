# Hardware

Everything we know about the robot's hardware, in one place. If you learn
something new about a part, write it down in the matching page here.

| Page | What's in it |
|---|---|
| [esp32-board.md](esp32-board.md) | The Yahboom MicroROS control board: motors, encoders, IMU, servos, power, configuration |
| [lidar.md](lidar.md) | The Oradar MS200 lidar and its driver |
| [battery.md](battery.md) | The 7.4 V pack: specs, competition rules, charging and storage |
| [chassis.md](chassis.md) | Our 3D-printed chassis, mecanum wheels, and the size and safety rules it must meet |
| [pi-setup.md](pi-setup.md) | Connecting the Pi 5 to the board and lidar, step by step, and checking it works |

## The robot at a glance

We bought a Yahboom **MicroROS-Pi5** robot car for its parts, and are building
our own chassis around them for [Unibots UK 2026-27](../unibots-rulebook-2026-27-v1.0.pdf).

| Part | What | Source | Status |
|---|---|---|---|
| Chassis | Our own 3D-printed design, 200 x 200 mm | us | in design |
| Wheels | Mecanum | bought separately | have |
| Drive motors | 4x encoder motors | Yahboom car | have |
| Motor controller | Yahboom MicroROS control board (ESP32-S3) | Yahboom car | have |
| IMU | ICM42670P 6-axis, on the control board | Yahboom car | have |
| Lidar | Oradar MS200 | Yahboom car | have |
| Battery | 7.4 V 2000 mAh Li-ion (2x 18650) | Yahboom car | have |
| Computer | Raspberry Pi 5, 8 GB | us | have |
| Camera | none | | not planned |
| Collection mechanism | not decided | | in design |
| Start button, power switch | required by the rules, not chosen yet | | to do |

## How the parts connect

```
                         ┌──────────── Raspberry Pi 5 ─────────────┐
                         │  micro-ROS agent    ROS 2 nodes          │
                         └──────┬──────────────────────┬────────────┘
             USB (serial 921600)│                      │USB
                                │                      │
  battery ──► ESP32-S3 control board                MS200 lidar
  7.4 V        │ 5.1 V / 5 A USB-C ──► powers the Pi
               │ 4x motor + encoder ports ──► mecanum wheels
               │ 2x PWM servo ports        ──► collection mechanism (TBD)
               └ ICM42670P IMU (on board)
```

The **Pi is never in the motor control loop**. The ESP32 runs wheel PID,
mecanum kinematics and odometry; the Pi only sends velocity commands and reads
back odometry, IMU and lidar.

## Still unknown

Fill these in as you find out, then move the answer into the right page.

- Exact motor model: Yahboom's board docs say **310** motors, the car's README
  says **370**. Check the label on the motor.
- Which ROS topics the shipped ESP32 firmware publishes (IMU? battery
  voltage?). See [pi-setup.md](pi-setup.md#7-record-what-the-board-publishes).
- Units of the IMU topic (see [esp32-board.md](esp32-board.md#imu)).
- Where the sensors sit on our chassis. The offsets in
  `ws/src/robot_bringup/launch/robot.launch.py` were measured on Yahboom's car
  and must be re-measured.

## Vendor documentation

Yahboom's tutorials for this car, including the ESP32 sample code, are PDFs in
[YahboomTechnology/MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5).
The most useful ones:

- `16. ESP32 basic course/` -- reading the IMU, battery voltage, driving motors,
  encoders
- `17. microros basic course/` -- what each micro-ROS topic looks like
- `19.Hardware related information/` -- a Google Drive link to schematics

Watch out: Yahboom also sells a **ROS V3 / STM32F103** board for the Pi 5, with
a different IMU (ICM20948) and different firmware. Its docs look almost the
same. Ignore anything that mentions STM32.
