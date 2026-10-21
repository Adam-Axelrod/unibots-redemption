# Yahboom MicroROS control board

The board that runs the motors. It came with the Yahboom MicroROS-Pi5 car.

## At a glance

| | |
|---|---|
| MCU | ESP32-S3-WROOM-1**U**-N4R2: dual core 240 MHz, 4 MB flash, 2 MB PSRAM. The `U` means an external antenna (u.FL), only needed for WiFi |
| IMU | ICM42670P, 6-axis (accelerometer + gyro, **no magnetometer**) |
| Motors | 4 brushed DC motor ports with quadrature encoders |
| Servos | 2 PWM servo ports, S1 and S2 |
| Lidar | radar port (UART1), read by the ESP32 and published as `/scan` |
| Power in | 7.4 V battery, T-plug |
| Charging | DC 8.4 V charger port on the board |
| Switch, buttons | main power switch, reset, BOOT, and one custom button (KEY1) |
| Power out | 5.1 V / 5 A over USB-C, using the Pi 5's power-delivery protocol, so the Pi needs no other supply |
| Battery sensing | built in |
| Also | buzzer (sounds a low-battery alarm), LEDs, two spare GPIOs, WiFi and Bluetooth |
| Link to the Pi | USB serial, shows up as `/dev/ttyUSB0` |

**Not** the STM32F103 "ROS V3" board Yahboom also sells for the Pi 5. Different
firmware, different IMU, similar marketing copy.

## What it does, and what the Pi does

The ESP32 does all the real-time work:

- mecanum inverse kinematics: `/cmd_vel` in, four wheel speeds out
- per-wheel PID, using the encoders (decoded in hardware by the ESP32's PCNT units)
- integrating wheel odometry, published as `/odom_raw`
- reading the lidar and IMU and publishing them (see [lidar.md](lidar.md))
- a **0.5 s command watchdog**: if `/cmd_vel` stops arriving, the motors stop

The Pi never touches the motors. The single USB cable is the entire link.

## Motors

| Port | Wheel |
|---|---|
| Motor1 | left front |
| Motor2 | left rear |
| Motor3 | right front |
| Motor4 | right rear |

The pin order for each port (M+, M-, GND, VCC, encoder A, encoder B) is printed
on the **back** of the board. With Yahboom's cables, the white connector end
goes to the board and the black end to the motor.

Motors are driven with 25 kHz PWM. Yahboom recommends 7.4 V for the motors.
They need about 3-5 V before they start turning, and the firmware skips that
dead zone for you.

The firmware's kinematics assume **this wheel order and mecanum wheels**. If a
wheel spins the wrong way on our chassis, swap that motor's two power wires
(M+ and M-) rather than changing code.

## Topics

What firmware 2.1.0 publishes and listens to, checked on our board:

| Topic | Type | Direction | Rate | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | board → Pi | 12.5 Hz | lidar; see [lidar.md](lidar.md) |
| `/imu` | `sensor_msgs/Imu` | board → Pi | 24 Hz | SI units; see [esp32-board.md](esp32-board.md#imu) |
| `/odom_raw` | `nav_msgs/Odometry` | board → Pi | 11 Hz | wheel odometry; `odom_tf` republishes it as `/odom` |
| `/battery` | `std_msgs/UInt16` | board → Pi | 1 Hz | battery voltage x 10, e.g. `75` = 7.5 V |
| `/cmd_vel` | `geometry_msgs/Twist` | Pi → board | | drive commands; stops after 0.5 s without one |
| `/beep` | `std_msgs/UInt16` | Pi → board | | buzzer |
| `/servo_s1`, `/servo_s2` | `std_msgs/Int32` | Pi → board | | servo ports S1 and S2 |

## IMU

ICM42670P on I2C (SCL GPIO39, SDA GPIO40, 400 kHz, interrupt on GPIO41).
Published on `/imu` as `sensor_msgs/Imu`, 24 Hz, `frame_id` `imu_frame`.

**Units are standard ROS units**, checked on our board: acceleration in m/s²
(about 9.8 on `z` at rest) and angular velocity in rad/s, anticlockwise
positive seen from above, as ROS expects. (Yahboom's sample code uses deg/s and
g; the shipped firmware converts.)

`angular_velocity` and `linear_acceleration` are filled in. `orientation` is
not: it is always `w: 1`, and all covariances are 0. Work out heading by
integrating `angular_velocity.z`, not from `orientation`.

With no magnetometer it cannot give an absolute compass heading, only how fast
the robot is turning. That is still the best fix for mecanum wheel slip: wheel
odometry is good for distance and poor for heading.

## Servos

Two PWM ports, S1 and S2, driven by publishing to `/servo_s1` and
`/servo_s2` (`std_msgs/Int32`, most likely the angle in degrees; check before
fitting the mechanism). Their centre offsets are stored on the board (see
[Configuration](#configuration)). Two may not be enough for the collection
mechanism; more would need a separate servo driver or the Pi's GPIO.

## Battery sensing

A 10 kΩ / 3.3 kΩ divider into GPIO3 (ADC1 channel 2). Battery voltage is the
pin voltage x 4.03. Published on `/battery` once a second, as volts x 10. See
[battery.md](battery.md) for what the numbers mean.

## Firmware

Ships as a prebuilt `.bin`. Ours is **version 2.1.0** (`config_robot.py`
prints it). We have **no source to modify** and almost never
reflash it. What we change is its stored configuration.

The firmware runs **micro-ROS**, a cut-down ROS client compiled into it. It is
not ROS 2: there is no `ros2` command on the board.

## Configuration

`scripts/config_robot.py` reads and writes the board's stored settings:

| Setting | Our value | Notes |
|---|---|---|
| `CAR_TYPE` | `1` (Raspberry Pi 5) | serial link to the agent, not WiFi |
| ROS domain ID | **20** | the Pi's `ROS_DOMAIN_ID` must match, or no topics appear |
| micro-ROS baud rate | 921600 | must match `scripts/start_agent.sh` |
| ROS namespace | empty | |
| Servo offsets | 0, 0 | |
| Motor PID | P 1, I 0.2, D 0.2 | |
| IMU yaw PID | P 1, I 0, D 0.2 | the firmware can hold heading with the gyro |

⚠️ `config_robot.py` talks at **115200 baud** using Yahboom's own byte
protocol. The micro-ROS agent uses **921600**. Same cable, two protocols, and
they cannot run at once:

1. Stop the micro-ROS agent.
2. Edit the values at the bottom of `config_robot.py`.
3. Press the board's **reset** button, then run `python3 scripts/config_robot.py`
   on the Pi **within 5 seconds** (needs `pyserial`). The board only accepts
   configuration in that window, while its MCU LED flashes every 300 ms. If
   every value prints as `None`, it missed the window: nothing was written.
4. Press **reset** again. New settings only take effect after a reboot.
5. Start the agent again.

## Buttons, switch and buzzer

- **Power switch**: the main power switch for the whole robot, Pi included.
  Useful for rule R11 if it's reachable once the chassis is built.
- **KEY1** (GPIO42) and **BOOT** (GPIO0): buttons the ESP32 can read. The
  shipped firmware does **not** publish either as a topic, and we can't change
  the firmware, so **the start button (rule R12) has to go on the Pi's GPIO**.
- **Buzzer**: an active buzzer, used by the firmware as a low-battery alarm.
  We can also sound it through `/beep`.

## Pin map

From Yahboom's *Introduction to microROS control board*. Only matters if we
ever write our own firmware.

| Function | GPIO |
|---|---|
| Motor 1 PWM A/B, encoder A/B | 4, 5, 6, 7 |
| Motor 2 PWM A/B, encoder A/B | 15, 16, 47, 48 |
| Motor 3 PWM A/B, encoder A/B | 9, 10, 11, 12 |
| Motor 4 PWM A/B, encoder A/B | 13, 14, 1, 2 |
| Servo S1, S2 | 8, 21 |
| IMU SCL, SDA, interrupt | 39, 40, 41 |
| Lidar (UART1) TX, RX | 17, 18 |
| Type-C serial (UART0) TX, RX | 43, 44 |
| Battery voltage | 3 |
| KEY1, BOOT | 42, 0 |
| MCU LED, buzzer | 45, 46 |
| Spare | 35, 36 |

## Sources

- Yahboom's *Introduction to microROS control board* and ESP32 tutorials, in
  [YahboomTechnology/MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5):
  `16. ESP32 basic course/` (*Read IMU data*, *Battery voltage detection*,
  *Drive motor*, *Read radar data*), `17. microros basic course/` (*Release
  IMU data topic*, *Publish lidar data topics*) and `01.Introduction/`
- [Yahboom store page for the board](https://www.yahboom.store/products/microros-board)
