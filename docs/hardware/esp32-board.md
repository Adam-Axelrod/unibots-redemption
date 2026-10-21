# Yahboom MicroROS control board

The board that runs the motors. It came with the Yahboom MicroROS-Pi5 car.

## At a glance

| | |
|---|---|
| MCU | ESP32-S3-WROOM-1**U**-N4R2: dual core 240 MHz, 4 MB flash, 2 MB PSRAM. The `U` means an external antenna (u.FL), only needed for WiFi |
| IMU | ICM42670P, 6-axis (accelerometer + gyro, **no magnetometer**) |
| Motors | 4 brushed DC motor ports with quadrature encoders |
| Servos | 2 PWM servo ports, S1 and S2 |
| Power in | 7.4 V battery |
| Power out | 5.1 V / 5 A over USB-C, using the Pi 5's power-delivery protocol, so the Pi needs no other supply |
| Battery sensing | built in |
| Also | buzzer, LED, one spare UART, WiFi and Bluetooth |
| Link to the Pi | USB serial, shows up as `/dev/ttyUSB0` |

**Not** the STM32F103 "ROS V3" board Yahboom also sells for the Pi 5. Different
firmware, different IMU, similar marketing copy.

## What it does, and what the Pi does

The ESP32 does all the real-time work:

- mecanum inverse kinematics: `/cmd_vel` in, four wheel speeds out
- per-wheel PID, using the encoders (decoded in hardware by the ESP32's PCNT units)
- integrating wheel odometry, published as `/odom_raw`
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

## IMU

ICM42670P on I2C (SCL GPIO39, SDA GPIO40, 400 kHz, interrupt on GPIO41), read
every 10 ms. Published as a `sensor_msgs/Imu` with `frame_id` `imu_frame`.
The topic name in the shipped firmware is not yet confirmed; see
[pi-setup.md](pi-setup.md#7-record-what-the-board-publishes).

⚠️ **Check the units before using it.** In Yahboom's sample code, angular
velocity is in **degrees/s** and acceleration in **g**. ROS expects rad/s and
m/s². The shipped firmware may convert them, or may not. Rotate the robot by
hand: a quarter turn per second should read about 1.57 if it's rad/s, or 90 if
it's degrees/s.

With no magnetometer it cannot give an absolute compass heading, only how fast
the robot is turning. That is still the best fix for mecanum wheel slip: wheel
odometry is good for distance and poor for heading.

## Servos

Two PWM ports, S1 and S2. Their centre offsets are stored on the board (see
[Configuration](#configuration)). Two may not be enough for the collection
mechanism; more would need a separate servo driver or the Pi's GPIO.

## Battery sensing

A 10 kΩ / 3.3 kΩ divider into GPIO3 (ADC1 channel 2). Battery voltage is the
pin voltage x 4.03. Whether the shipped firmware publishes it as a topic is not
yet confirmed. See [battery.md](battery.md) for what the numbers mean.

## Firmware

Ships as a prebuilt `.bin`. We have **no source to modify** and almost never
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
2. Edit the values at the bottom of `config_robot.py`, then run it on the Pi:
   `python3 scripts/config_robot.py` (needs `pyserial`).
3. **Reboot the board** (unplug and replug, or its reset button). New settings
   only take effect after a reboot.
4. Start the agent again.

## Sources

- Yahboom's ESP32 tutorials, in
  [YahboomTechnology/MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5):
  `16. ESP32 basic course/` (*Read IMU data*, *Battery voltage detection*,
  *Drive motor*) and `17. microros basic course/` (*Release IMU data topic*)
- [Yahboom store page for the board](https://www.yahboom.store/products/microros-board)
