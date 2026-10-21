# Connecting the Pi to the robot

First-time setup: from a bare Raspberry Pi 5 to the robot driving under ROS.
Do it once per Pi. For logging in to the Pi remotely, see
[remote-access.md](../remote-access.md).

The Pi: Raspberry Pi 5, 8 GB, Raspberry Pi OS (Debian 13 "trixie"), hostname
`raspberrypi`, user `pi`. ROS 2 is **not** installed on the Pi itself; it runs
in Docker.

## 1. Wire it up

With the battery **disconnected**:

1. Board's 5.1 V USB-C power output → Pi's USB-C power input.
2. Board's Type-C serial port → any Pi USB port. This is the micro-ROS link.
3. Lidar → the board's radar port. The ESP32 reads it and publishes `/scan`;
   the lidar is not plugged into the Pi.
4. Motors → Motor1-4 ports (left front, left rear, right front, right rear;
   see [esp32-board.md](esp32-board.md#motors)).

Check which board port is which against the labels printed on the board.
Then connect the battery and switch on.

**Lift the wheels off the ground** (prop the robot on a box) until step 8.

## 2. Install Docker

On the Pi:

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker pi
```

Log out and back in, then check: `docker run --rm hello-world`.

## 3. Clone the repo

The Pi has one shared clone of the repo, at `~/Projects/unibots-redemption`.
If it isn't there yet, clone it:

```bash
git clone https://github.com/Adam-Axelrod/unibots-redemption.git ~/Projects/unibots-redemption
```

Then go into it and make sure it's on an up-to-date `main`. **Run every
command from here on inside this folder.**

```bash
cd ~/Projects/unibots-redemption
git switch main
git pull --ff-only
```

Code is never edited on the Pi, only pulled; see
[Sharing the `pi` account](../remote-access.md#sharing-the-pi-account).

## 4. Check the board appears

```bash
ls -l /dev/ttyUSB0
```

That is the control board, and the only device the Pi needs: the lidar comes
through it.

If it's missing, `lsusb` lists everything plugged in, and
`sudo dmesg | tail -20` right after plugging it in shows what the Pi made of
it. A board that doesn't appear at all is usually a charge-only USB cable.

## 5. Check the board's configuration

The board must be on **ROS domain 20** at **921600 baud** in **Raspberry Pi
mode**. It probably already is; the setup script makes sure.

```bash
sudo apt install python3-serial
```

The board only listens for configuration for **5 seconds after a reset**
(its MCU LED flashes every 300 ms); after that it's in micro-ROS mode and
ignores the script. So type the command first, then:

1. Press the board's **reset** button.
2. Within 5 seconds, run:
   ```bash
   python3 scripts/config_robot.py
   ```
3. Read the printout: `car_type: 1`, `domain_id: 20` and
   `ros_serial_baudrate: 921600`. If every value is `None`, the board didn't
   answer, and nothing was written: you missed the 5 seconds, so try again.
4. Press **reset** again to load the new settings.

The script **writes** the values at the bottom of `config_robot.py` to the
board, then prints them all back. See
[esp32-board.md](esp32-board.md#configuration) for what each setting means.

## 6. Start everything

Three terminals (or three `tmux` windows), **all on the Pi**. In each one,
first:

```bash
ssh rpi5
cd ~/Projects/unibots-redemption
```

The prompt should read `pi@raspberrypi:~/Projects/unibots-redemption`. If it
shows your laptop's name, you're not on the Pi.

```bash
# 1 -- the micro-ROS agent. Leave running.
./scripts/start_agent.sh

# 2 -- odom_tf and the static transforms. Leave running.
#      The first run builds the image and takes several minutes.
docker compose --profile robot up robot

# 3 -- a shell to work in
docker compose exec robot bash
```

In terminal 1, the agent should print that a session was established within a
few seconds. If it just waits, press the board's reset button.

Terminal 3's `docker compose exec` only works once terminal 2 has finished
building and started the container. Until then it says
`service "robot" is not running`; wait for terminal 2 to show output from
`odom_tf`.

In terminal 3, check both halves:

```bash
ros2 topic echo /scan --once     # lidar, via the ESP32
ros2 topic echo /odom --once     # ESP32 + agent + odom_tf
```

Both come from the board. If neither appears, the problem is between the Pi
and the board: agent not running, wrong domain ID, or the board needs a
reboot. If `/odom` works but `/scan` doesn't, check the lidar's cable to the
board's radar port.

## 7. Record what the board publishes

In terminal 3, `ros2 topic list` should show these. This is what firmware
2.1.0 had on 2026-10-21; if a reflashed board differs, update this table.

| Topic | Type | Direction | Rate | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | board → Pi | 12.5 Hz | lidar; see [lidar.md](lidar.md) |
| `/imu` | `sensor_msgs/Imu` | board → Pi | 24 Hz | SI units; see [esp32-board.md](esp32-board.md#imu) |
| `/odom_raw` | `nav_msgs/Odometry` | board → Pi | 11 Hz | wheel odometry; `odom_tf` republishes it as `/odom` |
| `/battery` | `std_msgs/UInt16` | board → Pi | 1 Hz | battery voltage x 10, e.g. `75` = 7.5 V |
| `/cmd_vel` | `geometry_msgs/Twist` | Pi → board | | drive commands; stops after 0.5 s without one |
| `/beep` | `std_msgs/UInt16` | Pi → board | | buzzer |
| `/servo_s1`, `/servo_s2` | `std_msgs/Int32` | Pi → board | | servo ports S1 and S2 |

Plus `/odom`, `/tf` and `/tf_static` from the robot container. `ros2 topic hz
<topic>` checks a rate, `ros2 topic echo <topic> --once` shows one message.

The topics can take a few seconds to appear after the agent connects. If
`ros2 topic echo` says a topic "does not appear to be published yet", wait
and try again.

## 8. First drive

Wheels still off the ground. In terminal 3, spin the wheels forward slowly
(Ctrl-C stops it, and the watchdog stops the motors 0.5 s later):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
```

All four wheels should turn forwards. If one turns backwards, swap that
motor's two power wires. Then try `linear: {y: 0.1}` (strafe left) and
`angular: {z: 0.5}` (turn left).

Once all three look right, put it on the floor and run a pattern:

```bash
ros2 run robot_control square_driver
```

## Stopping the motors

Without cutting power, from gentlest to bluntest:

1. **Ctrl-C whatever is sending `/cmd_vel`** (a `ros2 run` node, a
   `ros2 topic pub`). The board stops the motors by itself 0.5 s after
   commands stop.
2. **Send a stop**, once the sender is stopped:
   `ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"`
3. **Ctrl-C the micro-ROS agent** (terminal 1). Nothing reaches the board, so
   the motors stop within 0.5 s whatever is running.
4. **Press the board's reset button.** Restarts only the ESP32: the motors stop
   at once. Restart the agent afterwards.

## Switching off

The board's power switch powers the **Pi** too, so switching it off is pulling
the plug on a running computer, which can corrupt the SD card. Shut the Pi down
first:

```bash
sudo shutdown now
```

Wait for the Pi's green light to stop flashing, then flip the switch.

Next time, after switching on, nothing is running: start terminals 1 and 2
again as in [step 6](#6-start-everything). The image is already built, so it
takes seconds, not minutes.

## If something is wrong

| Symptom | Likely cause |
|---|---|
| `no configuration file provided: not found` | you're not in `~/Projects/unibots-redemption`, or not on the Pi at all |
| `service "robot" is not running` | terminal 2 is still building, or isn't running |
| A topic "does not appear to be published yet" | wait a few seconds after the agent connects, then try again |
| No `/odom_raw` or `/odom`, agent waiting | board not rebooted after config, wrong `ROS_DOMAIN_ID`, or agent not running |
| `config_robot.py` prints `None` for everything | missed the 5 seconds after reset; see [step 5](#5-check-the-boards-configuration) |
| `config_robot.py` hangs or reads garbage | the agent is still running; stop it first |
| `/odom` works, `/scan` doesn't | lidar cable to the board's radar port |
| A wheel turns the wrong way | swap that motor's M+ and M- wires |
| Strafe makes it spin | mecanum wheels in the wrong pattern; see [chassis.md](chassis.md#wheels) |
| Pi shows a low-voltage warning or reboots | battery flat, or the Pi is not powered from the board's 5 A output |
