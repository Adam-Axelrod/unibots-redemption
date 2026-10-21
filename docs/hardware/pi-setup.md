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
python3 scripts/config_robot.py
```

This **writes** the values at the bottom of `config_robot.py` to the board and
then prints them all back. Read the printout. Then **reboot the board** (unplug
and replug its USB, or press its reset button). See
[esp32-board.md](esp32-board.md#configuration) for what each setting means.

## 6. Start everything

Three terminals (or three `tmux` windows):

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
few seconds. If it just waits, reboot the board (unplug and replug its USB).

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

We don't yet know exactly what the shipped firmware publishes. In terminal 3:

```bash
ros2 topic list
ros2 topic info /imu             # or whatever the IMU topic turns out to be
ros2 topic hz /imu
ros2 topic echo /imu --once
```

Fill in this table and commit it:

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/odom_raw` | `nav_msgs/Odometry` | ? | wheel odometry; `odom_tf` republishes it as `/odom` |
| `/cmd_vel` | `geometry_msgs/Twist` | n/a | subscribed |
| `/scan` | `sensor_msgs/LaserScan` | ? | lidar. Yahboom's sample: 360 points at 1°, -180° to 180°, 0.12-8 m, 11 Hz |
| ? (IMU) | `sensor_msgs/Imu` | ? | units: deg/s or rad/s? See [esp32-board.md](esp32-board.md#imu) |
| ? (battery) | ? | ? | |

Then update [esp32-board.md](esp32-board.md) and the
[still-unknown list](README.md#still-unknown).

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

## If something is wrong

| Symptom | Likely cause |
|---|---|
| No `/odom_raw` or `/odom`, agent waiting | board not rebooted after config, wrong `ROS_DOMAIN_ID`, or agent not running |
| `config_robot.py` hangs or reads garbage | the agent is still running; stop it first |
| `/odom` works, `/scan` doesn't | lidar cable to the board's radar port |
| A wheel turns the wrong way | swap that motor's M+ and M- wires |
| Strafe makes it spin | mecanum wheels in the wrong pattern; see [chassis.md](chassis.md#wheels) |
| Pi shows a low-voltage warning or reboots | battery flat, or the Pi is not powered from the board's 5 A output |
