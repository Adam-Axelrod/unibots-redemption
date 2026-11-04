# unibots-redemption

A 4-wheeled robot that sweeps a room, and a 2D simulator so you can work on it
without the robot.

Write a driving pattern, run it, watch the robot drive it in your browser. The
same code runs on the real car unchanged.

![what you get: a top-down room, a robot, its lidar and the path it has driven]

## Getting started

You need [Docker](https://docs.docker.com/get-started/get-docker/). That is the
only thing to install, and it works the same on Mac, Windows and Linux. You do
**not** need to install ROS.

```bash
git clone <this repo>
cd unibots-redemption
docker compose up
```

The first run takes a few minutes to download ROS. When you see
`simulating 'empty_room'`, open **<http://localhost:8080>**.

You should see a room with a robot in it. Click the page and use the arrow keys
to drive it around.

## Run a driving pattern

Leave `docker compose up` running. In a **second terminal**:

```bash
docker compose exec sim bash          # ROS is already sourced in here
ros2 run robot_control square_driver
```

Watch the browser: the robot maps the room with its lidar and sweeps all of it
in lanes lined up with the walls, going round anything in its way, and leaves a
trail.

Open **http://localhost:8081** as well for the driver's own view of things:
where it thinks it is, what the lidar sees right now, its map, the cells it has
visited, and where it is heading. The page at `:8080` shows where the robot
*really* is, and only exists in simulation. If you started `docker compose up`
before this port was added, run `docker compose down` and start it again.

Try the others:

```bash
ros2 run robot_control sweep_planner  # back-and-forth coverage of the room
ros2 run robot_control room_mapper    # prints its guess at the room size
```

## Write your own

1. Copy [`square_driver.py`](ws/src/robot_control/robot_control/square_driver.py)
   to a new file in the same folder. It is about 80 lines and shows the whole
   pattern: read `/odom`, decide, publish `/cmd_vel`.
2. Register it in [`setup.py`](ws/src/robot_control/setup.py) under
   `console_scripts` — **`ros2 run` cannot find it otherwise.**
3. Rebuild and run:

```bash
colcon build --symlink-install && source install/setup.bash
ros2 run robot_control my_driver
```

Try a harder room while you are at it:

```bash
docker compose down
# then edit docker-compose.yml, or just:
ros2 launch robot_bringup sim.launch.py world:=furnished_room
```

Worlds live in [`world.py`](ws/src/robot_sim/robot_sim/world.py) — adding one is
a few lines.

## What's where

```
ws/src/
  robot_control/    ← you work here. driving patterns, lidar logic
  robot_sim/        ← the 2D simulator and the browser view
  robot_bringup/    ← launch files: sim.launch.py and robot.launch.py
scripts/            ← real-robot only, runs on the Raspberry Pi
docker/             ← the container everything runs in
```

## The three topics

The simulator and the real robot speak exactly the same ROS topics, which is why
your code moves between them without changes:

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | you → robot: how fast to move |
| `/scan` | `sensor_msgs/LaserScan` | robot → you: lidar distances |
| `/odom` | `nav_msgs/Odometry` | robot → you: where it thinks it is |

The car has mecanum wheels, so `linear.y` really does strafe sideways.
`square_driver` uses that by default: it faces across its lanes and strafes down
them, left then right, stepping forward between them, so it hardly needs to turn.
If something has to face the way it is going, like a front-mounted collector, use
`--ros-args -p lane_style:=forward`: it faces along each lane, and slides across
to the next one while turning round.

It ignores lidar beams more than 100° from straight ahead, because the chassis
can block the lidar's rear and look like something touching the robot. If yours
has a clear view all round, run it with `--ros-args -p lidar_view:=180.0`.

**Testing on a prototype with ordinary wheels?** Run the driver with
`--ros-args -p drive:=diff`: it never sends `linear.y`, and turns to face where
it is going instead. Nothing else changes -- the ESP32 still takes `/cmd_vel`,
and with `linear.y` at 0 its mecanum maths is plain left/right tank steering.
Launch the simulator with `drive:=diff` too, and it ignores `linear.y` the same
way.

## If something breaks

**Port 8080 already in use** — something else has the port. Change both numbers
in `docker-compose.yml` under `ports`.

**`ros2: command not found`** — you are on your own machine, not inside the
container. Run `docker compose exec sim bash` first.

**Browser says "disconnected"** — the sim is not running. Check the terminal
with `docker compose up` in it.

**Your node runs but nothing moves** — the sim stops the robot if `/cmd_vel`
goes quiet for half a second. Publish continuously on a timer, not once.

**Changed the code but nothing changed** — rebuild: `colcon build
--symlink-install && source install/setup.bash`.

## The real robot

See [docs/hardware/](docs/hardware/README.md) for the hardware and first-time Pi
setup, and [CLAUDE.md](CLAUDE.md) for how it all fits together.

To watch `square_driver` on the real robot, open **http://raspberrypi:8081** in a
browser on a laptop on the Tailscale network. If that name doesn't work, run
`ssh -L 8081:localhost:8081 rpi5` and open http://localhost:8081 instead.

What to look for:

- **The orange lidar dots should sit on the white walls.** If they don't, or the
  walls look thick, doubled or bent, its position estimate has drifted.
- **Things should appear on the same side as in real life.** If not, the scan is
  mirrored or rotated: stop before it steers towards things.
- **Red text at the top** means a safety stop, or no lidar data.

The page only reads, so it can't drive the robot. For a match, turn it off with
`--ros-args -p view_port:=0`, since the rules ban remote control (R8).
