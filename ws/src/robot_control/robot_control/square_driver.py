#!/usr/bin/env python3
"""square_driver -- cover the whole room, lane by lane, using a lidar map.

Two maps are built as the robot drives:

    an occupancy grid   (coverage_map.GridMap, 5 cm cells, odom frame)
        from /scan: where the walls and obstacles are, and where is open floor
    a coverage grid     (lane_width cells, lined up with the walls)
        which parts of the floor the robot has already driven over

and the plan is simple:

    carry on down the current lane while the next cell is open and unvisited
    otherwise go to the nearest unvisited cell -- round obstacles, by the
    shortest path on the map -- and start a new lane there, pointing the way
    with the longest unvisited run
    when nothing reachable is left, start again

At the start the robot works out which way the walls run (coverage_map.wall_angle)
and lines the lanes up with them, so it does not matter which way it is facing.

How it drives depends on the `drive` parameter:

    mecanum  (default) drives straight at its target in any direction, using
             linear.y, and turns to face along each lane as it goes
    diff     for wheels that cannot strafe: never sends linear.y. It follows
             the path by pure pursuit -- curving towards a point `lookahead`
             ahead -- and only turns on the spot when well off course. Turns
             cost time, so choosing the next cell counts the turn to face it,
             and it sweeps into a new lane in one arc instead of stopping.
             The ESP32's mecanum maths with linear.y = 0 is plain left/right
             tank steering, so this needs no firmware change.

    ros2 run robot_control square_driver --ros-args -p drive:=diff

Every command also goes through _guard, which refuses to drive into anything
closer than `stop_gap` in the raw scan, map or no map: that catches things the
map has not caught up with, like another robot. With stopping when the lidar
goes quiet, that is the collision avoidance the rules ask for (R10).

Run it:   ros2 run robot_control square_driver
Watch it: http://localhost:8080
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from robot_control.util import yaw_from_quaternion, angle_diff
from robot_control.coverage_map import GridMap, wall_angle

# Lane directions in coverage-grid steps: along the walls, then round by 90 deg.
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def clamp(value, limit):
    return max(-limit, min(limit, value))


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('drive', 'mecanum')     # or 'diff', if it cannot strafe
        self.declare_parameter('speed', 0.25)          # m/s
        self.declare_parameter('turn_speed', 0.8)      # rad/s
        self.declare_parameter('lookahead', 0.4)       # m; diff drive steers for a point this far on
        self.declare_parameter('lane_width', 0.4)      # m between lanes
        self.declare_parameter('sweep_side', 'left')   # which way to work across the room first
        self.declare_parameter('half_width', 0.10)     # half the 200 mm chassis
        self.declare_parameter('clearance', 0.05)      # m spare round the chassis
        self.declare_parameter('stop_gap', 0.06)       # never drive closer than this
        self.declare_parameter('map_resolution', 0.05) # m per occupancy cell
        self.declare_parameter('map_range', 4.0)       # m of each beam added to the map
        self.declare_parameter('replan_period', 1.0)   # s between re-plans
        self.declare_parameter('stuck_time', 4.0)      # s stuck before skipping a cell
        self.declare_parameter('scan_timeout', 0.5)    # s; stop if the lidar goes quiet

        self.speed = self.get_parameter('speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.lookahead = self.get_parameter('lookahead').value
        self.lane_width = self.get_parameter('lane_width').value
        self.half_width = self.get_parameter('half_width').value
        self.clearance = self.get_parameter('clearance').value
        self.stop_gap = self.get_parameter('stop_gap').value
        self.map_range = self.get_parameter('map_range').value
        self.replan_period = self.get_parameter('replan_period').value
        self.stuck_time = self.get_parameter('stuck_time').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.side = -1 if self.get_parameter('sweep_side').value == 'right' else 1
        drive = self.get_parameter('drive').value
        if drive not in ('mecanum', 'diff'):
            raise ValueError(f"drive must be 'mecanum' or 'diff', not '{drive}'")
        self.strafe = drive == 'mecanum'

        self.map = GridMap(self.get_parameter('map_resolution').value,
                           self.half_width + self.clearance)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        # Sensor-data QoS (best effort): it hears a best-effort publisher --
        # micro-ROS boards often publish sensors that way -- as well as a
        # reliable one like the simulator. The default (reliable) would
        # silently hear nothing from a best-effort /scan.
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.pose = None                 # (x, y, yaw), filled in by on_odom
        self.points = []                 # lidar returns as (x, y), robot frame
        self.last_scan = None            # when they arrived
        self.last_map_update = None

        self.frame = None                # coverage grid: (origin x, origin y, wall angle)
        self.covered = set()             # coverage cells driven over
        self.skipped = set()             # coverage cells we could not reach
        self.reachable = {}              # coverage cell -> (x, y) we can reach nearest its centre
        self.lane = 0                    # index into DIRS
        self.target = None               # coverage cell we are heading for
        self.lane_start = False          # ...to start a lane there, not end one
        self.goal = None                 # (x, y) in it we are driving to
        self.path = []                   # occupancy cells from here to goal
        self.last_plan = None
        self.best_dist = math.inf        # closest we have been to goal...
        self.best_time = None            # ...and when
        self.started = None

        self.create_timer(0.05, self.tick)   # 20 Hz control loop
        self.get_logger().info(f'covering the room -- lanes {self.lane_width} m apart, '
                               f'{drive} drive')

    # ------------------------------------------------------------------ input

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_scan(self, msg: LaserScan):
        """Keep the scan as (x, y) points around the robot (x ahead, y left),
        and add it to the map a few times a second."""
        points, beams = [], []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment    # 0 == straight ahead
            if math.isfinite(r) and msg.range_min <= r < msg.range_max:
                points.append((r * math.cos(angle), r * math.sin(angle)))
                beams.append((angle, r, True))
            elif not math.isfinite(r) or r >= msg.range_max:
                beams.append((angle, msg.range_max, False))   # nothing out to max range
        self.points = points
        now = self.get_clock().now()
        self.last_scan = now

        if self.pose is not None and (self.last_map_update is None or
                                      self._age(self.last_map_update) > 0.2):
            self.map.update(*self.pose, beams, self.map_range)
            self.last_map_update = now

    # ----------------------------------------------------------- control loop

    def tick(self):
        if self.pose is None:
            return                        # no odometry yet, stay put
        if self.last_scan is None or self._age(self.last_scan) > self.scan_timeout:
            self.get_logger().warn('no /scan -- stopped until the lidar is back',
                                   throttle_duration_sec=2.0)
            self.pub.publish(Twist())     # driving blind is not allowed
            return

        x, y, yaw = self.pose
        if self.frame is None:
            self._line_up_with_walls(x, y, yaw)
            self.started = self.get_clock().now()

        # A cell is covered once we pass within half a lane of its reachable
        # point (its centre, unless that is too near a wall): that is what we
        # sweep. Not just "we are in it": along a wall, the reachable points
        # sit right on a cell's edge, and wobble would keep missing them.
        here = self._cov_cell(x, y)
        self.covered.add(here)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                c = (here[0] + di, here[1] + dj)
                cx, cy = self.reachable.get(c) or self._cov_centre(c)
                if math.hypot(cx - x, cy - y) <= self.lane_width / 2:
                    self.covered.add(c)

        # 8 cm, not 0: a goal against a wall is a hair inside _guard's stop
        # distance, and we would never quite get there. Without strafing,
        # the start of a lane only needs passing near, not stopping on: we
        # sweep into the lane in one arc instead of stopping to turn.
        reach = 0.15 if self.lane_start and not self.strafe else 0.08
        arrived = self.goal is not None and \
            math.hypot(self.goal[0] - x, self.goal[1] - y) < reach
        if self.target is None or arrived:
            self._plan(new_target=True)
        elif self._age(self.last_plan) > self.replan_period:
            self._plan(new_target=False)  # same cell, fresh path: the map may have changed
        elif self._age(self.best_time) > self.stuck_time:
            self.get_logger().info(f'cannot get to cell {self.target}, skipping it')
            self.skipped.add(self.target)
            self._plan(new_target=True)

        cmd = Twist()
        near = min(self.points, key=lambda p: math.hypot(*p), default=None)
        if near is not None and math.hypot(*near) < self.half_width + self.clearance:
            # Too close to something to plan from: back straight away from it
            # first -- or, without strafing, away along our own length.
            d = math.hypot(*near)
            if self.strafe:
                cmd.linear.x = -0.1 * near[0] / d
                cmd.linear.y = -0.1 * near[1] / d
            else:
                # Reverse if it is ahead, drive on if it is behind; if it is
                # beside us, turn to put it behind first.
                away = math.atan2(-near[1], -near[0])
                if abs(away) < 0.5:
                    cmd.linear.x = 0.1
                elif abs(away) > math.pi - 0.5:
                    cmd.linear.x = -0.1
                else:
                    cmd.angular.z = self.turn_speed * (1.0 if away > 0 else -1.0)
        elif self.goal is not None:
            self._drive(cmd, x, y, yaw)
        self._guard(cmd)
        self.pub.publish(cmd)

    def _drive(self, cmd, x, y, yaw):
        gx, gy = self.goal
        d = math.hypot(gx - x, gy - y)
        if d < self.best_dist - 0.05:
            self.best_dist, self.best_time = d, self.get_clock().now()

        # Aim for the furthest point along the path we can see straight to.
        # Squeezed in somewhere tight, we may see none of it: then just follow
        # the path a few cells ahead of where we are on it.
        ax, ay = gx, gy
        if not self.map.line_of_sight(x, y, gx, gy, skip=self.half_width):
            ax, ay = self._path_point_ahead(x, y, 4)
            for c in reversed(self.path[:40]):
                cx, cy = self.map.centre(c)
                if math.hypot(cx - x, cy - y) > 0.1 and \
                        self.map.line_of_sight(x, y, cx, cy, skip=self.half_width):
                    ax, ay = cx, cy
                    break

        a = math.hypot(ax - x, ay - y)
        if a < 0.01:
            return
        v = self.speed
        if not (self.lane_start and not self.strafe):
            v *= max(0.2, min(1.0, d / 0.15))     # slow for the final few cm

        if not self.strafe:
            # Differential: pure pursuit. Drive the circular arc that passes
            # through the aim point, starting along our current heading:
            # curvature 2 sin(error) / distance. The arc bows out from the
            # straight line the map checked, so keep the aim point close
            # (0.4 m) and the error small (under ~46 deg): then it bows out
            # under 9 cm. Further off course, turn on the spot first.
            if a > self.lookahead:
                ax, ay = x + (ax - x) * self.lookahead / a, y + (ay - y) * self.lookahead / a
                a = self.lookahead
            error = angle_diff(math.atan2(ay - y, ax - x), yaw)
            if abs(error) > 0.8:
                cmd.angular.z = self.turn_speed * (1.0 if error > 0 else -1.0)
                return
            curvature = 2.0 * math.sin(error) / a
            if abs(curvature) * v > self.turn_speed:
                v = self.turn_speed / abs(curvature)  # a tight arc: slow down for it
            cmd.linear.x = v
            cmd.angular.z = v * curvature
            return

        # Mecanum: world-frame velocity towards it, then into the robot's
        # frame -- it can drive any direction -- while turning along the lane.
        wx, wy = v * (ax - x) / a, v * (ay - y) / a
        cmd.linear.x = math.cos(yaw) * wx + math.sin(yaw) * wy
        cmd.linear.y = -math.sin(yaw) * wx + math.cos(yaw) * wy
        heading = self.frame[2] + self.lane * math.pi / 2.0
        cmd.angular.z = clamp(2.0 * angle_diff(heading, yaw), self.turn_speed)

    def _path_point_ahead(self, x, y, cells):
        """The path cell `cells` steps beyond the one nearest us."""
        if not self.path:
            return self.goal
        def gap(i):
            cx, cy = self.map.centre(self.path[i])
            return math.hypot(cx - x, cy - y)
        near = min(range(len(self.path)), key=gap)
        return self.map.centre(self.path[min(near + cells, len(self.path) - 1)])

    def _guard(self, cmd):
        """Last line of defence: never drive into anything, whatever was asked."""
        v = math.hypot(cmd.linear.x, cmd.linear.y)
        if v < 1e-6:
            return
        dx, dy = cmd.linear.x / v, cmd.linear.y / v
        # The chassis plus 2 cm -- deliberately less than the planner's
        # clearance, or the two fight in tight gaps and we never move.
        half = self.half_width + 0.02
        for px, py in self.points:
            along = px * dx + py * dy
            if 0.0 < along < self.half_width + self.stop_gap and abs(py * dx - px * dy) < half:
                cmd.linear.x = cmd.linear.y = 0.0
                return

    # --------------------------------------------------------------- planning

    def _line_up_with_walls(self, x, y, yaw):
        angle, sure = wall_angle(self.points)
        axis = yaw + angle if sure > 0.3 else yaw
        # Of the four wall directions, call the one nearest our heading "lane 0".
        axis += round(angle_diff(yaw, axis) / (math.pi / 2)) * math.pi / 2
        self.frame = (x, y, axis)
        self.get_logger().info('walls at %.0f deg (%.0f%% sure); lanes start along %.0f deg'
                               % (math.degrees(angle), 100 * sure, math.degrees(axis)))

    def _cov_cell(self, x, y):
        """Coverage cell holding (x, y). Cells are lane_width square, lined up
        with the walls, with the start position at the centre of one."""
        ox, oy, a = self.frame
        u = math.cos(a) * (x - ox) + math.sin(a) * (y - oy)
        v = -math.sin(a) * (x - ox) + math.cos(a) * (y - oy)
        return (math.floor(u / self.lane_width + 0.5), math.floor(v / self.lane_width + 0.5))

    def _cov_centre(self, c):
        ox, oy, a = self.frame
        u, v = c[0] * self.lane_width, c[1] * self.lane_width
        return (ox + math.cos(a) * u - math.sin(a) * v, oy + math.sin(a) * u + math.cos(a) * v)

    def _plan(self, new_target):
        x, y, _ = self.pose
        here = self._cov_cell(x, y)
        dist, parent = self.map.bfs(self.map.cell(x, y))

        # For every coverage cell we can reach, the reachable spot in it
        # nearest its centre. `todo` is the ones still to visit.
        spots = {}
        for f in dist:
            if not self.map.traversable(f):
                continue                  # only on the way out from a wall
            fx, fy = self.map.centre(f)
            c = self._cov_cell(fx, fy)
            cx, cy = self._cov_centre(c)
            off = math.hypot(fx - cx, fy - cy)
            if c not in spots or off < spots[c][1]:
                spots[c] = (f, off)
        self.reachable = {c: self.map.centre(f) for c, (f, _) in spots.items()}
        todo = {c: s for c, s in spots.items()
                if c not in self.covered and c not in self.skipped}

        if not todo:
            self.goal = self.target = None
            if len(self.covered) > 1:
                t = self._age(self.started)
                self.get_logger().info(f'room covered in {t:.0f} s -- starting again')
                self.covered = {here}
                self.skipped.clear()
            self.last_plan = self.get_clock().now()
            return

        if not new_target and self.target in spots:
            target = self.target          # may be covered on the way: still the lane's end
        else:
            target = self._choose(here, todo, dist)

        f = spots[target][0]
        self.path = GridMap.path_to(parent, f)
        if target != self.target:
            self.best_dist, self.best_time = math.inf, self.get_clock().now()
        self.target, self.goal = target, self.map.centre(f)
        self.last_plan = self.get_clock().now()

    def _choose(self, here, spots, dist):
        """Pick the next coverage cell to drive to, and the lane direction.
        `spots` holds only cells still to visit."""
        def run(start, d):
            """How many reachable, unvisited cells in a row from `start` in
            direction d. Something in one of them is fine: the path goes round."""
            n, c = 0, start
            while True:
                c = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
                if c not in spots:
                    return n
                n += 1

        # 1. A lane: drive straight down it to the last unvisited cell.
        n = run(here, self.lane)
        if n > 0:
            di, dj = DIRS[self.lane]
            self.lane_start = False
            return (here[0] + n * di, here[1] + n * dj)

        # 2. Otherwise the nearest unvisited cell, preferring the side of the
        #    room we are working across (so we do not leave strips behind).
        #    Without strafing, turning costs time too: count the turn to face
        #    a cell as the distance we could have driven in that time.
        self.lane_start = True
        x, y, yaw = self.pose
        across = 1 - abs(DIRS[self.lane][0])      # 0: lanes run along i, so "across" is j
        def cost(c):
            steps = dist[spots[c][0]] * self.map.res
            side = (c[across] - here[across]) * self.side
            turn = 0.0
            if not self.strafe:
                cx, cy = self.map.centre(spots[c][0])
                turn = abs(angle_diff(math.atan2(cy - y, cx - x), yaw))
            return steps + (self.lane_width if side < 0 else 0.0) + \
                turn * self.speed / self.turn_speed
        target = min(spots, key=cost)
        side = (target[across] - here[across]) * self.side
        if side < 0:
            self.side = -self.side                # nothing left that way: work back

        # New lane from there: point the way with the longest unvisited run.
        old = self.lane
        self.lane = max(range(4), key=lambda d: (run(target, d), d % 2 == old % 2,
                                                 d == (old + 2) % 4))
        return target

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C. rclpy's SIGINT handler shuts the context down before spin()
        # returns, so this arrives as ExternalShutdownException rather than
        # KeyboardInterrupt -- catching only the latter exits with a traceback.
        pass
    finally:
        # Only possible if we are exiting for our own reasons -- after Ctrl-C the
        # context is already gone and the 0.5 s command watchdog in the sim and
        # in the ESP32 firmware is what stops the robot. See CLAUDE.md.
        if rclpy.ok():
            node.pub.publish(Twist())        # always stop the robot on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
