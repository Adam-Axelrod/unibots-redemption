#!/usr/bin/env python3
"""sweep_planner -- cover as much floor as possible, and never keep pushing.

The robot repeats one decision:

    look at 24 headings, keep the ones with a robot-wide clear path
    turn to the one that reaches the most unvisited floor for the time it costs
    drive until that leg is done or something is in the way

Visited floor is a coarse grid in the /odom frame. When nothing new is in reach,
the robot heads for the nearest floor the lidar has seen but it has not visited.

If it is told to move and the lidar shows it is not moving, it is stuck: it backs
off and picks a different heading. This uses the lidar rather than /odom because
wheels slipping against a wall still count as distance on the encoders. It also
has to act well inside the 2 s after which the firmware latches a stalled wheel
off until reset.

Run it:   ros2 run robot_control sweep_planner
Watch it: http://localhost:8080
"""
import math
import random
from collections import deque
from itertools import chain

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from robot_control.util import yaw_from_quaternion, angle_diff

HEADINGS = 24                        # candidate directions, evenly spaced
AVOID_ARC = math.pi / 4.0            # after a stall, keep this far off that heading
MOTION_ARC = math.radians(35.0)      # beams that must change when driving straight
MOTION_MIN_RANGE = 0.40              # m, closer objects fill too much of the scan
BLOCKED_LENGTH = 0.30                # m ahead to stop aiming for after a stall
FRONTIER_RANGE = 2.5                 # m, how far a scan marks floor as seen
FRONTIER_TRIES = 4                   # attempts on one cell before giving up on it


class SweepPlanner(Node):

    def __init__(self):
        super().__init__('sweep_planner')

        self.declare_parameter('speed', 0.25)                # m/s
        self.declare_parameter('reverse_speed', 0.15)        # m/s
        self.declare_parameter('turn_speed', 0.8)            # rad/s
        self.declare_parameter('wall_margin', 0.20)          # m, centre to obstacle
        self.declare_parameter('corridor_half_width', 0.13)  # m, path checked ahead
        self.declare_parameter('body_half_width', 0.115)     # m, returns inside are us
        self.declare_parameter('min_range', 0.03)            # m, the MS200's own limit
        self.declare_parameter('tool_radius', 0.10)          # m, floor swept around us
        self.declare_parameter('cell_size', 0.10)            # m, visited-floor grid
        self.declare_parameter('max_leg', 1.5)               # m, before deciding again
        self.declare_parameter('backup_distance', 0.10)      # m, after a stall
        self.declare_parameter('stall_time', 0.75)           # s, must stay under 2 s
        self.declare_parameter('stall_change', 0.03)         # m, less than this = stuck
        self.declare_parameter('stall_memory', 30.0)         # s, avoid a stall's location
        self.declare_parameter('scan_timeout', 0.5)          # s, stop if the lidar stops
        self.declare_parameter('seed', 0)                    # breaks ties between headings

        self.speed = self.get_parameter('speed').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.wall_margin = self.get_parameter('wall_margin').value
        self.corridor = self.get_parameter('corridor_half_width').value
        self.body = self.get_parameter('body_half_width').value
        self.min_range = self.get_parameter('min_range').value
        self.tool_radius = self.get_parameter('tool_radius').value
        self.cell_size = self.get_parameter('cell_size').value
        self.max_leg = self.get_parameter('max_leg').value
        self.backup_distance = self.get_parameter('backup_distance').value
        self.stall_time = self.get_parameter('stall_time').value
        self.stall_change = self.get_parameter('stall_change').value
        self.stall_memory = self.get_parameter('stall_memory').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.rng = random.Random(self.get_parameter('seed').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.pose = None
        self.beams = []                  # (angle, range) of every usable return
        self.points = []                 # the same returns as (x, y), robot frame
        self.ranges = None               # per beam: range, or None if unusable
        self.beam_cos = []               # per beam: cos(angle)
        self.scan_time = None
        self.scan_count = 0
        self.mapped_scan = 0

        self.visited = set()             # grid cells we have driven over
        self.seen = set()                # grid cells the lidar shows are open
        self.frontier_tries = {}

        self.state = 'choose'            # choose, turn, drive, backup or wait
        self.target_yaw = 0.0
        self.leg = 0.0
        self.anchor = None
        self.state_time = 0.0
        self.avoid_yaw = None
        self.scan_history = deque()      # (time, ranges) since this motion began
        self.remembered = deque()        # (expiry, x, y) of unseen obstacles, /odom
        self.remembered_points = []      # the same obstacles, robot frame
        self.stalls = 0
        self.passes = 0

        self.create_timer(0.05, self.tick)
        self.get_logger().info('covering the floor')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ input

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_scan(self, msg: LaserScan):
        """Keep every usable return, in the robot's frame.

        This deliberately does not use msg.range_min: the board reports 0.12 m but
        still sends closer readings, and those are the ones that matter most. A
        reading of 0 means no return. Anything inside the chassis square is the
        robot seeing itself, such as the battery wires.
        """
        beams, points, ranges, beam_cos = [], [], [], []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            c, s = math.cos(angle), math.sin(angle)
            beam_cos.append(c)
            ranges.append(None)
            if not math.isfinite(r) or not self.min_range <= r < msg.range_max:
                continue
            x, y = r * c, r * s
            if abs(x) < self.body and abs(y) < self.body:
                continue
            ranges[-1] = r
            beams.append((angle, r))
            points.append((x, y))

        self.beams, self.points, self.ranges = beams, points, ranges
        self.beam_cos = beam_cos
        self.scan_time = self._now()
        self.scan_count += 1

    # ----------------------------------------------------------------- sensing

    def _clearance(self, direction):
        """Free distance along a robot-frame direction, in a robot-wide corridor."""
        c, s = math.cos(direction), math.sin(direction)
        nearest = math.inf
        for x, y in chain(self.points, self.remembered_points):
            ahead = x * c + y * s
            if 0.0 < ahead < nearest and abs(y * c - x * s) < self.corridor:
                nearest = ahead
        return nearest

    def _stalled(self, straight):
        """True if the scan shows no movement over the last stall_time seconds.

        Each new scan is compared with the one stall_time ago, so a stall is caught
        about stall_time after it starts, however long the robot was moving first.

        Driving straight, only beams near the line of travel are compared: beams
        hitting a wall we are driving alongside do not change even at full speed.
        Nearby returns are left out when possible, because another robot right next
        to us fills much of the scan, and its movement would look like ours.
        """
        history = self.scan_history
        if history and history[-1][1] is self.ranges:
            return False                  # nothing new to compare
        history.append((self.scan_time, self.ranges))
        while len(history) >= 2 and self.scan_time - history[1][0] >= self.stall_time:
            history.popleft()
        then, old_ranges = history[0]
        if self.scan_time - then < self.stall_time:
            return False

        limit = math.cos(MOTION_ARC)
        changes, far_changes = [], []
        for a, b, c in zip(self.ranges, old_ranges, self.beam_cos):
            if a is None or b is None or (straight and abs(c) < limit):
                continue
            changes.append(abs(a - b))
            if a >= MOTION_MIN_RANGE and b >= MOTION_MIN_RANGE:
                far_changes.append(abs(a - b))

        if len(far_changes) >= 10:
            changes = far_changes
        if len(changes) < 10:
            return False                  # too little to see, so no verdict
        changes.sort()
        return changes[int(0.75 * len(changes))] < self.stall_change

    # ------------------------------------------------------------- floor map

    def _cell(self, x, y):
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def _update_remembered(self):
        """Put remembered obstacles into the robot's frame, dropping expired ones."""
        now = self._now()
        while self.remembered and self.remembered[0][0] <= now:
            self.remembered.popleft()
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.remembered_points = [
            ((ox - x) * c + (oy - y) * s, (oy - y) * c - (ox - x) * s)
            for _, ox, oy in self.remembered]

    def _remember_obstacle(self, heading):
        """Something the lidar cannot see stopped us: note a robot-wide patch ahead."""
        x, y, _ = self.pose
        c, s = math.cos(heading), math.sin(heading)
        expiry = self._now() + self.stall_memory
        ahead = self.corridor
        for side in (-self.corridor / 2.0, 0.0, self.corridor / 2.0):
            self.remembered.append((expiry, x + ahead * c - side * s, y + ahead * s + side * c))
        self.visited |= self._leg_cells(heading, BLOCKED_LENGTH)

    def _update_map(self):
        x, y, yaw = self.pose
        reach = self.tool_radius
        steps = int(math.ceil(reach / (self.cell_size / 2.0)))
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx, dy = i * self.cell_size / 2.0, j * self.cell_size / 2.0
                if dx * dx + dy * dy <= reach * reach:
                    self.visited.add(self._cell(x + dx, y + dy))

        # Tracing rays is the slow part and the floor does not change quickly.
        if self.scan_count - self.mapped_scan < 4:
            return
        self.mapped_scan = self.scan_count
        for angle, r in self.beams[::3]:
            c, s = math.cos(yaw + angle), math.sin(yaw + angle)
            end = min(r - self.wall_margin, FRONTIER_RANGE)
            d = self.cell_size
            while d < end:
                self.seen.add(self._cell(x + d * c, y + d * s))
                d += self.cell_size

    def _leg_cells(self, heading, distance):
        """The grid cells a leg along this world heading would sweep."""
        x, y, _ = self.pose
        c, s = math.cos(heading), math.sin(heading)
        cells = set()
        d = self.cell_size
        while d <= distance + 1e-9:
            for side in (-self.tool_radius, 0.0, self.tool_radius):
                cells.add(self._cell(x + d * c - side * s, y + d * s + side * c))
            d += self.cell_size
        return cells

    def _new_floor(self, heading, distance):
        """How many unvisited cells a leg along this world heading would cross."""
        return len(self._leg_cells(heading, distance) - self.visited)

    def _nearest_frontier(self):
        """Closest cell seen but not visited, or None if there are none left."""
        x, y, _ = self.pose
        best, best_dist = None, math.inf
        for cell in self.seen - self.visited:
            cx = (cell[0] + 0.5) * self.cell_size
            cy = (cell[1] + 0.5) * self.cell_size
            dist = (cx - x) ** 2 + (cy - y) ** 2
            if dist < best_dist:
                best, best_dist = cell, dist
        return best

    # ----------------------------------------------------------- state machine

    def tick(self):
        cmd = Twist()
        fresh = (self.scan_time is not None
                 and self._now() - self.scan_time <= self.scan_timeout)
        if self.pose is None or not fresh:
            self.scan_history.clear()     # blind, so hold still
            self.pub.publish(cmd)
            return

        self._update_map()
        self._update_remembered()
        if self.state == 'choose':
            self._choose()

        if self.state == 'turn':
            self._do_turn(cmd)
        elif self.state == 'drive':
            self._do_drive(cmd)
        elif self.state == 'backup':
            self._do_backup(cmd)
        elif self.state == 'wait':
            if self._now() - self.state_time >= 0.5:
                self._enter('choose')

        self.pub.publish(cmd)

    def _enter(self, state):
        self.state = state
        self.state_time = self._now()
        self.anchor = self.pose
        self.scan_history.clear()

    def _travelled(self):
        return math.hypot(self.pose[0] - self.anchor[0], self.pose[1] - self.anchor[1])

    def _on_stall(self, blocked_yaw):
        """Called instead of commanding motion, so this tick publishes a stop."""
        self.stalls += 1
        self.avoid_yaw = blocked_yaw
        if blocked_yaw is not None:
            self._remember_obstacle(blocked_yaw)
        self.get_logger().info(f'stuck, backing off (stall {self.stalls})')
        self._enter('backup')

    def _do_turn(self, cmd):
        error = angle_diff(self.target_yaw, self.pose[2])
        if abs(error) <= 0.03:
            self._enter('drive')
            self._do_drive(cmd)
        elif self._stalled(straight=False):
            self._on_stall(None)
        else:
            cmd.angular.z = self.turn_speed * (1.0 if error > 0 else -1.0)

    def _do_drive(self, cmd):
        if self._travelled() >= self.leg or self._clearance(0.0) <= self.wall_margin:
            self._enter('choose')
        elif self._stalled(straight=True):
            self._on_stall(self.pose[2])
        else:
            cmd.linear.x = self.speed

    def _do_backup(self, cmd):
        done = (self._travelled() >= self.backup_distance
                or self._now() - self.state_time >= 1.0
                or self._clearance(math.pi) <= self.wall_margin)
        if done or self._stalled(straight=True):
            self._enter('choose')
        else:
            cmd.linear.x = -self.reverse_speed

    def _choose(self):
        """Pick the next heading and leg, or wait if nowhere is open."""
        yaw = self.pose[2]
        options = []                      # (turn, leg, new floor)
        for k in range(HEADINGS):
            turn = angle_diff(k * 2.0 * math.pi / HEADINGS, 0.0)
            if (self.avoid_yaw is not None
                    and abs(angle_diff(yaw + turn, self.avoid_yaw)) < AVOID_ARC):
                continue
            leg = min(self._clearance(turn) - self.wall_margin, self.max_leg)
            if leg >= 0.15:
                options.append((turn, leg, self._new_floor(yaw + turn, leg)))
        self.avoid_yaw = None

        if not options:
            self._enter('wait')           # boxed in: another robot may move away
            return

        def value(option):
            turn, leg, gain = option
            return gain / (leg / self.speed + abs(turn) / self.turn_speed + 0.5)

        best = max(options, key=value)
        if best[2] > 0:
            turn, leg = best[0], best[1]
        else:
            turn, leg = self._toward_frontier(options)

        self.target_yaw = yaw + turn
        self.leg = leg
        self._enter('turn')

    def _toward_frontier(self, options):
        """Nothing new in reach: take the open heading nearest to unvisited floor."""
        x, y, yaw = self.pose
        cell = self._nearest_frontier()
        if cell is None:
            self.passes += 1
            self.get_logger().info(f'floor covered (pass {self.passes}), starting again')
            self.visited.clear()
            self.frontier_tries.clear()
            want = yaw + self.rng.uniform(-math.pi, math.pi)
        else:
            # A cell we cannot get to must not hold the robot's attention forever.
            tries = self.frontier_tries.get(cell, 0) + 1
            self.frontier_tries[cell] = tries
            if tries >= FRONTIER_TRIES:
                self.visited.add(cell)
            want = math.atan2((cell[1] + 0.5) * self.cell_size - y,
                              (cell[0] + 0.5) * self.cell_size - x)

        def cost(option):
            return (abs(angle_diff(yaw + option[0], want))
                    + 0.5 * self.rng.random())

        best = min(options, key=cost)
        return best[0], min(best[1], 0.6)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SweepPlanner()            # Ctrl-C can arrive during start-up too
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C. rclpy's SIGINT handler shuts the context down before spin()
        # returns, so this arrives as ExternalShutdownException rather than
        # KeyboardInterrupt -- catching only the latter exits with a traceback.
        pass
    except rclpy_implementation.RCLError:
        # Ctrl-C can also land while the executor or a publish is using the context
        # it just invalidated. With the context still valid, this is a real error.
        if rclpy.ok():
            raise
    finally:
        # Only possible if we are exiting for our own reasons -- after Ctrl-C the
        # context is already gone and the 0.5 s command watchdog in the sim and
        # in the ESP32 firmware is what stops the robot. See CLAUDE.md.
        if node is not None:
            if rclpy.ok():
                node.pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
