#!/usr/bin/env python3
"""Benchmark live localisation against simulator ground truth.

    source /ws/install/setup.bash
    python3 scripts/benchmark_localisation.py --duration 10 --csv /tmp/run.csv

Listens only. /odom is the simulator's exact pose and is used here for scoring,
never published or passed on. Start this before the localisation node to
measure the time to the first valid estimate.

Each /localisation/pose is paired with the /odom message nearest in stamp,
if within --tolerance seconds (default 25 ms, half the simulator's 20 Hz step).
/localisation/valid is an unstamped Bool, so its statistics are reported
separately rather than attached to individual poses.
"""
import argparse
import bisect
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_of(msg, stamp):
    p = msg.pose.pose
    return stamp, p.position.x, p.position.y, yaw_of(p.orientation)


# Nearest-stamp pairing; returns (pairs, unmatched estimates)
def pair(estimates, truths, tolerance):
    truths = sorted(truths)
    times = [t[0] for t in truths]
    pairs, unmatched = [], 0
    for est in estimates:
        i = bisect.bisect_left(times, est[0])
        near = [j for j in (i - 1, i) if 0 <= j < len(times)]
        if not near:
            unmatched += 1
            continue
        j = min(near, key=lambda k: abs(times[k] - est[0]))
        if abs(times[j] - est[0]) > tolerance:
            unmatched += 1
        else:
            pairs.append((est, truths[j]))
    return pairs, unmatched


def stats(values):
    v = sorted(values)
    p95 = v[min(len(v) - 1, math.ceil(0.95 * len(v)) - 1)]
    return {
        'mean': sum(v) / len(v),
        'rmse': math.sqrt(sum(e * e for e in v) / len(v)),
        'max': v[-1],
        'p95': p95,
    }


class Recorder(Node):

    def __init__(self):
        super().__init__('localisation_benchmark')
        self.estimates, self.truths, self.valid = [], [], []
        self.start = self.get_clock().now()
        self.first_valid_s = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/localisation/pose',
            lambda m: self.estimates.append(pose_of(m, stamp_s(m.header))), 10)
        self.create_subscription(
            Odometry, '/odom',
            lambda m: self.truths.append(pose_of(m, stamp_s(m.header))), 50)
        self.create_subscription(Bool, '/localisation/valid', self.on_valid, 10)

    def elapsed(self):
        return (self.get_clock().now() - self.start).nanoseconds * 1e-9

    def on_valid(self, msg):
        self.valid.append(msg.data)
        if msg.data and self.first_valid_s is None:
            self.first_valid_s = self.elapsed()


def report(rec, tolerance, csv_path):
    pairs, unmatched = pair(rec.estimates, rec.truths, tolerance)
    print(f'estimates {len(rec.estimates)}, ground truth {len(rec.truths)}, '
          f'paired {len(pairs)}, unmatched {unmatched} (tolerance {tolerance * 1e3:.0f} ms)')

    n_valid = sum(rec.valid)
    print(f'validity: {n_valid}/{len(rec.valid)} true, {len(rec.valid) - n_valid} invalid'
          + (f', fraction {n_valid / len(rec.valid):.3f}' if rec.valid else ''))
    print('first valid: ' + (f'{rec.first_valid_s:.2f} s after start'
                             if rec.first_valid_s is not None else 'never'))

    rows = []
    for (te, xe, ye, he), (tt, xt, yt, ht) in pairs:
        rows.append((te, xe, ye, he, xt, yt, ht,
                     math.hypot(xe - xt, ye - yt), abs(wrap(he - ht))))

    if rows:
        last = rows[-1]
        print(f'last estimate x={last[1]:+.4f} y={last[2]:+.4f} yaw={math.degrees(last[3]):.2f} deg')
        print(f'last truth    x={last[4]:+.4f} y={last[5]:+.4f} yaw={math.degrees(last[6]):.2f} deg')
        p = stats([r[7] for r in rows])
        h = stats([math.degrees(r[8]) for r in rows])
        print('position error (m):   ' + '  '.join(f'{k} {v:.4f}' for k, v in p.items()))
        print('heading error (deg):  ' + '  '.join(f'{k} {v:.3f}' for k, v in h.items()))
    else:
        print('no paired samples: no errors to report')

    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['stamp_s', 'est_x', 'est_y', 'est_yaw', 'true_x', 'true_y',
                        'true_yaw', 'pos_err_m', 'yaw_err_rad'])
            w.writerows(rows)
        print(f'wrote {len(rows)} rows to {csv_path}')
    return bool(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--duration', type=float, default=10.0, help='seconds to record')
    parser.add_argument('--tolerance', type=float, default=0.025,
                        help='max pose/odom stamp difference for a pair, seconds')
    parser.add_argument('--csv', help='optional path for per-sample errors')
    args = parser.parse_args()
    if not (math.isfinite(args.duration) and args.duration > 0):
        parser.error('--duration must be positive')

    rclpy.init()
    rec = Recorder()
    try:
        while rclpy.ok() and rec.elapsed() < args.duration:
            rclpy.spin_once(rec, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if report(rec, args.tolerance, args.csv) else 1)


if __name__ == '__main__':
    main()
