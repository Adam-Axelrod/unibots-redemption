#!/usr/bin/env python3
"""lidar_viewer -- watch /scan live in a browser. A bench tool, not part of a match.

Draws the robot in the middle with its front pointing up, and every lidar
return around it as a dot. Use it to check that the lidar is mounted the right
way round (hold a hand in front of the robot: the dots must appear ABOVE it)
and to see how far it really reaches.

Run it next to anything that publishes /scan -- the real robot or the sim:

    ros2 run robot_bringup lidar_viewer

then open http://<machine>:8081. On the Pi, from a laptop on the tailnet, that
is http://raspberrypi:8081 (see docs/hardware/lidar.md).

It only listens: it never publishes, so it is safe to run next to a driving
node. Like the simulator's visualiser it uses only the standard library and
streams with Server-Sent Events, so there is nothing to install.

Not for matches: the rules forbid any remote connection during one (R8).
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

WEB_DIR = os.path.join(os.path.dirname(__file__), 'web')


class LidarViewer(Node):

    def __init__(self):
        super().__init__('lidar_viewer')

        self.declare_parameter('port', 8081)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('scan_topic', 'scan')

        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._blob = None        # latest scan, already JSON-encoded
        self._seq = 0            # bumped on every scan, so streams send each once
        self._stamps = []        # arrival times, for the scan-rate readout

        # Sensor QoS (best effort) matches every /scan publisher, reliable or not.
        # A reliable subscription would silently miss a best-effort publisher.
        topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)

        port = self.get_parameter('port').value
        host = self.get_parameter('host').value
        self._httpd = ThreadingHTTPServer((host, port), _make_handler(self))
        self._httpd.daemon_threads = True    # open browser tabs must not block exit
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        self.get_logger().info(f'showing /{topic.lstrip("/")} at http://{host}:{port}')

    def on_scan(self, msg: LaserScan):
        now = time.monotonic()
        self._stamps = [t for t in self._stamps if now - t < 2.0] + [now]
        hz = (len(self._stamps) - 1) / (now - self._stamps[0]) if len(self._stamps) > 1 else 0.0

        blob = json.dumps({
            'frame': msg.header.frame_id,
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'hz': round(hz, 1),
            # Millimetres as integers: a third the size of float JSON, and the
            # lidar is no better than about 4 mm anyway. inf/nan become 0, which
            # the page treats as "no return", like the board's own zeros.
            'ranges_mm': [round(r * 1000) if math.isfinite(r) else 0 for r in msg.ranges],
        })
        with self._cond:
            self._blob = blob
            self._seq += 1
            self._cond.notify_all()

    def wait_for_scan(self, last_seq, timeout):
        """Block until a scan newer than last_seq arrives. Returns (seq, blob)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._seq, self._blob

    def stop(self):
        self._httpd.shutdown()


def _make_handler(viewer):

    class Handler(BaseHTTPRequestHandler):

        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass                          # keep the ROS console readable

        def _send(self, code, body, content_type):
            payload = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = self.path.split('?', 1)[0]     # the page takes ?r=4 for its zoom
            if path in ('/', '/index.html'):
                try:
                    with open(os.path.join(WEB_DIR, 'lidar.html'), 'rb') as f:
                        self._send(200, f.read(), 'text/html; charset=utf-8')
                except FileNotFoundError:
                    self._send(404, 'lidar.html missing', 'text/plain')
            elif path == '/stream':
                self._stream()
            else:
                self._send(404, 'not found', 'text/plain')

        def _stream(self):
            """Server-Sent Events: one message per scan, as each one arrives."""
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            seq = -1
            try:
                while True:
                    new_seq, blob = viewer.wait_for_scan(seq, timeout=1.0)
                    if new_seq == seq or blob is None:
                        # No scan for a second. A comment line keeps the
                        # connection open and lets the page say "no data".
                        self.wfile.write(b': waiting\n\n')
                    else:
                        seq = new_seq
                        self.wfile.write(f'data: {blob}\n\n'.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass                      # browser tab closed

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = LidarViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C arrives as ExternalShutdownException; see CLAUDE.md.
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
