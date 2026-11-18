#!/usr/bin/env python3
"""lidar_viewer -- watch /scan live in a browser. A bench tool, not part of a match.

Draws the robot in the middle with its front pointing up, and every lidar
return around it as a dot. Use it to check that the lidar is mounted the right
way round (hold a hand in front of the robot: the dots must appear ABOVE it)
and to see how far it really reaches.

Run it next to anything that publishes /scan -- the real robot or the sim:

    ros2 run robot_bringup lidar_viewer

and Ctrl-click the address it prints. On the Pi that is its full Tailscale
name, which works from every laptop on the tailnet, including the ones the Pi
is only shared with (see docs/hardware/lidar.md).

It only listens: it never publishes, so it is safe to run next to a driving
node. Like the simulator's visualiser it uses only the standard library and
streams with Server-Sent Events, so there is nothing to install.

Not for matches: the rules forbid any remote connection during one (R8).
"""
import json
import math
import os
import socket
import struct
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
        # The name to print in the clickable address. Empty means ask Tailscale.
        self.declare_parameter('url_host', '')

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

        # host is what we listen on (0.0.0.0 = every network), which is not an
        # address anyone can open. Print ones that are, so they can be clicked.
        name = self.get_parameter('url_host').value or _tailscale_name()
        lines = [f'showing /{topic.lstrip("/")}. Ctrl-click to open:']
        if name:
            lines.append(f'    http://{name}:{port}    <- any laptop on Tailscale')
        lines.append(f'    http://localhost:{port}    <- this machine only')
        self.get_logger().info('\n'.join(lines))

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


def _tailscale_name(timeout=0.5):
    """This machine's full Tailscale name, e.g. raspberrypi.tail6c0787.ts.net.

    The full name matters: the short one (raspberrypi) only resolves inside the
    owner's own tailnet, not on the laptops the Pi is shared with.

    The Tailscale CLI is not in the container, so ask Tailscale's DNS server
    directly. It is always at 100.100.100.100 on a tailnet device, whatever
    /etc/resolv.conf says, and answers a reverse lookup of our own Tailscale
    address with our full name. Returns None when not on Tailscale (a laptop
    running the sim), in which case only localhost is printed.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout)
            s.connect(('100.100.100.100', 53))   # UDP: sends nothing, picks a route
            ip = s.getsockname()[0]               # our address on that route
            if not ip.startswith('100.'):
                return None                       # not routed via Tailscale
            # A DNS query: header (id, recursion wanted, 1 question), then
            # "4.3.2.1.in-addr.arpa" as length-prefixed labels, type PTR, class IN.
            labels = ip.split('.')[::-1] + ['in-addr', 'arpa']
            query = (struct.pack('>6H', 0x5eed, 0x0100, 1, 0, 0, 0)
                     + b''.join(bytes([len(p)]) + p.encode() for p in labels)
                     + b'\0' + struct.pack('>2H', 12, 1))
            s.send(query)
            reply = s.recv(512)
    except OSError:
        return None                               # no route, or no answer in time
    try:
        if struct.unpack('>H', reply[6:8])[0] == 0:
            return None                           # no answers
        # The reply repeats our question (same length as ours), then the answer:
        # its name, 10 bytes of type/class/ttl/length, then the name we want.
        _, pos = _read_dns_name(reply, len(query))
        name, _ = _read_dns_name(reply, pos + 10)
        return name or None
    except (IndexError, struct.error, UnicodeDecodeError):
        return None


def _read_dns_name(msg, pos):
    """Decode the DNS name at msg[pos].

    Returns (name, position just after it). A name is length-prefixed labels
    ending in a zero byte, or ending in a 2-byte pointer to the rest of the
    name elsewhere in the message (compression). Servers use either form.
    """
    parts, end = [], None
    for _ in range(64):                           # bound the loop on bad input
        n = msg[pos]
        if n == 0:
            return '.'.join(parts), (end if end is not None else pos + 1)
        if n >= 0xC0:                             # pointer: jump, but the name
            if end is None:                       # ends right after the pointer
                end = pos + 2
            pos = struct.unpack('>H', msg[pos:pos + 2])[0] & 0x3FFF
            continue
        parts.append(msg[pos + 1:pos + 1 + n].decode())
        pos += 1 + n
    raise IndexError('DNS name too long')


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
