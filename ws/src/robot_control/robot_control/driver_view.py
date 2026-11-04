"""A live view of what square_driver believes, in a browser.

The simulator's page at :8080 shows where the robot really is, and only exists
in simulation. This one shows what the driver believes -- where it thinks it is,
what the lidar sees, its map and its plan -- and works the same on the real
robot, from any laptop that can reach the Pi.

It only ever reads: there are no driving controls, because rule R8 bans remote
control during a match. Standard library only, like robot_sim's viz_server.

The page gets two kinds of Server-Sent Event: `state`, several times a second,
and `map`, which is bigger and changes slowly, about once a second.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = os.path.join(os.path.dirname(__file__), 'web', 'driver_view.html')


class DriverView:

    def __init__(self, port):
        self.port = port
        self._lock = threading.Lock()
        self._state = None
        self._map = None
        self._map_version = 0
        self._httpd = None

    def set_state(self, state):
        blob = json.dumps(state, separators=(',', ':'))
        with self._lock:
            self._state = blob

    def set_map(self, grid):
        blob = json.dumps(grid, separators=(',', ':'))
        with self._lock:
            self._map = blob
            self._map_version += 1

    def _read(self):
        with self._lock:
            return self._state, self._map, self._map_version

    def start(self):
        self._httpd = ThreadingHTTPServer(('0.0.0.0', self.port), _make_handler(self))
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()


def _make_handler(view):

    class Handler(BaseHTTPRequestHandler):

        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass                          # keep the ROS console readable

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                with open(PAGE, 'rb') as f:
                    page = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == '/stream':
                self._stream()
            else:
                self.send_response(404)
                self.send_header('Content-Length', '0')
                self.end_headers()

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            sent_map = -1
            try:
                while True:
                    state, grid, version = view._read()
                    if grid is not None and version != sent_map:
                        self.wfile.write(f'event: map\ndata: {grid}\n\n'.encode())
                        sent_map = version
                    if state is not None:
                        self.wfile.write(f'event: state\ndata: {state}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass                      # browser tab closed; nothing to do

    return Handler
