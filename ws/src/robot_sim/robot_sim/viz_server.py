"""A tiny web server so you can watch the robot in a browser.

Why a browser and not RViz: RViz needs an X11 display, which is a different
painful problem on Mac, Windows and Linux. Every machine already has a browser,
and a browser talks to localhost the same way everywhere.

This uses only the Python standard library -- no pip install, no rosbridge.
It streams robot state to the page with Server-Sent Events (a plain HTTP
response that never closes) and takes arrow-key commands back via POST.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_DIR = os.path.join(os.path.dirname(__file__), 'web')


class VizServer:

    def __init__(self, port, world):
        self.port = port
        self.world = world
        self._state = {}
        self._lock = threading.Lock()
        self._command = None
        self._httpd = None
        self._thread = None

    # ------------------------------------------------- called by the sim node

    def update(self, state):
        with self._lock:
            self._state = state

    def take_command(self):
        """Pop a pending arrow-key command from the browser, if any."""
        with self._lock:
            cmd, self._command = self._command, None
        return cmd

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def put_command(self, cmd):
        with self._lock:
            self._command = cmd

    # ----------------------------------------------------------- server plumbing

    def start(self):
        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer(('0.0.0.0', self.port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()


def _make_handler(viz):

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
            if self.path in ('/', '/index.html'):
                try:
                    with open(os.path.join(WEB_DIR, 'index.html'), 'rb') as f:
                        self._send(200, f.read(), 'text/html; charset=utf-8')
                except FileNotFoundError:
                    self._send(404, 'index.html missing', 'text/plain')

            elif self.path == '/world':
                self._send(200, json.dumps(viz.world), 'application/json')

            elif self.path == '/stream':
                self._stream()

            else:
                self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            if self.path != '/cmd':
                self._send(404, 'not found', 'text/plain')
                return
            length = int(self.headers.get('Content-Length', 0))
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
                viz.put_command((
                    float(data.get('vx', 0.0)),
                    float(data.get('vy', 0.0)),
                    float(data.get('wz', 0.0)),
                ))
                self._send(200, '{"ok":true}', 'application/json')
            except (ValueError, TypeError):
                self._send(400, '{"ok":false}', 'application/json')

        def _stream(self):
            """Server-Sent Events: one JSON state blob per frame, forever."""
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    blob = json.dumps(viz.snapshot())
                    self.wfile.write(f'data: {blob}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.05)      # 20 fps is plenty
            except (BrokenPipeError, ConnectionResetError):
                pass                      # browser tab closed; nothing to do

    return Handler
