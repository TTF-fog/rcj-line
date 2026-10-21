import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

PAGE = Path(__file__).with_name("dash.html")


class Dash:
    """Browser dashboard: camera stream, run state as JSON, and the start/LoP and new-run buttons."""

    def __init__(self, port, state, press, hold):
        self.port = port
        self._jpeg = None
        self._cond = threading.Condition()
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, body, content_type):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/":
                    self.reply(PAGE.read_bytes(), "text/html")
                elif path == "/state":
                    self.reply(json.dumps(state()).encode(), "application/json")
                elif path == "/frame.jpg":
                    jpeg = dash._jpeg
                    if jpeg is None:
                        self.send_response(204)
                        self.end_headers()
                    else:
                        self.reply(jpeg, "image/jpeg")
                elif path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    dash._stream(self.wfile)
                else:
                    self.send_error(404)

            def do_POST(self):
                action = {"/press": press, "/hold": hold}.get(self.path)
                if action is None:
                    self.send_error(404)
                    return
                action()
                self.reply(b"ok", "text/plain")

        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def url(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            host = s.getsockname()[0]
        except OSError:
            host = socket.gethostname()
        finally:
            s.close()
        return f"http://{host}:{self.port}/"

    def publish(self, image_bgr):
        ok, buf = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self._cond:
                self._jpeg = buf.tobytes()
                self._cond.notify_all()

    def _stream(self, out):
        last = None
        try:
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._jpeg is not last, timeout=2)
                    jpeg = self._jpeg
                if jpeg is None or jpeg is last:
                    continue
                last = jpeg
                out.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
