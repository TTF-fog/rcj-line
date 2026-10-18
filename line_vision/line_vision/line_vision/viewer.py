import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = b"""<!doctype html><title>line_vision</title>
<body style="margin:0;background:#111"><img src="/stream" style="width:100%;max-width:960px;display:block;margin:auto"></body>"""


class Viewer:
    """MJPEG stream of debug overlays, for checking the camera from a laptop browser."""

    def __init__(self, port):
        self.port = port
        self._jpeg = None
        self._cond = threading.Condition()
        viewer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    viewer._stream(self.wfile)
                else:
                    self.send_error(404)

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

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
                    self._cond.wait_for(lambda: self._jpeg is not None and self._jpeg is not last, timeout=2)
                    jpeg = self._jpeg
                if jpeg is None or jpeg is last:
                    continue
                last = jpeg
                out.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
