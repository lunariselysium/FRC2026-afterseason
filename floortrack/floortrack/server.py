from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
from typing import Any, Protocol
from urllib.parse import unquote, urlparse

from .config import AppConfig, package_root


class StateProvider(Protocol):
    def get_state(self) -> dict[str, Any]:
        ...

    def get_frame_jpeg(self) -> bytes | None:
        ...


class FloortrackServer(ThreadingHTTPServer):
    def __init__(self, config: AppConfig, provider: StateProvider) -> None:
        super().__init__((config.server.host, config.server.port), FloortrackRequestHandler)
        self.provider = provider
        self.static_root = package_root() / "static"
        self.field_image_path = config.field_image_path


class FloortrackRequestHandler(BaseHTTPRequestHandler):
    server: FloortrackServer

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(self.server.provider.get_state())
            return
        if parsed.path == "/api/frame.jpg":
            frame = self.server.provider.get_frame_jpeg()
            if frame is None:
                self._send_status(404, "no frame")
            else:
                self._send_bytes(frame, "image/jpeg")
            return
        if parsed.path == "/api/field-image":
            image_path = self.server.field_image_path
            if image_path is None or not image_path.exists():
                self._send_status(404, "no field image")
            else:
                self._send_file(image_path)
            return

        path = "/index.html" if parsed.path == "/" else parsed.path
        self._send_static(path)

    def _send_static(self, request_path: str) -> None:
        relative = unquote(request_path).lstrip("/")
        target = (self.server.static_root / relative).resolve()
        try:
            target.relative_to(self.server.static_root)
        except ValueError:
            self._send_status(403, "forbidden")
            return

        if not target.exists() or not target.is_file():
            self._send_status(404, "not found")
            return

        self._send_file(target)

    def _send_file(self, path: Path) -> None:
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self._send_bytes(path.read_bytes(), content_type)

    def _send_json(self, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self._send_bytes(body, "application/json; charset=utf-8")

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_status(self, status: int, message: str) -> None:
        body = message.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
