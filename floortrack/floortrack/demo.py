from __future__ import annotations

import math
import time
from typing import Any

from .config import AppConfig
from .field_layout import load_field_layout


class DemoProvider:
    def __init__(self, config: AppConfig) -> None:
        self.layout = load_field_layout(config.field_layout_path)
        self.started_at = time.time()

    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def get_frame_jpeg(self) -> bytes | None:
        return None

    def get_state(self) -> dict[str, Any]:
        elapsed = time.time() - self.started_at
        x = self.layout.length_m * (0.5 + 0.22 * math.sin(elapsed * 0.33))
        y = self.layout.width_m * (0.5 + 0.24 * math.cos(elapsed * 0.41))
        yaw = (elapsed * 24.0) % 360.0 - 180.0
        pieces = []
        for index in range(5):
            pieces.append(
                {
                    "label": "game_piece",
                    "score": 0.75 + 0.04 * index,
                    "x": self.layout.length_m * (0.2 + 0.12 * index),
                    "y": self.layout.width_m * (0.35 + 0.2 * math.sin(elapsed * 0.5 + index)),
                    "z": 0.0,
                    "onField": True,
                    "method": "demo",
                    "sourcePixel": {"x": 0.0, "y": 0.0},
                }
            )

        return {
            "status": "demo",
            "fps": 30.0,
            "frame": {"width": 1280, "height": 720},
            "field": self.layout.to_json(),
            "cameraPose": {"x": x, "y": y, "z": 1.0, "yawDeg": yaw, "tagId": 18, "reprojectionErrorPx": 0.4},
            "cameraPoseStale": False,
            "visibleTags": [{"id": 18, "corners": [], "reprojectionErrorPx": 0.4, "cameraToTagM": 3.2}],
            "detections": [],
            "pieces": pieces,
            "display": {"mapMode": "field"},
            "piecePositionMethod": "demo",
            "yoloStatus": "demo",
            "timestamp": time.time(),
        }
