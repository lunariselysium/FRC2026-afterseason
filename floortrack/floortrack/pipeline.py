from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import numpy as np

from .apriltag_pose import AprilTagPoseEstimator, CameraPoseEstimate
from .camera import open_capture
from .config import AppConfig
from .field_layout import load_field_layout
from .geometry import project_bbox_to_known_width, project_pixel_to_floor
from .yolo_directml import YoloDirectMlDetector
from .yolo_ncnn import Detection, YoloNcnnDetector


def create_yolo_detector(config: AppConfig) -> Any:
    if config.yolo.backend.lower() in {"directml", "dml", "onnx"}:
        return YoloDirectMlDetector(config.yolo)
    return YoloNcnnDetector(config.yolo)


class VisionPipeline:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.layout = load_field_layout(config.field_layout_path)
        self.tag_estimator = AprilTagPoseEstimator(
            self.layout,
            config.intrinsics,
            config.tag_size_m,
            config.tag_family,
        )
        self.detector = create_yolo_detector(config)
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_jpeg: bytes | None = None
        self._state: dict[str, Any] = {
            "status": "starting",
            "fps": 0.0,
            "frame": {"width": config.camera_width, "height": config.camera_height},
            "field": self.layout.to_json(),
            "cameraPose": None,
            "cameraPoseStale": True,
            "visibleTags": [],
            "detections": [],
            "pieces": [],
            "display": {"mapMode": config.map_mode},
            "piecePositionMethod": config.piece_position_method,
            "yoloStatus": self.detector.status,
            "timestamp": time.time(),
        }
        self._last_pose: CameraPoseEstimate | None = None
        self._last_pose_timestamp = 0.0
        self._frame_index = 0
        self._last_detections: list[Detection] = []
        self._last_yolo_time_ms = 0.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="floortrack-vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def get_frame_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg

    def _run(self) -> None:
        capture = open_capture(
            self.config.camera_source,
            self.config.camera_width,
            self.config.camera_height,
            self.config.capture_backend,
        )
        if not capture.isOpened():
            self._publish_state({"status": "camera unavailable", "timestamp": time.time()})
            return

        fps = 0.0
        previous_time = time.perf_counter()
        try:
            while self._running.is_set():
                ok, frame = capture.read()
                if not ok:
                    self._publish_state({"status": "frame read failed", "timestamp": time.time()})
                    time.sleep(0.05)
                    continue

                now = time.perf_counter()
                dt = max(now - previous_time, 1e-6)
                previous_time = now
                current_fps = 1.0 / dt
                fps = current_fps if fps == 0.0 else (fps * 0.9 + current_fps * 0.1)

                tag_started = time.perf_counter()
                tag_result = self.tag_estimator.detect(frame)
                tag_time_ms = (time.perf_counter() - tag_started) * 1000.0
                if tag_result.pose is not None:
                    self._last_pose = tag_result.pose
                    self._last_pose_timestamp = time.time()

                pose_age = time.time() - self._last_pose_timestamp
                pose_is_fresh = (
                    self._last_pose is not None
                    and pose_age <= self.config.pose_stale_after_seconds
                )

                self._frame_index += 1
                should_run_yolo = (
                    self._frame_index == 1
                    or self._frame_index % self.config.yolo.inference_stride == 0
                )
                if should_run_yolo:
                    yolo_started = time.perf_counter()
                    detections = self.detector.detect(frame)
                    self._last_yolo_time_ms = (time.perf_counter() - yolo_started) * 1000.0
                    self._last_detections = detections
                else:
                    detections = self._last_detections
                pieces = self._estimate_pieces(detections, self._last_pose if pose_is_fresh else None)
                jpeg_started = time.perf_counter()
                annotated = self._draw_overlay(frame, detections, tag_result.visible_tags, pose_is_fresh)
                jpeg = self._encode_jpeg(annotated)
                jpeg_time_ms = (time.perf_counter() - jpeg_started) * 1000.0

                state = {
                    "status": "running",
                    "fps": round(fps, 1),
                    "frame": {"width": int(frame.shape[1]), "height": int(frame.shape[0])},
                    "field": self.layout.to_json(),
                    "cameraPose": self._last_pose.to_json() if pose_is_fresh and self._last_pose else None,
                    "cameraPoseStale": not pose_is_fresh,
                    "visibleTags": [tag.to_json() for tag in tag_result.visible_tags],
                    "detections": [detection.to_json() for detection in detections],
                    "pieces": pieces,
                    "display": {"mapMode": self.config.map_mode},
                    "piecePositionMethod": self.config.piece_position_method,
                    "yoloStatus": self.detector.status,
                    "timingMs": {
                        "tag": round(tag_time_ms, 1),
                        "yolo": round(self._last_yolo_time_ms, 1),
                        "jpeg": round(jpeg_time_ms, 1),
                    },
                    "timestamp": time.time(),
                }

                with self._lock:
                    self._state = state
                    self._latest_jpeg = jpeg
        finally:
            capture.release()

    def _publish_state(self, updates: dict[str, Any]) -> None:
        with self._lock:
            state = dict(self._state)
            state.update(updates)
            self._state = state

    def _estimate_pieces(
        self,
        detections: list[Detection],
        pose: CameraPoseEstimate | None,
    ) -> list[dict[str, Any]]:
        if pose is None:
            return []

        pieces: list[dict[str, Any]] = []
        for detection in detections:
            method = self.config.piece_position_method
            if method == "known_width":
                point = project_bbox_to_known_width(
                    detection.xyxy,
                    self.config.game_piece_diameter_m,
                    self.config.intrinsics,
                    pose.field_to_camera_rotation,
                    pose.field_to_camera_translation,
                )
            elif method == "hybrid":
                point = project_bbox_to_known_width(
                    detection.xyxy,
                    self.config.game_piece_diameter_m,
                    self.config.intrinsics,
                    pose.field_to_camera_rotation,
                    pose.field_to_camera_translation,
                )
                method = "known_width" if point is not None else "floor_ray"
                if point is None:
                    point = project_pixel_to_floor(
                        detection.bottom_center(),
                        self.config.intrinsics,
                        pose.field_to_camera_rotation,
                        pose.field_to_camera_translation,
                    )
            else:
                method = "floor_ray"
                point = project_pixel_to_floor(
                    detection.bottom_center(),
                    self.config.intrinsics,
                    pose.field_to_camera_rotation,
                    pose.field_to_camera_translation,
                )
            if point is None:
                continue

            x, y, z = point
            on_field = -0.25 <= x <= self.layout.length_m + 0.25 and -0.25 <= y <= self.layout.width_m + 0.25
            pieces.append(
                {
                    "label": detection.label,
                    "score": detection.score,
                    "x": x,
                    "y": y,
                    "z": z,
                    "onField": on_field,
                    "method": method,
                    "sourcePixel": {
                        "x": detection.bottom_center()[0],
                        "y": detection.bottom_center()[1],
                    },
                }
            )
        return pieces

    @staticmethod
    def _encode_jpeg(frame: np.ndarray) -> bytes | None:
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            return None
        return encoded.tobytes()

    @staticmethod
    def _draw_overlay(
        frame: np.ndarray,
        detections: list[Detection],
        visible_tags: list[Any],
        pose_is_fresh: bool,
    ) -> np.ndarray:
        output = frame.copy()

        for tag in visible_tags:
            corners = np.array(tag.corners, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(output, [corners], True, (30, 220, 255), 2)
            label_origin = tuple(corners.reshape(4, 2)[0])
            cv2.putText(
                output,
                f"tag {tag.tag_id}",
                label_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (30, 220, 255),
                2,
                cv2.LINE_AA,
            )

        for detection in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in detection.xyxy]
            cv2.rectangle(output, (x1, y1), (x2, y2), (80, 255, 120), 2)
            cv2.circle(output, (int(round(detection.bottom_center()[0])), int(round(detection.bottom_center()[1]))), 4, (0, 180, 255), -1)
            cv2.putText(
                output,
                f"{detection.label} {detection.score:.2f}",
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (80, 255, 120),
                2,
                cv2.LINE_AA,
            )

        status = "pose live" if pose_is_fresh else "pose stale"
        color = (70, 240, 90) if pose_is_fresh else (80, 120, 255)
        cv2.putText(output, status, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
        return output
