from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .config import YoloConfig
from .yolo_ncnn import Detection


class YoloDirectMlDetector:
    def __init__(self, config: YoloConfig) -> None:
        self.config = config
        self.enabled = False
        self.status = "disabled"
        self.session = None
        self.input_name = config.input_name
        self.output_name = config.output_name
        self.target_class_ids = {
            idx for idx, label in enumerate(config.classes) if label in set(config.target_classes)
        }

        if config.backend.lower() not in {"directml", "dml", "onnx"}:
            self.status = f"unsupported backend: {config.backend}"
            return

        try:
            import onnxruntime as ort  # type: ignore
        except ImportError:
            self.status = "onnxruntime-directml is not installed"
            return

        available_providers = ort.get_available_providers()
        if "DmlExecutionProvider" not in available_providers:
            self.status = f"DirectML unavailable; providers={available_providers}"
            return

        model_path = self._find_model_path(config.model_path, config.model_dir)
        if model_path is None:
            self.status = f"missing ONNX model at {config.model_path or config.model_dir}"
            return

        providers: list[Any] = [
            ("DmlExecutionProvider", {"device_id": str(config.directml_device_id)}),
            "CPUExecutionProvider",
        ]
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        try:
            session = ort.InferenceSession(
                str(model_path),
                sess_options=session_options,
                providers=providers,
            )
        except Exception as exc:
            self.status = f"failed to create DirectML session: {exc}"
            return

        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if not inputs or not outputs:
            self.status = "ONNX model has no visible inputs or outputs"
            return

        self.input_name = self.input_name or inputs[0].name
        if self.output_name is None:
            self.output_name = outputs[0].name
        actual_providers = session.get_providers()

        self.session = session
        self.enabled = True
        self.status = (
            f"ready (DirectML, providers={actual_providers}, "
            f"input={self.input_name}, output={self.output_name}, model={model_path.name})"
        )

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        if not self.enabled or self.session is None or self.input_name is None:
            return []

        input_tensor, scale, pad_x, pad_y = self._preprocess(frame_bgr, self.config.input_size)
        try:
            outputs = self.session.run(
                [self.output_name] if self.output_name else None,
                {self.input_name: input_tensor},
            )
        except Exception as exc:
            self.status = f"DirectML inference failed: {exc}"
            return []

        if not outputs:
            return []
        return self._decode(np.asarray(outputs[0]), frame_bgr.shape[:2], scale, pad_x, pad_y)

    @staticmethod
    def _find_model_path(configured_path: Path | None, model_dir: Path) -> Path | None:
        if configured_path is not None and configured_path.exists():
            return configured_path

        candidates: list[Path] = []
        if model_dir.exists():
            candidates.extend(sorted(model_dir.glob("*.onnx")))
            candidates.extend(sorted(model_dir.glob("**/*.onnx")))

        if configured_path is not None and configured_path.parent.exists():
            candidates.extend(sorted(configured_path.parent.glob("*.onnx")))

        return candidates[0] if candidates else None

    @staticmethod
    def _preprocess(
        frame_bgr: np.ndarray,
        input_size: int,
    ) -> tuple[np.ndarray, float, float, float]:
        height, width = frame_bgr.shape[:2]
        scale = min(input_size / width, input_size / height)
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(frame_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

        pad_x = (input_size - resized_width) / 2.0
        pad_y = (input_size - resized_height) / 2.0
        left = int(round(pad_x - 0.1))
        right = int(round(pad_x + 0.1))
        top = int(round(pad_y - 0.1))
        bottom = int(round(pad_y + 0.1))
        letterboxed = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[np.newaxis, ...]
        return np.ascontiguousarray(tensor), scale, float(left), float(top)

    def _decode(
        self,
        output: np.ndarray,
        original_hw: tuple[int, int],
        scale: float,
        pad_x: float,
        pad_y: float,
    ) -> list[Detection]:
        pred = np.asarray(output, dtype=np.float32).squeeze()
        if pred.ndim == 1:
            pred = pred.reshape(1, -1)
        if pred.ndim != 2 or pred.size == 0:
            return []

        if pred.shape[0] < pred.shape[1] and pred.shape[0] <= max(128, len(self.config.classes) + 5):
            pred = pred.T

        columns = pred.shape[1]
        if columns == 6:
            boxes = pred[:, :4]
            scores = pred[:, 4]
            class_ids = pred[:, 5].astype(np.int32)
            xyxy_already = True
        elif columns >= 5 + len(self.config.classes):
            boxes = pred[:, :4]
            objectness = pred[:, 4]
            class_scores = pred[:, 5:5 + len(self.config.classes)]
            class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
            scores = objectness * class_scores[np.arange(class_scores.shape[0]), class_ids]
            xyxy_already = False
        else:
            boxes = pred[:, :4]
            class_scores = pred[:, 4:]
            if class_scores.shape[1] == 0:
                return []
            class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
            scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
            xyxy_already = False

        keep = scores >= self.config.score_threshold
        if self.target_class_ids:
            keep &= np.array([int(class_id) in self.target_class_ids for class_id in class_ids])

        boxes = boxes[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]
        if len(boxes) == 0:
            return []

        if np.nanmax(boxes) <= 1.5:
            boxes = boxes * float(self.config.input_size)

        if not xyxy_already:
            cx = boxes[:, 0]
            cy = boxes[:, 1]
            w = boxes[:, 2]
            h = boxes[:, 3]
            boxes = np.stack(
                [
                    cx - w / 2.0,
                    cy - h / 2.0,
                    cx + w / 2.0,
                    cy + h / 2.0,
                ],
                axis=1,
            )

        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale

        original_h, original_w = original_hw
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, original_w - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, original_h - 1)

        selected = self._nms(boxes, scores, self.config.nms_threshold)
        detections: list[Detection] = []
        for idx in selected:
            class_id = int(class_ids[idx])
            label = self.config.classes[class_id] if class_id < len(self.config.classes) else str(class_id)
            detections.append(
                Detection(
                    class_id=class_id,
                    label=label,
                    score=float(scores[idx]),
                    xyxy=tuple(float(v) for v in boxes[idx]),
                )
            )
        return detections

    @staticmethod
    def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> list[int]:
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        selected: list[int] = []

        while order.size > 0:
            current = int(order[0])
            selected.append(current)
            if order.size == 1:
                break

            rest = order[1:]
            xx1 = np.maximum(x1[current], x1[rest])
            yy1 = np.maximum(y1[current], y1[rest])
            xx2 = np.minimum(x2[current], x2[rest])
            yy2 = np.minimum(y2[current], y2[rest])
            intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[current] + areas[rest] - intersection
            iou = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
            order = rest[iou <= threshold]

        return selected
