from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .config import YoloConfig


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    score: float
    xyxy: tuple[float, float, float, float]

    def bottom_center(self) -> tuple[float, float]:
        x1, _, x2, y2 = self.xyxy
        return ((x1 + x2) / 2.0, y2)

    def to_json(self) -> dict[str, Any]:
        x1, y1, x2, y2 = self.xyxy
        return {
            "classId": self.class_id,
            "label": self.label,
            "score": self.score,
            "bbox": {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
            },
        }


class YoloNcnnDetector:
    def __init__(self, config: YoloConfig) -> None:
        self.config = config
        self.enabled = False
        self.status = "disabled"
        self.net = None
        self.input_name = config.input_name
        self.output_name = config.output_name
        self._ncnn = None
        self.using_vulkan = False
        self.gpu_count: int | None = None
        self.target_class_ids = {
            idx for idx, label in enumerate(config.classes) if label in set(config.target_classes)
        }

        if config.backend.lower() != "ncnn":
            self.status = f"unsupported backend: {config.backend}"
            return

        try:
            import ncnn  # type: ignore
        except ImportError:
            self.status = "ncnn is not installed"
            return

        requested_vulkan = config.use_vulkan
        actual_vulkan = False
        if requested_vulkan:
            actual_vulkan = self._initialize_vulkan(ncnn)
            if not actual_vulkan:
                self.status = "Vulkan requested, but NCNN did not report a GPU; falling back to CPU"

        param_path, bin_path = self._find_model_files(config.model_dir)
        if param_path is None or bin_path is None:
            self.status = f"missing .param/.bin in {config.model_dir}"
            return

        inferred_input, inferred_output = self._infer_blob_names(param_path)
        self.input_name = self.input_name or inferred_input or "in0"
        self.output_name = self.output_name or inferred_output or "out0"

        net = ncnn.Net()
        self._configure_options(net.opt, actual_vulkan)
        if net.load_param(str(param_path)) != 0:
            self.status = f"failed to load {param_path.name}"
            return
        if net.load_model(str(bin_path)) != 0:
            self.status = f"failed to load {bin_path.name}"
            return

        self._ncnn = ncnn
        self.net = net
        self.enabled = True
        self.using_vulkan = actual_vulkan
        compute = self._compute_label(requested_vulkan, actual_vulkan)
        self.status = f"ready ({compute}, input={self.input_name}, output={self.output_name})"

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        if not self.enabled or self.net is None or self._ncnn is None:
            return []

        input_image, scale, pad_x, pad_y = self._letterbox(frame_bgr, self.config.input_size)
        mat_in = self._ncnn.Mat.from_pixels(
            input_image,
            self._ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            self.config.input_size,
            self.config.input_size,
        )
        mat_in.substract_mean_normalize([], [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0])

        extractor = self.net.create_extractor()
        extractor.input(self.input_name, mat_in)
        ret, mat_out = extractor.extract(self.output_name)
        if ret != 0:
            self.status = f"extract failed for output blob {self.output_name}"
            return []

        output = np.array(mat_out)
        return self._decode(output, frame_bgr.shape[:2], scale, pad_x, pad_y)

    def _initialize_vulkan(self, ncnn: Any) -> bool:
        try:
            if hasattr(ncnn, "create_gpu_instance"):
                ncnn.create_gpu_instance()

            if hasattr(ncnn, "get_gpu_count"):
                self.gpu_count = int(ncnn.get_gpu_count())
                return self.gpu_count > 0

            if hasattr(ncnn, "get_gpu_device_count"):
                self.gpu_count = int(ncnn.get_gpu_device_count())
                return self.gpu_count > 0

            self.gpu_count = None
            return True
        except Exception as exc:
            self.status = f"Vulkan init failed: {exc}; falling back to CPU"
            self.gpu_count = 0
            return False

    def _configure_options(self, options: Any, use_vulkan: bool) -> None:
        settings = {
            "use_vulkan_compute": use_vulkan,
            "num_threads": self.config.cpu_threads,
            "lightmode": True,
            "use_packing_layout": True,
            "use_fp16_packed": self.config.use_fp16,
            "use_fp16_storage": self.config.use_fp16,
            "use_fp16_arithmetic": self.config.use_fp16,
        }
        for name, value in settings.items():
            if hasattr(options, name):
                setattr(options, name, value)

    def _compute_label(self, requested_vulkan: bool, actual_vulkan: bool) -> str:
        if actual_vulkan:
            if self.gpu_count is None:
                return "Vulkan requested, GPU count unknown"
            return f"Vulkan, {self.gpu_count} GPU(s)"
        if requested_vulkan:
            return "CPU fallback"
        return "CPU"

    @staticmethod
    def _find_model_files(model_dir: Path) -> tuple[Path | None, Path | None]:
        if not model_dir.exists():
            return None, None
        params = sorted(model_dir.glob("*.param"))
        bins = sorted(model_dir.glob("*.bin"))
        return (params[0] if params else None, bins[0] if bins else None)

    @staticmethod
    def _infer_blob_names(param_path: Path) -> tuple[str | None, str | None]:
        input_name: str | None = None
        output_name: str | None = None

        for raw_line in param_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                bottom_count = int(parts[2])
                top_count = int(parts[3])
            except ValueError:
                continue

            top_start = 4 + bottom_count
            tops = parts[top_start:top_start + top_count]
            if parts[0] == "Input" and tops:
                input_name = tops[0]
            elif tops:
                output_name = tops[-1]

        return input_name, output_name

    @staticmethod
    def _letterbox(
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

        output = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return output, scale, float(left), float(top)

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

        boxes: np.ndarray
        scores: np.ndarray
        class_ids: np.ndarray
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
