from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def package_root() -> Path:
    return Path(__file__).resolve().parent


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _resolve_path(value: str | None, base_dir: Path) -> Path | None:
    if value is None:
        return None

    path = Path(value)
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])

    @staticmethod
    def from_fov(width: int, height: int, horizontal_fov_deg: float) -> "CameraIntrinsics":
        fov_rad = math.radians(horizontal_fov_deg)
        fx = width / (2.0 * math.tan(fov_rad / 2.0))
        return CameraIntrinsics(
            width=width,
            height=height,
            fx=fx,
            fy=fx,
            cx=(width - 1) / 2.0,
            cy=(height - 1) / 2.0,
        )


@dataclass(frozen=True)
class YoloConfig:
    backend: str
    model_dir: Path
    input_size: int
    inference_stride: int
    score_threshold: float
    nms_threshold: float
    classes: list[str]
    target_classes: list[str]
    use_vulkan: bool
    use_fp16: bool
    cpu_threads: int
    input_name: str | None = None
    output_name: str | None = None


@dataclass(frozen=True)
class ServerConfig:
    host: str
    port: int


@dataclass(frozen=True)
class AppConfig:
    camera_source: int | str
    camera_width: int
    camera_height: int
    capture_backend: str | None
    horizontal_fov_deg: float
    intrinsics: CameraIntrinsics
    field_layout_path: Path
    field_image_path: Path | None
    tag_family: str
    tag_size_m: float
    map_mode: str
    piece_position_method: str
    game_piece_diameter_m: float
    pose_stale_after_seconds: float
    yolo: YoloConfig
    server: ServerConfig


def _default_raw_config() -> dict[str, Any]:
    root = repo_root()
    return {
        "camera_source": 0,
        "camera_width": 1280,
        "camera_height": 720,
        "capture_backend": "dshow",
        "horizontal_fov_deg": 70.0,
        "intrinsics": None,
        "field_layout_path": str(root / "docs" / "2026-rebuilt-andymark.json"),
        "field_image_path": str(root / "docs" / "field-layout.png"),
        "tag_family": "DICT_APRILTAG_36h11",
        "tag_size_m": 0.1651,
        "map_mode": "field",
        "piece_position_method": "floor_ray",
        "game_piece_diameter_m": 0.2413,
        "pose_stale_after_seconds": 0.4,
        "yolo": {
            "backend": "ncnn",
            "model_dir": str(root / "floortrack" / "models" / "game_piece_ncnn_model"),
            "input_size": 640,
            "inference_stride": 1,
            "score_threshold": 0.35,
            "nms_threshold": 0.45,
            "classes": ["game_piece"],
            "target_classes": ["game_piece"],
            "use_vulkan": True,
            "use_fp16": True,
            "cpu_threads": 4,
            "input_name": None,
            "output_name": None,
        },
        "server": {
            "host": "127.0.0.1",
            "port": 5808,
        },
    }


def load_config(config_path: str | Path | None = None) -> AppConfig:
    base_dir = repo_root() / "floortrack"
    raw = _default_raw_config()

    if config_path is not None:
        path = Path(config_path).resolve()
        base_dir = path.parent
        with path.open("r", encoding="utf-8") as fp:
            raw = _deep_merge(raw, json.load(fp))

    intrinsics_raw = raw.get("intrinsics")
    if intrinsics_raw is None:
        intrinsics = CameraIntrinsics.from_fov(
            int(raw["camera_width"]),
            int(raw["camera_height"]),
            float(raw["horizontal_fov_deg"]),
        )
    else:
        intrinsics = CameraIntrinsics(
            width=int(intrinsics_raw["width"]),
            height=int(intrinsics_raw["height"]),
            fx=float(intrinsics_raw["fx"]),
            fy=float(intrinsics_raw["fy"]),
            cx=float(intrinsics_raw["cx"]),
            cy=float(intrinsics_raw["cy"]),
            dist_coeffs=[float(v) for v in intrinsics_raw.get("dist_coeffs", [0, 0, 0, 0, 0])],
        )

    yolo_raw = raw["yolo"]
    yolo = YoloConfig(
        backend=str(yolo_raw["backend"]),
        model_dir=_resolve_path(yolo_raw["model_dir"], base_dir) or Path(),
        input_size=int(yolo_raw["input_size"]),
        inference_stride=max(1, int(yolo_raw.get("inference_stride", 1))),
        score_threshold=float(yolo_raw["score_threshold"]),
        nms_threshold=float(yolo_raw["nms_threshold"]),
        classes=[str(v) for v in yolo_raw["classes"]],
        target_classes=[str(v) for v in yolo_raw.get("target_classes", yolo_raw["classes"])],
        use_vulkan=bool(yolo_raw["use_vulkan"]),
        use_fp16=bool(yolo_raw.get("use_fp16", True)),
        cpu_threads=max(1, int(yolo_raw.get("cpu_threads", 4))),
        input_name=yolo_raw.get("input_name"),
        output_name=yolo_raw.get("output_name"),
    )

    server_raw = raw["server"]
    return AppConfig(
        camera_source=raw["camera_source"],
        camera_width=int(raw["camera_width"]),
        camera_height=int(raw["camera_height"]),
        capture_backend=raw.get("capture_backend"),
        horizontal_fov_deg=float(raw["horizontal_fov_deg"]),
        intrinsics=intrinsics,
        field_layout_path=_resolve_path(raw["field_layout_path"], base_dir) or Path(),
        field_image_path=_resolve_path(raw.get("field_image_path"), base_dir),
        tag_family=str(raw["tag_family"]),
        tag_size_m=float(raw["tag_size_m"]),
        map_mode=str(raw.get("map_mode", "field")),
        piece_position_method=str(raw.get("piece_position_method", "floor_ray")),
        game_piece_diameter_m=float(raw.get("game_piece_diameter_m", 0.2413)),
        pose_stale_after_seconds=float(raw["pose_stale_after_seconds"]),
        yolo=yolo,
        server=ServerConfig(host=str(server_raw["host"]), port=int(server_raw["port"])),
    )
