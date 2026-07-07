from __future__ import annotations

from typing import Any
import math

import numpy as np

from .config import CameraIntrinsics


def camera_matrix(intrinsics: CameraIntrinsics) -> np.ndarray:
    return np.array(
        [
            [intrinsics.fx, 0.0, intrinsics.cx],
            [0.0, intrinsics.fy, intrinsics.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def dist_coeffs(intrinsics: CameraIntrinsics) -> np.ndarray:
    return np.array(intrinsics.dist_coeffs, dtype=np.float64).reshape(-1, 1)


def project_pixel_to_floor(
    pixel: tuple[float, float],
    intrinsics: CameraIntrinsics,
    field_to_camera_rotation: np.ndarray,
    field_to_camera_translation: np.ndarray,
    floor_z_m: float = 0.0,
) -> tuple[float, float, float] | None:
    u, v = pixel
    ray_camera = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ],
        dtype=np.float64,
    )
    ray_camera /= np.linalg.norm(ray_camera)

    origin_field = field_to_camera_translation.astype(np.float64)
    ray_field = field_to_camera_rotation @ ray_camera
    if abs(ray_field[2]) < 1e-9:
        return None

    scale = (floor_z_m - origin_field[2]) / ray_field[2]
    if scale <= 0.0:
        return None

    point = origin_field + scale * ray_field
    return (float(point[0]), float(point[1]), float(point[2]))


def project_bbox_to_known_width(
    bbox_xyxy: tuple[float, float, float, float],
    physical_width_m: float,
    intrinsics: CameraIntrinsics,
    field_to_camera_rotation: np.ndarray,
    field_to_camera_translation: np.ndarray,
) -> tuple[float, float, float] | None:
    x1, y1, x2, y2 = bbox_xyxy
    width_px = max(1.0, x2 - x1)
    if physical_width_m <= 0.0:
        return None

    u = (x1 + x2) / 2.0
    v = (y1 + y2) / 2.0
    z_camera = intrinsics.fx * physical_width_m / width_px
    if z_camera <= 0.0 or not np.isfinite(z_camera):
        return None

    point_camera = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx * z_camera,
            (v - intrinsics.cy) / intrinsics.fy * z_camera,
            z_camera,
        ],
        dtype=np.float64,
    )
    point_field = field_to_camera_translation + field_to_camera_rotation @ point_camera
    return (float(point_field[0]), float(point_field[1]), 0.0)


def pose_to_json(
    field_to_camera_rotation: np.ndarray,
    field_to_camera_translation: np.ndarray,
) -> dict[str, Any]:
    forward = field_to_camera_rotation @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    yaw_deg = math.degrees(math.atan2(float(forward[1]), float(forward[0])))
    return {
        "x": float(field_to_camera_translation[0]),
        "y": float(field_to_camera_translation[1]),
        "z": float(field_to_camera_translation[2]),
        "yawDeg": yaw_deg,
    }


def reprojection_error_px(
    observed_corners: np.ndarray,
    projected_corners: np.ndarray,
) -> float:
    observed = observed_corners.reshape(-1, 2)
    projected = projected_corners.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(observed - projected, axis=1)))
