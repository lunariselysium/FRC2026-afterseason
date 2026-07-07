from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from .config import CameraIntrinsics
from .field_layout import FieldLayout
from .geometry import camera_matrix, dist_coeffs, pose_to_json, reprojection_error_px


@dataclass(frozen=True)
class VisibleTag:
    tag_id: int
    corners: list[list[float]]
    reprojection_error_px: float | None = None
    camera_to_tag_m: float | None = None

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.tag_id,
            "corners": self.corners,
            "reprojectionErrorPx": self.reprojection_error_px,
            "cameraToTagM": self.camera_to_tag_m,
        }


@dataclass(frozen=True)
class CameraPoseEstimate:
    field_to_camera_rotation: np.ndarray
    field_to_camera_translation: np.ndarray
    tag_id: int
    reprojection_error_px: float

    def to_json(self) -> dict[str, Any]:
        payload = pose_to_json(self.field_to_camera_rotation, self.field_to_camera_translation)
        payload["tagId"] = self.tag_id
        payload["reprojectionErrorPx"] = self.reprojection_error_px
        return payload


@dataclass(frozen=True)
class AprilTagDetectionResult:
    pose: CameraPoseEstimate | None
    visible_tags: list[VisibleTag]


class AprilTagPoseEstimator:
    # OpenCV square-marker coordinates: x right, y up, z out of tag.
    # WPILib AprilTag coordinates: x out of tag, y left, z up.
    _WPILIB_TAG_FROM_OPENCV_TAG = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    _OPENCV_TAG_FROM_WPILIB_TAG = _WPILIB_TAG_FROM_OPENCV_TAG.T

    def __init__(
        self,
        field_layout: FieldLayout,
        intrinsics: CameraIntrinsics,
        tag_size_m: float,
        tag_family: str,
    ) -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was installed without aruco support. Install opencv-contrib-python.")

        dictionary_id = getattr(cv2.aruco, tag_family)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        half = tag_size_m / 2.0
        self.object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        self.layout = field_layout
        self.camera_matrix = camera_matrix(intrinsics)
        self.dist_coeffs = dist_coeffs(intrinsics)

    def detect(self, frame_bgr: np.ndarray) -> AprilTagDetectionResult:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = self.detector.detectMarkers(gray)

        if marker_ids is None:
            return AprilTagDetectionResult(pose=None, visible_tags=[])

        best_pose: CameraPoseEstimate | None = None
        visible_tags: list[VisibleTag] = []

        for corners_raw, raw_id in zip(marker_corners, marker_ids.flatten()):
            tag_id = int(raw_id)
            corners = corners_raw.reshape(4, 2).astype(np.float64)
            tag_pose = self.layout.get_tag(tag_id)
            if tag_pose is None:
                visible_tags.append(VisibleTag(tag_id=tag_id, corners=corners.tolist()))
                continue

            ok, rvec, tvec = cv2.solvePnP(
                self.object_points,
                corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                visible_tags.append(VisibleTag(tag_id=tag_id, corners=corners.tolist()))
                continue

            projected, _ = cv2.projectPoints(
                self.object_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            reprojection_error = reprojection_error_px(corners, projected)
            camera_to_tag_m = float(np.linalg.norm(tvec.reshape(3)))

            rotation_camera_from_opencv_tag, _ = cv2.Rodrigues(rvec)
            rotation_camera_from_wpilib_tag = (
                rotation_camera_from_opencv_tag @ self._OPENCV_TAG_FROM_WPILIB_TAG
            )

            rotation_field_from_tag = np.array(tag_pose.rotation, dtype=np.float64)
            translation_field_from_tag = np.array(tag_pose.translation, dtype=np.float64)
            translation_camera_from_tag = tvec.reshape(3).astype(np.float64)

            rotation_field_from_camera = rotation_field_from_tag @ rotation_camera_from_wpilib_tag.T
            translation_field_from_camera = (
                translation_field_from_tag
                - rotation_field_from_camera @ translation_camera_from_tag
            )

            estimate = CameraPoseEstimate(
                field_to_camera_rotation=rotation_field_from_camera,
                field_to_camera_translation=translation_field_from_camera,
                tag_id=tag_id,
                reprojection_error_px=reprojection_error,
            )

            visible_tags.append(
                VisibleTag(
                    tag_id=tag_id,
                    corners=corners.tolist(),
                    reprojection_error_px=reprojection_error,
                    camera_to_tag_m=camera_to_tag_m,
                )
            )

            if best_pose is None or estimate.reprojection_error_px < best_pose.reprojection_error_px:
                best_pose = estimate

        return AprilTagDetectionResult(pose=best_pose, visible_tags=visible_tags)
