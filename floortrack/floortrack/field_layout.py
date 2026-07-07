from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


Matrix3 = list[list[float]]
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class TagPose:
    tag_id: int
    translation: Vector3
    rotation: Matrix3

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.tag_id,
            "x": self.translation[0],
            "y": self.translation[1],
            "z": self.translation[2],
        }


@dataclass(frozen=True)
class FieldLayout:
    length_m: float
    width_m: float
    tags: dict[int, TagPose]

    def get_tag(self, tag_id: int) -> TagPose | None:
        return self.tags.get(tag_id)

    def to_json(self) -> dict[str, Any]:
        return {
            "lengthM": self.length_m,
            "widthM": self.width_m,
            "tags": [tag.to_json() for tag in sorted(self.tags.values(), key=lambda item: item.tag_id)],
        }


def quaternion_to_matrix(w: float, x: float, y: float, z: float) -> Matrix3:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError("Cannot convert a zero-length quaternion to a rotation matrix.")

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def load_field_layout(path: str | Path) -> FieldLayout:
    layout_path = Path(path)
    with layout_path.open("r", encoding="utf-8") as fp:
        raw = json.load(fp)

    field = raw["field"]
    tags: dict[int, TagPose] = {}
    for raw_tag in raw["tags"]:
        pose = raw_tag["pose"]
        translation = pose["translation"]
        quaternion = pose["rotation"]["quaternion"]
        tag_id = int(raw_tag["ID"])
        tags[tag_id] = TagPose(
            tag_id=tag_id,
            translation=(
                float(translation["x"]),
                float(translation["y"]),
                float(translation["z"]),
            ),
            rotation=quaternion_to_matrix(
                float(quaternion["W"]),
                float(quaternion["X"]),
                float(quaternion["Y"]),
                float(quaternion["Z"]),
            ),
        )

    return FieldLayout(
        length_m=float(field["length"]),
        width_m=float(field["width"]),
        tags=tags,
    )
