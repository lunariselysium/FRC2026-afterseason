from __future__ import annotations

import cv2


def open_capture(
    source: int | str,
    width: int,
    height: int,
    backend: str | None,
) -> cv2.VideoCapture:
    backend_id = 0
    if backend == "dshow":
        backend_id = cv2.CAP_DSHOW
    elif backend == "msmf":
        backend_id = cv2.CAP_MSMF

    capture = cv2.VideoCapture(source, backend_id) if backend_id else cv2.VideoCapture(source)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return capture
