# Floortrack

Laptop-side field vision for single-AprilTag external-camera pose, YOLO game-piece detection, floor-plane position estimates, and a custom 2D field interface.

This tool is intentionally separate from `rio/` and does not use the robot's onboard camera transforms. It reads the same 2026 rebuilt AndyMark AprilTag layout from `docs/2026-rebuilt-andymark.json`, then estimates where the laptop/external camera is on the field from one visible tag at a time. Game pieces are detected by a YOLO model exported for NCNN, using Vulkan compute on the laptop GPU when the installed NCNN package supports it.

## Quick Start

From this folder:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e .
.\.venv\Scripts\floortrack.exe --config config\app.example.json
```

Open the printed local URL. The default is:

```text
http://127.0.0.1:5808/
```

For the UI without a camera or Python vision dependencies:

```powershell
py -m floortrack --demo
```

## YOLO Export Format

Export the game-piece detector as NCNN:

```powershell
yolo export model=game_piece.pt format=ncnn imgsz=640
```

Copy the exported directory into:

```text
floortrack/models/game_piece_ncnn_model/
```

The app expects a `.param` and `.bin` file in that directory. If your export uses different input or output blob names, set `input_name` and `output_name` in `config/app.example.json`.

The UI reports whether NCNN actually found a Vulkan GPU. If it says `CPU fallback`, the installed `ncnn` Python package was not built with a usable Vulkan path for this machine, or the Vulkan runtime/device was not visible to NCNN.

For speed testing, the main knobs are:

```json
"map_mode": "auto",
"piece_position_method": "known_width",
"game_piece_diameter_m": 0.2413,
"camera_width": 640,
"camera_height": 480,
"yolo": {
  "input_size": 416,
  "inference_stride": 2,
  "use_vulkan": true,
  "use_fp16": true
}
```

`inference_stride` reuses detections between frames. It makes the UI smoother, but true detector latency is shown in the UI as `yolo ... ms`.

For lab or desk testing, use `map_mode: "auto"` so camera/piece positions outside the FRC field rectangle are still visible. Use `piece_position_method: "known_width"` when a detected game piece has a reliable bounding-box width and a known physical diameter. This estimates distance from `fx * diameter / bbox_width` instead of intersecting a nearly-horizontal ray with the floor.

## Camera Calibration

Pose estimation needs camera intrinsics. The config supports calibrated values:

```json
"intrinsics": {
  "width": 1280,
  "height": 720,
  "fx": 910.0,
  "fy": 910.0,
  "cx": 640.0,
  "cy": 360.0,
  "dist_coeffs": [0, 0, 0, 0, 0]
}
```

If `intrinsics` is null, Floortrack uses the configured horizontal FOV as a rough fallback. That is fine for smoke testing, but measured calibration is needed for useful field coordinates.

## How It Works

- Detects AprilTag 36h11 markers with OpenCV.
- Solves single-tag PnP against the FRC field layout to get the camera pose directly.
- Picks the lowest-reprojection-error tag for camera pose.
- Runs YOLO through NCNN with Vulkan compute enabled by default.
- Projects the bottom-center pixel of each detection through the camera pose onto the floor plane at `z = 0`.
- Serves `/api/state`, `/api/frame.jpg`, and the browser UI from a small local HTTP server.

## Notes

- `tag_size_m` defaults to 0.1651 m, the standard 6.5 inch FRC AprilTag size.
- Floor projection assumes the detected object touches the carpet and that the bottom center of the bounding box is the contact point.
- Low camera angles make floor-plane ray intersections very sensitive to pitch, calibration, and box-bottom errors. For robot use, prefer a higher or downward-tilted camera when possible, and treat this mode as a first prototype unless the geometry is well conditioned.
- The NCNN backend is the preferred AMD iGPU route here. ROCm on Windows laptop integrated GPUs is much more machine-specific, so it is not the default backend.
