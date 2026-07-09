# Model Directory

Place the exported NCNN YOLO model folder here:

```text
floortrack/models/game_piece_ncnn_model/
```

That folder should contain the exported `.param` and `.bin` files.

For DirectML, export an ONNX model instead:

```powershell
yolo export model=game_piece.pt format=onnx imgsz=640 simplify=True
```

Place it at:

```text
floortrack/models/game_piece.onnx
```

The model files are ignored by git so large exports do not get committed.
