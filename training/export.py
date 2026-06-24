#!/usr/bin/env python3
"""
export.py — export a trained YOLOv8n to a Pi-friendly runtime (plan.md §10 step 4).

ncnn is the fastest CPU runtime on ARM; onnx is the portable default the
detector loads via onnxruntime. Export one or both, then drop the result into
the perception package's models/ dir and point detector.cnn.weights at it.

Usage:
    python export.py --weights training/runs/spacemine_yolov8n/weights/best.pt --format onnx
    python export.py --weights ... --format ncnn

The exported file is copied to:
    catkin_ws/src/minibunker_perception/models/minibunker_yolov8n.<ext>
which is the path config/minibunker.yaml -> detector.cnn.weights expects inside
the container.
"""
import argparse
import os
import shutil

PKG_MODELS = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "catkin_ws", "src",
    "minibunker_perception", "models"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--format", choices=["onnx", "ncnn"], default="onnx")
    ap.add_argument("--imgsz", type=int, default=416)
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    out = model.export(format=args.format, imgsz=args.imgsz, opset=12)

    os.makedirs(PKG_MODELS, exist_ok=True)
    # ncnn export is a folder; onnx is a single file
    if args.format == "onnx":
        dst = os.path.join(PKG_MODELS, "minibunker_yolov8n.onnx")
        shutil.copy(out, dst)
    else:
        dst = os.path.join(PKG_MODELS, "minibunker_yolov8n_ncnn_model")
        if os.path.isdir(out):
            shutil.copytree(out, dst, dirs_exist_ok=True)
        else:
            shutil.copy(out, dst)
    print("Exported %s -> %s" % (args.format, dst))
    print("Set config/minibunker.yaml -> detector.cnn.runtime accordingly "
          "(onnxruntime for .onnx, ncnn for the ncnn folder) and weights to the path above.")


if __name__ == "__main__":
    main()
