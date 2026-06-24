#!/usr/bin/env python3
"""
train.py — train YOLOv8-nano on the 2-class minibunker dataset (plan.md §10 step 2).

Targets a model small enough to run >=10 FPS on the Pi 5 CPU. Start from the
pretrained yolov8n.pt, ~416 input, light augmentation.

Usage:
    python train.py --data training/datasets/<project>/data.yaml --epochs 100
    # or with the bundled template for a quick smoke run:
    python train.py --data training/data.yaml --epochs 5

Output: runs/detect/train*/weights/best.pt  (export it with export.py).
"""
import argparse
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="path to the YOLOv8 data.yaml")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--project", default=os.path.join(os.path.dirname(__file__), "runs"))
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name="minibunker_yolov8n",
        # light aug — the arena is fairly controlled; avoid over-distorting colour
        hsv_h=0.015, hsv_s=0.5, hsv_v=0.4,
        fliplr=0.5, mosaic=1.0,
        patience=30,
    )
    print("Done. Best weights under:", args.project, "/minibunker_yolov8n*/weights/best.pt")
    print("Next: python export.py --weights <best.pt>")


if __name__ == "__main__":
    main()
