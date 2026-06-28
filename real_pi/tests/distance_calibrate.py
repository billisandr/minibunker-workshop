#!/usr/bin/env python3
# ============================================================================
#  distance_calibrate.py — calibrate the pixel distance estimator from the CLI.
#
#  Place the object (green ball or cone) at a KNOWN distance, hold it steady with
#  a clean detection, and this records its bbox height (px) -> the reference. The
#  UI Calibration tab does the same thing; this is the no-browser path.
#
#  Usage (on the Pi, venv active, from real_pi/):
#     python tests/distance_calibrate.py --class green_ball --dist 1.0
#     python tests/distance_calibrate.py --class cone --dist 0.8 --frames 30
#
#  Prints the config.yaml snippet to paste into detector/distance.
# ============================================================================
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from minibunker_real.config import Config              # noqa: E402
from minibunker_real.camera import Camera              # noqa: E402
from minibunker_real.detector import NAME_TO_CLASS, make_detector  # noqa: E402
from minibunker_real.distance import DistanceEstimator  # noqa: E402


def _bbox_h_px(dets, cls):
    hs = [d[4] for d in dets if d[0] == cls]
    return max(hs) if hs else 0


def main():
    ap = argparse.ArgumentParser(description="Calibrate pixel distance")
    ap.add_argument("--class", dest="cls", default="green_ball",
                    choices=["green_ball", "cone"])
    ap.add_argument("--dist", type=float, required=True, help="known distance (m)")
    ap.add_argument("--frames", type=int, default=20,
                    help="frames to median over (default 20)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = Config(args.config)
    cam = Camera(cfg)
    backend, det = make_detector(cfg.block("detector"))
    if backend != "hsv":
        print(f"[calib] backend is '{backend}', not hsv — detection sizes may differ.")
    cls_int = NAME_TO_CLASS[args.cls]

    print(f"[calib] hold the {args.cls} at {args.dist} m, steady, in clean view…")
    heights = []
    for i in range(args.frames):
        ok, frame = cam.read()
        if not ok:
            continue
        dets, _ = det.detect(frame)
        h = _bbox_h_px(dets, cls_int)
        if h > 0:
            heights.append(h)
        print(f"  frame {i:2d}: bbox height = {h} px", end="\r")
    cam.close()
    print()

    if not heights:
        print("[calib] no detections — fix the HSV mask / lighting first. Aborting.")
        sys.exit(1)
    heights.sort()
    ref_h = heights[len(heights) // 2]      # median, robust to flicker
    est = DistanceEstimator(cfg.block("detector/distance", {}))
    est.calibrate(args.cls, args.dist, ref_h)
    print(f"[calib] {args.cls}: ref_height_px = {ref_h} px @ {args.dist} m "
          f"({len(heights)}/{args.frames} frames had a detection)")
    print("\nPaste into config.yaml under detector/distance:")
    print(f"    {args.cls}: {{ ref_distance_m: {args.dist}, ref_height_px: {ref_h} }}")
    print(f"\nCheck: at {ref_h}px -> {est.estimate(args.cls, ref_h):.2f} m, "
          f"at {ref_h//2}px -> {est.estimate(args.cls, max(1, ref_h//2)):.2f} m")


if __name__ == "__main__":
    main()
