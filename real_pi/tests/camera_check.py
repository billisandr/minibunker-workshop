#!/usr/bin/env python3
# ============================================================================
#  camera_check.py — diagnose Pi camera colour + health (run on the Pi).
#
#  Captures one frame via picamera2 and writes BOTH interpretations so you can
#  SEE which channel order is right, plus prints per-channel means. Compare against
#  the stock libcamera tool (ground truth):
#       rpicam-still -o /tmp/cam_ref.jpg     # (or libcamera-still ...)
#  then view all three (e.g. cd /tmp && python3 -m http.server 8000).
#
#  Usage:  python tests/camera_check.py
# ============================================================================
import sys

try:
    import cv2
    import numpy as np
    from picamera2 import Picamera2
except Exception as exc:  # noqa: BLE001
    print(f"[camera_check] import failed: {exc}")
    sys.exit(1)


def main():
    print("=== detected cameras ===")
    try:
        for i, c in enumerate(Picamera2.global_camera_info()):
            print(f"  [{i}] {c}")
    except Exception as exc:  # noqa: BLE001
        print(f"  (global_camera_info failed: {exc})")

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}))
    cam.start()
    import time
    time.sleep(0.5)              # let AWB/AE settle
    arr = cam.capture_array()
    cam.stop()

    print(f"\ncaptured array shape={arr.shape} dtype={arr.dtype}")
    # channel means of the RAW array as returned (index 0,1,2)
    m = arr.reshape(-1, 3).mean(axis=0)
    print(f"raw array channel means [c0,c1,c2] = [{m[0]:.0f}, {m[1]:.0f}, {m[2]:.0f}]")

    # picamera2 'RGB888' is BGR-ordered -> 'asis' is the correct OpenCV BGR frame.
    cv2.imwrite("/tmp/cam_asis.jpg", arr)                     # use as BGR (default)
    cv2.imwrite("/tmp/cam_swapped.jpg", cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
    print("\nwrote /tmp/cam_asis.jpg  (default: array used as BGR)")
    print("wrote /tmp/cam_swapped.jpg  (R<->B swapped)")
    print("\nNEXT:")
    print("  1) rpicam-still -o /tmp/cam_ref.jpg     # ground-truth colours")
    print("  2) cd /tmp && python3 -m http.server 8000   # view all three")
    print("  3) whichever of asis/swapped matches cam_ref is correct:")
    print("       asis correct    -> camera/picam_swap_rb: false   (default)")
    print("       swapped correct -> camera/picam_swap_rb: true")


if __name__ == "__main__":
    main()
