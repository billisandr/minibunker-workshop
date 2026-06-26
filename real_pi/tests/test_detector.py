#!/usr/bin/env python3
# ============================================================================
#  test_detector.py — HSV detector + perception_state packing (no hardware).
#
#  Run:  python3 -m pytest real_pi/tests/test_detector.py -v
#    or: python3 real_pi/tests/test_detector.py
# ============================================================================
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from minibunker_real.detector import (  # noqa: E402
    CLASS_CONE, CLASS_GREEN_BALL, HsvDetector,
)
from minibunker_real import perception_state as psmod  # noqa: E402

HSV_CFG = {
    "green_ball": {"lower": [40, 70, 40], "upper": [85, 255, 255],
                   "min_area": 300, "close_ksize": 5, "open_ksize": 5},
    "cone": {"lower": [5, 120, 90], "upper": [25, 255, 255],
             "min_area": 300, "close_ksize": [61, 9], "open_ksize": 3},
}


def _frame_with_green_ball(w=640, h=480, cx=400, cy=300, r=40):
    img = np.full((h, w, 3), (40, 40, 50), np.uint8)
    import cv2
    cv2.circle(img, (cx, cy), r, (40, 220, 40), -1)   # green (BGR)
    return img


def test_hsv_finds_green_ball():
    det = HsvDetector(HSV_CFG)
    dets, _mask = det.detect(_frame_with_green_ball())
    balls = [d for d in dets if d[0] == CLASS_GREEN_BALL]
    assert len(balls) >= 1


def test_perception_state_target_packing():
    det = HsvDetector(HSV_CFG)
    # ball on the RIGHT half -> target_cx_norm should be positive
    dets, _ = det.detect(_frame_with_green_ball(cx=500, cy=240))
    target_cls, hazards = psmod.resolve_mission(
        {"follow_item": "ball", "hazard_items": ["cone"]})
    st = psmod.pack(dets, 640, 480, target_cls, hazards)
    assert st[0] == 1.0           # target_seen
    assert st[1] > 0.0            # centre x to the right of frame centre


def test_followed_class_excluded_from_hazards():
    # following the cone -> cone must NOT be in the hazard set
    target_cls, hazards = psmod.resolve_mission(
        {"follow_item": "cone", "hazard_items": ["cone"]})
    assert target_cls == CLASS_CONE
    assert CLASS_CONE not in hazards


def test_none_mission_has_no_target():
    target_cls, hazards = psmod.resolve_mission(
        {"follow_item": "none", "hazard_items": ["cone"]})
    assert target_cls is None
    assert CLASS_CONE in hazards


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
