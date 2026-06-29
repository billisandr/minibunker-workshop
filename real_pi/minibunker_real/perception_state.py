#!/usr/bin/env python3
# ============================================================================
#  perception_state.py — role-based 7-slot packing, lifted from the ROS
#  detector_node so the FSM contract is identical sim<->real (plan.md §20.3).
#
#  pack(dets, w, h, target_cls, hazard_classes, cone_danger_frac) -> [7 floats]:
#     [0] target_seen   [1] target_cx_norm  [2] target_cy_norm  [3] target_h_frac
#     [4] hazard_seen   [5] hazard_danger    [6] hazard_cx_norm
#  target = largest detection of the followed class; hazard = largest among the
#  (followed-class-excluded) hazard set.
# ============================================================================
from __future__ import annotations

import cv2

from .detector import LABELS, NAME_TO_CLASS, COLOURS


def resolve_mission(mission_cfg):
    """(follow_item, hazard_items) strings -> (target_cls|None, {hazard_cls})."""
    follow = str(mission_cfg.get("follow_item", "none")).lower()
    target_cls = NAME_TO_CLASS.get(follow)            # None for 'none'/unknown
    hazards = mission_cfg.get("hazard_items", ["cone"]) or []
    cls = {NAME_TO_CLASS[n] for n in hazards if n in NAME_TO_CLASS}
    cls.discard(target_cls)                            # never its own hazard
    return target_cls, cls


def pack(dets, w, h, target_cls, hazard_classes, cone_danger_frac=0.35):
    best_target = None     # largest detection of the followed class
    best_hazard = None     # largest detection among the hazard classes
    for (cls, x, y, bw, bh, _score) in dets:
        if cls == target_cls and (best_target is None or bh > best_target[3]):
            best_target = (x, y, bw, bh)
        if cls in hazard_classes and (best_hazard is None or bh > best_hazard[3]):
            best_hazard = (x, y, bw, bh)

    st = [0.0] * 7
    if best_target is not None:
        x, y, bw, bh = best_target
        st[0] = 1.0
        st[1] = ((x + bw / 2.0) / w) * 2.0 - 1.0
        st[2] = ((y + bh / 2.0) / h) * 2.0 - 1.0
        st[3] = bh / float(h)
    if best_hazard is not None:
        x, y, bw, bh = best_hazard
        st[4] = 1.0
        cx_norm = ((x + bw / 2.0) / w) * 2.0 - 1.0
        cy_norm = ((y + bh / 2.0) / h)
        big = (bh / float(h)) >= cone_danger_frac
        low_centre = cy_norm > 0.45 and abs(cx_norm) < 0.6
        st[5] = 1.0 if (big and low_centre) else 0.0
        st[6] = cx_norm
    return st


def annotate(bgr, dets, st, backend_name, target_cls, state_name,
             display=None, dist_est=None):
    """Draw boxes + a HUD on a copy of the frame (for the debug window / file).

    display:  {class_int: short label} from config detector/display_names (falls
              back to the full class name). dist_est: a DistanceEstimator; when a
              class is distance-calibrated the box label shows metres instead of
              the score, e.g. "b 0.85m"."""
    img = bgr.copy()
    disp = display or {}

    def name(c):
        return disp.get(c, LABELS.get(c, "?"))

    for (cls, x, y, bw, bh, score) in dets:
        colour = COLOURS.get(cls, (255, 255, 255))
        cv2.rectangle(img, (x, y), (x + bw, y + bh), colour, 2)
        d = dist_est.estimate(LABELS.get(cls), bh) if dist_est is not None else None
        label = "%s %.2fm" % (name(cls), d) if d else "%s %.2f" % (name(cls), score)
        cv2.putText(img, label, (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, colour, 2)
    follow = name(target_cls) if target_cls is not None else "none"
    # st[4]=obstacle seen (any hazard-class object), st[5]=hazard (close + in path)
    hud = "backend:%s follow:%s state:%s target:%d obstacle:%d hazard:%d" % (
        backend_name, follow, state_name, int(st[0]), int(st[4]), int(st[5]))
    cv2.putText(img, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2)
    return img
