#!/usr/bin/env python3
# ============================================================================
#  distance.py — pixel-based distance estimator (one-point reference model).
#
#  Pinhole geometry: an object's image size is inversely proportional to its
#  distance. Calibrate ONCE per object — place it at a known distance and record
#  its bounding-box height in pixels — then:
#
#      distance_m = ref_distance_m * ref_height_px / current_bbox_height_px
#
#  No focal length or real object size needed; one sample per class. This is the
#  lightweight stand-in for the depth-camera / Depth-Anything stage (plan §E).
#  Bbox HEIGHT is the size cue (for a ball that's its diameter).
#
#  cfg (the detector/distance block):
#     enabled: true
#     green_ball: { ref_distance_m: 1.0, ref_height_px: 120 }   # 0 = uncalibrated
#     cone:       { ref_distance_m: 1.0, ref_height_px: 0   }
# ============================================================================
from __future__ import annotations


class DistanceEstimator:
    def __init__(self, cfg=None):
        self.cfg = dict(cfg) if cfg else {}
        self.enabled = bool(self.cfg.get("enabled", True))

    def _refs(self, cls_name):
        sub = self.cfg.get(cls_name) or {}
        return sub.get("ref_distance_m"), sub.get("ref_height_px")

    def is_calibrated(self, cls_name) -> bool:
        d, h = self._refs(cls_name)
        return bool(d) and bool(h) and float(h) > 0

    def estimate(self, cls_name, bbox_height_px):
        """metres, or None if disabled / uncalibrated / no detection."""
        if not self.enabled or not bbox_height_px or bbox_height_px <= 0:
            return None
        d, h = self._refs(cls_name)
        if not d or not h or float(h) <= 0:
            return None
        return float(d) * float(h) / float(bbox_height_px)

    def calibrate(self, cls_name, distance_m, bbox_height_px):
        """Set the reference for a class from one (distance, pixel-height) sample.
        Returns the stored sub-dict."""
        sub = self.cfg.setdefault(cls_name, {})
        sub["ref_distance_m"] = float(distance_m)
        sub["ref_height_px"] = int(bbox_height_px)
        return sub

    def refs(self):
        """All per-class references, for the API/UI."""
        out = {}
        for c in ("green_ball", "cone"):
            d, h = self._refs(c)
            out[c] = {"ref_distance_m": d if d is not None else 1.0,
                      "ref_height_px": int(h) if h else 0,
                      "calibrated": self.is_calibrated(c)}
        return out
