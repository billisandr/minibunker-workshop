#!/usr/bin/env python3
# ============================================================================
#  test_distance.py — pixel distance estimator (no hardware).
#    python3 -m pytest real_pi/tests/test_distance.py -v   | or run directly
# ============================================================================
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from minibunker_real.distance import DistanceEstimator  # noqa: E402

CFG = {"enabled": True,
       "green_ball": {"ref_distance_m": 1.0, "ref_height_px": 120},
       "cone": {"ref_distance_m": 1.0, "ref_height_px": 0}}


def test_estimate_inverse_proportional():
    est = DistanceEstimator(CFG)
    # at the reference height -> reference distance
    assert abs(est.estimate("green_ball", 120) - 1.0) < 1e-6
    # half the pixel height -> twice the distance
    assert abs(est.estimate("green_ball", 60) - 2.0) < 1e-6
    # double the pixel height -> half the distance
    assert abs(est.estimate("green_ball", 240) - 0.5) < 1e-6


def test_uncalibrated_returns_none():
    est = DistanceEstimator(CFG)
    assert est.estimate("cone", 100) is None          # ref_height_px = 0
    assert not est.is_calibrated("cone")
    assert est.is_calibrated("green_ball")


def test_no_detection_or_disabled():
    est = DistanceEstimator(CFG)
    assert est.estimate("green_ball", 0) is None       # no bbox
    est2 = DistanceEstimator({**CFG, "enabled": False})
    assert est2.estimate("green_ball", 120) is None     # disabled


def test_calibrate_sets_reference():
    est = DistanceEstimator({"enabled": True})
    est.calibrate("cone", 0.8, 96)                       # cone at 0.8 m -> 96 px
    assert est.is_calibrated("cone")
    assert abs(est.estimate("cone", 96) - 0.8) < 1e-6
    assert abs(est.estimate("cone", 48) - 1.6) < 1e-6
    assert est.refs()["cone"]["ref_height_px"] == 96


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn(); print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1; print(f"FAIL {fn.__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
