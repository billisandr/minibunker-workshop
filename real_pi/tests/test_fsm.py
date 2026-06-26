#!/usr/bin/env python3
# ============================================================================
#  test_fsm.py — the behaviour FSM safety + transition contract (no hardware).
#
#  Run:  python3 -m pytest real_pi/tests/test_fsm.py -v
#    or: python3 real_pi/tests/test_fsm.py
# ============================================================================
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from minibunker_real.fsm import (  # noqa: E402
    APPROACH, AVOID, SEARCH, STOP, TELEOP, BehaviorFSM,
)

# minimal behavior block (defaults mirror config.yaml)
CFG = {
    "rate_hz": 20.0,
    "search": {"scan_angular_speed": 0.5},
    "approach": {"steer_gain": 0.8, "forward_speed": 0.25, "collect_bbox_frac": 0.45},
    "collect": {"pause_sec": 0.1, "turn_speed": 0.9, "turn_sec_min": 0.1,
                "turn_sec_max": 0.1, "away_speed": 0.25, "away_sec_min": 0.1,
                "away_sec_max": 0.1},
    "avoid": {"cone_danger_frac": 0.35, "backoff_speed": -0.15, "turn_speed": 0.6},
    "limits": {"max_linear": 0.4, "max_angular": 1.0, "lost_frames": 15},
    "teleop": {"timeout_ms": 400, "linear_speed": 0.25, "angular_speed": 0.8},
}

ZERO_PS = [0.0] * 7


def ps(target_seen=0, cx=0.0, h=0.0, hz_seen=0, danger=0, hz_cx=0.0):
    return [float(target_seen), cx, 0.0, h, float(hz_seen), float(danger), hz_cx]


def test_disarmed_is_always_zero():
    fsm = BehaviorFSM(CFG)
    lin, ang, st = fsm.step(ps(target_seen=1, cx=0.0, h=0.1), False, "ball", None, 0.0)
    assert (lin, ang, st) == (0.0, 0.0, STOP)


def test_search_when_no_target():
    fsm = BehaviorFSM(CFG)
    lin, ang, st = fsm.step(ZERO_PS, True, "ball", None, 0.0)
    assert st == SEARCH and lin == 0.0 and ang > 0.0   # rotating to scan


def test_approach_steers_toward_target():
    fsm = BehaviorFSM(CFG)
    # target to the RIGHT (cx>0) -> steer right (angular negative), drive forward
    lin, ang, st = fsm.step(ps(target_seen=1, cx=0.5, h=0.1), True, "ball", None, 0.0)
    assert st == APPROACH and lin > 0.0 and ang < 0.0


def test_avoid_has_priority_and_backs_off():
    fsm = BehaviorFSM(CFG)
    # hazard in danger zone -> AVOID overrides APPROACH: back off (lin<0) and
    # turn. Sign matches the validated sim (behavior_node): hazard_cx>=0 -> -turn.
    lin, ang, st = fsm.step(ps(target_seen=1, cx=0.0, h=0.1, hz_seen=1,
                               danger=1, hz_cx=0.5), True, "ball", None, 0.0)
    assert st == AVOID
    assert lin == CFG["avoid"]["backoff_speed"]      # -0.15, backing off
    assert ang == -CFG["avoid"]["turn_speed"]        # -0.6 for hazard on the right
    # mirror case: hazard on the left -> +turn
    fsm2 = BehaviorFSM(CFG)
    _, ang2, _ = fsm2.step(ps(target_seen=1, cx=0.0, h=0.1, hz_seen=1,
                              danger=1, hz_cx=-0.5), True, "ball", None, 0.0)
    assert ang2 == CFG["avoid"]["turn_speed"]


def test_collect_then_retreat_then_search():
    fsm = BehaviorFSM(CFG)
    # target close enough (h >= collect_frac) -> enter COLLECT
    fsm.step(ps(target_seen=1, cx=0.0, h=0.6), True, "ball", None, 0.0)
    assert fsm.collect_phase == "pause"
    # advance past pause -> turn -> away -> back to SEARCH (tiny durations in CFG)
    states = []
    t = 0.2
    for _ in range(8):
        _, _, st = fsm.step(ps(target_seen=1, cx=0.0, h=0.6), True, "ball", None, t)
        states.append(st)
        t += 0.2
    assert SEARCH in states          # sequence completes and resumes searching


def test_teleop_watchdog_zeros_on_stale():
    fsm = BehaviorFSM(CFG)
    # fresh intent at t=10 -> honoured
    lin, ang, st = fsm.step(ZERO_PS, True, "none", (0.25, 0.0, 10.0), 10.05)
    assert st == TELEOP and abs(lin - 0.25) < 1e-6
    # same intent now stale (>400 ms old) -> zero
    lin, ang, st = fsm.step(ZERO_PS, True, "none", (0.25, 0.0, 10.0), 11.0)
    assert st == TELEOP and lin == 0.0 and ang == 0.0


def test_limits_clamp():
    cfg = dict(CFG)
    cfg["approach"] = {"steer_gain": 100.0, "forward_speed": 100.0,
                       "collect_bbox_frac": 0.45}
    fsm = BehaviorFSM(cfg)
    lin, ang, st = fsm.step(ps(target_seen=1, cx=1.0, h=0.0), True, "ball", None, 0.0)
    assert lin <= CFG["limits"]["max_linear"] + 1e-9
    assert abs(ang) <= CFG["limits"]["max_angular"] + 1e-9


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
