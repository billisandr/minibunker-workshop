#!/usr/bin/env python3
# ============================================================================
#  fsm.py — reactive space-mining state machine, lifted from behavior_node.
#
#  Same states/transitions/control law as the ROS node (SEARCH -> APPROACH ->
#  (AVOID) -> COLLECT -> RETREAT -> SEARCH, plus TELEOP when follow_item==none),
#  but ROS-free: it is a plain stepper driven by run.py's loop clock.
#
#      fsm.step(ps, armed, follow, teleop, now) -> (linear, angular, state)
#
#  - `ps`     : the 7-slot perception_state list (see perception_state.py)
#  - `armed`  : ARM gate; when False -> STOP, zero motion (authoritative)
#  - `follow` : mission/follow_item this tick ("none" -> teleop pass-through)
#  - `teleop` : (linear, angular, stamp) latest WASD intent; watchdogged
#  - `now`    : monotonic seconds (time.monotonic()), replaces rospy.Time
#
#  Timing that used rospy.Time/Duration now uses these monotonic `now` values.
#  The final clamp to behavior/limits stays here; run.py applies the additional
#  hard CAN/HW clamp before the wire.
# ============================================================================
from __future__ import annotations

import random

SEARCH, APPROACH, AVOID, COLLECT, RETREAT, STOP, TELEOP = (
    "SEARCH", "APPROACH", "AVOID", "COLLECT", "RETREAT", "STOP", "TELEOP")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BehaviorFSM:
    def __init__(self, cfg):
        self.cfg = cfg                       # the `behavior` config block
        self.state = SEARCH
        self.last_seen_ticks = 0
        self.collect_phase = None            # None | "pause" | "turn" | "away"
        self.phase_end = 0.0                 # monotonic deadline for the phase
        self.retreat_turn_sign = 1.0

    def _g(self, path, default):
        # dotted lookup within the behavior block, e.g. _g("limits/max_linear")
        node = self.cfg
        for k in path.split("/"):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def on_disarm(self):
        """Mirror behavior_node.on_arm(False): drop any collect maneuver."""
        self.state = STOP
        self.collect_phase = None

    def _reached_target(self, target_seen, h_frac, target_dist):
        """COLLECT trigger: real distance if configured + calibrated, else the
        bbox-height fraction proxy."""
        if not target_seen:
            return False
        cd = self._g("approach/collect_distance_m", None)
        if cd is not None and target_dist is not None and target_dist > 0:
            return target_dist <= float(cd)
        return h_frac >= float(self._g("approach/collect_bbox_frac", 0.45))

    # -- main step -----------------------------------------------------------
    def step(self, ps, armed, follow, teleop, now, target_dist=None):
        if not armed:
            self.state = STOP
            return 0.0, 0.0, STOP

        if str(follow).lower() == "none":
            return self._step_teleop(teleop, now)

        if ps is None:
            self.state = STOP
            return 0.0, 0.0, STOP

        max_lin = float(self._g("limits/max_linear", 0.4))
        max_ang = float(self._g("limits/max_angular", 1.0))
        lost = int(self._g("limits/lost_frames", 15))
        scan_w = float(self._g("search/scan_angular_speed", 0.5))
        steer_gain = float(self._g("approach/steer_gain", 0.8))
        fwd = float(self._g("approach/forward_speed", 0.25))
        backoff = float(self._g("avoid/backoff_speed", -0.15))
        turn = float(self._g("avoid/turn_speed", 0.6))

        target_seen = ps[0] > 0.5
        cx = ps[1]
        h_frac = ps[3]
        hazard_danger = ps[5] > 0.5
        hazard_cx = ps[6]

        if target_seen:
            self.last_seen_ticks = 0
        else:
            self.last_seen_ticks += 1

        lin = ang = 0.0
        if self.collect_phase is not None:
            lin, ang = self._run_collect_phase(now)
        elif hazard_danger:
            self.state = AVOID
            lin = backoff
            ang = -turn if hazard_cx >= 0 else turn
        elif self._reached_target(target_seen, h_frac, target_dist):
            self._begin_collect(now)
        elif target_seen:
            self.state = APPROACH
            ang = -steer_gain * cx
            lin = fwd * (1.0 - min(0.8, h_frac))
        else:
            self.state = SEARCH
            ang = scan_w

        lin = _clamp(lin, -max_lin, max_lin)
        ang = _clamp(ang, -max_ang, max_ang)
        return lin, ang, self.state

    # -- collect/retreat timed sub-sequence ---------------------------------
    def _begin_collect(self, now):
        self.state = COLLECT
        self.collect_phase = "pause"
        self.phase_end = now + float(self._g("collect/pause_sec", 5.0))

    def _run_collect_phase(self, now):
        lin = ang = 0.0
        if self.collect_phase == "pause":
            self.state = COLLECT
            if now >= self.phase_end:
                self.retreat_turn_sign = random.choice([-1.0, 1.0])
                dur = random.uniform(float(self._g("collect/turn_sec_min", 2.5)),
                                     float(self._g("collect/turn_sec_max", 4.0)))
                self.phase_end = now + dur
                self.collect_phase = "turn"
        elif self.collect_phase == "turn":
            self.state = RETREAT
            ang = self.retreat_turn_sign * float(self._g("collect/turn_speed", 0.9))
            if now >= self.phase_end:
                dur = random.uniform(float(self._g("collect/away_sec_min", 1.5)),
                                     float(self._g("collect/away_sec_max", 3.0)))
                self.phase_end = now + dur
                self.collect_phase = "away"
        else:  # "away"
            self.state = RETREAT
            lin = float(self._g("collect/away_speed", 0.25))
            if now >= self.phase_end:
                self.collect_phase = None
                self.state = SEARCH
        return lin, ang

    # -- teleop pass-through (follow_item == none) --------------------------
    def _step_teleop(self, teleop, now):
        """teleop = (linear, angular, stamp). Watchdog -> zero on stale input."""
        max_lin = float(self._g("limits/max_linear", 0.4))
        max_ang = float(self._g("limits/max_angular", 1.0))
        timeout = float(self._g("teleop/timeout_ms", 400)) / 1000.0
        lin = ang = 0.0
        if teleop is not None:
            t_lin, t_ang, stamp = teleop
            if (now - stamp) <= timeout:
                lin, ang = t_lin, t_ang
        lin = _clamp(lin, -max_lin, max_lin)
        ang = _clamp(ang, -max_ang, max_ang)
        self.state = TELEOP
        return lin, ang, TELEOP
