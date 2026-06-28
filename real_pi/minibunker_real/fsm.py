#!/usr/bin/env python3
# ============================================================================
#  fsm.py — reactive space-mining state machine (native, ROS-free).
#
#      fsm.step(ps, armed, follow, teleop, now, target_dist) -> (lin, ang, state)
#
#  Missions (follow):
#     ball -> SEARCH -> APPROACH -> (AVOID a close cone) -> reach ball_retrieve_m
#             -> RETRIEVE: stop, set outcome "ball_retrieved" (run.py disarms +
#                mission none + "ball retrieved" message).
#     cone -> SEARCH -> APPROACH -> reach cone_danger_m -> DANGER: back up for
#             cone_backup_sec, then outcome "cone_danger" (run.py disarms + none +
#             danger message).
#     none -> TELEOP (WASD pass-through, watchdogged).
#
#  Reach test uses the calibrated pixel distance (target_dist) against the
#  per-mission distance; if the class isn't distance-calibrated it falls back to
#  the bbox-height fraction (approach/collect_bbox_frac).
#
#  Cross-cutting side effects (disarm, switch mission, show/persist the message)
#  belong to run.py: the FSM only raises them via `self.outcome` / `self.message`.
# ============================================================================
from __future__ import annotations

SEARCH, APPROACH, AVOID, BACKUP, DONE, STOP, TELEOP = (
    "SEARCH", "APPROACH", "AVOID", "BACKUP", "DONE", "STOP", "TELEOP")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _kind(follow):
    """Normalise the mission string to a target kind."""
    f = str(follow).lower()
    if f in ("ball", "green_ball"):
        return "ball"
    if f == "cone":
        return "cone"
    return None


class BehaviorFSM:
    def __init__(self, cfg):
        self.cfg = cfg                       # the `behavior` config block
        self.state = SEARCH
        self.last_seen_ticks = 0
        # mission-completion signals (run.py reads + acts, then reset_outcome()):
        self.outcome = None                  # None | "ball_retrieved" | "cone_danger"
        self.message = ""                    # human-facing event text
        self.message_kind = ""               # "success" | "danger" | ""
        self._backup_until = 0.0             # monotonic deadline for the cone backup
        self._backing = False

    def _g(self, path, default):
        node = self.cfg
        for k in path.split("/"):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def reset_outcome(self):
        self.outcome = None
        self.message = ""
        self.message_kind = ""
        self._backing = False

    def on_disarm(self):
        self.state = STOP
        self._backing = False
        self.reset_outcome()

    # -- reach test ----------------------------------------------------------
    def _reach_distance(self, kind):
        if kind == "ball":
            return float(self._g("approach/ball_retrieve_m", 0.7))
        if kind == "cone":
            return float(self._g("approach/cone_danger_m", 0.5))
        return None

    def _reached(self, kind, target_seen, h_frac, target_dist):
        if not target_seen:
            return False
        rd = self._reach_distance(kind)
        if rd is not None and target_dist is not None and target_dist > 0:
            return target_dist <= rd
        # uncalibrated fallback: bbox height fraction proxy
        return h_frac >= float(self._g("approach/collect_bbox_frac", 0.45))

    # -- main step -----------------------------------------------------------
    def step(self, ps, armed, follow, teleop, now, target_dist=None):
        if not armed:
            self.state = STOP
            return 0.0, 0.0, STOP

        kind = _kind(follow)
        if kind is None:                     # mission none -> teleop
            return self._step_teleop(teleop, now)

        if ps is None:
            self.state = STOP
            return 0.0, 0.0, STOP

        max_lin = float(self._g("limits/max_linear", 0.4))
        max_ang = float(self._g("limits/max_angular", 1.0))
        scan_w = float(self._g("search/scan_angular_speed", 0.5))
        steer_gain = float(self._g("approach/steer_gain", 0.8))
        fwd = float(self._g("approach/forward_speed", 0.25))
        backoff = float(self._g("avoid/backoff_speed", -0.15))
        turn = float(self._g("avoid/turn_speed", 0.6))
        backup_speed = float(self._g("approach/cone_backup_speed", -0.15))

        target_seen = ps[0] > 0.5
        cx = ps[1]
        h_frac = ps[3]
        hazard_danger = ps[5] > 0.5
        hazard_cx = ps[6]
        self.last_seen_ticks = 0 if target_seen else self.last_seen_ticks + 1

        # 1) already completed -> hold still (run.py disarms + resets this tick)
        if self.outcome is not None:
            self.state = DONE
            return 0.0, 0.0, DONE

        # 2) cone-danger backup running -> reverse until the timer expires
        if self._backing:
            if now < self._backup_until:
                self.state = BACKUP
                return _clamp(backup_speed, -max_lin, max_lin), 0.0, BACKUP
            self._backing = False
            self.outcome = "cone_danger"
            self.state = DONE
            return 0.0, 0.0, DONE

        lin = ang = 0.0
        # 3) AVOID a dangerous hazard (ball mission only; cone is its own target)
        if hazard_danger:
            self.state = AVOID
            lin = backoff
            ang = -turn if hazard_cx >= 0 else turn
        # 4) reached the target -> retrieve (ball) or danger+backup (cone)
        elif self._reached(kind, target_seen, h_frac, target_dist):
            if kind == "ball":
                self.message = "BALL RETRIEVED - mission complete"
                self.message_kind = "success"
                self.outcome = "ball_retrieved"
                self.state = DONE
                return 0.0, 0.0, DONE
            else:  # cone
                self.message = "DANGER - too close to the cone, backing up"
                self.message_kind = "danger"
                self._backing = True
                self._backup_until = now + float(self._g("approach/cone_backup_sec", 1.0))
                self.state = BACKUP
                return _clamp(backup_speed, -max_lin, max_lin), 0.0, BACKUP
        # 5) chase the target
        elif target_seen:
            self.state = APPROACH
            ang = -steer_gain * cx
            lin = fwd * (1.0 - min(0.8, h_frac))
        # 6) nothing in view -> rotating scan
        else:
            self.state = SEARCH
            ang = scan_w

        lin = _clamp(lin, -max_lin, max_lin)
        ang = _clamp(ang, -max_ang, max_ang)
        return lin, ang, self.state

    # -- teleop pass-through (follow_item == none) --------------------------
    def _step_teleop(self, teleop, now):
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
