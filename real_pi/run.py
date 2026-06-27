#!/usr/bin/env python3
# ============================================================================
#  run.py — the native MiniBunker station: ONE loop, no ROS, no Docker.
#
#      camera -> detector -> perception_state -> FSM -> clamp -> CAN(0x111)
#
#  This is the no-ROS path of plan.md §9.4. It is the real-robot equivalent of
#  minibunker_real.launch (pi_camera + detector + behavior + bunker_base), run
#  as a single Python process on the Pi.
#
#  SAFETY (mirrors the ROS station, non-negotiable):
#    * Boots DISARMED: no motion frame is sent until you ARM (key 'a' / web / --arm).
#    * Hard clamp = min(behavior/limits, can/hw_max) before the wire.
#    * Watchdog: a slow/failed frame still ticks the loop and sends zero.
#    * Ctrl-C / any exit -> stop() (zero Twist) + set_standby() (release CAN).
#  Keep the hardware e-stop in hand; run inside the fenced arena only.
#
#  Controls come from ONE hub (Controls): the stdin keys AND the optional web
#  panel both drive it, so there is still a single motion owner on CAN.
#
#  Usage:
#    python3 run.py                      # DISARMED; type 'a'<Enter> to ARM
#    python3 run.py --can vcan0          # dry-run on a virtual CAN bus
#    python3 run.py --no-can             # perception-only (no bus at all)
#    python3 run.py --headless           # no debug window (saves CPU on the Pi)
#    python3 run.py --save-frames out/   # dump annotated frames instead of a window
#    python3 run.py --save-video run.mp4 # record an annotated video (view later)
#    python3 run.py --web                # serve the web panel on 0.0.0.0:8080
# ============================================================================
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import cv2

from minibunker_real.config import Config
from minibunker_real.camera import Camera
from minibunker_real.detector import make_detector
from minibunker_real import perception_state as psmod
from minibunker_real.fsm import BehaviorFSM


def parse_args():
    ap = argparse.ArgumentParser(description="Native MiniBunker station")
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--can", default=None, help="override can/channel (e.g. vcan0)")
    ap.add_argument("--no-can", action="store_true",
                    help="perception/FSM only; never open a CAN bus")
    ap.add_argument("--arm", action="store_true",
                    help="UNSAFE: boot ARMED (skips the DISARMED default)")
    ap.add_argument("--headless", action="store_true", help="no debug window")
    ap.add_argument("--save-frames", default=None,
                    help="dir to write annotated frames into (implies headless)")
    ap.add_argument("--save-video", default=None,
                    help="write an annotated video here, e.g. /tmp/run.mp4 "
                         "(.mp4 -> mp4v, else MJPG/.avi). Implies headless.")
    ap.add_argument("--web", action="store_true",
                    help="serve the Flask web panel (implies headless)")
    ap.add_argument("--web-host", default="0.0.0.0",
                    help="web panel bind host (default 0.0.0.0 = reachable on the LAN)")
    ap.add_argument("--web-port", type=int, default=8080,
                    help="web panel port (default 8080)")
    return ap.parse_args()


class Controls:
    """The single control hub: ARM/DISARM, WASD teleop, mission, quit.

    Both the stdin reader (below) and the optional web panel call these methods,
    so motion still has exactly one owner. Fields are simple atomics (GIL-safe).

      a = ARM    d = DISARM    q = quit
      w/s = fwd/back   j/l = turn left/right   x = stop

    A teleop input sets a *fresh* intent that AUTO-EXPIRES via the FSM watchdog
    (behavior/teleop/timeout_ms): the rover drives for that window then stops
    unless re-pressed. Web 'press-and-hold' buttons just re-send while held.
    """
    def __init__(self, lin_speed=0.25, ang_speed=0.8, read_stdin=True):
        self.armed = False
        self.quit = False
        self.lin_speed = lin_speed
        self.ang_speed = ang_speed
        self.follow_override = None        # None -> use config mission/follow_item
        self._intent = (0.0, 0.0, 0.0)     # (linear, angular, stamp)
        if read_stdin:
            threading.Thread(target=self._stdin_loop, daemon=True).start()

    # -- queried by the loop --
    def teleop(self):
        return self._intent

    # -- driven by stdin + web --
    def arm(self):
        self.armed = True
        print("[run] >>> ARMED")

    def disarm(self):
        self.armed = False
        print("[run] >>> DISARMED")

    def request_quit(self):
        self.quit = True

    def set_teleop(self, lin, ang):
        self._intent = (lin, ang, time.monotonic())

    def nudge(self, key):
        """Map a w/s/j/l/x key to a fresh teleop intent."""
        k = (key or "").lower()[:1]
        if k == "w":
            self.set_teleop(self.lin_speed, 0.0)
        elif k == "s":
            self.set_teleop(-self.lin_speed, 0.0)
        elif k == "j":
            self.set_teleop(0.0, self.ang_speed)
        elif k == "l":
            self.set_teleop(0.0, -self.ang_speed)
        elif k == "x":
            self.set_teleop(0.0, 0.0)

    def set_follow(self, item):
        """Live mission override; 'none'|'ball'|'cone' (None -> back to config)."""
        self.follow_override = item

    def _stdin_loop(self):
        for line in sys.stdin:
            c = line.strip().lower()[:1]
            if c == "a":
                self.arm()
            elif c == "d":
                self.disarm()
            elif c == "q":
                self.request_quit()
                return
            elif c in ("w", "s", "j", "l", "x"):
                self.nudge(c)


def main():
    args = parse_args()
    cfg = Config(args.config)

    # --- perception ---
    cam = Camera(cfg)
    backend_name, detector = make_detector(cfg.block("detector"))
    fsm = BehaviorFSM(cfg.block("behavior"))
    cone_danger = float(cfg.get("behavior/avoid/cone_danger_frac", 0.35))
    hazard_items = cfg.get("mission/hazard_items", ["cone"]) or []

    # --- CAN (optional / overridable) ---
    bunker = None
    if not args.no_can:
        from minibunker_real.bunker_can import BunkerCAN
        channel = args.can or cfg.get("can/channel", "can0")
        bunker = BunkerCAN(
            channel=channel,
            interface=cfg.get("can/interface", "socketcan"),
            hw_max_linear=float(cfg.get("can/hw_max_linear", 1.5)),
            hw_max_angular=float(cfg.get("can/hw_max_angular", 0.7853)))
        print(f"[run] CAN up on {channel}")
    else:
        print("[run] --no-can: perception/FSM only (no motion)")

    controls = Controls(
        lin_speed=float(cfg.get("behavior/teleop/linear_speed", 0.25)),
        ang_speed=float(cfg.get("behavior/teleop/angular_speed", 0.8)))
    controls.armed = bool(args.arm or cfg.get("behavior/arm_on_start", False))
    if controls.armed:
        print("[run] WARNING: booting ARMED")

    rate = float(cfg.get("behavior/rate_hz", 20.0))
    period = 1.0 / rate
    enable_can_mode = bool(cfg.get("can/enable_can_mode", True))
    was_armed = False

    # --- optional web panel ---
    web = None
    if args.web:
        from minibunker_real.webpanel import Telemetry, start_web
        web = Telemetry()
        start_web(controls, web, host=args.web_host, port=args.web_port)
        print(f"[run] web panel: http://<this-host>:{args.web_port}  "
              f"(bind {args.web_host})")

    save_dir = args.save_frames
    save_video = args.save_video
    headless = args.headless or bool(save_dir) or bool(save_video) or bool(web)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
    writer = None          # lazy cv2.VideoWriter (needs the first frame's size)
    frame_i = 0
    last_print = 0.0       # headless telemetry throttle

    print("[run] DISARMED. Keys (+Enter): a=ARM d=DISARM q=quit | "
          "teleop (mission=none): w/s=fwd/back j/l=turn x=stop")
    try:
        while not controls.quit:
            t0 = time.monotonic()

            ok, frame = cam.read()
            if not ok:
                print("[run] camera read failed; sending zero")
                if bunker:
                    bunker.stop()
                time.sleep(period)
                continue
            h, w = frame.shape[:2]

            # effective mission = web override if set, else config
            follow = controls.follow_override or cfg.get("mission/follow_item", "none")
            target_cls, hazard_classes = psmod.resolve_mission(
                {"follow_item": follow, "hazard_items": hazard_items})

            dets, _mask = detector.detect(frame)
            ps = psmod.pack(dets, w, h, target_cls, hazard_classes, cone_danger)

            # ARM edge handling: on ARM, (re)enable CAN mode so the base listens.
            if controls.armed and not was_armed and bunker and enable_can_mode:
                bunker.enable_can_mode()
                print("[run] sent CONTROL_MODE_CAN")
            if was_armed and not controls.armed:
                fsm.on_disarm()
            was_armed = controls.armed

            lin, ang, state = fsm.step(ps, controls.armed, follow, controls.teleop(), t0)

            if bunker:
                if controls.armed:
                    bunker.send_motion(lin, ang)   # clamps to HW ceiling inside
                else:
                    bunker.stop()
                bunker.poll()                       # drain state frames

            # --- annotate once, fan out to window / frames / video / web ---
            want_frame_dump = save_dir and frame_i % 5 == 0
            need_annot = (not headless) or want_frame_dump or save_video or web
            annotated = None
            if need_annot:
                annotated = psmod.annotate(frame, dets, ps, backend_name,
                                           target_cls, state)
                _draw_status(annotated, controls.armed, lin, ang,
                             bunker.state if bunker else None)
            if not headless:
                cv2.imshow("minibunker (real)", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            else:
                if want_frame_dump:
                    cv2.imwrite(os.path.join(save_dir, f"f{frame_i:05d}.jpg"), annotated)
                if save_video:
                    if writer is None:
                        h0, w0 = frame.shape[:2]
                        fourcc = cv2.VideoWriter_fourcc(
                            *("mp4v" if save_video.lower().endswith(".mp4") else "MJPG"))
                        writer = cv2.VideoWriter(save_video, fourcc, rate, (w0, h0))
                        print(f"[run] recording video -> {save_video} @ {rate:.0f} fps")
                    writer.write(annotated)
            if web is not None:
                web.update(_snapshot(state, controls, lin, ang, follow,
                                     backend_name, ps, len(dets), bunker, rate),
                           annotated)
            frame_i += 1

            # headless: throttled telemetry (~1 Hz) so an SSH drive is observable
            if headless and (t0 - last_print) >= 1.0:
                last_print = t0
                extra = ""
                if bunker is not None:
                    s = bunker.state
                    extra = (f" | batt={s.battery_voltage:.1f}V ctrl_mode={s.control_mode}"
                             f" vstate={s.vehicle_state}"
                             f"{' ESTOP!' if s.estop_engaged else ''}"
                             f" actual_v={s.actual_linear:+.2f}")
                print(f"[{state:8s}] armed={controls.armed} cmd v={lin:+.2f} w={ang:+.2f}{extra}")

            # keep the loop at rate_hz (also paces the CAN motion frames)
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[run] Ctrl-C")
    finally:
        print("[run] shutting down -> zero motion + STANDBY")
        if bunker:
            bunker.shutdown()
        cam.close()
        if writer is not None:
            writer.release()
            print(f"[run] video saved -> {save_video}")
        if not headless:
            cv2.destroyAllWindows()


def _snapshot(state, controls, lin, ang, follow, backend, ps, n_dets, bunker, rate):
    """Build the JSON-able telemetry dict for the web panel."""
    snap = {
        "state": state, "armed": controls.armed, "cmd_lin": round(lin, 3),
        "cmd_ang": round(ang, 3), "follow": follow, "backend": backend,
        "rate_hz": rate, "n_dets": n_dets,
        "target_seen": ps[0] > 0.5, "target_cx": round(ps[1], 3),
        "target_h": round(ps[3], 3), "hazard_seen": ps[4] > 0.5,
        "hazard_danger": ps[5] > 0.5,
    }
    if bunker is not None:
        s = bunker.state
        snap.update({
            "battery": round(s.battery_voltage, 1), "ctrl_mode": s.control_mode,
            "vehicle_state": s.vehicle_state, "estop": s.estop_engaged,
            "error_code": s.error_code, "actual_lin": round(s.actual_linear, 3),
            "can": True,
        })
    else:
        snap["can"] = False
    return snap


def _draw_status(img, armed, lin, ang, state):
    txt = "%s  v=%.2f w=%.2f" % ("ARMED" if armed else "DISARMED", lin, ang)
    colour = (0, 0, 255) if armed else (0, 200, 0)
    cv2.putText(img, txt, (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, colour, 2)
    if state is not None and state.control_mode >= 0:
        estop = " ESTOP" if state.estop_engaged else ""
        cv2.putText(img, "batt=%.1fV mode=%d err=%d%s" % (
            state.battery_voltage, state.control_mode, state.error_code, estop),
            (8, img.shape[0] - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 165, 255) if estop else (255, 255, 0), 1)


if __name__ == "__main__":
    main()
